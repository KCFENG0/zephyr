/*
 * Copyright (c) 2022 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT aspeed_pcc

#include <device.h>
#include <kernel.h>
#include <soc.h>
#include <errno.h>
#include <string.h>
#include <logging/log.h>
#include <drivers/clock_control.h>
#include <drivers/misc/aspeed/pcc_aspeed.h>
LOG_MODULE_REGISTER(pcc_aspeed, CONFIG_LOG_DEFAULT_LEVEL);

/* LPC registers */
#define PCCR6   0x0c4
#define PCCR4   0x0d0
#define PCCR5   0x0d4
#define PCCR0   0x130
#define   PCCR0_EN_DMA_INT    BIT(31)
#define   PCCR0_EN_DMA_MODE   BIT(14)
#define   PCCR0_ADDR_SEL_MASK GENMASK(13, 12)
#define   PCCR0_ADDR_SEL_SHIFT    12
#define   PCCR0_RX_TRIG_LVL_MASK  GENMASK(10, 8)
#define   PCCR0_RX_TRIG_LVL_SHIFT 8
#define   PCCR0_CLR_RX_FIFO   BIT(7)
#define   PCCR0_MODE_SEL_MASK GENMASK(5, 4)
#define   PCCR0_MODE_SEL_SHIFT    4
#define   PCCR0_EN_RX_OVR_INT BIT(3)
#define   PCCR0_EN_RX_TMOUT_INT   BIT(2)
#define   PCCR0_EN_RX_AVAIL_INT   BIT(1)
#define   PCCR0_EN        BIT(0)
#define PCCR1   0x134
#define   PCCR1_DONT_CARE_BITS_MASK   GENMASK(21, 16)
#define   PCCR1_DONT_CARE_BITS_SHIFT  16
#define   PCCR1_BASE_ADDR_MASK        GENMASK(15, 0)
#define   PCCR1_BASE_ADDR_SHIFT       0
#define PCCR2   0x138
#define   PCCR2_DMA_DONE      BIT(4)
#define   PCCR2_DATA_RDY      PCCR2_DMA_DONE
#define   PCCR2_RX_OVR_INT    BIT(3)
#define   PCCR2_RX_TMOUT_INT  BIT(2)
#define   PCCR2_RX_AVAIL_INT  BIT(1)
#define PCCR3   0x13c
#define   PCCR3_FIFO_DATA_MASK    GENMASK(7, 0)

static uintptr_t lpc_base;
#define LPC_RD(reg)             sys_read32(lpc_base + reg)
#define LPC_WR(val, reg)        sys_write32(val, lpc_base + reg)

enum pcc_fifo_threthold {
	PCC_FIFO_THR_1_BYTE,
	PCC_FIFO_THR_1_EIGHTH,
	PCC_FIFO_THR_2_EIGHTH,
	PCC_FIFO_THR_3_EIGHTH,
	PCC_FIFO_THR_4_EIGHTH,
	PCC_FIFO_THR_5_EIGHTH,
	PCC_FIFO_THR_6_EIGHTH,
	PCC_FIFO_THR_7_EIGHTH,
	PCC_FIFO_THR_8_EIGHTH,
};

enum pcc_aspeed_record_mode {
	PCC_REC_1B,
	PCC_REC_2B,
	PCC_REC_4B,
	PCC_REC_FULL,
};

enum pcc_aspeed_hbits_select {
	PCC_HBIT_SEL_NONE,
	PCC_HBIT_SEL_45,
	PCC_HBIT_SEL_67,
	PCC_HBIT_SEL_89,
};

struct pcc_aspeed_fifo {
	intptr_t reserved;
	uint32_t byte;
};

struct pcc_aspeed_data {
	struct k_fifo fifo;
	uint8_t *dma_virt;
	uintptr_t dma_addr;
	uint32_t dma_size;
	uint32_t dma_virt_idx;
};

struct pcc_aspeed_config {
	uintptr_t base;
	uint32_t addr;
	uint32_t addr_xbit;
	uint32_t addr_hbit_sel;
	uint32_t rec_mode;
	bool dma_mode;
};

#define PCC_DMA_SIZE	0x1000

static uint8_t pcc_dma_buf[PCC_DMA_SIZE] NON_CACHED_BSS_ALIGN16;

int pcc_aspeed_read(const struct device *dev, uint8_t *out, bool blocking)
{
	struct pcc_aspeed_fifo *node;
	struct pcc_aspeed_data *data = (struct pcc_aspeed_data *)dev->data;

	node = k_fifo_get(&data->fifo, (blocking) ? K_FOREVER : K_NO_WAIT);
	if (!node)
		return -ENODATA;

	*out = (uint8_t)node->byte;

	k_free(node);

	return 0;
}

static void pcc_aspeed_isr_dma(const struct device *dev)
{
	uint32_t pre_idx, cur_idx;
	uint32_t reg;
	struct pcc_aspeed_fifo *node;
	struct pcc_aspeed_data *data = (struct pcc_aspeed_data *)dev->data;

	reg = LPC_RD(PCCR2);
	if (!(reg & PCCR2_DMA_DONE))
		return;

	/* ack DMA IRQ */
	LPC_WR(reg, PCCR2);

	/* copy DMA buffer to fifo */
	reg = LPC_RD(PCCR6);

	cur_idx = reg & (data->dma_size - 1);
	pre_idx = data->dma_virt_idx;

	do {
		node = k_malloc(sizeof(struct pcc_aspeed_fifo));
		if (node) {
			node->byte = data->dma_virt[pre_idx];
			k_fifo_put(&data->fifo, node);
		} else
			LOG_ERR("failed to allocate FIFO, drop data\n");

		pre_idx = (pre_idx + 1) % data->dma_size;
	} while (pre_idx != cur_idx);

	data->dma_virt_idx = cur_idx;
}

static void pcc_aspeed_isr_fifo(const struct device *dev)
{
	uint32_t reg;
	struct pcc_aspeed_fifo *node;
	struct pcc_aspeed_data *data = (struct pcc_aspeed_data *)dev->data;

	reg = LPC_RD(PCCR2);

	if (reg & PCCR2_RX_OVR_INT) {
		LOG_INF("RX FIFO overrun\n");
		LPC_WR(PCCR2_RX_OVR_INT, PCCR2);
	}

	if (reg & (PCCR2_RX_TMOUT_INT | PCCR2_RX_AVAIL_INT)) {
		while (reg & PCCR2_DATA_RDY) {
			node = k_malloc(sizeof(struct pcc_aspeed_fifo));
			if (node) {
				node->byte = LPC_RD(PCCR3) & PCCR3_FIFO_DATA_MASK;
				k_fifo_put(&data->fifo, node);
			} else
				LOG_ERR("failed to allocate FIFO, drop data\n");

			reg = LPC_RD(PCCR2);
		}
	}
}

static void pcc_aspeed_isr(const struct device *dev)
{
	struct pcc_aspeed_config *cfg = (struct pcc_aspeed_config *)dev->config;

	if (cfg->dma_mode)
		pcc_aspeed_isr_dma(dev);
	else
		pcc_aspeed_isr_fifo(dev);
}

static int pcc_aspeed_init(const struct device *dev)
{
	uint32_t reg;
	struct pcc_aspeed_data *data = (struct pcc_aspeed_data *)dev->data;
	struct pcc_aspeed_config *cfg = (struct pcc_aspeed_config *)dev->config;

	if (!lpc_base)
		lpc_base = cfg->base;

	k_fifo_init(&data->fifo);

	if (cfg->dma_mode) {
		data->dma_size = sizeof(pcc_dma_buf);
		data->dma_virt = pcc_dma_buf;
		data->dma_addr = TO_PHY_ADDR(data->dma_virt);
		data->dma_virt_idx = 0;
	}

	IRQ_CONNECT(DT_INST_IRQN(0),
		    DT_INST_IRQ(0, priority),
		    pcc_aspeed_isr,
		    DEVICE_DT_INST_GET(0), 0);

	irq_enable(DT_INST_IRQN(0));

	/* record mode */
	reg = LPC_RD(PCCR0);
	reg &= ~PCCR0_MODE_SEL_MASK;
	reg |= ((cfg->rec_mode << PCCR0_MODE_SEL_SHIFT) & PCCR0_MODE_SEL_MASK);
	LPC_WR(reg, PCCR0);

	/* port address */
	reg = LPC_RD(PCCR1);
	reg &= ~PCCR1_BASE_ADDR_MASK;
	reg |= ((cfg->addr << PCCR1_BASE_ADDR_SHIFT) & PCCR1_BASE_ADDR_MASK);
	LPC_WR(reg, PCCR1);

	/* port address hight bits selection or parser control */
	reg = LPC_RD(PCCR0);
	reg &= ~PCCR0_ADDR_SEL_MASK;
	if (cfg->rec_mode)
	    reg |= ((cfg->addr_hbit_sel << PCCR0_ADDR_SEL_SHIFT) & PCCR0_ADDR_SEL_MASK);
	else
	    reg |= ((0x3 << PCCR0_ADDR_SEL_SHIFT) & PCCR0_ADDR_SEL_MASK);
	LPC_WR(reg, PCCR0);

	/* port address dont care bits */
	reg = LPC_RD(PCCR1);
	reg &= ~PCCR1_DONT_CARE_BITS_MASK;
	reg |= ((cfg->addr_xbit << PCCR1_DONT_CARE_BITS_SHIFT) & PCCR1_DONT_CARE_BITS_MASK);
	LPC_WR(reg, PCCR1);

	/* cleanup FIFO and enable PCC w/wo DMA */
	reg = LPC_RD(PCCR0);
	reg |= PCCR0_CLR_RX_FIFO;

	if (cfg->dma_mode) {
		/* DMA address and size (4-bytes unit) */
		LPC_WR(data->dma_addr, PCCR4);
		LPC_WR(data->dma_size / 4, PCCR5);

		reg |= PCCR0_EN_DMA_INT | PCCR0_EN_DMA_MODE;
	} else {
		reg &= ~PCCR0_RX_TRIG_LVL_MASK;
		reg |= ((PCC_FIFO_THR_4_EIGHTH << PCCR0_RX_TRIG_LVL_SHIFT)
			& PCCR0_RX_TRIG_LVL_MASK);
		reg |= PCCR0_EN_RX_OVR_INT | PCCR0_EN_RX_TMOUT_INT | PCCR0_EN_RX_AVAIL_INT;
	}

	reg |= PCCR0_EN;
	LPC_WR(reg, PCCR0);

	return 0;
}

static struct pcc_aspeed_data pcc_aspeed_data;

static struct pcc_aspeed_config pcc_aspeed_config = {
	.base = DT_REG_ADDR(DT_PARENT(DT_DRV_INST(0))),
	.addr = DT_INST_PROP(0, addr),
	.addr_xbit = DT_INST_PROP(0, addr_xbit),
	.addr_hbit_sel = DT_INST_PROP(0, addr_hbit_sel),
	.rec_mode = DT_INST_PROP(0, rec_mode),
	.dma_mode = DT_INST_PROP_OR(0, dma_mode, 0),
};

DEVICE_DT_INST_DEFINE(0, pcc_aspeed_init, NULL,
		      &pcc_aspeed_data, &pcc_aspeed_config,
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		      NULL);
