/*
 * Copyright (c) 2026 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT nuvoton_npcm4_i3c

#include <drivers/clock_control.h>
#include <drivers/reset_control.h>
#include <drivers/i3c/i3c.h>
#include <soc.h>
#include <sys/util.h>
#include <device.h>
#include <kernel.h>
#include <init.h>
#include <sys/sys_io.h>
#include <logging/log.h>
#define LOG_LEVEL CONFIG_I3C_LOG_LEVEL
LOG_MODULE_REGISTER(i3c_npcm4);

#include <portability/cmsis_os2.h>

#define DEV_CFG(dev)		((const struct i3c_npcm4_config *)(dev)->config)
#define DEV_DATA(dev)		((struct i3c_npcm4_obj *)(dev)->data)
#define DESC_PRIV(desc)		((struct i3c_dev_data *)(desc)->priv_data)

#define I3C_MAX_DEVS		32
#define I3C_NPCM4_FIFO_SIZE	16
#define NUM_MODULES		6
#define DIV_ROUND_UP(n, d)	(((n) + (d) - 1) / (d))

/* NPCM4 PDMA Definitions */
#define PDMA_CH_MAX		14
#define PDMA_CHAN(b, c)		(b + c * 0x10)
#define PDMA_DSCT(cfg, ch)	((struct dsct_reg *)&(cfg)->pdma_base->DSCT[(ch)])

/* I3C DMA definitions */
#define DMA_BUF_SIZE		256
#define TXWIDTH_8		(0 << 12)
#define DAINC_1			(1 << 10)
#define DAINC_FIXED		(3 << 10)
#define SAINC_1			(1 << 8)
#define SAINC_FIXED		(3 << 8)
#define REQ_TYPE_SINGLE		(1 << 2)
#define BASIC_MODE		1
#define SG_MODE			2

/* IRQ number */
#define NUM_CALLBACKS		11
#define IRQ_START		8
#define IRQ_STOP		10
#define IRQ_DACHG		13
#define IRQ_EVENT		18

/* Slave State */
#define SLAVE_DA_ASSIGNED	BIT(0)
#define SLAVE_WAIT_FOR_RX	BIT(1)
#define SLAVE_WAIT_FOR_TX	BIT(2)
#define SLAVE_HAS_PENDING_TX	BIT(3)

/* NPCM4 I3C Register Definitions */
#define NPCM4_I3C_MAX_PPBAUD	15
#define NPCM4_I3C_MAX_PPLOW	15
#define NPCM4_I3C_MAX_ODBAUD	255
#define NPCM4_I3C_MAX_I2CBAUD	15

#define I3C_MCONFIG		0x000
#define   NPCM4_I3C_MCONFIG_MASTER_EN BIT(0)
#define   NPCM4_I3C_MCONFIG_DISTO(x) FIELD_PREP(BIT(3), (x))
#define   NPCM4_I3C_MCONFIG_HKEEP(x) FIELD_PREP(GENMASK(5, 4), (x))
#define   NPCM4_I3C_MCONFIG_ODSTOP(x) FIELD_PREP(BIT(6), (x))
#define   NPCM4_I3C_MCONFIG_PPBAUD(x) FIELD_PREP(GENMASK(11, 8), (x))
#define   NPCM4_I3C_MCONFIG_PPLOW(x) FIELD_PREP(GENMASK(15, 12), (x))
#define   NPCM4_I3C_MCONFIG_ODBAUD(x) FIELD_PREP(GENMASK(23, 16), (x))
#define   NPCM4_I3C_MCONFIG_ODHPP(x) FIELD_PREP(BIT(24), (x))
#define   NPCM4_I3C_MCONFIG_SKEW(x) FIELD_PREP(GENMASK(27, 25), (x))
#define   NPCM4_I3C_MCONFIG_SKEW_MASK GENMASK(27, 25)
#define   NPCM4_I3C_MCONFIG_I2CBAUD(x) FIELD_PREP(GENMASK(31, 28), (x))
#define I3C_CONFIG		0x004
#define   I3C_CONFIG_MATCHSS	BIT(2)
#define   I3C_CONFIG_SLVENA	BIT(0)
#define I3C_STATUS		0x008
#define   I3C_STATUS_HJDIS	BIT(27)
#define   I3C_STATUS_MRDIS	BIT(25)
#define   I3C_STATUS_IBIDIS	BIT(24)
#define   I3C_STATUS_EVENT	BIT(18)
#define   I3C_STATUS_RXPEND	BIT(11)
#define   I3C_STATUS_STOP	BIT(10)
#define   I3C_STATUS_MATCHED	BIT(9)
#define   I3C_STATUS_STNOTSTOP	BIT(0)
#define   I3C_STATUS_EVENT_NACKED	2
#define   I3C_STATUS_EVENT_ACKED	3
#define I3C_CTRL		0x00C
#define   I3C_CTRL_EVENT(x) FIELD_GET(GENMASK(1, 0), (x))
#define I3C_INTSET		0x010
#define I3C_INTCLR		0x014
#define I3C_INTMASKED		0x018
#define I3C_DMACTRL		0x020
#define   DMACTRL_DMAWIDTH(x)	FIELD_PREP(GENMASK(5, 4), (x))
#define   DMACTRL_DMATB_EN	BIT(3)
#define   DMACTRL_DMAFB_EN	BIT(1)
#define I3C_DATACTRL		0x02C
#define   I3C_DATACTRL_FLUSHTB BIT(0)
#define   I3C_DATACTRL_FLUSHFB BIT(1)
#define   I3C_DATACTRL_TXCOUNT(x) FIELD_GET(GENMASK(20, 16), (x))
#define I3C_WDATAB		0x030
#define I3C_WDATABE		0x034
#define I3C_RDATAB		0x040
#define I3C_WDATAB1		0x054
#define I3C_DYNADDR		0x064
#define I3C_MAXLIMITS		0x068
#define I3C_PARTNO		0x06C
#define I3C_IDEXT		0x070
#define I3C_MCTRL		0x084
#define   I3C_MCTRL_REQUEST_START_ADDR 1
#define   I3C_MCTRL_REQUEST_STOP 2
#define   I3C_MCTRL_REQUEST_IBI_ACKNACK 3
#define   I3C_MCTRL_REQUEST_PROC_DAA 4
#define   I3C_MCTRL_REQUEST_FORCE_EXIT 6
#define   I3C_MCTRL_REQUEST_AUTO_IBI 7
#define   I3C_MCTRL_TYPE_I3C 0
#define   I3C_MCTRL_TYPE_I2C BIT(4)
#define   I3C_MCTRL_TYPE_I3C_DDR BIT(5)
#define   I3C_MCTRL_IBIRESP_AUTO 0
#define   I3C_MCTRL_IBIRESP_ACK_WITHOUT_BYTE 0
#define   I3C_MCTRL_IBIRESP_ACK_WITH_BYTE BIT(7)
#define   I3C_MCTRL_IBIRESP_NACK BIT(6)
#define   I3C_MCTRL_IBIRESP_MANUAL GENMASK(7, 6)
#define   I3C_MCTRL_DIR(x) FIELD_PREP(BIT(8), (x))
#define   I3C_MCTRL_DIR_WRITE 0
#define   I3C_MCTRL_DIR_READ 1
#define   I3C_MCTRL_ADDR(x) FIELD_PREP(GENMASK(15, 9), (x))
#define   I3C_MCTRL_RDTERM(x) FIELD_PREP(GENMASK(23, 16), (x))
#define I3C_MSTATUS		0x088
#define   I3C_MSTATUS_STATE(x) FIELD_GET(GENMASK(2, 0), (x))
#define   I3C_MSTATUS_STATE_DAA(x) (I3C_MSTATUS_STATE(x) == 5)
#define   I3C_MSTATUS_STATE_IDLE(x) (I3C_MSTATUS_STATE(x) == 0)
#define   I3C_MSTATUS_STATE_SLVREQ(x) (I3C_MSTATUS_STATE(x) == 1)
#define   I3C_MSTATUS_STATE_IBIACK(x) (I3C_MSTATUS_STATE(x) == 6)
#define   I3C_MSTATUS_BETWEEN(x) FIELD_GET(BIT(4), (x))
#define   I3C_MSTATUS_NACKED(x) FIELD_GET(BIT(5), (x))
#define   I3C_MSTATUS_IBITYPE(x) FIELD_GET(GENMASK(7, 6), (x))
#define   I3C_MSTATUS_IBITYPE_IBI 1
#define   I3C_MSTATUS_IBITYPE_MASTER_REQUEST 2
#define   I3C_MSTATUS_IBITYPE_HOT_JOIN 3
#define   I3C_MINT_SLVSTART BIT(8)
#define   I3C_MINT_MCTRLDONE BIT(9)
#define   I3C_MINT_COMPLETE BIT(10)
#define   I3C_MINT_RXPEND BIT(11)
#define   I3C_MINT_TXNOTFULL BIT(12)
#define   I3C_MINT_IBIWON BIT(13)
#define   I3C_MINT_ERRWARN BIT(15)
#define   I3C_MSTATUS_NACK BIT(5)
#define   I3C_MSTATUS_SLVSTART(x) FIELD_GET(I3C_MINT_SLVSTART, (x))
#define   I3C_MSTATUS_MCTRLDONE(x) FIELD_GET(I3C_MINT_MCTRLDONE, (x))
#define   I3C_MSTATUS_COMPLETE(x) FIELD_GET(I3C_MINT_COMPLETE, (x))
#define   I3C_MSTATUS_RXPEND(x) FIELD_GET(I3C_MINT_RXPEND, (x))
#define   I3C_MSTATUS_TXNOTFULL(x) FIELD_GET(I3C_MINT_TXNOTFULL, (x))
#define   I3C_MSTATUS_IBIWON(x) FIELD_GET(I3C_MINT_IBIWON, (x))
#define   I3C_MSTATUS_ERRWARN(x) FIELD_GET(I3C_MINT_ERRWARN, (x))
#define   I3C_MSTATUS_IBIADDR(x) FIELD_GET(GENMASK(30, 24), (x))
#define I3C_MERRWARN		0x09C
#define   I3C_MERRWARN_TIMEOUT BIT(20)
#define I3C_MDATACTRL		0x0AC
#define   I3C_DATACTRL_FLUSHTB BIT(0)
#define   I3C_DATACTRL_FLUSHFB BIT(1)
#define   I3C_DATACTRL_RXCOUNT(x) FIELD_GET(GENMASK(28, 24), (x))
#define   I3C_DATACTRL_TXCOUNT(x) FIELD_GET(GENMASK(20, 16), (x))
#define   I3C_DATACTRL_TXFULL BIT(30)
#define   I3C_DATACTRL_RXEMPTY BIT(31)
#define I3C_MINTSET		0x090
#define I3C_MINTCLR		0x094
#define I3C_MDMACTRL		0x0A0
#define I3C_MWDATAB		0x0B0
#define I3C_MWDATABE		0x0B4
#define I3C_MRDATAB		0x0C0
#define I3C_MWDATAB1		0x0CC
#define I3C_IBIEXT1		0x140
#define I3C_IBIEXT2		0x144

/* PMC register */
#define PMC_SW_RST1		0x13

/* Linux-style MMIO accessors built on Zephyr sys_read/write */
#ifndef readl
#define readl(addr)        sys_read32((mm_reg_t)(addr))
#endif
#ifndef writel
#define writel(val, addr)  sys_write32((val), (mm_reg_t)(addr))
#endif

/* Address slot status used by the address allocation bitmap */
enum i3c_addr_slot_status {
	I3C_ADDR_SLOT_FREE    = 0,
	I3C_ADDR_SLOT_RSVD    = 1,
	I3C_ADDR_SLOT_I3C_DEV = 2,
	I3C_ADDR_SLOT_I2C_DEV = 3,
};

/* Device configuration structure */
struct i3c_npcm4_config {
	int inst_id;

	/* Slave config */
	int assigned_addr;
	uint8_t bcr;
	uint8_t dcr;
	uint16_t part_id; /* PID[3:2] */
	uint16_t vendor_def_id; /* PID[1:0] */

	int dma_tx_channel;
	int dma_rx_channel;
	uintptr_t regs;
	struct pdma_reg *pdma_base;
	uintptr_t pmc_base;
	uint32_t irq;
	struct npcm4xx_clk_cfg clk_cfg;
	bool slave;
	bool secondary;
	uint32_t i3c_scl_hz;
	uint32_t i2c_scl_hz;
	int32_t hj_timeout_ms;
	bool enable_hj;
};

typedef int (*isr_cb_t)(const struct device *dev);

/* Device runtime data structure */
struct i3c_npcm4_obj {
	const struct device *dev;
	uint32_t apb3_rate;

	/* Slave mode data */
	struct i3c_slave_setup slave_data;
	int state;

	/* Semaphore for complete event */
	struct k_sem complete;
	int xfer_ret;

	/* Prevent racing between ISR and transfer */
	struct k_spinlock isr_lock;
	/* Prevent racing  between transfers */
	struct k_mutex xfer_lock;

	/* Work item for DAA process triggered by hot-join event */
	struct k_work hj_work;

	/* DMA structure */
	uint8_t *dma_buf;
	struct dsct_reg *rx_desc;
	struct dsct_reg *tx_desc;
	uint32_t txlen;
	bool use_dma_tx;

	/* Master: maintain device list */
	uint32_t free_slots;
	uint8_t addrs[I3C_MAX_DEVS];
	struct i3c_dev_desc *i3c_devs[I3C_MAX_DEVS];
	/* 2-bit address slot bitmap: covers addr 0x00..0x7F (128 × 2 = 256 bits) */
	uint64_t addr_slots[4];

	/* ISR callbacks */
	isr_cb_t isr_cb[NUM_CALLBACKS];
};

struct i3c_context {
	uint32_t config;
	uint32_t idext;
	uint32_t partno;
	uint32_t maxlimits;
};

struct i3c_cmd {
	uint8_t type;
	uint8_t addr;
	uint8_t rnw;
	uint16_t len;
	uint16_t read_len;
	uint8_t *buf;
	int err;
	bool continued;
};

struct i3c_dev_data {
	uint8_t index;
	struct {
		int enable;
		struct i3c_ibi_callbacks *callbacks;
		struct i3c_dev_desc *context;
		struct i3c_ibi_payload *incomplete;
	} ibi;
};

static char dma_buf_pool[DMA_BUF_SIZE * NUM_MODULES];
/* DSCT table for scatter-gather mode */
struct dsct_reg sg_dsct[NUM_MODULES * 2] __aligned(256);

int i3c_npcm4_slave_get_dynamic_addr(const struct device *dev, uint8_t *dynamic_addr);
static int i3c_npcm4_isr_ibi_event(const struct device *dev);

/*
 * readl_poll_timeout - Poll until condition becomes true or timeout (may sleep)
 * @addr:       address to poll
 * @val:        variable to store the read value (must be declared by caller)
 * @cond:       condition expression using val
 * @delay_us:   delay between polls in microseconds (0 = yield)
 * @timeout_us: timeout in microseconds
 */
#define readl_poll_timeout(addr, val, cond, delay_us, timeout_us)	\
({									\
	int64_t __deadline = k_uptime_get() + (timeout_us) / 1000;	\
	int __ret = -ETIMEDOUT;						\
	do {								\
		(val) = readl(addr);					\
		if (cond) {						\
			__ret = 0; break;				\
		}							\
		if ((delay_us) > 0)					\
			k_usleep(delay_us);				\
		else							\
			k_yield();					\
	} while (k_uptime_get() <= __deadline);				\
	__ret;								\
})

/*
 * readl_poll_timeout_atomic - Poll until condition becomes true (no sleep)
 * Same signatures as readl_poll_timeout.
 * Safe to use in IRQ/atomic context; uses k_busy_wait() instead of sleep.
 */
#define readl_poll_timeout_atomic(addr, val, cond, delay_us, timeout_us)	\
({									\
	int64_t __deadline = k_uptime_get() + (timeout_us) / 1000;	\
	int __ret = -ETIMEDOUT;						\
	do {								\
		(val) = readl(addr);					\
		if (cond) {						\
			__ret = 0; break;				\
		}							\
		if ((delay_us) > 0)					\
			k_busy_wait(delay_us);				\
	} while (k_uptime_get() <= __deadline);				\
	__ret;								\
})

/* DMA API implementations */

static inline void i3c_npcm4_stop_dma_rx(const struct device *dev)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	uint32_t reg_dmactl = config->slave ? I3C_DMACTRL : I3C_MDMACTRL;
	uint32_t val = readl(config->regs + reg_dmactl) & ~GENMASK(1, 0);

	config->pdma_base->STOP = (1 << config->dma_rx_channel);
	writel(val, config->regs + reg_dmactl);
	obj->rx_desc = NULL;
	if (config->slave)
		obj->state &= ~SLAVE_WAIT_FOR_RX;
}

static inline void i3c_npcm4_stop_dma_tx(const struct device *dev)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	uint32_t reg_dmactl = config->slave ? I3C_DMACTRL : I3C_MDMACTRL;
	uint32_t val = readl(config->regs + reg_dmactl) & ~GENMASK(3, 2);

	config->pdma_base->STOP = (1 << config->dma_tx_channel);
	writel(val, config->regs + reg_dmactl);
	obj->tx_desc = NULL;
}

static int i3c_npcm4_dma_read(const struct device *dev, int len)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct dsct_reg *desc = PDMA_DSCT(config, config->dma_rx_channel);
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	uint32_t reg_dmactl = config->slave ? I3C_DMACTRL : I3C_MDMACTRL;
	uint32_t val = readl(config->regs + reg_dmactl) & ~GENMASK(1, 0);

	if (len < 1 || len > DMA_BUF_SIZE)
		return -EINVAL;

	obj->rx_desc = desc;
	obj->state |= SLAVE_WAIT_FOR_RX;
	/*
	 * Fill the pdma descriptor:
	 * Transfer width: 8 bit
	 * Source Address: fixed
	 * Destination Address: increaded by 1 transfer width
	 * Single request type
	 * Basic mode
	 */
	desc->CTL = ((len - 1) << 16) | TXWIDTH_8 | SAINC_FIXED | DAINC_1 |
			REQ_TYPE_SINGLE | BASIC_MODE;
	if (config->slave)
		desc->ENDSA = (uint32_t)(config->regs + I3C_RDATAB);
	else
		desc->ENDSA = (uint32_t)(config->regs + I3C_MRDATAB);
	desc->ENDDA = (uint32_t)obj->dma_buf;
	desc->NEXT = 0;

	/* Enable PDMA RX channel */
	config->pdma_base->CHCTL |= 1 << config->dma_rx_channel;

	/*
	 * Enable I3C DMA (setup DMACTRL)
	 * transfer width: 1 byte
	 * direction: DMA from bus
	 */
	writel(val | DMACTRL_DMAFB_EN, config->regs + reg_dmactl);

	return 0;
}

static int i3c_npcm4_dma_write(const struct device *dev, uint8_t *buf, int len)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct dsct_reg *head = PDMA_DSCT(config, config->dma_tx_channel);
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	uint32_t reg_dmactl = config->slave ? I3C_DMACTRL : I3C_MDMACTRL;
	struct dsct_reg *desc = &sg_dsct[config->inst_id * 2];
	uint32_t val = readl(config->regs + reg_dmactl) & ~GENMASK(3, 2);

	obj->tx_desc = head;
	obj->txlen = len;

	if (len < 1 || len > DMA_BUF_SIZE) {
		LOG_ERR("Invalid write len %d", len);
		return -EINVAL;
	}

	/*
	 * Use scatter-gather mode for DMA write.
	 * Setup PDMA_DSCT struct
	 * Scatter-Gather mode, Next to sg descriptor
	 */
	head->CTL = SG_MODE;
	head->NEXT = (uint32_t)desc;

	/*
	 * Setup first descriptor
	 * Transfer width: 8 bit
	 * Source Address: fixed
	 * Destination Address: increment
	 * Single request type, Scatter-gather mode
	 */
	if (len > 1) {
		desc->CTL = ((len - 2) << 16) | TXWIDTH_8 | SAINC_1 | DAINC_FIXED |
			     REQ_TYPE_SINGLE | SG_MODE;
		desc->ENDSA = (uint32_t)buf;
		if (config->slave)
			desc->ENDDA = (uint32_t)(config->regs + I3C_WDATAB1);
		else
			desc->ENDDA = (uint32_t)(config->regs + I3C_MWDATAB1);
		desc->NEXT = (uint32_t)desc + 16;
		desc++;
	}

	/*
	 * Setup last descriptor for last byte destinated to WDATABE
	 * Transfer width: 8 bit
	 * Source Address: fixed
	 * Destination Address: increment
	 * Single request type, basic mode
	 */
	desc->CTL = (0 << 16) | TXWIDTH_8 | SAINC_1 | DAINC_FIXED |
		     REQ_TYPE_SINGLE | BASIC_MODE;
	desc->ENDSA = (uint32_t)&buf[len - 1];
	if (config->slave)
		desc->ENDDA = (uint32_t)(config->regs + I3C_WDATABE);
	else
		desc->ENDDA = (uint32_t)(config->regs + I3C_MWDATABE);
	desc->NEXT = 0;

	/* Enable PDMA TX channel */
	config->pdma_base->CHCTL |= 1 << config->dma_tx_channel;

	/*
	 * Setup I3C DMACTRL
	 * transfer width: 1 byte
	 * direction: DMA to bus
	 */
	writel(val | DMACTRL_DMATB_EN, config->regs + reg_dmactl);

	return 0;
}

/* Slave mode API implementations */

static void i3c_npcm4_register_isr_cb(const struct device *dev, int irq, isr_cb_t cb)
{
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	const struct i3c_npcm4_config *config = DEV_CFG(dev);

	if (irq < IRQ_START || irq > IRQ_EVENT)
		return;

	obj->isr_cb[irq - IRQ_START] = cb;

	if (cb) {
		/* Clear IRQ status and enable interrupt */
		writel(BIT(irq), config->regs + I3C_STATUS);
		writel(BIT(irq), config->regs + I3C_INTSET);
	} else {
		/* Disable interrupt */
		writel(BIT(irq), config->regs + I3C_INTCLR);
	}
}

static void i3c_npcm4_slave_reset(const struct device *dev, struct i3c_context *ctx)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	uint32_t val;

	if (ctx) {
		/* Store the context */
		ctx->idext = readl(config->regs + I3C_IDEXT);
		ctx->partno = readl(config->regs + I3C_PARTNO);
		ctx->maxlimits = readl(config->regs + I3C_MAXLIMITS);
		ctx->config = readl(config->regs + I3C_CONFIG);
	}

	if (obj->tx_desc) {
		i3c_npcm4_stop_dma_tx(dev);
	}
	if (obj->rx_desc) {
		i3c_npcm4_stop_dma_rx(dev);
	}

	/* I3C software reset */
	val = sys_read8(config->pmc_base + PMC_SW_RST1);
	sys_write8(val | BIT(config->inst_id), config->pmc_base + PMC_SW_RST1);
	readl_poll_timeout(config->regs + I3C_PARTNO, val, !val, 0, 10);
	sys_write8(val, config->pmc_base + PMC_SW_RST1);

	obj->state = 0;
	/* Clear ISR callbacks */
	for (int i = 0; i < NUM_CALLBACKS; i++)
		obj->isr_cb[i] = NULL;

}

static void i3c_npcm4_slave_restore(const struct device *dev, struct i3c_context *ctx)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);

	if (!ctx)
		return;

	writel(ctx->idext, config->regs + I3C_IDEXT);
	writel(ctx->partno, config->regs + I3C_PARTNO);
	writel(ctx->maxlimits, config->regs + I3C_MAXLIMITS);
	writel(ctx->config, config->regs + I3C_CONFIG);
}

static int i3c_npcm4_slave_write_fifo(uintptr_t regs, uint8_t *buf, int len)
{
	int i;

	for (i = 0; i < len - 1; i++)
		writel(buf[i], regs + I3C_WDATAB);

	/* Write last byte */
	writel(buf[len - 1], regs + I3C_WDATABE);
	LOG_DBG("write %d bytes to fifo done", len);

	return 0;
}

static int i3c_npcm4_slave_write(const struct device *dev, uint8_t *buf, int len)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	int ret;

	if (len <= 0 || len > DMA_BUF_SIZE)
		return -EINVAL;

	/* Flush TX FIFO */
	writel(I3C_DATACTRL_FLUSHTB, config->regs + I3C_DATACTRL);

	/* Write to FIFO directly if data length is less than FIFO space */
	if (len <= I3C_NPCM4_FIFO_SIZE) {
		obj->use_dma_tx = false;
		return i3c_npcm4_slave_write_fifo(config->regs, buf, len);
	}

	/* Start the DMA write operation */
	obj->use_dma_tx = true;
	ret = i3c_npcm4_dma_write(dev, buf, len);
	if (ret) {
		obj->use_dma_tx = false;
		obj->tx_desc = NULL;
	}

	return ret;
}

static int i3c_npcm4_slave_generate_ibi(const struct device *dev, uint8_t *payload, int len)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	uint32_t ctrl_val, ibiext_val;
	int i;

	if (!payload || len <= 0) {
		LOG_ERR("Invalid IBI payload");
		return -EINVAL;
	}

	if (len > 8) {
		LOG_ERR("IBI data too large");
		return -EINVAL;
	}
	LOG_DBG("generate IBI payload, len = %d", len);
	ctrl_val = (payload[0] << 8) | BIT(0); /* IBI event */

	if (len > 4) {
		ibiext_val = 0;
		for (i = 4; i < len; i++)
			ibiext_val |= (payload[i] << ((i - 4) * 8));
		writel(ibiext_val, config->regs + I3C_IBIEXT2);
	}
	if (len > 1) {
		ctrl_val |= BIT(3); /* Has extended IBI data */
		ibiext_val = len - 1;
		for (i = 1; i < 4; i++)
			ibiext_val |= (payload[i] << (i * 8));
		writel(ibiext_val, config->regs + I3C_IBIEXT1);
	}

	writel(I3C_STATUS_EVENT, config->regs + I3C_STATUS);
	i3c_npcm4_register_isr_cb(dev, IRQ_EVENT, i3c_npcm4_isr_ibi_event);
	writel(ctrl_val, config->regs + I3C_CTRL);

	return 0;
}

static inline void i3c_npcm4_rollback_event_state(const struct device *dev)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	int retry = 100;

	i3c_npcm4_register_isr_cb(dev, IRQ_EVENT, NULL);

	/*
	 * Stop event request.
	 * Retrying may be required to cancel the HJ event.
	 */
	while (retry--) {
		if (I3C_CTRL_EVENT(readl(config->regs + I3C_CTRL)))
			writel(0, config->regs + I3C_CTRL);
		else
			break;
	}
}

static int i3c_npcm4_isr_rx_done(const struct device *dev)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	struct dsct_reg *desc = obj->rx_desc;
	uintptr_t regs = config->regs;
	const struct i3c_slave_callbacks *cb;
	struct i3c_slave_payload *payload;
	uint32_t val;
	int rxcnt;

	if (!desc)
		return 0;

	/* Wait for RX FIFO empty */
	if (readl_poll_timeout_atomic(regs + I3C_STATUS, val,
				      !(val & I3C_STATUS_RXPEND), 0, 1000) != 0) {
		LOG_WRN("RX FIFO is not empty, flush FIFO and restart DMA");
		i3c_npcm4_stop_dma_rx(dev);
		writel(I3C_DATACTRL_FLUSHFB, regs + I3C_DATACTRL);
		i3c_npcm4_dma_read(dev, DMA_BUF_SIZE);
		return 0;
	}

	/* Get transfer count */
	val = FIELD_GET(GENMASK(29, 16), desc->CTL);
	if ((val == 0) && (config->pdma_base->TDSTS & BIT(config->dma_rx_channel))) {
		LOG_DBG("DMA transfer is finished");
		rxcnt = DMA_BUF_SIZE;
		/* Clear flag */
		config->pdma_base->TDSTS = BIT(config->dma_rx_channel);
	} else {
		rxcnt = DMA_BUF_SIZE - (val + 1);
	}

	if (rxcnt > 0) {
		LOG_DBG("RX DONE(%d bytes)", rxcnt);
		i3c_npcm4_stop_dma_rx(dev);
	} else {
		LOG_DBG("no rx data");
		return 0;
	}

	/* Execute slave callbacks */
	cb = obj->slave_data.callbacks;
	if (!cb) {
		LOG_WRN("No slave callbacks registered");
		goto quit;
	}
	if (cb->write_requested) {
		payload = cb->write_requested(obj->slave_data.dev);
		payload->size = rxcnt;
		memcpy(payload->buf, obj->dma_buf, rxcnt);
	}
	if (cb->write_done) {
		cb->write_done(obj->slave_data.dev);
	}
quit:
	/* Start a new dma operation for next transfer */
	i3c_npcm4_dma_read(dev, DMA_BUF_SIZE);

	return 0;
}

static int i3c_npcm4_isr_tx_done(const struct device *dev)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	struct dsct_reg *desc = obj->tx_desc;
	uint32_t val, tx_done_flag;
	int txcnt;
	int ret = 0;

	if (!obj->use_dma_tx) {
		val = readl(config->regs + I3C_DATACTRL);
		if (I3C_DATACTRL_TXCOUNT(val)) {
			LOG_WRN("TX FIFO is not empty: %d", (int)I3C_DATACTRL_TXCOUNT(val));
			/* Flush TX FIFO */
			writel(I3C_DATACTRL_FLUSHTB, config->regs + I3C_DATACTRL);
		}
		LOG_DBG("FIFO TX DONE");
		obj->xfer_ret = 0;
		obj->state &= ~SLAVE_WAIT_FOR_TX;
		k_sem_give(&obj->complete);
		return 0;
	}

	if (!desc) {
		LOG_WRN("No tx desc");
		ret = -EFAULT;
		goto err_quit;
	}

	/* Wait for Transfer Done flag */
	tx_done_flag = BIT(config->dma_tx_channel);
	if (readl_poll_timeout_atomic((uintptr_t)&config->pdma_base->TDSTS, val,
				      (val & tx_done_flag) == tx_done_flag, 0, 1000) != 0) {
		LOG_WRN("DMA transfer is not finished");
		ret = -ETIMEDOUT;
		goto err_quit;
	}
	/* Clear flag */
	config->pdma_base->TDSTS = (1 << config->dma_tx_channel);

	txcnt = obj->txlen - FIELD_GET(GENMASK(29, 16), desc->CTL);
	if (txcnt > 0) {
		LOG_DBG("DMA TX DONE(%d bytes)", txcnt);
	} else {
		LOG_WRN("tx count is 0");
	}

err_quit:
	obj->xfer_ret = ret;
	obj->state &= ~SLAVE_WAIT_FOR_TX;
	i3c_npcm4_stop_dma_tx(dev);
	k_sem_give(&obj->complete);

	return ret;
}

static int i3c_npcm4_isr_ibi_event(const struct device *dev)
{
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);

	i3c_npcm4_register_isr_cb(dev, IRQ_EVENT, NULL);

	if (obj->state & SLAVE_HAS_PENDING_TX)
		obj->state |= SLAVE_WAIT_FOR_TX;
	else
		k_sem_give(&obj->complete);

	return 0;
}

static int i3c_npcm4_isr_hj_event(const struct device *dev)
{
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);

	i3c_npcm4_register_isr_cb(dev, IRQ_EVENT, NULL);

	k_sem_give(&obj->complete);

	return 0;
}

static int i3c_npcm4_isr_receive_stop(const struct device *dev)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	uint32_t val;

	LOG_DBG("Received STOP: state=0x%x", obj->state);
	val = readl(config->regs + I3C_STATUS);
	if ((val & I3C_STATUS_MATCHED) == 0) {
		LOG_INF("Address not matched: Status=0x%08x", val);
		return 0;
	}
	/* Clear MATCHED */
	writel(I3C_STATUS_MATCHED, config->regs + I3C_STATUS);

	if ((obj->state & SLAVE_WAIT_FOR_RX) && obj->rx_desc) {
		i3c_npcm4_isr_rx_done(dev);
	}
	if ((obj->state & SLAVE_WAIT_FOR_TX)) {
		i3c_npcm4_isr_tx_done(dev);
	}

	return 0;
}

static int i3c_npcm4_isr_da_changed(const struct device *dev)
{
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	uint8_t addr = 0;


	i3c_npcm4_slave_get_dynamic_addr(dev, &addr);
	LOG_WRN("DA changed: 0x%x", addr);

	if (addr) {
		obj->state |= SLAVE_DA_ASSIGNED;
		if (!obj->rx_desc)
			i3c_npcm4_dma_read(dev, DMA_BUF_SIZE);
		i3c_npcm4_register_isr_cb(dev, IRQ_STOP, i3c_npcm4_isr_receive_stop);
	} else {
		i3c_npcm4_stop_dma_rx(dev);
		obj->state &= ~SLAVE_DA_ASSIGNED;
		i3c_npcm4_register_isr_cb(dev, IRQ_STOP, NULL);
	}

	return 0;
}

static void i3c_npcm4_slave_isr(const struct device *dev)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	uint32_t status = readl(config->regs + I3C_INTMASKED);
	int i;

	LOG_DBG("status=0x%x", status);
	for (i = IRQ_START; i <= IRQ_EVENT; i++) {
		if (status & (1 << i)) {
			/* Clear status bit */
			writel(1 << i, config->regs + I3C_STATUS);

			/* Run callbacks */
			if (obj->isr_cb[(i - IRQ_START)])
				obj->isr_cb[(i - IRQ_START)](dev);
		}
	}
}


/**
 * @brief Register as I3C slave device
 */
int i3c_npcm4_slave_register(const struct device *dev, struct i3c_slave_setup *slave_data)
{
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	uint32_t status;
	uint8_t addr;

	if (!slave_data || !slave_data->callbacks) {
		LOG_ERR("Invalid slave setup");
		return -EINVAL;
	}

	obj->slave_data.max_payload_len = slave_data->max_payload_len;
	obj->slave_data.callbacks = slave_data->callbacks;
	obj->slave_data.dev = slave_data->dev;

	/* Clear status */
	status = readl(config->regs + I3C_STATUS);
	writel(status, config->regs + I3C_STATUS);

	if (i3c_npcm4_slave_get_dynamic_addr(dev, &addr) == 0) {
		obj->state |= SLAVE_DA_ASSIGNED;
		if (!obj->rx_desc)
			i3c_npcm4_dma_read(dev, DMA_BUF_SIZE);
		i3c_npcm4_register_isr_cb(dev, IRQ_STOP, i3c_npcm4_isr_receive_stop);
	} else {
		obj->state &= ~SLAVE_DA_ASSIGNED;
	}

	i3c_npcm4_register_isr_cb(dev, IRQ_DACHG, i3c_npcm4_isr_da_changed);

	return 0;
}

/**
 * @brief Prepare read data for master read transfer
 */
int i3c_npcm4_slave_put_read_data(const struct device *dev, struct i3c_slave_payload *data,
				  struct i3c_ibi_payload *ibi_notify)
{
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	uint32_t val;
	int ret;

	__ASSERT_NO_MSG(data);
	__ASSERT_NO_MSG(data->buf);
	__ASSERT_NO_MSG(data->size);

	LOG_DBG("put data: size %d", data->size);

	if (!config->slave)
		return 0;

	/* Wait for bus STOP */
	if (readl_poll_timeout(config->regs + I3C_STATUS, val,
			       !(val & I3C_STATUS_STNOTSTOP), 0, 100000) != 0) {
		LOG_ERR("%s: Bus is busy", __func__);
		return -EBUSY;
	}

	/* Copy data to FIFO */
	ret = i3c_npcm4_slave_write(dev, data->buf, data->size);
	if (ret) {
		LOG_ERR("Unable to write data to FIFO");
		return ret;
	}

	obj->xfer_ret = 0;
	k_sem_init(&obj->complete, 0, 1);

	if (ibi_notify) {
		obj->state |= SLAVE_HAS_PENDING_TX;
		/* Generate an IBI event */
		ret = i3c_npcm4_slave_generate_ibi(dev, ibi_notify->buf,
						   ibi_notify->size);
		if (ret) {
			obj->state &= ~SLAVE_HAS_PENDING_TX;
			return ret;
		}
	} else {
		obj->state |= SLAVE_WAIT_FOR_TX;
	}

	/* Wait for complete */
	if (k_sem_take(&obj->complete, K_MSEC(3000)))
		ret = -ETIMEDOUT;
	else if (obj->xfer_ret)
		ret = obj->xfer_ret;

	if (ret) {
		LOG_ERR("Send data failed: %d", ret);
		if (obj->use_dma_tx)
			i3c_npcm4_stop_dma_tx(dev);
		i3c_npcm4_rollback_event_state(dev);
	}
	obj->state &= ~(SLAVE_HAS_PENDING_TX | SLAVE_WAIT_FOR_TX);

	return ret;
}

/**
 * @brief Query whether controller is configured as slave mode
 */
int i3c_npcm4_is_slave_mode(const struct device *dev)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);

	return config->slave ? 1 : 0;
}

/**
 * @brief Get dynamically assigned address in slave mode
 */
int i3c_npcm4_slave_get_dynamic_addr(const struct device *dev, uint8_t *dynamic_addr)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	uint32_t val;

	val = readl(config->regs + I3C_DYNADDR);
	if ((val & BIT(0)) == 0) {
		LOG_DBG("No invalid dynamic addr");
		return -EINVAL;
	}

	*dynamic_addr = (uint8_t)(FIELD_GET(GENMASK(7, 1), val));
	LOG_DBG("dynaddr=0x%x", *dynamic_addr);

	return 0;
}

/**
 * @brief Get event enabling status in slave mode
 */
int i3c_npcm4_slave_get_event_enabling(const struct device *dev, uint32_t *event_en)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	uint32_t status;

	status = readl(config->regs + I3C_STATUS);

	*event_en = 0;
	if ((status & I3C_STATUS_IBIDIS) == 0)
		*event_en |= I3C_SLAVE_EVENT_SIR;
	if ((status & I3C_STATUS_MRDIS) == 0)
		*event_en |= I3C_SLAVE_EVENT_MR;
	if ((status & I3C_STATUS_HJDIS) == 0)
		*event_en |= I3C_SLAVE_EVENT_HJ;

	LOG_DBG("event_en = 0x%x", *event_en);

	return 0;
}

/**
 * @brief Send SIR (Slave Interrupt Request) with payload
 */
int i3c_npcm4_slave_send_sir(const struct device *dev, struct i3c_ibi_payload *payload)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	uint8_t addr = 0;
	uint32_t val;
	int ret;

	LOG_DBG("Send IBI request");

	if (!config->slave)
		return 0;

	if (i3c_npcm4_slave_get_dynamic_addr(dev, &addr) < 0) {
		LOG_ERR("No valid Dynamic address\n");
		return -EINVAL;
	}

	if (readl(config->regs + I3C_STATUS) & I3C_STATUS_IBIDIS) {
		LOG_ERR("IBI event is disabled\n");
		return -EINVAL;
	}

	if (!payload)
		return -EINVAL;

	/* Wait for bus STOP */
	if (readl_poll_timeout(config->regs + I3C_STATUS, val,
			       !(val & I3C_STATUS_STNOTSTOP), 0, 10000) != 0) {
		LOG_ERR("%s: Bus is busy", __func__);
		return -EBUSY;
	}

	k_sem_init(&obj->complete, 0, 1);
	/* Generate an IBI event */
	ret = i3c_npcm4_slave_generate_ibi(dev, payload->buf, payload->size);
	if (ret)
		return ret;

	/* Wait for complete */
	if (k_sem_take(&obj->complete, K_MSEC(3000))) {
		LOG_ERR("IBI timeout");
		i3c_npcm4_rollback_event_state(dev);
		return -ETIMEDOUT;
	}

	return 0;
}

int i3c_npcm4_slave_hj_req(const struct device *dev)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	struct i3c_context ctx;
	uint32_t timeout_ms;
	uint8_t addr;
	int ret = 0;

	LOG_DBG("Send HJ req");

	if (!config->slave)
		return 0;

	if (i3c_npcm4_slave_get_dynamic_addr(dev, &addr) == 0) {
		LOG_WRN("Dynamic address present = 0x%x\n", addr);
		return 0;
	}

	if (readl(config->regs + I3C_STATUS) & I3C_STATUS_HJDIS) {
		LOG_ERR("Hot-join event is disabled\n");
		return -EINVAL;
	}

	k_sem_init(&obj->complete, 0, 1);
	i3c_npcm4_slave_reset(dev, &ctx);

	/* Set HJ event */
	i3c_npcm4_register_isr_cb(dev, IRQ_EVENT, i3c_npcm4_isr_hj_event);
	writel(0x3, config->regs + I3C_CTRL);

	/* Enable slave */
	i3c_npcm4_slave_restore(dev, &ctx);

	/* Wait for complete */
	timeout_ms = config->hj_timeout_ms > 0 ? config->hj_timeout_ms : 100;
	if (k_sem_take(&obj->complete, K_MSEC(timeout_ms))) {
		i3c_npcm4_rollback_event_state(dev);
		LOG_INF("HJ event timeout");
		ret = -ETIMEDOUT;
	} else {
		LOG_INF("HJ event sent");
	}
	i3c_npcm4_register_isr_cb(dev, IRQ_DACHG, i3c_npcm4_isr_da_changed);

	return ret;
}

/**
 * @brief Set static address in slave mode
 */
int i3c_npcm4_slave_set_static_addr(const struct device *dev, uint8_t static_addr)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	uint32_t val;

	/* Update bit field [31:25] in the CONFIG register */
	LOG_INF("change static addr to 0x%x", static_addr);
	val = readl(config->regs + I3C_CONFIG) & ~GENMASK(31, 25);
	val |= FIELD_PREP(GENMASK(31, 25), static_addr);
	writel(val, config->regs + I3C_CONFIG);

	return 0;
}

/**
 * @brief Set PID extra info
 */
int i3c_npcm4_set_pid_extra_info(const struct device *dev, uint16_t extra_info)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	uint32_t val;

	/* Update bit field [11:0] in the PARTNO register */
	LOG_INF("change PID extra_info to 0x%04x", extra_info);
	val = readl(config->regs + I3C_PARTNO) & ~GENMASK(11, 0);
	val |= FIELD_PREP(GENMASK(11, 0), extra_info);
	writel(val, config->regs + I3C_PARTNO);

	return 0;
}

/* End of Slave mode API implementations */

/* Master mode API implementations */

static void i3c_addr_slot_set(struct i3c_npcm4_obj *obj, uint8_t addr,
			      enum i3c_addr_slot_status status)
{
	uint32_t shift = ((uint32_t)addr * 2) % 64;
	uint32_t word  = ((uint32_t)addr * 2) / 64;

	obj->addr_slots[word] &= ~(3ULL << shift);
	obj->addr_slots[word] |= ((uint64_t)status << shift);
}

static enum i3c_addr_slot_status i3c_addr_slot_get(struct i3c_npcm4_obj *obj,
						   uint8_t addr)
{
	uint32_t shift = ((uint32_t)addr * 2) % 64;
	uint32_t word  = ((uint32_t)addr * 2) / 64;

	return (enum i3c_addr_slot_status)((obj->addr_slots[word] >> shift) & 3ULL);
}

static int i3c_npcm4_master_reserve_slot(struct i3c_npcm4_obj *master)
{
	unsigned int slot;

	if (!(master->free_slots & GENMASK(I3C_MAX_DEVS - 1, 0)))
		return -ENOSPC;

	slot = find_lsb_set(master->free_slots) - 1;

	master->free_slots &= ~BIT(slot);

	return slot;
}

static void i3c_npcm4_master_release_slot(struct i3c_npcm4_obj *master,
					  unsigned int slot)
{
	master->free_slots |= BIT(slot);
}

static int i3c_master_get_free_addr(struct i3c_npcm4_obj *obj, uint8_t start_addr)
{
	uint8_t addr;

	for (addr = start_addr; addr < I3C_MAX_ADDR; addr++) {
		if (i3c_addr_slot_get(obj, addr) == I3C_ADDR_SLOT_FREE)
			return addr;
	}

	return -EADDRNOTAVAIL;
}

static struct i3c_dev_desc *
i3c_npcm4_master_dev_from_pid(struct i3c_npcm4_obj *master, uint64_t pid)
{
	int i;

	for (i = 0; i < I3C_MAX_DEVS; i++)
		if (master->i3c_devs[i] && master->i3c_devs[i]->info.pid == pid)
			break;

	if (i == I3C_MAX_DEVS)
		return NULL;

	return master->i3c_devs[i];
}

static struct i3c_dev_desc *
i3c_npcm4_master_dev_from_addr(struct i3c_npcm4_obj *master, unsigned int ibiaddr)
{
	int i;

	for (i = 0; i < I3C_MAX_DEVS; i++)
		if (master->addrs[i] == ibiaddr)
			break;

	if (i == I3C_MAX_DEVS)
		return NULL;

	return master->i3c_devs[i];
}

static void i3c_npcm4_master_clear_merrwarn(const struct i3c_npcm4_config *master)
{
	writel(readl(master->regs + I3C_MERRWARN),
	       master->regs + I3C_MERRWARN);
}

static void i3c_npcm4_master_flush_fifo(const struct i3c_npcm4_config *master)
{
	writel(I3C_DATACTRL_FLUSHTB | I3C_DATACTRL_FLUSHFB,
	       master->regs + I3C_MDATACTRL);
}

static int i3c_npcm4_master_ack_ibi(const struct i3c_npcm4_config *master,
				    bool mandatory_byte)
{
	unsigned int ibi_ack_nack;
	uint32_t reg;

	ibi_ack_nack = I3C_MCTRL_REQUEST_IBI_ACKNACK;
	if (mandatory_byte)
		ibi_ack_nack |= I3C_MCTRL_IBIRESP_ACK_WITH_BYTE;
	else
		ibi_ack_nack |= I3C_MCTRL_IBIRESP_ACK_WITHOUT_BYTE;

	writel(ibi_ack_nack, master->regs + I3C_MCTRL);

	return readl_poll_timeout_atomic(master->regs + I3C_MSTATUS, reg,
					 I3C_MSTATUS_MCTRLDONE(reg), 1, 1000);

}

static int i3c_npcm4_master_nack_ibi(const struct i3c_npcm4_config *master)
{
	uint32_t reg;

	writel(I3C_MCTRL_REQUEST_IBI_ACKNACK |
	       I3C_MCTRL_IBIRESP_NACK,
	       master->regs + I3C_MCTRL);

	return readl_poll_timeout_atomic(master->regs + I3C_MSTATUS, reg,
					 I3C_MSTATUS_MCTRLDONE(reg), 0, 1000);
}

static void i3c_npcm4_master_emit_stop(const struct i3c_npcm4_config *master)
{
	uint32_t reg = readl(master->regs + I3C_MSTATUS);

	/* Do not emit stop in the IDLE or SLVREQ state */
	if (I3C_MSTATUS_STATE_IDLE(reg) || I3C_MSTATUS_STATE_SLVREQ(reg))
		return;

	/*
	 * A spurious IBI event may change the controller state to IBIACK;
	 * transition to NORMACT state before issuing the emitSTOP request.
	 */
	if (I3C_MSTATUS_STATE_IBIACK(reg)) {
		i3c_npcm4_master_nack_ibi(master);
		writel(I3C_MINT_IBIWON, master->regs + I3C_MSTATUS);
	}

	writel(I3C_MCTRL_REQUEST_STOP, master->regs + I3C_MCTRL);
	/*
	 * Wait for STOP condition to complete, to prevent a subsequent
	 * EmitStartAddr from being issued too quickly.
	 */
	readl_poll_timeout_atomic(master->regs + I3C_MSTATUS, reg,
				  I3C_MSTATUS_MCTRLDONE(reg), 0, 1000);
}

/**
 * @brief Handle the IBIWON condition
 */
static int i3c_npcm4_master_handle_ibi_won(const struct i3c_npcm4_config *master,
					   uint32_t mstatus)
{
	uint32_t ibitype;
	int ret = 0;

	ibitype = I3C_MSTATUS_IBITYPE(mstatus);

	writel(I3C_MINT_IBIWON, master->regs + I3C_MSTATUS);

	/* Hardware can't auto emit NACK for hot join and master request */
	switch (ibitype) {
	case I3C_MSTATUS_IBITYPE_HOT_JOIN:
	case I3C_MSTATUS_IBITYPE_MASTER_REQUEST:
		ret = i3c_npcm4_master_nack_ibi(master);
	}

	return ret;
}

/**
 * @brief Read pending data in Master receive FIFO
 */
static int i3c_npcm4_master_readb(const struct i3c_npcm4_config *master, uint8_t *dst,
				  unsigned int len)
{
	uint32_t reg;
	int ret, i;

	for (i = 0; i < len; i++) {
		ret = readl_poll_timeout_atomic(master->regs + I3C_MSTATUS,
						reg,
						I3C_MSTATUS_RXPEND(reg),
						0, 1000);
		if (ret)
			return ret;

		dst[i] = readl(master->regs + I3C_MRDATAB);
	}

	return 0;
}

/**
 * @brief Drain the Master RX FIFO until the I3C transfer completes
 */
static int i3c_npcm4_master_read(const struct i3c_npcm4_config *master,
				 uint8_t *in, unsigned int len)
{
	int offset = 0, i;
	uint32_t mdctrl, mstatus;
	bool completed = false;
	unsigned int count;

	while (!completed) {
		mstatus = readl(master->regs + I3C_MSTATUS);
		if (I3C_MSTATUS_COMPLETE(mstatus) != 0)
			completed = true;

		mdctrl = readl(master->regs + I3C_MDATACTRL);
		count = I3C_DATACTRL_RXCOUNT(mdctrl);
		if (offset + count > len) {
			LOG_ERR("I3C receive length too long!");
			return -EINVAL;
		}
		for (i = 0; i < count; i++)
			in[offset + i] = readl(master->regs + I3C_MRDATAB);

		offset += count;
	}

	return offset;
}

/**
 * @brief Write data bytes to the Master TX FIFO, blocking until FIFO space is available
 */
static int i3c_npcm4_master_write(const struct i3c_npcm4_config *master,
				  const uint8_t *out, unsigned int len)
{
	int offset = 0, ret;
	uint32_t mdctrl;

	while (offset < len) {
		ret = readl_poll_timeout_atomic(master->regs + I3C_MDATACTRL,
						mdctrl,
						!(mdctrl & I3C_DATACTRL_TXFULL),
						0, 1000);
		if (ret)
			return ret;

		/*
		 * The last byte must be written to MWDATABE to signal
		 * the end of the transfer.
		 */
		if (likely(offset < (len - 1)))
			writel(out[offset++], master->regs + I3C_MWDATAB);
		else
			writel(out[offset++], master->regs + I3C_MWDATABE);
	}

	return 0;
}

/**
 * @brief Perform one I3C privae or CCC transfer
 */
static int i3c_npcm4_master_transfer_one(const struct device *dev,
					 struct i3c_cmd *cmd)
{
	const struct i3c_npcm4_config *master = DEV_CFG(dev);
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	bool no_data = cmd->len ? false : true;
	uint32_t rdterm = cmd->len;
	uint32_t mstatus, reg;
	bool restarted = false;
	bool use_dma = (cmd->len > I3C_NPCM4_FIFO_SIZE);
	int ret, i, count, space;
	uint16_t len = cmd->len;
	uint8_t *buf = cmd->buf;
	k_spinlock_key_t key;

	k_mutex_lock(&obj->xfer_lock, K_FOREVER);
	key = k_spin_lock(&obj->isr_lock);
	i3c_npcm4_master_flush_fifo(master);
restart:
	writel(I3C_MCTRL_REQUEST_START_ADDR |
	       cmd->type |
	       I3C_MCTRL_IBIRESP_NACK |
	       I3C_MCTRL_DIR(cmd->rnw) |
	       I3C_MCTRL_ADDR(cmd->addr) |
	       I3C_MCTRL_RDTERM(rdterm),
	       master->regs + I3C_MCTRL);

	/* Prefill the TX FIFO (only in non-DMA path) */
	if (!cmd->rnw && len && !use_dma) {
		reg = readl(master->regs + I3C_MDATACTRL);
		space = I3C_NPCM4_FIFO_SIZE - I3C_DATACTRL_TXCOUNT(reg);
		count = len > space ? space : len;
		for (i = 0; i < count; i++) {
			if (i == len - 1)
				writel(buf[0], master->regs + I3C_MWDATABE);
			else
				writel(buf[0], master->regs + I3C_MWDATAB);
			buf++;
		}
		len -= count;
	}

	if (readl_poll_timeout_atomic(master->regs + I3C_MSTATUS, reg,
				      I3C_MSTATUS_MCTRLDONE(reg), 0, 1000) != 0) {
		LOG_ERR("EmitStartAddr request timeout");
		LOG_ERR("MCTRL=0x%08x", readl(master->regs + I3C_MCTRL));
		ret = -ETIMEDOUT;
		goto emit_stop;
	}
	mstatus = readl(master->regs + I3C_MSTATUS);

	/* NACK the slave request and restart the original transfer */
	if (I3C_MSTATUS_IBIWON(mstatus) && !restarted) {
		ret = i3c_npcm4_master_handle_ibi_won(master, reg);
		if (ret)
			goto emit_stop;
		restarted = true;
		goto restart;
	}

	if (I3C_MSTATUS_NACKED(mstatus)) {
		LOG_DBG("addr 0x%x NACK\n", cmd->addr);
		ret = -EIO;
		goto emit_stop;
	}

	if (cmd->rnw)
		ret = use_dma ? i3c_npcm4_dma_read(dev, len)
			      : i3c_npcm4_master_read(master, buf, len);
	else
		ret = use_dma ? i3c_npcm4_dma_write(dev, buf, len)
			      : i3c_npcm4_master_write(master, buf, len);
	if (ret < 0)
		goto emit_stop;

	if (!use_dma && cmd->rnw)
		cmd->read_len = ret;

	if (!no_data) {
		if (use_dma && cmd->rnw) {
			struct dsct_reg *desc = PDMA_DSCT(master, master->dma_rx_channel);

			/* Use COMPLETE interrupt to indicate transfer completion. */
			k_sem_reset(&obj->complete);
			writel(I3C_MINT_COMPLETE, master->regs + I3C_MINTSET);

			/* Wait for COMPLETE, then confirm RX FIFO empty */
			k_spin_unlock(&obj->isr_lock, key);
			ret = k_sem_take(&obj->complete, K_MSEC(100));
			if (ret) {
				i3c_npcm4_stop_dma_rx(dev);
				LOG_WRN("RX: Wait for COMPLETE interrupt timeout");
				LOG_ERR("MSTATUS=0x%08x", readl(master->regs + I3C_MSTATUS));
				LOG_ERR("MERRWARN=0x%08x", readl(master->regs + I3C_MERRWARN));
				LOG_ERR("MCTRL=0x%08x", readl(master->regs + I3C_MCTRL));
				goto emit_stop;
			}
			/* Confirm RX FIFO empty */
			if (readl_poll_timeout_atomic(master->regs + I3C_MSTATUS, reg,
					!I3C_MSTATUS_RXPEND(reg), 0, 1000) != 0) {
				i3c_npcm4_stop_dma_rx(dev);
				LOG_WRN("RX FIFO is not empty");
				ret = -EFAULT;
				goto emit_stop;
			}

			/* Get transfer count */
			reg = FIELD_GET(GENMASK(29, 16), desc->CTL);
			if ((reg == 0) && (master->pdma_base->TDSTS & BIT(master->dma_rx_channel))) {
				LOG_DBG("DMA transfer has finished all bytes");
				cmd->read_len = len;
				/* Clear DMA transfer-done flag */
				master->pdma_base->TDSTS = BIT(master->dma_rx_channel);
			} else {
				cmd->read_len = cmd->len - (reg + 1);
			}
			LOG_DBG("RX DONE(%d bytes)", cmd->read_len);
			i3c_npcm4_stop_dma_rx(dev);
			memcpy(buf, obj->dma_buf, cmd->read_len);
		} else if (use_dma && !cmd->rnw) {
			/* Use COMPLETE interrupt to indicate transfer completion. */
			k_sem_reset(&obj->complete);
			writel(I3C_MINT_COMPLETE, master->regs + I3C_MINTSET);
			k_spin_unlock(&obj->isr_lock, key);
			/* Wait for COMPLETE */
			ret = k_sem_take(&obj->complete, K_MSEC(100));
			if (ret) {
				i3c_npcm4_stop_dma_tx(dev);
				LOG_WRN("TX: Wait for COMPLETE interrupt timeout");
				LOG_ERR("MSTATUS=0x%08x", readl(master->regs + I3C_MSTATUS));
				LOG_ERR("MERRWARN=0x%08x", readl(master->regs + I3C_MERRWARN));
				LOG_ERR("MCTRL=0x%08x", readl(master->regs + I3C_MCTRL));
				goto emit_stop;
			}

			/* Clear DMA transfer-done flag */
			master->pdma_base->TDSTS = BIT(master->dma_tx_channel);
			LOG_DBG("TX DONE(%d bytes)", cmd->len);
			i3c_npcm4_stop_dma_tx(dev);
		} else {
			ret = readl_poll_timeout_atomic(master->regs + I3C_MSTATUS, reg,
							I3C_MSTATUS_COMPLETE(reg), 0, 1000);
			if (ret)
				goto emit_stop;
		}
		writel(I3C_MINT_COMPLETE, master->regs + I3C_MSTATUS);
	}


	if (!cmd->continued)
		i3c_npcm4_master_emit_stop(master);

	if (!use_dma)
		k_spin_unlock(&obj->isr_lock, key);
	k_mutex_unlock(&obj->xfer_lock);

	return 0;
emit_stop:
	i3c_npcm4_master_flush_fifo(master);
	i3c_npcm4_master_emit_stop(master);
	i3c_npcm4_master_clear_merrwarn(master);
	if (!use_dma)
		k_spin_unlock(&obj->isr_lock, key);
	k_mutex_unlock(&obj->xfer_lock);

	return ret;
}

static int i3c_npcm4_master_send_bdcast_ccc_cmd(const struct device *dev,
						struct i3c_ccc_cmd *ccc)
{
	unsigned int xfer_len = ccc->payload.length + 1;
	struct i3c_cmd cmd;
	uint8_t *buf;
	int ret;

	buf = k_malloc(xfer_len);
	if (!buf)
		return -ENOMEM;
	buf[0] = ccc->id;
	memcpy(&buf[1], ccc->payload.data, ccc->payload.length);

	cmd.type = I3C_MCTRL_TYPE_I3C;
	cmd.addr = ccc->addr;
	cmd.rnw = ccc->rnw;
	cmd.len = xfer_len;
	cmd.buf = buf;
	cmd.continued = false;
	ret = i3c_npcm4_master_transfer_one(dev, &cmd);
	k_free(buf);

	return ret;
}

static int i3c_npcm4_master_send_direct_ccc_cmd(const struct device *dev,
						struct i3c_ccc_cmd *ccc)
{
	unsigned int xfer_len = ccc->payload.length;
	struct i3c_cmd cmd;
	int ret;

	/* Broadcast command code */
	cmd.type = I3C_MCTRL_TYPE_I3C;
	cmd.addr = I3C_BROADCAST_ADDR;
	cmd.rnw = 0;
	cmd.len = 1;
	cmd.buf = &ccc->id;
	cmd.continued = true;
	ret = i3c_npcm4_master_transfer_one(dev, &cmd);
	if (ret) {
		LOG_ERR("direct_ccc(0x%x)  broadcast err %d", ccc->id, ret);
		return -EIO;
	}

	/* Directed message */
	cmd.addr = ccc->addr;
	cmd.rnw = ccc->rnw;
	cmd.len = xfer_len;
	cmd.buf = ccc->payload.data;
	cmd.read_len = xfer_len;
	cmd.continued = false;
	ret = i3c_npcm4_master_transfer_one(dev, &cmd);
	if (ret) {
		LOG_ERR("direct_ccc(0x%x) direct message err %d", ccc->id, ret);
		return -EIO;
	}

	ccc->payload.length = cmd.read_len;

	return ret;
}

/**
 * @brief Work handler: Schedule the DAA work outside of ISR context
 */
static void i3c_npcm4_hj_work_handler(struct k_work *work)
{
	struct i3c_npcm4_obj *obj = CONTAINER_OF(work, struct i3c_npcm4_obj, hj_work);
	uint8_t addrs[I3C_MAX_DEVS];
	int dev_nb = 0;
	int ret, i;

	LOG_DBG("Hot-join: starting DAA");
	ret = i3c_npcm4_master_do_daa(obj->dev, addrs, &dev_nb);
	if (ret < 0 && ret != -ETIMEDOUT)
		LOG_ERR("Hot-join DAA failed: %d", ret);
	else
		LOG_INF("Hot-join DAA done: %d device(s) assigned", dev_nb);

	for (i = 0; i < dev_nb; i++) {
		ret = i3c_master_register_i3c_dev(obj->dev, addrs[i]);
		if (ret) {
			printk("Fail to register i3c dev@%02x\n", addrs[i]);
		}
	}
}

/**
 * @brief handle IBI in the ISR context
 */
static int i3c_npcm4_master_handle_ibi(const struct i3c_npcm4_config *master,
				       struct i3c_dev_desc *dev)
{
	struct i3c_dev_data *data = DESC_PRIV(dev);
	struct i3c_ibi_callbacks *cb = data->ibi.callbacks;
	struct i3c_ibi_payload *payload = NULL;
	int ret, val;
	int rx_len = 0;

	/*
	 * Wait for transfer to complete before returning. Otherwise, the EmitStop
	 * request might be sent before the transfer completes.
	 */
	ret = readl_poll_timeout_atomic(master->regs + I3C_MSTATUS, val,
					I3C_MSTATUS_COMPLETE(val), 0, 1000);
	if (ret) {
		LOG_ERR("Timeout when polling for COMPLETE");
		return ret;
	}

	if (cb && cb->write_requested) {
		payload = cb->write_requested(dev);
	}
	if (!payload || !payload->buf) {
		LOG_ERR("No IBI payload");
		goto end;
	}

	while (I3C_MSTATUS_RXPEND(readl(master->regs + I3C_MSTATUS))) {
		payload->buf[rx_len++] = (uint8_t)readl(master->regs + I3C_MRDATAB);
		if (rx_len == payload->max_payload_size)
			break;
	}
	payload->size = rx_len;

end:
	if (cb && cb->write_done)
		cb->write_done(dev);

	return 0;
}

static void i3c_npcm4_master_isr(const struct device *master_dev)
{
	const struct i3c_npcm4_config *master = DEV_CFG(master_dev);
	struct i3c_npcm4_obj *obj = DEV_DATA(master_dev);
	struct i3c_dev_desc *dev;
	uint32_t ibitype, ibiaddr;
	uint32_t status, val;
	k_spinlock_key_t key;
	int ret;

	key = k_spin_lock(&obj->isr_lock);

	status = readl(master->regs + I3C_MSTATUS);
	if (I3C_MSTATUS_COMPLETE(status)) {
		/* Clear COMPLETE interrupt */
		writel(I3C_MINT_COMPLETE, master->regs + I3C_MSTATUS);
		writel(I3C_MINT_COMPLETE, master->regs + I3C_MINTCLR);
		k_sem_give(&obj->complete);
	}

	if (!I3C_MSTATUS_SLVSTART(status))
		goto quit;

	/* Clear SLVSTART interrupt status */
	writel(I3C_MINT_SLVSTART, master->regs + I3C_MSTATUS);

	/* Quit Spurious SLVSTART interrupt */
	if (!I3C_MSTATUS_STATE_SLVREQ(status))
		goto quit;

	if (!(readl(master->regs + I3C_MINTSET) & I3C_MINT_SLVSTART)) {
		LOG_WRN("Slvstart occurred, but not enabled");
		LOG_INF("MSTATUS=0x%08x", readl(master->regs + I3C_MSTATUS));
		goto quit;
	}

	/* Clear IBIWON */
	writel(I3C_MINT_IBIWON, master->regs + I3C_MSTATUS);

	/* Issue REQUEST_START_ADDR to emit broadcast address for IBI arbitration */
	writel(I3C_MCTRL_REQUEST_START_ADDR |
	       I3C_MCTRL_TYPE_I3C |
	       I3C_MCTRL_IBIRESP_MANUAL |
	       I3C_MCTRL_DIR(I3C_MCTRL_DIR_WRITE) |
	       I3C_MCTRL_ADDR(I3C_BROADCAST_ADDR),
	       master->regs + I3C_MCTRL);

	/* Wait for IBIWON */
	ret = readl_poll_timeout_atomic(master->regs + I3C_MSTATUS, val,
					I3C_MSTATUS_IBIWON(val), 0, 1000);
	if (ret) {
		LOG_ERR("Timeout when polling for IBIWON");
		LOG_INF("MSTATUS=0x%08x", readl(master->regs + I3C_MSTATUS));
		LOG_INF("MCTRL=0x%08x", readl(master->regs + I3C_MCTRL));
		i3c_npcm4_master_clear_merrwarn(master);
		i3c_npcm4_master_emit_stop(master);
		goto quit;
	}

	status = readl(master->regs + I3C_MSTATUS);
	ibitype = I3C_MSTATUS_IBITYPE(status);
	ibiaddr = I3C_MSTATUS_IBIADDR(status);

	LOG_DBG("ibitype=%d ibiaddr=%d", ibitype, ibiaddr);
	LOG_DBG("ibiwon: mctrl=0x%x mstatus=0x%x",
		readl(master->regs + I3C_MCTRL), status);
	/* Handle responses to incoming requests */
	switch (ibitype) {
	case I3C_MSTATUS_IBITYPE_IBI:
		dev = i3c_npcm4_master_dev_from_addr(obj, ibiaddr);
		if (!dev) {
			i3c_npcm4_master_nack_ibi(master);
		} else {
			if (dev->info.bcr & I3C_BCR_IBI_PAYLOAD)
				i3c_npcm4_master_ack_ibi(master, true);
			else
				i3c_npcm4_master_ack_ibi(master, false);
			i3c_npcm4_master_handle_ibi(master, dev);
		}
		break;
	case I3C_MSTATUS_IBITYPE_HOT_JOIN:
		i3c_npcm4_master_ack_ibi(master, false);
		k_work_submit(&obj->hj_work);
		break;
	case I3C_MSTATUS_IBITYPE_MASTER_REQUEST:
		i3c_npcm4_master_nack_ibi(master);
		break;
	default:
		LOG_ERR("Invalid ibi type: ibitype=%d ibiaddr=%d", ibitype, ibiaddr);
		break;
	}
	i3c_npcm4_master_emit_stop(master);

	/* Prevent IRQ storms when SDA is stuck low */
	if (ibitype == I3C_MSTATUS_IBITYPE_MASTER_REQUEST)
		writel(I3C_MINT_SLVSTART, master->regs + I3C_MSTATUS);

	/* Check for error conditions */
	status = readl(master->regs + I3C_MSTATUS);
	if (I3C_MSTATUS_ERRWARN(status)) {
		uint32_t merrwarn;

		merrwarn = readl(master->regs + I3C_MERRWARN);
		writel(merrwarn, master->regs + I3C_MERRWARN);

		/* Ignore timeout error */
		if (!(merrwarn & I3C_MERRWARN_TIMEOUT))
			LOG_ERR("Error condition: MSTATUS 0x%08x, MERRWARN 0x%08x\n",
				status, merrwarn);
	}

	/* Clear IBIWON */
	writel(I3C_MINT_IBIWON, master->regs + I3C_MSTATUS);

quit:
	k_spin_unlock(&obj->isr_lock, key);
}

/**
 * @brief Get I3C device descriptor
 */
struct i3c_dev_desc *i3c_npcm4_master_get_desc(const struct device *dev, uint8_t addr)
{
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);

	return i3c_npcm4_master_dev_from_addr(obj, addr);
}

/**
 * @brief Attach an I3C device to the controller
 */
int i3c_npcm4_master_attach_device(const struct device *dev, struct i3c_dev_desc *slave)
{
	struct i3c_npcm4_obj *master = DEV_DATA(dev);
	struct i3c_dev_data *priv;
	struct i3c_dev_desc *desc, *old;
	int slot;

	if (!slave->info.assigned_dynamic_addr)
		return -EINVAL;

	/* Check for duplicated device attachment */
	for (int i = 0; i < I3C_MAX_DEVS; i++) {
		old = master->i3c_devs[i];
		if (!old)
			continue;
		if (old->info.pid == slave->info.pid) {
			LOG_DBG("PID 0x%llx already attached at slot %d, replacing",
				old->info.pid, i);
			i3c_npcm4_master_detach_device(dev, old);
			break;
		}
	}

	slot = i3c_npcm4_master_reserve_slot(master);
	if (slot < 0)
		return slot;

	/* Allocate descriptor */
	desc = (struct i3c_dev_desc *)k_calloc(sizeof(struct i3c_dev_desc), 1);
	if (!desc)
		return -ENOMEM;

	priv = (struct i3c_dev_data *)k_calloc(sizeof(struct i3c_dev_data), 1);
	if (!priv) {
		k_free(desc);
		i3c_npcm4_master_release_slot(master, slot);
		return -ENOMEM;
	}
	memcpy(desc, slave, sizeof(struct i3c_dev_desc));

	priv->index = slot;
	desc->priv_data = priv;
	desc->bus = dev;
	desc->info.dynamic_addr = slave->info.assigned_dynamic_addr;
	master->addrs[slot] = slave->info.assigned_dynamic_addr;
	master->i3c_devs[slot] = desc;
	i3c_addr_slot_set(master, desc->info.dynamic_addr, I3C_ADDR_SLOT_I3C_DEV);

	return 0;
}

/**
 * @brief Detach an I3C device from the controller
 */
int i3c_npcm4_master_detach_device(const struct device *dev, struct i3c_dev_desc *slave)
{
	struct i3c_npcm4_obj *master = DEV_DATA(dev);
	struct i3c_dev_data *data;
	struct i3c_dev_desc *desc;

	/* Find the matched descriptor */
	desc = i3c_npcm4_master_dev_from_pid(master, slave->info.pid);
	if (!desc) {
		LOG_WRN("Unknown device(0x%llx) to detach", slave->info.pid);
		return -EINVAL;
	}

	data = DESC_PRIV(desc);
	if (!data || data->index >= I3C_MAX_DEVS) {
		LOG_WRN("Invalid descriptor\n");
		return -EINVAL;
	}

	if (master->addrs[data->index])
		i3c_addr_slot_set(master, master->addrs[data->index], I3C_ADDR_SLOT_FREE);
	master->addrs[data->index] = 0;
	master->i3c_devs[data->index] = NULL;
	i3c_npcm4_master_release_slot(master, data->index);
	if (desc->priv_data)
		k_free(desc->priv_data);
	k_free(desc);

	return 0;
}

/**
 * @brief Send CCC (Common Command Code) to an I3C device
 */
int i3c_npcm4_master_send_ccc(const struct device *dev, struct i3c_ccc_cmd *cmd)
{
	bool broadcast = cmd->id < 0x80;
	int ret;

	if (broadcast)
		ret = i3c_npcm4_master_send_bdcast_ccc_cmd(dev, cmd);
	else
		ret = i3c_npcm4_master_send_direct_ccc_cmd(dev, cmd);

	return ret;
}

/**
 * @brief Perform private I3C transfer
 */
int i3c_npcm4_master_priv_xfer(struct i3c_dev_desc *i3cdev, struct i3c_priv_xfer *xfers,
			       int nxfers)
{
	struct i3c_cmd cmd;
	int ret = 0, i;

	for (i = 0; i < nxfers; i++) {
		cmd.type = I3C_MCTRL_TYPE_I3C;
		cmd.addr = i3cdev->info.assigned_dynamic_addr;
		cmd.rnw = xfers[i].rnw;
		cmd.len = xfers[i].len;
		if (cmd.rnw)
			cmd.buf = xfers[i].data.in;
		else
			cmd.buf = xfers[i].data.out;
		cmd.continued = (i + 1) < nxfers;
		ret = i3c_npcm4_master_transfer_one(i3cdev->bus, &cmd);
		if (ret) {
			LOG_ERR("priv transfer error: %d", ret);
			break;
		}
		if (cmd.rnw)
			xfers[i].len = cmd.read_len;
	}

	return ret;
}

/**
 * @brief Request IBI (In-Band Interrupt) from slave device
 */
int i3c_npcm4_master_request_ibi(struct i3c_dev_desc *i3cdev, struct i3c_ibi_callbacks *cb)
{
	struct i3c_dev_data *priv = DESC_PRIV(i3cdev);

	priv->ibi.callbacks = cb;
	priv->ibi.context = i3cdev;
	priv->ibi.incomplete = NULL;

	return 0;
}

/**
 * @brief Enable IBI for specific device
 */
int i3c_npcm4_master_enable_ibi(struct i3c_dev_desc *i3cdev)
{
	const struct i3c_npcm4_config *master = DEV_CFG(i3cdev->bus);
	int ret;

	ret = i3c_master_send_enec(i3cdev->bus, i3cdev->info.dynamic_addr, I3C_CCC_EVT_SIR);
	writel(I3C_MINT_SLVSTART, master->regs + I3C_MINTSET);

	return 0;
}

/**
 * @brief Run ENTDAA process (return a target address list)
 */
int i3c_npcm4_master_do_daa(const struct device *dev, uint8_t *addrs, int *count)
{
	const struct i3c_npcm4_config *master = DEV_CFG(dev);
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	uint64_t prov_id[I3C_MAX_DEVS] = {}, nacking_prov_id = 0;
	uint32_t last_addr = 0, dyn_addr = 0;
	int dev_nb = 0;
	uint32_t reg;
	int ret, i;

	k_mutex_lock(&obj->xfer_lock, K_FOREVER);
	i3c_npcm4_master_flush_fifo(master);

	while (true) {
		/* clean I3C_MINT_IBIWON w1c bits */
		writel(I3C_MINT_IBIWON, master->regs + I3C_MSTATUS);

		/* I3C_MCTRL_REQUEST_PROC_DAA have two mode, ENTER DAA or PROCESS DAA.
		 *
		 * ENTER DAA:
		 *   1 will issue START, 7E, ENTDAA, and then emits 7E/R to process first target.
		 *   2 Stops just before the new Dynamic Address (DA) is to be emitted.
		 *
		 * PROCESS DAA:
		 *   1 The DA is written using MWDATAB or ADDR bits 6:0.
		 *   2 ProcessDAA is requested again to write the new address, and then starts the
		 *     next (START, 7E, ENTDAA)  unless marked to STOP; an MSTATUS indicating NACK
		 *     means DA was not accepted (e.g. parity error). If PROCESSDAA is NACKed on the
		 *     7E/R, which means no more Slaves need a DA, then a COMPLETE will be signaled
		 *     (along with DONE), and a STOP issued automatically.
		 */
		writel(I3C_MCTRL_REQUEST_PROC_DAA |
		       I3C_MCTRL_TYPE_I3C |
		       I3C_MCTRL_IBIRESP_NACK |
		       I3C_MCTRL_DIR(I3C_MCTRL_DIR_WRITE),
		       master->regs + I3C_MCTRL);

		/*
		 * Either one slave will send its ID, or the assignment process
		 * is done.
		 */
		ret = readl_poll_timeout_atomic(master->regs + I3C_MSTATUS,
						reg,
						I3C_MSTATUS_RXPEND(reg) |
						I3C_MSTATUS_MCTRLDONE(reg),
						0, 1000);
		if (ret)
			break;

		if (I3C_MSTATUS_RXPEND(reg)) {
			uint8_t data[6];

			ret = i3c_master_get_free_addr(obj, last_addr + 1);
			if (ret < 0)
				break;

			dyn_addr = ret;
			writel(dyn_addr, master->regs + I3C_MWDATAB);

			/*
			 * We only care about the 48-bit provisioned ID yet to
			 * be sure a device does not nack an address twice.
			 * Otherwise, we would just need to flush the RX FIFO.
			 */
			ret = i3c_npcm4_master_readb(master, data, 6);
			if (ret)
				break;

			for (i = 0; i < 6; i++)
				prov_id[dev_nb] |= (uint64_t)(data[i]) << (8 * (5 - i));

			/* We do not care about the BCR and DCR yet */
			ret = i3c_npcm4_master_readb(master, data, 2);
			if (ret)
				break;
		} else if (I3C_MSTATUS_IBIWON(reg)) {
			ret = i3c_npcm4_master_handle_ibi_won(master, reg);
			if (ret)
				break;
			continue;
		} else if (I3C_MSTATUS_MCTRLDONE(reg)) {
			if (I3C_MSTATUS_STATE_IDLE(reg) &&
			    I3C_MSTATUS_COMPLETE(reg)) {
				/*
				 * All devices received and acked they dynamic
				 * address, this is the natural end of the DAA
				 * procedure.
				 *
				 * Hardware will auto emit STOP at this case.
				 */
				*count = dev_nb;
				k_mutex_unlock(&obj->xfer_lock);
				return 0;

			} else if (I3C_MSTATUS_NACKED(reg)) {
				/* No I3C devices attached */
				if (dev_nb == 0) {
					/*
					 * Hardware can't treat first NACK for ENTAA as normal
					 * COMPLETE. So need manual emit STOP.
					 */
					ret = 0;
					*count = 0;
					break;
				}

				/*
				 * A slave device nacked the address, this is
				 * allowed only once, DAA will be stopped and
				 * then resumed. The same device is supposed to
				 * answer again immediately and shall ack the
				 * address this time.
				 */
				if (prov_id[dev_nb] == nacking_prov_id) {
					ret = -EIO;
					break;
				}

				dev_nb--;
				nacking_prov_id = prov_id[dev_nb];
				i3c_npcm4_master_emit_stop(master);

				continue;
			} else {
				break;
			}
		}

		/* Wait for the slave to be ready to receive its address */
		ret = readl_poll_timeout_atomic(master->regs + I3C_MSTATUS,
						reg,
						I3C_MSTATUS_MCTRLDONE(reg) &&
						I3C_MSTATUS_STATE_DAA(reg) &&
						I3C_MSTATUS_BETWEEN(reg),
						0, 1000);
		if (ret)
			break;

		addrs[dev_nb] = dyn_addr;
		LOG_INF("DAA: device %d assigned to 0x%02x", dev_nb, addrs[dev_nb]);
		last_addr = addrs[dev_nb++];

		if (dev_nb == I3C_MAX_DEVS) {
			LOG_ERR("DAA: Reach max devs\n");
			ret = -EOVERFLOW;
			break;
		}
	}

	/* Need manual issue STOP except for Complete condition */
	i3c_npcm4_master_emit_stop(master);
	i3c_npcm4_master_flush_fifo(master);
	k_mutex_unlock(&obj->xfer_lock);

	return ret;
}

/**
 * @brief Send ENTDAA (return the first device descriptor)
 */
int i3c_npcm4_master_send_entdaa(struct i3c_dev_desc *i3cdev)
{
	int dev_nb = 0;
	uint8_t addrs[I3C_MAX_DEVS];
	int ret;

	ret = i3c_npcm4_master_do_daa(i3cdev->bus, addrs, &dev_nb);
	if (dev_nb > 0)
		i3cdev->info.assigned_dynamic_addr = addrs[0];

	return ret;
}
/* End of Master mode API implementations */

static void i3c_npcm4_isr(const struct device *dev)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);

	if (config->slave)
		return i3c_npcm4_slave_isr(dev);

	i3c_npcm4_master_isr(dev);
}


static int i3c_npcm4_slave_init(const struct device *dev,
				const struct i3c_npcm4_config *config)
{
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	uint32_t val;

	LOG_DBG("bcr=0x%x, dcr=0x%x", config->bcr, config->dcr);

	obj->rx_desc = NULL;
	obj->tx_desc = NULL;
	obj->state = 0;

	/* Setup slave BCR/DCR/PID */
	writel((config->bcr << 16) | (config->dcr << 8), config->regs + I3C_IDEXT);
	writel((config->part_id << 16) | (config->vendor_def_id), config->regs + I3C_PARTNO);
	/* Setup max rd/wr length */
	writel((DMA_BUF_SIZE << 16) | DMA_BUF_SIZE, config->regs + I3C_MAXLIMITS);

	/* Setup static address and enable slave mode */
	val = FIELD_PREP(GENMASK(31, 25), config->assigned_addr);
	val |= FIELD_PREP(GENMASK(22, 16), (obj->apb3_rate / 1000000UL));
	val |= I3C_CONFIG_MATCHSS | I3C_CONFIG_SLVENA;
	writel(val, config->regs + I3C_CONFIG);

	return 0;
}

static int i3c_npcm4_master_init(const struct device *dev,
				 const struct i3c_npcm4_config *config)
{
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	uint32_t fclk_rate, fclk_period_ns;
	uint32_t pp_high_period_ns, od_low_period_ns, i2c_period_ns;
	uint32_t scl_period_ns;
	uint32_t ppbaud, pplow, odhpp, odbaud, i2cbaud, val;
	uint32_t i3c_scl_rate, i2c_scl_rate;

	obj->free_slots = GENMASK(I3C_MAX_DEVS - 1, 0);
	k_sem_init(&obj->complete, 0, 1);

	/* Initialize address slot bitmap */
	memset(obj->addr_slots, 0, sizeof(obj->addr_slots));

	/* Reserve addresses 0x00..0x08 (dynamic addresses start at 0x09) */
	for (int _i = 0; _i < 0x09; _i++)
		i3c_addr_slot_set(obj, _i, I3C_ADDR_SLOT_RSVD);

	/*
	 * Reserve broadcast address and all addresses that might collide
	 * with the broadcast address when facing a single bit error.
	 */
	i3c_addr_slot_set(obj, I3C_BROADCAST_ADDR, I3C_ADDR_SLOT_RSVD);
	for (int _i = 0; _i < 7; _i++)
		i3c_addr_slot_set(obj, I3C_BROADCAST_ADDR ^ BIT(_i), I3C_ADDR_SLOT_RSVD);
	fclk_rate = obj->apb3_rate;
	fclk_period_ns = 1000000000 / fclk_rate;

	/*
	 * Configure for Push-Pull mode.
	 */
	scl_period_ns = DIV_ROUND_UP(1000000000, config->i3c_scl_hz);
	/* 50% duty-cycle */
	ppbaud = DIV_ROUND_UP((scl_period_ns / 2), fclk_period_ns) - 1;
	pplow = 0;
	if (ppbaud > NPCM4_I3C_MAX_PPBAUD)
		ppbaud = NPCM4_I3C_MAX_PPBAUD;
	pp_high_period_ns = (ppbaud + 1) * fclk_period_ns;

	/*
	 * Configure for Open-Drain mode.
	 */
	/* Set default OD timing: 1MHz/1000ns with 50% duty cycle */
	odhpp = 0;
	odbaud = DIV_ROUND_UP(500, pp_high_period_ns) - 1;
	if (odbaud > NPCM4_I3C_MAX_ODBAUD)
		odbaud = NPCM4_I3C_MAX_ODBAUD;
	od_low_period_ns = (odbaud + 1) * pp_high_period_ns;

	/* Configure for I2C mode */
	i2c_period_ns = DIV_ROUND_UP(1000000000, config->i2c_scl_hz);
	if (i2c_period_ns < od_low_period_ns * 2)
		i2c_period_ns = od_low_period_ns * 2;
	i2cbaud = DIV_ROUND_UP(i2c_period_ns, od_low_period_ns) - 2;
	if (i2cbaud > NPCM4_I3C_MAX_I2CBAUD)
		i2cbaud = NPCM4_I3C_MAX_I2CBAUD;

	i3c_scl_rate = 1000000000 / (((ppbaud + 1) * 2 + pplow) * fclk_period_ns);
	i2c_scl_rate = 1000000000 / ((i2cbaud + 2) * od_low_period_ns);

	/* Clear the interrupt status */
	writel(I3C_MINT_SLVSTART, config->regs + I3C_MSTATUS);

	if (config->enable_hj) {
		LOG_INF("enabling hot-join interrupt");
		writel(I3C_MINT_SLVSTART, config->regs + I3C_MINTSET);
	}

	val = NPCM4_I3C_MCONFIG_MASTER_EN |
	      NPCM4_I3C_MCONFIG_DISTO(0) |
	      NPCM4_I3C_MCONFIG_HKEEP(3) |
	      NPCM4_I3C_MCONFIG_ODSTOP(0) |
	      NPCM4_I3C_MCONFIG_PPBAUD(ppbaud) |
	      NPCM4_I3C_MCONFIG_PPLOW(pplow) |
	      NPCM4_I3C_MCONFIG_ODBAUD(odbaud) |
	      NPCM4_I3C_MCONFIG_ODHPP(odhpp) |
	      NPCM4_I3C_MCONFIG_SKEW(0) |
	      NPCM4_I3C_MCONFIG_I2CBAUD(i2cbaud);
	writel(val, config->regs + I3C_MCONFIG);

	LOG_INF("fclk=%u, period_ns=%u", fclk_rate, fclk_period_ns);
	LOG_INF("i3c scl_rate=%u", i3c_scl_rate);
	LOG_INF("i2c scl_rate=%u", i2c_scl_rate);
	LOG_INF("pp_high=%u, pp_low=%u", pp_high_period_ns,
		(ppbaud + 1 + pplow) * fclk_period_ns);
	LOG_INF("od_high=%d, od_low=%d", odhpp ? pp_high_period_ns : od_low_period_ns,
		od_low_period_ns);

	return 0;
}

static int i3c_npcm4_setup_dma(const struct device *dev)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	uint32_t *PDMA_REQSEL;
	uint32_t val, shift;

	if (config->dma_tx_channel >= PDMA_CH_MAX || config->dma_rx_channel >= PDMA_CH_MAX) {
		LOG_ERR("Invalid DMA channel");
		return -EINVAL;
	}

	LOG_DBG("setup dma: tx chanel %d, rx channel %d",
		config->dma_tx_channel, config->dma_rx_channel);

	/* Select I3C module for DMA TX channel */
	PDMA_REQSEL = (uint32_t *)&config->pdma_base->REQSEL0_3 + (config->dma_tx_channel / 4);
	shift = (config->dma_tx_channel % 4) * 8;
	val = *PDMA_REQSEL & ~(0xFF << shift);
	val |= (6 + config->inst_id * 2) << shift;
	*PDMA_REQSEL = val;

	/* Select I3C module for DMA RX channel */
	PDMA_REQSEL = (uint32_t *)&config->pdma_base->REQSEL0_3 + (config->dma_rx_channel / 4);
	shift = (config->dma_rx_channel % 4) * 8;
	val = *PDMA_REQSEL & ~(0xFF << shift);
	val |= (5 + config->inst_id * 2) << shift;
	*PDMA_REQSEL = val;

	/* Setup dsct base address */
	config->pdma_base->SCATBA = (uint32_t)sg_dsct;
	obj->dma_buf = &dma_buf_pool[DMA_BUF_SIZE * config->inst_id];

	return 0;
}

/* Device initialization */
static int i3c_npcm4_init(const struct device *dev)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	const struct device *const clk_dev = device_get_binding(NPCM4XX_CLK_CTRL_NAME);
	uint32_t apb3_rate;
	uint8_t swrst;
	int ret;

	LOG_WRN("Built at:   %s %s", __DATE__, __TIME__);

	/* Turn on device clock first and get source clock freq. */
	ret = clock_control_on(clk_dev, (clock_control_subsys_t *)
			&config->clk_cfg);
	if (ret < 0) {
		LOG_ERR("Turn on I3C clock fail %d", ret);
		return ret;
	}
	ret = clock_control_get_rate(clk_dev, (clock_control_subsys_t *)
			&config->clk_cfg, &apb3_rate);
	if (ret < 0) {
		LOG_ERR("Get I3C clock rate error %d", ret);
		return ret;
	}

	/* I3C software reset */
	swrst = sys_read8(config->pmc_base + PMC_SW_RST1) | BIT(config->inst_id);
	sys_write8(swrst, config->pmc_base + PMC_SW_RST1);
	swrst &= ~BIT(config->inst_id);
	sys_write8(swrst, config->pmc_base + PMC_SW_RST1);

	obj->apb3_rate = apb3_rate;
	obj->dev = dev;
	k_mutex_init(&obj->xfer_lock);
	k_work_init(&obj->hj_work, i3c_npcm4_hj_work_handler);

	ret = i3c_npcm4_setup_dma(dev);
	if (ret < 0) {
		LOG_ERR("i3c_npcm4_setup_dma fail %d", ret);
		return ret;
	}

	if (config->slave) {
		ret = i3c_npcm4_slave_init(dev, config);
		if (ret < 0) {
			LOG_ERR("i3c_npcm4_slave_init fail %d", ret);
			return ret;
		}
	} else {
		ret = i3c_npcm4_master_init(dev, config);
		if (ret < 0) {
			LOG_ERR("i3c_npcm4_master_init fail %d", ret);
			return ret;
		}
	}

	LOG_INF("NPCM4 I3C %s initialized successfully",
		config->slave ? "Slave" : "Master");
	return 0;
}

/* Device instantiation macro */
#define I3C_NPCM4_INIT(n) static int i3c_npcm4_config_func_##n(const struct device *dev);\
	static const struct i3c_npcm4_config i3c_npcm4_config_##n = {\
		.inst_id = DT_INST_PROP_OR(n, instance_id, 0),\
		.assigned_addr = DT_INST_PROP_OR(n, assigned_address, 0),\
		.slave = DT_INST_PROP_OR(n, slave, 0),\
		.secondary = DT_INST_PROP_OR(n, secondary, 0),\
		.bcr = DT_INST_PROP_OR(n, bcr, 0),\
		.dcr = DT_INST_PROP_OR(n, dcr, 0),\
		.part_id = DT_INST_PROP_OR(n, part_id, 0),\
		.vendor_def_id = DT_INST_PROP_OR(n, vendor_def_id, 0),\
		.dma_tx_channel = DT_INST_PROP_OR(n, dma_tx_channel, 0xff),\
		.dma_rx_channel = DT_INST_PROP_OR(n, dma_rx_channel, 0xff),\
		.i2c_scl_hz = DT_INST_PROP_OR(n, i2c_scl_hz, 0),\
		.i3c_scl_hz = DT_INST_PROP_OR(n, i3c_scl_hz, 0),\
		.hj_timeout_ms = DT_INST_PROP_OR(n, hj_request_timeout_ms, -1),\
		.enable_hj = DT_INST_PROP(n, enable_hj),\
		.regs = DT_INST_REG_ADDR_BY_NAME(n, i3c),\
		.pdma_base = (struct pdma_reg *)DT_INST_REG_ADDR_BY_NAME(n, pdma),\
		.pmc_base = DT_INST_REG_ADDR_BY_NAME(n, pmc),\
		.clk_cfg = NPCM4XX_DT_CLK_CFG_ITEM(n),\
		.irq = DT_INST_IRQN(n),\
	};\
	static struct i3c_npcm4_obj i3c_npcm4_obj##n;\
	DEVICE_DT_INST_DEFINE(n, &i3c_npcm4_config_func_##n, NULL, &i3c_npcm4_obj##n,\
			      &i3c_npcm4_config_##n, POST_KERNEL,\
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);\
	static int i3c_npcm4_config_func_##n(const struct device *dev)\
	{\
		int ret;\
		ret = i3c_npcm4_init(dev);\
		if (ret < 0) \
			return ret;\
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), i3c_npcm4_isr,\
			DEVICE_DT_INST_GET(n), 0);\
		irq_enable(DT_INST_IRQN(n));\
		return 0;\
	}

DT_INST_FOREACH_STATUS_OKAY(I3C_NPCM4_INIT)
