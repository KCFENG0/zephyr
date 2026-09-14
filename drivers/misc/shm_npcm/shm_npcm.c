/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcm_shm

#define LOG_MODULE_NAME shm_npcm

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/misc/shm_npcm/shm_npcm.h>
#include <soc.h>

#define SNOOP_FIFO_DEPTH	16

static uint8_t snoop_ptr[SNOOP_FIFO_DEPTH];

struct shm_npcm_config{
	struct shm_reg *const base;
};

struct shm_npcm_data{
        snoop_npcm_rx_callback_t *rx_cb;
};

struct shm_npcm_data shm_data;

static const struct shm_npcm_config shm_npcm_cfg = {
	.base = (struct shm_reg *)DT_INST_REG_ADDR(0),
};

int snoop_npcm_register_rx_callback(const struct device *dev, snoop_npcm_rx_callback_t cb)
{
	struct shm_npcm_data *data = (struct shm_npcm_data *)dev->data;

	if (data->rx_cb) {
		LOG_ERR("Snoop RX callback is registered\n");
		return -EBUSY;
	}

	data->rx_cb = cb;

	return 0;
}

void shm_npcm_set_win_size(uint32_t win, uint8_t size)
{
	struct shm_reg *const shm_regs = shm_npcm_cfg.base;

	if(size > SHM_WIN_SIZE_4KB)
		size = SHM_WIN_SIZE_4KB;

	if(size < SHM_WIN_SIZE_8B)
		size = SHM_WIN_SIZE_8B;

	switch(win){
		case SHM_WIN1:
		case SHM_WIN2:
			shm_regs->WIN_SIZE &= 0xF0 >> 4 * (win - SHM_WIN1);
			shm_regs->WIN_SIZE |= (size << 4 * (win - SHM_WIN1));
			break;
		case SHM_WIN3:
		case SHM_WIN4:
			shm_regs->WIN_SIZE2 &= 0xF0 >> 4 * (win - SHM_WIN3);
			shm_regs->WIN_SIZE2 |= (size << 4 * (win - SHM_WIN3));
			break;
		case SHM_WIN5:
			shm_regs->WIN_SIZE3 = (size & 0x0F);
			break;
		case SHM_IMA_WIN1:
		case SHM_IMA_WIN2:
			size <<= 4 * (win - SHM_IMA_WIN1);
			shm_regs->IMA_WIN_SIZE ^= (size ^ shm_regs->IMA_WIN_SIZE) & (0x0F << 4 * (win - SHM_IMA_WIN1));
			break;
	}
}

void shm_npcm_set_memory(uint8_t win, SHM_Mod_T *shm_mod)
{
	struct shm_reg *const shm_regs = shm_npcm_cfg.base;
	uint32_t base = (uint32_t)shm_mod->base;

	switch(win){
		case SHM_WIN1:
			shm_regs->WIN_BASE1 = base;
			shm_regs->WIN1_WR_PROT = shm_mod->wr_protect;
			shm_regs->WIN1_RD_PROT = shm_mod->rd_protect;
			shm_npcm_set_win_size(SHM_WIN1, shm_mod->size);
			shm_regs->COFS1 = shm_mod->offset;
			break;
		case SHM_WIN2:
			shm_regs->WIN_BASE2 = base;
			shm_regs->WIN2_WR_PROT = shm_mod->wr_protect;
			shm_regs->WIN2_RD_PROT = shm_mod->rd_protect;
			shm_npcm_set_win_size(SHM_WIN2, shm_mod->size);
			shm_regs->COFS2 = shm_mod->offset;
			break;
		case SHM_WIN3:
			shm_regs->WIN_BASE3 = base;
			shm_regs->WIN3_WR_PROT = shm_mod->wr_protect;
			shm_regs->WIN3_RD_PROT = shm_mod->rd_protect;
			shm_npcm_set_win_size(SHM_WIN3, shm_mod->size);
			shm_regs->COFS3 = shm_mod->offset;
			break;
		case SHM_WIN4:
			shm_regs->WIN_BASE4 = base;
			shm_regs->WIN4_WR_PROT = shm_mod->wr_protect;
			shm_regs->WIN4_RD_PROT = shm_mod->rd_protect;
			shm_npcm_set_win_size(SHM_WIN4, shm_mod->size);
			shm_regs->COFS4 = shm_mod->offset;
			break;
		case SHM_WIN5:
			shm_regs->WIN_BASE5 = base;
			shm_regs->WIN5_WR_PROT = shm_mod->wr_protect;
			shm_regs->WIN5_RD_PROT = shm_mod->rd_protect;
			shm_npcm_set_win_size(SHM_WIN5, shm_mod->size);
			shm_regs->COFS5 = shm_mod->offset;
			break;
		case SHM_IMA_WIN1:
			shm_regs->IMA_BASE = base;
			shm_regs->IMA_WR_PROT = shm_mod->wr_protect;
			shm_regs->IMA_RD_PROT = shm_mod->rd_protect;
			shm_npcm_set_win_size(SHM_IMA_WIN1, shm_mod->size);
			shm_regs->SHCFG |= BIT(0); // Enable IMA
			break;
		case SHM_IMA_WIN2:
			shm_regs->IMA2_BASE = base;
			shm_regs->IMA2_WR_PROT = shm_mod->wr_protect;
			shm_regs->IMA2_RD_PROT = shm_mod->rd_protect;
			shm_npcm_set_win_size(SHM_IMA_WIN2, shm_mod->size);
			shm_regs->SHCFG |= BIT(9); // Enable IMA2
			break;
		case SHM_AIMA_WIN:
			// Additional IMA  only	support 256 bytes window size
			shm_regs->AIMA_BASE = base;
			shm_regs->SHCFG |= BIT(8); // Enable AIMA
			break;
	}
}

void shm_npcm_offset_intr(uint8_t win, uint8_t flags, uint8_t enable)
{
	struct shm_reg *const shm_regs = shm_npcm_cfg.base;

	if(win != SHM_WIN5){
		// window 1~4
		flags <<= 2 * (win - SHM_WIN1);
		if(enable)
			shm_regs->HOFS_CTL |= flags;
		else
			shm_regs->HOFS_CTL &= ~flags;
	}else{
		// window 5
		if(enable)
			shm_regs->HOFS_CTL2 |= flags;
		else
			shm_regs->HOFS_CTL2 &= ~flags;
	}
}

void shm_npcm_enable_sem(uint8_t win)
{
	struct shm_reg *const shm_regs = shm_npcm_cfg.base;

	switch(win){
		case SHM_WIN1:
		case SHM_WIN2:
			shm_regs->SHCFG &= ~((0x01) << (7 - (win - SHM_WIN1)));
			break;
		case SHM_WIN3:
		case SHM_WIN4:
			shm_regs->SHCFG &= ~((0x01) << (4 - (win - SHM_WIN3)));
			break;
		case SHM_WIN5:
			shm_regs->SHCFG &= ~BIT(10); // W5 SEM_EN
			break;
		case SHM_IMA_WIN1:
			shm_regs->SHCFG &= ~BIT(5); // IMA1 SEM_EN
			break;
		case SHM_IMA_WIN2:
			shm_regs->SHCFG &= ~BIT(2); // IMA1 SEM_EN
			break;
		case SHM_AIMA_WIN:
			shm_regs->SHCFG &= ~BIT(1); // AIMA SEM_EN
			break;
	}

}


void shm_npcm_disable_sem_intr(uint8_t win)
{
	struct shm_reg *const shm_regs = shm_npcm_cfg.base;

	switch(win){
		case SHM_WIN1:
		case SHM_WIN2:
			shm_regs->SMC_CTL &= ~(((0x01) << 3) << win);
			break;
		case SHM_WIN3:
		case SHM_WIN4:
			shm_regs->SMC_CTL2 &= ~((0x01) << (win - SHM_WIN3));
			break;
		case SHM_WIN5:
			shm_regs->SMC_CTL2 &= ~BIT(4); // W5 SEM_EN_INTR
			break;
		case SHM_IMA_WIN1:
			shm_regs->SMC_CTL &= ~BIT(6); // IMA1 SEM_EN_INTR
			break;
		case SHM_IMA_WIN2:
			shm_regs->SMC_CTL2 &= ~BIT(2); // IMA2 SEM_EN_INTR
			break;
		case SHM_AIMA_WIN:
			shm_regs->SMC_CTL2 &= ~BIT(3); // AIMA SEM_EN_INTR
			break;
	}
}

void shm_npcm_enable_sem_intr(uint8_t win)
{
	struct shm_reg *const shm_regs = shm_npcm_cfg.base;

	switch(win){
		case SHM_WIN1:
		case SHM_WIN2:
			shm_regs->SMC_CTL |= (((0x01) << 3) << win);
			break;
		case SHM_WIN3:
		case SHM_WIN4:
			shm_regs->SMC_CTL2 |= ((0x01) << (win - SHM_WIN3));
			break;
		case SHM_WIN5:
			shm_regs->SMC_CTL2 |= BIT(4); // W5 SEM_EN_INTR
			break;
		case SHM_IMA_WIN1:
			shm_regs->SMC_CTL |= BIT(6); // IMA1 SEM_EN_INTR
			break;
		case SHM_IMA_WIN2:
			shm_regs->SMC_CTL2 |= BIT(2); // IMA2 SEM_EN_INTR
			break;
		case SHM_AIMA_WIN:
			shm_regs->SMC_CTL2 |= BIT(3); // AIMA SEM_EN_INTR
			break;
	}
}

int port80_npcm_read(uint8_t *port, uint8_t *bytelen, uint8_t *data)
{
	struct shm_reg *const shm_regs = shm_npcm_cfg.base;
	uint16_t buf = 0;
	uint32_t buf1 = 0;
	uint16_t offset = 0;

	if(!(shm_regs->DP80STS & BIT(NPCM_DP80STS_FNE)))
		return -1;

	*port = 0;
	buf = shm_regs->DP80BUF;
	buf1 = shm_regs->DP80BUF1;

	if((buf & BIT(NPCM_DP80BUF_RNG)) == 0){
		/* Range 1 */
		*port = 0x10 | ((buf & 0x00FF) >> 8);
		printf("Range 1 port: %02X\n", *port);
	}else{
		/* Range 2 */
		*port = 0x20 | ((buf & 0x00FF) >> 8);
		printf("Range 2 port: %02X\n", *port);
	}

	/* get offset */
	offset = GET_FIELD(buf, NPCM_DP80BUF_OFFS);
	printf(" offset %02X\n", offset);

	switch (GET_FIELD(buf, NPCM_DP80BUF_BTYE_LEN)){
		case 0: /* 1 byte */
			*bytelen = 1;
			data[0] = buf & 0xFF;
			printf("data %X \n", data[0]);
			break;
		case 1: /* 2 bytes */
			*bytelen = 2;
			data[0] = buf & 0xFF;
			data[1] = buf1 >> 8 & 0xFF;
			printf("data %X %X \n", data[0], data[1]);
			break;
		case 2: /* 4 bytes */
			*bytelen = 4;
			data[0] = buf & 0xFF;
			data[1] = (buf1 >> 8) & 0xFF;
			data[2] = (buf1 >> 16) & 0xFF;
			data[3] = (buf1 >> 24) & 0xFF;
			printf("data %X %X %X %X \n", data[0], data[1], data[2], data[3]);
			break;
	}
	return 0;
}

static void port80_npcm_isr(const void *arg)
{
	ARG_UNUSED(arg);
	struct shm_reg *const shm_regs = shm_npcm_cfg.base;
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(shm));
	struct shm_npcm_data *data = (struct shm_npcm_data *)dev->data;

	/* Read out port80 data continuously if FIFO is not empty */
	while (NPCM_IS_BIT_SET(shm_regs->DP80STS, NPCM_DP80STS_FNE)) {
		LOG_INF("p80: %04x", shm_regs->DP80BUF);
	}

	/* If FIFO is overflow, show error msg */
	if (NPCM_IS_BIT_SET(shm_regs->DP80STS, NPCM_DP80STS_FOR)) {
		shm_regs->DP80STS |= BIT(NPCM_DP80STS_FOR);
		LOG_DBG("Port80 FIFO Overflow!");
	}

	/* If there are pending post codes remains in FIFO after processing and sending previous
	 * post codes, do not clear the FNE bit. This allows this handler to be called again
	 * immediately after it exists.
	 */
	if (!NPCM_IS_BIT_SET(shm_regs->DP80STS, NPCM_DP80STS_FNE)) {
		/* Clear all pending bit indicates that FIFO was written by host */
		shm_regs->DP80STS |= BIT(NPCM_DP80STS_FWR);
	}

	uint8_t len;
	uint16_t buf = 0;
	uint32_t buf1 = 0;

	buf = shm_regs->DP80BUF;
	buf1 = shm_regs->DP80BUF1;

	switch (GET_FIELD(shm_regs->DP80BUF, NPCM_DP80BUF_BTYE_LEN)){
		case 0: /* 1 byte */
			len = 1;
			snoop_ptr[0] = buf & 0xFF;
			break;
		case 1: /* 2 bytes */
			len = 2;
			snoop_ptr[0] = buf & 0xFF;
			snoop_ptr[1] = buf1 >> 8 & 0xFF;
			break;
		case 2: /* 4 bytes */
			len = 4;
			snoop_ptr[0] = buf & 0xFF;
			snoop_ptr[1] = (buf1 >> 8) & 0xFF;
			snoop_ptr[2] = (buf1 >> 16) & 0xFF;
			snoop_ptr[3] = (buf1 >> 24) & 0xFF;
			break;
	}

	if (data->rx_cb)
		data->rx_cb(snoop_ptr, len);
}

static int port80_npcm_init(const struct device *dev)
{
	struct shm_reg *const shm_regs = shm_npcm_cfg.base;

	/* Reset the  FIFO */
	shm_regs->DP80CTL |= BIT(NPCM_DP80CTL_RFIFO);
	/* Enable the Port80 */
	shm_regs->DP80CTL = BIT(NPCM_DP80CTL_DP80EN) | BIT(NPCM_DP80CTL_SYNCEN) |
			BIT(NPCM_DP80CTL_RAA);

	/* enable Port80 interrupt */
	shm_regs->DP80IE |= BIT(NPCM_DP80IE_FWRIE) | BIT(NPCM_DP80IE_FNEIE) | BIT(NPCM_DP80IE_FORIE);

	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, p80, irq),
		    DT_INST_IRQ_BY_NAME(0, p80, priority),
		    port80_npcm_isr,
		    NULL, 0);
	irq_enable(DT_INST_IRQ_BY_NAME(0, p80, irq));
	return 0;
}

static int shm_npcm_reset(const struct device *dev)
{
	struct shm_reg *const shm_regs = shm_npcm_cfg.base;
	uint32_t win_dis = 0xFFFFFFF8;

	shm_regs->RST_CFG &= ~(BIT(7)); //set SMC_RST

	/* Reset all windows */
	shm_regs->SHM_CTL &= ~BIT(6); //STALL_HOST: host access termainates normally

	/* Set all windows base to 0 */
#if !defined(CONFIG_ESPI_PERIPHERAL_EC_HOST_CMD)
	shm_regs->WIN_BASE1 = win_dis;
#endif
	shm_regs->WIN_BASE2 = win_dis;
	shm_regs->WIN_BASE3 = win_dis;
	shm_regs->WIN_BASE4 = win_dis;
	shm_regs->WIN_BASE5 = win_dis;
	shm_regs->IMA_BASE  = win_dis;
	shm_regs->IMA2_BASE = win_dis;
	shm_regs->AIMA_BASE = win_dis;

	/* Set all windows size to 16 bytes */
#if !defined(CONFIG_ESPI_PERIPHERAL_EC_HOST_CMD)
	shm_npcm_set_win_size(SHM_WIN1, SHM_WIN_SIZE_16B);
#endif
	shm_npcm_set_win_size(SHM_WIN2, SHM_WIN_SIZE_16B);
	shm_npcm_set_win_size(SHM_WIN3, SHM_WIN_SIZE_16B);
	shm_npcm_set_win_size(SHM_WIN4, SHM_WIN_SIZE_16B);
	shm_npcm_set_win_size(SHM_WIN5, SHM_WIN_SIZE_16B);
	shm_npcm_set_win_size(SHM_IMA_WIN1, SHM_WIN_SIZE_16B);
	shm_npcm_set_win_size(SHM_IMA_WIN2, SHM_WIN_SIZE_16B);

	/* Clear all pending status */
	shm_regs->SMC_STS = shm_regs->SMC_STS;

	return 0;
}

static int shm_npcm_init(const struct device *dev)
{
	//int rc;
        //struct shm_reg *const shm_regs = shm_npcm_cfg.base;

	shm_npcm_reset(dev);
	/* Enable Port80 function */
	port80_npcm_init(dev);

	return 0;
}


DEVICE_DT_INST_DEFINE(0, &shm_npcm_init, NULL,
		    &shm_data, &shm_npcm_cfg,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    NULL);
