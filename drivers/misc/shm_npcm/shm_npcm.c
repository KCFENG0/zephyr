#define DT_DRV_COMPAT nuvoton_npcm_shm

#define LOG_MODULE_NAME shm_npcm

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <soc.h>

struct shm_npcm_config{
	struct shm_reg *const base;
};

struct shm_npcm_data{
        uint8_t port80_data[256];
};

struct shm_npcm_data shm_data;

static const struct shm_npcm_config shm_npcm_cfg = {
	.base = (struct shm_reg *)DT_INST_REG_ADDR(0),
};

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

	/* Read out port80 data continuously if FIFO is not empty */
	while (NPCM_IS_BIT_SET(shm_regs->DP80STS, NPCM_DP80STS_FNE)) {
		LOG_DBG("p80: %04x", shm_regs->DP80BUF);
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

	return ;
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
	//irq_enable(DT_INST_IRQ_BY_NAME(0, p80, irq));
	return 0;
}

static int shm_npcm_init(const struct device *dev)
{
	//int rc;
        //struct shm_reg *const shm_regs = shm_npcm_cfg.base;

	/* Enable Port80 function */
	port80_npcm_init(dev);

	return 0;
}


DEVICE_DT_INST_DEFINE(0, &shm_npcm_init, NULL,
		    &shm_data, &shm_npcm_cfg,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    NULL);