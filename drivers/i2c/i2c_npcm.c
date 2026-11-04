/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcm_i2c

#include <zephyr/drivers/clock_control.h>
#include <zephyr/cache.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/i2c/i2c.h>
#include <string.h>
#include <soc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2c_npcm, LOG_LEVEL_ERR);

/* Configure TX and RX buffer size for I2C DMA
 * This setting applies to all (12c1a, 12c1b, 12c2a ...)
 */
#define I2C_MAX_TX_SIZE 256
#define I2C_MAX_RX_SIZE 256

/* Default maximum time we allow for an I2C transfer (unit:ms) */
#define I2C_TRANS_TIMEOUT K_MSEC(500)

/* Default max waiting time for i2c ready (unit:ms) */
#define I2C_WAITING_TIME K_MSEC(1000)

/* Data abort timeout */
#define ABORT_TIMEOUT 10000
#define I2C_SLAVE_NUM 10
#define I2C_RECOVER_BUS_DELAY_US 5

/* Longest a transfer waits for another controller to release the bus (SMBus tTIMEOUT max) */
#define I2C_BUS_FREE_TIMEOUT_MS 35
/* SMBus tHIGH max: SCL and SDA high for longer than this means the bus is idle */
#define I2C_BUS_IDLE_TIME_US 50
#define I2C_BUS_IDLE_POLL_US 5

/* I2C operation state */
enum i2c_npcm_oper_state {
	I2C_NPCM_OPER_STA_IDLE,
	I2C_NPCM_OPER_STA_START,
	I2C_NPCM_OPER_STA_WRITE,
	I2C_NPCM_OPER_STA_READ,
	I2C_NPCM_OPER_STA_QUICK,
};

enum i2c_npcm_slave_addrno {
    I2C_SLAVE_ADDR_1,
    I2C_SLAVE_ADDR_2,
    I2C_SLAVE_ADDR_3,
    I2C_SLAVE_ADDR_4,
    I2C_SLAVE_ADDR_5,
    I2C_SLAVE_ADDR_6,
    I2C_SLAVE_ADDR_7,
    I2C_SLAVE_ADDR_8,
    I2C_SLAVE_ADDR_9,
    I2C_SLAVE_ADDR_10,
};

/* Device configuration */
struct i2c_npcm_config {
	uintptr_t base;				/* i2c controller base address */
	uint32_t clk_cfg;			/* clock configuration */
	uint32_t default_bitrate;
	uint8_t irq;				/* i2c controller irq */
	const struct pinctrl_dev_config *pcfg;  /* pinmux configuration */
};

#ifdef CONFIG_DCACHE_LINE_SIZE
#define I2C_DMA_BUF_ALIGN CONFIG_DCACHE_LINE_SIZE
#else
#define I2C_DMA_BUF_ALIGN 32
#endif

BUILD_ASSERT((I2C_MAX_RX_SIZE % I2C_DMA_BUF_ALIGN) == 0 &&
	     (I2C_MAX_TX_SIZE % I2C_DMA_BUF_ALIGN) == 0,
	     "I2C DMA buffers must be a multiple of the D-cache line size");

/* DMA buffers own whole cache lines so invalidating them never drops other fields */
struct i2c_npcm_data {
	uint8_t rx_buf[I2C_MAX_RX_SIZE] __aligned(I2C_DMA_BUF_ALIGN);
	uint8_t tx_buf[I2C_MAX_TX_SIZE] __aligned(I2C_DMA_BUF_ALIGN);
	struct k_sem lock_sem;  /* mutex of i2c controller */
	struct k_sem sync_sem;  /* semaphore used for synchronization */
	enum i2c_npcm_oper_state ctrl_oper_state;
	enum i2c_npcm_oper_state target_oper_state;
	uint32_t bitrate;
	uint32_t source_clk;
	uint16_t rx_cnt;
	uint16_t tx_cnt;
	uint8_t dev_addr; /* device address (8 bits) */
	uint8_t *rx_msg_buf;
	int err_code;
	bool lost_bus; /* the pending controller transfer lost arbitration */
	struct i2c_target_config *target_cfg[I2C_SLAVE_NUM];
	enum i2c_npcm_slave_addrno match_addr_no;
	uint8_t reg_buf[16];
};

/* Driver convenience defines */
#define I2C_INSTANCE(dev) ((struct i2c_reg *)((const struct i2c_npcm_config *)(dev)->config)->base)




/* This macro should be set only when in Controller mode or when requesting Controller mode.
 * Set START bit to CTL1 register of I2C module, but exclude STOP bit, ACK bit.
 */
static inline void i2c_npcm_start(const struct device *dev)
{
	struct i2c_reg *const inst = I2C_INSTANCE(dev);

	inst->SMBnCTL1 = (inst->SMBnCTL1 & ~0x13) | BIT(NPCM_SMBnCTL1_START);
}

static inline void i2c_npcm_stop(const struct device *dev)
{
	struct i2c_reg *const inst = I2C_INSTANCE(dev);

	(inst)->SMBnCTL1 = ((inst)->SMBnCTL1 & ~0x13) | BIT(NPCM_SMBnCTL1_STOP);
}

static inline void i2c_npcm_enable_stall(const struct device *dev)
{
	struct i2c_reg *const inst = I2C_INSTANCE(dev);

	inst->SMBnCTL1 = (inst->SMBnCTL1 & ~0x13) | BIT(NPCM_SMBnCTL1_STASTRE);
}

static inline void i2c_npcm_disable_stall(const struct device *dev)
{
	struct i2c_reg *const inst = I2C_INSTANCE(dev);

	inst->SMBnCTL1 = inst->SMBnCTL1 & ~(0x13 | BIT(NPCM_SMBnCTL1_STASTRE));
}

static inline void i2c_npcm_nack(const struct device *dev)
{
	struct i2c_reg *const inst = I2C_INSTANCE(dev);

	inst->SMBnCTL1 = (inst->SMBnCTL1 & ~0x13) | BIT(NPCM_SMBnCTL1_ACK);
}

static void i2c_npcm_start_DMA(const struct device *dev,
				  uint32_t addr, uint16_t len)
{
	struct i2c_reg *const inst = I2C_INSTANCE(dev);
	struct i2c_npcm_data *data = dev->data;

	data->reg_buf[6] |= inst->DMA_CTRL;
	/* address */
	inst->DMA_ADDR1 = (uint8_t)((addr) >> 0);
	inst->DMA_ADDR2 = (uint8_t)((addr) >> 8);
	inst->DMA_ADDR3 = (uint8_t)((addr) >> 16);
	inst->DMA_ADDR4 = (uint8_t)((addr) >> 24);

	/* length */
	inst->DATA_LEN1 = (uint8_t)((len) >> 0);
	inst->DATA_LEN2 = (uint8_t)((len) >> 8);

	/* clear interrupt bit */
	inst->DMA_CTRL = BIT(NPCM_DMA_CTRL_DMA_INT_CLR);

	/* Set DMA enable */
	inst->DMA_CTRL = BIT(NPCM_DMA_CTRL_DMA_EN);
}

/* return Negative Acknowledge when DMA received last byte. */
static void i2c_npcm_DMA_lastbyte(const struct device *dev)
{
	struct i2c_reg *const inst = I2C_INSTANCE(dev);

	inst->DMA_CTRL = (inst->DMA_CTRL & ~BIT(NPCM_DMA_CTRL_DMA_INT_CLR)) |
			 BIT(NPCM_DMA_CTRL_LAST_PEC);
}

static uint16_t i2c_npcm_get_dma_cnt(const struct device *dev)
{
	struct i2c_reg *const inst = I2C_INSTANCE(dev);

	return (uint16_t)(((uint16_t)(inst->DATA_CNT1) << 8) + (uint16_t)(inst->DATA_CNT2));

}

#define NPCM_TIMEOUT_ST_ALL (BIT(NPCM_TIMEOUT_ST_T_OUTST1) | BIT(NPCM_TIMEOUT_ST_T_OUTST2))
#define NPCM_TIMEOUT_ST_EN_ALL                                                                 \
	(BIT(NPCM_TIMEOUT_ST_T_OUTST1_EN) | BIT(NPCM_TIMEOUT_ST_T_OUTST2_EN))

#if defined(CONFIG_CONTROLLER_HW_TIMEOUT_EN) || defined(CONFIG_TARGET_HW_TIMEOUT_EN)
#define I2C_NPCM_HW_TIMEOUT 1

#ifdef CONFIG_TARGET_HW_TIMEOUT_EN
#define I2C_HW_TIMEOUT_CLK_CYCLE_MS CONFIG_TARGET_HW_TIMEOUT_CLK_CYCLE_TIME
#define I2C_HW_TIMEOUT_CLK_LOW_MS CONFIG_TARGET_HW_TIMEOUT_CLK_LOW_TIME
#else
#define I2C_HW_TIMEOUT_CLK_CYCLE_MS CONFIG_CONTROLLER_HW_TIMEOUT_CLK_CYCLE_TIME
#define I2C_HW_TIMEOUT_CLK_LOW_MS CONFIG_CONTROLLER_HW_TIMEOUT_CLK_LOW_TIME
#endif

#define NPCM_TO_CKDIV_MIN 0x03
#define NPCM_TO_CKDIV_MAX 0x3F

/* Convert a time in ms to timeout clock ticks of source / ((div + 1) * 1000) Hz */
static uint8_t i2c_npcm_hw_timeout_ticks(uint32_t ms, uint32_t source_clk, uint32_t div)
{
	uint64_t ticks = (uint64_t)ms * source_clk / ((div + 1) * 1000ULL * 1000ULL);

	return MIN(ticks, UINT8_MAX);
}

/* TO_CKDIV may only be written while the module is disabled */
static void i2c_npcm_hw_timeout_init(const struct device *dev)
{
	struct i2c_reg *const inst = I2C_INSTANCE(dev);
	struct i2c_npcm_data *const data = dev->data;
	uint32_t div = data->source_clk / 1000 / 1000 - 1;

	if (div < NPCM_TO_CKDIV_MIN) {
		LOG_ERR("%s: no HW timeout divider for %u Hz", dev->name, data->source_clk);
		return;
	}

	/*
	 * TO_CKDIV only reaches a 1 kHz timeout clock up to 64 MHz. Above that the
	 * timeout clock runs faster, so scale the tick counts to keep the
	 * configured times in ms.
	 */
	div = MIN(div, NPCM_TO_CKDIV_MAX);

	inst->TIMEOUT_EN = div << NPCM_TIMEOUT_EN_TO_CKDIV;
	inst->TIMEOUT_CTL1 =
		i2c_npcm_hw_timeout_ticks(I2C_HW_TIMEOUT_CLK_CYCLE_MS, data->source_clk, div);
	inst->TIMEOUT_CTL2 =
		i2c_npcm_hw_timeout_ticks(I2C_HW_TIMEOUT_CLK_LOW_MS, data->source_clk, div);
}

/* T_OUTSTx are write-1-to-clear and share the register with their enables */
static void i2c_npcm_hw_timeout_arm(const struct device *dev)
{
	struct i2c_reg *const inst = I2C_INSTANCE(dev);

	inst->TIMEOUT_ST = NPCM_TIMEOUT_ST_EN_ALL | NPCM_TIMEOUT_ST_ALL;
	inst->TIMEOUT_EN |= BIT(NPCM_TIMEOUT_EN_TIMEOUT_EN);
}
#endif

static void i2c_npcm_reset_module(const struct device *dev)
{
	struct i2c_reg *const inst = I2C_INSTANCE(dev);
	struct i2c_npcm_data *const data = dev->data;
	uint32_t ctl1_tmp;
	uint32_t timeout_en_tmp;
	uint8_t timeout_st_en;

	ctl1_tmp = inst->SMBnCTL1;
	timeout_en_tmp = inst->TIMEOUT_EN;
	timeout_st_en = inst->TIMEOUT_ST & NPCM_TIMEOUT_ST_EN_ALL;

	/* Disable and then Enable I2C module */
	inst->SMBnCTL2 &= ~BIT(NPCM_SMBnCTL2_ENABLE);
	/* TO_CKDIV is written while disabled, TIMEOUT_EN only sticks once enabled */
	inst->TIMEOUT_EN = timeout_en_tmp & ~BIT(NPCM_TIMEOUT_EN_TIMEOUT_EN);
	inst->SMBnCTL2 |= BIT(NPCM_SMBnCTL2_ENABLE);

	inst->SMBnCTL1 = (ctl1_tmp & (BIT(NPCM_SMBnCTL1_INTEN) | BIT(NPCM_SMBnCTL1_EOBINTE) |
				      BIT(NPCM_SMBnCTL1_GCMEN) | BIT(NPCM_SMBnCTL1_NMINTE)));
	inst->TIMEOUT_ST = timeout_st_en | NPCM_TIMEOUT_ST_ALL;
	inst->TIMEOUT_EN = timeout_en_tmp;

	data->ctrl_oper_state = I2C_NPCM_OPER_STA_IDLE;
}

static void i2c_npcm_abort_data(const struct device *dev)
{
	struct i2c_reg *const inst = I2C_INSTANCE(dev);
	uint16_t timeout = ABORT_TIMEOUT;

	/* Generate a STOP condition */
	i2c_npcm_stop(dev);

	/* Clear NEGACK, STASTR and BER bits */
	inst->SMBnST = (BIT(NPCM_SMBnST_STASTR) |
			BIT(NPCM_SMBnST_NEGACK) |
			BIT(NPCM_SMBnST_BER));

	/* Wait till STOP condition is generated */
	while (--timeout) {
		if ((inst->SMBnCTL1 & BIT(NPCM_SMBnCTL1_STOP)) == 0x00) {
			break;
		}
	}

	/* Clear BB (BUS BUSY) bit */
	inst->SMBnCST = BIT(NPCM_SMBnCST_BB);
}

static uint16_t i2c_npcm_get_DMA_cnt(const struct device *dev)
{
	struct i2c_reg *const inst = I2C_INSTANCE(dev);

	return (uint16_t)(((uint16_t)(inst->DATA_CNT1) << 8) + (uint16_t)(inst->DATA_CNT2));
}

static void i2c_npcm_set_baudrate(const struct device *dev, uint32_t bus_freq)
{
	uint32_t reg_tmp;
	const struct i2c_npcm_config *const config = dev->config;
	struct i2c_reg *const inst = I2C_INSTANCE(dev);
	const struct device *const clk_dev = DEVICE_DT_GET(DT_NODELABEL(pcc));
	struct i2c_npcm_data *const data = dev->data;

	if (clock_control_get_rate(clk_dev, (clock_control_subsys_t)config->clk_cfg,
				   &data->source_clk) != 0) {
		LOG_ERR("Get %s clock source.", dev->name);
	}

	LOG_DBG("i2c clock source: %d", data->source_clk);

	reg_tmp = (data->source_clk) / (bus_freq * 4);

	if (bus_freq < 400000) {
		if (reg_tmp < 8) {
			reg_tmp = 8;
		} else if (reg_tmp > 511) {
			reg_tmp = 511;
		}

		/* Disable fast mode and fast mode plus */
		inst->SMBnCTL3 &= ~BIT(NPCM_SMBnCTL3_400K_MODE);
		SET_FIELD(inst->SMBnCTL2, NPCM_SMBnCTL2_SCLFRQ60_FIELD, reg_tmp & 0x7f);
		SET_FIELD(inst->SMBnCTL3, NPCM_SMBnCTL3_SCLFRQ87_FIELD, reg_tmp >> 7);

		/* Set HLDT (48MHz, HLDT = 17, Hold Time = 360ns) */
		if (data->source_clk >= 40000000) {
			inst->SMBnCTL4 = 17;
		} else if (data->source_clk >= 20000000) {
			inst->SMBnCTL4 = 9;
		} else {
			inst->SMBnCTL4 = 7;
		}
	} else {
		if (reg_tmp < 5) {
			reg_tmp = 5;
		} else if (reg_tmp > 255) {
			reg_tmp = 255;
		}

		/* Enable fast mode and fast mode plus */
		inst->SMBnCTL3 |= BIT(NPCM_SMBnCTL3_400K_MODE);
		SET_FIELD(inst->SMBnCTL2, NPCM_SMBnCTL2_SCLFRQ60_FIELD, 0);
		SET_FIELD(inst->SMBnCTL3, NPCM_SMBnCTL3_SCLFRQ87_FIELD, 0);
		inst->SMBnSCLHT = reg_tmp - 3;
		inst->SMBnSCLLT = reg_tmp - 1;

		/* Set HLDT (48MHz, HLDT = 17, Hold Time = 360ns) */
		if (data->source_clk >= 40000000) {
			inst->SMBnCTL4 = 17;
		} else if (data->source_clk >= 20000000) {
			inst->SMBnCTL4 = 9;
		} else {
			inst->SMBnCTL4 = 7;
		}
	}
}

static void i2c_npcm_notify(const struct device *dev, int error)
{
#if defined(CONFIG_CONTROLLER_HW_TIMEOUT_EN) && !defined(CONFIG_TARGET_HW_TIMEOUT_EN)
	struct i2c_reg *const inst = I2C_INSTANCE(dev);
#endif
	struct i2c_npcm_data *const data = dev->data;

#if defined(CONFIG_CONTROLLER_HW_TIMEOUT_EN) && !defined(CONFIG_TARGET_HW_TIMEOUT_EN)
	/* The timeout only guards controller transfers */
	inst->TIMEOUT_EN &= ~BIT(NPCM_TIMEOUT_EN_TIMEOUT_EN);
#endif

	data->ctrl_oper_state = I2C_NPCM_OPER_STA_IDLE;
	data->err_code = error;

	k_sem_give(&data->sync_sem);
}

static int i2c_npcm_wait_completion(const struct device *dev)
{
	struct i2c_npcm_data *const data = dev->data;
	int ret;

	ret = k_sem_take(&data->sync_sem, I2C_TRANS_TIMEOUT);
	if (ret != 0) {
		i2c_npcm_reset_module(dev);
		data->err_code = -ETIMEDOUT;
	}

	return data->err_code;
}

/* NPCX specific I2C controller functions */
static int i2c_npcm_mutex_lock(const struct device *dev, k_timeout_t timeout)
{
	struct i2c_npcm_data *const data = dev->data;

	/* target (un)register may be called from a target callback in ISR */
	if (k_is_in_isr()) {
		timeout = K_NO_WAIT;
	}

	return k_sem_take(&data->lock_sem, timeout);
}

static void i2c_npcm_mutex_unlock(const struct device *dev)
{
	struct i2c_npcm_data *const data = dev->data;

	k_sem_give(&data->lock_sem);
}

static void i2c_npcm_ctrl_isr(const struct device *dev)
{
	struct i2c_reg *const inst = I2C_INSTANCE(dev);
	struct i2c_npcm_data *const data = dev->data;

	/* --------------------------------------------- */
	/* NACK occurred                                 */
	/* --------------------------------------------- */
	if (inst->SMBnST & BIT(NPCM_SMBnST_NEGACK)) {
		i2c_npcm_abort_data(dev);
		/* Clear DMA flag */
		inst->DMA_CTRL = BIT(NPCM_DMA_CTRL_DMA_INT_CLR);
		data->ctrl_oper_state = I2C_NPCM_OPER_STA_IDLE;
		i2c_npcm_notify(dev, -ENXIO);
	}

	/* --------------------------------------------- */
	/* BUS ERROR occurred                            */
	/* --------------------------------------------- */
	if (inst->SMBnST & BIT(NPCM_SMBnST_BER)) {
		i2c_npcm_abort_data(dev);
		data->ctrl_oper_state = I2C_NPCM_OPER_STA_IDLE;
		i2c_npcm_reset_module(dev);
		i2c_npcm_notify(dev, -EAGAIN);
	}

	/* --------------------------------------------- */
	/* SDA status is set - transmit or receive       */
	/* --------------------------------------------- */
	if (inst->SMBnST & BIT(NPCM_SMBnST_SDAST)) {
		if (data->ctrl_oper_state == I2C_NPCM_OPER_STA_START) {
			if (data->tx_cnt == 0 && data->rx_cnt == 0) {
				/* Quick command (SMBUS protocol) */
				data->ctrl_oper_state = I2C_NPCM_OPER_STA_QUICK;
				i2c_npcm_enable_stall(dev);
				/* quick read or quick write is determined by address */
				inst->SMBnSDA = data->dev_addr;
			} else if (data->tx_cnt == 0 && data->rx_cnt > 0) {
				/* receive mode */
				data->ctrl_oper_state = I2C_NPCM_OPER_STA_READ;
				i2c_npcm_enable_stall(dev);
				/* send read address */
				inst->SMBnSDA = data->dev_addr | 0x1;
			} else if (data->tx_cnt > 0) {
				/* transmit mode */
				data->ctrl_oper_state = I2C_NPCM_OPER_STA_WRITE;
				/* send write address */
				inst->SMBnSDA = data->dev_addr & 0xFE;
			}
		} else if (data->ctrl_oper_state == I2C_NPCM_OPER_STA_WRITE) {
			/* Set DMA register to send data */
			sys_cache_data_flush_range((void *)data->tx_buf, data->tx_cnt);
			i2c_npcm_start_DMA(dev, (uint32_t)data->tx_buf, data->tx_cnt);
		} else {
			/* Error */
		}
	}

	/* --------------------------------------------- */
	/* stall occurred                                */
	/* --------------------------------------------- */
	if (inst->SMBnST & BIT(NPCM_SMBnST_STASTR)) {
		if (data->ctrl_oper_state == I2C_NPCM_OPER_STA_READ) {
			/* Set DMA register to read data */
			sys_cache_data_invd_range(data->rx_buf, data->rx_cnt);
			i2c_npcm_DMA_lastbyte(dev);
			i2c_npcm_start_DMA(dev, (uint32_t)data->rx_buf, data->rx_cnt);
		} else if (data->ctrl_oper_state == I2C_NPCM_OPER_STA_QUICK) {
			i2c_npcm_stop(dev);
			data->ctrl_oper_state = I2C_NPCM_OPER_STA_IDLE;
			i2c_npcm_notify(dev, 0);
		} else {
			/* error */
		}

		i2c_npcm_disable_stall(dev);
		/* clear STASTR flag */
		inst->SMBnST = BIT(NPCM_SMBnST_STASTR);
	}

	/* --------------------------------------------- */
	/* DMA IRQ occurred                              */
	/* --------------------------------------------- */
	if (inst->DMA_CTRL & BIT(NPCM_DMA_CTRL_DMA_IRQ)) {
		if (data->ctrl_oper_state == I2C_NPCM_OPER_STA_WRITE) {
			/* Transmit mode */
			if (data->rx_cnt == 0) {
				/* no need to receive data */
				i2c_npcm_stop(dev);
				data->ctrl_oper_state = I2C_NPCM_OPER_STA_IDLE;
				i2c_npcm_notify(dev, 0);
			} else {
				data->ctrl_oper_state = I2C_NPCM_OPER_STA_READ;
				i2c_npcm_enable_stall(dev);
				i2c_npcm_start(dev);
				inst->SMBnSDA = (data->dev_addr | 0x1);
			}
		} else {
			/* received mode */
			i2c_npcm_stop(dev);
			data->ctrl_oper_state = I2C_NPCM_OPER_STA_IDLE;
			data->rx_cnt = i2c_npcm_get_DMA_cnt(dev);
			sys_cache_data_invd_range(data->rx_buf, data->rx_cnt);
			i2c_npcm_notify(dev, 0);
		}
		/* Clear DMA flag */
		inst->DMA_CTRL = BIT(NPCM_DMA_CTRL_DMA_INT_CLR);
	}
}

static void i2c_npcm_target_stop(const struct device *dev)
{
	struct i2c_reg *const inst = I2C_INSTANCE(dev);
	struct i2c_npcm_data *const data = dev->data;
	struct i2c_target_config *cfg = data->target_cfg[data->match_addr_no];
	const struct i2c_target_callbacks *target_cb = cfg ? cfg->callbacks : NULL;
	uint16_t len;

	if (!target_cb) {
		data->target_oper_state = I2C_NPCM_OPER_STA_IDLE;
	} else if (data->target_oper_state == I2C_NPCM_OPER_STA_READ) {
		sys_cache_data_invd_range(data->rx_buf, sizeof(data->rx_buf));
		len = 0;
		while (len < i2c_npcm_get_dma_cnt(dev)) {
			target_cb->write_received(cfg, data->rx_buf[len]);
			len++;
		}
	}
	if (data->target_oper_state != I2C_NPCM_OPER_STA_IDLE) {
		target_cb->stop(cfg);
	}
	data->target_oper_state = I2C_NPCM_OPER_STA_START;
	/* clear STOP flag */
	inst->SMBnST = BIT(NPCM_SMBnST_SLVSTP);
}

static void i2c_npcm_target_isr(const struct device *dev)
{
	struct i2c_reg *const inst = I2C_INSTANCE(dev);
	struct i2c_npcm_data *const data = dev->data;
	uint16_t len;
	uint8_t overflow_data;

	/* --------------------------------------------- */
	/* NACK occurred                                 */
	/* --------------------------------------------- */
	if (inst->SMBnST & BIT(NPCM_SMBnST_NEGACK)) {
		inst->SMBnST = BIT(NPCM_SMBnST_NEGACK);
	}

	/* --------------------------------------------- */
	/* BUS ERROR occurred                            */
	/* --------------------------------------------- */
	if (inst->SMBnST & BIT(NPCM_SMBnST_BER)) {
		if (data->target_oper_state != I2C_NPCM_OPER_STA_QUICK) {
			LOG_ERR("target: bus error");
		}
		data->target_oper_state = I2C_NPCM_OPER_STA_START;
		/* clear BER */
		inst->SMBnST = BIT(NPCM_SMBnST_BER);
		i2c_npcm_reset_module(dev);
	}

	/* --------------------------------------------- */
	/* DMA IRQ occurred                              */
	/* --------------------------------------------- */
	if (inst->DMA_CTRL & BIT(NPCM_DMA_CTRL_DMA_IRQ)) {
		if (data->target_oper_state == I2C_NPCM_OPER_STA_READ) {
			/* if DMA overflow, send NACK to Controller, and next IRQ is SDAST alert */
			i2c_npcm_nack(dev);
		}
		/* clear DMA flag */
		inst->DMA_CTRL = BIT(NPCM_DMA_CTRL_DMA_INT_CLR);
	}

	/* --------------------------------------------- */
	/* Address match occurred                        */
	/* --------------------------------------------- */
	if (inst->SMBnST & BIT(NPCM_SMBnST_NMATCH)) {
		uint8_t i = 0;
		const struct i2c_target_callbacks *target_cb;

		if (inst->SMBnCST2 & BIT(NPCM_SMBnCST2_MATCHA1F))
			i = I2C_SLAVE_ADDR_1;
		else if (inst->SMBnCST2 & BIT(NPCM_SMBnCST2_MATCHA2F))
			i = I2C_SLAVE_ADDR_2;
		else if (inst->SMBnCST2 & BIT(NPCM_SMBnCST2_MATCHA3F))
			i = I2C_SLAVE_ADDR_3;
		else if (inst->SMBnCST2 & BIT(NPCM_SMBnCST2_MATCHA4F))
			i = I2C_SLAVE_ADDR_4;
		else if (inst->SMBnCST2 & BIT(NPCM_SMBnCST2_MATCHA5F))
			i = I2C_SLAVE_ADDR_5;
		else if (inst->SMBnCST2 & BIT(NPCM_SMBnCST2_MATCHA6F))
			i = I2C_SLAVE_ADDR_6;
		else if (inst->SMBnCST2 & BIT(NPCM_SMBnCST2_MATCHA7F))
			i = I2C_SLAVE_ADDR_7;
		else if (inst->SMBnCST3 & BIT(NPCM_SMBnCST3_MATCHA8F))
			i = I2C_SLAVE_ADDR_8;
		else if (inst->SMBnCST3 & BIT(NPCM_SMBnCST3_MATCHA9F))
			i = I2C_SLAVE_ADDR_9;
		else if (inst->SMBnCST3 & BIT(NPCM_SMBnCST3_MATCHA10F))
			i = I2C_SLAVE_ADDR_10;

		data->match_addr_no = i;

		if (!data->target_cfg[i]) {
			/*
			 * The target was unregistered (possibly from one of its
			 * own callbacks) while a match was pending: don't serve it.
			 */
			if (inst->SMBnST & BIT(NPCM_SMBnST_XMIT)) {
				data->target_oper_state = I2C_NPCM_OPER_STA_QUICK;
			} else {
				data->target_oper_state = I2C_NPCM_OPER_STA_IDLE;
				i2c_npcm_nack(dev);
			}
		} else if (inst->SMBnST & BIT(NPCM_SMBnST_XMIT)) {
			target_cb = data->target_cfg[i]->callbacks;
			/* target received Read-Address */
			if (data->target_oper_state != I2C_NPCM_OPER_STA_START) {
				/* target received data before */
				sys_cache_data_invd_range(data->rx_buf, sizeof(data->rx_buf));
				len = 0;
				while (len < i2c_npcm_get_dma_cnt(dev)) {
					target_cb->write_received(data->target_cfg[i],
								 data->rx_buf[len]);
					len++;
				}
			}

			/* prepare tx data */
			if (target_cb->read_requested(data->target_cfg[i], data->tx_buf) == 0) {
				len = 0;
				while (++len < sizeof(data->tx_buf)) {
					if (target_cb->read_processed(data->target_cfg[i],
							data->tx_buf + len) != 0) {
						break;
					}
				}
				data->tx_cnt = len;
			} else {
				/* target has no data to send */
				data->tx_cnt = 0;
			}

			data->target_oper_state = I2C_NPCM_OPER_STA_WRITE;

			if (data->tx_cnt != 0) {
				/* Set DMA register to send data */
				sys_cache_data_flush_range((void *)data->tx_buf, data->tx_cnt);
				i2c_npcm_start_DMA(dev, (uint32_t)data->tx_buf, data->tx_cnt);
			} else {
				data->target_oper_state = I2C_NPCM_OPER_STA_QUICK;
			}
		} else {
			target_cb = data->target_cfg[i]->callbacks;
			/* target received Write-Address */
			data->target_oper_state = I2C_NPCM_OPER_STA_READ;
			/* Set DMA register to get data */
			sys_cache_data_invd_range(data->rx_buf, sizeof(data->rx_buf));
			i2c_npcm_start_DMA(dev, (uint32_t)data->rx_buf, sizeof(data->rx_buf));
			target_cb->write_requested(data->target_cfg[i]);
		}
		/* Clear address match bit & SDA pull high */
		inst->SMBnST = BIT(NPCM_SMBnST_NMATCH);
	}

	/* --------------------------------------------- */
	/* SDA status is set - transmit or receive       */
	/* --------------------------------------------- */
	/*
	 * A START pending as controller is generated right after the STOP that
	 * ends a target write, so MASTER and SDAST can be set by the time the
	 * target STOP is handled. That SDAST asks for the controller address
	 * byte: leave it to the controller handler, or it is taken for a target
	 * overflow and a stale SMBnSDA byte is appended to the received data.
	 */
	if ((inst->SMBnST & (BIT(NPCM_SMBnST_SDAST) | BIT(NPCM_SMBnST_MASTER))) ==
	    BIT(NPCM_SMBnST_SDAST)) {
		struct i2c_target_config *cfg = data->target_cfg[data->match_addr_no];
		const struct i2c_target_callbacks *target_cb = cfg ? cfg->callbacks : NULL;

		if (!target_cb) {
			/* No registered target: drain or pad the byte */
			if (inst->SMBnST & BIT(NPCM_SMBnST_XMIT)) {
				inst->SMBnSDA = 0xFF;
			} else {
				overflow_data = inst->SMBnSDA;
			}
		} else if (data->target_oper_state == I2C_NPCM_OPER_STA_READ) {
			/* Over Flow */
			overflow_data = inst->SMBnSDA;
			sys_cache_data_invd_range(data->rx_buf, sizeof(data->rx_buf));

			len = 0;
			while (len < i2c_npcm_get_dma_cnt(dev)) {
				if (target_cb->write_received(data->target_cfg[data->match_addr_no], data->rx_buf[len]))
					break;

				len++;
			}
			target_cb->write_received(data->target_cfg[data->match_addr_no], overflow_data);
			data->target_oper_state = I2C_NPCM_OPER_STA_START;
		} else if (inst->SMBnST & BIT(NPCM_SMBnST_XMIT)) {
			/* No Enough DMA data to send */
			inst->SMBnSDA = 0xFF;
		} else {
			/* SDAST in receive mode is only cleared by reading SMBnSDA */
			overflow_data = inst->SMBnSDA;
		}
	}

	/* --------------------------------------------- */
	/* Target STOP occurred                           */
	/* --------------------------------------------- */
	if (inst->SMBnST & BIT(NPCM_SMBnST_SLVSTP)) {
		i2c_npcm_target_stop(dev);
	}
}

static void i2c_conf_slave(const struct device *dev, int index, uint8_t value)
{
	struct i2c_reg *const inst = I2C_INSTANCE(dev);

	switch (index) {
		case 0:
			inst->SMBnADDR1 = value;
			break;
		case 1:
			inst->SMBnADDR2 = value;
			break;
		case 2:
			inst->SMBnADDR3 = value;
			break;
		case 3:
			inst->SMBnADDR4 = value;
			break;
		case 4:
			inst->SMBnADDR5 = value;
			break;
		case 5:
			inst->SMBnADDR6 = value;
			break;
		case 6:
			inst->SMBnADDR7 = value;
			break;
		case 7:
			inst->SMBnADDR8 = value;
			break;
		case 8:
			inst->SMBnADDR9 = value;
			break;
		case 9:
			inst->SMBnADDR10 = value;
			break;
	}
}

static void i2c_set_target_addr(const struct device *dev, int index, uint8_t target_addr)
{
	struct i2c_reg *const inst = I2C_INSTANCE(dev);

	/* set slave addr */
	i2c_conf_slave(dev, index, (target_addr | BIT(NPCM_SMBnADDR_SAEN)));

	/* Enable I2C address match interrupt */
	if (!(inst->SMBnCTL1 & BIT(NPCM_SMBnCTL1_NMINTE)))
		inst->SMBnCTL1 |= BIT(NPCM_SMBnCTL1_NMINTE);
}

static int i2c_npcm_target_register(const struct device *dev,
				      struct i2c_target_config *cfg)
{
	struct i2c_npcm_data *data = dev->data;
	int ret = 0, free_index = -1, i;

	if (!cfg) {
		return -EINVAL;
	}
	if (cfg->flags & I2C_TARGET_FLAGS_ADDR_10_BITS) {
		return -ENOTSUP;
	}
	if (i2c_npcm_mutex_lock(dev, I2C_WAITING_TIME) != 0) {
		return -EBUSY;
	}

	for (i = 0; i < I2C_SLAVE_NUM; i++) {
		if (!data->target_cfg[i]) {
			if (free_index == -1)
				free_index = i;
			continue;
		}

		if (data->target_cfg[i]->address == cfg->address) {
			/* slave is already registered */
			ret = -EBUSY;
			goto exit;
		}
	}

	if (free_index == -1) {
		ret = -EBUSY;
		goto exit;
	}

	data->target_cfg[free_index] = cfg;
	data->target_oper_state = I2C_NPCM_OPER_STA_START;
	/* set target addr, cfg->address is 7 bit address */
	i2c_set_target_addr(dev, free_index, cfg->address);

exit:
	i2c_npcm_mutex_unlock(dev);
	return ret;
}

static int i2c_npcm_target_unregister(const struct device *dev,
					struct i2c_target_config *config)
{
	struct i2c_reg *const inst = I2C_INSTANCE(dev);
	struct i2c_npcm_data *data = dev->data;
	int i, found_index = -1;


	if (data->target_oper_state != I2C_NPCM_OPER_STA_START &&
	    data->ctrl_oper_state != I2C_NPCM_OPER_STA_IDLE) {
		return -EBUSY;
	}

	if (i2c_npcm_mutex_lock(dev, I2C_WAITING_TIME) != 0) {
		return -EBUSY;
	}

	for (i = 0; i < I2C_SLAVE_NUM; i++) {
		if (!data->target_cfg[i])
			continue;

		if (data->target_cfg[i]->address == config->address) {
			found_index = i;
			break;
		}
	}

	if (found_index == -1) {
		i2c_npcm_mutex_unlock(dev);
		return -EINVAL;
	}

	/* clear slave addr */
	i2c_conf_slave(dev, found_index, 0);

	/* clear all interrupt status */
	inst->SMBnST = 0xFF;

	data->target_oper_state = I2C_NPCM_OPER_STA_IDLE;
	data->target_cfg[found_index] = 0;

	/* Disable I2C address match interrupt
	 * only if all the slave addresses are
	 * removed.
	 */
	for (i = 0; i < I2C_SLAVE_NUM; i++) {
		if (data->target_cfg[i])
			break;
	}

	if (i == I2C_SLAVE_NUM)
		inst->SMBnCTL1 &= ~BIT(NPCM_SMBnCTL1_NMINTE);

	i2c_npcm_mutex_unlock(dev);

	return 0;
}

/* Disable/enable the module (drops any transfer) and keep serving as a target */
static void i2c_npcm_reset_keep_targets(const struct device *dev)
{
	struct i2c_reg *const inst = I2C_INSTANCE(dev);
	struct i2c_npcm_data *const data = dev->data;
	bool registered = false;
	int i;

	i2c_npcm_reset_module(dev);
	/* also clears a DMA_EN left from the dropped transaction */
	inst->DMA_CTRL = BIT(NPCM_DMA_CTRL_DMA_INT_CLR);
	inst->SMBnST = 0xFF;

	for (i = 0; i < I2C_SLAVE_NUM; i++) {
		if (data->target_cfg[i]) {
			registered = true;
			i2c_set_target_addr(dev, i, data->target_cfg[i]->address);
		}
	}
	data->target_oper_state = registered ? I2C_NPCM_OPER_STA_START : I2C_NPCM_OPER_STA_IDLE;
}

static bool i2c_npcm_lines_high(struct i2c_reg *const inst)
{
	return NPCM_IS_BIT_SET(inst->SMBnCTL3, NPCM_SMBnCTL3_SCL_LVL) &&
	       NPCM_IS_BIT_SET(inst->SMBnCTL3, NPCM_SMBnCTL3_SDA_LVL);
}

#ifdef I2C_NPCM_HW_TIMEOUT
/*
 * SCL was held low (T_OUTST1) or a transaction did not reach STOP
 * (T_OUTST2) in time. The detector does not touch the bus, so the
 * module has to be reset to release SCL/SDA, which also drops any
 * transaction it was part of.
 */
static void i2c_npcm_hw_timeout_recover(const struct device *dev, uint8_t timeout_st)
{
	struct i2c_reg *const inst = I2C_INSTANCE(dev);
	struct i2c_npcm_data *const data = dev->data;
	bool ctrl_pending = data->ctrl_oper_state != I2C_NPCM_OPER_STA_IDLE;
	/* still waiting for the bus: nothing was sent, so it can be retried */
	bool ctrl_no_start = data->ctrl_oper_state == I2C_NPCM_OPER_STA_START;

	/*
	 * The cycle detector counts from a START and is not always stopped by
	 * the STOP that ended the transaction: it then expires on a free bus.
	 * Resetting the module then could drop a target transfer that is
	 * about to start, so only clear the status when nothing uses the bus.
	 */
	if (!(timeout_st & BIT(NPCM_TIMEOUT_ST_T_OUTST1)) &&
	    !(inst->SMBnCST & BIT(NPCM_SMBnCST_BB)) && i2c_npcm_lines_high(inst) &&
	    !(inst->SMBnST & BIT(NPCM_SMBnST_MASTER)) &&
	    !ctrl_pending &&
	    data->target_oper_state != I2C_NPCM_OPER_STA_READ &&
	    data->target_oper_state != I2C_NPCM_OPER_STA_WRITE &&
	    data->target_oper_state != I2C_NPCM_OPER_STA_QUICK) {
		inst->TIMEOUT_ST = timeout_st | NPCM_TIMEOUT_ST_ALL;
		return;
	}

	LOG_ERR("%s: %s timeout (ST 0x%02x CST 0x%02x), resetting", dev->name,
		(timeout_st & BIT(NPCM_TIMEOUT_ST_T_OUTST1)) ? "SCL low" : "transaction",
		inst->SMBnST, inst->SMBnCST);

	i2c_npcm_reset_keep_targets(dev);

	if (ctrl_pending) {
		data->lost_bus = ctrl_no_start;
		i2c_npcm_notify(dev, ctrl_no_start ? -EAGAIN : -ETIMEDOUT);
	}
}
#endif

/* I2C controller isr function */
static void i2c_npcm_isr(const struct device *dev)
{
	struct i2c_reg *const inst = I2C_INSTANCE(dev);
	struct i2c_npcm_data *data = dev->data;

	data->reg_buf[0] |= inst->SMBnST;
	data->reg_buf[1] |= inst->SMBnCST;
	data->reg_buf[2] |= inst->SMBnCTL1;
	data->reg_buf[3] |= inst->TIMEOUT_ST;
	data->reg_buf[4] |= inst->TIMEOUT_EN;
	data->reg_buf[5] |= inst->SMBnCTL3;
	data->reg_buf[6] |= inst->DMA_CTRL;
	data->reg_buf[7] |= inst->SMBnCST2;
	data->reg_buf[8] |= inst->SMBnCST3;
	data->reg_buf[9] |= inst->SMBnCTL4;
	data->reg_buf[10] |= inst->SMBnCTL5;
	data->reg_buf[11] |= inst->SMBnSCLLT;
	data->reg_buf[12] |= inst->SMBnSCLHT;
	data->reg_buf[13] |= inst->TIMEOUT_CTL1;
	data->reg_buf[14] |= inst->TIMEOUT_CTL2;
	data->reg_buf[15] |= inst->SMBnPEC;

#ifdef I2C_NPCM_HW_TIMEOUT
	uint8_t timeout_st = inst->TIMEOUT_ST;

	if ((inst->TIMEOUT_EN & BIT(NPCM_TIMEOUT_EN_TIMEOUT_EN)) &&
	    (timeout_st & NPCM_TIMEOUT_ST_ALL)) {
		i2c_npcm_hw_timeout_recover(dev, timeout_st);
		return;
	}
#endif

	uint8_t st = inst->SMBnST;

	/*
	 * BER while requesting or owning the bus as controller means the
	 * module lost arbitration (or saw a misplaced START/STOP); MASTER is
	 * already cleared. It keeps matching the address of the controller
	 * that won (NPCM4xx SMBus Slave Mode), so do not reset it here: drop
	 * the controller transfer and let a match reach the target handler.
	 * The transfer resets the module once the bus is free again.
	 */
	if ((st & BIT(NPCM_SMBnST_BER)) && !(st & BIT(NPCM_SMBnST_MASTER)) &&
	    data->ctrl_oper_state != I2C_NPCM_OPER_STA_IDLE) {
		inst->SMBnST = BIT(NPCM_SMBnST_BER) | BIT(NPCM_SMBnST_NEGACK) |
			       BIT(NPCM_SMBnST_STASTR);
		inst->SMBnCTL1 &= ~(BIT(NPCM_SMBnCTL1_START) | BIT(NPCM_SMBnCTL1_STOP));
		/* also clears a DMA_EN set up for the dropped transfer */
		inst->DMA_CTRL = BIT(NPCM_DMA_CTRL_DMA_INT_CLR);
		data->lost_bus = true;
		i2c_npcm_notify(dev, -EAGAIN);
		st = inst->SMBnST;
	}

	bool target_active = data->target_oper_state == I2C_NPCM_OPER_STA_READ ||
			     data->target_oper_state == I2C_NPCM_OPER_STA_WRITE ||
			     data->target_oper_state == I2C_NPCM_OPER_STA_QUICK;

	/*
	 * A controller transfer can be requested while another controller is
	 * still addressing this module as a target. NMATCH/SLVSTP are
	 * write-1-to-clear and NMATCH stalls the bus, so they must reach the
	 * target handler even if a controller transfer is pending; otherwise
	 * the interrupt never deasserts and the CPU livelocks in this ISR.
	 */
	if (!(st & BIT(NPCM_SMBnST_MASTER)) &&
	    ((st & (BIT(NPCM_SMBnST_NMATCH) | BIT(NPCM_SMBnST_SLVSTP))) || target_active)) {
		i2c_npcm_target_isr(dev);
	} else if (data->ctrl_oper_state != I2C_NPCM_OPER_STA_IDLE) {
		/* A target write may have ended right before we got the bus */
		if (inst->SMBnST & BIT(NPCM_SMBnST_SLVSTP)) {
			i2c_npcm_target_stop(dev);
		}
		i2c_npcm_ctrl_isr(dev);
	} else {
		if (data->target_oper_state == I2C_NPCM_OPER_STA_IDLE) {
			/*
			 * No target is being served, but general call / ARP
			 * matching can still address the module. SDAST is only
			 * cleared by an SMBnSDA access, so service it here or
			 * the interrupt fires forever and the bus stays held.
			 */
			if (inst->SMBnST & BIT(NPCM_SMBnST_SDAST)) {
				if (inst->SMBnST & BIT(NPCM_SMBnST_XMIT)) {
					inst->SMBnSDA = 0xFF;
				} else {
					i2c_npcm_nack(dev);
					(void)inst->SMBnSDA;
				}
			}
			/* clear all interrupt status */
			inst->SMBnST = 0xFF;
		} else {
			i2c_npcm_target_isr(dev);
		}
	}
}


/* I2C controller driver registration */
static int i2c_npcm_init(const struct device *dev)
{
	const struct i2c_npcm_config *const config = dev->config;
	struct i2c_npcm_data *const data = dev->data;
	const struct device *const clk_dev = DEVICE_DT_GET(DT_NODELABEL(pcc));
	struct i2c_reg *const inst = I2C_INSTANCE(dev);
	int ret, i;

	LOG_DBG("Device name: %s", dev->name);

	/* Configure pin-mux for I2C device */
	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("I2C pinctrl setup failed (%d)", ret);
		return ret;
	}

	/* Turn on device clock first and get source clock freq. */
	if (clock_control_on(clk_dev, (clock_control_subsys_t)config->clk_cfg) != 0) {
		LOG_ERR("Turn on %s clock fail.", dev->name);
		return -EIO;
	}

	/* reset data */
	gdma_memset_u8((uint8_t *)data, 0, sizeof(struct i2c_npcm_data));

	/* Set default baud rate for i2c*/
	data->bitrate = config->default_bitrate;
	LOG_DBG("bitrate: %d", data->bitrate);
	i2c_npcm_set_baudrate(dev, data->bitrate);
	//inst->SMBnCTL5=0x07;

#ifdef I2C_NPCM_HW_TIMEOUT
	i2c_npcm_hw_timeout_init(dev);
#endif

	/* Enable I2C module */
	inst->SMBnCTL2 |= BIT(NPCM_SMBnCTL2_ENABLE);
	/* Enable I2C interrupt */
	inst->SMBnCTL1 |= BIT(NPCM_SMBnCTL1_INTEN);

#ifdef CONFIG_TARGET_HW_TIMEOUT_EN
	/* a target can be addressed at any time, so the timeout stays armed */
	i2c_npcm_hw_timeout_arm(dev);
#endif

	/* initialize mutux and semaphore for i2c controller */
	ret = k_sem_init(&data->lock_sem, 1, 1);
	ret = k_sem_init(&data->sync_sem, 0, 1);

	/* Initialize driver status machine */
	data->ctrl_oper_state = I2C_NPCM_OPER_STA_IDLE;

	for (i = 0; i < I2C_SLAVE_NUM; i++) {
		data->target_cfg[i] = NULL;
	}

	return 0;
}

static int i2c_npcm_check(const struct device *dev,uint8_t *reg_buf)
{
	struct i2c_npcm_data *const data = dev->data;

	memcpy(reg_buf, data->reg_buf, sizeof(data->reg_buf));

	return 0;
}
static int i2c_npcm_configure(const struct device *dev, uint32_t dev_config)
{
	struct i2c_npcm_data *const data = dev->data;

	if (!(dev_config & I2C_MODE_CONTROLLER)) {
		return -ENOTSUP;
	}

	if (dev_config & I2C_ADDR_10_BITS) {
		return -ENOTSUP;
	}

	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		/* 100 Kbit/s */
		data->bitrate = I2C_BITRATE_STANDARD;
		break;

	case I2C_SPEED_FAST:
		/* 400 Kbit/s */
		data->bitrate = I2C_BITRATE_FAST;
		break;

	case I2C_SPEED_FAST_PLUS:
		/* 1 Mbit/s */
		data->bitrate = I2C_BITRATE_FAST_PLUS;
		break;

	default:
		/* Not supported */
		return -ERANGE;
	}

	i2c_npcm_set_baudrate(dev, data->bitrate);

	return 0;
}

static int i2c_npcm_combine_msg(const struct device *dev,
				   struct i2c_msg *msgs, uint8_t num_msgs)
{
	struct i2c_npcm_data *const data = dev->data;
	uint8_t step = 0;
	uint8_t i;

	for (i = 0U; i < num_msgs; i++) {
		struct i2c_msg *msg = msgs + i;

		if ((msg->flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE) {
			/* support more than one write msg in a transfer */
			if (step == 0) {
				gdma_memcpy_u8(data->tx_buf + data->tx_cnt, msg->buf, msg->len);
				data->tx_cnt += msg->len;
			} else {
				return -1;
			}
		} else {
			/* just support one read msg in a transfer */
			if (step == 1) {
				return -1;
			}
			step = 1;
			data->rx_cnt = msg->len;
			data->rx_msg_buf = msg->buf;
		}
	}
	return 0;
}

int i2c_npcm_recover_bus(const struct device *dev)
{
	struct i2c_reg *const inst = I2C_INSTANCE(dev);
	struct i2c_npcm_data *const data  = dev->data;
	bool done = false;
	int32_t iter = 27;

	if (NPCM_IS_BIT_SET(inst->SMBnCTL3, NPCM_SMBnCTL3_SCL_LVL) &&
		NPCM_IS_BIT_SET(inst->SMBnCTL3, NPCM_SMBnCTL3_SDA_LVL)) {
		i2c_npcm_abort_data(dev);
		data->ctrl_oper_state = I2C_NPCM_OPER_STA_IDLE;
		i2c_npcm_reset_module(dev);
		LOG_INF("bus recovery skipped, bus is not stuck");
		return 0;
	}

	/* Repeat the following sequence until SDA is released */
	do {
		/* Issue a single SCL toggle */
		inst->SMBnCST |= BIT(NPCM_SMBnCST_TGSCL);
		k_busy_wait(I2C_RECOVER_BUS_DELAY_US);

		/* If SDA line is inactive (high), stop */
		if (NPCM_IS_BIT_SET(inst->SMBnCTL3, NPCM_SMBnCTL3_SDA_LVL)) {
			done = true;
		}

	} while (!done && iter--);


	i2c_npcm_abort_data(dev);
	data->ctrl_oper_state = I2C_NPCM_OPER_STA_IDLE;
	i2c_npcm_reset_module(dev);

	if (!(NPCM_IS_BIT_SET(inst->SMBnCTL3, NPCM_SMBnCTL3_SCL_LVL) &&
		NPCM_IS_BIT_SET(inst->SMBnCTL3, NPCM_SMBnCTL3_SDA_LVL))) {
		return -EBUSY;
	}

	return 0;
}

/*
 * Wait until no other controller owns the bus. Own addresses stay enabled
 * meanwhile, so a transaction addressed to this module is still served.
 */
static int i2c_npcm_wait_bus_free(const struct device *dev)
{
	struct i2c_reg *const inst = I2C_INSTANCE(dev);
	int64_t deadline = k_uptime_get() + I2C_BUS_FREE_TIMEOUT_MS;
	uint32_t idle_us;

	do {
		if (!(inst->SMBnCST & BIT(NPCM_SMBnCST_BB))) {
			return 0;
		}

		/*
		 * BB is only cleared by a STOP. If both lines stay high longer
		 * than SMBus tHIGH(max) no transaction is in progress, so the
		 * STOP was missed (e.g. the module was enabled mid-transaction).
		 */
		for (idle_us = 0; idle_us < I2C_BUS_IDLE_TIME_US;
		     idle_us += I2C_BUS_IDLE_POLL_US) {
			if (!i2c_npcm_lines_high(inst) ||
			    !(inst->SMBnCST & BIT(NPCM_SMBnCST_BB))) {
				break;
			}
			k_busy_wait(I2C_BUS_IDLE_POLL_US);
		}
		if (!(inst->SMBnCST & BIT(NPCM_SMBnCST_BB))) {
			return 0;
		}
		if (idle_us >= I2C_BUS_IDLE_TIME_US) {
			inst->SMBnCST = BIT(NPCM_SMBnCST_BB);
			return 0;
		}

		k_msleep(1);
	} while (k_uptime_get() < deadline);

	return -EAGAIN;
}

/*
 * After a lost arbitration the module may not generate another START until
 * it is disabled and enabled again (SMB Bus Error Recovery). Do that once
 * the other controller is done, unless it addresses this module again.
 */
static void i2c_npcm_reset_after_ber(const struct device *dev)
{
	struct i2c_reg *const inst = I2C_INSTANCE(dev);
	struct i2c_npcm_data *const data = dev->data;
	unsigned int key = irq_lock();

	if (!(inst->SMBnCST & BIT(NPCM_SMBnCST_BB)) &&
	    !(inst->SMBnST & BIT(NPCM_SMBnST_NMATCH)) &&
	    data->target_oper_state != I2C_NPCM_OPER_STA_READ &&
	    data->target_oper_state != I2C_NPCM_OPER_STA_WRITE &&
	    data->target_oper_state != I2C_NPCM_OPER_STA_QUICK) {
		i2c_npcm_reset_keep_targets(dev);
	}
	irq_unlock(key);
}

static int i2c_npcm_transfer(const struct device *dev, struct i2c_msg *msgs,
				uint8_t num_msgs, uint16_t addr)
{
	int ret;

	struct i2c_npcm_data *const data = dev->data;

	if (i2c_npcm_mutex_lock(dev, I2C_WAITING_TIME) != 0) {
		return -EBUSY;
	}

	ret = i2c_npcm_wait_bus_free(dev);
	if (ret != 0) {
		i2c_npcm_mutex_unlock(dev);
		return ret;
	}

	/*
	 * Own addresses stay enabled: if another controller wins arbitration
	 * and addresses this module, it is served as a target and the
	 * transfer is retried, instead of NACKing that controller.
	 */
	/* prepare data to transfer */
	data->rx_cnt = 0;
	data->tx_cnt = 0;
	data->dev_addr = addr << 1;
	data->ctrl_oper_state = I2C_NPCM_OPER_STA_START;
	data->err_code = 0;
	data->lost_bus = false;
	if (i2c_npcm_combine_msg(dev, msgs, num_msgs) < 0) {
		data->ctrl_oper_state = I2C_NPCM_OPER_STA_IDLE;
		i2c_npcm_mutex_unlock(dev);
		return -EPROTONOSUPPORT;
	}

	if (data->rx_cnt == 0 && data->tx_cnt == 0) {
		/* Quick command */
		if (num_msgs != 1) {
			/* Quick command must have one msg */
			data->ctrl_oper_state = I2C_NPCM_OPER_STA_IDLE;
			i2c_npcm_mutex_unlock(dev);
			return -EPROTONOSUPPORT;
		}
		if ((msgs->flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE) {
			/* set address to write address */
			data->dev_addr = data->dev_addr & 0xFE;
		} else {
			/* set address to read address */
			data->dev_addr = data->dev_addr | 0x1;
		}
	}

#if defined(CONFIG_CONTROLLER_HW_TIMEOUT_EN) && !defined(CONFIG_TARGET_HW_TIMEOUT_EN)
	i2c_npcm_hw_timeout_arm(dev);
#endif

	k_sem_reset(&data->sync_sem);

	i2c_npcm_start(dev);

	ret = i2c_npcm_wait_completion(dev);

	if (data->rx_cnt != 0) {
		sys_cache_data_invd_range(data->rx_buf, data->rx_cnt);
		gdma_memcpy_u8(data->rx_msg_buf, data->rx_buf, data->rx_cnt);
	}

	if (data->err_code == -EAGAIN) {
		/*
		 * Another controller owns the bus now. Let it finish; only
		 * clock the bus free if it never does. The message was not
		 * sent either way, so the caller has to retry it.
		 */
		if (!data->lost_bus || i2c_npcm_wait_bus_free(dev) != 0) {
			(void)i2c_npcm_recover_bus(dev);
		} else {
			i2c_npcm_reset_after_ber(dev);
		}
		ret = -EAGAIN;
	}

	i2c_npcm_mutex_unlock(dev);

	return ret;
}

static const struct i2c_driver_api i2c_npcm_driver_api = {
	.configure = i2c_npcm_configure,
	.transfer = i2c_npcm_transfer,
	.chech_reg = i2c_npcm_check,
	.target_register = i2c_npcm_target_register,
	.target_unregister = i2c_npcm_target_unregister,
	.recover_bus = i2c_npcm_recover_bus,
};


/* I2C controller init macro functions */
#define I2C_NPCM_CTRL_INIT_FUNC(inst) _CONCAT(i2c_npcm_init_, inst)
#define I2C_NPCM_CTRL_INIT_FUNC_DECL(inst)					 \
	static int i2c_npcm_init_##inst(const struct device *dev)
#define I2C_NPCM_CTRL_INIT_FUNC_IMPL(inst)					 \
	static int i2c_npcm_init_##inst(const struct device *dev)		 \
	{									 \
		int ret;							 \
										 \
		ret = i2c_npcm_init(dev);					 \
		IRQ_CONNECT(DT_INST_IRQN(inst),					 \
			    DT_INST_IRQ(inst, priority),			 \
			    i2c_npcm_isr,					 \
			    DEVICE_DT_INST_GET(inst),				 \
			    0);							 \
		irq_enable(DT_INST_IRQN(inst));					 \
										 \
		return ret;							 \
	}


#define I2C_NPCM_CTRL_INIT(inst)						 \
	PINCTRL_DT_INST_DEFINE(inst);						 \
										 \
	I2C_NPCM_CTRL_INIT_FUNC_DECL(inst);					 \
										 \
	static const struct i2c_npcm_config i2c_npcm_cfg_##inst = {		 \
		.base = DT_INST_REG_ADDR(inst),					 \
		.clk_cfg = DT_INST_PHA(inst, clocks, clk_cfg),			 \
		.default_bitrate = DT_INST_PROP(inst, clock_frequency),		 \
		.irq = DT_INST_IRQN(inst),					 \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),			 \
	};									 \
										 \
	static struct i2c_npcm_data i2c_npcm_data_##inst;			 \
										 \
	DEVICE_DT_INST_DEFINE(inst,						 \
			      I2C_NPCM_CTRL_INIT_FUNC(inst),			 \
			      NULL,						 \
			      &i2c_npcm_data_##inst, &i2c_npcm_cfg_##inst,	 \
			      PRE_KERNEL_1, CONFIG_I2C_INIT_PRIORITY,		 \
			      &i2c_npcm_driver_api);				 \
										 \
	I2C_NPCM_CTRL_INIT_FUNC_IMPL(inst)

DT_INST_FOREACH_STATUS_OKAY(I2C_NPCM_CTRL_INIT)
