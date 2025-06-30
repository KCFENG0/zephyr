/*
 * Copyright (c) 2025 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_I3C_I3C_NPCM_H_
#define ZEPHYR_DRIVERS_I3C_I3C_NPCM_H_

#include <zephyr/toolchain.h>
#include <zephyr/types.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/drivers/gpio.h>

/* Operation type */
enum npcm_i3c_oper_state {
	I3C_OP_STATE_IDLE,
	I3C_OP_STATE_WR,
	I3C_OP_STATE_RD,
	I3C_OP_STATE_IBI,
	I3C_OP_STATE_MAX,
};

/* I3C timing configuration for each i3c/i2c speed */
struct npcm_i3c_timing_cfg {
	uint8_t ppbaud;   /* Push-Pull high period */
	uint8_t pplow;    /* Push-Pull low period */
	uint8_t odhpp;    /* Open-Drain high period */
	uint8_t odbaud;   /* Open-Drain low period */
	uint8_t i2c_baud; /* I2C period */
};

/* NPCM I3C Configuration */
struct npcm_i3c_config {
	/* Common I3C Driver Config */
	struct i3c_driver_config common;

	/* Pointer to controller registers. */
	struct i3c_reg *base;

	/* Clock control subsys related struct. */
	uint32_t clk_cfg;

	/* Pointer to pin control device. */
	const struct pinctrl_dev_config *pincfg;

	/* Interrupt configuration function. */
	void (*irq_config_func)(const struct device *dev);

	struct {
		uint32_t i3c_pp_scl_hz; /* I3C push pull clock frequency in Hz. */
		uint32_t i3c_od_scl_hz; /* I3C open drain clock frequency in Hz. */
		uint32_t i2c_scl_hz;    /* I2C clock frequency in Hz */
	} clocks;

#ifdef CONFIG_I3C_NPCM_DMA
	struct pdma_dsct_reg *pdma_rx;
	struct pdma_dsct_reg *pdma_tx;
#endif
};

/* NPCM I3C Data */
struct npcm_i3c_data {
	struct i3c_driver_data common; /* Common i3c driver data */
	struct k_mutex lock_mutex;     /* Mutex of i3c controller */
	struct k_sem sync_sem;         /* Semaphore used for synchronization */
	struct k_sem ibi_lock_sem;     /* Semaphore used for ibi */

	/* Target data */
	struct i3c_target_config *target_config;
	/* Configuration parameters for I3C hardware to act as target device */
	struct i3c_config_target config_target;
	struct k_sem target_lock_sem;       /* Semaphore used for i3c target */
	struct k_sem target_event_lock_sem; /* Semaphore used for i3c target ibi_raise() */

	enum npcm_i3c_oper_state oper_state; /* Operation state */

#ifdef CONFIG_I3C_NPCM_DMA
	uint8_t dma_rx_buf[4096];
	uint16_t dma_rx_len;
	uint16_t dma_triggered; /* The bit n is set to 1 if the n-th DMA channel is triggered */
	bool tx_valid;          /* Tx has valid data */
#else
	uint8_t rx_buf[4096];
	uint16_t rx_len;
	uint8_t *tx_buf;
	uint16_t tx_len;
#endif

#ifdef CONFIG_I3C_USE_IBI
	struct {
		/* List of addresses used in the MIBIRULES register. */
		uint8_t addr[5];

		/* Number of valid addresses in MIBIRULES. */
		uint8_t num_addr;

		/* True if all addresses have MSB set. */
		bool msb;

		/* True if all target devices require mandatory byte for IBI. */
		bool has_mandatory_byte;
	} ibi;
#endif

#ifdef CONFIG_I3C_NPCM_DMA
	struct pdma_dsct_reg dsct_sg[4] __aligned(4); /* use for dma, 4-bytes align */
#endif
};

#endif /* ZEPHYR_DRIVERS_I3C_I3C_NPCM_H_ */
