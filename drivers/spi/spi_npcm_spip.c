/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcm_spip

#if defined(CONFIG_SPIP_NPCM_DMA)
#include <zephyr/drivers/dma.h>
#endif
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_npcm_spip, CONFIG_SPI_LOG_LEVEL);

#include "spi_context.h"

/* Transfer this NOP value when tx buf is null */
#define SPI_NPCM_SPIP_TX_NOP                 0x00
#define SPI_NPCM_SPIP_WAIT_STATUS_TIMEOUT_US 1000

#define SPI_NPCM_SPIP_SINGLE                 0x0
#define SPI_NPCM_SPIP_DUAL                   0x1
#define SPI_NPCM_SPIP_QUAD                   0x2

/* SPIP_CLKDIV.DIVIDER is 9 bits: bus clock = source / (DIVIDER + 1) */
#define SPI_NPCM_MAX_PRESCALER_DIV 511

/* Maximum SPIP_CLK frequency in master mode */
#define SPI_NPCM_MAX_MASTER_FREQ MHZ(50)

struct spi_npcm_spip_data {
	struct spi_context ctx;
	uint32_t src_clock_freq;
	uint8_t bytes_per_frame;
	uint8_t access_mode;
#ifdef CONFIG_SPIP_NPCM_DMA
	uint8_t dma_rx_buf[4096] __attribute__((aligned(32)));
#endif
};

struct spi_npcm_spip_cfg {
	struct spip_reg *reg_base;
	uint32_t clk_cfg;
	const struct pinctrl_dev_config *pcfg;
#ifdef CONFIG_SPIP_NPCM_INTERRUPT
	/* routine for configuring SPIP ISR */
	void (*irq_cfg_func)(const struct device *dev);
#endif
#ifdef CONFIG_SPIP_NPCM_DMA
	const struct device *dma_dev;
	const uint32_t dma_tx_channel;
	const uint32_t dma_tx_perid;
	const uint32_t dma_rx_channel;
	const uint32_t dma_rx_perid;
#endif
};

static int spi_npcm_spip_configure(const struct device *dev, const struct spi_config *spi_cfg)
{
	uint32_t divider;
	const struct spi_npcm_spip_cfg *const config = dev->config;
	struct spi_npcm_spip_data *const data = dev->data;
	struct spip_reg *const reg_base = config->reg_base;
	spi_operation_t operation = spi_cfg->operation;
	uint8_t frame_size;

#ifndef CONFIG_SPI_NPCM_USE_RUNTIME_CONFIGURE
	if (spi_context_configured(&data->ctx, spi_cfg)) {
		/* This configuration is already in use */
		return 0;
	}
#endif

	if (operation & SPI_FULL_DUPLEX) {
		if ((operation & SPI_LINES_MASK) != SPI_LINES_SINGLE) {
			LOG_ERR("Full duplex mode only support for single line");
			return -ENOTSUP;
		}
	}

	if (SPI_OP_MODE_GET(operation) != SPI_OP_MODE_MASTER) {
		LOG_ERR("Only SPI controller mode is supported");
		return -ENOTSUP;
	}

	if (operation & SPI_MODE_LOOP) {
		LOG_ERR("Loopback mode is not supported");
		return -ENOTSUP;
	}

	if (spi_cfg->frequency == 0) {
		LOG_ERR("SPI frequency must be non-zero");
		return -EINVAL;
	}

	/*
	 * CTL/CLKDIV/SSCTL/FIFOCTL may only be changed with the controller
	 * disabled, confirmed by SPIENSTS.
	 */
	reg_base->SPIP_CTL &= ~BIT(NPCM_CTL_SPIEN);
	if (WAIT_FOR(!NPCM_IS_BIT_SET(reg_base->SPIP_STATUS, NPCM_STATUS_SPIENSTS),
		     SPI_NPCM_SPIP_WAIT_STATUS_TIMEOUT_US, NULL) == false) {
		LOG_ERR("Check SPIENSTS Timeout");
		return -ETIMEDOUT;
	}

	/* Get the frame length */
	frame_size = SPI_WORD_SIZE_GET(operation);

	switch (frame_size) {
		case 8:
			data->bytes_per_frame = 1;
			break;
		case 16:
			data->bytes_per_frame = 2;
			break;
		case 24:
			data->bytes_per_frame = 3;
			break;
		case 32:
			data->bytes_per_frame = 4;
			break;
		default:
			LOG_ERR("Only support word sizes 8/16/32 bits");
			return -ENOTSUP;
	}

	if (frame_size == 32) {
		SET_FIELD(reg_base->SPIP_CTL, NPCM_CTL_DWIDTH, 0x0);
	} else {
		SET_FIELD(reg_base->SPIP_CTL, NPCM_CTL_DWIDTH,
				SPI_WORD_SIZE_GET(operation));
	}

	switch (operation & SPI_LINES_MASK) {
		case SPI_LINES_SINGLE:
			reg_base->SPIP_CTL &= ~BIT(NPCM_CTL_DUALIOEN);
			reg_base->SPIP_CTL &= ~BIT(NPCM_CTL_QUADIOEN);
			data->access_mode = SPI_NPCM_SPIP_SINGLE;
			break;
		case SPI_LINES_DUAL:
			reg_base->SPIP_CTL |= BIT(NPCM_CTL_DUALIOEN);
			reg_base->SPIP_CTL &= ~BIT(NPCM_CTL_QUADIOEN);
			data->access_mode = SPI_NPCM_SPIP_DUAL;
			break;
		case SPI_LINES_QUAD:
			reg_base->SPIP_CTL &= ~BIT(NPCM_CTL_DUALIOEN);
			reg_base->SPIP_CTL |= BIT(NPCM_CTL_QUADIOEN);
			data->access_mode = SPI_NPCM_SPIP_QUAD;
			break;
		default:
			LOG_ERR("Only single/dual/quad line mode is supported");
			return -ENOTSUP;
	}

	/* Set the endianness */
	if (operation & SPI_TRANSFER_LSB) {
		reg_base->SPIP_CTL |= BIT(NPCM_CTL_LSB);
	} else {
		reg_base->SPIP_CTL &= ~BIT(NPCM_CTL_LSB);
	}

	/*
	 * Set CPOL and CPHA.
	 * The following is how to map npcm spip control register to CPOL and CPHA
	 *   CPOL    CPHA  |  CLKPOL  TXNEG   RXNEG
	 *   --------------------------------------
	 *    0       0    |    0       1       0
	 *    0       1    |    0       0       1
	 *    1       0    |    1       0       1
	 *    1       1    |    1       1       0
	 */
	if (operation & SPI_MODE_CPOL) {
		reg_base->SPIP_CTL |= BIT(NPCM_CTL_CLKPOL);
		if (operation & SPI_MODE_CPHA) {
			reg_base->SPIP_CTL |= BIT(NPCM_CTL_TXNEG);
			reg_base->SPIP_CTL &= ~BIT(NPCM_CTL_RXNEG);
		} else {
			reg_base->SPIP_CTL &= ~BIT(NPCM_CTL_TXNEG);
			reg_base->SPIP_CTL |= BIT(NPCM_CTL_RXNEG);
		}
	} else {
		reg_base->SPIP_CTL &= ~BIT(NPCM_CTL_CLKPOL);
		if (operation & SPI_MODE_CPHA) {
			reg_base->SPIP_CTL &= ~BIT(NPCM_CTL_TXNEG);
			reg_base->SPIP_CTL |= BIT(NPCM_CTL_RXNEG);
		} else {
			reg_base->SPIP_CTL |= BIT(NPCM_CTL_TXNEG);
			reg_base->SPIP_CTL &= ~BIT(NPCM_CTL_RXNEG);
		}
	}

	/* Active high CS logic */
	if (operation & SPI_CS_ACTIVE_HIGH) {
		reg_base->SPIP_SSCTL |= BIT(NPCM_SSCTL_SSACTPOL);
	} else {
		reg_base->SPIP_SSCTL &= ~BIT(NPCM_SSCTL_SSACTPOL);
	}

	/* Disable AUTOSS */
	reg_base->SPIP_SSCTL &= ~BIT(NPCM_SSCTL_AUTOSS);

	/* Set RX FIFO threshold to 0 */
	SET_FIELD(reg_base->SPIP_FIFOCTL, NPCM_FIFOCTL_RXTH, 0x0);

	/* Set the SPI frequency: the closest rate not above the requested one */
	divider = DIV_ROUND_UP(data->src_clock_freq,
			       MIN(spi_cfg->frequency, SPI_NPCM_MAX_MASTER_FREQ));
	if (divider > SPI_NPCM_MAX_PRESCALER_DIV + 1) {
		LOG_ERR("SPI frequency %u too low, min %u", spi_cfg->frequency,
			data->src_clock_freq / (SPI_NPCM_MAX_PRESCALER_DIV + 1));
		return -ENOTSUP;
	}
	SET_FIELD(reg_base->SPIP_CLKDIV, NPCM_CLKDIV_DIVIDER, divider - 1);

	data->ctx.config = spi_cfg;

	return 0;
}

static void spi_npcm_spip_cs_control(const struct device *dev, bool on)
{
	const struct spi_npcm_spip_cfg *const config = dev->config;
	struct spip_reg *const reg_base = config->reg_base;

	if (on) {
		reg_base->SPIP_SSCTL |= BIT(NPCM_SSCTL_SS);
	} else {
		reg_base->SPIP_SSCTL &= ~BIT(NPCM_SSCTL_SS);
	}
}

#ifndef CONFIG_SPIP_NPCM_DMA
static void spi_npcm_spip_process_tx_buf(struct spi_npcm_spip_data *const data, uint32_t *tx_frame)
{
	/* Get the tx_frame from tx_buf only when tx_buf != NULL */
	if (spi_context_tx_buf_on(&data->ctx)) {
		const uint8_t *buf = data->ctx.tx_buf;

		switch (data->bytes_per_frame) {
		case 2:
			*tx_frame = sys_get_le16(buf);
			break;
		case 3:
			*tx_frame = sys_get_le24(buf);
			break;
		case 4:
			*tx_frame = sys_get_le32(buf);
			break;
		default:
			*tx_frame = *buf;
			break;
		}
	}

	/*
	 * The update is ignored if TX is off (tx_len == 0).
	 * Note: if tx_buf == NULL && tx_len != 0, the update still counts.
	 */
	spi_context_update_tx(&data->ctx, data->bytes_per_frame, 1);
}
#endif

static void spi_npcm_spip_process_rx_buf(struct spi_npcm_spip_data *const data, uint32_t rx_frame)
{
	if (spi_context_rx_buf_on(&data->ctx)) {
		uint8_t *buf = data->ctx.rx_buf;

		switch (data->bytes_per_frame) {
		case 2:
			sys_put_le16((uint16_t)rx_frame, buf);
			break;
		case 3:
			sys_put_le24(rx_frame, buf);
			break;
		case 4:
			sys_put_le32(rx_frame, buf);
			break;
		default:
			*buf = (uint8_t)rx_frame;
			break;
		}
	}

	spi_context_update_rx(&data->ctx, data->bytes_per_frame, 1);
}

#ifdef CONFIG_SPIP_NPCM_DMA
static void spi_npcm_dma_callback(const struct device *dma_dev,
				  void *user_data,
				  uint32_t channel,
				  int status)
{
	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);
	ARG_UNUSED(status);

	const struct device *dev = user_data;
	struct spi_npcm_spip_data *drv_data = dev->data;

	k_sem_give(&drv_data->ctx.lock);
}

static int spi_npcm_dma_txrx(const struct device *dev,
			     const uint8_t *tx_buf,
			     const uint8_t *rx_buf,
			     const uint32_t len)
{
	const struct spi_npcm_spip_cfg *const config = dev->config;
	struct spip_reg *const reg_base = config->reg_base;
	const struct device *dma_dev = config->dma_dev;
	const int tx_channel = config->dma_tx_channel;
	const int rx_channel = config->dma_rx_channel;
	struct dma_config dma_tx_cfg = {0};
	struct dma_config dma_rx_cfg = {0};
	struct dma_block_config dma_tx_block_cfg = {0};
	struct dma_block_config dma_rx_block_cfg = {0};
	int ret;

	if (!device_is_ready(dma_dev)) {
		LOG_ERR("DMA device is not ready");
		return -ENODEV;
	}

	__ASSERT_NO_MSG(rx_buf != NULL || tx_buf != NULL);

	if (tx_buf != NULL) {
		/* Configure TX channel */
		dma_tx_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
		dma_tx_cfg.source_data_size = 1;
		dma_tx_cfg.dest_data_size = 1;
		dma_tx_cfg.block_count = 1,
		dma_tx_cfg.dma_slot = config->dma_tx_perid;
		dma_tx_cfg.source_burst_length = 1;
		dma_tx_cfg.dest_burst_length = 1;
		dma_tx_cfg.dma_callback = spi_npcm_dma_callback;
		dma_tx_cfg.user_data = (void *)dev;
		dma_tx_cfg.complete_callback_en = true;

		dma_tx_block_cfg.source_gather_en = 1;
		dma_tx_block_cfg.block_size = len;
		dma_tx_block_cfg.source_address = (uint32_t)tx_buf;
		dma_tx_block_cfg.dest_address = (uint32_t)&reg_base->SPIP_TX;
		dma_tx_block_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		dma_tx_block_cfg.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

		dma_tx_cfg.head_block = &dma_tx_block_cfg;

		ret = dma_config(dma_dev, tx_channel, &dma_tx_cfg);
		if (ret != 0) {
			LOG_ERR("Failed to configure TX channel");
			return ret;
		}
	}

	if (rx_buf != NULL) {
		/* Configure RX channel */
		dma_rx_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
		dma_rx_cfg.source_data_size = 1;
		dma_rx_cfg.dest_data_size = 1;
		dma_rx_cfg.block_count = 1,
		dma_rx_cfg.dma_slot = config->dma_rx_perid;
		dma_rx_cfg.source_burst_length = 1;
		dma_rx_cfg.dest_burst_length = 1;
		dma_rx_cfg.dma_callback = NULL;
		dma_rx_cfg.user_data = (void *)dev;
		dma_rx_cfg.complete_callback_en = true;

		dma_rx_block_cfg.dest_scatter_en = 1;
		dma_rx_block_cfg.block_size = len;
		dma_rx_block_cfg.source_address = (uint32_t)&reg_base->SPIP_RX;
		dma_rx_block_cfg.dest_address = (uint32_t)rx_buf;
		dma_rx_block_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		dma_rx_block_cfg.dest_addr_adj =  DMA_ADDR_ADJ_INCREMENT;

		dma_rx_cfg.head_block = &dma_rx_block_cfg;

		ret = dma_config(dma_dev, rx_channel, &dma_rx_cfg);
		if (ret != 0) {
			LOG_ERR("Failed to configure RX channel");
			return ret;
		}
	}

	return ret;
}
#endif

static int spi_npcm_spip_xfer_frame(const struct device *dev)
{
	const struct spi_npcm_spip_cfg *const config = dev->config;
	struct spip_reg *const reg_base = config->reg_base;
	struct spi_npcm_spip_data *const data = dev->data;
#ifndef CONFIG_SPIP_NPCM_DMA
	uint32_t tx_frame = SPI_NPCM_SPIP_TX_NOP;
#endif
	uint32_t rx_frame = 0;
#ifdef CONFIG_SPIP_NPCM_DMA
	int ret;
	const struct device *dma_dev = config->dma_dev;
	const int tx_channel = config->dma_tx_channel;
	const int rx_channel = config->dma_rx_channel;
#endif

#ifdef CONFIG_SPIP_NPCM_INTERRUPT
	reg_base->SPIP_FIFOCTL |= BIT(NPCM_FIFOCTL_TXTHIEN);
	reg_base->SPIP_FIFOCTL |= BIT(NPCM_FIFOCTL_RXTHIEN);
#endif

#ifdef CONFIG_SPIP_NPCM_DMA
	/* Add SPIP TX PDMA */
	if (data->ctx.tx_len) {
		/* Disable TX PDMA */
		reg_base->SPIP_PDMACTL &= ~BIT(NPCM_PDMACTL_TXPDMAEN);
		ret = dma_stop(dma_dev, tx_channel);
		if (ret != 0) {
			LOG_ERR("Failed to stop TX channel");
			return ret;
		}
		reg_base->SPIP_FIFOCTL |= (BIT(NPCM_FIFOCTL_TXRST) | BIT(NPCM_FIFOCTL_RXRST));
		while(reg_base->SPIP_STATUS & BIT(NPCM_STATUS_TXRXRST)) {};

		/* Configure and Enable TX */
		ret = spi_npcm_dma_txrx(dev,
					data->ctx.tx_buf,
					NULL,
					data->bytes_per_frame);
		if (ret != 0)
			return ret;

		/* Enable TX PDMA */
		reg_base->SPIP_PDMACTL |= BIT(NPCM_PDMACTL_TXPDMAEN);
		ret = dma_start(dma_dev, tx_channel);
		if (ret != 0) {
			LOG_ERR("Failed to start TX channel");
			return ret;
		}

		spi_context_update_tx(&data->ctx, data->bytes_per_frame, 1);
	}
#else
	spi_npcm_spip_process_tx_buf(data, &tx_frame);
	reg_base->SPIP_TX = tx_frame;
#endif

	if (WAIT_FOR(NPCM_IS_BIT_SET(reg_base->SPIP_STATUS, NPCM_STATUS_TXEMPTY),
			SPI_NPCM_SPIP_WAIT_STATUS_TIMEOUT_US, NULL) == false) {
		LOG_ERR("Check TX DONE Timeout");
		return -ETIMEDOUT;
	}

	/*
	 * TXEMPTY is already set while the frame is still being shifted out;
	 * in master mode BUSY stays set until the transaction has finished.
	 * This also leaves the controller idle for the next frame.
	 */
	if (WAIT_FOR(!NPCM_IS_BIT_SET(reg_base->SPIP_STATUS, NPCM_STATUS_BUSY),
		     SPI_NPCM_SPIP_WAIT_STATUS_TIMEOUT_US, NULL) == false) {
		LOG_ERR("Check Frame Done Timeout");
		return -ETIMEDOUT;
	}

	/* Every frame clocks one word into the RX FIFO, including the last one */
	if (WAIT_FOR(!NPCM_IS_BIT_SET(reg_base->SPIP_STATUS, NPCM_STATUS_RXEMPTY),
		     SPI_NPCM_SPIP_WAIT_STATUS_TIMEOUT_US, NULL) == false) {
		LOG_ERR("Check Status RBF Timeout");
		return -ETIMEDOUT;
	}

#ifdef CONFIG_SPIP_NPCM_DMA
	/* Add SPIP RX PDMA */
	if (data->ctx.rx_len) {
		/* Disable RX PDMA */
		reg_base->SPIP_PDMACTL &= ~BIT(NPCM_PDMACTL_RXPDMAEN);
		ret = dma_stop(dma_dev, rx_channel);
		if (ret != 0) {
			LOG_ERR("Failed to stop RX channel");
			return ret;
		}

		/* Configure and Enable RX */
		ret = spi_npcm_dma_txrx(dev,
					NULL,
					data->dma_rx_buf,
					data->bytes_per_frame);
		if (ret != 0)
			return ret;

		/* Enable RX PDMA */
		reg_base->SPIP_PDMACTL |= BIT(NPCM_PDMACTL_RXPDMAEN);
		ret = dma_start(dma_dev, rx_channel);
		if (ret != 0) {
			LOG_ERR("Failed to start RX channel");
			return ret;
		}

		rx_frame = data->dma_rx_buf[0];

		reg_base->SPIP_PDMACTL &= ~BIT(NPCM_PDMACTL_RXPDMAEN);
	}
#else
	rx_frame = reg_base->SPIP_RX;
#endif
	spi_npcm_spip_process_rx_buf(data, rx_frame);

#ifdef CONFIG_SPIP_NPCM_INTERRUPT
	reg_base->SPIP_FIFOCTL &= ~BIT(NPCM_FIFOCTL_TXTHIEN);
	reg_base->SPIP_FIFOCTL &= ~BIT(NPCM_FIFOCTL_RXTHIEN);
#endif

	return 0;
}

static bool spi_npcm_spip_transfer_ongoing(struct spi_npcm_spip_data *data)
{
	return spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx);
}

#ifdef CONFIG_SPIP_NPCM_INTERRUPT
static void spi_npcm_spip_isr(const struct device *dev)
{
	const struct spi_npcm_spip_cfg *const config = dev->config;
	struct spip_reg *const reg_base = config->reg_base;

	/* Transmit FIFO Threshold Interrupt or Transmit FIFO Empty Interrupt */
	if (NPCM_IS_BIT_SET(reg_base->SPIP_STATUS, NPCM_STATUS_TXTHIF) ||
	    NPCM_IS_BIT_SET(reg_base->SPIP_STATUS, NPCM_STATUS_TXEMPTY)) {
		reg_base->SPIP_FIFOCTL &= ~BIT(NPCM_FIFOCTL_TXTHIEN);
	}

	if (!NPCM_IS_BIT_SET(reg_base->SPIP_STATUS, NPCM_STATUS_RXEMPTY) &&
	    NPCM_IS_BIT_SET(reg_base->SPIP_STATUS, NPCM_STATUS_RXTHIF)) {
		reg_base->SPIP_FIFOCTL &= ~BIT(NPCM_FIFOCTL_RXTHIEN);
	}
}
#endif

static int spi_npcm_spip_transceive(const struct device *dev, const struct spi_config *spi_cfg,
				    const struct spi_buf_set *tx_bufs,
				    const struct spi_buf_set *rx_bufs)
{
	const struct spi_npcm_spip_cfg *const config = dev->config;
	struct spip_reg *const reg_base = config->reg_base;
	struct spi_npcm_spip_data *const data = dev->data;
	struct spi_context *ctx = &data->ctx;
	int rc;

	if (!tx_bufs && !rx_bufs) {
		return 0;
	}

	/* Lock the SPI Context */
	spi_context_lock(ctx, false, NULL, NULL, spi_cfg);

	rc = spi_npcm_spip_configure(dev, spi_cfg);
	if (rc < 0) {
		spi_context_release(ctx, rc);
		return rc;
	}

	spi_context_buffers_setup(ctx, tx_bufs, rx_bufs, data->bytes_per_frame);
	if (!spi_npcm_spip_transfer_ongoing(data)) {
		spi_context_release(ctx, 0);
		return 0;
	}

	/* Cleaning junk data in the buffer */
	while (!NPCM_IS_BIT_SET(reg_base->SPIP_STATUS, NPCM_STATUS_RXEMPTY)) {
		uint8_t unused __attribute__((unused));

		unused = reg_base->SPIP_RX;
	}

	/*
	 * Dual/quad I/O is half duplex: the whole transfer is output when it carries
	 * tx data and input otherwise. Set the direction before enabling SPIEN.
	 */
	if (data->access_mode != SPI_NPCM_SPIP_SINGLE) {
		if (spi_context_tx_buf_on(ctx)) {
			reg_base->SPIP_CTL |= BIT(NPCM_CTL_QDIODIR);
		} else {
			reg_base->SPIP_CTL &= ~BIT(NPCM_CTL_QDIODIR);
		}
	}

	/* Enable SPIP module */
	reg_base->SPIP_CTL |= BIT(NPCM_CTL_SPIEN);

	/* if cs is defined: software cs control, set active true */
	if (spi_cs_is_gpio(spi_cfg)) {
		spi_context_cs_control(ctx, true);
	} else {
		spi_npcm_spip_cs_control(dev, true);
	}

	do {
		rc = spi_npcm_spip_xfer_frame(dev);
		if (rc < 0) {
			break;
		}
	} while (spi_npcm_spip_transfer_ongoing(data));

	if (!(spi_cfg->operation & SPI_HOLD_ON_CS)) {
		/* if cs is defined: software cs control, set active false */
		if (spi_cs_is_gpio(spi_cfg)) {
			spi_context_cs_control(ctx, false);
		} else {
			spi_npcm_spip_cs_control(dev, false);
		}
	}

	/* Disable SPIP module */
	reg_base->SPIP_CTL &= ~BIT(NPCM_CTL_SPIEN);

	spi_context_release(ctx, rc);

	return rc;
}

static int spi_npcm_spip_release(const struct device *dev, const struct spi_config *spi_cfg)
{
	struct spi_npcm_spip_data *const data = dev->data;
	struct spi_context *ctx = &data->ctx;

	if (!spi_context_configured(ctx, spi_cfg)) {
		return -EINVAL;
	}

	spi_context_unlock_unconditionally(ctx);

	return 0;
}

static int spi_npcm_spip_init(const struct device *dev)
{
	int ret;
	struct spi_npcm_spip_data *const data = dev->data;
	const struct spi_npcm_spip_cfg *const config = dev->config;
	const struct device *const clk_dev = DEVICE_DT_GET(DT_NODELABEL(pcc));

	if (!device_is_ready(clk_dev)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	ret = clock_control_on(clk_dev, (clock_control_subsys_t)config->clk_cfg);
	if (ret < 0) {
		LOG_ERR("Turn on SPIP clock fail %d", ret);
		return ret;
	}

	ret = clock_control_get_rate(clk_dev, (clock_control_subsys_t)config->clk_cfg,
				     &data->src_clock_freq);
	if (ret < 0) {
		LOG_ERR("Get SPIP clock source rate error %d", ret);
		return ret;
	}

	ret = spi_context_cs_configure_all(&data->ctx);
	if (ret < 0) {
		return ret;
	}

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	/* Make sure the context is unlocked */
	spi_context_unlock_unconditionally(&data->ctx);

#ifdef CONFIG_SPIP_NPCM_INTERRUPT
	config->irq_cfg_func(dev);
#endif

	return 0;
}

static struct spi_driver_api spi_npcm_spip_api = {
	.transceive = spi_npcm_spip_transceive,
	.release = spi_npcm_spip_release,
};

#ifdef CONFIG_SPIP_NPCM_INTERRUPT
#define NPCM_SPIP_IRQ_HANDLER(n)                                                                   \
	static void spi_npcm_spip_irq_cfg_func_##n(const struct device *dev)                       \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), spi_npcm_spip_isr,          \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}

#define NPCM_SPIP_IRQ_HANDLER_FUNC(n) .irq_cfg_func = spi_npcm_spip_irq_cfg_func_##n,
#else
#define NPCM_SPIP_IRQ_HANDLER_FUNC(n)
#define NPCM_SPIP_IRQ_HANDLER(n)
#endif

#define NPCM_SPI_INIT(n)                                                                           \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	NPCM_SPIP_IRQ_HANDLER(n)                                                                   \
	static struct spi_npcm_spip_data spi_npcm_spip_data_##n = {                                \
		SPI_CONTEXT_INIT_LOCK(spi_npcm_spip_data_##n, ctx),                                \
		SPI_CONTEXT_INIT_SYNC(spi_npcm_spip_data_##n, ctx),                                \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)};                             \
                                                                                                   \
	static struct spi_npcm_spip_cfg spi_npcm_spip_cfg_##n = {                                  \
		.reg_base = (struct spip_reg *)DT_INST_REG_ADDR(n),                                \
		.clk_cfg = DT_INST_PHA(n, clocks, clk_cfg),                                        \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		IF_ENABLED(CONFIG_SPIP_NPCM_DMA, (                                                 \
			.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, tx)),                \
			.dma_tx_channel = DT_INST_DMAS_CELL_BY_NAME(n, tx, channel),               \
			.dma_tx_perid = DT_INST_DMAS_CELL_BY_NAME(n, tx, perid),                   \
			.dma_rx_channel = DT_INST_DMAS_CELL_BY_NAME(n, rx, channel),               \
			.dma_rx_perid = DT_INST_DMAS_CELL_BY_NAME(n, rx, perid),                   \
		))                                                                                 \
		NPCM_SPIP_IRQ_HANDLER_FUNC(n)};                                                    \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, spi_npcm_spip_init, NULL, &spi_npcm_spip_data_##n,                \
			      &spi_npcm_spip_cfg_##n, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,       \
			      &spi_npcm_spip_api);

DT_INST_FOREACH_STATUS_OKAY(NPCM_SPI_INIT)
