/*
 * Copyright (c) 2025 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcm_pdma

#include <zephyr/device.h>
#include <zephyr/cache.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/dt-bindings/dma/npcm_pdma.h>
#include <zephyr/irq.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dma_npcm_pdma, CONFIG_DMA_LOG_LEVEL);

/* Register access helper */
#define NPCM_SET_REG_FIELD(reg, mask, value)	((reg) = ((reg) & ~mask) | FIELD_PREP(mask, value))
#define NPCM_GET_REG_FIELD(reg, mask)		FIELD_GET(mask, reg)
#define NPCM_PDMA_REQSEL_CHANNEL(n)		GENMASK((n * 8) + 7, n * 8)
#define NPCM_CH_NUMS				DT_INST_PROP(0, dma_channels)
#define NPCM_PDMA_WAIT_STATUS_TIMEOUT_US	1000
#define Traffic_Abort	0x01
#define Transfer_Done	0x02
#define Table_Empty	0x04

volatile uint32_t u32PDMA_Done = 0;

struct dma_npcm_channel {
	dma_callback_t cb;
	void *user_data;
	enum dma_channel_direction direction;
	uint32_t src_addr;
	uint32_t dst_addr;
	size_t block_size;
};

struct dma_npcm_config {
	enum dma_channel_direction direction;
	struct pdma_dsct_reg *reg_base;
	void (*irq_config)(void);
};

struct dma_npcm_data {
	struct dma_npcm_channel channels[NPCM_CH_NUMS];
	struct pdma_dsct_reg dsct_sg[4] __aligned(4);  /* use for dma, 4-bytes align */
	uint16_t dma_triggered; /* The bit n is set to 1 if the n-th DMA channel is triggered */
	uint8_t dma_rx_buf[4096] __attribute__((aligned(32)));
};

static int npcm_pdma_dsct(const struct device *dev, uint32_t channel,
			  struct pdma_dsct_reg **dsct_inst)
{
	const struct dma_npcm_config *config = dev->config;

	*dsct_inst = (struct pdma_dsct_reg *)(config->reg_base + channel);

	return 0;
}

static int dma_npcm_cache_maintain(const void *addr, size_t size, bool flush)
{
	int ret;

	if ((addr == NULL) || (size == 0)) {
		return 0;
	}

	ret = flush ? sys_cache_data_flush_range((void *)addr, size)
		    : sys_cache_data_invd_range((void *)addr, size);
	if (ret < 0 && ret != -ENOTSUP) {
		LOG_ERR("cache %s failed (%d)", flush ? "flush" : "invalidate", ret);
		return ret;
	}

	return 0;
}

static int dma_npcm_config(const struct device *dev, uint32_t channel,
			   struct dma_config *dma_cfg)
{
	struct dma_npcm_channel *channel_control;
	struct dma_npcm_config *cfg = (struct dma_npcm_config *)dev->config;
	struct dma_npcm_data *data = dev->data;
	struct pdma_dsct_reg *dsct_inst = NULL;
	struct pdma_reg *pdma_inst = (struct pdma_reg *)cfg->reg_base;
	uint32_t ctrl = 0;
	uint32_t dst_addr;
	uint32_t src_addr;
	bool is_rx = (dma_cfg->channel_direction == PERIPHERAL_TO_MEMORY);
	bool is_scatter_mode = (dma_cfg->head_block->source_gather_en |
				dma_cfg->head_block->dest_scatter_en);

	npcm_pdma_dsct(dev, channel, &dsct_inst);
	if (dsct_inst == NULL) {
		LOG_ERR("dsct(%d) not exist", channel);
		return -EINVAL;
	}

	if (channel >= NPCM_CH_NUMS) {
		LOG_ERR("channel must be < %" PRIu32 " (%" PRIu32 ")", NPCM_CH_NUMS, channel);
		return -EINVAL;
	}

	if (dma_cfg->head_block->source_addr_adj != DMA_ADDR_ADJ_INCREMENT &&
	    dma_cfg->head_block->source_addr_adj != DMA_ADDR_ADJ_NO_CHANGE) {
		LOG_ERR("invalid source_addr_adj %" PRIu16, dma_cfg->head_block->source_addr_adj);
		return -ENOTSUP;
	}
	if (dma_cfg->head_block->dest_addr_adj != DMA_ADDR_ADJ_INCREMENT &&
	    dma_cfg->head_block->dest_addr_adj != DMA_ADDR_ADJ_NO_CHANGE) {
		LOG_ERR("invalid dest_addr_adj %" PRIu16, dma_cfg->head_block->dest_addr_adj);
		return -ENOTSUP;
	}

	if (dma_cfg->source_data_size != 1 && dma_cfg->source_data_size != 2 &&
	    dma_cfg->source_data_size != 4) {
		LOG_ERR("source_data_size must be 1, 2, or 4 (%" PRIu32 ")",
			dma_cfg->source_data_size);
		return -EINVAL;
	}

	if (dma_cfg->source_data_size != dma_cfg->dest_data_size) {
		return -EINVAL;
	}

	if (dma_cfg->dest_data_size != 1 && dma_cfg->dest_data_size != 2 &&
	    dma_cfg->dest_data_size != 4) {
		LOG_ERR("dest_data_size must be 1, 2, or 4 (%" PRIu32 ")", dma_cfg->dest_data_size);
		return -EINVAL;
	}

	if (dma_cfg->channel_direction != MEMORY_TO_PERIPHERAL &&
	    dma_cfg->channel_direction != PERIPHERAL_TO_MEMORY) {
		LOG_ERR("channel_direction must be MEMORY_TO_PERIPHERAL , "
			"PERIPHERAL_TO_MEMORY (%" PRIu32 ")",
			dma_cfg->channel_direction);
		return -ENOTSUP;
	}

	/* Setup channel request selection */
	NPCM_SET_REG_FIELD(pdma_inst->PDMA_REQSEL[channel / NPCM_PDMA_CHANNEL_PER_REQ],
			   NPCM_PDMA_REQSEL_CHANNEL(channel % NPCM_PDMA_CHANNEL_PER_REQ),
			   dma_cfg->dma_slot);

	if (is_scatter_mode) {
		dsct_inst->CTL = NPCM_PDMA_DSCT_CTL_OPMODE_SGM;
		dsct_inst->SA = 0x0;
		dsct_inst->DA = 0x0;
		dsct_inst->NEXT = (is_rx) ? ((uint32_t)&data->dsct_sg[0]) & 0xFFFF
					  : ((uint32_t)&data->dsct_sg[2]) & 0xFFFF;

		/* Configure scatter-gather table base MSB address. */
		pdma_inst->PDMA_SCATBA = (is_rx) ? ((uint32_t)&data->dsct_sg[0]) & 0xFFFF0000
						 : ((uint32_t)&data->dsct_sg[2]) & 0xFFFF0000;
	} else {
		dsct_inst->CTL = NPCM_PDMA_DSCT_CTL_OPMODE_BASIC;
		dsct_inst->SA = dma_cfg->head_block->source_address;
		dsct_inst->DA = dma_cfg->head_block->dest_address;
		dsct_inst->NEXT = 0x0;	
	}

	/* Set 8 bits transfer width */
	NPCM_SET_REG_FIELD(ctrl, NPCM_PDMA_DSCT_CTL_TXWIDTH, NPCM_PDMA_DSCT_CTL_TX_WIDTH_8);

	/* Set DMA single request type */
	ctrl |= NPCM_PDMA_DSCT_CTL_TXTYPE_SINGLE;

	/* Set mode as basic means the last descriptor */
	NPCM_SET_REG_FIELD(ctrl, NPCM_PDMA_DSCT_CTL_OPMODE, NPCM_PDMA_DSCT_CTL_OPMODE_BASIC);

	if (is_scatter_mode == false) {
		LOG_DBG(" in basic mode");
		/* Set transfer size, TXCNT + 1 */
		NPCM_SET_REG_FIELD(ctrl, NPCM_PDMA_DSCT_CTL_TXCNT,
				   dma_cfg->head_block->block_size - 1);
		NPCM_SET_REG_FIELD(ctrl, NPCM_PDMA_DSCT_CTL_SAINC,
				   (dma_cfg->head_block->source_addr_adj == DMA_ADDR_ADJ_INCREMENT) ?
				   0x0 : NPCM_PDMA_DSCT_CTL_SAINC_FIX);
		NPCM_SET_REG_FIELD(ctrl, NPCM_PDMA_DSCT_CTL_DAINC,
				   (dma_cfg->head_block->dest_addr_adj == DMA_ADDR_ADJ_INCREMENT) ?
				   0x0 : NPCM_PDMA_DSCT_CTL_DAINC_FIX);

		/* Set control value to descriptor */
		dsct_inst->CTL = ctrl;
	} else {
		LOG_DBG(" in scatter-gather mode");
		/*
		* For read DMA, fixed src address.
		* For write DMA, fixed dst address.
		*/
		if (is_rx) {
			/* Set transfer size, TXCNT + 1 */
			NPCM_SET_REG_FIELD(ctrl, NPCM_PDMA_DSCT_CTL_TXCNT, dma_cfg->head_block->block_size - 1);
			NPCM_SET_REG_FIELD(ctrl, NPCM_PDMA_DSCT_CTL_SAINC, NPCM_PDMA_DSCT_CTL_SAINC_FIX);
			NPCM_SET_REG_FIELD(ctrl, NPCM_PDMA_DSCT_CTL_DAINC, 0x0);

			/* Set source address */
			src_addr = dma_cfg->head_block->source_address;
		
			/* Fixed destination address */
			dst_addr = dma_cfg->head_block->dest_address;
		} else {
			NPCM_SET_REG_FIELD(ctrl, NPCM_PDMA_DSCT_CTL_DAINC, NPCM_PDMA_DSCT_CTL_DAINC_FIX);
			NPCM_SET_REG_FIELD(ctrl, NPCM_PDMA_DSCT_CTL_SAINC, 0x0);

			/* Fixed source address */
			src_addr = dma_cfg->head_block->source_address;

			/* Set transfer size, TXCNT + 1 */
			NPCM_SET_REG_FIELD(ctrl, NPCM_PDMA_DSCT_CTL_TXCNT, dma_cfg->head_block->block_size - 1);

			/* Set destination address */
			dst_addr = dma_cfg->head_block->dest_address;

			if (dma_cfg->head_block->block_size > 1) {
				/*
				* In this case need use second descriptor table, re-configure
				* first descriptor SGM and (tx_length - 2), the last byte use
				* second decriptor table.
				*/
				NPCM_SET_REG_FIELD(ctrl, NPCM_PDMA_DSCT_CTL_OPMODE, NPCM_PDMA_DSCT_CTL_OPMODE_SGM);
				NPCM_SET_REG_FIELD(ctrl, NPCM_PDMA_DSCT_CTL_TXCNT, (dma_cfg->head_block->block_size - 2));
			}
		}

		/* Set next descriptor */
		if (is_rx) {
			data->dsct_sg[0].CTL = ctrl;
			data->dsct_sg[0].SA = src_addr;
			data->dsct_sg[0].DA = dst_addr;
			data->dsct_sg[0].NEXT = 0x0;
		} else {
			data->dsct_sg[2].CTL = ctrl;
			data->dsct_sg[2].SA = src_addr;
			data->dsct_sg[2].DA = dst_addr;
			data->dsct_sg[2].NEXT = 0x0;

			/* If first descriptor use scatter-gather mode */
			if (NPCM_GET_REG_FIELD(data->dsct_sg[2].CTL, NPCM_PDMA_DSCT_CTL_OPMODE) ==
			    NPCM_PDMA_DSCT_CTL_OPMODE_SGM) {
				/* Configure next descriptor. */
				data->dsct_sg[2].NEXT = (uint32_t)&data->dsct_sg[3];

				/* Set basic mode for last descriptor */
				NPCM_SET_REG_FIELD(ctrl, NPCM_PDMA_DSCT_CTL_OPMODE, NPCM_PDMA_DSCT_CTL_OPMODE_BASIC);
				NPCM_SET_REG_FIELD(ctrl, NPCM_PDMA_DSCT_CTL_TXCNT, 0x0);

				data->dsct_sg[3].CTL = ctrl;
				data->dsct_sg[3].SA = 0x0;
				data->dsct_sg[3].DA = 0x0;
				data->dsct_sg[3].NEXT = 0x0;
			}
		}
	}

	channel_control = &data->channels[channel];
	channel_control->cb = dma_cfg->dma_callback;
	channel_control->user_data = dma_cfg->user_data;
	channel_control->direction = dma_cfg->channel_direction;
	channel_control->src_addr = dma_cfg->head_block->source_address;
	channel_control->dst_addr = dma_cfg->head_block->dest_address;
	channel_control->block_size = dma_cfg->head_block->block_size;

	LOG_DBG("Configured channel %d for %08X to %08X (%u)",
		channel,
		dsct_inst->SA,
		dsct_inst->DA,
		dma_cfg->head_block->block_size);

	if (is_scatter_mode) {
		LOG_DBG("Configure channel %d dsct_sg[0] 0x%x 0x%x 0x%x 0x%x",
			channel,
			data->dsct_sg[0].CTL,
			data->dsct_sg[0].SA,
			data->dsct_sg[0].DA,
			data->dsct_sg[0].NEXT);
		LOG_DBG("Configure channel %d dsct_sg[1] 0x%x 0x%x 0x%x 0x%x",
			channel,
			data->dsct_sg[1].CTL,
			data->dsct_sg[1].SA,
			data->dsct_sg[1].DA,
			data->dsct_sg[1].NEXT);
		LOG_DBG("Configure channel %d dsct_sg[2] 0x%x 0x%x 0x%x 0x%x",
			channel,
			data->dsct_sg[2].CTL,
			data->dsct_sg[2].SA,
			data->dsct_sg[2].DA,
			data->dsct_sg[2].NEXT);
		LOG_DBG("Configure channel %d dsct_sg[3] 0x%x 0x%x 0x%x 0x%x",
			channel,
			data->dsct_sg[3].CTL,
			data->dsct_sg[3].SA,
			data->dsct_sg[3].DA,
			data->dsct_sg[3].NEXT);
	}

	return 0;
 }

static void dma_npcm_pdma_isr(const struct device *dev)
{
	struct dma_npcm_config *cfg = (struct dma_npcm_config *)dev->config;
	struct pdma_reg *pdma_inst = (struct pdma_reg *)cfg->reg_base;
	uint32_t status = pdma_inst->PDMA_INTSTS;

	for (int channel = 0; channel < NPCM_CH_NUMS; channel++)
	{
		if (status & NPCM_PDMA_INTSTS_ABTIF) {			// abort
			if (pdma_inst->PDMA_ABTSTS & BIT(channel)) {
				u32PDMA_Done = Traffic_Abort;
				pdma_inst->PDMA_ABTSTS |= BIT(channel);
			}
		} else if (status & NPCM_PDMA_INTSTS_TDIF) {		// done
			if(pdma_inst->PDMA_TDSTS & BIT(channel)) {
				u32PDMA_Done = Transfer_Done;
				pdma_inst->PDMA_TDSTS |= BIT(channel);
			}
		} else if (status & NPCM_PDMA_INTSTS_TEIF) {		// empty
			if (pdma_inst->PDMA_SCATSTS & BIT(channel)) {
				u32PDMA_Done = Table_Empty;
				pdma_inst->PDMA_SCATSTS |= BIT(channel);
			}
		} else {
			LOG_DBG("unknown interrupt, status=0x%x!!\n", status);
		}
	}
}

 static int dma_npcm_reload(const struct device *dev, uint32_t channel, uint32_t src,
			      uint32_t dst, size_t size)
{
	struct dma_npcm_data *data = dev->data;
	struct pdma_dsct_reg *dsct_inst = NULL;

	npcm_pdma_dsct(dev, channel, &dsct_inst);
	if (dsct_inst == NULL) {
		LOG_ERR("dsct(%d) not exist", channel);
		return -EINVAL;
	}

	/* Reload source, destination and size */
	dsct_inst->SA = src;
	dsct_inst->DA = dst;
	NPCM_SET_REG_FIELD(dsct_inst->CTL, NPCM_PDMA_DSCT_CTL_TXCNT, size - 1);
	/* Reset NEXT pointer to the first scatter-gather descriptor */
	dsct_inst->NEXT = (dsct_inst->CTL & NPCM_PDMA_DSCT_CTL_OPMODE) ==
			  NPCM_PDMA_DSCT_CTL_OPMODE_SGM ?
			  (uint32_t)&(((struct dma_npcm_data *)dev->data)->dsct_sg[channel % 2 * 2]) & 0xFFFF : 0x0;

	data->channels[channel].src_addr = src;
	data->channels[channel].dst_addr = dst;
	data->channels[channel].block_size = size;

	LOG_DBG("Reloaded channel %d for %08X to %08X (%zu)",
		channel,
		dsct_inst->SA,
		dsct_inst->DA,
		size);

	return 0;
}

static int dma_npcm_start(const struct device *dev, uint32_t channel)
{
	struct dma_npcm_data *data = dev->data;
	struct dma_npcm_channel *channel_control;
	struct dma_npcm_config *cfg = (struct dma_npcm_config *)dev->config;
	struct pdma_reg *pdma_inst = (struct pdma_reg *)cfg->reg_base;
	uint32_t key;
	int ret;

	if (channel >= NPCM_CH_NUMS) {
		return -EINVAL;
	}

	channel_control = &data->channels[channel];

	if (channel_control->direction == MEMORY_TO_PERIPHERAL) {
		ret = dma_npcm_cache_maintain((void *)(uintptr_t)channel_control->src_addr,
					      channel_control->block_size, true);
		if (ret < 0) {
			return ret;
		}
	} else if (channel_control->direction == PERIPHERAL_TO_MEMORY) {
		ret = dma_npcm_cache_maintain((void *)(uintptr_t)channel_control->dst_addr,
					      channel_control->block_size, false);
		if (ret < 0) {
			return ret;
		}
	}

	key = irq_lock();

	/* Clear transfer done flag */
	if (pdma_inst->PDMA_TDSTS & BIT(channel))
		pdma_inst->PDMA_TDSTS |= BIT(channel);

	/* Start PDMA */
	LOG_DBG("Starting channel %d", channel);
	pdma_inst->PDMA_CHCTL |= BIT(channel);

	u32PDMA_Done = 0;
	/* Enable interrupt */
	pdma_inst->PDMA_INTEN |= BIT(channel);

	/* Set DMA triggered flag */
	data->dma_triggered |= BIT(channel);

#if 0 /* No needed, use interrupt to handle done */
	/* Check dma transfer done */	
	if (WAIT_FOR(NPCM_IS_BIT_SET(pdma_inst->PDMA_TDSTS, channel),
		NPCM_PDMA_WAIT_STATUS_TIMEOUT_US, NULL) == false) {
//		LOG_ERR("Check dma channel %d transfer done timed out", channel);
	}
#endif

	/* Wait for PDMA transfer done */
	while(u32PDMA_Done == Transfer_Done);

	pdma_inst->PDMA_INTEN &= ~BIT(channel);

	irq_unlock(key);

	if (channel_control->direction == PERIPHERAL_TO_MEMORY) {
		ret = dma_npcm_cache_maintain((void *)(uintptr_t)channel_control->dst_addr,
					      channel_control->block_size, false);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static int dma_npcm_stop(const struct device *dev, uint32_t channel)
{
	struct dma_npcm_data *data = dev->data;
	struct dma_npcm_config *cfg = (struct dma_npcm_config *)dev->config;
	struct pdma_reg *pdma_inst = (struct pdma_reg *)cfg->reg_base;
	uint32_t key;

	key = irq_lock();

	/* Clear transfer done flag */
	if (pdma_inst->PDMA_TDSTS & BIT(channel))
		pdma_inst->PDMA_TDSTS |= BIT(channel);

	pdma_inst->PDMA_CHCTL &= ~BIT(channel);

	/* Clear DMA triggered flag */
	data->dma_triggered &= ~BIT(channel);

	irq_unlock(key);

	return 0;
}

static int dma_npcm_get_status(const struct device *dev,
			       uint32_t channel,
			       struct dma_status *stat)
{
	struct dma_npcm_data *data = dev->data;
	struct dma_npcm_config *cfg = (struct dma_npcm_config *)dev->config;

	stat->busy = (data->dma_triggered & BIT(channel)) ? 1 : 0;
	stat->dir = cfg->direction;

	return 0;
}

static int dma_npcm_suspend(const struct device *dev, uint32_t channel)
{
	LOG_DBG("%s()", __func__);

	return -ENOSYS;
}

static int dma_npcm_resume(const struct device *dev, uint32_t channel)
{
	LOG_DBG("%s()", __func__);

	return -ENOSYS;
}

static const struct dma_driver_api dma_npcm_driver_api = {
	.config = dma_npcm_config,
	.reload = dma_npcm_reload,
	.start = dma_npcm_start,
	.stop = dma_npcm_stop,
	.get_status = dma_npcm_get_status,
	.suspend = dma_npcm_suspend,
	.resume = dma_npcm_resume,
};

static int dma_npcm_init(const struct device *dev)
{
	const struct dma_npcm_config *const cfg = dev->config;
	struct pdma_reg *pdma_inst = (struct pdma_reg *)cfg->reg_base;

	/* Disable all channels */
	pdma_inst->PDMA_CHCTL = 0x0;
	/* Disable all channel interrupts */
	pdma_inst->PDMA_INTEN = 0x0;

	/* Configure interrupts */
	cfg->irq_config();

	LOG_DBG("Device %s initialized", dev->name);
	return 0;
}

static void dma_npcm_irq_config(void)
{
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), dma_npcm_pdma_isr,
		    DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));
}

static const struct dma_npcm_config npcm_pdma_config = {
	.reg_base = (struct pdma_dsct_reg *)DT_INST_REG_ADDR(0),
	.irq_config = dma_npcm_irq_config,
};

static struct dma_npcm_data npcm_pdma_data;

DEVICE_DT_INST_DEFINE(0, &dma_npcm_init, NULL,
		      &npcm_pdma_data, &npcm_pdma_config,
		      PRE_KERNEL_1, CONFIG_DMA_INIT_PRIORITY,
		      &dma_npcm_driver_api);
