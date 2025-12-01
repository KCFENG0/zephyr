/*
 * Copyright (c) 2024 Nuvoton Technology Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcm_sar_adc

#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <soc.h>

LOG_MODULE_REGISTER(adc_npcm_saradc, CONFIG_ADC_LOG_LEVEL);

/* NPCM SARADC register definitions */
#include "adc_npcm_sar.h"

/* NPCM SARADC timing and configuration constants */
#define NPCM_ADC_CLK                2000000     /* Target ADC clock: 2MHz */
#define NPCM_ADC_DELAY_US           10          /* ADC stabilization delay */
#define NPCM_ADC_TIMEOUT_US         1000        /* ADC conversion timeout */

/* Helper macro to get ADC register base */
#define HAL_INSTANCE(dev) \
    ((struct npcm_adc_regs *)(((const struct adc_npcm_saradc_config *)dev->config)->base))

/* Forward declarations */
struct adc_npcm_saradc_data;
struct adc_npcm_saradc_config;

/* Driver configuration structure */
struct adc_npcm_saradc_config {
    /* ADC controller base address */
    uintptr_t base;
    /* ADC input clock frequency in Hz */
    uint32_t input_clk_freq;
    /* Number of ADC channels */
    const uint8_t channel_count;
    /* Number of thresholds supported */
    const uint8_t threshold_count;
    /* IRQ configuration function */
    void (*irq_cfg_func)(void);
    /* Pin control configuration */
    const struct pinctrl_dev_config *pcfg;
};

/* Driver data structure */
struct adc_npcm_saradc_data {
    const struct device *adc_dev;
    struct k_sem acq_sem;
    struct k_sem sync_sem;
    uint16_t *buffer;
    uint32_t input_clk;
    const struct adc_sequence *sequence;
    bool calibration_done;
#ifdef CONFIG_PM
    bool pm_state_lock;
#endif
};

/* ADC interrupt handler */
static void adc_npcm_saradc_isr(const struct device *dev)
{
    struct adc_npcm_saradc_data *const data = dev->data;
    struct npcm_adc_regs *const inst = HAL_INSTANCE(dev);
    uint32_t status = inst->adcsts;
    uint16_t result;

    LOG_DBG("ADC ISR: status=0x%x", status);

    /* Handle single channel conversion completion */
    if (status & NPCM_ADCSTS_EOAINTS) {
        LOG_DBG("Single conversion completion interrupt");

        if (data->buffer) {
            result = FIELD_GET(NPCM_ADCDAT_ADCDATA_FIELD, inst->adcdat);
            *data->buffer = result;
            LOG_DBG("Single result: %d", result);
        }

        /* Stop single conversion and signal completion */
        inst->adccon = 0;
        inst->adcsts = NPCM_ADCSTS_EOAINTS;
        k_sem_give(&data->acq_sem);
    }
}

/* Start single channel conversion */
static int adc_npcm_saradc_start_single(const struct device *dev, uint8_t channel)
{
    const struct adc_npcm_saradc_config *const config = dev->config;
    struct npcm_adc_regs *const inst = HAL_INSTANCE(dev);

    if (channel >= config->channel_count) {
        LOG_ERR("Invalid channel %d (max %d)", channel, config->channel_count - 1);
        return -EINVAL;
    }

    LOG_DBG("Starting single conversion on channel %d", channel);

    /* Configure single channel operation */
    inst->adccon = NPCM_ADCCON_ADCINTEN |     /* Enable interrupts */
               NPCM_ADCCON_ADCEN;          /* Enable ADC */

    /* Set channel selection for single mode */
    inst->adcchansel = BIT(channel);

    /* Start single conversion */
    inst->adccon |= NPCM_ADCCON_ADCST;

    return 0;
}

/* Channel setup implementation */
static int adc_npcm_saradc_channel_setup(const struct device *dev,
                      const struct adc_channel_cfg *channel_cfg)
{
    const struct adc_npcm_saradc_config *const config = dev->config;

    LOG_DBG("Setting up channel %d", channel_cfg->channel_id);

    /* Validate channel */
    if (channel_cfg->channel_id >= config->channel_count) {
        LOG_ERR("Channel %d out of range (max %d)", 
            channel_cfg->channel_id, config->channel_count - 1);
        return -EINVAL;
    }

    /* Only single-ended mode supported currently */
    if (channel_cfg->differential) {
        LOG_ERR("Differential mode not supported");
        return -EINVAL;
    }

    /* Validate gain (should be 1 for SAR ADC) */
    if (channel_cfg->gain != ADC_GAIN_1) {
        LOG_ERR("Gain %d not supported, only ADC_GAIN_1", channel_cfg->gain);
        return -EINVAL;
    }

    /* Validate reference (should be internal) */
    if (channel_cfg->reference != ADC_REF_INTERNAL) {
        LOG_ERR("Reference %d not supported, only ADC_REF_INTERNAL", 
            channel_cfg->reference);
        return -EINVAL;
    }

    LOG_DBG("Channel %d configured successfully", channel_cfg->channel_id);
    return 0;
}

/* ADC read implementation */
static int adc_npcm_saradc_read(const struct device *dev,
                const struct adc_sequence *sequence)
{
    struct adc_npcm_saradc_data *const data = dev->data;
    int ret;

    if (!sequence) {
        LOG_ERR("Invalid sequence");
        return -EINVAL;
    }

    /* Take sync semaphore to serialize operations */
    if (k_sem_take(&data->sync_sem, K_MSEC(100)) != 0) {
        LOG_ERR("Failed to acquire sync semaphore");
        return -ETIMEDOUT;
    }

    /* Store sequence for ISR use */
    data->sequence = sequence;
    data->buffer = (uint16_t *)sequence->buffer;

    /* Only single channel conversion for now */
    uint8_t channel_count = __builtin_popcount(sequence->channels);
    if (channel_count != 1) {
        LOG_ERR("Only single channel conversion supported");
        k_sem_give(&data->sync_sem);
        return -ENOTSUP;
    }

    uint8_t channel = __builtin_ctz(sequence->channels);
    ret = adc_npcm_saradc_start_single(dev, channel);

    if (ret == 0) {
        /* Wait for conversion completion */
        ret = k_sem_take(&data->acq_sem, K_MSEC(1000));
        if (ret != 0) {
            LOG_ERR("Conversion timeout");
            ret = -ETIMEDOUT;
        }
    }

    /* Release sync semaphore */
    k_sem_give(&data->sync_sem);

    return ret;
}

/* Device initialization */
static int adc_npcm_saradc_init(const struct device *dev)
{
    const struct adc_npcm_saradc_config *const config = dev->config;
    struct adc_npcm_saradc_data *const data = dev->data;
    struct npcm_adc_regs *const inst = HAL_INSTANCE(dev);
    int prescaler = 0;

    /* Save device pointer */
    data->adc_dev = dev;

    /* Use fixed input clock frequency for NPCM5 (96MHz APB1) */
    data->input_clk = config->input_clk_freq;

    /* Calculate and set prescaler */
    prescaler = DIV_ROUND_UP(data->input_clk, NPCM_ADC_CLK);
    if (prescaler > 0x40) {
        prescaler = 0x40;
    }

    /* Set timing control - prescaler and delay */
    inst->atctl = FIELD_PREP(NPCM_ATCTL_SCLKDIV_FIELD, prescaler - 1) |
              FIELD_PREP(NPCM_ATCTL_DLY_FIELD, 0x8);

    LOG_INF("NPCM SAR ADC initialized: base=0x%lx, clk=%dHz, prescaler=%d", 
        config->base, data->input_clk, prescaler);

    /* Configure pin control */
    if (config->pcfg) {
        int ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
        if (ret != 0) {
            LOG_ERR("Failed to configure pins: %d", ret);
            return ret;
        }
    }

    /* Initialize semaphores */
    k_sem_init(&data->acq_sem, 0, 1);
    k_sem_init(&data->sync_sem, 1, 1);

    /* Configure interrupt */
    if (config->irq_cfg_func) {
        config->irq_cfg_func();
    }

    /* Calibration if needed */
    data->calibration_done = true;

    return 0;
}

/* Device instantiation macros */
#define ADC_NPCM_SARADC_IRQ_CFG_FUNC(n)						\
    static void adc_npcm_saradc_irq_cfg_func_##n(void)			\
    {									\
        IRQ_CONNECT(DT_INST_IRQN(n),					\
                DT_INST_IRQ(n, priority),				\
                adc_npcm_saradc_isr,				\
                DEVICE_DT_INST_GET(n), 0);				\
        irq_enable(DT_INST_IRQN(n));					\
    }

#define NPCM_SARADC_INIT(n)							\
    ADC_NPCM_SARADC_IRQ_CFG_FUNC(n)					\
                                        \
    static const struct adc_driver_api adc_npcm_saradc_driver_api_##n = {	\
        .channel_setup	= adc_npcm_saradc_channel_setup,		\
        .read		= adc_npcm_saradc_read,				\
    };									\
                                        \
    PINCTRL_DT_INST_DEFINE(n);						\
                                        \
    static const struct adc_npcm_saradc_config adc_npcm_saradc_cfg_##n = {	\
        .base			= DT_INST_REG_ADDR(n),			\
        .input_clk_freq		= 96000000, /* 96MHz APB1 for NPCM5 */	\
        .channel_count		= DT_INST_PROP(n, channel_count),	\
        .threshold_count	= DT_INST_PROP(n, threshold_count),	\
        .pcfg			= PINCTRL_DT_INST_DEV_CONFIG_GET(n),	\
        .irq_cfg_func		= adc_npcm_saradc_irq_cfg_func_##n,	\
    };									\
                                        \
    static struct adc_npcm_saradc_data adc_npcm_saradc_data_##n;		\
                                        \
    DEVICE_DT_INST_DEFINE(n, adc_npcm_saradc_init,				\
                   NULL,						\
                   &adc_npcm_saradc_data_##n,			\
                   &adc_npcm_saradc_cfg_##n,			\
                   PRE_KERNEL_1, CONFIG_ADC_INIT_PRIORITY,		\
                   &adc_npcm_saradc_driver_api_##n);

DT_INST_FOREACH_STATUS_OKAY(NPCM_SARADC_INIT)
