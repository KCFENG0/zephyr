/*
 * Copyright (c) 2024 Nuvoton Technology Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcm_sar_adc

#include <assert.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/adc/adc_npcm_saradc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/policy.h>
#include <zephyr/irq.h>
#include <soc.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

LOG_MODULE_REGISTER(adc_npcm_saradc, CONFIG_ADC_LOG_LEVEL);

/* ADC targeted operating frequency (2MHz) */
#define NPCM_SARADC_TARGET_CLK      2000000

/* ADC conversion modes */
#define NPCM_SARADC_CHN_CONVERSION_MODE  0
#define NPCM_SARADC_SCAN_CONVERSION_MODE 1

/* Max channel number to be converted in ADCCS */
#define NPCM_SARADC_MAX_CHANNEL_COUNT    16

/* ADC resolution */
#define NPCM_SARADC_RESOLUTION          10
#define ADC_NPCM_THRVAL_RESOLUTION      10
#define ADC_NPCM_THRVAL_MAX             BIT_MASK(ADC_NPCM_THRVAL_RESOLUTION)

/* Helper macros */
#define HAL_INSTANCE(dev) \
    ((struct adc_reg *)(((const struct adc_npcm_saradc_config *)dev->config)->base))

/* Forward declarations */
struct adc_npcm_saradc_data;
struct adc_npcm_saradc_config;

/* Threshold control internal structure */
struct adc_npcm_threshold_control {
	uint8_t chnsel;
	bool l_h;
	uint16_t thrval;
	uint16_t thr_dval;
	bool dual_mode;
	struct k_work *work;
};

/* Threshold data structure */
struct adc_npcm_threshold_data {
	uint32_t repetitive_channels;
	uint8_t active_thresholds;
	struct adc_npcm_threshold_control control[3];
};

/* Driver configuration structure */
struct adc_npcm_saradc_config {
    uintptr_t base;
    uint32_t input_clk_freq;
    const uint8_t channel_count;
    const uint8_t threshold_count;
    void (*irq_cfg_func)(void);
    const struct pinctrl_dev_config *pcfg;
};

/* Driver data structure */
struct adc_npcm_saradc_data {
    const struct device *adc_dev;
    struct adc_context ctx;
    uint32_t channels;
    uint32_t v2t_channels_msk;      /* V2T channel mask */
    uint32_t input_clk;
    uint16_t *buffer;
    uint16_t *repeat_buffer;
    uint16_t *buf_end;
    struct adc_npcm_threshold_data *threshold_data;
    uint8_t timer_triggered_channel;
    bool timer_triggered_enabled;
#ifdef CONFIG_PM
    atomic_t current_pm_lock;
#endif
};

/*
 * Pointer of internal work queue thread to be notified when threshold assertion
 * occurs if CONFIG_ADC_CMP_NPCX_WORKQUEUE is enabled.
 */
struct k_work_q *work_q;

/* Power management helpers */
#ifdef CONFIG_PM
static void adc_npcm_saradc_pm_policy_state_lock_get(struct adc_npcm_saradc_data *data)
{
    if (atomic_test_and_set_bit(&data->current_pm_lock, 0) == 0) {
        pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
    }
}

static void adc_npcm_saradc_pm_policy_state_lock_put(struct adc_npcm_saradc_data *data)
{
    if (atomic_test_and_clear_bit(&data->current_pm_lock, 0) == 1) {
        pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
    }
}
#endif

/* Channel configuration helper */
static inline void adc_npcm_saradc_config_channels(const struct device *dev, uint32_t channels)
{
    struct adc_reg *const inst = HAL_INSTANCE(dev);

    /* SARADC supports 16 channels maximum via ADCCS register */
    inst->ADCCS = channels & BIT_MASK(NPCM_SARADC_MAX_CHANNEL_COUNT);
}

/* V2T (Voltage to Temperature) functions */
#ifdef CONFIG_ADC_V2T_NPCX
int adc_npcm_v2t_set_channels(const struct device *dev, uint32_t channel_mask)
{
    struct adc_reg *const inst = HAL_INSTANCE(dev);
    const struct adc_npcm_saradc_config *config = dev->config;
    struct adc_npcm_saradc_data *const data = dev->data;

    /* Check selected channel mask within range */
    if (channel_mask & ~BIT_MASK(config->channel_count)) {
        LOG_ERR("Invalid V2T channel mask: 0x%08x", channel_mask);
        return -EINVAL;
    }

    /* Check V2T is disabled */
    if (IS_BIT_SET(inst->V2T_CTRL, NPCX_V2T_CTRL_TEMP_EN)) {
        LOG_ERR("V2T must be disabled before setting channels");
        return -EBUSY;
    }

    /* Set V2T channel mask */
    data->v2t_channels_msk = channel_mask & BIT_MASK(config->channel_count);

    return 0;
}

uint32_t adc_npcm_v2t_get_channels(const struct device *dev)
{
    struct adc_npcm_saradc_data *const data = dev->data;

    return data->v2t_channels_msk;
}

static void adc_npcm_v2t_config_channel(const struct device *dev, uint32_t ch_bitmap)
{
    struct adc_reg *const inst = HAL_INSTANCE(dev);

    inst->TEMP_CH = ch_bitmap;
}

static void adc_npcm_v2t_ch_apply(const struct device *dev, uint16_t enabled_ch)
{
    struct adc_reg *const inst = HAL_INSTANCE(dev);

    /* Check V2T channel is configured */
    if (enabled_ch == 0) {
        return;
    }

    /* Check V2T is not enabled before configuration */
    if (IS_BIT_SET(inst->V2T_CTRL, NPCX_V2T_CTRL_TEMP_EN)) {
        LOG_ERR("V2T already enabled");
        return;
    }

    /* Enable ADC temperature channel (TEMP_CH) */
    adc_npcm_v2t_config_channel(dev, enabled_ch);

    /* Clear temperature end of cyclic conversion event status flag */
    inst->ADCSTS |= BIT(NPCX_ADCSTS_TEOCCEV);

    /* Disable ADC raw end of cyclic conversion event interrupt */
    inst->ADCCNF &= ~BIT(NPCX_ADCCNF_INTECCEN);

    /* Enable ADC temperature end of cyclic conversion event interrupt */
    inst->V2T_CTRL |= BIT(NPCX_V2T_CTRL_TINTCC_EN);

    /* Enable the voltage to temperature function */
    inst->V2T_CTRL |= BIT(NPCX_V2T_CTRL_TEMP_EN);

    LOG_DBG("ADC temperature channel TEMP_CH is %#x", inst->TEMP_CH);
}
#endif /* CONFIG_ADC_V2T_NPCX */

/* Threshold detection functions */
static inline void adc_npcm_enable_threshold_detect(const struct device *dev, uint8_t th_sel,
                                                    bool enable)
{
    const struct adc_npcm_saradc_config *config = dev->config;

    if (enable) {
        THRCTL(config->base, th_sel) |= BIT(15);  /* NPCX_THRCTL_THEN bit */
    } else {
        THRCTL(config->base, th_sel) &= ~BIT(15);  /* NPCX_THRCTL_THEN bit */
    }
}

/* Setup dual threshold mode */
static void adc_npcm_setup_dual_threshold(const struct device *dev, uint8_t th_sel,
                                          uint16_t assert_val, uint16_t deassert_val)
{
    const struct adc_npcm_saradc_config *config = dev->config;

    /* Set assertion threshold in THRCTL */
    SET_FIELD(THRCTL(config->base, th_sel), NPCX_THR_DCTL_THR_DVAL, assert_val);

    /* Set deassertion threshold in THR_DCTL and enable dual mode */
#ifdef CONFIG_ADC_CMP_NPCX
    uint16_t thr_dctl_val = BIT(NPCX_THR_DCTL_THRD_EN);
    SET_FIELD(thr_dctl_val, NPCX_THR_DCTL_THR_DVAL, deassert_val);
    THR_DCTL(config->base, th_sel) = thr_dctl_val;
#endif
}

/* Timer-triggered mode functions */
static void adc_npcm_timer_trigger_enable(const struct device *dev, uint8_t channel)
{
    struct adc_reg *const inst = HAL_INSTANCE(dev);
    struct adc_npcm_saradc_data *const data = dev->data;

#ifdef CONFIG_PM
    adc_npcm_saradc_pm_policy_state_lock_get(data);
#endif

    /* Turn on ADC */
    inst->ADCCNF |= BIT(NPCX_ADCCNF_ADCEN);

    /* Set the channel to be converted */
    SET_FIELD(inst->ASCADD, NPCX_ASCADD_SADDR_FIELD, channel);

    /* Clear status */
    inst->ADCSTS = BIT(NPCX_ADCSTS_EOTCEV);

    /* Enable timer-triggered mode and interrupt */
    inst->ADCCNF |= BIT(NPCX_ADCCNF_ADCTTE) | BIT(NPCX_ADCCNF_INTETCEN);

    data->timer_triggered_channel = channel;
    data->timer_triggered_enabled = true;

    LOG_DBG("Timer-triggered mode enabled on channel %d", channel);
}

static void adc_npcm_timer_trigger_disable(const struct device *dev)
{
    struct adc_reg *const inst = HAL_INSTANCE(dev);
    struct adc_npcm_saradc_data *const data = dev->data;

    /* Disable timer-triggered mode */
    inst->ADCCNF &= ~(BIT(NPCX_ADCCNF_ADCTTE) | BIT(NPCX_ADCCNF_INTETCEN));

    /* Clear status */
    inst->ADCSTS = BIT(NPCX_ADCSTS_EOTCEV);

    data->timer_triggered_enabled = false;

#ifdef CONFIG_PM
    adc_npcm_saradc_pm_policy_state_lock_put(data);
#endif

    LOG_DBG("Timer-triggered mode disabled");
}

/* Repetitive mode control - API function for external use */
__maybe_unused
static void adc_npcm_set_repetitive(const struct device *dev, int chnsel,
                                    uint8_t enable)
{
    struct adc_reg *const inst = HAL_INSTANCE(dev);

    if (enable) {
#ifdef CONFIG_PM
        struct adc_npcm_saradc_data *const data = dev->data;
        adc_npcm_saradc_pm_policy_state_lock_get(data);
#endif

        /* Enable ADC */
        inst->ADCCNF |= BIT(NPCX_ADCCNF_ADCEN);

        /* Stop any ongoing conversion */
        inst->ADCCNF |= BIT(NPCX_ADCCNF_STOP);

        /* Clear status */
        inst->ADCSTS = 0xFF;

        /* Set channel */
        adc_npcm_saradc_config_channels(dev, BIT(chnsel));

        /* Enable repetitive mode */
        inst->ADCCNF |= BIT(NPCX_ADCCNF_ADCRPTC);

        /* Start conversion */
        inst->ADCCNF |= BIT(NPCX_ADCCNF_START);

        LOG_DBG("Repetitive mode enabled on channel %d", chnsel);
    } else {
        /* Stop conversion */
        inst->ADCCNF |= BIT(NPCX_ADCCNF_STOP);

        /* Disable repetitive mode */
        inst->ADCCNF &= ~BIT(NPCX_ADCCNF_ADCRPTC);

#ifdef CONFIG_PM
        adc_npcm_saradc_pm_policy_state_lock_put(data);
#endif

        LOG_DBG("Repetitive mode disabled");
    }
}

/* ADC interrupt handler */
static void adc_npcm_saradc_isr(const struct device *dev)
{
    const struct adc_npcm_saradc_config *const config = dev->config;
    struct adc_npcm_saradc_data *const data = dev->data;
    struct adc_reg *const inst = HAL_INSTANCE(dev);
    uint16_t status = inst->ADCSTS;
    uint16_t result;
    uint8_t channel;

    LOG_DBG("ADC ISR: status=0x%x", status);

    /* Handle threshold detection interrupts */
    if (data->threshold_data != NULL) {
        uint16_t thr_status = inst->THRCTS;

        for (uint8_t i = 0; i < config->threshold_count; i++) {
            if (thr_status & BIT(i + 1)) {  /* THRx_STS bits */
                LOG_DBG("Threshold %d triggered", i + 1);

                /* Notify work queue if configured */
                if (data->threshold_data->control[i].work != NULL) {
                    k_work_submit_to_queue(work_q, data->threshold_data->control[i].work);
                }

                /* Clear threshold status */
                inst->THRCTS = BIT(i + 1);
            }
        }
    }

    /* Handle overflow interrupt */
    if ((status & BIT(NPCX_ADCSTS_OVFEV)) && (inst->ADCCNF & BIT(9))) {  /* INTOVFEN */
        LOG_WRN("ADC overflow detected");

        /* Clear overflow flag */
        inst->ADCSTS = BIT(NPCX_ADCSTS_OVFEV);
    }

    /* Handle timer-triggered conversion interrupt */
    if ((status & BIT(NPCX_ADCSTS_EOTCEV)) && (inst->ADCCNF & BIT(8))) {  /* INTETCEN */
        channel = data->timer_triggered_channel;
        result = CHNDAT(config->base, channel);

        LOG_DBG("Timer-triggered conversion complete on channel %u: %u", 
                (unsigned int)channel, (unsigned int)(result & 0x3FF));

        /* Clear status */
        inst->ADCSTS = BIT(NPCX_ADCSTS_EOTCEV);
    }

    /* Handle single conversion interrupt */
    if ((status & BIT(NPCX_ADCSTS_EOCEV)) && (inst->ADCCNF & BIT(6))) {  /* INTECEN */
        LOG_DBG("Single conversion complete");

        /* Clear status */
        inst->ADCSTS = BIT(NPCX_ADCSTS_EOCEV);
    }

    /* Handle scan conversion cycle complete interrupt */
    if ((status & BIT(NPCX_ADCSTS_EOCCEV)) && (inst->ADCCNF & BIT(7))) {  /* INTECCEN */
        /* Stop conversion */
        inst->ADCCNF |= BIT(NPCX_ADCCNF_STOP);

        /* Read all selected channels */
        for (channel = 0; channel < config->channel_count; channel++) {
            if (data->channels & BIT(channel)) {
                result = CHNDAT(config->base, channel) & 0x3FF;  /* 10-bit data */

                if (data->buffer < data->buf_end) {
                    *data->buffer++ = result;
                    LOG_DBG("Channel %d: %d", channel, result);
                }
            }
        }

#ifdef CONFIG_PM
        adc_npcm_saradc_pm_policy_state_lock_put(data);
#endif
        adc_context_on_sampling_done(&data->ctx, dev);

        /* Clear status */
        inst->ADCSTS = BIT(NPCX_ADCSTS_EOCCEV);
    }

#ifdef CONFIG_ADC_V2T_NPCX
    /* Handle temperature conversion cycle complete interrupt */
    if ((status & BIT(NPCX_ADCSTS_TEOCCEV)) && (inst->V2T_CTRL & BIT(NPCX_V2T_CTRL_TINTCC_EN))) {
        /* Stop conversion */
        inst->ADCCNF |= BIT(NPCX_ADCCNF_STOP);

        /* Read temperature data for all V2T channels */
        for (channel = 0; channel < config->channel_count; channel++) {
            if (data->v2t_channels_msk & BIT(channel)) {
                result = TCHNDAT(config->base, channel);  /* Temperature data register */

                if (data->buffer < data->buf_end) {
                    *data->buffer++ = result;  /* 11-bit signed temperature data */
                    LOG_DBG("Temperature channel %d: %d", channel, (int16_t)result);
                }
            }
        }

#ifdef CONFIG_PM
        adc_npcm_saradc_pm_policy_state_lock_put(data);
#endif
        adc_context_on_sampling_done(&data->ctx, dev);

        /* Clear temperature status */
        inst->ADCSTS = BIT(NPCX_ADCSTS_TEOCCEV);
    }
#endif /* CONFIG_ADC_V2T_NPCX */
}

/* Validate buffer size */
static int adc_npcm_saradc_validate_buffer_size(const struct device *dev,
                                                const struct adc_sequence *sequence)
{
    const struct adc_npcm_saradc_config *const config = dev->config;
    uint8_t channel_count = 0;
    size_t required_size;

    for (uint8_t i = 0; i < config->channel_count; i++) {
        if (sequence->channels & BIT(i)) {
            channel_count++;
        }
    }

    required_size = channel_count * sizeof(uint16_t);
    if (sequence->options) {
        required_size *= (1 + sequence->options->extra_samplings);
    }

    if (sequence->buffer_size < required_size) {
        LOG_ERR("Buffer too small: need %zu, have %u", required_size, sequence->buffer_size);
        return -ENOMEM;
    }

    return 0;
}

/* Start scan conversion */
static void adc_npcm_saradc_start_scan(const struct device *dev)
{
    struct adc_npcm_saradc_data *const data = dev->data;
    struct adc_reg *const inst = HAL_INSTANCE(dev);

#ifdef CONFIG_PM
    adc_npcm_saradc_pm_policy_state_lock_get(data);
#endif

    /* Enable ADC */
    inst->ADCCNF |= BIT(NPCX_ADCCNF_ADCEN);

    /* Stop any ongoing conversion */
    inst->ADCCNF |= BIT(NPCX_ADCCNF_STOP);

    /* Clear status */
    inst->ADCSTS = BIT(NPCX_ADCSTS_EOCCEV);

    /* Configure channel selection */
    adc_npcm_saradc_config_channels(dev, data->channels);

    /* Set scan mode */
    uint16_t adccnf = inst->ADCCNF;
    adccnf = (adccnf & ~0x06) | (NPCM_SARADC_SCAN_CONVERSION_MODE << 1);  /* ADCMD field [2:1] */
    inst->ADCCNF = adccnf;

    /* Enable interrupt on conversion complete */
    inst->ADCCNF |= BIT(7);  /* INTECCEN */

    LOG_DBG("Starting scan: channels=0x%08x", data->channels);

    /* Start conversion */
    inst->ADCCNF |= BIT(NPCX_ADCCNF_START);
}

/* Start read operation */
static int adc_npcm_saradc_start_read(const struct device *dev,
                                      const struct adc_sequence *sequence)
{
    const struct adc_npcm_saradc_config *const config = dev->config;
    struct adc_npcm_saradc_data *const data = dev->data;
    int error;

    if (!sequence->channels || 
        (sequence->channels >= BIT(config->channel_count))) {
        LOG_ERR("Invalid channel mask: 0x%08x", sequence->channels);
        return -EINVAL;
    }

    if (sequence->resolution != NPCM_SARADC_RESOLUTION) {
        LOG_ERR("Resolution must be %d bits", NPCM_SARADC_RESOLUTION);
        return -EINVAL;
    }

    error = adc_npcm_saradc_validate_buffer_size(dev, sequence);
    if (error) {
        return error;
    }

    data->buffer = sequence->buffer;
    data->buf_end = data->buffer + (sequence->buffer_size / sizeof(uint16_t));

    adc_context_start_read(&data->ctx, sequence);

    return adc_context_wait_for_completion(&data->ctx);
}

/* ADC context callbacks */
static void adc_context_start_sampling(struct adc_context *ctx)
{
    struct adc_npcm_saradc_data *data =
        CONTAINER_OF(ctx, struct adc_npcm_saradc_data, ctx);

    data->repeat_buffer = data->buffer;
    data->channels = ctx->sequence.channels;

    adc_npcm_saradc_start_scan(data->adc_dev);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx,
                                              bool repeat_sampling)
{
    struct adc_npcm_saradc_data *data =
        CONTAINER_OF(ctx, struct adc_npcm_saradc_data, ctx);

    if (repeat_sampling) {
        data->buffer = data->repeat_buffer;
    }
}

/* Threshold control API functions */
int adc_npcm_threshold_ctrl_set_param(const struct device *dev,
                                      const uint8_t th_sel,
                                      const struct adc_npcm_threshold_param *param)
{
    const struct adc_npcm_saradc_config *config = dev->config;
    struct adc_npcm_saradc_data *data = dev->data;

    if (th_sel >= config->threshold_count) {
        return -EINVAL;
    }

    if (data->threshold_data == NULL) {
        return -ENOTSUP;
    }

    if (param->val_mv > (config->threshold_count * 1000)) {  /* Simplified check */
        return -EINVAL;
    }

    /* Save threshold parameters */
    data->threshold_data->control[th_sel].chnsel = param->chnsel;
    data->threshold_data->control[th_sel].l_h = param->l_h;
    data->threshold_data->control[th_sel].thrval = param->thrval;
    data->threshold_data->control[th_sel].work = param->work;
    data->threshold_data->control[th_sel].dual_mode = false;

    return 0;
}

int adc_npcm_threshold_ctrl_set_dual_param(const struct device *dev,
                                           const uint8_t th_sel,
                                           uint16_t thr_dval,
                                           bool l_h)
{
    const struct adc_npcm_saradc_config *config = dev->config;
    struct adc_npcm_saradc_data *data = dev->data;

    if (th_sel >= config->threshold_count) {
        return -EINVAL;
    }

    if (data->threshold_data == NULL) {
        return -ENOTSUP;
    }

    /* Save deassertion threshold */
    data->threshold_data->control[th_sel].thr_dval = thr_dval;
    data->threshold_data->control[th_sel].l_h = l_h;
    data->threshold_data->control[th_sel].dual_mode = true;

    return 0;
}

static int adc_npcm_threshold_ctrl_setup(const struct device *dev,
                                         const uint8_t th_sel)
{
    const struct adc_npcm_saradc_config *config = dev->config;
    struct adc_npcm_saradc_data *data = dev->data;
    struct adc_npcm_threshold_control *ctrl;

    if (th_sel >= config->threshold_count || data->threshold_data == NULL) {
        return -EINVAL;
    }

    ctrl = &data->threshold_data->control[th_sel];

    /* Configure threshold control register */
    uint16_t thrctl_val = ((ctrl->chnsel & 0xF) << 10) |  /* CHNSEL field [13:10] */
                          (ctrl->thrval & 0x3FF) |          /* THRVAL field [9:0] */
                          (ctrl->l_h ? BIT(14) : 0);        /* L_H bit */

    /* Write to THRCTLx register */
    THRCTL(config->base, th_sel) = thrctl_val;

    /* Setup dual threshold if enabled */
    if (ctrl->dual_mode) {
        adc_npcm_setup_dual_threshold(dev, th_sel, ctrl->thrval, ctrl->thr_dval);
    }

    return 0;
}

int adc_npcm_threshold_ctrl_enable(const struct device *dev, uint8_t th_sel,
                                   const bool enable)
{
    const struct adc_npcm_saradc_config *config = dev->config;
    struct adc_npcm_saradc_data *data = dev->data;
    struct adc_reg *const inst = HAL_INSTANCE(dev);
    int ret;

    if (th_sel >= config->threshold_count) {
        return -EINVAL;
    }

    if (enable) {
        /* Setup threshold before enabling */
        ret = adc_npcm_threshold_ctrl_setup(dev, th_sel);
        if (ret < 0) {
            return ret;
        }

        /* Enable threshold detection */
        adc_npcm_enable_threshold_detect(dev, th_sel, true);

        /* Enable threshold interrupt */
        inst->THRCTS |= BIT(NPCX_THRCTS_THR1_IEN + th_sel);

        /* Enable repetitive mode for the threshold channel */
        if (data->threshold_data != NULL) {
            data->threshold_data->active_thresholds |= BIT(th_sel);
            uint8_t chnsel = data->threshold_data->control[th_sel].chnsel;
            data->threshold_data->repetitive_channels |= BIT(chnsel);
            adc_npcm_set_repetitive(dev, chnsel, true);
        }
    } else {
        /* Disable threshold interrupt */
        inst->THRCTS &= ~BIT(NPCX_THRCTS_THR1_IEN + th_sel);

        /* Disable threshold detection */
        adc_npcm_enable_threshold_detect(dev, th_sel, false);

        /* Disable repetitive mode for the threshold channel */
        if (data->threshold_data != NULL) {
            data->threshold_data->active_thresholds &= ~BIT(th_sel);
            uint8_t chnsel = data->threshold_data->control[th_sel].chnsel;
            data->threshold_data->repetitive_channels &= ~BIT(chnsel);
            /* Only disable repetitive if no other threshold uses this channel */
            bool channel_used = false;
            for (uint8_t i = 0; i < config->threshold_count; i++) {
                if (i != th_sel &&
                    (data->threshold_data->active_thresholds & BIT(i)) &&
                    data->threshold_data->control[i].chnsel == chnsel) {
                    channel_used = true;
                    break;
                }
            }
            if (!channel_used) {
                adc_npcm_set_repetitive(dev, chnsel, false);
            }
        }
    }

    return 0;
}

int adc_npcm_threshold_mv_to_thrval(const struct device *dev, uint32_t val_mv,
                                    uint32_t *thrval)
{
    uint32_t vref_mv = 3300;  /* 3.3V reference */

    if (val_mv > vref_mv) {
        return -EINVAL;
    }

    /* Convert mV to 10-bit ADC value */
    *thrval = (val_mv * ADC_NPCM_THRVAL_MAX) / vref_mv;

    return 0;
}

/* Timer-triggered API functions */
int adc_npcm_timer_trigger_setup(const struct device *dev, uint8_t channel)
{
    const struct adc_npcm_saradc_config *config = dev->config;

    if (channel >= config->channel_count) {
        return -EINVAL;
    }

    adc_npcm_timer_trigger_enable(dev, channel);

    return 0;
}

int adc_npcm_timer_trigger_stop(const struct device *dev)
{
    adc_npcm_timer_trigger_disable(dev);

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

    /* Validate acquisition time */
    if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
        LOG_ERR("Custom acquisition time not supported");
        return -ENOTSUP;
    }

    LOG_DBG("Channel %d configured successfully", channel_cfg->channel_id);
    return 0;
}

/* ADC read implementation */
static int adc_npcm_saradc_read(const struct device *dev,
                const struct adc_sequence *sequence)
{
    struct adc_npcm_saradc_data *const data = dev->data;
    int error;

    adc_context_lock(&data->ctx, false, NULL);
    error = adc_npcm_saradc_start_read(dev, sequence);
    adc_context_release(&data->ctx, error);

    return error;
}

#ifdef CONFIG_ADC_ASYNC
static int adc_npcm_saradc_read_async(const struct device *dev,
                                     const struct adc_sequence *sequence,
                                     struct k_poll_signal *async)
{
    struct adc_npcm_saradc_data *const data = dev->data;
    int error;

    adc_context_lock(&data->ctx, true, async);
    error = adc_npcm_saradc_start_read(dev, sequence);
    adc_context_release(&data->ctx, error);

    return error;
}
#endif

/* Device initialization following SARADC spec exactly */
static int adc_npcm_saradc_init(const struct device *dev)
{
    const struct adc_npcm_saradc_config *const config = dev->config;
    struct adc_npcm_saradc_data *const data = dev->data;
    struct adc_reg *const inst = HAL_INSTANCE(dev);
    int prescaler = 0;
    volatile uint16_t *speed1_reg, *speed2_reg, *speed3_reg;

    LOG_INF("Initializing NPCM SAR ADC");

    /* Save device pointer */
    data->adc_dev = dev;

    /* Use input clock frequency from config */
    data->input_clk = config->input_clk_freq;

    /*
     * Initialize ADC following SARADC spec Section 4.16.7:
     * The ADC module must be initialized before started by setting START or ADCTTE bits.
     */

    /* Step 1: Basic Clock Division Factor - SCLKDIV field in ATCTL register */
    prescaler = DIV_ROUND_UP(data->input_clk, NPCM_SARADC_TARGET_CLK);
    if (prescaler > 0x40) {  /* SCLKDIV max is 6 bits = 64 */
        prescaler = 0x40;
    }
    if (prescaler < 1) {
        prescaler = 1;
    }

    /* Step 2: Channel Delay - DLY field in ATCTL register to "010" */
    inst->ATCTL = ((prescaler - 1) & 0x3F) |  /* SCLKDIV field [5:0] */
                  ((2 & 0x7) << 8);           /* DLY field [10:8] = 010 */

    LOG_INF("ATCTL configured: prescaler=%d (SCLKDIV=%d), delay=2", 
            prescaler, prescaler - 1);

    /* Step 3: Speed 1 Register, Word at offset 20h (ADCCNF2) */
    speed1_reg = (volatile uint16_t *)(config->base + 0x020);
    *speed1_reg = NPCX_ADCCNF2_DEFAULT_VAL;
    LOG_DBG("Speed 1 Register (offset 0x20) = 0x%04x", NPCX_ADCCNF2_DEFAULT_VAL);

    /* Step 4: Speed 2 Register, Word at offset 22h (GENDLY) */
    speed2_reg = (volatile uint16_t *)(config->base + 0x022);
    *speed2_reg = NPCX_GENDLY_DEFAULT_VAL;
    LOG_DBG("Speed 2 Register (offset 0x22) = 0x%04x", NPCX_GENDLY_DEFAULT_VAL);

    /* Step 5: Speed 3 Register, Word at offset 26h (MEAST) */
    speed3_reg = (volatile uint16_t *)(config->base + 0x026);
    *speed3_reg = NPCX_MEAST_DEFAULT_VAL;
    LOG_DBG("Speed 3 Register (offset 0x26) = 0x%04x", NPCX_MEAST_DEFAULT_VAL);

    /* Step 6: ADC Operation Mode - ADCMD field in ADCCNF register */
    /* Default to scan mode, will be set per operation */
    inst->ADCCNF = (NPCM_SARADC_SCAN_CONVERSION_MODE << 1);  /* ADCMD field [2:1] */

    /* Step 7: ADC Repetitive Mode - ADCRPTC bit in ADCCNF register */
    /* Will be set per operation as needed */

    /* Step 8: Setup the MFT16-2 timer - done by application if timer-triggered mode used */

    /* Step 9: ADC module Enable - ADCEN bit in ADCCNF register */
    /* Enabled per operation, not here during init */

    LOG_INF("SARADC initialized: base=0x%lx, APB1_CLK=%uHz, target=%uHz, actual=%uHz",
            config->base, data->input_clk, (unsigned int)NPCM_SARADC_TARGET_CLK,
            data->input_clk / prescaler);

    /* Configure pin control */
    if (config->pcfg) {
        int ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
        if (ret != 0) {
            LOG_ERR("Failed to configure pins: %d", ret);
            return ret;
        }
        LOG_DBG("Pin control configured");
    }

    /* Configure interrupt */
    if (config->irq_cfg_func) {
        config->irq_cfg_func();
        LOG_DBG("Interrupt configured");
    }

    /* Unlock ADC context */
    adc_context_unlock_unconditionally(&data->ctx);

    return 0;
}

/* Work queue for threshold comparator if enabled */
#if defined(CONFIG_ADC_CMP_NPCX_WORKQUEUE)
struct k_work_q adc_npcm_work_q;

static K_KERNEL_STACK_DEFINE(adc_npcm_work_q_stack,
                             CONFIG_ADC_CMP_NPCX_WORKQUEUE_STACK_SIZE);

static int adc_npcm_init_cmp_work_q(void)
{
    struct k_work_queue_config cfg = {
        .name = "adc_npcm_cmp",
        .no_yield = false,
    };

    k_work_queue_start(&adc_npcm_work_q,
                       adc_npcm_work_q_stack,
                       K_KERNEL_STACK_SIZEOF(adc_npcm_work_q_stack),
                       CONFIG_ADC_CMP_NPCX_WORKQUEUE_PRIORITY,
                       &cfg);

    work_q = &adc_npcm_work_q;

    return 0;
}

SYS_INIT(adc_npcm_init_cmp_work_q, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY);
#endif

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
    PINCTRL_DT_INST_DEFINE(n);						\
                                        \
    static const struct adc_driver_api adc_npcm_saradc_driver_api_##n = {	\
        .channel_setup	= adc_npcm_saradc_channel_setup,		\
        .read		= adc_npcm_saradc_read,				\
        IF_ENABLED(CONFIG_ADC_ASYNC, (.read_async = adc_npcm_saradc_read_async,))  \
        .ref_internal = DT_INST_PROP(n, vref_mv),                       \
    };									\
                                        \
    IF_ENABLED(CONFIG_ADC_CMP_NPCX,                                     \
    (static struct adc_npcm_threshold_data adc_npcm_threshold_data_##n;))  \
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
    static struct adc_npcm_saradc_data adc_npcm_saradc_data_##n = {	\
        ADC_CONTEXT_INIT_TIMER(adc_npcm_saradc_data_##n, ctx),         \
        ADC_CONTEXT_INIT_LOCK(adc_npcm_saradc_data_##n, ctx),          \
        ADC_CONTEXT_INIT_SYNC(adc_npcm_saradc_data_##n, ctx),          \
        IF_ENABLED(CONFIG_ADC_CMP_NPCX,                                 \
        (.threshold_data = &adc_npcm_threshold_data_##n,))             \
    };									\
                                        \
    DEVICE_DT_INST_DEFINE(n, adc_npcm_saradc_init,				\
                   NULL,						\
                   &adc_npcm_saradc_data_##n,			\
                   &adc_npcm_saradc_cfg_##n,			\
                   PRE_KERNEL_1, CONFIG_ADC_INIT_PRIORITY,		\
                   &adc_npcm_saradc_driver_api_##n);

DT_INST_FOREACH_STATUS_OKAY(NPCM_SARADC_INIT)
