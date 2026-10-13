/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcm_pwm

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/dt-bindings/clock/npcm_clock.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/kernel.h>
#include <soc.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pwm_npcm, LOG_LEVEL_ERR);

/* 16-bit period cycles/prescaler in NPCM PWM modules */
#define NPCM_PWM_MAX_PRESCALER     (1UL << (16))
#define NPCM_PWM_MAX_PERIOD_CYCLES (1UL << (16))

#define NPCM_PWM_LFCLK 32768

/* PWM clock sources */
#define NPCM_PWM_CLOCK_APB2_LFCLK 0
#define NPCM_PWM_CLOCK_FX         1
#define NPCM_PWM_CLOCK_FR         2
#define NPCM_PWM_CLOCK_RESERVED   3

/* PWM heart-beat mode selection */
#define NPCM_PWM_HBM_NORMAL   0
#define NPCM_PWM_HBM_VISUAL   1
#define NPCM_PWM_HBM_STANDARD 2

/* Device config */
struct pwm_npcm_config {
	/* pwm controller base address */
	struct pwm_reg *base;
	/* clock configuration */
	uint32_t clk_cfg;
	/* pinmux configuration */
	const struct pinctrl_dev_config *pcfg;
};

/* Driver data */
struct pwm_npcm_data {
	/* PWM cycles per second */
	uint32_t cycles_per_sec;
};

/* Define the pwm_hb_config structure */
struct pwm_hb_config {
	uint32_t prsc;
	uint32_t ctr;
	uint32_t n_step_rs;
	uint32_t dcr;
	uint32_t ctr_fl;
	uint32_t max_dc_fl;
	uint32_t n_step_fl;
	uint32_t ext_on;
	uint32_t ext_off;
	uint32_t hb_dc_ctl_field;
};

struct pwm_hb_config hb_standard[5] = {{// Heartbeat Standard Example: Type A
					.prsc = 0,
					.ctr = 745,
					.n_step_rs = 33,
					.dcr = 745,
					.ctr_fl = 745,
					.max_dc_fl = 745,
					.n_step_fl = 33,
					.ext_on = 0,
					.ext_off = 0,
					.hb_dc_ctl_field = NPCM_PWM_HBM_STANDARD},
				       {// Heartbeat Standard Example: Type B
					.prsc = 0,
					.ctr = 181,
					.n_step_rs = 45,
					.dcr = 181,
					.ctr_fl = 181,
					.max_dc_fl = 181,
					.n_step_fl = 45,
					.ext_on = 0,
					.ext_off = 0,
					.hb_dc_ctl_field = NPCM_PWM_HBM_STANDARD},
				       {// Heartbeat Standard Example: Type C
					.prsc = 0,
					.ctr = 69,
					.n_step_rs = 17,
					.dcr = 69,
					.ctr_fl = 69,
					.max_dc_fl = 69,
					.n_step_fl = 17,
					.ext_on = 0,
					.ext_off = 28,
					.hb_dc_ctl_field = NPCM_PWM_HBM_STANDARD},
				       {// Heartbeat Standard Example: Type D
					.prsc = 0,
					.ctr = 222,
					.n_step_rs = 56,
					.dcr = 222,
					.ctr_fl = 256,
					.max_dc_fl = 256,
					.n_step_fl = 64,
					.ext_on = 20,
					.ext_off = 80,
					.hb_dc_ctl_field = NPCM_PWM_HBM_STANDARD},
				       {// Heartbeat Standard Example: Type D1
					.prsc = 0,
					.ctr = 151,
					.n_step_rs = 28,
					.dcr = 112,
					.ctr_fl = 151,
					.max_dc_fl = 112,
					.n_step_fl = 28,
					.ext_on = 40,
					.ext_off = 80,
					.hb_dc_ctl_field = NPCM_PWM_HBM_STANDARD}};

struct pwm_hb_config hb_visual[5] = {{// Heartbeat Visual Example: Type A
				      .prsc = 0,
				      .ctr = 745,
				      .n_step_rs = 33,
				      .dcr = 745,
				      .ctr_fl = 745,
				      .max_dc_fl = 745,
				      .n_step_fl = 33,
				      .ext_on = 0,
				      .ext_off = 40,
				      .hb_dc_ctl_field = NPCM_PWM_HBM_VISUAL},
				     {// Heartbeat Visual Example: Type B
				      .prsc = 0,
				      .ctr = 438,
				      .n_step_rs = 19,
				      .dcr = 438,
				      .ctr_fl = 438,
				      .max_dc_fl = 438,
				      .n_step_fl = 19,
				      .ext_on = 0,
				      .ext_off = 0,
				      .hb_dc_ctl_field = NPCM_PWM_HBM_VISUAL},
				     {// Heartbeat Visual Example: Type C
				      .prsc = 0,
				      .ctr = 162,
				      .n_step_rs = 7,
				      .dcr = 162,
				      .ctr_fl = 162,
				      .max_dc_fl = 162,
				      .n_step_fl = 7,
				      .ext_on = 0,
				      .ext_off = 28,
				      .hb_dc_ctl_field = NPCM_PWM_HBM_VISUAL},
				     {// Heartbeat Visual Example: Type D
				      .prsc = 0,
				      .ctr = 530,
				      .n_step_rs = 23,
				      .dcr = 530,
				      .ctr_fl = 622,
				      .max_dc_fl = 622,
				      .n_step_fl = 27,
				      .ext_on = 20,
				      .ext_off = 80,
				      .hb_dc_ctl_field = NPCM_PWM_HBM_VISUAL},
				     {// Heartbeat Visual Example: Type D1
				      .prsc = 0,
				      .ctr = 369,
				      .n_step_rs = 12,
				      .dcr = 277,
				      .ctr_fl = 369,
				      .max_dc_fl = 277,
				      .n_step_fl = 12,
				      .ext_on = 40,
				      .ext_off = 80,
				      .hb_dc_ctl_field = NPCM_PWM_HBM_VISUAL}};

/* PWM local functions */
static int pwm_npcm_configure(const struct device *dev, uint32_t cycles)
{
	const struct pwm_npcm_config *config = dev->config;
	struct pwm_reg *inst = config->base;

	/* Disable PWM for module configuration first */
	inst->PWMCTL &= ~BIT(NPCM_PWMCTL_PWR);

	/* Set default PWM polarity to normal */
	inst->PWMCTL &= ~BIT(NPCM_PWMCTL_INVP);

	/* Turn off PWM heart-beat mode */
	SET_FIELD(inst->PWMCTL, NPCM_PWMCTL_HB_DC_CTL_FIELD, NPCM_PWM_HBM_NORMAL);

	/* Select APB CLK/LFCLK clock sources to PWM module by default */
	SET_FIELD(inst->PWMCTLEX, NPCM_PWMCTLEX_FCK_SEL_FIELD, NPCM_PWM_CLOCK_APB2_LFCLK);

	/* Select input clock source to LFCLK or APB2 */
	if (cycles == NPCM_PWM_LFCLK) {
		inst->PWMCTL |= BIT(NPCM_PWMCTL_CKSEL);
	} else {
		inst->PWMCTL &= ~BIT(NPCM_PWMCTL_CKSEL);
	}

	return 0;
}

/* Get the heartbeat configuration */
struct pwm_hb_config *get_heartbeat_config(int flags, struct pwm_hb_config *hb_array)
{
	if (flags & PWM_HB_TYPE_A) {
		return &hb_array[0];
	} else if (flags & PWM_HB_TYPE_B) {
		return &hb_array[1];
	} else if (flags & PWM_HB_TYPE_C) {
		return &hb_array[2];
	} else if (flags & PWM_HB_TYPE_D) {
		return &hb_array[3];
	} else if (flags & PWM_HB_TYPE_D1) {
		return &hb_array[4];
	} else {
		return &hb_array[0]; // Default case
	}
}

/* Set the heartbeat configuration */
void pwm_npcm_configure_heartbeat(struct pwm_reg *inst, struct pwm_hb_config *hb_cfg)
{
	inst->PWMCTL &= ~BIT(NPCM_PWMCTL_PWR);
	inst->PRSC = hb_cfg->prsc;
	inst->PWMCTL |= BIT(NPCM_PWMCTL_CKSEL);
	SET_FIELD(inst->PWMCTL, NPCM_PWMCTL_HB_DC_CTL_FIELD, hb_cfg->hb_dc_ctl_field);
	inst->PWMCTL |= BIT(NPCM_PWMCTL_HBNK_SEL);
	inst->CTR = hb_cfg->ctr;
	inst->N_STEP_RS = hb_cfg->n_step_rs;
	inst->DCR = hb_cfg->dcr;
	inst->CTR_FL = hb_cfg->ctr_fl;
	inst->MAX_DC_FL = hb_cfg->max_dc_fl;
	inst->N_STEP_FL = hb_cfg->n_step_fl;
	inst->EXT_ON = hb_cfg->ext_on;
	inst->EXT_OFF = hb_cfg->ext_off;
	inst->PWMCTL |= BIT(NPCM_PWMCTL_PWR);
}

/* PWM API functions */
static int pwm_npcm_set_cycles(const struct device *dev, uint32_t channel, uint32_t period_cycles,
			       uint32_t pulse_cycles, pwm_flags_t flags)
{
	/* Single channel for each PWM device */
	ARG_UNUSED(channel);

	const struct pwm_npcm_config *config = dev->config;
	struct pwm_npcm_data *const data = dev->data;
	struct pwm_reg *inst = config->base;

	uint32_t ctl, ctr, dcr, prsc;
	int prescaler, ret;

	/* Configure pin-mux for PWM device */
	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("PWM configure failed (%d)", ret);
		return ret;
	}

	/* Leave heartbeat mode left over from a previous call */
	if (!(flags & (PWMCTL_HB_STANDARD | PWMCTL_HB_VISUAL)) &&
	    ((inst->PWMCTL & BIT(NPCM_PWMCTL_HBNK_SEL)) ||
	     GET_FIELD(inst->PWMCTL, NPCM_PWMCTL_HB_DC_CTL_FIELD) != NPCM_PWM_HBM_NORMAL)) {
		inst->PWMCTL &= ~BIT(NPCM_PWMCTL_PWR);
		inst->PWMCTL &= ~BIT(NPCM_PWMCTL_HBNK_SEL);
		SET_FIELD(inst->PWMCTL, NPCM_PWMCTL_HB_DC_CTL_FIELD, NPCM_PWM_HBM_NORMAL);
		if (data->cycles_per_sec == NPCM_PWM_LFCLK) {
			inst->PWMCTL |= BIT(NPCM_PWMCTL_CKSEL);
		} else {
			inst->PWMCTL &= ~BIT(NPCM_PWMCTL_CKSEL);
		}
	}

	ctl = inst->PWMCTL | BIT(NPCM_PWMCTL_PWR);

	/* Set PWM polarity */
	if (flags & PWM_POLARITY_INVERTED) {
		ctl |= BIT(NPCM_PWMCTL_INVP);
	} else {
		ctl &= ~BIT(NPCM_PWMCTL_INVP);
	}

	/* Set PWM operation synchronization */
	if (flags & PWMCTL_SYNC) {
		ctl |= BIT(NPCM_PWMCTL_SYNC);
	} else {
		ctl &= ~BIT(NPCM_PWMCTL_SYNC);
	}

	/* If pulse_cycles is 0, disable PWM and return */
	if (pulse_cycles == 0) {
		ctl &= ~BIT(NPCM_PWMCTL_PWR);
		inst->PWMCTL = ctl;
		return 0;
	}

	/* Calculate prescaler */
	prescaler = DIV_ROUND_UP(period_cycles, NPCM_PWM_MAX_PERIOD_CYCLES);
	if (prescaler > NPCM_PWM_MAX_PRESCALER) {
		return -EINVAL;
	}

	prsc = prescaler - 1;
	ctr = (period_cycles / prescaler) - 1;
	dcr = (pulse_cycles / prescaler) - 1;

	LOG_DBG("freq %d, pre %d, period %d, pulse %d", data->cycles_per_sec / period_cycles, prsc,
		ctr, dcr);

	/* Reconfigure only if necessary */
	if (inst->PWMCTL != ctl || inst->PRSC != prsc || inst->CTR != ctr ||
	    (flags & PWMCTL_HB_STANDARD) || (flags & PWMCTL_HB_VISUAL) || (flags & PWMCTL_SYNC)) {

		/* Disable PWM before configuring */
		inst->PWMCTL &= ~BIT(NPCM_PWMCTL_PWR);

		inst->PRSC = prsc;
		inst->CTR = ctr;
		inst->DCR = dcr;

		/* Enable PWM */
		inst->PWMCTL = ctl;

		/* Configure heartbeat modes */
		if (flags & PWMCTL_HB_STANDARD) {
			pwm_npcm_configure_heartbeat(inst,
						     get_heartbeat_config(flags, hb_standard));
		} else if (flags & PWMCTL_HB_VISUAL) {
			pwm_npcm_configure_heartbeat(inst, get_heartbeat_config(flags, hb_visual));
		}

		return 0;
	}

	inst->DCR = dcr;

	return 0;
}

static int pwm_npcm_get_cycles_per_sec(const struct device *dev, uint32_t channel, uint64_t *cycles)
{
	/* Single channel for each pwm device */
	ARG_UNUSED(channel);
	struct pwm_npcm_data *const data = dev->data;

	*cycles = data->cycles_per_sec;
	return 0;
}

/* PWM driver registration */
static const struct pwm_driver_api pwm_npcm_driver_api = {
	.set_cycles = pwm_npcm_set_cycles, .get_cycles_per_sec = pwm_npcm_get_cycles_per_sec};

static int pwm_npcm_init(const struct device *dev)
{
	const struct pwm_npcm_config *const config = dev->config;
	struct pwm_npcm_data *const data = dev->data;
	struct pwm_reg *const inst = config->base;
	const struct device *const clk_dev = DEVICE_DT_GET(DT_NODELABEL(pcc));
	int ret;

	/*
	 * NPCM PWM module mixes byte and word registers together. Make sure
	 * word reg access via structure won't break into two byte reg accesses
	 * unexpectedly by toolchains options or attributes. If so, stall here.
	 */
	NPCM_REG_WORD_ACCESS_CHECK(inst->PRSC, 0xA55A);

	if (!device_is_ready(clk_dev)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	/* Turn on device clock first and get source clock freq. */
	ret = clock_control_on(clk_dev, (clock_control_subsys_t)config->clk_cfg);
	if (ret < 0) {
		LOG_ERR("Turn on PWM clock fail %d", ret);
		return ret;
	}

	ret = clock_control_get_rate(clk_dev, (clock_control_subsys_t)config->clk_cfg,
				     &data->cycles_per_sec);
	if (ret < 0) {
		LOG_ERR("Get PWM clock rate error %d", ret);
		return ret;
	}

	/* Bus only could be APB2 or Low frequency clock */
	if (((config->clk_cfg >> NPCM_CLOCK_BUS_OFFSET) & NPCM_CLOCK_BUS_MASK) !=
	    NPCM_CLOCK_BUS_APB2) {
		if ((data->cycles_per_sec != NPCM_PWM_LFCLK)) {
			LOG_ERR("PWM only support source LF or APB2");
			return -EINVAL;
		}
	}

	/* Configure PWM device initially */
	ret = pwm_npcm_configure(dev, data->cycles_per_sec);
	if (ret < 0) {
		LOG_ERR("PWM configure failed (%d)", ret);
		return ret;
	}

	/* Configure pin-mux for PWM device */
	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("PWM pinctrl setup failed (%d)", ret);
		return ret;
	}

	return 0;
}

#define NPCM_PWM_INIT(inst)                                                                        \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
                                                                                                   \
	static const struct pwm_npcm_config pwm_npcm_cfg_##inst = {                                \
		.base = (struct pwm_reg *)DT_INST_REG_ADDR(inst),                                  \
		.clk_cfg = DT_INST_PHA(inst, clocks, clk_cfg),                                     \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
	};                                                                                         \
                                                                                                   \
	static struct pwm_npcm_data pwm_npcm_data_##inst;                                          \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &pwm_npcm_init, NULL, &pwm_npcm_data_##inst,                   \
			      &pwm_npcm_cfg_##inst, PRE_KERNEL_1, CONFIG_PWM_INIT_PRIORITY,        \
			      &pwm_npcm_driver_api);

DT_INST_FOREACH_STATUS_OKAY(NPCM_PWM_INIT)
