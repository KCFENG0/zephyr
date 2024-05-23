/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <assert.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/pinctrl/npcm-pinctrl.h>
#include <soc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pinctrl_npcm, LOG_LEVEL_ERR);

/* Driver config */
struct npcm_pinctrl_config {
	/* Device base address used for pinctrl driver */
	uintptr_t base_scfg;
	uintptr_t base_glue;
};

/*
 * Get io list which default functionality are not IOs. Then switch them to
 * GPIO in pin-mux init function.
 *
 * def-gpio {
 *     def-gpio-list = <&alt5c_psin
 *                      &alt5c_psout
 *                      ...>;
 * };
 */
#define NPCM_NO_GPIO_ALT_ITEM(node_id, prop, idx)                                                  \
	{                                                                                          \
		.id = DT_PROP_BY_IDX(DT_PHANDLE_BY_IDX(node_id, prop, idx), pinmux, 0),            \
		.inverted = DT_PROP_BY_IDX(DT_PHANDLE_BY_IDX(node_id, prop, idx), pinmux, 1),      \
	},

static const struct npcm_periph_pinmux def_alts[] = {DT_FOREACH_PROP_ELEM(
	DT_INST(0, nuvoton_npcm_pinctrl_def), def_gpio_list, NPCM_NO_GPIO_ALT_ITEM)};

static const struct npcm_pinctrl_config npcm_pinctrl_cfg = {
	.base_scfg = DT_REG_ADDR_BY_NAME(DT_NODELABEL(scfg), scfg),
	.base_glue = DT_REG_ADDR_BY_NAME(DT_NODELABEL(scfg), glue),
};

/* SCFG multi-registers */
#define NPCM_SCFG_DEV_CTL(base, n)     (*(volatile uint8_t *)(base + (n)))
#define NPCM_SCFG_DEVALT_OFFSET(n)     (((n) >> NPCM_PINCTRL_GROUP_SHIFT) & NPCM_PINCTRL_GROUP_MASK)
#define NPCM_SCFG_DEVALT(base, n)      (*(volatile uint8_t *)(base + NPCM_SCFG_DEVALT_OFFSET(n)))
#define NPCM_SCFG_DEVALT_BIT_OFFSET(n) ((n) & NPCM_PINCTRL_GROUP_BIT_MASK)
#define NPCM_SCFG_PUPD_EN_OFFSET(n)    (((n) >> NPCM_PINCTRL_PUPD_SHIFT) & NPCM_PINCTRL_PUPD_MASK)
#define NPCM_SCFG_PUPD_EN(base, n)     (*(volatile uint8_t *)(base + NPCM_SCFG_PUPD_EN_OFFSET(n)))
#define NPCM_SCFG_PUPD_BIT_OFFSET(n)   ((n) & NPCM_PINCTRL_PUPD_BIT_MASK)

/* NPCM register access helper */
#define NPCM_SCFG_SET_FIELD(reg, offset, size, value)                                              \
	((reg) = ((reg) & (~(((1 << (size)) - 1) << (offset)))) | ((value) << (offset)))

static void npcm_periph_pinmux_configure(const struct npcm_periph_pinmux *alt, bool is_alternate)
{
	const uintptr_t scfg_base = npcm_pinctrl_cfg.base_scfg;
	uint8_t alt_mask = BIT(NPCM_SCFG_DEVALT_BIT_OFFSET(alt->id));

	/*
	 * is_alternate == 0 means select GPIO, otherwise Alternate Func.
	 * inverted == 0:
	 *    Set devalt bit to select Alternate Func.
	 * inverted == 1:
	 *    Clear devalt bit to select Alternate Func.
	 */
	if (is_alternate != alt->inverted) {
		NPCM_SCFG_DEVALT(scfg_base, alt->id) |= alt_mask;
	} else {
		NPCM_SCFG_DEVALT(scfg_base, alt->id) &= ~alt_mask;
	}
}

static void npcm_periph_pupd_configure(const struct npcm_periph_pupd *pupd,
				       enum npcm_io_bias_type type)
{
	const uintptr_t scfg_base = npcm_pinctrl_cfg.base_scfg;
	uint8_t pupd_mask = BIT(NPCM_SCFG_PUPD_BIT_OFFSET(pupd->id));

	if (type == NPCM_BIAS_TYPE_NONE) {
		NPCM_SCFG_PUPD_EN(scfg_base, pupd->id) &= ~pupd_mask;
	} else {
		NPCM_SCFG_PUPD_EN(scfg_base, pupd->id) |= pupd_mask;
	}
}

static void npcm_device_control_configure(const pinctrl_soc_pin_t *pin)
{
	const struct npcm_dev_ctl *ctrl = (const struct npcm_dev_ctl *)&pin->cfg.dev_ctl;
	const uintptr_t scfg_base = npcm_pinctrl_cfg.base_scfg;

	NPCM_SCFG_SET_FIELD(NPCM_SCFG_DEV_CTL(scfg_base, ctrl->offest), ctrl->field_offset,
			    ctrl->field_size, ctrl->field_value);
}

/* Pinctrl API implementation */
int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	ARG_UNUSED(reg);
	uint8_t i;

	/* Configure all peripheral devices' properties here. */
	for (i = 0; i < pin_cnt; i++) {
		if (pins[i].flags.type == NPCM_PINCTRL_TYPE_PERIPH_PINMUX) {
			/* Configure peripheral device's pinmux functionality */
			npcm_periph_pinmux_configure(&pins[i].cfg.pinmux,
						     !pins[i].flags.pinmux_gpio);
		} else if (pins[i].flags.type == NPCM_PINCTRL_TYPE_PERIPH_PUPD) {
			/* Configure peripheral device's internal PU/PD */
			npcm_periph_pupd_configure(&pins[i].cfg.pupd, pins[i].flags.io_bias_type);
		} else if (pins[i].flags.type == NPCM_PINCTRL_TYPE_DEVICE_CTRL) {
			/* Configure device's io characteristics */
			npcm_device_control_configure(&pins[i]);
		} else {
			return -ENOTSUP;
		}
	}

	return 0;
}

/* Pin-control init local functions */
static void npcm_pinctrl_alt_sel(const struct npcm_periph_pinmux *alt, int alt_func)
{
	const uint32_t scfg_base = npcm_pinctrl_cfg.base_scfg;
	uint8_t alt_mask = BIT(NPCM_SCFG_DEVALT_BIT_OFFSET(alt->id));

	/*
	 * alt_fun == 0 means select GPIO, otherwise Alternate Func.
	 * inverted == 0:
	 *    Set devalt bit to select Alternate Func.
	 * inverted == 1:
	 *    Clear devalt bit to select Alternate Func.
	 */
	if (!!alt_func != !!alt->inverted) {
		NPCM_SCFG_DEVALT(scfg_base, alt->id) |= alt_mask;
	} else {
		NPCM_SCFG_DEVALT(scfg_base, alt->id) &= ~alt_mask;
	}
}

/* Pin-control driver registration */
static int npcm_pinctrl_init(void)
{
	int i;

	/* Change all pads whose default functionality isn't IO to GPIO */
	for (i = 0; i < ARRAY_SIZE(def_alts); i++) {
		npcm_pinctrl_alt_sel(&def_alts[i], 0);
	}

	return 0;
}

SYS_INIT(npcm_pinctrl_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
