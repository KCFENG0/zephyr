/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcm_miwu

/**
 * @file
 * @brief Nuvoton NPCM MIWU driver
 *
 * The device Multi-Input Wake-Up Unit (MIWU) supports the Nuvoton embedded
 * controller (EC) to exit 'Sleep' or 'Deep Sleep' power state.
 * Also, it provides signal conditioning such as
 * 'Level' and 'Edge' trigger type and grouping of external interrupt sources
 * of NVIC. The NPCM series has three identical MIWU modules: MIWU0, MIWU1,
 * MIWU2. Together, they support a total of 143 internal and/or external
 * wake-up input (WUI) sources.
 *
 * This driver uses device tree files to present the relationship bewteen
 * MIWU and the other devices in npcm target.
 * it include:
 *  1. npcm-miwus-wui-map.dtsi: it presents relationship between wake-up inputs
 *     (WUI) and its source device such as gpio, timer, eSPI VWs and so on.
 *  2. npcm-miwus-int-map.dtsi: it presents relationship between MIWU group
 *     and NVIC interrupt in npcm target. Basically, it's a 1-to-1 mapping.
 *     There is a group which has 2 interrupts as an exception.
 *
 * INCLUDE FILES: soc_miwu.h
 *
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <soc.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/irq_nextlevel.h>
#include <zephyr/drivers/gpio.h>

#include "soc_miwu.h"
#include "soc_gpio.h"

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(intc_npcm_miwu, LOG_LEVEL_ERR);

/* MIWU module instances */
#define NPCM_MIWU_DEV(inst) DEVICE_DT_INST_GET(inst),

static const struct device *miwu_devs[] = {
	DT_INST_FOREACH_STATUS_OKAY(NPCM_MIWU_DEV)
};

BUILD_ASSERT(ARRAY_SIZE(miwu_devs) == NPCM_MIWU_TABLE_COUNT,
	     "Size of miwu_devs array must equal to NPCM_MIWU_TABLE_COUNT");

/* Driver config */
struct intc_miwu_config {
	/* miwu controller base address */
	uintptr_t base;
	/* index of miwu controller */
	uint8_t index;
};

/* Driver data */
struct intc_miwu_data {
	/* Callback functions list for each MIWU group */
	sys_slist_t cb_list_grp[8];
};

BUILD_ASSERT(sizeof(struct miwu_io_params) == sizeof(gpio_port_pins_t),
	     "Size of struct miwu_io_params must equal to struct gpio_port_pins_t");

BUILD_ASSERT(offsetof(struct miwu_callback, io_cb.params) +
	sizeof(struct miwu_io_params) == sizeof(struct gpio_callback),
	"Failed in size check of miwu_callback and gpio_callback structures!");

BUILD_ASSERT(offsetof(struct miwu_callback, io_cb.params.cb_type) ==
	offsetof(struct miwu_callback, dev_cb.params.cb_type),
	"Failed in offset check of cb_type field of miwu_callback structure");

/* MIWU local functions */
static void intc_miwu_dispatch_isr(sys_slist_t *cb_list, uint8_t mask)
{
	struct miwu_callback *cb, *tmp;

	SYS_SLIST_FOR_EACH_CONTAINER_SAFE(cb_list, cb, tmp, node) {

		if (cb->io_cb.params.cb_type == NPCM_MIWU_CALLBACK_GPIO) {
			if (BIT(cb->io_cb.params.wui.bit) & mask) {
				__ASSERT(cb->io_cb.handler, "No GPIO callback handler!");
				cb->io_cb.handler(
					npcm_get_gpio_dev(cb->io_cb.params.gpio_port),
					(struct gpio_callback *)cb,
					cb->io_cb.params.pin_mask);
			}
		} else {
			if (BIT(cb->dev_cb.params.wui.bit) & mask) {
				__ASSERT(cb->dev_cb.handler, "No device callback handler!");

				cb->dev_cb.handler(cb->dev_cb.params.source,
						   &cb->dev_cb.params.wui);
			}
		}
	}
}

static void intc_miwu_isr_pri(int wui_table, int wui_group)
{
	const struct intc_miwu_config *config = miwu_devs[wui_table]->config;
	struct intc_miwu_data *data = miwu_devs[wui_table]->data;
	const uint32_t base = config->base;
	uint8_t mask = NPCM_WKPND(base, wui_group) & NPCM_WKEN(base, wui_group);

	/* Clear pending bits before dispatch ISR */
	if (mask) {
		NPCM_WKPCL(base, wui_group) = mask;
	}

	/* Dispatch registered gpio isrs */
	intc_miwu_dispatch_isr(&data->cb_list_grp[wui_group], mask);
}

/* Platform specific MIWU functions */
void npcm_miwu_irq_enable(const struct npcm_wui *wui)
{
	const struct intc_miwu_config *config = miwu_devs[wui->table]->config;
	const uint32_t base = config->base;

	NPCM_WKEN(base, wui->group) |= BIT(wui->bit);
}

void npcm_miwu_irq_disable(const struct npcm_wui *wui)
{
	const struct intc_miwu_config *config = miwu_devs[wui->table]->config;
	const uint32_t base = config->base;

	NPCM_WKEN(base, wui->group) &= ~BIT(wui->bit);
}

void npcm_miwu_io_enable(const struct npcm_wui *wui)
{
	const struct intc_miwu_config *config = miwu_devs[wui->table]->config;
	const uint32_t base = config->base;

	NPCM_WKINEN(base, wui->group) |= BIT(wui->bit);
}

void npcm_miwu_io_disable(const struct npcm_wui *wui)
{
	const struct intc_miwu_config *config = miwu_devs[wui->table]->config;
	const uint32_t base = config->base;

	NPCM_WKINEN(base, wui->group) &= ~BIT(wui->bit);
}

bool npcm_miwu_irq_get_state(const struct npcm_wui *wui)
{
	const struct intc_miwu_config *config = miwu_devs[wui->table]->config;
	const uint32_t base = config->base;

	return IS_BIT_SET(NPCM_WKEN(base, wui->group), wui->bit);
}

bool npcm_miwu_irq_get_and_clear_pending(const struct npcm_wui *wui)
{
	const struct intc_miwu_config *config = miwu_devs[wui->table]->config;
	const uint32_t base = config->base;
	bool pending = IS_BIT_SET(NPCM_WKPND(base, wui->group), wui->bit);

	if (pending) {
		NPCM_WKPCL(base, wui->group) = BIT(wui->bit);
	}

	return pending;
}

int npcm_miwu_interrupt_configure(const struct npcm_wui *wui,
				     enum miwu_int_mode mode, enum miwu_int_trig trig)
{
	const struct intc_miwu_config *config = miwu_devs[wui->table]->config;
	const uint32_t base = config->base;
	uint8_t pmask = BIT(wui->bit);

	/* Disable interrupt of wake-up input source before configuring it */
	npcm_miwu_irq_disable(wui);

	/* Handle interrupt for level trigger */
	if (mode == NPCM_MIWU_MODE_LEVEL) {
		/* Set detection mode to level */
		NPCM_WKMOD(base, wui->group) |= pmask;
		switch (trig) {
		/* Enable interrupting on level high */
		case NPCM_MIWU_TRIG_HIGH:
			NPCM_WKEDG(base, wui->group) &= ~pmask;
			break;
		/* Enable interrupting on level low */
		case NPCM_MIWU_TRIG_LOW:
			NPCM_WKEDG(base, wui->group) |= pmask;
			break;
		default:
			return -EINVAL;
		}
		/* Handle interrupt for edge trigger */
	} else {
		/* Set detection mode to edge */
		NPCM_WKMOD(base, wui->group) &= ~pmask;
		switch (trig) {
		/* Handle interrupting on falling edge */
		case NPCM_MIWU_TRIG_LOW:
			NPCM_WKAEDG(base, wui->group) &= ~pmask;
			NPCM_WKEDG(base, wui->group) |= pmask;
			break;
		/* Handle interrupting on rising edge */
		case NPCM_MIWU_TRIG_HIGH:
			NPCM_WKAEDG(base, wui->group) &= ~pmask;
			NPCM_WKEDG(base, wui->group) &= ~pmask;
			break;
		/* Handle interrupting on both edges */
		case NPCM_MIWU_TRIG_BOTH:
			/* Enable any edge */
			NPCM_WKAEDG(base, wui->group) |= pmask;
			break;
		default:
			return -EINVAL;
		}
	}

	/* Enable wake-up input sources */
	NPCM_WKINEN(base, wui->group) |= pmask;

	/*
	 * Clear pending bit since it might be set if WKINEN bit is
	 * changed.
	 */
	NPCM_WKPCL(base, wui->group) |= pmask;

	return 0;
}

void npcm_miwu_init_gpio_callback(struct miwu_callback *callback,
				     const struct npcm_wui *io_wui, int port)
{
	/* Initialize WUI and GPIO settings in unused bits field */
	callback->io_cb.params.wui.table = io_wui->table;
	callback->io_cb.params.wui.bit = io_wui->bit;
	callback->io_cb.params.gpio_port = port;
	callback->io_cb.params.cb_type = NPCM_MIWU_CALLBACK_GPIO;
	callback->io_cb.params.wui.group = io_wui->group;
}

void npcm_miwu_init_dev_callback(struct miwu_callback *callback,
				    const struct npcm_wui *dev_wui,
				    miwu_dev_callback_handler_t handler,
				    const struct device *source)
{
	/* Initialize WUI and input device settings */
	callback->dev_cb.params.wui.table = dev_wui->table;
	callback->dev_cb.params.wui.group = dev_wui->group;
	callback->dev_cb.params.wui.bit = dev_wui->bit;
	callback->dev_cb.params.source = source;
	callback->dev_cb.params.cb_type = NPCM_MIWU_CALLBACK_DEV;
	callback->dev_cb.handler = handler;
}

int npcm_miwu_manage_callback(struct miwu_callback *cb, bool set)
{
	struct npcm_wui *wui;
	struct intc_miwu_data *data;
	sys_slist_t *cb_list;

	if (cb->io_cb.params.cb_type == NPCM_MIWU_CALLBACK_GPIO) {
		wui = &cb->io_cb.params.wui;
	} else {
		wui = &cb->dev_cb.params.wui;
	}

	data = miwu_devs[wui->table]->data;
	cb_list = &data->cb_list_grp[wui->group];
	if (!sys_slist_is_empty(cb_list)) {
		if (!sys_slist_find_and_remove(cb_list, &cb->node)) {
			if (!set) {
				return -EINVAL;
			}
		}
	}

	if (set) {
		sys_slist_prepend(cb_list, &cb->node);
	}

	return 0;
}

/* MIWU driver registration */
#define NPCM_MIWU_ISR_FUNC(index) _CONCAT(intc_miwu_isr, index)
#define NPCM_MIWU_INIT_FUNC(inst) _CONCAT(intc_miwu_init, inst)
#define NPCM_MIWU_INIT_FUNC_DECL(inst) \
	static int intc_miwu_init##inst(const struct device *dev)

/* MIWU ISR implementation */
#define NPCM_MIWU_ISR_FUNC_IMPL(inst)						\
	static void intc_miwu_isr##inst(void *arg)				\
	{									\
		uint8_t grp_mask = (uint32_t)arg;				\
		int group = 0;							\
										\
		/* Check all MIWU groups belong to the same irq */		\
		do {								\
			if (grp_mask & 0x01)					\
				intc_miwu_isr_pri(inst, group);			\
			group++;						\
			grp_mask = grp_mask >> 1;				\
										\
		} while (grp_mask != 0);					\
	}

/* MIWU init function implementation */
#define NPCM_MIWU_INIT_FUNC_IMPL(inst)						\
	static int intc_miwu_init##inst(const struct device *dev)		\
	{									\
		int i;								\
		const struct intc_miwu_config *config = dev->config;		\
		const uint32_t base = config->base;				\
										\
		/* Clear all MIWUs' pending and enable bits of MIWU device */	\
		for (i = 0; i < NPCM_MIWU_GROUP_COUNT; i++) {			\
			NPCM_WKEN(base, i) = 0;					\
			NPCM_WKPCL(base, i) = 0xFF;				\
		}								\
										\
		/* Config IRQ and MWIU group directly */			\
		DT_FOREACH_CHILD(NPCM_DT_NODE_FROM_MIWU_MAP(inst),		\
				 NPCM_DT_MIWU_IRQ_CONNECT_IMPL_CHILD_FUNC)	\
		return 0;							\
	}									\

#define NPCM_MIWU_INIT(inst)							\
	NPCM_MIWU_INIT_FUNC_DECL(inst);						\
										\
	static const struct intc_miwu_config miwu_config_##inst = {		\
		.base = DT_REG_ADDR(DT_NODELABEL(miwu##inst)),			\
		.index = DT_PROP(DT_NODELABEL(miwu##inst), index),		\
	};									\
	struct intc_miwu_data miwu_data_##inst;					\
										\
	DEVICE_DT_INST_DEFINE(inst,						\
			      NPCM_MIWU_INIT_FUNC(inst),			\
			      NULL,						\
			      &miwu_data_##inst, &miwu_config_##inst,		\
			      PRE_KERNEL_1,					\
			      CONFIG_INTC_INIT_PRIORITY, NULL);			\
										\
	NPCM_MIWU_ISR_FUNC_IMPL(inst)						\
										\
	NPCM_MIWU_INIT_FUNC_IMPL(inst)

DT_INST_FOREACH_STATUS_OKAY(NPCM_MIWU_INIT)
