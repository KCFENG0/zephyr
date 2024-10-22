/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcm_gpio

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_npcm.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/pinctrl/npcm-pinctrl.h>
#include <soc.h>

#include <zephyr/drivers/gpio/gpio_utils.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gpio_npcm, LOG_LEVEL_ERR);

/* GPIO module instances */
#define NPCM_GPIO_DEV(inst) DEVICE_DT_INST_GET(inst),
static const struct device *gpio_devs[] = {DT_INST_FOREACH_STATUS_OKAY(NPCM_GPIO_DEV)};

/* SCFG multi-registers */
#define NPCM_SCFG_OFFSET(n)     (((n) >> NPCM_PINCTRL_SHIFT) & NPCM_PINCTRL_MASK)
#define NPCM_SCFG(base, n)      (*(volatile uint8_t *)(base + NPCM_SCFG_OFFSET(n)))
#define NPCM_SCFG_BIT_OFFSET(n) ((n) & NPCM_PINCTRL_BIT_MASK)

/* SCFG base */
struct npcm_scfg_config {
	/* scfg device base address */
	uintptr_t base_scfg;
};

static const struct npcm_scfg_config npcm_scfg_cfg = {
	.base_scfg = DT_REG_ADDR_BY_NAME(DT_PARENT(DT_DRV_INST(0)), scfg),
};

/* Driver convenience defines */
#define HAL_INSTANCE(dev)                                                                          \
	((struct gpio_reg *)((const struct gpio_npcm_config *)(dev)->config)->base)
#define PORT_INSTANCE(dev) (((const struct gpio_npcm_config *)(dev)->config)->port)

#define HAL_SCFG_INST() (struct scfg_reg *)(npcm_scfg_cfg.base_scfg)

/* Platform specific GPIO functions */
const struct device *npcm_get_gpio_dev(int port)
{
	uint8_t i;

	for (i = 0; i < ARRAY_SIZE(gpio_devs); i++) {
		if (PORT_INSTANCE(gpio_devs[i]) == port) {
			return gpio_devs[i];
		}
	}

	return NULL;
}

static int npcm_gpio_set_pincfg(const struct device *dev, gpio_pin_t pin)
{
	int port = PORT_INSTANCE(dev);
	pinctrl_soc_pin_t pin_cfg;

	/* Check if port and pin are valid */
	if ((port >= NPCM_GPIO_PORT_NUM) || (pin >= NPCM_GPIO_PORT_PIN_NUM)) {
		return -EINVAL;
	}

	pin_cfg.flags.type = NPCM_PINCTRL_TYPE_PERIPH_PINMUX;
	pin_cfg.cfg.pinmux.id = NPCM_PINCTRL_NUM_IDX(port, pin);
	pin_cfg.cfg.pinmux.alt_group = 0; /* Default group to GPIO is 0 */

	/* Set GPIO by pinctrl api */
	if (pinctrl_configure_pins(&pin_cfg, 1, PINCTRL_REG_NONE) < 0) {
		return -ENOTSUP;
	}

	return 0;
}

/* GPIO api functions */
static int gpio_npcm_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	struct gpio_reg *const inst = HAL_INSTANCE(dev);
	struct scfg_reg *inst_scfg = HAL_SCFG_INST();
	uint32_t mask = BIT(pin);

	/* Check unsupported "open source" mode is set */
	if (((flags & GPIO_SINGLE_ENDED) != 0) && ((flags & GPIO_LINE_OPEN_DRAIN) == 0)) {
		return -ENOTSUP;
	}

	/* Set pinmux */
	if (npcm_gpio_set_pincfg(dev, pin) < 0) {
		return -ENOTSUP;
	}

	/*
	 * Configure pin as input.
	 * Output is configured only after setting all other attributes.
	 */
	if ((flags & GPIO_OUTPUT) == 0) {
		inst->PDIR &= ~mask;
	}

	/* Select open drain 0:push-pull 1:open-drain */
	if ((flags & GPIO_OPEN_DRAIN) != 0) {
		inst->PTYPE |= mask;
	} else {
		inst->PTYPE &= ~mask;
	}

	/* Open drain output mode want to enable internal pull up */
	if ((flags & GPIO_OPEN_DRAIN) && (flags & GPIO_OUTPUT)) {
		if ((flags & GPIO_PULL_UP)) {
			inst_scfg->DEVALTCX |= BIT(NPCM_DEVALTCX_GPIO_PULL_EN);
		} else {
			inst_scfg->DEVALTCX &= ~BIT(NPCM_DEVALTCX_GPIO_PULL_EN);
		}
	}

	/* Enable and select pull-up/down of GPIO 0:pull-up 1:pull-down */
	if ((flags & GPIO_PULL_UP) != 0) {
		inst->PPUD &= ~mask;
		inst->PPULL |= mask;
	} else if ((flags & GPIO_PULL_DOWN) != 0) {
		inst->PPUD |= mask;
		inst->PPULL |= mask;
	} else {
		/* disable pull down/up */
		inst->PPULL &= ~mask;
	}

	/* Set level 0:low 1:high */
	if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
		inst->PDOUT |= mask;
	} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
		inst->PDOUT &= ~mask;
	}

	/* Configure pin as output, if requested 0:input 1:output */
	if ((flags & GPIO_OUTPUT) != 0) {
		inst->PDIR |= mask;
	}

	return 0;
}

#ifdef CONFIG_GPIO_GET_CONFIG
static int gpio_npcm_pin_get_config(const struct device *port, gpio_pin_t pin,
				    gpio_flags_t *out_flags)
{
	struct gpio_reg *const inst = HAL_INSTANCE(port);
	uint32_t mask = BIT(pin);
	gpio_flags_t flags = 0;

	/* 0:input 1:output */
	if (inst->PDIR & mask) {
		flags |= GPIO_OUTPUT;

		/* 0:push-pull 1:open-drain */
		if (inst->PTYPE & mask) {
			flags |= GPIO_OPEN_DRAIN;
		}

		/* 0:low 1:high */
		if (inst->PDOUT & mask) {
			flags |= GPIO_OUTPUT_HIGH;
		} else {
			flags |= GPIO_OUTPUT_LOW;
		}
	} else {
		flags |= GPIO_INPUT;

		/* 0:disabled 1:enabled pull */
		if (inst->PPULL & mask) {
			/* 0:pull-up 1:pull-down */
			if (inst->PPUD & mask) {
				flags |= GPIO_PULL_DOWN;
			} else {
				flags |= GPIO_PULL_UP;
			}
		}
	}

	*out_flags = flags;

	return 0;
}
#endif

static int gpio_npcm_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
	struct gpio_reg *const inst = HAL_INSTANCE(dev);

	/* Get raw bits of GPIO input registers */
	*value = inst->PDIN;

	return 0;
}

static int gpio_npcm_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
					 gpio_port_value_t value)
{
	struct gpio_reg *const inst = HAL_INSTANCE(dev);
	uint8_t out = inst->PDOUT;

	inst->PDOUT = ((out & ~mask) | (value & mask));

	return 0;
}

static int gpio_npcm_port_set_bits_raw(const struct device *dev, gpio_port_value_t mask)
{
	struct gpio_reg *const inst = HAL_INSTANCE(dev);

	/* Set raw bits of GPIO output registers */
	inst->PDOUT |= mask;

	return 0;
}

static int gpio_npcm_port_clear_bits_raw(const struct device *dev, gpio_port_value_t mask)
{
	struct gpio_reg *const inst = HAL_INSTANCE(dev);

	/* Clear raw bits of GPIO output registers */
	inst->PDOUT &= ~mask;

	return 0;
}

static int gpio_npcm_port_toggle_bits(const struct device *dev, gpio_port_value_t mask)
{
	struct gpio_reg *const inst = HAL_INSTANCE(dev);

	/* Toggle raw bits of GPIO output registers */
	inst->PDOUT ^= mask;

	return 0;
}

/* GPIO driver registration */
static const struct gpio_driver_api gpio_npcm_driver = {
	.pin_configure = gpio_npcm_pin_configure,
#ifdef CONFIG_GPIO_GET_CONFIG
	.pin_get_config = gpio_npcm_pin_get_config,
#endif
	.port_get_raw = gpio_npcm_port_get_raw,
	.port_set_masked_raw = gpio_npcm_port_set_masked_raw,
	.port_set_bits_raw = gpio_npcm_port_set_bits_raw,
	.port_clear_bits_raw = gpio_npcm_port_clear_bits_raw,
	.port_toggle_bits = gpio_npcm_port_toggle_bits,
};

int gpio_npcm_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

#define NPCM_GPIO_DEVICE_INIT(inst)                                                                \
	static const struct gpio_npcm_config gpio_npcm_cfg_##inst = {                              \
		.common =                                                                          \
			{                                                                          \
				.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(inst),            \
			},                                                                         \
		.base = DT_INST_REG_ADDR(inst),                                                    \
		.port = DT_PROP(DT_DRV_INST(inst), index),                                         \
	};                                                                                         \
	static struct gpio_npcm_data gpio_npcm_data_##inst;                                        \
	BUILD_ASSERT(DT_PROP(DT_DRV_INST(inst), index) < NPCM_GPIO_PORT_NUM,                       \
		     "prop. port must be less than maximum port number");                          \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, gpio_npcm_init, NULL, &gpio_npcm_data_##inst,                  \
			      &gpio_npcm_cfg_##inst, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY,      \
			      &gpio_npcm_driver);
DT_INST_FOREACH_STATUS_OKAY(NPCM_GPIO_DEVICE_INIT)
