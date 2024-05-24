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
#include <soc.h>

#include <zephyr/drivers/gpio/gpio_utils.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gpio_npcm, LOG_LEVEL_ERR);

/* GPIO module instances */
#define NPCM_GPIO_DEV(inst) DEVICE_DT_INST_GET(inst),
static const struct device *gpio_devs[] = {DT_INST_FOREACH_STATUS_OKAY(NPCM_GPIO_DEV)};

/*
 * General-Purpose I/O (GPIO) device registers
 */
struct gpio_reg {
	/* 0x000: Port GPIOx Data Out */
	volatile uint8_t PDOUT;
	/* 0x001: Port GPIOx Data In */
	volatile uint8_t PDIN;
	/* 0x002: Port GPIOx Direction */
	volatile uint8_t PDIR;
	/* 0x003: Port GPIOx Pull-Up or Pull-Down Enable */
	volatile uint8_t PPULL;
	/* 0x004: Port GPIOx Pull-Up/Down Selection */
	volatile uint8_t PPUD;
	volatile uint8_t reserved1;
	/* 0x006: Port GPIOx Output Type */
	volatile uint8_t PTYPE;
};

/*
 * System Configuration (SCFG) device registers
 */
struct scfg_reg {
	/* 0x000: Device Control */
	volatile uint8_t DEVCNT;
	/* 0x001: Straps Status */
	volatile uint8_t STRPST;
	/* 0x002: Reset Control and Status */
	volatile uint8_t RSTCTL;
	volatile uint8_t reserved1[3];
	/* 0x006: Device Control 4 */
	volatile uint8_t DEV_CTL4;
	volatile uint8_t reserved2[4];
	volatile uint8_t DEVALT10;
	volatile uint8_t DEVALT11;
	volatile uint8_t DEVALT12;
	volatile uint8_t reserved3[2];
	/* 0x010 - 1F: Device Alternate Function 0 - F */
	volatile uint8_t DEVALT0[16];
	volatile uint8_t reserved4[4];
	/* 0x024: DEVALTCX */
	volatile uint8_t DEVALTCX;
	volatile uint8_t reserved5[3];
	/* 0x028: Device Pull-Up Enable 0 */
	volatile uint8_t DEVPU0;
	/* 0x029: Device Pull-Down Enable 1 */
	volatile uint8_t DEVPD1;
	volatile uint8_t reserved6;
	/* 0x02B: Low-Voltage Pins Control 1 */
	volatile uint8_t LV_CTL1;
};

/* SCFG register fields */
#define NPCM_DEVCNT_F_SPI_TRIS               6
#define NPCM_DEVCNT_JEN1_HEN                 5
#define NPCM_DEVCNT_JEN0_HEN                 4
#define NPCM_STRPST_TRIST                    1
#define NPCM_STRPST_TEST                     2
#define NPCM_STRPST_JEN1                     4
#define NPCM_STRPST_JEN0                     5
#define NPCM_STRPST_SPI_COMP                 7
#define NPCM_RSTCTL_VCC1_RST_STS             0
#define NPCM_RSTCTL_DBGRST_STS               1
#define NPCM_RSTCTL_VCC1_RST_SCRATCH         3
#define NPCM_RSTCTL_LRESET_PLTRST_MODE       5
#define NPCM_RSTCTL_HIPRST_MODE              6
#define NPCM_DEV_CTL4_F_SPI_SLLK             2
#define NPCM_DEV_CTL4_SPI_SP_SEL             4
#define NPCM_DEV_CTL4_WP_IF                  5
#define NPCM_DEV_CTL4_VCC1_RST_LK            6
#define NPCM_DEVPU0_I2C0_0_PUE               0
#define NPCM_DEVPU0_I2C0_1_PUE               1
#define NPCM_DEVPU0_I2C1_0_PUE               2
#define NPCM_DEVPU0_I2C2_0_PUE               4
#define NPCM_DEVPU0_I2C3_0_PUE               6
#define NPCM_DEVPU1_F_SPI_PUD_EN             7
#define NPCM_DEVALT10_CRGPIO_SELECT_SL_CORE  0
#define NPCM_DEVALT10_CRGPIO_SELECT_SL_POWER 1
#define NPCM_DEVALTCX_GPIO_PULL_EN           7
#define NPCM_DEVALT6A_SIOX1_PU_EN            2
#define NPCM_DEVALT6A_SIOX2_PU_EN            3

/* Driver config */
struct gpio_npcm_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	/* GPIO controller base address */
	uintptr_t base;
	/* IO port */
	int port;
};

/* Driver data */
struct gpio_npcm_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
};

struct npcm_scfg_config {
	/* scfg device base address */
	uintptr_t base_scfg;
};

static const struct npcm_scfg_config npcm_scfg_cfg = {
	.base_scfg = DT_REG_ADDR_BY_NAME(DT_NODELABEL(scfg), scfg),
};

/* Driver convenience defines */
#define HAL_INSTANCE(dev)                                                                          \
	((struct gpio_reg *)((const struct gpio_npcm_config *)(dev)->config)->base)

#define HAL_SFCG_INST() (struct scfg_reg *)(npcm_scfg_cfg.base_scfg)

/* Platform specific GPIO functions */
const struct device *npcm_get_gpio_dev(int port)
{
	if (port >= ARRAY_SIZE(gpio_devs)) {
		return NULL;
	}

	return gpio_devs[port];
}

/* GPIO api functions */
static int gpio_npcm_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	struct gpio_reg *const inst = HAL_INSTANCE(dev);
	struct scfg_reg *inst_scfg = HAL_SFCG_INST();
	uint32_t mask = BIT(pin);

	/* Don't support simultaneous in/out mode */
	if (((flags & GPIO_INPUT) != 0) && ((flags & GPIO_OUTPUT) != 0)) {
		return -ENOTSUP;
	}

	/* Don't support "open source" mode */
	if (((flags & GPIO_SINGLE_ENDED) != 0) && ((flags & GPIO_LINE_OPEN_DRAIN) == 0)) {
		return -ENOTSUP;
	}

	/*
	 * Configure pin as input, if requested. Output is configured only
	 * after setting all other attributes, so as not to create a
	 * temporary incorrect logic state 0:input 1:output
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
				.port_pin_mask =                                                   \
					GPIO_PORT_PIN_MASK_FROM_NGPIOS(NPCM_GPIO_PORT_PIN_NUM),    \
			},                                                                         \
		.base = DT_INST_REG_ADDR(inst),                                                    \
		.port = inst,                                                                      \
	};                                                                                         \
	static struct gpio_npcm_data gpio_npcm_data_##inst;                                        \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, gpio_npcm_init, NULL, &gpio_npcm_data_##inst,                  \
			      &gpio_npcm_cfg_##inst, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY,      \
			      &gpio_npcm_driver);
DT_INST_FOREACH_STATUS_OKAY(NPCM_GPIO_DEVICE_INIT)
