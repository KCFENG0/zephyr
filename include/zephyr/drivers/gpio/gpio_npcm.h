/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_GPIO_GPIO_NPCM_H_
#define ZEPHYR_INCLUDE_DRIVERS_GPIO_GPIO_NPCM_H_

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Numbers of GPIO device */
#define NPCM_GPIO_PORT_NUM 16U

/* Pin number for each GPIO device */
#define NPCM_GPIO_PORT_PIN_NUM 8U

/* General-Purpose I/O (GPIO) device registers */
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

/* System Configuration (SCFG) device registers */
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
#define NPCM_DEVALTCX_GPIO_PULL_EN 7

/* GPIO property */
struct npcm_gpio_prop {
	uint8_t port;
	uint8_t pin;
} __packed;

/* GPIO pimux and device control mapping data */
struct npcm_gpio_scfg {
	uint16_t id: 11;
	bool is_set: 1;
	uint16_t reserved: 4;
} __packed;

struct npcm_gpio_scfg_map {
	uint8_t cfg_size;
	const struct npcm_gpio_scfg *scfg;
} __packed;

/* Driver config */
struct gpio_npcm_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	/* GPIO controller base address */
	uintptr_t base;
	/* IO port */
	uint8_t port;
};

/* Driver data */
struct gpio_npcm_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
};

/**
 * @brief Get GPIO device instance by port index
 *
 * @param port GPIO device index
 *
 * @retval Pointer to structure device
 * @retval NULL Invalid parameter of GPIO port index
 */
const struct device *npcm_get_gpio_dev(int port);

/**
 * @brief Enable the connection between io pads and GPIO instance
 *
 * @param dev Pointer to device structure for the gpio driver instance.
 * @param pin Pin number.
 */
void npcm_gpio_enable_io_pads(const struct device *dev, int pin);

/**
 * @brief Disable the connection between io pads and GPIO instance
 *
 * @param dev Pointer to device structure for the gpio driver instance.
 * @param pin Pin number.
 */
void npcm_gpio_disable_io_pads(const struct device *dev, int pin);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_GPIO_GPIO_NPCM_H_ */
