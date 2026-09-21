/*
 * Copyright (c) 2023 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * NPCM eSPI Virtual Wire GPIO - single-bank driver
 *
 * Compatible: nuvoton,npcm-vw-gpio
 *
 * Each DT child node represents exactly ONE 32-bit VW register (4 wires):
 *   VWGPSM0~15  (Slave-to-Master, output, BIC->Host):  0x4000A180 + n*4
 *   VWGPMS0~15  (Master-to-Slave, input,  Host->BIC):  0x4000A1C0 + n*4
 *
 * Register bit layout (from datasheet):
 *   Bits [3:0]  Wire 3-0 data
 *   Bits [7:4]  Wire 3-0 Valid (write 1 to mark wire as valid/changed)
 *
 * Pin numbering within a bank: 0 = Wire 0, 1 = Wire 1, 2 = Wire 2, 3 = Wire 3
 *
 * Global pin to bank mapping (defined by the HAL):
 *   bank_idx 0~15  (VWGPSM), wire w  ->  global pin = bank_idx*4 + w    (0~63)
 *   bank_idx 16~31 (VWGPMS), wire w  ->  global pin = bank_idx*4 + w    (64~127)
 */

#define DT_DRV_COMPAT nuvoton_npcm_vw_gpio

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/sys/sys_io.h>
#include <soc.h>

#include <zephyr/drivers/gpio/gpio_utils.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gpio_npcm_vw, CONFIG_GPIO_LOG_LEVEL);

#define VW_BANK_NPINS      4U  /* wires per bank register */
#define VW_INPUT_BANK_BASE 16U /* bank_idx of VWGPMS0 */

/* Bit position of the direction flag inside espi_event.evt_details (espi_npcm) */
#define VW_EVT_DIRECTION_POS 7U

/* Driver config (per DT instance, read-only) */
struct gpio_npcm_vw_bank_config {
	/* gpio_driver_config must be first */
	struct gpio_driver_config common;
	uintptr_t reg;       /* absolute address of the 4-byte bank register */
	bool      is_output; /* true = VWGPSM (BIC->Host), false = VWGPMS (Host->BIC) */
	uint8_t   bank_idx;  /* 0~31 globally: 0~15=VWGPSM, 16~31=VWGPMS */
};

/* Driver data (per DT instance, mutable) */
struct gpio_npcm_vw_bank_data {
	/* gpio_driver_data must be first */
	struct gpio_driver_data common;
	sys_slist_t callbacks;
	const struct device *self;
	/* eSPI callback -- only used by VWGPMS (input) banks */
	const struct device  *espi_dev;
	struct espi_callback  espi_cb;
};

#define DRV_CONFIG(dev)  ((const struct gpio_npcm_vw_bank_config *)(dev)->config)
#define DRV_DATA(dev)    ((struct gpio_npcm_vw_bank_data *)(dev)->data)

static int gpio_npcm_vw_bank_configure(const struct device *dev,
				       gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_npcm_vw_bank_config *cfg = DRV_CONFIG(dev);

	if (pin >= VW_BANK_NPINS) {
		return -EINVAL;
	}

	if ((flags & GPIO_INPUT) && (flags & GPIO_OUTPUT)) {
		return -ENOTSUP;
	}
	if (flags & (GPIO_PULL_UP | GPIO_PULL_DOWN | GPIO_OPEN_DRAIN)) {
		return -ENOTSUP;
	}

	if (flags & GPIO_OUTPUT) {
		if (!cfg->is_output) {
			LOG_ERR("VWGPMS%d pin %d: cannot configure as output",
				cfg->bank_idx - VW_INPUT_BANK_BASE, pin);
			return -EINVAL;
		}
		uint32_t reg_val = sys_read32(cfg->reg);

		reg_val &= ~((1U << (4 + pin)) | (1U << pin));
		reg_val |=  (1U << (4 + pin));
		if (flags & GPIO_OUTPUT_INIT_HIGH) {
			reg_val |= (1U << pin);
		}
		sys_write32(reg_val, cfg->reg);
		LOG_DBG("VWGPSM%d pin %d: OUTPUT init=%s reg=0x%08x val=0x%08x",
			cfg->bank_idx, pin,
			(flags & GPIO_OUTPUT_INIT_HIGH) ? "HIGH" : "LOW",
			(uint32_t)cfg->reg, reg_val);
	} else if (flags & GPIO_INPUT) {
		if (cfg->is_output) {
			LOG_ERR("VWGPSM%d pin %d: cannot configure as input",
				cfg->bank_idx, pin);
			return -EINVAL;
		}
		LOG_DBG("VWGPMS%d pin %d: INPUT",
			cfg->bank_idx - VW_INPUT_BANK_BASE, pin);
	}

	return 0;
}

static int gpio_npcm_vw_bank_port_get_raw(const struct device *dev,
					  gpio_port_value_t *value)
{
	const struct gpio_npcm_vw_bank_config *cfg = DRV_CONFIG(dev);
	uint32_t raw = sys_read32(cfg->reg);

	/* bits[3:0] = Wire 3-0 data */
	*value = raw & 0x0FU;

	LOG_DBG("%s%d get_raw: reg=0x%08x raw=0x%08x wires=0x%x",
		cfg->is_output ? "VWGPSM" : "VWGPMS",
		cfg->is_output ? cfg->bank_idx : cfg->bank_idx - VW_INPUT_BANK_BASE,
		(uint32_t)cfg->reg, raw, (uint32_t)*value);
	return 0;
}

static int gpio_npcm_vw_bank_port_set_masked_raw(const struct device *dev,
						 gpio_port_pins_t mask,
						 gpio_port_value_t value)
{
	const struct gpio_npcm_vw_bank_config *cfg = DRV_CONFIG(dev);

	if (!cfg->is_output) {
		return -ENOTSUP;
	}

	uint32_t m       = mask & 0x0FU;
	uint32_t old_val = sys_read32(cfg->reg);
	uint32_t reg_val = old_val;

	reg_val |= (m << 4);
	reg_val  = (reg_val & ~m) | (value & m);

	sys_write32(reg_val, cfg->reg);
	LOG_DBG("VWGPSM%d set_masked: mask=0x%x val=0x%x reg=0x%08x 0x%08x->0x%08x",
		cfg->bank_idx, (uint32_t)m, (uint32_t)(value & m),
		(uint32_t)cfg->reg, old_val, reg_val);
	return 0;
}

static int gpio_npcm_vw_bank_port_set_bits_raw(const struct device *dev,
					       gpio_port_value_t mask)
{
	const struct gpio_npcm_vw_bank_config *cfg = DRV_CONFIG(dev);

	if (!cfg->is_output) {
		return -ENOTSUP;
	}

	uint32_t m       = mask & 0x0FU;
	uint32_t old_val = sys_read32(cfg->reg);
	uint32_t reg_val = old_val | (m << 4) | m;

	sys_write32(reg_val, cfg->reg);
	LOG_DBG("VWGPSM%d set_bits: mask=0x%x reg=0x%08x 0x%08x->0x%08x",
		cfg->bank_idx, (uint32_t)m, (uint32_t)cfg->reg, old_val, reg_val);
	return 0;
}

static int gpio_npcm_vw_bank_port_clear_bits_raw(const struct device *dev,
						 gpio_port_value_t mask)
{
	const struct gpio_npcm_vw_bank_config *cfg = DRV_CONFIG(dev);

	if (!cfg->is_output) {
		return -ENOTSUP;
	}

	uint32_t m       = mask & 0x0FU;
	uint32_t old_val = sys_read32(cfg->reg);
	uint32_t reg_val = (old_val | (m << 4)) & ~m;

	sys_write32(reg_val, cfg->reg);
	LOG_DBG("VWGPSM%d clear_bits: mask=0x%x reg=0x%08x 0x%08x->0x%08x",
		cfg->bank_idx, (uint32_t)m, (uint32_t)cfg->reg, old_val, reg_val);
	return 0;
}

static int gpio_npcm_vw_bank_port_toggle_bits(const struct device *dev,
					      gpio_port_value_t mask)
{
	const struct gpio_npcm_vw_bank_config *cfg = DRV_CONFIG(dev);

	if (!cfg->is_output) {
		return -ENOTSUP;
	}

	uint32_t m       = mask & 0x0FU;
	uint32_t reg_val = sys_read32(cfg->reg);

	reg_val ^= m;
	reg_val |= (m << 4);
	sys_write32(reg_val, cfg->reg);
	return 0;
}

static int gpio_npcm_vw_bank_pin_interrupt_configure(const struct device *dev,
						     gpio_pin_t pin,
						     enum gpio_int_mode mode,
						     enum gpio_int_trig trig)
{
	const struct gpio_npcm_vw_bank_config *cfg = DRV_CONFIG(dev);

	ARG_UNUSED(mode);
	ARG_UNUSED(trig);
	ARG_UNUSED(pin);

	if (cfg->is_output) {
		return -ENOTSUP;
	}
	/* VW inputs are interrupt-driven via eSPI events -- always enabled */
	return 0;
}

static int gpio_npcm_vw_bank_manage_callback(const struct device *dev,
					     struct gpio_callback *callback,
					     bool set)
{
	struct gpio_npcm_vw_bank_data *data = DRV_DATA(dev);

	return gpio_manage_callback(&data->callbacks, callback, set);
}

/*
 * eSPI VW event callback -- registered only by VWGPMS (input) banks.
 *
 * espi_npcm reports VWGPMS updates as
 *   evt_details = (NPCM_VWGP_M_TO_S << 7) | vwgpms_index
 *   evt_data    = VWGPMS[index] & 0xFF
 * while ordinary virtual wires report evt_details = ESPI_VWIRE_SIGNAL_*, so the
 * direction bit must be checked before the index is trusted.
 *
 * GPIO callbacks are fired with the GLOBAL pin NUMBER (not a bitmask bit)
 * so the platform layer can cast it directly to uint8_t as a pin index
 * for vw_gpio_get().
 */
static void gpio_npcm_vw_bank_espi_callback(const struct device *espi_dev,
					    struct espi_callback *cb,
					    struct espi_event event)
{
	struct gpio_npcm_vw_bank_data *data =
		CONTAINER_OF(cb, struct gpio_npcm_vw_bank_data, espi_cb);
	const struct gpio_npcm_vw_bank_config *cfg = DRV_CONFIG(data->self);

	ARG_UNUSED(espi_dev);

	if (event.evt_type != ESPI_BUS_EVENT_VWIRE_RECEIVED) {
		return;
	}

	if (((event.evt_details >> VW_EVT_DIRECTION_POS) & 0x01U) !=
	    NPCM_VWGP_M_TO_S) {
		return;
	}

	uint8_t group = event.evt_details & 0x0FU;

	if (group != (cfg->bank_idx - VW_INPUT_BANK_BASE)) {
		return;
	}

	uint8_t reg_raw   = event.evt_data & 0xFFU;
	uint8_t valid     = (reg_raw >> 4) & 0x0FU;
	uint8_t wire_data = reg_raw & 0x0FU;

	LOG_DBG("VWGPMS%d IRQ: valid=0x%x data=0x%x reg=0x%08x",
		group, valid, wire_data, (uint32_t)cfg->reg);

	for (int wire = 0; wire < (int)VW_BANK_NPINS; wire++) {
		if (!(valid & BIT(wire))) {
			continue;
		}

		uint8_t global_pin = cfg->bank_idx * VW_BANK_NPINS + wire;

		LOG_DBG("  wire=%d val=%d global_pin=%d",
			wire, (wire_data >> wire) & 1, global_pin);

		gpio_fire_callbacks(&data->callbacks, data->self, global_pin);
	}
}

static int gpio_npcm_vw_bank_init(const struct device *dev)
{
	struct gpio_npcm_vw_bank_data *data = DRV_DATA(dev);
	const struct gpio_npcm_vw_bank_config *cfg = DRV_CONFIG(dev);

	sys_slist_init(&data->callbacks);
	data->self = dev;

	if (!cfg->is_output) {
		data->espi_dev = DEVICE_DT_GET(DT_NODELABEL(espi0));
		if (!device_is_ready(data->espi_dev)) {
			LOG_ERR("VWGPMS%d: ESPI device not ready",
				cfg->bank_idx - VW_INPUT_BANK_BASE);
			return -ENODEV;
		}

		espi_init_callback(&data->espi_cb,
				   gpio_npcm_vw_bank_espi_callback,
				   ESPI_BUS_EVENT_VWIRE_RECEIVED);

		if (espi_add_callback(data->espi_dev, &data->espi_cb) < 0) {
			LOG_ERR("VWGPMS%d: failed to add eSPI callback",
				cfg->bank_idx - VW_INPUT_BANK_BASE);
			return -EIO;
		}

		LOG_DBG("VWGPMS%d init: reg=0x%08x (input  pins %d~%d)",
			cfg->bank_idx - VW_INPUT_BANK_BASE, (uint32_t)cfg->reg,
			cfg->bank_idx * VW_BANK_NPINS,
			cfg->bank_idx * VW_BANK_NPINS + VW_BANK_NPINS - 1);
	} else {
		LOG_DBG("VWGPSM%d init: reg=0x%08x (output pins %d~%d)",
			cfg->bank_idx, (uint32_t)cfg->reg,
			cfg->bank_idx * VW_BANK_NPINS,
			cfg->bank_idx * VW_BANK_NPINS + VW_BANK_NPINS - 1);
	}

	return 0;
}

static const struct gpio_driver_api gpio_npcm_vw_bank_driver_api = {
	.pin_configure           = gpio_npcm_vw_bank_configure,
	.port_get_raw            = gpio_npcm_vw_bank_port_get_raw,
	.port_set_masked_raw     = gpio_npcm_vw_bank_port_set_masked_raw,
	.port_set_bits_raw       = gpio_npcm_vw_bank_port_set_bits_raw,
	.port_clear_bits_raw     = gpio_npcm_vw_bank_port_clear_bits_raw,
	.port_toggle_bits        = gpio_npcm_vw_bank_port_toggle_bits,
	.pin_interrupt_configure = gpio_npcm_vw_bank_pin_interrupt_configure,
	.manage_callback         = gpio_npcm_vw_bank_manage_callback,
};

/*
 * Parent-child DTS design:
 *   Parent node holds compatible = "nuvoton,npcm-vw-gpio".
 *   Each child node (vwgpsm0~15, vwgpms0~15) has no compatible; it carries
 *   nuvoton,vw-bank-index (0~31) and optionally nuvoton,vw-is-output.
 *   Address = parent_base + bank_idx * 4
 *     bank_idx  0~15 -> VWGPSM0~15 (output, 0x4000A180~0x4000A1BC)
 *     bank_idx 16~31 -> VWGPMS0~15 (input,  0x4000A1C0~0x4000A1FC)
 *
 * Status inheritance rule:
 *   - Parent disabled  -> NO children are instantiated, regardless of child status.
 *   - Parent okay      -> only children with status = "okay" are instantiated.
 *
 * This is achieved by two-level macro expansion:
 *   DT_INST_FOREACH_STATUS_OKAY  - outer loop, only visits parent nodes that are okay.
 *   DT_FOREACH_CHILD_STATUS_OKAY - inner loop, visits okay children of that parent.
 */
#define GPIO_NPCM_VW_BANK_DEVICE_INIT(child_node_id)                                \
	static const struct gpio_npcm_vw_bank_config                                \
		gpio_npcm_vw_bank_cfg_##child_node_id = {                           \
		.common = {                                                         \
			.port_pin_mask =                                            \
				GPIO_PORT_PIN_MASK_FROM_DT_NODE(child_node_id),     \
		},                                                                  \
		.reg       = DT_REG_ADDR(DT_PARENT(child_node_id)) +                \
			     DT_PROP(child_node_id, nuvoton_vw_bank_index) * 4U,    \
		.is_output = DT_PROP_OR(child_node_id, nuvoton_vw_is_output, 0),    \
		.bank_idx  = DT_PROP(child_node_id, nuvoton_vw_bank_index),         \
	};                                                                          \
                                                                                    \
	static struct gpio_npcm_vw_bank_data                                        \
		gpio_npcm_vw_bank_data_##child_node_id;                             \
                                                                                    \
	DEVICE_DT_DEFINE(child_node_id,                                             \
			 gpio_npcm_vw_bank_init,                                    \
			 NULL,                                                      \
			 &gpio_npcm_vw_bank_data_##child_node_id,                   \
			 &gpio_npcm_vw_bank_cfg_##child_node_id,                    \
			 POST_KERNEL,                                               \
			 CONFIG_GPIO_NPCM_VW_INIT_PRIORITY,                         \
			 &gpio_npcm_vw_bank_driver_api);

/* Outer: only enter if the parent (vwgpio) has status = "okay" */
#define GPIO_NPCM_VW_INIT_CHILDREN(inst)                                            \
	DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(inst),                             \
				     GPIO_NPCM_VW_BANK_DEVICE_INIT)

DT_INST_FOREACH_STATUS_OKAY(GPIO_NPCM_VW_INIT_CHILDREN)
