/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcm_rstc

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/drivers/misc/c2h_npcm/c2h_npcm.h>
#include <string.h>

/* Device ID */
#define NPCM_C2H_DEV_NODE   DT_NODELABEL(c2h)
#define NPCM_SIO_CFG_SCHIDH 0x20
#define NPCM_SIO_CFG_SCHIDL 0x21

#define NPCM_DEVICE_ID_LENGTH 2

/* Reset cause */
#define NPCM_ROM_RESET_VCC_POWERUP  BIT(0)
#define NPCM_ROM_RESET_WDT_RST      BIT(1)
#define NPCM_ROM_RESET_DEBUGGER_RST BIT(2)

#define NPCM_SUPPORTED_RESET_CAUSE_FLAG (RESET_DEBUG | RESET_WATCHDOG | RESET_POR)
#define NPCM_ROM_INFO_RSTC_ADDR         0x84
#define NPCM_ROM_RSTC_BASE              (DT_INST_REG_ADDR(0) + NPCM_ROM_INFO_RSTC_ADDR)
#define NPCM_WDT_BASE                   (DT_REG_ADDR(DT_NODELABEL(twd)))
#define NPCM_WDT_T0CSR_OFFSET           0x6

__pinned_bss static uint32_t npcm_reset_cause;

static int npcm_hwinfo_init(void)
{
	/* Get reset cause */
	uint8_t reset = sys_read8(NPCM_ROM_RSTC_BASE);

	if (reset & NPCM_ROM_RESET_VCC_POWERUP) {
		npcm_reset_cause |= RESET_POR;
	}

	if (reset & NPCM_ROM_RESET_WDT_RST) {
		npcm_reset_cause |= RESET_WATCHDOG;
	}

	if (reset & NPCM_ROM_RESET_DEBUGGER_RST) {
		npcm_reset_cause |= RESET_DEBUG;
	}

	/* Clear WDT reset flag */
	sys_write8(sys_read8(NPCM_WDT_BASE + NPCM_WDT_T0CSR_OFFSET) | BIT(NPCM_T0CSR_WDRST_STS),
		   NPCM_WDT_BASE + NPCM_WDT_T0CSR_OFFSET);

	return 0;
}

/* HWINFO APIs */
ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	const struct device *dev = DEVICE_DT_GET(NPCM_C2H_DEV_NODE);
	uint8_t id[NPCM_DEVICE_ID_LENGTH];

	id[0] = c2h_read_io_cfg_reg(dev, NPCM_SIO_CFG_SCHIDH);
	id[1] = c2h_read_io_cfg_reg(dev, NPCM_SIO_CFG_SCHIDL);

	if (length > sizeof(id)) {
		length = sizeof(id);
	}

	memcpy(buffer, id, length);

	return length;
}

int z_impl_hwinfo_get_reset_cause(uint32_t *cause)
{
	*cause = npcm_reset_cause;

	return 0;
}

int z_impl_hwinfo_clear_reset_cause(void)
{
	npcm_reset_cause = 0;

	return 0;
}

int z_impl_hwinfo_get_supported_reset_cause(uint32_t *supported)
{
	*supported = NPCM_SUPPORTED_RESET_CAUSE_FLAG;

	return 0;
}

SYS_INIT(npcm_hwinfo_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_OBJECTS);
