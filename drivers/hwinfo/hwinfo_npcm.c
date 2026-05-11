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

#if !CONFIG_SOC_NPCM500
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
#endif /* !CONFIG_SOC_NPCM500 */

#if CONFIG_SOC_NPCM500
/* NPCM500 reads M7 reset cause from SYSREG.scratchpad0[15:8] (a.k.a.
 * RESET_STATUS_LOG), populated by MCU ROM. MCU ROM is the sole clearer
 * of RSTST.WDTRST_STS, so the M7 side no longer touches RSTST here.
 *
 * Layout (set by MCU ROM in npcm-common/src/npcm_boot_status.rs):
 *   scratchpad0[31:24] = SOC_BOOT_COUNT
 *   scratchpad0[23:16] = SOC_WDT_COUNT
 *   scratchpad0[15:8]  = RESET_STATUS_LOG  <-- snapshot of RSTST[7:0]
 *   scratchpad0[7:0]   = FLASH_SELECTION
 *
 * The contract is documented in caliptra-mcu-sw:
 *   docs/superpowers/specs/2026-05-11-m7-wdt-recovery-clear-ownership-design.md
 *
 * Scratchpad0 lives inside the SCFG register window declared by the
 * pinctrl node (npcm.dtsi: pinctrl@400c3000, reg-name "scfg") at
 * offset 0xB0. Derive the address from DT so the binding stays the
 * single source of truth for the SCFG base.
 */
#define NPCM500_SCFG_BASE              DT_REG_ADDR_BY_NAME(DT_NODELABEL(pinctrl), scfg)
#define NPCM500_SCRATCHPAD0_OFFSET     0xB0
#define NPCM500_SCRATCHPAD0_ADDR       (NPCM500_SCFG_BASE + NPCM500_SCRATCHPAD0_OFFSET)
#define NPCM500_RESET_STATUS_LOG_MASK  0x0000FF00UL
#define NPCM500_RESET_STATUS_LOG_SHIFT 8

#define NPCM500_RSTST_PORST_STS  BIT(0)
#define NPCM500_RSTST_CORST_STS  BIT(1)
#define NPCM500_RSTST_DBGRST_STS BIT(2)
#define NPCM500_RSTST_WDTRST_STS BIT(3)

#define NPCM500_SUPPORTED_RESET_CAUSE_FLAG                                                         \
	(RESET_POR | RESET_PIN | RESET_DEBUG | RESET_WATCHDOG)

static int npcm_hwinfo_init(void)
{
	uint32_t spad0 = sys_read32(NPCM500_SCRATCHPAD0_ADDR);
	uint8_t cause = (uint8_t)((spad0 & NPCM500_RESET_STATUS_LOG_MASK) >>
				  NPCM500_RESET_STATUS_LOG_SHIFT);

	if (cause & NPCM500_RSTST_PORST_STS) {
		npcm_reset_cause |= RESET_POR;
	}
	if (cause & NPCM500_RSTST_CORST_STS) {
		npcm_reset_cause |= RESET_PIN;
	}
	if (cause & NPCM500_RSTST_DBGRST_STS) {
		npcm_reset_cause |= RESET_DEBUG;
	}
	if (cause & NPCM500_RSTST_WDTRST_STS) {
		npcm_reset_cause |= RESET_WATCHDOG;
	}

	/* Do NOT clear scratchpad0 -- MCU ROM is the sole writer/clearer. */

	/* T0CSR.WDRST_STS is the WDT module's own latch and must still be
	 * cleared by M7 (unrelated to MCU-owned RSTST.WDTRST_STS).
	 */
	sys_write8(sys_read8(NPCM_WDT_BASE + NPCM_WDT_T0CSR_OFFSET) | BIT(NPCM_T0CSR_WDRST_STS),
		   NPCM_WDT_BASE + NPCM_WDT_T0CSR_OFFSET);

	return 0;
}
#endif /* CONFIG_SOC_NPCM500 */

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
#if CONFIG_SOC_NPCM500
	*supported = NPCM500_SUPPORTED_RESET_CAUSE_FLAG;
#else
	*supported = NPCM_SUPPORTED_RESET_CAUSE_FLAG;
#endif

	return 0;
}

SYS_INIT(npcm_hwinfo_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_OBJECTS);
