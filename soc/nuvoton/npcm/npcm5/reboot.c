/*
 * Copyright (c) 2026 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief NPCM500 SoC reboot hook.
 *
 * Provides a strong sys_arch_reboot() that overrides the __weak ARM
 * Cortex-M implementation in arch/arm/core/cortex_m/scb.c.
 *
 *   kernel reboot cold/warm -> npcm_soc_reset() (full SoC reset via TWD)
 *
 * A reboot drives the reset through the NPCM Timer-Watchdog so the SoC
 * power/clock state is re-initialised
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <cmsis_core.h>
#include <soc.h>

/* NPCM Timer-Watchdog (TWD) block base — DT node "twd". */
#define NPCM_TWD_BASE DT_REG_ADDR(DT_NODELABEL(twd))

/*
 * SCFG Watchdog Reset Control register. Bit 0 (CM7) routes the watchdog
 * reset to the Cortex-M7 core. WDTRST_CTL powers up as 0, so the reset
 * path must set it itself rather than relying on wdt_npcm_init().
 */
#define NPCM_SCFG_WDTRST_CTL (SCFG_BASE_ADDR + 0x8C)
#define NPCM_WDTRST_CTL_CM7  BIT(0)

/* TWCFG bit 6 (WDCT0I_SEL) — required for the watchdog reset to propagate. */
#define NPCM_TWCFG_WDCT0I_SEL BIT(6)

FUNC_NORETURN void npcm_soc_reset(void)
{
	struct twd_reg *const twd = (struct twd_reg *)NPCM_TWD_BASE;
	volatile uint32_t *const wdtrst_ctl =
		(volatile uint32_t *)NPCM_SCFG_WDTRST_CTL;

	/* sys_reboot() already locked IRQs; lock again so this stays safe
	 * if called directly from other SoC code (e.g. a fault handler).
	 */
	(void)irq_lock();

	/* TWCFG = 0x70: WDCT0I_SEL (reset propagation) | WDSDME (service
	 * only via WDSDM=0x5C, so the watchdog cannot be accidentally fed)
	 * | WDCT0I (T0IN as watchdog prescaler clock). No lock bits.
	 */
	twd->TWCFG = NPCM_TWCFG_WDCT0I_SEL | BIT(NPCM_TWCFG_WDSDME) |
		     BIT(NPCM_TWCFG_WDCT0I);

	/* T0CSR = 0x90: TESDIS disables early-touch, so loading WDCNT below
	 * arms the counter instead of being treated as a service; WDRST_STS
	 * is W1C — clear stale WD reset status
	 */
	twd->T0CSR = BIT(NPCM_T0CSR_TESDIS) | BIT(NPCM_T0CSR_WDRST_STS);

	/* Prescalers: T0 /32 and watchdog /32 -> WDCNT tick = LFCLK/1024
	 * ~= 32 Hz. With WDCNT = 2 below, reset fires in ~62 ms.
	 */
	twd->TWCP = 0x05;
	twd->WDCP = 0x05;

	/* Park the T0 timer at max preset so its (unused) timer event does
	 * not fire before the watchdog reset.
	 */
	twd->TWDT0 = 0xFFFF;

	/* Route the watchdog reset to the CM7 core */
	*wdtrst_ctl |= NPCM_WDTRST_CTL_CM7;

	/* Load WDCNT last — this arms the already-running watchdog. */
	twd->WDCNT = 0x02;

	/* Watchdog is counting down; spin until it resets the SoC. */
	for (;;) {
		/* nothing */
	}
}

extern void sys_arch_reboot(int type);

void sys_arch_reboot(int type)
{
	npcm_soc_reset();
}
