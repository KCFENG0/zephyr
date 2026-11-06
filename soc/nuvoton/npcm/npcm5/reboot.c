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

#define NPCM_SCFG_CORST_CTL (SCFG_BASE_ADDR + 0x88)

/*
 * SCFG Watchdog Reset Control register. Bit 0 (CM7) routes the watchdog
 * reset to the Cortex-M7 core. WDTRST_CTL powers up as 0, so the reset
 * path must set it itself rather than relying on wdt_npcm_init().
 */
#define NPCM_SCFG_WDTRST_CTL (SCFG_BASE_ADDR + 0x8C)
#define NPCM_WDTRST_CTL_CM7  BIT(0)
#define NPCM_WDTRST_CTL_USBD BIT(4)

#define NPCM_RESET_ROUTE_MASK (BIT(0) | BIT(4))

/* TWCFG bit 6 (WDCT0I_SEL) — required for the watchdog reset to propagate. */
#define NPCM_TWCFG_WDCT0I_SEL BIT(6)

/*
 * CR_GLUE scratchpad register 1 (SPAD1). Survives the watchdog/core reset
 * driven by npcm_soc_reset(), so it carries the kernel reboot type across
 * the reset for post-reset code to read back.
 */
#define NPCM_SCFG_SCRATCHPAD1 (SCFG_BASE_ADDR + 0xB4)

/* SPAD1[15:8] records the reboot type: 0 = cold reboot, 1 = warm reboot. */
#define NPCM_SPAD1_REBOOT_TYPE_MASK  GENMASK(15, 8)
#define NPCM_SPAD1_REBOOT_TYPE_SHIFT 8

FUNC_NORETURN void npcm_soc_reset(void)
{
	struct twd_reg *const twd = (struct twd_reg *)NPCM_TWD_BASE;
	volatile uint32_t *const wdtrst_ctl =
		(volatile uint32_t *)NPCM_SCFG_WDTRST_CTL;
	volatile uint32_t *const corst_ctl =
		(volatile uint32_t *)NPCM_SCFG_CORST_CTL;

	/* sys_reboot() already locked IRQs; lock again so this stays safe
	 * if called directly from other SoC code (e.g. a fault handler).
	 */
	(void)irq_lock();

	const bool wd_running = (twd->T0CSR & BIT(NPCM_T0CSR_WD_RUN)) != 0;

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

	/*
	 * Route the watchdog reset to the CM7 and USBD domains. Clear and
	 * reassert bits 0 and 4 so stale reset routing from ROM/boot code does
	 * not linger across resets. This pattern avoids leaving unrelated bits
	 * set while still preserving the rest of the register state.
	 */
	*wdtrst_ctl = (*wdtrst_ctl & ~NPCM_RESET_ROUTE_MASK) |
		      (NPCM_WDTRST_CTL_CM7 | NPCM_WDTRST_CTL_USBD);
	*corst_ctl = (*corst_ctl & ~NPCM_RESET_ROUTE_MASK) |
		     (NPCM_CORST_CTL_CM7 | NPCM_CORST_CTL_USBD);

	/* Load WDCNT last — this arms the already-running watchdog. */
	twd->WDCNT = 0x02;

	/* If the application watchdog is already running with WDSDME=1, the
	 * WDCNT write above only updates the reload value; touch it so the
	 * short count takes effect instead of waiting for the old timeout.
	 */
	if (wd_running) {
		twd->WDSDM = 0x5C;
	}

	/* Watchdog is counting down; spin until it resets the SoC. */
	for (;;) {
		/* nothing */
	}
}

/*
 * Map a Zephyr reboot type to the value stored in SPAD1[15:8]: only an
 * explicit SYS_REBOOT_WARM records 1; a cold reboot and any other caller
 * (e.g. a fault handler invoking sys_arch_reboot() directly) record 0.
 */
static uint8_t npcm_reboot_type_to_spad1(int type)
{
	return (type == SYS_REBOOT_WARM) ? 1U : 0U;
}

/* Record the reboot type in SPAD1[15:8], preserving the other SPAD1 bits. */
static void npcm_scratchpad1_record_reboot(int type)
{
	volatile uint32_t *const spad1 =
		(volatile uint32_t *)NPCM_SCFG_SCRATCHPAD1;
	uint32_t val = *spad1;

	val &= ~NPCM_SPAD1_REBOOT_TYPE_MASK;
	val |= ((uint32_t)npcm_reboot_type_to_spad1(type)
		<< NPCM_SPAD1_REBOOT_TYPE_SHIFT) & NPCM_SPAD1_REBOOT_TYPE_MASK;
	*spad1 = val;
}

extern void sys_arch_reboot(int type);

void sys_arch_reboot(int type)
{
	/* SPAD1 reboot-type logging is an FPGA bring-up aid only. */
	if (IS_ENABLED(CONFIG_NPCM500_FPGA_REBOOT_TYPE_LOG)) {
		npcm_scratchpad1_record_reboot(type);
	}

	npcm_soc_reset();
}
