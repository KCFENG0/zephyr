/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _NUVOTON_NPCM_SOC_H_
#define _NUVOTON_NPCM_SOC_H_

/* CMSIS required definitions */
#define __FPU_PRESENT CONFIG_CPU_HAS_FPU
#define __MPU_PRESENT CONFIG_CPU_HAS_ARM_MPU

/* Add include for DTS generated information */
#include <zephyr/devicetree.h>
#include <zephyr/toolchain.h>
#include <cmsis_core_m_defaults.h>

#include <reg/reg_access.h>
#include <reg/reg_def.h>
#if defined(CONFIG_ADC_NPCM_SARADC)
/* Include NPCX-compatible SAR ADC register definitions */
#include <npcm5/reg_def_adc.h>
#endif
#include <soc_dt.h>
#include <soc_espi_taf.h>
#include <soc_pins.h>
#include "gdma.h"

#define NPCM_PUPD_EN_OFFSET(n)		(((n) == 2 ) ? 0x073 : ((n) == 3 ) ? 0x07b : (0x028 + n))

#define SCFG_BASE_ADDR (0x400C3000)

/**
 * @brief Trigger a full SoC-level cold reset via the NPCM watchdog.
 *
 * Configures and arms the NPCM Timer-Watchdog (TWD) so the watchdog
 * underflow drives a reset that propagates to the Cortex-M7 core. Does not
 * return. Safe to call with interrupts locked / the kernel torn down: it is
 * the SYS_REBOOT_COLD path of sys_arch_reboot().
 */
FUNC_NORETURN void npcm_soc_reset(void);

/**
 * @brief Check if the post kernel stage is done
 *
 * @return true if invoked before application initialization
 * @return false if invoked during/after application initialization
 */
static inline bool npcm_is_post_kernel(void)
{
#if CONFIG_SOC_LATE_INIT_HOOK
	extern bool npcm_sys_application;

	return !npcm_sys_application;
#else
	return false;
#endif
}

#endif /* _NUVOTON_NPCM_SOC_H_ */
