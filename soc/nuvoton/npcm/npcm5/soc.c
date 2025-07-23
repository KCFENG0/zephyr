/*
 * Copyright (c) 2025 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/linker/sections.h>
#include <soc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(soc, CONFIG_SOC_LOG_LEVEL);

#ifdef CONFIG_SOC_RESET_HOOK
void soc_reset_hook(void)
{
	uintptr_t scfg_base = SCFG_BASE_ADDR;
	struct scfg_reg *inst_scfg = (struct scfg_reg *)scfg_base;

#ifdef CONFIG_GPIO_NPCM_RESET_SL_POWER_UP
	inst_scfg->DEVALT10 = NPCM_DEVALT10_CRGPIO_SELECT_SL_POWER;
#else
	inst_scfg->DEVALT10 = NPCM_DEVALT10_CRGPIO_SELECT_SL_CORE;
#endif
}
#endif

#if CONFIG_SOC_LATE_INIT_HOOK
__pinned_bss bool npcm_sys_application;

/* Hook at the end of the kernel initialization */
void soc_late_init_hook(void)
{
	/* Set the flag to indicate the system is in application stage */
	npcm_sys_application = true;
}
#endif
