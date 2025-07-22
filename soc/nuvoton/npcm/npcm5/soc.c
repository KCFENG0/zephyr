/*
 * Copyright (c) 2025 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <soc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(soc, CONFIG_SOC_LOG_LEVEL);

void z_arm_platform_init(void)
{
	uintptr_t scfg_base = SCFG_BASE_ADDR;
	struct scfg_reg *inst_scfg = (struct scfg_reg *)scfg_base;

#ifdef CONFIG_GPIO_NPCM_RESET_SL_POWER_UP
	inst_scfg->DEVALT10 = NPCM_DEVALT10_CRGPIO_SELECT_SL_POWER;
#else
	inst_scfg->DEVALT10 = NPCM_DEVALT10_CRGPIO_SELECT_SL_CORE;
#endif
}

static int soc_npcm5_init(void)
{
	return 0;
}

SYS_INIT(soc_npcm5_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
