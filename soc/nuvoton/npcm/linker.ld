/*
 * Copyright (c) 2025 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/arm/cortex_m/scripts/linker.ld>

SECTIONS
{
	SECTION_PROLOGUE(.rom_hooks,, SUBALIGN(4))
	{
		KEEP(*(.rom_hooks_data))
		KEEP(*(.rom_hooks_fn))
	} GROUP_LINK_IN(ROMABLE_REGION)
}
