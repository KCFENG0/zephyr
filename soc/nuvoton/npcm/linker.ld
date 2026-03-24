/*
 * Copyright (c) 2025 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/arm/cortex_m/scripts/linker.ld>

SECTIONS
{
#ifdef CONFIG_XIP
	/* XIP: .rom_hooks must be in ROMABLE_REGION (flash) so the ROM
	 * bootloader can call hook functions before Zephyr starts.
	 * Placed here (after the main linker script) rather than via
	 * zephyr_linker_sources(DATA_SECTIONS) to avoid corrupting
	 * __data_region_end — see CMakeLists.txt for details.
	 */
	SECTION_PROLOGUE(.rom_hooks,, SUBALIGN(4))
	{
		KEEP(*(.rom_hooks_data))
		KEEP(*(.rom_hooks_fn))
	} GROUP_LINK_IN(ROMABLE_REGION)
#endif

	/* Application-defined sections (e.g., .fw_version) */
#include <snippets-app-sections.ld>
}
