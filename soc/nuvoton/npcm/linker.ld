/*
 * Copyright (c) 2025 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/arm/cortex_m/scripts/linker.ld>

SECTIONS
{
	/* Application-defined sections (e.g., .fw_version) */
#include <snippets-app-sections.ld>
}
