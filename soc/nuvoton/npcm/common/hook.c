/*
 * Copyright (c) 2025 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* This file contains the hook functions that are called by the ROM code
 * during the boot process. The ROM code will call these functions at
 * specific points in the boot process to allow for customization of the
 * boot process.
 */

#include <zephyr/kernel.h>

#define HOOK_DATA_ATTRIBUTE __attribute__((section(".rom_hooks_data")))
#define HOOK_FUNC_ATTRIBUTE __attribute__((section(".rom_hooks_fn")))

/* Hook functions */
extern void Rom_hook2(void);
extern void Rom_hook3(void);
extern void Rom_hook4(void);

/* Hook function address table */
HOOK_DATA_ATTRIBUTE const void *hook_func_table[] = {
	(void *)0x00000000, /* Hook 1 must be flash code */
	(void *)Rom_hook2,
	(void *)Rom_hook3,
	(void *)Rom_hook4,
};

/* Default hook functions */
HOOK_FUNC_ATTRIBUTE __attribute((weak)) void Rom_hook2(void)
{
#define adr_FIU_FwInfo_status 0x100c59d4
#define bit_auth_pass         0x02
#define DEVALT5C              0x400c305c
#define GPIO5_IN              0x4008b011
#define GPIO5_DIR             0x4008b012
#define GPIO5_PULL            0x4008b013
#define GPIO5_PUD             0x4008b014
#define GPIO50_BIT            0

	M8(DEVALT5C) |= BIT(1); // DEVALT5C[1].ACPI_GPIO_SEL_0[1]=1
	M8(GPIO5_DIR) &= ~BIT(GPIO50_BIT);
	M8(GPIO5_PULL) |= BIT(GPIO50_BIT);
	M8(GPIO5_PUD) &= ~BIT(GPIO50_BIT);

	if (M32(adr_FIU_FwInfo_status) & bit_auth_pass) {
		/* The GPIO50 is set to pull-up by default, pull-down to enter recovery mode */
		if (0 == (M8(GPIO5_IN) & BIT(GPIO50_BIT))) {
			M32(adr_FIU_FwInfo_status) &= ~bit_auth_pass;
		}
	}

	M8(GPIO5_PULL) &= ~BIT(GPIO50_BIT);
}

HOOK_FUNC_ATTRIBUTE __attribute((weak)) void Rom_hook3(void)
{
	/* hook 3 */
}

HOOK_FUNC_ATTRIBUTE __attribute((weak)) void Rom_hook4(void)
{
	/* hook 4 */
}
