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
#define DEVALT5               0x400c3015
#define EMC_CTL               0x400c304d
#define GPIOB0_DEVALT         6
#define RMII_EN               0
#define GPIOB_IN              0x40097001
#define GPIOB_DIR             0x40097002
#define GPIOB_PULL            0x40097003
#define GPIOB_PUD             0x40097004
#define GPIOB0_BIT            0

	M8(DEVALT5) &= ~BIT(GPIOB0_DEVALT);
	M8(EMC_CTL) &= ~BIT(RMII_EN);
	M8(GPIOB_DIR) &= ~BIT(GPIOB0_BIT);
	M8(GPIOB_PULL) |= BIT(GPIOB0_BIT);
	M8(GPIOB_PUD) &= ~BIT(GPIOB0_BIT);

	if (M32(adr_FIU_FwInfo_status) & bit_auth_pass) {
		/* The GPIOB0 is set to pull-up by default, pull-down to enter recovery mode */
		if (0 == (M8(GPIOB_IN) & BIT(GPIOB0_BIT))) {
			M32(adr_FIU_FwInfo_status) &= ~bit_auth_pass;
		}
	}

	M8(GPIOB_PULL) &= ~BIT(GPIOB0_BIT);
}

HOOK_FUNC_ATTRIBUTE __attribute((weak)) void Rom_hook3(void)
{
	/* hook 3 */
}

HOOK_FUNC_ATTRIBUTE __attribute((weak)) void Rom_hook4(void)
{
	/* hook 4 */
}
