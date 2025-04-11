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
	/* hook 2 */
}

HOOK_FUNC_ATTRIBUTE __attribute((weak)) void Rom_hook3(void)
{
	/* hook 3 */
}

HOOK_FUNC_ATTRIBUTE __attribute((weak)) void Rom_hook4(void)
{
	/* hook 4 */
}
