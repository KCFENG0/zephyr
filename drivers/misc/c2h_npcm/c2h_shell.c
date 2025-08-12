/*
 * Copyright (c) 2025 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/misc/c2h_npcm/c2h_npcm.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>

static int c2h_set(const struct shell *shell, size_t argc, char *argv[])
{
	const struct device *c2h_dev = device_get_binding(argv[1]);
	uint32_t sib_dev, offset, val;

	if (!device_is_ready(c2h_dev)) {
		shell_error(shell, "cannot get the device, %s\n", argv[1]);
		return -ENODEV;
	}

	sib_dev = strtoul(argv[2], NULL, 16);
	offset = strtoul(argv[3], NULL, 16);
	val = strtoul(argv[4], NULL, 16);
	shell_print(shell, "sib_dev: 0x%x, offset: 0x%x, val: 0x%x\n",
		    sib_dev, offset, val);
	sib_write_offset(c2h_dev, sib_dev, offset, val);

	return 0;
}

static int c2h_get(const struct shell *shell, size_t argc, char *argv[])
{
	const struct device *c2h_dev = device_get_binding(argv[1]);
	uint32_t sib_dev, offset;
	uint8_t val;

	if (!device_is_ready(c2h_dev)) {
		shell_error(shell, "cannot get the device, %s\n", argv[1]);
		return -ENODEV;
	}

	sib_dev = strtoul(argv[2], NULL, 16);
	offset = strtoul(argv[3], NULL, 16);
	shell_print(shell, "sib_dev: 0x%x, offset: 0x%x\n", sib_dev, offset);
	val = sib_read_offset(c2h_dev, sib_dev, offset);
	shell_print(shell, "value: 0x%x\n", val);

	return 0;
}

#define C2H_GET_HELP					\
	("Get C2H\n"					\
	 "Usage: c2h get <DEV> <SIB_DEV> <OFFSET>")

#define C2H_SET_HELP						\
	("Set C2H\n"						\
	 "Usage: c2h set <DEV> <SIB_DEV> <OFFSET> <VALUE>")

SHELL_STATIC_SUBCMD_SET_CREATE(sub_c2h,
	/* Alphabetically sorted */
	SHELL_CMD_ARG(get, NULL, C2H_GET_HELP, c2h_get, 4, 0),
	SHELL_CMD_ARG(set, NULL, C2H_SET_HELP, c2h_set, 5, 0),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(c2h, &sub_c2h, "C2H shell commands", NULL);
