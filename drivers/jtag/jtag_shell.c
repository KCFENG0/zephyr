/*
 * Copyright (c) 2021 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief JTAG shell commands.
 */

#include <zephyr/shell/shell.h>
#include <zephyr/drivers/jtag.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#define JTAG_DEVICE_PREFIX              "jtagm"

#define JTAG_SHELL_BUF_SIZE	512

/* Word aligned: the driver reads and writes the buffers 32 bits at a time */
static uint32_t tdo_buffer[JTAG_SHELL_BUF_SIZE / 4];
static uint32_t tdi_buffer[JTAG_SHELL_BUF_SIZE / 4];

/*
 * Parse a hex string (optional 0x prefix, any length) into buf, least
 * significant byte first, so values wider than 32 bits can be scanned.
 */
static int parse_hex(const char *str, uint8_t *buf, size_t buf_len)
{
	size_t len;

	if (str[0] == '0' && (str[1] == 'x' || str[1] == 'X')) {
		str += 2;
	}
	len = strlen(str);
	if (len == 0 || DIV_ROUND_UP(len, 2) > buf_len) {
		return -EINVAL;
	}

	memset(buf, 0, buf_len);
	for (size_t i = 0; i < len; i++) {
		char c = str[len - 1 - i];
		uint8_t nibble;

		if (c >= '0' && c <= '9') {
			nibble = c - '0';
		} else if (c >= 'a' && c <= 'f') {
			nibble = c - 'a' + 10;
		} else if (c >= 'A' && c <= 'F') {
			nibble = c - 'A' + 10;
		} else {
			return -EINVAL;
		}
		buf[i / 2] |= nibble << ((i % 2) * 4);
	}
	return 0;
}

static int cmd_scan(const struct shell *shell, char **argv, bool ir)
{
	const struct device *dev;
	uint8_t *tdo = (uint8_t *)tdo_buffer;
	uint8_t *tdi = (uint8_t *)tdi_buffer;
	static char line[2 * JTAG_SHELL_BUF_SIZE + 1];
	uint32_t bit_len, byte_len;
	int err, index;

	dev = device_get_binding(argv[-1]);
	if (!dev || !device_is_ready(dev)) {
		shell_error(shell, "JTAG device not found or not ready");
		return -EINVAL;
	}

	bit_len = strtoul(argv[1], NULL, 0);
	if (bit_len == 0 || bit_len > JTAG_SHELL_BUF_SIZE * 8) {
		shell_error(shell, "length must be 1..%d bits", JTAG_SHELL_BUF_SIZE * 8);
		return -EINVAL;
	}
	if (parse_hex(argv[2], tdo, sizeof(tdo_buffer))) {
		shell_error(shell, "invalid hex value: %s", argv[2]);
		return -EINVAL;
	}
	memset(tdi, 0, sizeof(tdi_buffer));

	if (ir) {
		err = jtag_ir_scan(dev, bit_len, tdo, tdi, TAP_IDLE);
	} else {
		err = jtag_dr_scan(dev, bit_len, tdo, tdi, TAP_IDLE);
	}
	if (err) {
		shell_error(shell, "failed to %s scan (err %d)", ir ? "IR" : "DR", err);
		return err;
	}

	/* Captured bits as one hex number, most significant byte first */
	byte_len = DIV_ROUND_UP(bit_len, 8);
	for (index = byte_len - 1; index >= 0; index--) {
		snprintk(&line[2 * (byte_len - 1 - index)], 3, "%02x", tdi[index]);
	}
	shell_print(shell, "%s", line);
	return 0;
}

static int cmd_ir_scan(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	return cmd_scan(shell, argv, true);
}

static int cmd_dr_scan(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	return cmd_scan(shell, argv, false);
}

static int cmd_frequency(const struct shell *shell, size_t argc, char **argv)
{
	const struct device *dev;
	uint32_t freq;
	int err;

	dev = device_get_binding(argv[-1]);
	if (!dev || !device_is_ready(dev)) {
		shell_error(shell, "JTAG device not found or not ready");
		return -EINVAL;
	}

	freq = strtoul(argv[1], NULL, 0);

	err = jtag_freq_set(dev, freq);
	if (err) {
		shell_error(shell, "failed to setup JTAG frequency(err %d)",
				err);
		return err;
	}
	err = jtag_freq_get(dev, &freq);
	if (err) {
		shell_error(shell, "failed to get JTAG frequency (err %d)",
				err);
		return err;
	}
	shell_print(shell, "%d\n", freq);

	return 0;
}

static int cmd_tap_state(const struct shell *shell, size_t argc, char **argv, void *data)
{
	const struct device *dev;
	enum tap_state state = (enum tap_state) data;
	int err;

	dev = device_get_binding(argv[-2]);
	if (!dev || !device_is_ready(dev)) {
		shell_error(shell, "JTAG device not found or not ready");
		return -EINVAL;
	}

	err = jtag_tap_set(dev, state);
	if (err) {
		shell_error(shell, "failed to set JTAG tap_state to %d(err %d)",
				state, err);
		return err;
	}

	return 0;
}

static int cmd_sw_xfer(const struct shell *shell, size_t argc, char **argv, void *data)
{
	const struct device *dev;
	enum jtag_pin pin = (enum jtag_pin) data;
	uint8_t value;
	int err;

	dev = device_get_binding(argv[-3]);
	if (!dev || !device_is_ready(dev)) {
		shell_error(shell, "JTAG device not found or not ready");
		return -EINVAL;
	}
	value = strcmp("high", argv[-1]) ? 0 : 1;

	err = jtag_sw_xfer(dev, pin, value);
	if (err) {
		shell_error(shell, "failed to transfer pin%d = %d(err %d)",
				pin, value, err);
		return err;
	}

	return 0;
}

SHELL_SUBCMD_DICT_SET_CREATE(sub_tap_cmds, cmd_tap_state,
		(DREXIT2, TAP_DREXIT2, "DR-Exit2 state"),
		(DREXIT1, TAP_DREXIT1, "DR-Exit1 state"),
		(DRSHIFT, TAP_DRSHIFT, "DR-Shift state"),
		(DRPAUSE, TAP_DRPAUSE, "DR-Pause state"),
		(IRSELECT, TAP_IRSELECT, "IR-Select state"),
		(DRUPDATE, TAP_DRUPDATE, "DR-Update state"),
		(DRCAPTURE, TAP_DRCAPTURE, "DR-Capture state"),
		(DRSELECT, TAP_DRSELECT, "DR-Select state"),
		(IREXIT2, TAP_IREXIT2, "IR-Exit2 state"),
		(IREXIT1, TAP_IREXIT1, "IR-Exit1 state"),
		(IRSHIFT, TAP_IRSHIFT, "IR-Shift state"),
		(IRPAUSE, TAP_IRPAUSE, "IR-Pause state"),
		(IDLE, TAP_IDLE, "Run-Test/Idle state"),
		(IRUPDATE, TAP_IRUPDATE, "IR-Update state"),
		(IRCAPTURE, TAP_IRCAPTURE, "IR-Capture state"),
		(RESET, TAP_RESET, "Test-Logic-Reset state")
		);

SHELL_SUBCMD_DICT_SET_CREATE(sub_pin_cmds, cmd_sw_xfer,
		(TDI, JTAG_TDI, "Test Data In"),
		(TCK, JTAG_TCK, "Test Clock"),
		(TMS, JTAG_TMS, "Test Mode Select"),
		(TRST, JTAG_TRST, "Test Reset")
		);

	SHELL_STATIC_SUBCMD_SET_CREATE(sub_action_cmds,
			SHELL_CMD(high, &sub_pin_cmds, "<pin>", NULL),
			SHELL_CMD(low, &sub_pin_cmds, "<pin>", NULL),
			SHELL_SUBCMD_SET_END
			);

	SHELL_STATIC_SUBCMD_SET_CREATE(
			sub_jtag_cmds,
			SHELL_CMD_ARG(frequency, NULL, "<frequency>", cmd_frequency, 2,
				0),
			SHELL_CMD_ARG(ir_scan, NULL, "<len> <hex value, any width>", cmd_ir_scan, 3,
				0),
			SHELL_CMD_ARG(dr_scan, NULL, "<len> <hex value, any width>", cmd_dr_scan, 3,
				0),
			SHELL_CMD(tap_set, &sub_tap_cmds, "<tap_state>", NULL),
			SHELL_CMD(sw_xfer, &sub_action_cmds, "<high/low> <pin>", NULL),
			SHELL_SUBCMD_SET_END);

static void cmd_jtag_dev_get(size_t idx, struct shell_static_entry *entry)
{
	const struct device *dev = shell_device_lookup(idx, JTAG_DEVICE_PREFIX);

	if (dev == NULL) {
		entry->syntax = NULL;
		entry->handler = NULL;
		entry->help = NULL;
		entry->subcmd = NULL;
		return;
	}

	printk("idx=%zu, jtag device=%s\n", idx, dev->name);
	entry->syntax = (dev != NULL) ? dev->name : NULL;
	entry->handler = NULL;
	entry->help = "Select JTAG device for subcommand.\n";
	entry->subcmd = &sub_jtag_cmds;
}
SHELL_DYNAMIC_CMD_CREATE(sub_jtag_dev, cmd_jtag_dev_get);

SHELL_CMD_REGISTER(jtag, &sub_jtag_dev, "JTAG shell commands", NULL);
