/*
 * Copyright (c) 2024 Yishai Jaffe <yishai1999@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/uart.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uart_shell, CONFIG_LOG_DEFAULT_LEVEL);

static int cmd_uart_write(const struct shell *sh, size_t argc, char **argv)
{
	char *s_dev_name = argv[1];
	const struct device *dev = device_get_binding(s_dev_name);

	if (dev == NULL) {
		shell_error(sh, "UART: Device driver %s not found.", s_dev_name);
		return -ENODEV;
	}

	char *buf = argv[2];
	int msg_len = strlen(buf);

	for (int i = 0; i < msg_len; i++) {
		uart_poll_out(dev, buf[i]);
	}

	return 0;
}

static int cmd_uart_baudrate(const struct shell *sh, size_t argc, char **argv)
{
	char *s_dev_name = argv[1];
	const struct device *dev;
	struct uart_config cfg;
	uint32_t baudrate;
	int ret;

	dev = device_get_binding(s_dev_name);
	if (dev == NULL) {
		shell_error(sh, "UART: Device driver %s not found.", s_dev_name);
		return -ENODEV;
	}

	baudrate = strtol(argv[2], NULL, 10);
	ret = uart_config_get(dev, &cfg);
	if (ret < 0) {
		shell_error(sh, "UART: Failed to get current configuration: %d", ret);
		return ret;
	}

	cfg.baudrate = baudrate;

	ret = uart_configure(dev, &cfg);
	if (ret < 0) {
		shell_error(sh, "UART: Failed to configure device: %d", ret);
		return ret;
	}
	return 0;
}

static int cmd_uart_conf(const struct shell *sh, size_t argc, char **argv)
{
	char *s_dev_name = argv[1];
	const struct device *dev;
	struct uart_config cfg;
	uint32_t baudrate;
	uint32_t databits;
	uint32_t parity;
	uint32_t stopbits;
	int ret;

	dev = device_get_binding(s_dev_name);
	if (dev == NULL) {
		shell_error(sh, "UART: Device driver %s not found.", s_dev_name);
		return -ENODEV;
	}

	baudrate = strtol(argv[2], NULL, 10);
	databits = strtol(argv[3], NULL, 10);
	parity = strtol(argv[4], NULL, 10);
	stopbits = strtol(argv[5], NULL, 10);

	if (databits == 7) {
		databits = UART_CFG_DATA_BITS_7;
	} else if (databits == 8) {
		databits = UART_CFG_DATA_BITS_8;
	} else if (databits == 9) {
		databits = UART_CFG_DATA_BITS_9;
	} else {
		shell_error(sh, "UART: Failed databits (7/8/9).");
		return -EINVAL;
	}

	if ((parity < 0) || (parity > 3)) {
		shell_error(sh, "UART: Failed parity (0~3).");
		return -EINVAL;
	}

	if (stopbits == 1) {
		stopbits = UART_CFG_STOP_BITS_1;
	} else if (stopbits == 2) {
		stopbits = UART_CFG_STOP_BITS_2;
	}
	else {
		shell_error(sh, "UART: Failed stopbits (1/2).");
		return -EINVAL;
	}

	ret = uart_config_get(dev, &cfg);
	if (ret < 0) {
		shell_error(sh, "UART: Failed to get current configuration: %d", ret);
		return ret;
	}

	cfg.baudrate = baudrate;
	cfg.data_bits = databits;
	cfg.parity = parity;
	cfg.stop_bits = stopbits;

	ret = uart_configure(dev, &cfg);
	if (ret < 0) {
		shell_error(sh, "UART: Failed to configure device: %d", ret);
		return ret;
	}

	return 0;
}

static int cmd_uart_flow_control(const struct shell *sh, size_t argc, char **argv)
{
	char *s_dev_name = argv[1];
	const struct device *dev;
	struct uart_config cfg;
	uint8_t flow_control;
	int ret;

	dev = device_get_binding(s_dev_name);
	if (dev == NULL) {
		shell_error(sh, "UART: Device driver %s not found.", s_dev_name);
		return -ENODEV;
	}

	if (!strcmp(argv[2], "none")) {
		flow_control = UART_CFG_FLOW_CTRL_NONE;
	} else if (!strcmp(argv[2], "rtscts")) {
		flow_control = UART_CFG_FLOW_CTRL_RTS_CTS;
	} else if (!strcmp(argv[2], "dtrdsr")) {
		flow_control = UART_CFG_FLOW_CTRL_DTR_DSR;
	} else if (!strcmp(argv[2], "rs485")) {
		flow_control = UART_CFG_FLOW_CTRL_RS485;
	} else {
		shell_error(sh, "Unknown: '%s'", argv[2]);
		shell_help(sh);
		return SHELL_CMD_HELP_PRINTED;
	}

	ret = uart_config_get(dev, &cfg);
	if (ret < 0) {
		shell_error(sh, "UART: Failed to get current configuration: %d", ret);
		return ret;
	}
	cfg.flow_ctrl = flow_control;

	ret = uart_configure(dev, &cfg);
	if (ret < 0) {
		shell_error(sh, "UART: Failed to configure device: %d", ret);
		return ret;
	}
	return 0;
}

static void device_name_get(size_t idx, struct shell_static_entry *entry)
{
	const struct device *dev = shell_device_lookup(idx, NULL);

	entry->syntax = (dev != NULL) ? dev->name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(dsub_device_name, device_name_get);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_uart_cmds,
	SHELL_CMD_ARG(write, &dsub_device_name,
		      "Write data to the UART device\n"
		      "Usage: write <device> <data>",
		      cmd_uart_write, 3, 0),
	SHELL_CMD_ARG(baudrate, &dsub_device_name,
		      "Configure UART device baudrate\n"
		      "Usage: baudrate <device> <baudrate>",
		      cmd_uart_baudrate, 3, 0),
	SHELL_CMD_ARG(conf, &dsub_device_name,
		      "Configure UART device conifg\n"
		      "Usage: uart conf <device> <baudrate> <databits> <parity> <stopbits>",
		      cmd_uart_conf, 6, 0),
	SHELL_CMD_ARG(fc, &dsub_device_name,
		      "Configure UART device flow control\n"
		      "Usage: fc <device> <none|rtscts|dtrdsr|rs485>",
		      cmd_uart_flow_control, 3, 0),
	SHELL_SUBCMD_SET_END     /* Array terminated. */
);

SHELL_CMD_REGISTER(uart, &sub_uart_cmds, "UART commands", NULL);
