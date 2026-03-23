/*
 * Copyright (c) 2021 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_I3C_TARGET_MQUEUE_H_
#define ZEPHYR_INCLUDE_DRIVERS_I3C_TARGET_MQUEUE_H_

#include <zephyr/kernel.h>

int i3c_target_mqueue_write(const struct device *dev, uint8_t *buf, int len);
int i3c_target_mqueue_read(const struct device *dev, uint8_t *buf, int budget);

/**
 * @brief Blocking read - waits for a message to be available
 *
 * @param dev The SMQ device
 * @param buf Buffer to store the message
 * @param budget Maximum bytes to read
 * @param timeout Timeout for waiting (K_FOREVER for indefinite)
 * @return Number of bytes read, 0 if timeout, negative on error
 */
int i3c_target_mqueue_read_blocking(const struct device *dev, uint8_t *buf, int budget,
				    k_timeout_t timeout);

#endif
