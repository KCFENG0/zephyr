/*
 * Copyright (c) 2021 ASPEED Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief tinycrypt driver context info
 *
 * The file defines the structure which is used to store per session context
 * by the driver. Placed in common location so that crypto applications
 * can allocate memory for the required number of sessions, to free driver
 * from dynamic memory allocation.
 */

#ifndef ZEPHYR_DRIVERS_CRYPTO_HASH_ASPEED_PRIV_H_
#define ZEPHYR_DRIVERS_CRYPTO_HASH_ASPEED_PRIV_H_

#include <kernel.h>

struct aspeed_sg {
	uint32_t len;
	uint32_t addr;
};

struct aspeed_hash_ctx {
	struct aspeed_sg sg[2]; /* Must be 8 byte aligned */
	uint8_t digest[64]; /* Must be 8 byte aligned */
	uint32_t method;
	uint32_t block_size;
	uint64_t digcnt[2]; /* total length */
	uint32_t bufcnt;
	uint8_t buffer[256];
};

struct aspeed_hash_drv_state {
	struct aspeed_hash_ctx data;
	bool in_use;
};

#endif  /* ZEPHYR_DRIVERS_CRYPTO_HASH_ASPEED_PRIV_H_ */
