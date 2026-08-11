/*
 * Copyright (c) 2026 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Nuvoton NPCM extensions to the OTP driver API.
 *
 * Reading and programming go through the generic API in
 * <zephyr/drivers/otp.h>. This header covers what the generic API does not
 * express: the size of the array, and the controller's region protection.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_OTP_OTP_NPCM_H_
#define ZEPHYR_INCLUDE_DRIVERS_OTP_OTP_NPCM_H_

#include <stdbool.h>
#include <stddef.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Lockable OTP regions.
 *
 * Offsets 288-383 of the array are covered by no lock bit and therefore have
 * no entry here; they can be read and programmed but never protected.
 */
enum otp_npcm_region {
	OTP_NPCM_REGION_0 = 0,        /**< offsets 0-191 */
	OTP_NPCM_REGION_SESSION_CRED, /**< offsets 192-287, read/write lock only */
	OTP_NPCM_REGION_1,            /**< offsets 384-511 */
	OTP_NPCM_REGION_2,            /**< offsets 512-575 */
	OTP_NPCM_REGION_3,            /**< offsets 576-639 */
	OTP_NPCM_REGION_4,            /**< offsets 640-767 */
	OTP_NPCM_REGION_5,            /**< offsets 768-895 */
	OTP_NPCM_REGION_6,            /**< offsets 896-1023 */
	OTP_NPCM_REGION_COUNT,
};

/** @brief Protection to apply to a region. */
enum otp_npcm_lock_flags {
	/** Block programming; reads still succeed. */
	OTP_NPCM_LOCK_WRITE = 0,
	/** Block both programming and reads. */
	OTP_NPCM_LOCK_RW,
};

/**
 * @brief Number of array bytes this device may address.
 *
 * Taken from the node's @c size property. The generic OTP API has no
 * size query, so callers that need bounds must ask here.
 *
 * @param dev OTP device.
 *
 * @return Reachable array size in bytes.
 */
size_t otp_npcm_size(const struct device *dev);

/**
 * @brief Apply write or read/write protection to a region.
 *
 * @warning Protection cannot be removed. There is no unlock call, and none can
 *          exist: the lock registers ignore attempts to clear a bit once
 *          written. Protection is dropped only by a Core Domain reset, after
 *          which it must be applied again.
 *
 * Applying protection that is already in effect succeeds without doing
 * anything.
 *
 * Protection is independent of the addressable window: every region can be
 * locked regardless of the node's @c size property, so a region that
 * otp_read() and otp_program() cannot reach can still be protected against a
 * later owner that raises the size.
 *
 * @param dev    OTP device.
 * @param region Region to protect.
 * @param flags  Protection to apply.
 *
 * @retval 0 on success.
 * @retval -EINVAL @p region or @p flags is not a defined value, or @p region
 *         has no write-only lock bit (the session credential region supports
 *         @ref OTP_NPCM_LOCK_RW only).
 * @retval -EIO the lock bit did not read back as set.
 */
int otp_npcm_lock_region(const struct device *dev, enum otp_npcm_region region,
			 enum otp_npcm_lock_flags flags);

/**
 * @brief Report whether protection is in effect for a region.
 *
 * A read/write lock also prevents programming, so it reports as locked for
 * @ref OTP_NPCM_LOCK_WRITE as well as @ref OTP_NPCM_LOCK_RW.
 *
 * @param dev    OTP device.
 * @param region Region to check.
 * @param flags  Protection to check.
 *
 * @retval true the requested protection is in effect.
 * @retval false the requested protection is not in effect, or @p region or
 *         @p flags is not a defined value.
 */
bool otp_npcm_region_is_locked(const struct device *dev, enum otp_npcm_region region,
			       enum otp_npcm_lock_flags flags);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_OTP_OTP_NPCM_H_ */
