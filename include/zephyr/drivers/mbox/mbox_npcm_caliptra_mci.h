/*
 * Copyright (c) 2026 Nuvoton Technology Corporation
 * SPDX-License-Identifier: Apache-2.0
 *
 * Side-channel API for the Caliptra MCU_MBOX driver. The standard mbox
 * subsystem API (mbox.h) handles the doorbell and callbacks; this header
 * carries the lock / cmd / dlen / SRAM access the mbox API cannot express.
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_MBOX_MBOX_NPCM_CALIPTRA_MCI_H_
#define ZEPHYR_INCLUDE_DRIVERS_MBOX_MBOX_NPCM_CALIPTRA_MCI_H_

#include <stddef.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/mbox/mbox_npcm_caliptra_mci_proto.h>

/*
 * Acquire the mailbox lock (requester role).
 * Returns 0 on success, -EBUSY if the lock is already held, -EPERM if the
 * channel is not in the IDLE state.
 */
int caliptra_mci_mbox_acquire(const struct device *dev);

/*
 * Release the lock by clearing EXECUTE. Always returns the channel to IDLE.
 */
void caliptra_mci_mbox_release(const struct device *dev);

/*
 * Stage a command and payload into CMD/DLEN/SRAM. Must hold the lock.
 * Returns 0, -EINVAL if len is not dword-aligned, -ENOSPC if len exceeds
 * sram-size, -EPERM if the channel has not acquired the lock.
 */
int caliptra_mci_mbox_write_payload(const struct device *dev, uint32_t cmd,
				const void *buf, size_t len);

/*
 * Read the peer's command and payload out of CMD/DLEN/SRAM.
 * Returns 0, -ENOSPC if DLEN exceeds cap. *out_len receives the byte count.
 */
int caliptra_mci_mbox_read_payload(const struct device *dev, uint32_t *cmd,
			       void *buf, size_t cap, size_t *out_len);

/*
 * Responder-side reply (spec "MCU to SOC Receiver" steps 8 + 9). Stages
 * the response payload into SRAM/DLEN, then sets TARGET_STATUS = DONE |
 * status to signal the requester. The driver then enters a CLOSING state
 * and waits (via dispatch) for the requester to clear EXECUTE.
 *
 * Must be called from the responder callback (channel = 1) while the
 * channel state is LOCKED. Returns 0, -EINVAL on bad alignment, -ENOSPC
 * on oversize, -EPERM on wrong state.
 */
int caliptra_mci_mbox_write_response(const struct device *dev,
				 enum caliptra_mci_mbox_status status,
				 const void *buf, size_t len);

/*
 * Return the decoded CMD_STATUS (requester) value.
 */
enum caliptra_mci_mbox_status caliptra_mci_mbox_get_status(const struct device *dev);

/*
 * Inspect HW_STATUS. Returns 0 if clean, -EIO on a double-bit ECC error,
 * 1 on a (correctable) single-bit ECC error.
 */
int caliptra_mci_mbox_check_ecc(const struct device *dev);

#endif /* ZEPHYR_INCLUDE_DRIVERS_MBOX_MBOX_NPCM_CALIPTRA_MCI_H_ */
