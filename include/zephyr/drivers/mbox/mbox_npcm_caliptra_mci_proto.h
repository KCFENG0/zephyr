/*
 * Copyright (c) 2026 Nuvoton Technology Corporation
 * SPDX-License-Identifier: Apache-2.0
 *
 * Hardware-free protocol logic for the Caliptra MCU_MBOX driver.
 * Kept separate so it can be unit-tested on native_sim.
 */
#ifndef ZEPHYR_DRIVERS_MBOX_MBOX_NPCM_CALIPTRA_MCI_PROTO_H_
#define ZEPHYR_DRIVERS_MBOX_MBOX_NPCM_CALIPTRA_MCI_PROTO_H_

#include <stdint.h>
#include <stddef.h>
#include <errno.h>

/* TARGET_STATUS / CMD_STATUS register field layout. */
#define CALIPTRA_MCI_MBOX_STATUS_CODE_MASK 0x0FU
#define CALIPTRA_MCI_MBOX_STATUS_DONE_BIT  (1U << 4)

/* HW_STATUS register field layout. */
#define CALIPTRA_MCI_MBOX_HW_ECC_SINGLE_BIT (1U << 0)
#define CALIPTRA_MCI_MBOX_HW_ECC_DOUBLE_BIT (1U << 1)

enum caliptra_mci_mbox_status {
	CALIPTRA_MCI_MBOX_BUSY = 0,
	CALIPTRA_MCI_MBOX_DATA_READY = 1,
	CALIPTRA_MCI_MBOX_COMPLETE = 2,
	CALIPTRA_MCI_MBOX_FAILURE = 3,
};

enum caliptra_mci_mbox_state {
	CALIPTRA_MCI_MBOX_STATE_IDLE = 0,
	CALIPTRA_MCI_MBOX_STATE_LOCKED,
	CALIPTRA_MCI_MBOX_STATE_STAGED,
	CALIPTRA_MCI_MBOX_STATE_BUSY,
	/*
	 * Responder-only: reply has been committed (TARGET_STATUS.DONE
	 * written). Waits for the requester (MCU) to clear EXECUTE,
	 * which closes the transaction and triggers HW SRAM zeroization.
	 */
	CALIPTRA_MCI_MBOX_STATE_CLOSING,
};

/*
 * Channel-state events. EV_ACQUIRE means "lock obtained" on a requester
 * channel and "request received" on a responder channel - both move
 * IDLE -> LOCKED. EV_RESPOND and EV_CLOSED are responder-only.
 */
enum caliptra_mci_mbox_event {
	CALIPTRA_MCI_MBOX_EV_ACQUIRE = 0,
	CALIPTRA_MCI_MBOX_EV_STAGE,
	CALIPTRA_MCI_MBOX_EV_SEND,
	CALIPTRA_MCI_MBOX_EV_COMPLETE,
	CALIPTRA_MCI_MBOX_EV_RELEASE,
	CALIPTRA_MCI_MBOX_EV_RESPOND, /* responder: LOCKED -> CLOSING */
	CALIPTRA_MCI_MBOX_EV_CLOSED,  /* responder: CLOSING -> IDLE  */
};

/* Decode a TARGET_STATUS/CMD_STATUS register value into a status enum. */
static inline enum caliptra_mci_mbox_status caliptra_mci_mbox_decode_status(uint32_t reg)
{
	return (enum caliptra_mci_mbox_status)(reg & CALIPTRA_MCI_MBOX_STATUS_CODE_MASK);
}

/* True once the peer (responder) has set the DONE bit in TARGET_STATUS. */
static inline int caliptra_mci_mbox_status_done(uint32_t reg)
{
	return (reg & CALIPTRA_MCI_MBOX_STATUS_DONE_BIT) != 0;
}

/*
 * Requester-side completion check. Per spec (SOC Sender -> MCU flow step 8),
 * the MCU only writes a status code to CMD_STATUS - there is no DONE bit on
 * the requester side. Any value other than BUSY (0) means the MCU has replied.
 */
static inline int caliptra_mci_mbox_cmd_status_responded(uint32_t reg)
{
	return caliptra_mci_mbox_decode_status(reg) != CALIPTRA_MCI_MBOX_BUSY;
}

/*
 * Validate a payload length against the integrated SRAM size.
 * Returns 0 if ok, -EINVAL if not dword-aligned, -ENOSPC if too large.
 */
static inline int caliptra_mci_mbox_check_len(size_t len, size_t sram_size)
{
	if ((len & 0x3U) != 0U) {
		return -EINVAL;
	}
	if (len > sram_size) {
		return -ENOSPC;
	}
	return 0;
}

/*
 * Advance the per-channel state machine. On a legal transition writes
 * *out and returns 0. On an illegal transition returns -EPERM and leaves
 * *out unchanged.
 */
static inline int caliptra_mci_mbox_state_next(enum caliptra_mci_mbox_state cur,
					   enum caliptra_mci_mbox_event ev,
					   enum caliptra_mci_mbox_state *out)
{
	enum caliptra_mci_mbox_state next;

	switch (ev) {
	case CALIPTRA_MCI_MBOX_EV_ACQUIRE:
		if (cur != CALIPTRA_MCI_MBOX_STATE_IDLE) {
			return -EPERM;
		}
		next = CALIPTRA_MCI_MBOX_STATE_LOCKED;
		break;
	case CALIPTRA_MCI_MBOX_EV_STAGE:
		if (cur != CALIPTRA_MCI_MBOX_STATE_LOCKED) {
			return -EPERM;
		}
		next = CALIPTRA_MCI_MBOX_STATE_STAGED;
		break;
	case CALIPTRA_MCI_MBOX_EV_SEND:
		if (cur != CALIPTRA_MCI_MBOX_STATE_STAGED) {
			return -EPERM;
		}
		next = CALIPTRA_MCI_MBOX_STATE_BUSY;
		break;
	case CALIPTRA_MCI_MBOX_EV_COMPLETE:
		if (cur != CALIPTRA_MCI_MBOX_STATE_BUSY) {
			return -EPERM;
		}
		next = CALIPTRA_MCI_MBOX_STATE_LOCKED;
		break;
	case CALIPTRA_MCI_MBOX_EV_RESPOND:
		if (cur != CALIPTRA_MCI_MBOX_STATE_LOCKED) {
			return -EPERM;
		}
		next = CALIPTRA_MCI_MBOX_STATE_CLOSING;
		break;
	case CALIPTRA_MCI_MBOX_EV_CLOSED:
		if (cur != CALIPTRA_MCI_MBOX_STATE_CLOSING) {
			return -EPERM;
		}
		next = CALIPTRA_MCI_MBOX_STATE_IDLE;
		break;
	case CALIPTRA_MCI_MBOX_EV_RELEASE:
		next = CALIPTRA_MCI_MBOX_STATE_IDLE;
		break;
	default:
		return -EINVAL;
	}

	*out = next;
	return 0;
}

#endif /* ZEPHYR_DRIVERS_MBOX_MBOX_NPCM_CALIPTRA_MCI_PROTO_H_ */
