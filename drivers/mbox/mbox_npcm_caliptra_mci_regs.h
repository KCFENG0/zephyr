/*
 * Copyright (c) 2026 Nuvoton Technology Corporation
 * SPDX-License-Identifier: Apache-2.0
 *
 * Caliptra MCU_MBOX CSR offsets, relative to the "csr" reg region.
 */
#ifndef ZEPHYR_DRIVERS_MBOX_MBOX_NPCM_CALIPTRA_MCI_REGS_H_
#define ZEPHYR_DRIVERS_MBOX_MBOX_NPCM_CALIPTRA_MCI_REGS_H_

#define CALIPTRA_MCI_MBOX_REG_LOCK              0x00U
#define CALIPTRA_MCI_MBOX_REG_USER              0x04U
#define CALIPTRA_MCI_MBOX_REG_TARGET_USER       0x08U
#define CALIPTRA_MCI_MBOX_REG_TARGET_USER_VALID 0x0CU
#define CALIPTRA_MCI_MBOX_REG_CMD               0x10U
#define CALIPTRA_MCI_MBOX_REG_DLEN              0x14U
#define CALIPTRA_MCI_MBOX_REG_EXECUTE           0x18U
#define CALIPTRA_MCI_MBOX_REG_TARGET_STATUS     0x1CU
#define CALIPTRA_MCI_MBOX_REG_CMD_STATUS        0x20U
#define CALIPTRA_MCI_MBOX_REG_HW_STATUS         0x24U

#define CALIPTRA_MCI_MBOX_LOCK_BIT              (1U << 0)
#define CALIPTRA_MCI_MBOX_EXECUTE_BIT           (1U << 0)
#define CALIPTRA_MCI_MBOX_TARGET_USER_VALID_BIT (1U << 0)

/*
 * MCI bus-id control register, absolute address (outside the per-mailbox CSR
 * window). The mailbox HW decodes the requester's USER from this register on
 * lock acquire; if M7's BUSID does not match a MCU-provisioned slot, all
 * subsequent CSR/SRAM writes from the M7 are silently filtered.
 */
#define CALIPTRA_MCI_BUSID_CTL0_ADDR 0x400C30C0U

/* AXI USER conventionally provisioned by the MCU ROM for the M7 SoC side.
 *
 * IMPORTANT: This is also the AXI USER ID baked into the M7's bus master.
 * The mailbox/MCI hardware accepts an M7 access only if (a) the M7 has set
 * its BUSID to this value (driver init does this at POST_KERNEL) AND (b) the
 * MCU has provisioned this same value into one of the MCI valid_axi_user
 * slots and locked them. Any M7 access to mailbox CSR or SRAM before MCU
 * provisioning completes bus-faults - this is the dual-CPU boot constraint
 * the M7 must respect. See CONFIG_MBOX_SVC_MCU_WARMUP_MS in the test app
 * Kconfig for the workaround pattern.
 */
#define CALIPTRA_MCI_MBOX_USER_SOC 0x00000001U

#endif /* ZEPHYR_DRIVERS_MBOX_MBOX_NPCM_CALIPTRA_MCI_REGS_H_ */
