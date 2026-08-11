/*
 * Copyright (c) 2026 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * NPCM One-Time-Programmable memory controller registers.
 *
 * Transcribed from the NPCM5MNX datasheet section 4.8 "One Time Programmable
 * Memory Interface (OTP)". All registers are byte wide.
 */

#ifndef ZEPHYR_DRIVERS_OTP_OTP_NPCM_REGS_H_
#define ZEPHYR_DRIVERS_OTP_OTP_NPCM_REGS_H_

#include <stdint.h>

#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

/* Register offsets from the OTP block base */
#define NPCM_OTP_CTL    0x00U
#define NPCM_OTP_ADDR_H 0x01U
#define NPCM_OTP_ADDR_L 0x02U
#define NPCM_OTP_ECODE  0x03U
#define NPCM_OTP_STS    0x04U
#define NPCM_OTP_CLKDIV 0x05U
#define NPCM_OTP_DOUT   0x07U
#define NPCM_OTP_LOCK   0x08U
#define NPCM_OTP_LOCK2  0x09U
#define NPCM_OTP_KEYSTS 0x1EU
#define NPCM_OTP_VER    0x1FU

/* OTP_CTL fields. RDEN and PGEN are self-clearing; OTPRDY is read-only. */
#define NPCM_OTP_CTL_RDEN   BIT(0)
#define NPCM_OTP_CTL_PGEN   BIT(1)
#define NPCM_OTP_CTL_DEEPEN BIT(2)
#define NPCM_OTP_CTL_OTPRDY BIT(6)
#define NPCM_OTP_CTL_ENOTP  BIT(7)

/* OTP_ADDR_H holds OTP_ADDR[9:5]; bits 7-5 are reserved. */
#define NPCM_OTP_ADDR_H_MASK  0x1FU
#define NPCM_OTP_ADDR_H_SHIFT 5

/* OTP_ADDR_L holds OTP_ADDR[4:0] in bits 7-3 and the program bit select in 2-0. */
#define NPCM_OTP_ADDR_L_ADDR_MASK  0xF8U
#define NPCM_OTP_ADDR_L_ADDR_SHIFT 3
#define NPCM_OTP_ADDR_L_PGBIT_MASK 0x07U

/*
 * Writing OTP_ECODE with the entry code enters OTP operation mode; writing any
 * other value leaves it. The datasheet names only the entry value, so the exit
 * value below is an arbitrary non-entry constant.
 */
#define NPCM_OTP_ECODE_ENTRY 0x66U
#define NPCM_OTP_ECODE_EXIT  0x00U

/* OTP_STS fields, all read-only and cleared by hardware. */
#define NPCM_OTP_STS_PERROR    BIT(0)
#define NPCM_OTP_STS_PGDSTS    BIT(1)
#define NPCM_OTP_STS_RDDSTS    BIT(2)
#define NPCM_OTP_STS_PWE       BIT(3)
#define NPCM_OTP_STS_PRD       BIT(6)
#define NPCM_OTP_STS_ENTRY_OTP BIT(7)

/* OTP_CLKDIV divides APB1_CLK down to OTP_CLK. Zero is unsupported. */
#define NPCM_OTP_CLKDIV_MASK 0x7FU

/*
 * OTP_LOCK and OTP_LOCK2 are typed "first write after Core Domain reset is an
 * unrestricted write, every later write is write-1-to-set". Bits can therefore
 * never be cleared once the register has been written, and a Core Domain reset
 * restores them -- they are sticky registers, not fuses.
 */
#define NPCM_OTP_LOCK_RGN0_WR   BIT(0)
#define NPCM_OTP_LOCK_RGN1_WR   BIT(1)
#define NPCM_OTP_LOCK_RGN2_WR   BIT(2)
#define NPCM_OTP_LOCK_RGN0_RW   BIT(3)
#define NPCM_OTP_LOCK_RGN1_RW   BIT(4)
#define NPCM_OTP_LOCK_RGN2_RW   BIT(5)
#define NPCM_OTP_LOCK_SESSKEY_RW BIT(6)
/* Bit 7 (UnmapROM/ROM_DIS) unmaps the boot ROM and is deliberately not used. */

#define NPCM_OTP_LOCK2_RGN3_WR BIT(0)
#define NPCM_OTP_LOCK2_RGN4_WR BIT(1)
#define NPCM_OTP_LOCK2_RGN5_WR BIT(2)
#define NPCM_OTP_LOCK2_RGN6_WR BIT(3)
#define NPCM_OTP_LOCK2_RGN3_RW BIT(4)
#define NPCM_OTP_LOCK2_RGN4_RW BIT(5)
#define NPCM_OTP_LOCK2_RGN5_RW BIT(6)
#define NPCM_OTP_LOCK2_RGN6_RW BIT(7)

/* OTP_KEYSTS fields, all read-only. */
#define NPCM_OTP_KEYSTS_KEYSTS  BIT(0)
#define NPCM_OTP_KEYSTS_INITSTS BIT(1)
#define NPCM_OTP_KEYSTS_KEYLOCK BIT(2)

/* OTP_VER reads back this value on the NPCM5MNX series. */
#define NPCM_OTP_VER_VALUE 0x02U

/* Target OTP_CLK frequency; OTP_CLKDIV is APB1_CLK divided down to this. */
#define NPCM_OTP_CLK_HZ 1000000U

/* Array size in bytes, and the bound OTP_ADDR can reach. */
#define NPCM_OTP_MAX_SIZE ((NPCM_OTP_ADDR_H_MASK + 1U) << NPCM_OTP_ADDR_H_SHIFT)

#endif /* ZEPHYR_DRIVERS_OTP_OTP_NPCM_REGS_H_ */
