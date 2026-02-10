/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_SHM_NPCM_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_SHM_NPCM_H_

#include <soc.h>

#define SHM_RD_IE_FLAG 0x01
#define SHM_WR_IE_FLAG 0x02
typedef enum{
	SHM_WIN_SIZE_8B = 0x03,
	SHM_WIN_SIZE_16B,
	SHM_WIN_SIZE_32B,
	SHM_WIN_SIZE_64B,
	SHM_WIN_SIZE_128B,
	SHM_WIN_SIZE_256B,
	SHM_WIN_SIZE_512B,
	SHM_WIN_SIZE_1KB,
	SHM_WIN_SIZE_2KB,
	SHM_WIN_SIZE_4KB,
}SHM_WIN_SIZE_T;

typedef enum {
	SHM_WIN1 = 0,
	SHM_WIN2,
	SHM_WIN3,
	SHM_WIN4,
	SHM_WIN5,
	SHM_IMA_WIN1,
	SHM_IMA_WIN2,
	SHM_AIMA_WIN,
} SHM_DEVICE_T;

typedef struct {
	uint32_t *base;
	uint8_t size;
	uint8_t wr_protect, rd_protect;
	uint16_t offset;
} SHM_Mod_T;

/*
 * callback to handle snoop RX data
 * @snoop: pointer to the byte snooped by channel 0, NULL if no data
 * @len: data len
 */
typedef void snoop_npcm_rx_callback_t(const uint8_t *snoop, int len);

void shm_npcm_set_win_size(uint32_t win, uint8_t size);
void shm_npcm_set_memory(uint8_t win, SHM_Mod_T *shm_mod);
void shm_npcm_offset_intr(uint8_t win, uint8_t flags, uint8_t enable);
void shm_npcm_enable_sem(uint8_t win);
void shm_npcm_enable_sem_intr(uint8_t win);
int port80_npcm_read(uint8_t *port, uint8_t *bytelen, uint8_t *data);
int snoop_npcm_register_rx_callback(const struct device *dev, snoop_npcm_rx_callback_t cb);

#endif