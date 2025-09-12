/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_SHM_NPCM_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_SHM_NPCM_H_

#include <soc.h>

int port80_npcm_read(uint8_t *port, uint8_t *bytelen, uint8_t *data);

#endif