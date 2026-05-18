/*
 * Copyright (c) 2026 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/arm/mpu/arm_mpu.h>
#include <zephyr/arch/arm/cortex_m/arm_mpu_mem_cfg.h>

/* NPCM500 has no DTCM at the architectural Cortex-M7 DTCM base (0x20000000):
 * the chip's real SRAM lives at 0x10008000.
 *
 * The ARMv7-M default attribute for 0x20000000-0x3FFFFFFF is Normal cacheable,
 * Override it to Device (same default as the 0x40000000 peripheral region).
 */
#define REGION_DEVICE_512M_ATTR \
	{ (DEVICE_NON_SHAREABLE | NOT_EXEC | REGION_512M | P_RW_U_NA_Msk) }

static const struct arm_mpu_region mpu_regions[] = {
	MPU_REGION_ENTRY("FLASH_0",
			 CONFIG_FLASH_BASE_ADDRESS,
			 REGION_FLASH_ATTR(REGION_FLASH_SIZE)),
	MPU_REGION_ENTRY("SRAM_0",
			 CONFIG_SRAM_BASE_ADDRESS,
			 REGION_RAM_ATTR(REGION_SRAM_SIZE)),
	MPU_REGION_ENTRY("DTCM_ALIAS_DEVICE",
			 0x20000000,
			 REGION_DEVICE_512M_ATTR),
};

const struct arm_mpu_config mpu_config = {
	.num_regions = ARRAY_SIZE(mpu_regions),
	.mpu_regions = mpu_regions,
};
