#
# Copyright (c) 2024 Nuvoton Technology Corporation.
#
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(
  .
  ${ZEPHYR_BASE}/drivers
)

zephyr_sources(
  soc.c
  gdma.c
  reboot.c
)

zephyr_sources_ifdef(
  CONFIG_ARM_MPU
  mpu_regions.c
)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")
