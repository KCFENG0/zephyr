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
)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")

# NPCM400 boots the esiost-headered image; NPCM500 packs its own header in
# flash_image.py (see npcm5/CMakeLists.txt).
set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
  COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../common/esiost/esiost.py
  -i ${KERNEL_BIN_NAME}
  -elf ${CONFIG_KERNEL_BIN_NAME}.elf
  -o ${CONFIG_KERNEL_BIN_NAME}_signed.bin
  -v
  )
