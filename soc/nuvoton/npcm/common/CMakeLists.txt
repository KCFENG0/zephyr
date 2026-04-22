#
# Copyright (c) 2024 Nuvoton Technology Corporation.
#
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(npcm_common)

zephyr_include_directories(.)
zephyr_library_sources(
  hook.c
)

if(NOT CONFIG_XIP)
  # For non-XIP builds, place .rom_hooks before bss to avoid objcopy
  # filling the NOBITS gap with padding (~140 KB saving).
  # For XIP builds, .rom_hooks must stay in the SoC linker.ld overlay
  # because GROUP_LINK_IN(ROMABLE_REGION) inside DATA_SECTIONS would
  # switch the location counter to flash, corrupting __data_region_end.
  zephyr_linker_sources(DATA_SECTIONS rom_hooks.ld)
endif()

# NPCM4 still uses the legacy esiost booter header (${KERNEL_BIN_NAME}_signed.bin).
# NPCM5 consumes the raw ${KERNEL_BIN_NAME} downstream and the application wraps
# it in the Caliptra flash-image format, so no SoC-level signing is needed here.
if(CONFIG_SOC_SERIES_NPCM4)
  set(NPCM_BIN_NAME ${CONFIG_KERNEL_BIN_NAME}_signed.bin)
  set(NPCM_ELF_NAME ${CONFIG_KERNEL_BIN_NAME}.elf)
  string(TOUPPER "${SOC_NAME}" soc_name_upper)

  set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
    COMMAND ${PYTHON_EXECUTABLE} ${SOC_${soc_name_upper}_DIR}/common/esiost/esiost.py
    -i ${KERNEL_BIN_NAME}
    -elf ${NPCM_ELF_NAME}
    -o ${NPCM_BIN_NAME}
    -v
    )
endif()
