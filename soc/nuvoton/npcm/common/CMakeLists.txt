#
# Copyright (c) 2024 Nuvoton Technology Corporation.
#
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(npcm_common)

zephyr_include_directories(.)
zephyr_library_sources(
  hook.c
)

zephyr_linker_sources(DATA_SECTIONS rom_hooks.ld)

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
