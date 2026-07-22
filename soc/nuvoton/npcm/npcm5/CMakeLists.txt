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

#
# NPCM flash image (${CONFIG_KERNEL_BIN_NAME}_flash.bin).
#
# flash_image.py builds the flash layout only -- partition table, FLSH container,
# no signing -- driven by a slot-table layout JSON. It synthesizes the NPCM boot
# header itself, so it consumes the raw kernel binary, not the esiost-signed one.
#
# The layout file decides which slots the image ships; flags only supply paths.
#
# Options:
#   -DFLASH_LAYOUT=<x>     image variant: a1 (default, SoC fw only) | a1-mcu
#                          (adds MCU fw) | a2 (full Caliptra) | path to a
#                          project-local layout JSON
#   -DMCU_FW=<path>        MCU firmware (default: caliptra-mcu-sw bypass binary)
#   -DCALIPTRA_FW=<path>   signed Caliptra FMC+RT bundle (a2 layout)
#   -DSOC_MANIFEST=<path>  SoC manifest (a2 layout)
#   -DFLASH_CONNECTION=<x> internal (default) | external-ab | external
#   -DFLASH_IMAGE_SIZE=<n> pad to this flash size (default: sized to content)
#
# ponytail: -D cache vars, promote to Kconfig when a second board needs a
# different flash geometry.
#
string(TOUPPER "${SOC_NAME}" soc_name_upper)
set(FLASH_IMAGE_DIR  ${SOC_${soc_name_upper}_DIR}/common/caliptra_flash_image)
set(FLASH_IMAGE_TOOL ${FLASH_IMAGE_DIR}/flash_image.py)
set(MCU_FW_BYPASS    ${WEST_TOPDIR}/modules/caliptra/caliptra-mcu-sw/platforms/nuvoton/npcm500/images/mcu-fw/bypass/npcm-mcu-fw.bin)

# Layout selection: a name resolves under layouts/, a path is used as-is.
if(NOT FLASH_LAYOUT)
  set(FLASH_LAYOUT a1)
endif()
if(EXISTS ${FLASH_LAYOUT})
  set(flash_layout_json ${FLASH_LAYOUT})
else()
  set(flash_layout_json ${FLASH_IMAGE_DIR}/layouts/${FLASH_LAYOUT}.json)
endif()
if(NOT EXISTS ${flash_layout_json})
  message(FATAL_ERROR "FLASH_LAYOUT=${FLASH_LAYOUT}: ${flash_layout_json} does not exist")
endif()

set(pack_args --layout ${flash_layout_json})

# MCU_FW is only a path override; whether the image contains MCU firmware is the
# layout's call. The source is offered whenever its file exists and the packer
# consumes it only if the layout has an mcu-fw slot.
if(MCU_FW STREQUAL "ON")
  message(FATAL_ERROR "MCU_FW=ON is gone; select the variant with -DFLASH_LAYOUT=a1-mcu")
endif()
if(MCU_FW)
  if(NOT EXISTS ${MCU_FW})
    message(FATAL_ERROR "MCU_FW=${MCU_FW} does not exist")
  endif()
else()
  set(MCU_FW ${MCU_FW_BYPASS})  # default source; absent is fine unless the layout wants it
endif()
if(EXISTS ${MCU_FW})
  list(APPEND pack_args --mcu-fw ${MCU_FW})
endif()

if(CALIPTRA_FW)
  list(APPEND pack_args --caliptra-fw ${CALIPTRA_FW})
endif()

if(SOC_MANIFEST)
  list(APPEND pack_args --soc-manifest ${SOC_MANIFEST})
endif()

if(FLASH_CONNECTION)
  list(APPEND pack_args --flash-connection ${FLASH_CONNECTION})
endif()

if(FLASH_IMAGE_SIZE)
  list(APPEND pack_args --flash-size ${FLASH_IMAGE_SIZE})
endif()

set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
  COMMAND ${PYTHON_EXECUTABLE} ${FLASH_IMAGE_TOOL}
          --soc-fw ${KERNEL_BIN_NAME}
          ${pack_args}
          --output ${CONFIG_KERNEL_BIN_NAME}_flash.bin
  )

# Verify magic/version/checksums at the layout's slot offset; flash_image.py exits
# non-zero on a bad image, failing the build rather than shipping a corrupt one.
set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
  COMMAND ${PYTHON_EXECUTABLE} ${FLASH_IMAGE_TOOL} verify
          ${CONFIG_KERNEL_BIN_NAME}_flash.bin --layout ${flash_layout_json}
  )
