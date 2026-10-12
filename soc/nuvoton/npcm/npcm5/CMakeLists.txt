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
# flash_image.py builds the flash layout only -- partition table, flash-image
# container, no signing -- driven by a slot-table layout JSON. It synthesizes
# the NPCM boot header itself, so it consumes the raw kernel binary, not the
# esiost-signed one.
#
# The layout file decides which slots the image ships; flags only supply paths.
#
# Options:
#   -DFLASH_LAYOUT=<x>      image variant: a1 (default, SoC fw + MCU runtime +
#                           signed manifest) | a1-mcu (MCU fw, no manifest) | a2 (full Caliptra) | path to a
#                           project-local layout JSON
#   -DFLASH_FORMAT=<x>      container version: v1 (default, byte-identical to
#                           flash_image_tool.py gen-test-flash) | v3 (2.1.2+
#                           ROM, page-aligned)
#   -DMCU_FW=<path>         MCU firmware (default: bypass build, or the active
#                           build when the layout wants Caliptra -- active/ on
#                           FPGA, active-evb/ on silicon)
#   -DCALIPTRA_FW=<path>    signed Caliptra FMC+RT bundle (a2 layout)
#   -DSOC_MANIFEST=<path>   prebuilt SoC manifest; omit to generate and sign one
#   -DMANIFEST_SVN=<n>      security version of the generated manifest
#   -DMANIFEST_KEY_DIR=<d>  signing keys (default: caliptra development keys)
#   -DFLASH_CONNECTION=<x>  internal (default) | external-ab | external
#   -DFLASH_IMAGE_SIZE=<n>  pad to this flash size (default: sized to content)
#
# ponytail: -D cache vars, promote to Kconfig when a second board needs a
# different flash geometry.
#
string(TOUPPER "${SOC_NAME}" soc_name_upper)
set(FLASH_IMAGE_DIR  ${SOC_${soc_name_upper}_DIR}/common/caliptra_flash_image)
set(FLASH_IMAGE_TOOL ${FLASH_IMAGE_DIR}/flash_image.py)
set(caliptra_images_dir ${WEST_TOPDIR}/modules/caliptra/caliptra-mcu-sw/platforms/nuvoton/npcm500/images)
set(MCU_FW_BYPASS       ${caliptra_images_dir}/mcu-fw/bypass/npcm-mcu-fw.bin)
# The active runtime's console divisor is fixed at build time: the FPGA build
# assumes the 10 MHz FPGA clock and garbles the MCU log on silicon, so silicon
# boards take the image built without npcm500-fpga.
if(CONFIG_NPCM500_FPGA)
  set(mcu_fw_active_variant active)
  set(mcu_fw_active_features npcm500-compat,npcm500-fpga,caliptra-soc-release)
else()
  set(mcu_fw_active_variant active-evb)
  set(mcu_fw_active_features npcm500-compat,caliptra-soc-release)
endif()
set(MCU_FW_ACTIVE       ${caliptra_images_dir}/mcu-fw/${mcu_fw_active_variant}/npcm-mcu-fw.bin)
set(CALIPTRA_FW_DEFAULT ${caliptra_images_dir}/caliptra/caliptra-fw.bin)

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

if(NOT FLASH_FORMAT)
  set(FLASH_FORMAT v1)
endif()
if(NOT FLASH_FORMAT MATCHES "^(v1|v3)$")
  message(FATAL_ERROR "FLASH_FORMAT=${FLASH_FORMAT}: must be v1 or v3")
endif()

# The layout already decides the variant, so let it pick the default sources
# too. A layout with a caliptra-fw slot is a full-Caliptra (A2) image, which
# needs the active MCU runtime: the bypass build has no load-soc-fw or
# caliptra-soc-release support and would silently boot as A1.
file(READ ${flash_layout_json} flash_layout_contents)
string(FIND "${flash_layout_contents}" "\"caliptra-fw\"" caliptra_fw_slot)
if(caliptra_fw_slot GREATER -1)
  set(layout_is_caliptra TRUE)
else()
  set(layout_is_caliptra FALSE)
endif()

set(pack_args --layout ${flash_layout_json} --format ${FLASH_FORMAT})

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
elseif(layout_is_caliptra)
  set(MCU_FW ${MCU_FW_ACTIVE})
  if(NOT EXISTS ${MCU_FW})
    message(FATAL_ERROR
      "FLASH_LAYOUT=${FLASH_LAYOUT} needs the active MCU runtime:\n"
      "  ${MCU_FW}\n"
      "Build it from caliptra-mcu-sw with --features "
      "${mcu_fw_active_features}, or pass -DMCU_FW=<path>.\n"
      "The bypass runtime cannot perform the Caliptra boot flow.")
  endif()
else()
  set(MCU_FW ${MCU_FW_BYPASS})  # default source; absent is fine unless the layout wants it
endif()
if(EXISTS ${MCU_FW})
  list(APPEND pack_args --mcu-fw ${MCU_FW})
endif()

if(NOT CALIPTRA_FW AND layout_is_caliptra)
  set(CALIPTRA_FW ${CALIPTRA_FW_DEFAULT})
endif()
if(CALIPTRA_FW)
  if(NOT EXISTS ${CALIPTRA_FW})
    message(FATAL_ERROR
      "FLASH_LAYOUT=${FLASH_LAYOUT} needs the Caliptra FMC+RT bundle:\n"
      "  ${CALIPTRA_FW}\n"
      "Copy it from a caliptra-sw build, or pass -DCALIPTRA_FW=<path>.")
  endif()
  list(APPEND pack_args --caliptra-fw ${CALIPTRA_FW})
endif()

if(SOC_MANIFEST)
  list(APPEND pack_args --soc-manifest ${SOC_MANIFEST})
endif()

if(MANIFEST_SVN)
  list(APPEND pack_args --manifest-svn ${MANIFEST_SVN})
endif()

if(MANIFEST_KEY_DIR)
  list(APPEND pack_args --key-dir ${MANIFEST_KEY_DIR})
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

# Extract each embedded image to its own file for inspection, e.g.
#   python3 flash_image_tool.py extract input/NPCM5_flash.bin
set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
  COMMAND ${PYTHON_EXECUTABLE} ${FLASH_IMAGE_TOOL} extract
          ${CONFIG_KERNEL_BIN_NAME}_flash.bin --layout ${flash_layout_json}
          --out-dir ${CONFIG_KERNEL_BIN_NAME}_flash_extract
  )
