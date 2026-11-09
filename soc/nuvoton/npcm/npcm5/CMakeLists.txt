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
#   -DFLASH_LAYOUT=<x>      boot flow, chip-neutral: bypass (default on A1; SoC
#                           fw + MCU runtime + signed manifest, gen-test-flash
#                           compatible) | bypass-mcu (MCU fw, no manifest) |
#                           bypass-soc (SoC fw only; the A1 default when the
#                           bypass MCU runtime is absent) | caliptra (full
#                           Caliptra boot; the default on A2) | path to a
#                           project-local layout JSON
#   -DFLASH_FORMAT=<x>      container version (default: the one the chip's MCU
#                           ROM parses): v1 (A1, byte-identical to
#                           flash_image_tool.py gen-test-flash) | v3 (A2 /
#                           2.1.2+ ROM, page-aligned)
#   -DMCU_FW=<path>         MCU firmware (default: the prebuilt runtime under
#                           mcu-fw/<chip>/<target>/<bypass|caliptra>/)
#   -DCALIPTRA_FW=<path>    signed Caliptra FMC+RT bundle (caliptra layout)
#   -DSOC_MANIFEST=<path>   prebuilt SoC manifest; omit to generate and sign one
#   -DMANIFEST_SVN=<n>      security version of the generated manifest
#   -DMANIFEST_KEY_DIR=<d>  signing keys (default: caliptra development keys)
#   -DFLASH_CONNECTION=<x>  internal (default) | external-ab | external
#   -DFLASH_IMAGE_SIZE=<n>  pad to this flash size (default: sized to content)
#
# The chip comes from the board revision (CONFIG_NPCM500_REV_A1/_A2) and the
# target from CONFIG_NPCM500_FPGA; the layout only names the boot flow.
#
# ponytail: -D cache vars, promote to Kconfig when a second board needs a
# different flash geometry.
#
string(TOUPPER "${SOC_NAME}" soc_name_upper)
set(FLASH_IMAGE_DIR  ${SOC_${soc_name_upper}_DIR}/common/caliptra_flash_image)
set(FLASH_IMAGE_TOOL ${FLASH_IMAGE_DIR}/flash_image.py)
set(caliptra_images_dir ${WEST_TOPDIR}/modules/caliptra/caliptra-mcu-sw/platforms/nuvoton/npcm500/images)
set(CALIPTRA_FW_DEFAULT ${caliptra_images_dir}/caliptra/caliptra-fw.bin)

# The chip decides the container format its MCU ROM parses and the boot flows it
# has (A2 boots only through Caliptra). The target decides which prebuilt MCU
# runtime fits: an npcm500-fpga build fixes the console divisor for the 10 MHz
# FPGA clock and garbles the MCU log on silicon, and the other way round.
if(CONFIG_NPCM500_REV_A2)
  set(npcm500_chip a2)
  set(chip_format v3)
  set(chip_flows caliptra)
else()
  set(npcm500_chip a1)
  set(chip_format v1)
  set(chip_flows bypass caliptra)
endif()
if(CONFIG_NPCM500_FPGA)
  set(npcm500_target fpga)
else()
  set(npcm500_target evb)
endif()
set(mcu_fw_dir ${caliptra_images_dir}/mcu-fw/${npcm500_chip}/${npcm500_target})

# Layout names used to carry the chip (<old>:<new>:<chip>). Keep them working
# where they agree with CONFIG_NPCM500_REV_*, and name the replacement either way.
set(retired_layouts a1:bypass:a1 a1-mcu:bypass-mcu:a1 a1-soc:bypass-soc:a1
                    a1-caliptra:caliptra:a1 a2:caliptra:a2 b1:caliptra:a2)
foreach(entry ${retired_layouts})
  string(REPLACE ":" ";" entry ${entry})
  list(GET entry 0 old_name)
  list(GET entry 1 new_name)
  list(GET entry 2 old_chip)
  if(FLASH_LAYOUT STREQUAL old_name)
    string(TOUPPER ${old_chip} old_chip_upper)
    if(NOT old_chip STREQUAL npcm500_chip)
      message(FATAL_ERROR "FLASH_LAYOUT=${old_name} is the ${old_chip_upper} ${new_name} layout; "
                          "build for ${BOARD}@${old_chip} and use -DFLASH_LAYOUT=${new_name}")
    endif()
    message(DEPRECATION "FLASH_LAYOUT=${old_name} is now ${new_name}; the chip comes from "
                        "the board revision (${BOARD}@${npcm500_chip})")
    set(FLASH_LAYOUT ${new_name})
  endif()
endforeach()

# A2 has only the Caliptra flow. On A1 the default bypass image packs the bypass
# MCU runtime from the caliptra-mcu-sw module; without the module, fall back to
# the SoC-only variant, which the A1 bypass ROM boots directly, instead of failing.
if(NOT FLASH_LAYOUT)
  list(GET chip_flows 0 FLASH_LAYOUT)
  if(FLASH_LAYOUT STREQUAL "bypass" AND NOT MCU_FW AND NOT EXISTS ${mcu_fw_dir}/bypass/npcm-mcu-fw.bin)
    set(FLASH_LAYOUT bypass-soc)
    message(STATUS "Flash image: no bypass MCU runtime under ${mcu_fw_dir}, "
                   "packing the SoC-only bypass-soc layout")
  endif()
endif()
if(EXISTS ${FLASH_LAYOUT})
  set(flash_layout_json ${FLASH_LAYOUT})
else()
  set(flash_layout_json ${FLASH_IMAGE_DIR}/layouts/${FLASH_LAYOUT}.json)
endif()
if(NOT EXISTS ${flash_layout_json})
  message(FATAL_ERROR "FLASH_LAYOUT=${FLASH_LAYOUT}: ${flash_layout_json} does not exist")
endif()

# The layout already decides the variant, so let it pick the default sources
# too. A layout with a caliptra-fw slot is a full-Caliptra image, which needs
# the Caliptra MCU runtime: the bypass build has no load-soc-fw or
# caliptra-soc-release support and would silently boot as bypass.
file(READ ${flash_layout_json} flash_layout_contents)
string(FIND "${flash_layout_contents}" "\"caliptra-fw\"" caliptra_fw_slot)
if(caliptra_fw_slot GREATER -1)
  set(layout_is_caliptra TRUE)
  set(layout_flow caliptra)
else()
  set(layout_is_caliptra FALSE)
  set(layout_flow bypass)
endif()
if(NOT layout_flow IN_LIST chip_flows)
  message(FATAL_ERROR "FLASH_LAYOUT=${FLASH_LAYOUT} is a ${layout_flow} image, but "
                      "${npcm500_chip} boots only through: ${chip_flows}")
endif()
set(MCU_FW_DEFAULT ${mcu_fw_dir}/${layout_flow}/npcm-mcu-fw.bin)

if(NOT FLASH_FORMAT)
  set(FLASH_FORMAT ${chip_format})
endif()
if(NOT FLASH_FORMAT MATCHES "^(v1|v3)$")
  message(FATAL_ERROR "FLASH_FORMAT=${FLASH_FORMAT}: must be v1 or v3")
endif()

set(pack_args --layout ${flash_layout_json} --format ${FLASH_FORMAT})

# MCU_FW is only a path override; whether the image contains MCU firmware is the
# layout's call. The source is offered whenever its file exists and the packer
# consumes it only if the layout has an mcu-fw slot.
if(MCU_FW STREQUAL "ON")
  message(FATAL_ERROR "MCU_FW=ON is gone; select the variant with -DFLASH_LAYOUT=bypass-mcu")
endif()
if(MCU_FW)
  if(NOT EXISTS ${MCU_FW})
    message(FATAL_ERROR "MCU_FW=${MCU_FW} does not exist")
  endif()
elseif(layout_is_caliptra)
  set(MCU_FW ${MCU_FW_DEFAULT})
  if(NOT EXISTS ${MCU_FW})
    message(FATAL_ERROR
      "FLASH_LAYOUT=${FLASH_LAYOUT} needs the Caliptra MCU runtime:\n"
      "  ${MCU_FW}\n"
      "Build npcm-mcu-fw from caliptra-mcu-sw with caliptra-soc-release (its "
      "release_note lists the exact features), or pass -DMCU_FW=<path>.\n"
      "The bypass runtime cannot perform the Caliptra boot flow.")
  endif()
else()
  set(MCU_FW ${MCU_FW_DEFAULT})  # default source; absent is fine unless the layout wants it
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
