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
#   -DFLASH_LAYOUT=<x>      boot flow, chip-neutral: bypass (A1 only, its
#                           default; SoC fw + NPCM header only, needs nothing
#                           from caliptra-mcu-sw) | caliptra (full Caliptra
#                           boot; A1, and A2's only flow) | path to a
#                           project-local layout JSON
#   -DFLASH_FORMAT=<x>      container version (default: the one the chip's MCU
#                           ROM parses): v1 (A1, byte-identical to
#                           flash_image_tool.py gen-test-flash) | v3 (A2 /
#                           2.1.2+ ROM, page-aligned)
#   -DMCU_FW=<path>         MCU firmware for a layout with an mcu-fw slot
#                           (default: the prebuilt runtime under
#                           mcu-fw/<chip>/<target>/caliptra/); bypass has none
#   -DCALIPTRA_FW=<path>    signed Caliptra FMC+RT bundle (caliptra layout;
#                           default: the prebuilt one under
#                           caliptra/<chip>/<target>/)
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

# The chip decides the container format its MCU ROM parses and the boot flows it
# has (A2 boots only through Caliptra). The target decides which prebuilt MCU
# runtime fits: an npcm500-fpga build fixes the console divisor for the 10 MHz
# FPGA clock and garbles the MCU log on silicon, and the other way round.
# Each chip has its own Caliptra runtime, and the SoC manifest must be sized
# for it: A1's predates caliptra-sw #3824 (127 metadata entries), A2's has it
# (80).
if(CONFIG_NPCM500_REV_A2)
  set(npcm500_chip a2)
  set(chip_format v3)
  set(chip_flows caliptra)
  set(chip_manifest_entries 80)
else()
  set(npcm500_chip a1)
  set(chip_format v1)
  set(chip_flows bypass caliptra)
  set(chip_manifest_entries 127)
endif()
if(CONFIG_NPCM500_FPGA)
  set(npcm500_target fpga)
else()
  set(npcm500_target evb)
endif()
set(mcu_fw_dir ${caliptra_images_dir}/mcu-fw/${npcm500_chip}/${npcm500_target})
# Per target: the FPGA needs a Caliptra RT built with fpga_realtime (no
# fixed-address DMA bursts), silicon the regular one.
set(CALIPTRA_FW_DEFAULT
    ${caliptra_images_dir}/caliptra/${npcm500_chip}/${npcm500_target}/caliptra-fw.bin)

# Layout names used to carry the chip (<old>:<new>:<chip>). Keep them working
# where they agree with CONFIG_NPCM500_REV_*, and name the replacement either way.
set(retired_layouts a1:bypass:a1 a1-caliptra:caliptra:a1 a2:caliptra:a2 b1:caliptra:a2)
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

# The chip's first flow is its default: bypass on A1, caliptra on A2.
if(NOT FLASH_LAYOUT)
  list(GET chip_flows 0 FLASH_LAYOUT)
endif()
if(EXISTS ${FLASH_LAYOUT})
  set(flash_layout_json ${FLASH_LAYOUT})
else()
  set(flash_layout_json ${FLASH_IMAGE_DIR}/layouts/${FLASH_LAYOUT}.json)
endif()
if(NOT EXISTS ${flash_layout_json})
  message(FATAL_ERROR "FLASH_LAYOUT=${FLASH_LAYOUT}: ${flash_layout_json} does not exist")
endif()

# A layout with a caliptra-fw slot is a full-Caliptra image; one without is a
# bypass image, which only an A1 can boot.
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

# One MCU runtime per chip and target serves every boot mode: it reads the boot
# strap at startup the same way the ROM does, so the image does not depend on
# how the board will boot. Only a layout with an mcu-fw slot packs it: bypass
# does not, since the A1 ROM's bypass SoC path loads the SoC firmware itself.
set(MCU_FW_DEFAULT ${mcu_fw_dir}/caliptra/npcm-mcu-fw.bin)

if(NOT FLASH_FORMAT)
  set(FLASH_FORMAT ${chip_format})
endif()
if(NOT FLASH_FORMAT MATCHES "^(v1|v3)$")
  message(FATAL_ERROR "FLASH_FORMAT=${FLASH_FORMAT}: must be v1 or v3")
endif()

set(pack_args --layout ${flash_layout_json} --format ${FLASH_FORMAT}
              --manifest-entries ${chip_manifest_entries})

# MCU_FW is only a path override; whether the image contains MCU firmware is the
# layout's call, so a layout without an mcu-fw slot reads nothing from the module.
if(MCU_FW STREQUAL "ON")
  message(FATAL_ERROR "MCU_FW=ON is gone; the layout decides whether MCU firmware is packed")
endif()
if(NOT MCU_FW)
  set(MCU_FW ${MCU_FW_DEFAULT})
endif()
string(FIND "${flash_layout_contents}" "\"mcu-fw\"" mcu_fw_slot)
if(mcu_fw_slot GREATER -1)
  if(NOT EXISTS ${MCU_FW})
    message(FATAL_ERROR
      "FLASH_LAYOUT=${FLASH_LAYOUT} needs its MCU runtime:\n"
      "  ${MCU_FW}\n"
      "Build that runtime from caliptra-mcu-sw (a release_note next to each "
      "prebuilt lists the exact features), or pass -DMCU_FW=<path>.")
  endif()
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

# Say which combination this build packs: the defaults depend on the chip,
# target, layout and boot mode, and a wrong MCU fw only shows up on hardware.
set(shown_mcu_fw none)
if(mcu_fw_slot GREATER -1)
  set(shown_mcu_fw ${MCU_FW})
endif()
set(shown_caliptra_fw none)
if(layout_is_caliptra)
  set(shown_caliptra_fw ${CALIPTRA_FW})
endif()
string(FIND "${flash_layout_contents}" "\"soc-manifest\"" manifest_slot)
set(shown_manifest none)
if(SOC_MANIFEST)
  set(shown_manifest ${SOC_MANIFEST})
elseif(manifest_slot GREATER -1)
  set(shown_manifest generated)
endif()
# Prebuilts are shown relative to the module's images/; overrides stay absolute.
foreach(shown shown_mcu_fw shown_caliptra_fw shown_manifest)
  string(REPLACE "${caliptra_images_dir}/" "" ${shown} "${${shown}}")
endforeach()
# Printed post-build, next to the pack output, so it shows on every build
# (configure does not re-run on incremental builds).
set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
  COMMAND ${CMAKE_COMMAND} -E echo
          "NPCM5 flash image: chip=${npcm500_chip} target=${npcm500_target}"
          "layout=${FLASH_LAYOUT} format=${FLASH_FORMAT}"
          "manifest_entries=${chip_manifest_entries}"
  COMMAND ${CMAKE_COMMAND} -E echo "  MCU fw:       ${shown_mcu_fw}"
  COMMAND ${CMAKE_COMMAND} -E echo "  Caliptra fw:  ${shown_caliptra_fw}"
  COMMAND ${CMAKE_COMMAND} -E echo "  SoC manifest: ${shown_manifest}"
  )

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
          --record ${CONFIG_KERNEL_BIN_NAME}_flash_extract/sources.json
  )

# Verify magic/version/checksums at the layout's slot offset; flash_image.py exits
# non-zero on a bad image, failing the build rather than shipping a corrupt one.
set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
  COMMAND ${PYTHON_EXECUTABLE} ${FLASH_IMAGE_TOOL} verify
          ${CONFIG_KERNEL_BIN_NAME}_flash.bin --layout ${flash_layout_json}
          --manifest-entries ${chip_manifest_entries}
  )

# Extract each embedded image to its own file, next to the sources.json record
# the pack step wrote (which file each blob came from). These are the
# uart_normal_boot inputs. By hand:
#   python3 flash_image_tool.py extract input/NPCM5_flash.bin
set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
  COMMAND ${PYTHON_EXECUTABLE} ${FLASH_IMAGE_TOOL} extract
          ${CONFIG_KERNEL_BIN_NAME}_flash.bin --layout ${flash_layout_json}
          --out-dir ${CONFIG_KERNEL_BIN_NAME}_flash_extract
  )
