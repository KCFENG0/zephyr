# SPDX-License-Identifier: Apache-2.0

# Flash: MCP SPI flash via the Caliptra MCU
board_set_flasher_ifnset(npcm500_mcp)
board_runner_args(npcm500_mcp "--file=${PROJECT_BINARY_DIR}/${CONFIG_KERNEL_BIN_NAME}_flash.bin")
board_finalize_runner_args(npcm500_mcp)

# Debug only (west debug/debugserver/attach); flashing goes through npcm500_mcp
board_runner_args(jlink "--device=Cortex-M7" "--speed=4000")
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
