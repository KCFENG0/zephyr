# SPDX-License-Identifier: Apache-2.0
#
# Rebuild the prebuilt on-target MCP flash programmers (Caliptra MCU, RISC-V; commit the
# output): ../mcp_stub.bin and the SEGGER flash loader ../jlink/NPCM500_MCP.FLM.
#
# mcp_stub.bin is reproducible byte for byte with riscv64-unknown-elf-gcc 13.2.0 (Ubuntu 24.04),
# NPCM500_MCP.FLM with the Zephyr SDK 0.16.8 toolchain:
#   make RV_CROSS=<sdk>/riscv64-zephyr-elf/bin/riscv64-zephyr-elf-

RV_CROSS  ?= riscv64-unknown-elf-
RV_CFLAGS := -march=rv32imc_zicsr -mabi=ilp32 -Os -ffreestanding -nostdlib -mno-relax

all: ../mcp_stub.bin ../jlink/NPCM500_MCP.FLM

../mcp_stub.bin: mcp_stub.c mcp_uma.h mcp_stub.ld
	$(RV_CROSS)gcc $(RV_CFLAGS) -T mcp_stub.ld mcp_stub.c -o mcp_stub.elf
	$(RV_CROSS)objcopy -O binary mcp_stub.elf $@
	chmod a-x $@
	rm -f mcp_stub.elf

# Position independent (all auipc-relative, no gp): J-Link loads it at WorkRAMAddr.
FLM_CFLAGS := $(RV_CFLAGS) -mcmodel=medany -fno-jump-tables -msmall-data-limit=0 -fno-common \
	      -Wall -Wextra -Wl,--no-relax -Wl,-e,0

../jlink/NPCM500_MCP.FLM: mcp_flm.c mcp_uma.h mcp_flm.ld
	$(RV_CROSS)gcc $(FLM_CFLAGS) -T mcp_flm.ld mcp_flm.c -o $@
	$(RV_CROSS)strip --strip-debug $@
	chmod a-x $@

.PHONY: all
