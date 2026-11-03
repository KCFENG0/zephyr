# SPDX-License-Identifier: Apache-2.0
#
# Rebuild the prebuilt on-target MCP flash programmer (Caliptra MCU, RISC-V; commit the
# output): ../mcp_stub.bin.
#
# mcp_stub.bin is reproducible byte for byte with riscv64-unknown-elf-gcc 13.2.0 (Ubuntu 24.04).
# The Zephyr SDK toolchain also works: make RV_CROSS=<sdk>/riscv64-zephyr-elf/bin/riscv64-zephyr-elf-

RV_CROSS  ?= riscv64-unknown-elf-
RV_CFLAGS := -march=rv32imc_zicsr -mabi=ilp32 -Os -ffreestanding -nostdlib -mno-relax

all: ../mcp_stub.bin

../mcp_stub.bin: mcp_stub.c mcp_uma.h mcp_stub.ld
	$(RV_CROSS)gcc $(RV_CFLAGS) -T mcp_stub.ld mcp_stub.c -o mcp_stub.elf
	$(RV_CROSS)objcopy -O binary mcp_stub.elf $@
	chmod a-x $@
	rm -f mcp_stub.elf

.PHONY: all
