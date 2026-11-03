/* SPDX-License-Identifier: Apache-2.0 */
/*
 * NPCM500 MCP flash on-target programmer, run on the Caliptra MCU (VeeR EL2,
 * mcp_stub.bin) by mcp_flash_program.tcl (npcm500_mcp_write_fast).
 * Same FIU2 UMA sequence as the Tcl path: per-sector erase, EXT_DB 16-byte page
 * program, DRA read-back compare.
 *
 * Build: make -C boards/nuvoton/npcm500_evb/support/src (see Makefile)
 *
 * Calling convention (the host sets a0..a2, resumes at an entry, waits for the
 * ebreak that ends it; results come back in a0/a1):
 *   +0: a0 = mcp_flash_sector(dst, src, len): 0 ok, >0 words differ, <0 error
 *       a1 = byte offset of the first differing word (when a0 > 0)
 *   +4: a0 = mcp_crc32(dst, len): CRC-32 (IEEE, as in the gzip trailer) of flash
 *   +8: a0/a1 = mcp_compare(dst, src, len): like +0 without erase/program (read-only)
 * A trap returns a0 = 0xE0000000 | mcause, a1 = mepc.
 */
#include "mcp_uma.h"

__asm__(
	".section .text.entry, \"ax\"\n"
	".option push\n"
	".option norvc\n"
	".global _start\n"
	"_start:\n"
	"	j	flash_entry\n"
	"	j	crc_entry\n"
	"	j	compare_entry\n"
	".option pop\n"
	"flash_entry:\n"
	"	la	t1, mcp_flash_sector\n"
	"	j	stub_call\n"
	"compare_entry:\n"
	"	la	t1, mcp_compare\n"
	"	j	stub_call\n"
	"crc_entry:\n"
	"	la	t1, mcp_crc32\n"
	"stub_call:\n"
	"	csrci	mstatus, 8\n"
	"	la	t0, stub_trap\n"
	"	csrw	mtvec, t0\n"
	"	la	sp, __stack_top\n"
	"	jalr	t1\n"
	"	ebreak\n"
	".balign 4\n"
	"stub_trap:\n"
	"	csrr	a0, mcause\n"
	"	li	t0, 0xE0000000\n"
	"	or	a0, a0, t0\n"
	"	csrr	a1, mepc\n"
	"	ebreak\n"
	"1:	j	1b\n"
	".text\n");

/*
 * Read a 32-bit word from the DRA window, waiting until two consecutive reads match.
 */
static uint32_t stable_read(const volatile uint32_t *p)
{
	uint32_t v = *p;

	for (int i = 0; i < 8; i++) {
		uint32_t again = *p;

		if (again == v)
			break;
		v = again;
	}
	return v;
}

/* RV32 returns uint64_t in a0 (low) / a1 (high): mismatch count, first bad byte offset */
uint64_t mcp_compare(uint32_t dst, const uint32_t *src, uint32_t len)
{
	const volatile uint32_t *flash = (const volatile uint32_t *)(MCP_DRA + dst);
	uint32_t bad = 0, first = 0;

	for (uint32_t w = len / 4; w--; ) {
		if (stable_read(&flash[w]) != src[w]) {
			bad++;
			first = w * 4;
		}
	}
	return (uint64_t)first << 32 | bad;
}

/* dst: 4KB aligned; src: word aligned; len: multiple of 16, <= 4KB */
uint64_t mcp_flash_sector(uint32_t dst, const uint32_t *src, uint32_t len)
{
	struct uma_session s;
	int rc;

	uma_begin(&s);
	rc = sector_erase(dst);
	for (uint32_t off = 0; off < len && !rc; off += MCP_PAGE) {
		uint32_t n = len - off < MCP_PAGE ? len - off : MCP_PAGE;

		rc = page_program(dst + off, src + off / 4, n);
	}
	uma_end(&s);
	return rc ? (uint32_t)rc : mcp_compare(dst, src, len);
}

/* dst: word aligned; reflected CRC-32, init/xorout 0xFFFFFFFF (zlib/gzip) */
uint32_t mcp_crc32(uint32_t dst, uint32_t len)
{
	const volatile uint32_t *flash = (const volatile uint32_t *)(MCP_DRA + dst);
	uint32_t crc = 0xFFFFFFFFu, w = 0;

	for (uint32_t i = 0; i < len; i++) {
		if (!(i & 3))
			w = stable_read(&flash[i / 4]);
		crc ^= (w >> (8 * (i & 3))) & 0xFF;
		for (int b = 0; b < 8; b++)
			crc = (crc >> 1) ^ (0xEDB88320u & -(crc & 1));
	}
	return ~crc;
}
