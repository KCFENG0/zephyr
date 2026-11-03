/* SPDX-License-Identifier: Apache-2.0 */
/*
 * NPCM500 MCP flash (W25Q16PW on FIU2 CS0, read-mapped at 0xB000_0000) loader for the
 * J-Link flash download, run on the Caliptra MCU (VeeR EL2): a SEGGER Flash Loader (SFL)
 * with the CMSIS entries as well. Same FIU2 UMA sequence as mcp_stub.c (mcp_uma.h).
 *
 * Linked at 0 and position independent (-mcmodel=medany, no jump tables, no absolute
 * pointers in data): J-Link loads PrgCode + PrgData into the device's work RAM.
 *
 * Needs a J-Link whose RISC-V support attaches to this core (hardware V11 or later);
 * see mcp_jlink.py for why a J-Link V9 cannot. mcp_jlink.py calls the same entries itself.
 */
#include <stddef.h>

#include "mcp_uma.h"

#define MCP_SECTOR  0x1000u
#define MCP_SIZE    0x200000u

/* MCI watchdog enables; only the MCU may write them (not the debug system bus) */
#define MCI_WDT0_EN (*(volatile uint32_t *)0x200000B0u)
#define MCI_WDT1_EN (*(volatile uint32_t *)0x200000C0u)

struct sector_info {
	uint32_t size;
	uint32_t addr;
};

struct flash_device {
	uint16_t vers;
	char name[128];
	uint16_t type;
	uint32_t base;
	uint32_t size;
	uint32_t page_size;
	uint32_t reserved;
	uint8_t erased_val;
	uint32_t timeout_prog_ms;
	uint32_t timeout_erase_ms;
	struct sector_info sectors[2];
};

__attribute__((used, section("DevDscr")))
const struct flash_device FlashDevice = {
	.vers = 0x0101,
	.name = "NPCM500 MCP SPI flash (W25Q16PW, FIU2)",
	.type = 5, /* external SPI */
	.base = MCP_DRA,
	.size = MCP_SIZE,
	.page_size = MCP_PAGE,
	.erased_val = 0xFF,
	.timeout_prog_ms = 100,
	.timeout_erase_ms = 1000,
	.sectors = { { MCP_SECTOR, 0 }, { 0xFFFFFFFFu, 0xFFFFFFFFu } },
};

/* SEGGER_FL_Flags: none */
__attribute__((used)) const uint32_t SEGGER_FL_Flags = 0;

static uint32_t saved_mstatus, saved_mtvec;

/* A trap stops at an ebreak instead of entering the firmware's handler */
__asm__(
	".text\n"
	".balign 4\n"
	"flm_trap:\n"
	"	ebreak\n"
	"	j	flm_trap\n");

#define SPI_READ 0x03

/*
 * Compare flash contents through UMA (SPI normal read, CS0 held low across the range)
 * against data[] or, if data is NULL, against fill. *bad = offset of the first difference,
 * n if none.
 */
static int uma_compare(uint32_t addr, uint32_t n, const uint8_t *data, uint8_t fill,
		       uint32_t *bad)
{
	int rc;

	*bad = n;
	R8(FIU_UMA_ECTS) = ECTS_CS0_LOW;
	R8(FIU_UMA_CODE) = SPI_READ;
	uma_set_addr(addr);
	rc = uma_exec(CTS_WR_ADDR);
	for (uint32_t i = 0; i < n && !rc; i++) {
		rc = uma_exec(CTS_RD_1B);
		if (!rc && R8(FIU_UMA_DB0) != (data ? data[i] : fill)) {
			*bad = i;
			break;
		}
	}
	cs_high();
	return rc;
}

int SEGGER_FL_Prepare(uint32_t p0, uint32_t p1, uint32_t p2)
{
	uint32_t trap;

	(void)p0;
	(void)p1;
	(void)p2;
	__asm__ volatile("csrrci %0, mstatus, 8" : "=r"(saved_mstatus));
	__asm__ volatile("la %0, flm_trap" : "=r"(trap));
	__asm__ volatile("csrrw %0, mtvec, %1" : "=r"(saved_mtvec) : "r"(trap));
	/* Disable MCI watchdogs while the hart is halted */
	MCI_WDT0_EN = 0;
	MCI_WDT1_EN = 0;
	return 0;
}

int SEGGER_FL_Restore(uint32_t p0, uint32_t p1, uint32_t p2)
{
	(void)p0;
	(void)p1;
	(void)p2;
	__asm__ volatile("csrw mtvec, %0" ::"r"(saved_mtvec));
	__asm__ volatile("csrw mstatus, %0" ::"r"(saved_mstatus));
	return 0;
}

int SEGGER_FL_Erase(uint32_t addr, uint32_t index, uint32_t n)
{
	struct uma_session s;
	int rc = 0;

	(void)index;
	addr -= MCP_DRA;
	uma_begin(&s);
	for (; n && !rc; n--, addr += MCP_SECTOR)
		rc = sector_erase(addr);
	uma_end(&s);
	return rc ? 1 : 0;
}

/*
 * addr: DRA address; returns the offset of the first difference, n if none, -1 on error.
 * Compares through the DRA window (fast) and confirms any differing words through UMA.
 */
static int32_t verify(uint32_t addr, uint32_t n, const uint8_t *data, uint8_t fill)
{
	const volatile uint32_t *dra = (const volatile uint32_t *)(addr & ~3u);
	uint32_t i = 0;

	while (i < n) {
		uint32_t pos = (addr + i) & 3, k = 4 - pos, want = 0, got;

		if (k > n - i)
			k = n - i;
		got = dra[(addr + i - (addr & ~3u)) / 4] >> (8 * pos);
		for (uint32_t b = 0; b < k; b++)
			want |= (uint32_t)(data ? data[i + b] : fill) << (8 * b);
		if ((got ^ want) & (0xFFFFFFFFu >> (32 - 8 * k))) {
			struct uma_session s;
			uint32_t bad;
			int rc;

			uma_begin(&s);
			rc = uma_compare(addr + i - MCP_DRA, k, data ? data + i : NULL, fill, &bad);
			uma_end(&s);
			if (rc)
				return -1;
			if (bad < k)
				return (int32_t)(i + bad);
		}
		i += k;
	}
	return (int32_t)n;
}

/* Returns 2: programmed and read back */
int SEGGER_FL_Program(uint32_t addr, uint32_t n, uint8_t *src)
{
	uint32_t page[MCP_PAGE / 4];
	struct uma_session s;
	int rc = 0;

	uma_begin(&s);
	for (uint32_t off = 0; off < n && !rc; off += MCP_PAGE) {
		const uint32_t *p = (const uint32_t *)(src + off);

		if ((uintptr_t)p & 3) {
			for (uint32_t i = 0; i < MCP_PAGE; i++)
				((uint8_t *)page)[i] = src[off + i];
			p = page;
		}
		rc = page_program(addr - MCP_DRA + off, p, MCP_PAGE);
	}
	uma_end(&s);
	if (rc || verify(addr, n, src, 0) != (int32_t)n)
		return 1;
	return 2;
}

uint32_t SEGGER_FL_Verify(uint32_t addr, uint32_t n, uint8_t *data)
{
	int32_t ok = verify(addr, n, data, 0);

	return addr + (ok < 0 ? 0 : (uint32_t)ok);
}

int SEGGER_FL_CheckBlank(uint32_t addr, uint32_t n, uint8_t blank)
{
	int32_t ok = verify(addr, n, NULL, blank);

	return ok < 0 ? -1 : ok != (int32_t)n;
}

/*
 * Not a SEGGER entry (J-Link ignores it): CRC-32 (IEEE, as zlib) of RAM, so mcp_jlink.py can
 * check an image buffer it loaded by SBA without reading it all back over JTAG.
 */
uint32_t mcp_crc32(const uint8_t *p, uint32_t n)
{
	uint32_t crc = 0xFFFFFFFFu;

	while (n--) {
		crc ^= *p++;
		for (int b = 0; b < 8; b++)
			crc = (crc >> 1) ^ (0xEDB88320u & -(crc & 1));
	}
	return ~crc;
}

/* CMSIS flash algorithm entries */
int Init(uint32_t adr, uint32_t clk, uint32_t fnc)
{
	(void)adr;
	(void)clk;
	(void)fnc;
	return SEGGER_FL_Prepare(0, 0, 0);
}

int UnInit(uint32_t fnc)
{
	(void)fnc;
	return SEGGER_FL_Restore(0, 0, 0);
}

int EraseSector(uint32_t adr)
{
	return SEGGER_FL_Erase(adr, 0, 1);
}

int ProgramPage(uint32_t adr, uint32_t sz, uint8_t *buf)
{
	/* CMSIS may pass a partial last page: pad with the erased value */
	uint8_t page[MCP_PAGE];

	if (sz % MCP_PAGE == 0)
		return SEGGER_FL_Program(adr, sz, buf) == 1;
	for (uint32_t i = 0; i < MCP_PAGE; i++)
		page[i] = i < sz ? buf[i] : 0xFF;
	return SEGGER_FL_Program(adr, MCP_PAGE, page) == 1;
}
