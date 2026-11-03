/* SPDX-License-Identifier: Apache-2.0 */
/*
 * NPCM500 MCP flash (W25Q16PW on FIU2 SW_CS0) UMA access for the on-target programmer
 * mcp_stub.c. Same sequence as mcp_flash_program.tcl: per-sector erase, EXT_DB 16-byte
 * page program. Every function is static: the programmer is a single translation unit.
 */
#ifndef MCP_UMA_H
#define MCP_UMA_H

#include <stdint.h>

#define FIU2_BASE   0x40023000u
#define MCP_DRA     0xB0000000u
#define MCP_PAGE    256u
#define TIMEOUT     2000000u

#define R8(off)     (*(volatile uint8_t *)(FIU2_BASE + (off)))
#define R32(off)    (*(volatile uint32_t *)(FIU2_BASE + (off)))

enum {
	FIU_SPI_FL_CFG = 0x14, FIU_UMA_CODE = 0x16, FIU_UMA_AB0 = 0x17, FIU_UMA_AB1 = 0x18,
	FIU_UMA_AB2 = 0x19, FIU_UMA_DB0 = 0x1A, FIU_UMA_CTS = 0x1E, FIU_UMA_ECTS = 0x1F,
	FIU_MSR_IE_CFG = 0x43, FIU_EXT_DB_CFG = 0x48, FIU_EXT_DB = 0x50,
};

enum {
	CTS_EXEC = 0x80, CTS_WR_CODE = 0xA0, CTS_WR_ADDR = 0xA8, CTS_WR_DATA = 0xB0,
	CTS_RD_1B = 0x91, ECTS_IDLE = 0x07, ECTS_CS0_LOW = 0x06, IE_UMA_BLOCK = 0x08,
	EXT_DB_16B = 0x30,
};

enum { SPI_WREN = 0x06, SPI_RDSR1 = 0x05, SPI_PP = 0x02, SPI_SE_4K = 0x20 };
enum { SR1_BUSY = 0x01, SR1_WEL = 0x02 };
enum { ERR_UMA = -1, ERR_BUSY = -2, ERR_WEL = -3 };

#ifdef __riscv
#define fence() __asm__ volatile("fence" ::: "memory")
#else
#define fence() __asm__ volatile("dsb sy" ::: "memory")
#endif

/*
 * Short FIU register read delay after EXEC_DONE clears and when deasserting CS.
 */
#define SETTLE_READS 64

static void settle(void)
{
	fence();
	for (int i = 0; i < SETTLE_READS; i++)
		(void)R8(FIU_UMA_CTS);
}

/*
 * Datasheet 4.1 EXT_DB example step 8 -> 9: transaction fields go in first, EXEC_DONE as
 * a separate write; fences keep code/addr/EXT_DB stores ahead of the trigger and the
 * trigger ahead of the first status poll whatever the MCU region attributes are.
 */
static int uma_exec(uint8_t cts)
{
	fence();
	R8(FIU_UMA_CTS) = cts & ~CTS_EXEC;
	fence();
	R8(FIU_UMA_CTS) = cts;
	fence();
	for (uint32_t n = TIMEOUT; n; n--) {
		if (!(R8(FIU_UMA_CTS) & CTS_EXEC)) {
			settle();
			return 0;
		}
	}
	return ERR_UMA;
}

/*
 * Release CS0 and wait briefly to satisfy CS high time (tSHSL).
 */
static void cs_high(void)
{
	R8(FIU_UMA_ECTS) = ECTS_IDLE;
	settle();
}

static void uma_set_addr(uint32_t addr)
{
	R8(FIU_UMA_AB0) = addr;
	R8(FIU_UMA_AB1) = addr >> 8;
	R8(FIU_UMA_AB2) = addr >> 16;
}

static int spi_cmd(uint8_t op)
{
	int rc;

	R8(FIU_UMA_ECTS) = ECTS_CS0_LOW;
	R8(FIU_UMA_CODE) = op;
	rc = uma_exec(CTS_WR_CODE);
	cs_high();
	return rc;
}

static int read_sr1(uint8_t *sr)
{
	int rc;

	/* 0xFF = BUSY|WEL: a UMA_DB0 the read failed to refresh keeps the caller waiting */
	R8(FIU_UMA_DB0) = 0xFF;
	R8(FIU_UMA_ECTS) = ECTS_CS0_LOW;
	R8(FIU_UMA_CODE) = SPI_RDSR1;
	rc = uma_exec(CTS_WR_CODE);
	if (!rc)
		rc = uma_exec(CTS_RD_1B);
	*sr = R8(FIU_UMA_DB0);
	cs_high();
	return rc;
}

/*
 * Poll SR1 until two consecutive reads report BUSY=0.
 */
static int wait_ready(void)
{
	uint8_t sr;
	int idle = 0;

	for (uint32_t n = TIMEOUT; n && idle < 2; n--) {
		int rc = read_sr1(&sr);

		if (rc)
			return rc;
		idle = (sr & SR1_BUSY) ? 0 : idle + 1;
	}
	return idle == 2 ? 0 : ERR_BUSY;
}

/* A busy flash ignores WREN: wait for it first, even if the last wait_ready() said ready */
static int wren(void)
{
	uint8_t sr = 0;
	int rc = wait_ready();

	if (!rc)
		rc = spi_cmd(SPI_WREN);

	if (!rc)
		rc = read_sr1(&sr);
	if (!rc && (sr & (SR1_BUSY | SR1_WEL)) != SR1_WEL)
		rc = ERR_WEL;
	return rc;
}

static int sector_erase(uint32_t addr)
{
	int rc = wren();

	if (rc)
		return rc;
	R8(FIU_UMA_ECTS) = ECTS_CS0_LOW;
	R8(FIU_UMA_CODE) = SPI_SE_4K;
	uma_set_addr(addr);
	rc = uma_exec(CTS_WR_ADDR);
	cs_high();
	return rc ? rc : wait_ready();
}

/* n: multiple of 16, <= 256; first 16 bytes ride with code + address, then C_SIZE=1 data-only */
static int page_program(uint32_t addr, const uint32_t *src, uint32_t n)
{
	uint8_t cts = CTS_WR_ADDR;
	int rc = wren();

	if (rc)
		return rc;
	R8(FIU_UMA_ECTS) = ECTS_CS0_LOW;
	R8(FIU_UMA_CODE) = SPI_PP;
	uma_set_addr(addr);
	R8(FIU_EXT_DB_CFG) = EXT_DB_16B;
	for (uint32_t w = 0; w < n / 4 && !rc; w += 4) {
		R32(FIU_EXT_DB + 0) = src[w + 0];
		R32(FIU_EXT_DB + 4) = src[w + 1];
		R32(FIU_EXT_DB + 8) = src[w + 2];
		R32(FIU_EXT_DB + 12) = src[w + 3];
		rc = uma_exec(cts);
		cts = CTS_WR_DATA;
	}
	R8(FIU_EXT_DB_CFG) = 0;
	cs_high();
	return rc ? rc : wait_ready();
}

struct uma_session {
	uint8_t fl_cfg;
	uint8_t ie_cfg;
};

static void uma_begin(struct uma_session *s)
{
	s->fl_cfg = R8(FIU_SPI_FL_CFG);
	s->ie_cfg = R8(FIU_MSR_IE_CFG);

	/* a trap mid page_program leaves EXT_DB_EN set: the next WREN would carry 16 extra bytes */
	R8(FIU_EXT_DB_CFG) = 0;
	/* UMA needs RD_MODE Normal/Fast (datasheet 4.1); UMA_BLOCK as the Zephyr driver does */
	R8(FIU_SPI_FL_CFG) = s->fl_cfg & 0x3F;
	R8(FIU_MSR_IE_CFG) = s->ie_cfg | IE_UMA_BLOCK;
	cs_high();
}

static void uma_end(const struct uma_session *s)
{
	R8(FIU_EXT_DB_CFG) = 0;
	cs_high();
	R8(FIU_MSR_IE_CFG) = s->ie_cfg;
	R8(FIU_SPI_FL_CFG) = s->fl_cfg;
	fence();
}

#endif /* MCP_UMA_H */
