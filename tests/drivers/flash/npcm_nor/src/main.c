/*
 * Copyright (c) 2026 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Automated coverage for the NPCM FIU/SPI-NOR flash driver
 * (drivers/flash/flash_npcm_nor.c + flash_npcm_fiu_qspi.c).
 *
 * These cases turn the manual EVB shell verification
 * ("flash test/erase/write/read w25q256jv@0 ...") into a repeatable
 * ztest suite so both CI and real hardware (EVB, and FPGA once its
 * devicetree gains a nuvoton,npcm-nor node) can run the same checks:
 *
 *  - erase/write/verify at the very start of the device (3-byte
 *    addressing range)
 *  - erase/write/verify at an offset that only exists in the 4-byte
 *    addressing range (>= 16 MiB), exercising ADDR_4B_EN / enter-4ba
 *  - erase/write/verify near the very end of the device
 *  - multiple independent page-program calls inside a single erased
 *    4K sector, confirming a later write does not corrupt data an
 *    earlier write placed in the same sector
 *  - a single write whose buffer straddles a 256-byte program-page
 *    boundary
 *  - out-of-range erase/write rejected with a negative error code
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>

#if !DT_HAS_COMPAT_STATUS_OKAY(nuvoton_npcm_nor)
#error "No enabled nuvoton,npcm-nor flash device in the devicetree"
#endif

#define NOR_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(nuvoton_npcm_nor)

/* size property is in bits, per jedec,jesd216.yaml convention. */
#define NOR_FLASH_SIZE (DT_PROP(NOR_NODE, size) / 8)

/* Matches SPI_NOR_CMD_SE erase granularity used by flash_npcm_nor_erase(). */
#define SECTOR_4K_SIZE   KB(4)
/* Standard SPI-NOR program-page size (SPI_NOR_CMD_PP boundary). */
#define PROGRAM_PAGE_SIZE 256U
/* First offset that requires 4-byte addressing on this device. */
#define ADDR_4B_BOUNDARY_OFFSET 0x1000000UL

static const struct device *const flash_dev = DEVICE_DT_GET(NOR_NODE);

static void *npcm_nor_setup(void)
{
	zassert_true(device_is_ready(flash_dev),
		     "flash device %s is not ready", flash_dev->name);

	TC_PRINT("Testing device %s, size 0x%x bytes\n", flash_dev->name, NOR_FLASH_SIZE);

	return NULL;
}

static void erase_write_verify(off_t offset, uint32_t fill_word)
{
	int rc;
	uint32_t readback = 0;

	rc = flash_erase(flash_dev, offset, SECTOR_4K_SIZE);
	zassert_equal(rc, 0, "flash_erase(0x%lx) failed: %d", (long)offset, rc);

	rc = flash_write(flash_dev, offset, &fill_word, sizeof(fill_word));
	zassert_equal(rc, 0, "flash_write(0x%lx) failed: %d", (long)offset, rc);

	rc = flash_read(flash_dev, offset, &readback, sizeof(readback));
	zassert_equal(rc, 0, "flash_read(0x%lx) failed: %d", (long)offset, rc);

	zassert_equal(readback, fill_word,
		      "readback mismatch at 0x%lx: got 0x%08x, expected 0x%08x",
		      (long)offset, readback, fill_word);
}

/* Manual test case #1: offset 0x0 */
ZTEST(flash_npcm_nor, test_erase_write_verify_offset_zero)
{
	erase_write_verify(0x0, 0xdeadbeef);
}

/* Manual test case #2: offset 0x1000000 (16 MiB, first 4-byte-address-only offset) */
ZTEST(flash_npcm_nor, test_erase_write_verify_4byte_addr_boundary)
{
	if (NOR_FLASH_SIZE <= ADDR_4B_BOUNDARY_OFFSET + SECTOR_4K_SIZE) {
		ztest_test_skip();
	}

	erase_write_verify(ADDR_4B_BOUNDARY_OFFSET, 0xcafef00d);
}

/* Manual test case #3: offset 0x1FF0000 on a 32 MiB part == (size - 64K) */
ZTEST(flash_npcm_nor, test_erase_write_verify_near_end)
{
	off_t offset = (off_t)(NOR_FLASH_SIZE - KB(64));

	zassert_true(offset >= 0, "flash too small for this test");

	erase_write_verify(offset, 0x600dc0de);
}

/*
 * Manual test case #4: erase once, then perform three independent
 * page-program calls inside the same 4K sector and confirm none of
 * them clobbers a previous one (mirrors the 0xFFF000/004/008 manual
 * sequence).
 */
ZTEST(flash_npcm_nor, test_multiple_writes_same_sector_no_corruption)
{
	static const uint32_t words[3] = {0x11111111, 0x22222222, 0x33333333};
	off_t base = (off_t)(NOR_FLASH_SIZE - KB(64) - SECTOR_4K_SIZE);
	uint32_t readback[3];
	int rc;

	zassert_true(base >= 0, "flash too small for this test");

	rc = flash_erase(flash_dev, base, SECTOR_4K_SIZE);
	zassert_equal(rc, 0, "flash_erase failed: %d", rc);

	for (int i = 0; i < 3; i++) {
		rc = flash_write(flash_dev, base + i * (off_t)sizeof(uint32_t),
				  &words[i], sizeof(words[i]));
		zassert_equal(rc, 0, "flash_write #%d failed: %d", i, rc);
	}

	rc = flash_read(flash_dev, base, readback, sizeof(readback));
	zassert_equal(rc, 0, "flash_read failed: %d", rc);

	for (int i = 0; i < 3; i++) {
		zassert_equal(readback[i], words[i],
			      "word #%d corrupted: got 0x%08x, expected 0x%08x",
			      i, readback[i], words[i]);
	}
}

/*
 * Additional coverage (not in the original manual session): a single
 * write whose buffer straddles a 256-byte program-page boundary.
 * flash_npcm_nor_write() splits such requests internally; make sure
 * the split does not drop or duplicate bytes.
 */
ZTEST(flash_npcm_nor, test_write_across_page_boundary)
{
	uint8_t pattern[32];
	uint8_t readback[32] = {0};
	off_t sector_base = (off_t)(NOR_FLASH_SIZE - KB(64) - 2 * SECTOR_4K_SIZE);
	/* Start 16 bytes before a page boundary so the buffer below straddles it. */
	off_t offset = sector_base + PROGRAM_PAGE_SIZE - 16;
	int rc;

	zassert_true(sector_base >= 0, "flash too small for this test");

	for (size_t i = 0; i < sizeof(pattern); i++) {
		pattern[i] = (uint8_t)(0xA0 + i);
	}

	rc = flash_erase(flash_dev, sector_base, SECTOR_4K_SIZE);
	zassert_equal(rc, 0, "flash_erase failed: %d", rc);

	rc = flash_write(flash_dev, offset, pattern, sizeof(pattern));
	zassert_equal(rc, 0, "flash_write across page boundary failed: %d", rc);

	rc = flash_read(flash_dev, offset, readback, sizeof(readback));
	zassert_equal(rc, 0, "flash_read failed: %d", rc);

	zassert_mem_equal(readback, pattern, sizeof(pattern),
			   "data corrupted across program-page boundary");
}

/*
 * Boundary/negative coverage: operations fully or partially outside
 * the device must be rejected, not silently clamped or wrapped.
 */
ZTEST(flash_npcm_nor, test_erase_write_out_of_bounds_rejected)
{
	uint32_t word = 0x12345678;
	int rc;

	rc = flash_erase(flash_dev, (off_t)NOR_FLASH_SIZE, SECTOR_4K_SIZE);
	zassert_true(rc < 0, "erase at/after end of flash should fail, got %d", rc);

	rc = flash_erase(flash_dev, (off_t)(NOR_FLASH_SIZE - SECTOR_4K_SIZE), SECTOR_4K_SIZE + 1);
	zassert_true(rc < 0, "erase spanning past end of flash should fail, got %d", rc);

	rc = flash_write(flash_dev, (off_t)NOR_FLASH_SIZE, &word, sizeof(word));
	zassert_true(rc < 0, "write at/after end of flash should fail, got %d", rc);
}

ZTEST_SUITE(flash_npcm_nor, NULL, npcm_nor_setup, NULL, NULL, NULL);
