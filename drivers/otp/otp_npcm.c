/*
 * Copyright (c) 2026 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcm_otp

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/otp.h>
#include <zephyr/drivers/otp/otp_npcm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include "otp_npcm_regs.h"

LOG_MODULE_REGISTER(otp_npcm, CONFIG_OTP_LOG_LEVEL);

#ifdef CONFIG_OTP_PROGRAM
/*
 * Interval between program-done polls. The controller has no interrupt.
 * This is a floor, not a period: the poll sleeps, so the real interval is
 * whichever is longer, this value or one system tick. A build with a coarse
 * SYS_CLOCK_TICKS_PER_SEC therefore gets proportionally fewer polls out of
 * CONFIG_OTP_NPCM_PROGRAM_TIMEOUT_MS, which bounds the wait either way.
 */
#define OTP_NPCM_PROGRAM_POLL_US 200U
#endif

struct otp_npcm_config {
	mem_addr_t base;
	size_t size;
	const struct device *clk_dev;
	uint32_t clk_cfg;
};

struct otp_npcm_data {
	struct k_mutex lock;
};

/* Which OTP_LOCK/OTP_LOCK2 bits protect each region. */
struct otp_npcm_region_lock {
	uint8_t reg;
	uint8_t wr_bit; /* zero when the region has no write-only lock */
	uint8_t rw_bit;
};

static const struct otp_npcm_region_lock otp_npcm_region_locks[OTP_NPCM_REGION_COUNT] = {
	[OTP_NPCM_REGION_0] = {NPCM_OTP_LOCK, NPCM_OTP_LOCK_RGN0_WR, NPCM_OTP_LOCK_RGN0_RW},
	[OTP_NPCM_REGION_SESSION_CRED] = {NPCM_OTP_LOCK, 0, NPCM_OTP_LOCK_SESSKEY_RW},
	[OTP_NPCM_REGION_1] = {NPCM_OTP_LOCK, NPCM_OTP_LOCK_RGN1_WR, NPCM_OTP_LOCK_RGN1_RW},
	[OTP_NPCM_REGION_2] = {NPCM_OTP_LOCK, NPCM_OTP_LOCK_RGN2_WR, NPCM_OTP_LOCK_RGN2_RW},
	[OTP_NPCM_REGION_3] = {NPCM_OTP_LOCK2, NPCM_OTP_LOCK2_RGN3_WR, NPCM_OTP_LOCK2_RGN3_RW},
	[OTP_NPCM_REGION_4] = {NPCM_OTP_LOCK2, NPCM_OTP_LOCK2_RGN4_WR, NPCM_OTP_LOCK2_RGN4_RW},
	[OTP_NPCM_REGION_5] = {NPCM_OTP_LOCK2, NPCM_OTP_LOCK2_RGN5_WR, NPCM_OTP_LOCK2_RGN5_RW},
	[OTP_NPCM_REGION_6] = {NPCM_OTP_LOCK2, NPCM_OTP_LOCK2_RGN6_WR, NPCM_OTP_LOCK2_RGN6_RW},
};

static void otp_npcm_set_addr(mem_addr_t base, uint16_t addr, uint8_t pg_bit)
{
	sys_write8((addr >> NPCM_OTP_ADDR_H_SHIFT) & NPCM_OTP_ADDR_H_MASK,
		   base + NPCM_OTP_ADDR_H);
	/*
	 * OTP_ADDR[4:0] and PG_BIT share this register, so one write covers
	 * both the "write the address" and "write the bit" steps.
	 */
	sys_write8(((addr << NPCM_OTP_ADDR_L_ADDR_SHIFT) & NPCM_OTP_ADDR_L_ADDR_MASK) |
			   (pg_bit & NPCM_OTP_ADDR_L_PGBIT_MASK),
		   base + NPCM_OTP_ADDR_L);
}

/* Leave OTP operation mode and wait for the controller to power down. */
static int otp_npcm_teardown(mem_addr_t base)
{
	/* The order of these two writes is mandated by the datasheet. */
	sys_write8(NPCM_OTP_ECODE_EXIT, base + NPCM_OTP_ECODE);
	sys_write8(0, base + NPCM_OTP_CTL);

	if (!WAIT_FOR((sys_read8(base + NPCM_OTP_CTL) & NPCM_OTP_CTL_OTPRDY) == 0,
		      CONFIG_OTP_NPCM_READ_TIMEOUT_US, k_busy_wait(1))) {
		LOG_ERR("OTP stuck ready after teardown");
		return -ETIMEDOUT;
	}

	return 0;
}

static int otp_npcm_read_byte(mem_addr_t base, uint16_t addr, uint8_t *out)
{
	int ret = 0;

	sys_write8(NPCM_OTP_CTL_ENOTP, base + NPCM_OTP_CTL);
	sys_write8(NPCM_OTP_ECODE_ENTRY, base + NPCM_OTP_ECODE);
	otp_npcm_set_addr(base, addr, 0);
	sys_write8(NPCM_OTP_CTL_ENOTP | NPCM_OTP_CTL_RDEN, base + NPCM_OTP_CTL);

	/*
	 * Only RDDSTS is polled. OTP_STS.PERROR reports the trim check the
	 * controller runs after a program and has no meaning for a read.
	 */
	if (!WAIT_FOR(sys_read8(base + NPCM_OTP_STS) & NPCM_OTP_STS_RDDSTS,
		      CONFIG_OTP_NPCM_READ_TIMEOUT_US, k_busy_wait(1))) {
		LOG_ERR("OTP read timeout at 0x%03x", addr);
		ret = -ETIMEDOUT;
	} else {
		*out = sys_read8(base + NPCM_OTP_DOUT);
	}

	if (ret == 0) {
		return otp_npcm_teardown(base);
	}

	(void)otp_npcm_teardown(base);
	return ret;
}

#ifdef CONFIG_OTP_PROGRAM
static int otp_npcm_program_bit(mem_addr_t base, uint16_t addr, uint8_t bit)
{
	int64_t deadline;
	int ret = -ETIMEDOUT;

	sys_write8(NPCM_OTP_CTL_ENOTP, base + NPCM_OTP_CTL);
	sys_write8(NPCM_OTP_ECODE_ENTRY, base + NPCM_OTP_ECODE);
	otp_npcm_set_addr(base, addr, bit);
	sys_write8(NPCM_OTP_CTL_ENOTP | NPCM_OTP_CTL_PGEN, base + NPCM_OTP_CTL);

	/*
	 * Only PGDSTS decides the outcome. The controller's trim check runs
	 * "after write operation done", so PERROR can lag PGDSTS by an
	 * unspecified amount; the caller verifies by reading the byte back
	 * rather than racing it.
	 */
	deadline = k_uptime_get() + CONFIG_OTP_NPCM_PROGRAM_TIMEOUT_MS;
	do {
		if (sys_read8(base + NPCM_OTP_STS) & NPCM_OTP_STS_PGDSTS) {
			ret = 0;
			break;
		}

		k_sleep(K_USEC(OTP_NPCM_PROGRAM_POLL_US));
	} while (k_uptime_get() < deadline);

	if (ret == -ETIMEDOUT) {
		LOG_ERR("OTP program timeout at 0x%03x bit %u", addr, bit);
	}

	if (ret == 0) {
		return otp_npcm_teardown(base);
	}

	(void)otp_npcm_teardown(base);
	return ret;
}

/*
 * Confirm every requested bit reached the array.
 *
 * The controller's own trim check (OTP_STS.PERROR) is deliberately not used
 * for this. The datasheet documents no condition that clears PERROR, unlike
 * RDDSTS and PGDSTS, and it states the check runs after the write completes,
 * so a program can look done before PERROR reflects it. Reading the byte back
 * relies only on documented behaviour and catches a refused write -- a locked
 * region, say -- however the controller chose to report it.
 */
static int otp_npcm_verify_byte(mem_addr_t base, uint16_t addr, uint8_t want)
{
	uint8_t got;
	int ret;

	ret = otp_npcm_read_byte(base, addr, &got);
	if (ret != 0) {
		return ret;
	}

	if ((got & want) != want) {
		LOG_ERR("OTP verify failed at 0x%03x: want 0x%02x got 0x%02x, OTP_STS 0x%02x",
			addr, want, got, sys_read8(base + NPCM_OTP_STS));
		return -EIO;
	}

	return 0;
}

#endif /* CONFIG_OTP_PROGRAM */

static int otp_npcm_api_read(const struct device *dev, off_t offset, void *buf, size_t len)
{
	const struct otp_npcm_config *cfg = dev->config;
	struct otp_npcm_data *data = dev->data;
	uint8_t *dst = buf;
	int ret = 0;

	if (buf == NULL || offset < 0 || len > cfg->size || (size_t)offset > cfg->size - len) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	for (size_t i = 0; i < len; i++) {
		ret = otp_npcm_read_byte(cfg->base, (uint16_t)(offset + (off_t)i), &dst[i]);
		if (ret != 0) {
			break;
		}
	}

	k_mutex_unlock(&data->lock);
	return ret;
}

#ifdef CONFIG_OTP_PROGRAM
static int otp_npcm_api_program(const struct device *dev, off_t offset, const void *buf,
				size_t len)
{
	const struct otp_npcm_config *cfg = dev->config;
	struct otp_npcm_data *data = dev->data;
	const uint8_t *src = buf;
	int ret = 0;

	if (buf == NULL || offset < 0 || len > cfg->size || (size_t)offset > cfg->size - len) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	for (size_t i = 0; i < len && ret == 0; i++) {
		uint16_t addr = (uint16_t)(offset + (off_t)i);
		uint8_t current;
		uint8_t todo;

		ret = otp_npcm_read_byte(cfg->base, addr, &current);
		if (ret != 0) {
			break;
		}

		/*
		 * Only 0 -> 1 transitions are possible. Masking against the
		 * current contents also means an already-programmed bit is
		 * never programmed a second time, which the controller
		 * disallows.
		 */
		todo = (uint8_t)(~current & src[i]);

		for (uint8_t bit = 0; bit < 8 && ret == 0; bit++) {
			if (todo & BIT(bit)) {
				ret = otp_npcm_program_bit(cfg->base, addr, bit);
			}
		}

		if (ret == 0) {
			ret = otp_npcm_verify_byte(cfg->base, addr, src[i]);
		}
	}

	k_mutex_unlock(&data->lock);
	return ret;
}

#endif /* CONFIG_OTP_PROGRAM */

static const struct otp_driver_api otp_npcm_driver_api = {
	.read = otp_npcm_api_read,
#ifdef CONFIG_OTP_PROGRAM
	.program = otp_npcm_api_program,
#endif
};

size_t otp_npcm_size(const struct device *dev)
{
	const struct otp_npcm_config *cfg = dev->config;

	return cfg->size;
}

int otp_npcm_lock_region(const struct device *dev, enum otp_npcm_region region,
			 enum otp_npcm_lock_flags flags)
{
	const struct otp_npcm_config *cfg = dev->config;
	struct otp_npcm_data *data = dev->data;
	const struct otp_npcm_region_lock *entry;
	uint8_t bit;
	int ret;

	if ((unsigned int)region >= OTP_NPCM_REGION_COUNT) {
		return -EINVAL;
	}

	entry = &otp_npcm_region_locks[region];

	switch (flags) {
	case OTP_NPCM_LOCK_WRITE:
		bit = entry->wr_bit;
		break;
	case OTP_NPCM_LOCK_RW:
		bit = entry->rw_bit;
		break;
	default:
		return -EINVAL;
	}

	if (bit == 0) {
		/* Only the session credential region lacks a write-only lock. */
		LOG_ERR("session credential region supports OTP_NPCM_LOCK_RW only");
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	/*
	 * Write only the bit being set. These registers ignore attempts to
	 * clear a bit once written, so there is no read-modify-write to do and
	 * no way to undo this. Read back inside the lock: there is no unlock
	 * call to retry with, so protection that did not take must not be
	 * reported as applied.
	 */
	sys_write8(bit, cfg->base + entry->reg);
	ret = (sys_read8(cfg->base + entry->reg) & bit) ? 0 : -EIO;
	k_mutex_unlock(&data->lock);

	if (ret != 0) {
		LOG_ERR("region %u lock bit 0x%02x did not take", (unsigned int)region, bit);
	}

	return ret;
}

bool otp_npcm_region_is_locked(const struct device *dev, enum otp_npcm_region region,
			       enum otp_npcm_lock_flags flags)
{
	const struct otp_npcm_config *cfg = dev->config;
	struct otp_npcm_data *data = dev->data;
	const struct otp_npcm_region_lock *entry;
	uint8_t bits;
	bool locked;

	if ((unsigned int)region >= OTP_NPCM_REGION_COUNT) {
		return false;
	}

	entry = &otp_npcm_region_locks[region];

	switch (flags) {
	case OTP_NPCM_LOCK_WRITE:
		/* A read/write lock necessarily prevents programming too. */
		bits = entry->wr_bit | entry->rw_bit;
		break;
	case OTP_NPCM_LOCK_RW:
		bits = entry->rw_bit;
		break;
	default:
		return false;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	locked = (sys_read8(cfg->base + entry->reg) & bits) != 0;
	k_mutex_unlock(&data->lock);

	return locked;
}

static int otp_npcm_init(const struct device *dev)
{
	const struct otp_npcm_config *cfg = dev->config;
	struct otp_npcm_data *data = dev->data;
	uint32_t rate;
	uint32_t div;
	int ret;

	k_mutex_init(&data->lock);

	if (!device_is_ready(cfg->clk_dev)) {
		LOG_ERR("clock controller not ready");
		return -ENODEV;
	}

	/*
	 * The controller has no power-down bit, so there is no clock to turn
	 * on; the rate is needed only to derive the divider.
	 */
	ret = clock_control_get_rate(cfg->clk_dev, (clock_control_subsys_t)(uintptr_t)cfg->clk_cfg,
				     &rate);
	if (ret != 0) {
		LOG_ERR("failed to get APB1 rate (%d)", ret);
		return ret;
	}

	div = DIV_ROUND_CLOSEST(rate, NPCM_OTP_CLK_HZ);
	if (div == 0 || div > NPCM_OTP_CLKDIV_MASK) {
		LOG_ERR("APB1 rate %u Hz needs divider %u, out of range", rate, div);
		return -EINVAL;
	}
	sys_write8((uint8_t)div, cfg->base + NPCM_OTP_CLKDIV);

	/* The controller reads its initial security region at reset. */
	if (!WAIT_FOR(sys_read8(cfg->base + NPCM_OTP_KEYSTS) & NPCM_OTP_KEYSTS_INITSTS,
		      CONFIG_OTP_NPCM_INIT_TIMEOUT_US, k_busy_wait(1))) {
		LOG_ERR("OTP not ready after init");
		return -ETIMEDOUT;
	}

#ifdef CONFIG_OTP_NPCM_UNLOCK_ON_INIT
	/*
	 * The first write after a Core Domain reset is unrestricted. Subsequent
	 * writes can only set bits, so this must be enabled only by test builds.
	 */
	sys_write8(0, cfg->base + NPCM_OTP_LOCK);
	sys_write8(0, cfg->base + NPCM_OTP_LOCK2);
	if (sys_read8(cfg->base + NPCM_OTP_LOCK) != 0 ||
	    sys_read8(cfg->base + NPCM_OTP_LOCK2) != 0) {
		LOG_ERR("failed to clear OTP region locks; reset Core Domain first");
		return -EACCES;
	}
#endif

	return 0;
}

#define OTP_NPCM_INIT(inst)                                                                        \
	BUILD_ASSERT(DT_INST_PROP(inst, size) <= NPCM_OTP_MAX_SIZE,                                \
		     "size exceeds what OTP_ADDR can address");                                    \
                                                                                                   \
	static struct otp_npcm_data otp_npcm_data_##inst;                                          \
                                                                                                   \
	static const struct otp_npcm_config otp_npcm_config_##inst = {                             \
		.base = (mem_addr_t)DT_INST_REG_ADDR(inst),                                        \
		.size = DT_INST_PROP(inst, size),                                                  \
		.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                               \
		.clk_cfg = DT_INST_PHA(inst, clocks, clk_cfg),                                     \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, otp_npcm_init, NULL, &otp_npcm_data_##inst,                    \
			      &otp_npcm_config_##inst, POST_KERNEL,                                \
			      CONFIG_OTP_INIT_PRIORITY, &otp_npcm_driver_api);

DT_INST_FOREACH_STATUS_OKAY(OTP_NPCM_INIT)
