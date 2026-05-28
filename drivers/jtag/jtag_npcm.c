/*
 * Copyright (c) 2023 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/sys/printk.h"
#define DT_DRV_COMPAT nuvoton_npcm_jtag

#include <stdlib.h>
#include <errno.h>
#include <zephyr/drivers/jtag.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/sys_io.h>

#define LOG_LEVEL CONFIG_JTAG_LOG_LEVEL
#define DEFAULT_JTAG_FREQ 1000000U
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(jtag_npcm);
#include "jtag_npcm.h"

struct jtag_info {
	enum jtag_npcm_tap_states tap_state;
};

static struct jtag_info gjtag;

struct jtag_npcm_cfg {
	uintptr_t base;
	uint32_t clk_cfg;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_cfg_func)(const struct device *dev);
};

struct jtag_npcm_data {
	uint32_t src_clk_hz;
	uint32_t tck_freq_hz;
	struct k_sem done_sem; /* signalled by ISR when DONE fires */
};

#define DEV_CFG(dev) ((const struct jtag_npcm_cfg *const)(dev)->config)
#define DEV_DATA(dev) ((struct jtag_npcm_data *)(dev)->data)

#define NPCM_JTAGM_CTL			0x00
#define NPCM_JTAGM_STAT			0x04
#define NPCM_JTAGM_CMD			0x08
/* FIFO registers: n = 0..3 (128-bit FIFO, 4 × 32-bit words) */
#define NPCM_JTAGM_TDO_OUT(n)		(0x10 + (n) * 4)  /* master → target */
#define NPCM_JTAGM_TDO_EN(n)		(0x20 + (n) * 4)  /* output enable   */
#define NPCM_JTAGM_TMS_OUT(n)		(0x30 + (n) * 4)  /* TMS output      */
#define NPCM_JTAGM_TDI_IN(n)		(0x40 + (n) * 4)  /* target → master */

#define NPCM_JTAGM_CTL_JTAGM_EN		BIT(0)
#define NPCM_JTAGM_CTL_DONE_IE		BIT(4)
#define NPCM_JTAGM_CTL_TRST_OUT		BIT(7)
#define NPCM_JTAGM_CTL_INSMP_EJ		BIT(12)
#define NPCM_JTAGM_CTL_CKDV_POS		16
#define NPCM_JTAGM_CTL_CKDV_MASK	GENMASK(23, 16)

#define NPCM_JTAGM_STAT_BUSY		BIT(0)
#define NPCM_JTAGM_STAT_DONE		BIT(1)

#define NPCM_JTAGM_CMD_ST_OP		BIT(0)
#define NPCM_JTAGM_CMD_CK_CNT		GENMASK(15, 8)

/* Datasheet default divide-by-10: (CKDV + 1) * 2 */
#define NPCM_JTAGM_CKDV_DEFAULT		4
/* HW FIFO depth in bits */
#define NPCM_JTAGM_FIFO_BITS		128U

static inline uint32_t jtag_npcm_reg_read(const struct jtag_npcm_cfg *config, uint32_t reg)
{
	return sys_read32(config->base + reg);
}

static inline void jtag_npcm_reg_write(const struct jtag_npcm_cfg *config, uint32_t reg, uint32_t val)
{
	sys_write32(val, config->base + reg);
}

static void jtag_npcm_isr(const struct device *dev)
{
	const struct jtag_npcm_cfg *config = DEV_CFG(dev);
	struct jtag_npcm_data *data = DEV_DATA(dev);
	uint32_t stat, ctl;

	stat = jtag_npcm_reg_read(config, NPCM_JTAGM_STAT);

	if (stat & NPCM_JTAGM_STAT_DONE) {
		/* W1C: clear DONE */
		jtag_npcm_reg_write(config, NPCM_JTAGM_STAT, NPCM_JTAGM_STAT_DONE);
		/* disable interrupt until next transfer */
		ctl = jtag_npcm_reg_read(config, NPCM_JTAGM_CTL);
		ctl &= ~NPCM_JTAGM_CTL_DONE_IE;
		jtag_npcm_reg_write(config, NPCM_JTAGM_CTL, ctl);
		k_sem_give(&data->done_sem);
	}
}

/*
 * Fill TDO_OUT and TMS_OUT FIFOs with up to NPCM_JTAGM_FIFO_BITS bits
 * of data, then kick off the transfer via JTAGM_CMD.
 *
 * @tx_buf   : outgoing bit-stream (master → target); NULL → send zeros
 * @tms_buf  : TMS bit-stream; NULL → all zeros (stay in Shift-xR state)
 * @tx_bits  : total remaining bits to transfer in this call-chain
 * @end_tms_high : assert TMS on the very last bit of this chunk
 *
 * Returns the number of bits actually queued (≤ NPCM_JTAGM_FIFO_BITS).
 */
static uint32_t jtag_npcm_hw_send(const struct jtag_npcm_cfg *config,
				   const uint8_t *tx_buf, const uint8_t *tms_buf,
				   uint32_t tx_bits, bool end_tms_high)
{
	const uint32_t *tdo32 = (const uint32_t *)tx_buf;
	const uint32_t *tms32 = (const uint32_t *)tms_buf;
	const uint8_t  *tdo8, *tms8;
	uint32_t cnt = MIN(tx_bits, NPCM_JTAGM_FIFO_BITS);
	uint32_t val, tmsval;
	int words = cnt / 32;
	int bytes = DIV_ROUND_UP(cnt % 32, 8);
	int n, i;

	/* Write full 32-bit words */
	for (n = 0; n < words; n++) {
		val    = tdo32 ? tdo32[n] : 0U;
		tmsval = tms32 ? tms32[n] : 0U;
		/* Set TMS=1 on the MSB of the last word when requested */
		if (end_tms_high && !bytes && (n == words - 1))
			tmsval |= BIT(31);
		jtag_npcm_reg_write(config, NPCM_JTAGM_TDO_OUT(n), val);
		jtag_npcm_reg_write(config, NPCM_JTAGM_TMS_OUT(n), tmsval);
	}

	/* Write the leftover partial word */
	if (bytes) {
		tdo8 = tdo32 ? (const uint8_t *)&tdo32[words] : NULL;
		tms8 = tms32 ? (const uint8_t *)&tms32[words] : NULL;
		val    = 0U;
		tmsval = 0U;
		for (i = 0; i < bytes; i++) {
			if (tdo8)
				val    |= (uint32_t)tdo8[i] << (i * 8);
			if (tms8)
				tmsval |= (uint32_t)tms8[i] << (i * 8);
		}
		if (end_tms_high)
			tmsval |= BIT((cnt % 32) - 1);
		jtag_npcm_reg_write(config, NPCM_JTAGM_TDO_OUT(n), val);
		jtag_npcm_reg_write(config, NPCM_JTAGM_TMS_OUT(n), tmsval);
	}

	/* Kick off the hardware transfer */
	jtag_npcm_reg_write(config, NPCM_JTAGM_CMD,
			    FIELD_PREP(NPCM_JTAGM_CMD_CK_CNT, cnt) |
			    NPCM_JTAGM_CMD_ST_OP);

	return cnt;
}

/*
 * Read cnt bits from TDI_IN FIFO into rx_buf.
 * rx_buf may be NULL (discard received data).
 */
static void jtag_npcm_hw_recv(const struct jtag_npcm_cfg *config,
			      uint8_t *rx_buf, uint32_t cnt)
{
	uint32_t *buf32 = (uint32_t *)rx_buf;
	uint8_t  *buf8;
	uint32_t val;
	int words = cnt / 32;
	int bytes = DIV_ROUND_UP(cnt % 32, 8);
	int n, i;

	for (n = 0; n < words; n++) {
		val = jtag_npcm_reg_read(config, NPCM_JTAGM_TDI_IN(n));
		if (buf32)
			buf32[n] = val;
	}

	if (bytes) {
		buf8 = buf32 ? (uint8_t *)&buf32[words] : NULL;
		val  = jtag_npcm_reg_read(config, NPCM_JTAGM_TDI_IN(n));
		if (buf8)
			for (i = 0; i < bytes; i++)
				buf8[i] = (val >> (i * 8)) & 0xFFU;
	}
}

/*
 * Send a TMS-only sequence through the JTAGM HW (TDO=0).
 * Used for TAP state transitions and idle TCK clocking.
 * tms_buf=NULL means TMS stays 0 throughout.
 */
static int jtag_npcm_hw_tms_shift(const struct device *dev,
				   const uint8_t *tms_buf, uint32_t bits)
{
	const struct jtag_npcm_cfg *config = DEV_CFG(dev);
	struct jtag_npcm_data *data = DEV_DATA(dev);
	uint32_t remain = bits;
	uint32_t offset = 0U;
	uint32_t ck_cnt;
	uint32_t ctl;
	int ret;

	while (remain > 0) {
		ctl = jtag_npcm_reg_read(config, NPCM_JTAGM_CTL);
		ctl |= NPCM_JTAGM_CTL_DONE_IE;
		jtag_npcm_reg_write(config, NPCM_JTAGM_CTL, ctl);

		ck_cnt = jtag_npcm_hw_send(config,
					    NULL,   /* TDO = 0 */
					    tms_buf ? tms_buf + offset : NULL,
					    remain,
					    false);

		ret = k_sem_take(&data->done_sem, K_MSEC(10000));
		if (ret) {
			LOG_ERR("JTAGM TMS shift timed out");
			ctl = jtag_npcm_reg_read(config, NPCM_JTAGM_CTL);
			ctl &= ~NPCM_JTAGM_CTL_DONE_IE;
			jtag_npcm_reg_write(config, NPCM_JTAGM_CTL, ctl);
			return -ETIMEDOUT;
		}
		/* No rx for TMS-only shift */

		offset += ck_cnt / 8U;
		remain -= ck_cnt;
	}
	return 0;
}

static void jtag_npcm_set_tap_state(const struct device *dev,
				     enum jtag_npcm_tap_states from,
				     enum jtag_npcm_tap_states to)
{
	/* 9 bits of TMS=1 guarantee reaching Test-Logic Reset from any state */
	static const uint8_t tms_reset[2] = { 0xFF, 0x01 };
	uint8_t tmsbuf[2];
	uint8_t tmsbits;
	uint8_t count;

	if (from == to) {
		return;
	}
	if (from == NPCM_TAP_CURRENT_STATE) {
		from = gjtag.tap_state;
	}
	if (from > NPCM_TAP_CURRENT_STATE || to > NPCM_TAP_CURRENT_STATE) {
		return;
	}

	if (to == NPCM_TAP_RESET) {
		jtag_npcm_hw_tms_shift(dev, tms_reset, 9);
		gjtag.tap_state = NPCM_TAP_RESET;
		return;
	}

	tmsbits = _tms_cycle_lookup[from][to].tms_bits;
	count   = _tms_cycle_lookup[from][to].count;

	if (count == 0) {
		return;
	}

	/* Pack tmsbits into a byte buffer for the HW */
	tmsbuf[0] = tmsbits;
	tmsbuf[1] = 0;
	jtag_npcm_hw_tms_shift(dev, tmsbuf, count);

	gjtag.tap_state = to;
}

static enum jtag_npcm_tap_states jtag_npcm_get_tap_state(void)
{
	return gjtag.tap_state;
}

/*
 * Hardware (JTAGM FIFO) data transfer
 *
 * Feeds the 128-bit HW FIFO in a loop; waits for IRQ via semaphore.
 * TMS is 0 for every bit except the very last one when end_tms_high=true,
 * which causes the TAP to exit Shift-xR on that final clock edge.
 *
 * out_data / in_data may be NULL (send zeros / discard received data).
 */
static int jtag_npcm_shift(const struct device *dev,
			       uint32_t out_bytes_len, const uint8_t *out_data,
			       uint32_t in_byte_len, uint8_t *in_data,
			       bool end_tms_high)
{
	const struct jtag_npcm_cfg *config = DEV_CFG(dev);
	struct jtag_npcm_data *data = DEV_DATA(dev);
	uint32_t total_bits = MAX(out_bytes_len, in_byte_len) * 8U;
	uint32_t offset = 0U;
	uint32_t ck_cnt;
	uint32_t ctl;
	int ret;

	if (jtag_npcm_reg_read(config, NPCM_JTAGM_STAT) & NPCM_JTAGM_STAT_BUSY) {
		LOG_ERR("JTAGM HW busy at xfer start");
		return -EBUSY;
	}

	while (total_bits > 0) {
		/* Only the last FIFO chunk gets end_tms_high */
		bool is_last = (total_bits <= NPCM_JTAGM_FIFO_BITS);

		/* Enable DONE interrupt before kicking HW (avoids race with ISR) */
		ctl = jtag_npcm_reg_read(config, NPCM_JTAGM_CTL);
		ctl |= NPCM_JTAGM_CTL_DONE_IE;
		jtag_npcm_reg_write(config, NPCM_JTAGM_CTL, ctl);

		ck_cnt = jtag_npcm_hw_send(config,
					    out_data ? out_data + offset : NULL,
					    NULL,       /* TMS = 0 (shift state) */
					    total_bits,
					    end_tms_high && is_last);

		/* Wait for ISR to signal completion (10 s timeout) */
		ret = k_sem_take(&data->done_sem, K_MSEC(10000));
		if (ret) {
			LOG_ERR("JTAGM HW transfer timed out");
			ctl = jtag_npcm_reg_read(config, NPCM_JTAGM_CTL);
			ctl &= ~NPCM_JTAGM_CTL_DONE_IE;
			jtag_npcm_reg_write(config, NPCM_JTAGM_CTL, ctl);
			return -ETIMEDOUT;
		}

		jtag_npcm_hw_recv(config,
				  in_data ? in_data + offset : NULL,
				  ck_cnt);

		offset     += ck_cnt / 8U;
		total_bits -= ck_cnt;
	}

	return 0;
}

static void jtag_npcm_readwrite_scan(const struct device *dev, int bits_len,
		const uint8_t *out_data, uint8_t *in_data,
		enum jtag_npcm_tap_states end_state)
{
	bool end_tms_high = false;

	/*
	 * If we need to leave Shift-xR, raise TMS on the last clock edge.
	 * The TAP enters Exit1-xR automatically — update tapstate now.
	 */
	if (end_state != NPCM_TAP_SHIFT_DR &&
	    end_state != NPCM_TAP_SHIFT_IR &&
	    end_state != NPCM_TAP_CURRENT_STATE) {
		end_tms_high = true;
		gjtag.tap_state = (gjtag.tap_state == NPCM_TAP_SHIFT_DR) ?
				  NPCM_TAP_EXIT1_DR : NPCM_TAP_EXIT1_IR;
	}

	if (jtag_npcm_shift(dev,
			       DIV_ROUND_UP(bits_len, 8), out_data,
			       DIV_ROUND_UP(bits_len, 8), in_data,
			       end_tms_high) < 0)
		return;

	jtag_npcm_set_tap_state(dev, NPCM_TAP_CURRENT_STATE, end_state);
}

static void jtag_npcm_ir_scan(const struct device *dev, int bits_len, const uint8_t *out_data,
		uint8_t *in_data, enum jtag_npcm_tap_states end_state)
{
	if (bits_len == 0) {
		return;
	}

	jtag_npcm_set_tap_state(dev, NPCM_TAP_CURRENT_STATE, NPCM_TAP_SHIFT_IR);

	jtag_npcm_readwrite_scan(dev, bits_len, out_data, in_data, end_state);
}

static void jtag_npcm_dr_scan(const struct device *dev, int bits_len, const uint8_t *out_data,
		uint8_t *in_data, enum jtag_npcm_tap_states end_state)
{
	if (bits_len == 0) {
		return;
	}

	jtag_npcm_set_tap_state(dev, NPCM_TAP_CURRENT_STATE, NPCM_TAP_SHIFT_DR);

	jtag_npcm_readwrite_scan(dev, bits_len, out_data, in_data, end_state);
}

/* implement general function for device API */

static enum jtag_npcm_tap_states jtag_npcm_covert_tap_state(enum tap_state state)
{
	enum jtag_npcm_tap_states npcm_state = NPCM_TAP_CURRENT_STATE;

	switch(state) {
		case TAP_INVALID:
			npcm_state = NPCM_TAP_CURRENT_STATE;
			break;
		case TAP_DREXIT2:
			npcm_state = NPCM_TAP_EXIT2_DR;
			break;
		case TAP_DREXIT1:
			npcm_state = NPCM_TAP_EXIT1_DR;
			break;
		case TAP_DRSHIFT:
			npcm_state = NPCM_TAP_SHIFT_DR;
			break;
		case TAP_DRPAUSE:
			npcm_state = NPCM_TAP_PAUSE_DR;
			break;
		case TAP_IRSELECT:
			npcm_state = NPCM_TAP_SELECT_IR;
			break;
		case TAP_DRUPDATE:
			npcm_state = NPCM_TAP_UPDATE_DR;
			break;
		case TAP_DRCAPTURE:
			npcm_state = NPCM_TAP_CAPTURE_DR;
			break;
		case TAP_DRSELECT:
			npcm_state = NPCM_TAP_SELECT_DR;
			break;
		case TAP_IREXIT2:
			npcm_state = NPCM_TAP_EXIT2_IR;
			break;
		case TAP_IREXIT1:
			npcm_state = NPCM_TAP_EXIT1_IR;
			break;
		case TAP_IRSHIFT:
			npcm_state = NPCM_TAP_SHIFT_IR;
			break;
		case TAP_IRPAUSE:
			npcm_state = NPCM_TAP_PAUSE_IR;
			break;
		case TAP_IDLE:
			npcm_state = NPCM_TAP_IDLE;
			break;
		case TAP_IRUPDATE:
			npcm_state = NPCM_TAP_UPDATE_IR;
			break;
		case TAP_IRCAPTURE:
			npcm_state = NPCM_TAP_CAPTURE_IR;
			break;
		case TAP_RESET:
			npcm_state = NPCM_TAP_RESET;
			break;
		default:
			npcm_state = NPCM_TAP_CURRENT_STATE;
			break;
	}

	return npcm_state;
}

static enum tap_state jtag_npcm_covert_npcm_tap_state(enum jtag_npcm_tap_states npcm_tap_state)
{
	enum tap_state state = TAP_INVALID;

	switch(npcm_tap_state) {
		case NPCM_TAP_RESET:
			state = TAP_RESET;
			break;
		case NPCM_TAP_IDLE:
			state = TAP_IDLE;
			break;
		case NPCM_TAP_SELECT_DR:
			state = TAP_DRSELECT;
			break;
		case NPCM_TAP_CAPTURE_DR:
			state = TAP_DRCAPTURE;
			break;
		case NPCM_TAP_SHIFT_DR:
			state = TAP_DRSHIFT;
			break;
		case NPCM_TAP_EXIT1_DR:
			state = TAP_DREXIT1;
			break;
		case NPCM_TAP_PAUSE_DR:
			state = TAP_DRPAUSE;
			break;
		case NPCM_TAP_EXIT2_DR:
			state = TAP_DREXIT2;
			break;
		case NPCM_TAP_UPDATE_DR:
			state = TAP_DRUPDATE;
			break;
		case NPCM_TAP_SELECT_IR:
			state = TAP_IRSELECT;
			break;
		case NPCM_TAP_CAPTURE_IR:
			state = TAP_IRCAPTURE;
			break;
		case NPCM_TAP_SHIFT_IR:
			state = TAP_IRSHIFT;
			break;
		case NPCM_TAP_EXIT1_IR:
			state = TAP_IREXIT1;
			break;
		case NPCM_TAP_PAUSE_IR:
			state = TAP_IRPAUSE;
			break;
		case NPCM_TAP_EXIT2_IR:
			state = TAP_IREXIT2;
			break;
		case NPCM_TAP_UPDATE_IR:
			state = TAP_IRUPDATE;
			break;
		default:
			state = TAP_INVALID;
			break;
	}

	return state;
}

int jtag_npcm_freq_get(const struct device *dev, uint32_t *freq)
{
	const struct jtag_npcm_cfg *config = DEV_CFG(dev);
	struct jtag_npcm_data *data = DEV_DATA(dev);
	uint32_t ctl;
	uint32_t ckdv;

	if (data->src_clk_hz == 0U) {
		return -ENOTSUP;
	}

	ctl = jtag_npcm_reg_read(config, NPCM_JTAGM_CTL);
	ckdv = FIELD_GET(NPCM_JTAGM_CTL_CKDV_MASK, ctl);
	*freq = data->src_clk_hz / ((ckdv + 1U) * 2U);
	data->tck_freq_hz = *freq;

	return 0;
}

int jtag_npcm_freq_set(const struct device *dev, uint32_t freq)
{
	const struct jtag_npcm_cfg *config = DEV_CFG(dev);
	struct jtag_npcm_data *data = DEV_DATA(dev);
	uint32_t ctl;
	uint32_t ckdv;
	uint32_t div;

	if (data->src_clk_hz == 0U) {
		return -ENOTSUP;
	}

	if (freq == 0U) {
		return -EINVAL;
	}

	/* CKDV setting only when JTAG master is disabled */
	ctl = jtag_npcm_reg_read(config, NPCM_JTAGM_CTL);
	ctl &= ~NPCM_JTAGM_CTL_JTAGM_EN;
	jtag_npcm_reg_write(config, NPCM_JTAGM_CTL, ctl);

	div = DIV_ROUND_UP(data->src_clk_hz, freq);
	div = DIV_ROUND_UP(div, 2U);
	if (div == 0U) {
		div = 1U;
	}

	ckdv = div - 1U;
	if (ckdv > 0xFFU) {
		ckdv = 0xFFU;
	}

	ctl = jtag_npcm_reg_read(config, NPCM_JTAGM_CTL);
	ctl &= ~NPCM_JTAGM_CTL_CKDV_MASK;

	ctl |= FIELD_PREP(NPCM_JTAGM_CTL_CKDV_MASK, ckdv);
	ctl |= NPCM_JTAGM_CTL_JTAGM_EN;
	jtag_npcm_reg_write(config, NPCM_JTAGM_CTL, ctl);

	data->tck_freq_hz = data->src_clk_hz / ((ckdv + 1U) * 2U);

	return 0;
}

int jtag_npcm_tap_get(const struct device *dev, enum tap_state *state)
{
	*state = jtag_npcm_covert_npcm_tap_state(jtag_npcm_get_tap_state());

	return 0;
}

static int jtag_npcm_tap_set(const struct device *dev, enum tap_state state)
{
	enum jtag_npcm_tap_states end_state = NPCM_TAP_CURRENT_STATE;

	end_state = jtag_npcm_covert_tap_state(state);

	if (end_state == NPCM_TAP_CURRENT_STATE)
		return -EINVAL;

	jtag_npcm_set_tap_state(dev, NPCM_TAP_CURRENT_STATE, end_state);

	return 0;
}

static int jtag_npcm_tck_run(const struct device *dev, uint32_t run_count)
{
	return jtag_npcm_hw_tms_shift(dev, NULL, run_count);
}

static int jtag_npcm_xfer(const struct device *dev, struct scan_command_s *scan)
{
	struct scan_field_s *fields;
	enum jtag_npcm_tap_states npcm_state = NPCM_TAP_CURRENT_STATE;
	const struct jtag_npcm_cfg *config = DEV_CFG(dev);
	uint32_t stat;

	stat = jtag_npcm_reg_read(config, NPCM_JTAGM_STAT);

	if (stat & NPCM_JTAGM_STAT_BUSY) {
		LOG_ERR("JTAGM is busy");
		return -EBUSY;
	}

	fields = &scan->fields;

	npcm_state = jtag_npcm_covert_tap_state(scan->end_state);

	if (npcm_state == NPCM_TAP_CURRENT_STATE)
		return -1;

	if (scan->ir_scan) {
		jtag_npcm_ir_scan(dev, fields->num_bits,
				fields->out_value, fields->in_value, npcm_state);
	} else {
		jtag_npcm_dr_scan(dev, fields->num_bits,
				fields->out_value, fields->in_value, npcm_state);
	}

	return 0;
}

static int jtag_npcm_sw_xfer(const struct device *dev, enum jtag_pin pin,
		uint8_t value)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(pin);
	ARG_UNUSED(value);
	return -ENOTSUP;
}

static int jtag_npcm_tdo_get(const struct device *dev, uint8_t *value)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(value);
	return -ENOTSUP;
}

static int jtag_npcm_init(const struct device *dev)
{
	const struct jtag_npcm_cfg *config = DEV_CFG(dev);
	struct jtag_npcm_data *data = DEV_DATA(dev);
	const struct device *const clk_dev = DEVICE_DT_GET(DT_NODELABEL(pcc));
	int ret;
	uint32_t ctl;

	/* --- Clock --- */
	if (!device_is_ready(clk_dev)) {
		LOG_ERR("Clock control device not ready");
		return -ENODEV;
	}

	/* --- Pinctrl --- */
	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Failed to apply pinctrl state: %d", ret);
		return ret;
	}

	ret = clock_control_on(clk_dev, (clock_control_subsys_t)config->clk_cfg);
	if (ret < 0) {
		LOG_ERR("Failed to enable JTAGM clock: %d", ret);
		return ret;
	}

	ret = clock_control_get_rate(clk_dev, (clock_control_subsys_t)config->clk_cfg,
				     &data->src_clk_hz);
	if (ret < 0) {
		LOG_ERR("Failed to get JTAGM clock rate: %d", ret);
		return ret;
	}

	/* --- Semaphore (IRQ  xfer synchronisation) --- */
	k_sem_init(&data->done_sem, 0, 1);

	/* --- HW init (mirrors Linux npcm_jtm_probe) --- */

	/* Disable JTAGM while configuring */
	ctl = jtag_npcm_reg_read(config, NPCM_JTAGM_CTL);
	ctl &= ~NPCM_JTAGM_CTL_JTAGM_EN;
	jtag_npcm_reg_write(config, NPCM_JTAGM_CTL, ctl);

	/* Clear all bits we will configure, keep others */
	ctl &= ~(NPCM_JTAGM_CTL_CKDV_MASK | NPCM_JTAGM_CTL_INSMP_EJ |
		 NPCM_JTAGM_CTL_DONE_IE);
	/* Deassert TRST for normal operation */
	ctl |= NPCM_JTAGM_CTL_TRST_OUT;
	jtag_npcm_reg_write(config, NPCM_JTAGM_CTL, ctl);

	/* Clear any stale DONE flag */
	jtag_npcm_reg_write(config, NPCM_JTAGM_STAT, NPCM_JTAGM_STAT_DONE);

	/* Set default baud rate (DONE_IE left off; enabled per-transfer) */
	jtag_npcm_freq_set(dev, DEFAULT_JTAG_FREQ);

	/* Enable JTAGM */
	ctl = jtag_npcm_reg_read(config, NPCM_JTAGM_CTL);
	ctl |= NPCM_JTAGM_CTL_JTAGM_EN;
	jtag_npcm_reg_write(config, NPCM_JTAGM_CTL, ctl);

	/* Register and enable IRQ */
	config->irq_cfg_func(dev);

	LOG_INF("JTAGM source clock = %u Hz", data->src_clk_hz);
	LOG_INF("JTAGM TCK clock = %u Hz", DEFAULT_JTAG_FREQ);

	data->tck_freq_hz = DEFAULT_JTAG_FREQ;
	/* Test-Logic Reset state */
	gjtag.tap_state = NPCM_TAP_RESET;

	return 0;
}

static struct jtag_driver_api jtag_npcm_api = {
	.freq_get = jtag_npcm_freq_get,
	.freq_set = jtag_npcm_freq_set,
	.tap_get = jtag_npcm_tap_get,
	.tap_set = jtag_npcm_tap_set,
	.tck_run = jtag_npcm_tck_run,
	.xfer = jtag_npcm_xfer,
	.sw_xfer = jtag_npcm_sw_xfer,
	.tdo_get = jtag_npcm_tdo_get,
};

#define NPCM_JTAG_IRQ_HANDLER(n)                                                         \
	static void jtag_npcm_irq_cfg_func_##n(const struct device *dev)                  \
	{                                                                                   \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), jtag_npcm_isr,       \
			    DEVICE_DT_INST_GET(n), 0);                                          \
		irq_enable(DT_INST_IRQN(n));                                                \
	}

#define NPCM_JTAG_IRQ_HANDLER_FUNC(n) .irq_cfg_func = jtag_npcm_irq_cfg_func_##n,

#define NPCM_JTAG_INIT(n)								\
	PINCTRL_DT_INST_DEFINE(n);						\
	NPCM_JTAG_IRQ_HANDLER(n)						\
	static struct jtag_npcm_data jtag_npcm_data_##n;				\
	static const struct jtag_npcm_cfg jtag_npcm_cfg_##n = {			\
		.base = DT_INST_REG_ADDR(n),					\
		.clk_cfg = DT_INST_PHA(n, clocks, clk_cfg),			\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),			\
		NPCM_JTAG_IRQ_HANDLER_FUNC(n)					\
	};										\
	DEVICE_DT_INST_DEFINE(n, jtag_npcm_init, NULL,				\
			&jtag_npcm_data_##n, &jtag_npcm_cfg_##n,			\
			POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
			&jtag_npcm_api);

DT_INST_FOREACH_STATUS_OKAY(NPCM_JTAG_INIT)
