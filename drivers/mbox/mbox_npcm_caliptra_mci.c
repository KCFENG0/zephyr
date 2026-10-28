/*
 * Copyright (c) 2026 Nuvoton Technology Corporation
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT nuvoton_npcm_caliptra_mci_mbox

#include <zephyr/device.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/logging/log.h>

#include <zephyr/drivers/mbox/mbox_npcm_caliptra_mci_proto.h>
#include "mbox_npcm_caliptra_mci_regs.h"

LOG_MODULE_REGISTER(mbox_npcm_caliptra_mci, CONFIG_MBOX_LOG_LEVEL);

#define CALIPTRA_MCI_MBOX_CHANNELS     2
#define CALIPTRA_MCI_MBOX_CH_REQUESTER 0
#define CALIPTRA_MCI_MBOX_CH_RESPONDER 1

/* Bounded retry for acquire: absorbs the HW zeroization window after a
 * release. Total wait ~= RETRIES * RETRY_US. Defaults ~5ms - plenty for
 * an empty-payload acquire/release cycle; tune up if large DLEN sessions
 * push zeroization time higher. */
#define CALIPTRA_MCI_MBOX_ACQUIRE_RETRIES  100
#define CALIPTRA_MCI_MBOX_ACQUIRE_RETRY_US 50

struct caliptra_mci_mbox_config {
	mem_addr_t sram_base;
	mem_addr_t csr_base;
	size_t sram_size;
	uint32_t poll_interval_ms;
	unsigned int irq;
};

struct caliptra_mci_mbox_channel {
	mbox_callback_t cb;
	void *user_data;
	enum caliptra_mci_mbox_state state;
	bool enabled;
};

struct caliptra_mci_mbox_data {
	struct caliptra_mci_mbox_channel ch[CALIPTRA_MCI_MBOX_CHANNELS];
	struct k_spinlock lock;
#ifdef CONFIG_MBOX_NPCM_CALIPTRA_MCI_IRQ
	struct k_timer timer;
#endif
};

static inline uint32_t csr_rd(const struct caliptra_mci_mbox_config *cfg, uint32_t off)
{
	return sys_read32(cfg->csr_base + off);
}

static inline void csr_wr(const struct caliptra_mci_mbox_config *cfg, uint32_t off,
			  uint32_t val)
{
	sys_write32(val, cfg->csr_base + off);
}

#ifdef CONFIG_MBOX_NPCM_CALIPTRA_MCI_IRQ
/*
 * The M7 interrupt is soc_mcu_mbox*_data_avail = EXECUTE && USER == MCU, so
 * it only covers MCU-initiated requests. Requester completion (CMD_STATUS)
 * and responder close (MCU clearing EXECUTE) raise nothing on the M7 side;
 * this one-shot timer watches them only while one is outstanding.
 */
static void caliptra_mci_mbox_kick(const struct device *dev)
{
	const struct caliptra_mci_mbox_config *cfg = dev->config;
	struct caliptra_mci_mbox_data *data = dev->data;

	k_timer_start(&data->timer, K_MSEC(cfg->poll_interval_ms), K_NO_WAIT);
}
#else
static inline void caliptra_mci_mbox_kick(const struct device *dev)
{
	ARG_UNUSED(dev);
}
#endif

int caliptra_mci_mbox_acquire(const struct device *dev)
{
	const struct caliptra_mci_mbox_config *cfg = dev->config;
	struct caliptra_mci_mbox_data *data = dev->data;
	struct caliptra_mci_mbox_channel *ch = &data->ch[CALIPTRA_MCI_MBOX_CH_REQUESTER];
	enum caliptra_mci_mbox_state next;
	int ret;

	K_SPINLOCK(&data->lock) {
		ret = caliptra_mci_mbox_state_next(ch->state,
					       CALIPTRA_MCI_MBOX_EV_ACQUIRE, &next);
		if (ret == 0) {
			ch->state = next;
		}
	}
	if (ret != 0) {
		return ret;
	}

	/* Reading LOCK returns 0 and acquires if free, 1 if already held.
	 * Per spec: after release, HW zeroes SRAM[0..max-DLEN] and blocks
	 * re-acquire until that completes. Retry a bounded window so a
	 * release-followed-by-acquire from the same caller succeeds. */
	for (int i = 0; i < CALIPTRA_MCI_MBOX_ACQUIRE_RETRIES; i++) {
		if ((csr_rd(cfg, CALIPTRA_MCI_MBOX_REG_LOCK) &
		     CALIPTRA_MCI_MBOX_LOCK_BIT) == 0U) {
			return 0;
		}
		k_busy_wait(CALIPTRA_MCI_MBOX_ACQUIRE_RETRY_US);
	}

	K_SPINLOCK(&data->lock) {
		ch->state = CALIPTRA_MCI_MBOX_STATE_IDLE;
	}
	return -EBUSY;
}

void caliptra_mci_mbox_release(const struct device *dev)
{
	const struct caliptra_mci_mbox_config *cfg = dev->config;
	struct caliptra_mci_mbox_data *data = dev->data;
	struct caliptra_mci_mbox_channel *ch = &data->ch[CALIPTRA_MCI_MBOX_CH_REQUESTER];
	enum caliptra_mci_mbox_state next;

	csr_wr(cfg, CALIPTRA_MCI_MBOX_REG_EXECUTE, 0U);

	K_SPINLOCK(&data->lock) {
		(void)caliptra_mci_mbox_state_next(ch->state,
					       CALIPTRA_MCI_MBOX_EV_RELEASE, &next);
		ch->state = next;
	}
}

int caliptra_mci_mbox_write_payload(const struct device *dev, uint32_t cmd,
				const void *buf, size_t len)
{
	const struct caliptra_mci_mbox_config *cfg = dev->config;
	struct caliptra_mci_mbox_data *data = dev->data;
	struct caliptra_mci_mbox_channel *ch = &data->ch[CALIPTRA_MCI_MBOX_CH_REQUESTER];
	const uint32_t *words = buf;
	enum caliptra_mci_mbox_state next;
	int ret;

	ret = caliptra_mci_mbox_check_len(len, cfg->sram_size);
	if (ret != 0) {
		return ret;
	}

	K_SPINLOCK(&data->lock) {
		ret = caliptra_mci_mbox_state_next(ch->state,
					       CALIPTRA_MCI_MBOX_EV_STAGE, &next);
		if (ret == 0) {
			ch->state = next;
		}
	}
	if (ret != 0) {
		return ret;
	}

	csr_wr(cfg, CALIPTRA_MCI_MBOX_REG_CMD, cmd);
	csr_wr(cfg, CALIPTRA_MCI_MBOX_REG_DLEN, (uint32_t)len);
	for (size_t i = 0; i < len / 4U; i++) {
		sys_write32(words[i], cfg->sram_base + i * 4U);
	}
#ifdef CONFIG_MBOX_NPCM_CALIPTRA_MCI_FORCE_TARGET_USER
	/* FPGA bring-up quirk: strictly out-of-spec for the SOC Sender -> MCU
	 * flow, but earlier images failed to wake the MCU's CMD_AVAIL handler
	 * without it. Guarded by Kconfig so prod builds stay spec-clean. */
	csr_wr(cfg, CALIPTRA_MCI_MBOX_REG_TARGET_USER, CALIPTRA_MCI_MBOX_USER_SOC);
	csr_wr(cfg, CALIPTRA_MCI_MBOX_REG_TARGET_USER_VALID,
	       CALIPTRA_MCI_MBOX_TARGET_USER_VALID_BIT);
#endif
	return 0;
}

int caliptra_mci_mbox_read_payload(const struct device *dev, uint32_t *cmd,
			       void *buf, size_t cap, size_t *out_len)
{
	const struct caliptra_mci_mbox_config *cfg = dev->config;
	uint32_t *words = buf;
	size_t len;

	len = csr_rd(cfg, CALIPTRA_MCI_MBOX_REG_DLEN);
	if (len > cap || len > cfg->sram_size) {
		return -ENOSPC;
	}

	if (cmd != NULL) {
		*cmd = csr_rd(cfg, CALIPTRA_MCI_MBOX_REG_CMD);
	}
	for (size_t i = 0; i < len / 4U; i++) {
		words[i] = sys_read32(cfg->sram_base + i * 4U);
	}
	if (out_len != NULL) {
		*out_len = len;
	}
	return 0;
}

/*
 * Spec "MCU to SOC Receiver" steps 8 + 9: stage SRAM/DLEN, then commit by
 * writing TARGET_STATUS = DONE | status. Channel state transitions
 * LOCKED -> CLOSING; dispatch will move it back to IDLE once the MCU has
 * cleared EXECUTE (step 10). Reply payload is owned entirely by the caller;
 * the driver only transcribes.
 *
 * Write order is significant: SRAM before DLEN before TARGET_STATUS, so the
 * requester never observes the DONE flag with stale data behind it.
 */
int caliptra_mci_mbox_write_response(const struct device *dev,
				 enum caliptra_mci_mbox_status status,
				 const void *buf, size_t len)
{
	const struct caliptra_mci_mbox_config *cfg = dev->config;
	struct caliptra_mci_mbox_data *data = dev->data;
	struct caliptra_mci_mbox_channel *ch = &data->ch[CALIPTRA_MCI_MBOX_CH_RESPONDER];
	const uint32_t *words = buf;
	enum caliptra_mci_mbox_state next;
	int ret;

	ret = caliptra_mci_mbox_check_len(len, cfg->sram_size);
	if (ret != 0) {
		return ret;
	}

	K_SPINLOCK(&data->lock) {
		ret = caliptra_mci_mbox_state_next(ch->state,
					       CALIPTRA_MCI_MBOX_EV_RESPOND, &next);
	}
	if (ret != 0) {
		return ret;
	}

	for (size_t i = 0; i < len / 4U; i++) {
		sys_write32(words[i], cfg->sram_base + i * 4U);
	}
	csr_wr(cfg, CALIPTRA_MCI_MBOX_REG_DLEN, (uint32_t)len);
	csr_wr(cfg, CALIPTRA_MCI_MBOX_REG_TARGET_STATUS,
	       CALIPTRA_MCI_MBOX_STATUS_DONE_BIT | (uint32_t)status);

	/* Only dispatch moves a responder out of LOCKED, and it never does, so
	 * committing CLOSING after DONE is race-free. Committing it earlier
	 * would let dispatch see CLOSING with DONE still 0 and take it as
	 * already closed. */
	K_SPINLOCK(&data->lock) {
		ch->state = next;
	}
	caliptra_mci_mbox_kick(dev);
	return 0;
}

enum caliptra_mci_mbox_status caliptra_mci_mbox_get_status(const struct device *dev)
{
	const struct caliptra_mci_mbox_config *cfg = dev->config;

	return caliptra_mci_mbox_decode_status(
		csr_rd(cfg, CALIPTRA_MCI_MBOX_REG_CMD_STATUS));
}

int caliptra_mci_mbox_check_ecc(const struct device *dev)
{
	const struct caliptra_mci_mbox_config *cfg = dev->config;
	uint32_t hw = csr_rd(cfg, CALIPTRA_MCI_MBOX_REG_HW_STATUS);

	if ((hw & CALIPTRA_MCI_MBOX_HW_ECC_DOUBLE_BIT) != 0U) {
		return -EIO;
	}
	if ((hw & CALIPTRA_MCI_MBOX_HW_ECC_SINGLE_BIT) != 0U) {
		return 1;
	}
	return 0;
}

/*
 * Single notification chokepoint. The poll thread, the ISR and the IRQ-mode
 * timer all call this. It decodes mailbox state for the given channel and
 * invokes the registered mbox callback. In IRQ mode the callback runs in ISR
 * context.
 */
static void caliptra_mci_mbox_dispatch(const struct device *dev,
				   mbox_channel_id_t channel_id)
{
	const struct caliptra_mci_mbox_config *cfg = dev->config;
	struct caliptra_mci_mbox_data *data = dev->data;
	struct caliptra_mci_mbox_channel *ch = &data->ch[channel_id];
	bool fire = false;

	if (!ch->enabled || ch->cb == NULL) {
		return;
	}

	if (caliptra_mci_mbox_check_ecc(dev) == -EIO) {
		LOG_ERR("mbox%p: double-bit ECC error", dev);
	}

	if (channel_id == CALIPTRA_MCI_MBOX_CH_REQUESTER) {
		uint32_t st = csr_rd(cfg, CALIPTRA_MCI_MBOX_REG_CMD_STATUS);

		/* CMD_STATUS carries no DONE bit per spec; any non-BUSY code
		 * means the MCU has replied. */
		K_SPINLOCK(&data->lock) {
			if (ch->state == CALIPTRA_MCI_MBOX_STATE_BUSY &&
			    caliptra_mci_mbox_cmd_status_responded(st)) {
				ch->state = CALIPTRA_MCI_MBOX_STATE_LOCKED;
				fire = true;
			}
		}
	} else {
		/* EXECUTE is also raised by our own requester transactions. Only
		 * the MCU is ever our peer, so key on it holding the lock, as the
		 * data_avail interrupt does, rather than on our own AXI USER:
		 * that is whatever the fabric stamps on the M7, not necessarily
		 * CONFIG_MBOX_NPCM_CALIPTRA_MCI_BUSID. */
		if (csr_rd(cfg, CALIPTRA_MCI_MBOX_REG_USER) != CALIPTRA_MCI_MBOX_USER_MCU) {
			return;
		}

		uint32_t ex = csr_rd(cfg, CALIPTRA_MCI_MBOX_REG_EXECUTE);
		bool execute_asserted = (ex & CALIPTRA_MCI_MBOX_EXECUTE_BIT) != 0U;
		bool done = caliptra_mci_mbox_status_done(
			csr_rd(cfg, CALIPTRA_MCI_MBOX_REG_TARGET_STATUS));

		K_SPINLOCK(&data->lock) {
			/* Spec steps 10-11: MCU cleared EXECUTE, which HW-clears
			 * TARGET_STATUS.DONE. Key off DONE, not EXECUTE: the MCU
			 * may already have raised EXECUTE for the next request
			 * before we look, and that must still be seen below. */
			if (ch->state == CALIPTRA_MCI_MBOX_STATE_CLOSING && !done) {
				ch->state = CALIPTRA_MCI_MBOX_STATE_IDLE;
			}
			if (ch->state == CALIPTRA_MCI_MBOX_STATE_IDLE &&
			    execute_asserted) {
				/* Spec step 7: new request arrived. */
				ch->state = CALIPTRA_MCI_MBOX_STATE_LOCKED;
				fire = true;
			}
		}
	}

	if (fire) {
		ch->cb(dev, channel_id, ch->user_data, NULL);
	}
}

static int caliptra_mci_mbox_send(const struct device *dev,
			      mbox_channel_id_t channel_id,
			      const struct mbox_msg *msg)
{
	const struct caliptra_mci_mbox_config *cfg = dev->config;
	struct caliptra_mci_mbox_data *data = dev->data;
	struct caliptra_mci_mbox_channel *ch;
	enum caliptra_mci_mbox_state next;
	int ret;

	if (channel_id != CALIPTRA_MCI_MBOX_CH_REQUESTER) {
		/* Responder must use caliptra_mci_mbox_write_response() so SRAM,
		 * DLEN and TARGET_STATUS are committed together. */
		return -ENOTSUP;
	}

	/* Payload is staged via caliptra_mci_mbox_write_payload(); the doorbell
	 * carries no inline data. A non-NULL msg would be silently dropped,
	 * so reject it rather than pretend it was delivered. */
	if (msg != NULL) {
		return -ENOTSUP;
	}
	ch = &data->ch[channel_id];

	K_SPINLOCK(&data->lock) {
		ret = caliptra_mci_mbox_state_next(ch->state,
					       CALIPTRA_MCI_MBOX_EV_SEND, &next);
		if (ret == 0) {
			ch->state = next;
		}
	}
	if (ret != 0) {
		return ret;
	}

	csr_wr(cfg, CALIPTRA_MCI_MBOX_REG_EXECUTE, CALIPTRA_MCI_MBOX_EXECUTE_BIT);
	caliptra_mci_mbox_kick(dev);
	return 0;
}

static int caliptra_mci_mbox_register_callback(const struct device *dev,
					   mbox_channel_id_t channel_id,
					   mbox_callback_t cb, void *user_data)
{
	struct caliptra_mci_mbox_data *data = dev->data;

	if (channel_id >= CALIPTRA_MCI_MBOX_CHANNELS) {
		return -EINVAL;
	}

	K_SPINLOCK(&data->lock) {
		data->ch[channel_id].cb = cb;
		data->ch[channel_id].user_data = user_data;
	}
	return 0;
}

static int caliptra_mci_mbox_mtu_get(const struct device *dev)
{
	const struct caliptra_mci_mbox_config *cfg = dev->config;

	return (int)cfg->sram_size;
}

static uint32_t caliptra_mci_mbox_max_channels_get(const struct device *dev)
{
	ARG_UNUSED(dev);
	return CALIPTRA_MCI_MBOX_CHANNELS;
}

static int caliptra_mci_mbox_set_enabled(const struct device *dev,
				     mbox_channel_id_t channel_id, bool enabled)
{
	struct caliptra_mci_mbox_data *data = dev->data;

	if (channel_id >= CALIPTRA_MCI_MBOX_CHANNELS) {
		return -EINVAL;
	}

	K_SPINLOCK(&data->lock) {
		data->ch[channel_id].enabled = enabled;
	}
	/* The timer owns unmasking the IRQ; see caliptra_mci_mbox_timer_fn(). */
	caliptra_mci_mbox_kick(dev);
	return 0;
}

static const struct mbox_driver_api caliptra_mci_mbox_api = {
	.send = caliptra_mci_mbox_send,
	.register_callback = caliptra_mci_mbox_register_callback,
	.mtu_get = caliptra_mci_mbox_mtu_get,
	.max_channels_get = caliptra_mci_mbox_max_channels_get,
	.set_enabled = caliptra_mci_mbox_set_enabled,
};

#ifdef CONFIG_MBOX_NPCM_CALIPTRA_MCI_POLL
static void caliptra_mci_mbox_poll_entry(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	const struct caliptra_mci_mbox_config *cfg = dev->config;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		caliptra_mci_mbox_dispatch(dev, CALIPTRA_MCI_MBOX_CH_REQUESTER);
		caliptra_mci_mbox_dispatch(dev, CALIPTRA_MCI_MBOX_CH_RESPONDER);
		k_msleep(cfg->poll_interval_ms);
	}
}

#define CALIPTRA_MCI_MBOX_NOTIFY_DEFINE(inst)                                            \
	static K_THREAD_STACK_DEFINE(caliptra_mci_mbox_stack_##inst,                     \
		CONFIG_MBOX_NPCM_CALIPTRA_MCI_POLL_THREAD_STACK_SIZE);                   \
	static struct k_thread caliptra_mci_mbox_thread_##inst;

#define CALIPTRA_MCI_MBOX_NOTIFY_START(inst)                                             \
	k_thread_create(&caliptra_mci_mbox_thread_##inst, caliptra_mci_mbox_stack_##inst,    \
			CONFIG_MBOX_NPCM_CALIPTRA_MCI_POLL_THREAD_STACK_SIZE,            \
			caliptra_mci_mbox_poll_entry, (void *)dev, NULL, NULL,           \
			CONFIG_MBOX_NPCM_CALIPTRA_MCI_POLL_THREAD_PRIO, 0, K_NO_WAIT);   \
	k_thread_name_set(&caliptra_mci_mbox_thread_##inst, "mbox" #inst "_poll");
#elif defined(CONFIG_MBOX_NPCM_CALIPTRA_MCI_IRQ)
static bool caliptra_mci_mbox_waiting(const struct caliptra_mci_mbox_channel *ch)
{
	/* BUSY is requester-only and CLOSING responder-only, so one test
	 * covers both channels. */
	return ch->enabled && ch->cb != NULL &&
	       (ch->state == CALIPTRA_MCI_MBOX_STATE_BUSY ||
		ch->state == CALIPTRA_MCI_MBOX_STATE_CLOSING);
}

static void caliptra_mci_mbox_timer_fn(struct k_timer *timer)
{
	const struct device *dev = k_timer_user_data_get(timer);
	const struct caliptra_mci_mbox_config *cfg = dev->config;
	struct caliptra_mci_mbox_data *data = dev->data;
	struct caliptra_mci_mbox_channel *rsp = &data->ch[CALIPTRA_MCI_MBOX_CH_RESPONDER];
	bool pending = false;

	caliptra_mci_mbox_dispatch(dev, CALIPTRA_MCI_MBOX_CH_REQUESTER);
	caliptra_mci_mbox_dispatch(dev, CALIPTRA_MCI_MBOX_CH_RESPONDER);

	K_SPINLOCK(&data->lock) {
		for (int i = 0; i < CALIPTRA_MCI_MBOX_CHANNELS; i++) {
			pending |= caliptra_mci_mbox_waiting(&data->ch[i]);
		}
		/* Unmask only once the last request has closed. If a new one
		 * is already pending the level line re-enters the ISR at once. */
		if (rsp->enabled && rsp->state == CALIPTRA_MCI_MBOX_STATE_IDLE) {
			irq_enable(cfg->irq);
		}
	}
	if (pending) {
		caliptra_mci_mbox_kick(dev);
	}
}

static void caliptra_mci_mbox_isr(const struct device *dev)
{
	const struct caliptra_mci_mbox_config *cfg = dev->config;

	/* data_avail is level and stays high until the MCU clears EXECUTE,
	 * which it does only after our TARGET_STATUS.DONE. Mask it here; the
	 * timer unmasks once the transaction has closed. */
	irq_disable(cfg->irq);
	caliptra_mci_mbox_dispatch(dev, CALIPTRA_MCI_MBOX_CH_RESPONDER);
}

#define CALIPTRA_MCI_MBOX_NOTIFY_DEFINE(inst)
/* IRQ stays masked until a consumer enables the responder channel: that is
 * the first point the M7 may touch mailbox CSRs (MCI AXI USER gating). */
#define CALIPTRA_MCI_MBOX_NOTIFY_START(inst)                                             \
	k_timer_init(&data->timer, caliptra_mci_mbox_timer_fn, NULL);                    \
	k_timer_user_data_set(&data->timer, (void *)dev);                                \
	IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority),                     \
		    caliptra_mci_mbox_isr, DEVICE_DT_INST_GET(inst), 0);
#define CALIPTRA_MCI_MBOX_IRQ_CFG(inst) .irq = DT_INST_IRQN(inst),
#endif /* CONFIG_MBOX_NPCM_CALIPTRA_MCI_POLL */

#ifndef CALIPTRA_MCI_MBOX_IRQ_CFG
#define CALIPTRA_MCI_MBOX_IRQ_CFG(inst)
#endif

#define CALIPTRA_MCI_MBOX_INIT(inst)                                                     \
	static struct caliptra_mci_mbox_data caliptra_mci_mbox_data_##inst;                  \
	static const struct caliptra_mci_mbox_config caliptra_mci_mbox_cfg_##inst = {        \
		.sram_base = DT_INST_REG_ADDR_BY_NAME(inst, sram),                   \
		.csr_base = DT_INST_REG_ADDR_BY_NAME(inst, csr),                     \
		.sram_size = DT_INST_PROP(inst, sram_size),                          \
		.poll_interval_ms = DT_INST_PROP(inst, poll_interval_ms),            \
		CALIPTRA_MCI_MBOX_IRQ_CFG(inst)                                      \
	};                                                                           \
	CALIPTRA_MCI_MBOX_NOTIFY_DEFINE(inst)                                            \
	static int caliptra_mci_mbox_init_##inst(const struct device *dev)               \
	{                                                                            \
		struct caliptra_mci_mbox_data *data = dev->data;                         \
		/* MCI bus-id setup. Without this the M7's USER does not match  \
		 * the MCU-provisioned slot and mailbox writes are filtered.    \
		 * Idempotent across instances - last write wins, same value. */\
		sys_write32(CONFIG_MBOX_NPCM_CALIPTRA_MCI_BUSID,                         \
			    CALIPTRA_MCI_BUSID_CTL0_ADDR);                           \
		for (int i = 0; i < CALIPTRA_MCI_MBOX_CHANNELS; i++) {                   \
			data->ch[i].state = CALIPTRA_MCI_MBOX_STATE_IDLE;                \
		}                                                                    \
		CALIPTRA_MCI_MBOX_NOTIFY_START(inst)                                     \
		return 0;                                                            \
	}                                                                            \
	DEVICE_DT_INST_DEFINE(inst, caliptra_mci_mbox_init_##inst, NULL,                 \
			      &caliptra_mci_mbox_data_##inst,                            \
			      &caliptra_mci_mbox_cfg_##inst, POST_KERNEL,                \
			      CONFIG_MBOX_INIT_PRIORITY, &caliptra_mci_mbox_api);

DT_INST_FOREACH_STATUS_OKAY(CALIPTRA_MCI_MBOX_INIT)
