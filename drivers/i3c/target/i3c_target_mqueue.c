/*
 * Copyright (c) 2023 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT i3c_target_mqueue

#include <zephyr/sys/util.h>
#include <zephyr/sys/crc.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/i3c.h>

#include <zephyr/logging/log.h>
#define LOG_MODULE_NAME			i3c_target_mqueue
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_I3C_LOG_LEVEL);

struct i3c_target_mqueue_config {
	const struct device *controller;
	int msg_size;
	int num_of_msgs;
	int mdb;
};

struct mq_msg {
	int size;
	uint8_t *buf;
};

struct i3c_target_mqueue_data {
	struct i3c_target_config target_config;
	const struct i3c_target_mqueue_config *config;
	struct mq_msg *msg_curr;
	struct mq_msg *msg_queue;
	int in;
	int out;
	int wr_index;
	struct k_sem msg_avail_sem; /* Semaphore signaled when message is available */
};

int i3c_target_mqueue_write(const struct device *dev, uint8_t *buf, int len)
{
	const struct i3c_target_mqueue_config *config = dev->config;
	struct i3c_target_mqueue_data *data = dev->data;
	struct i3c_ibi ibi;
	uint8_t ibi_payload[2] = {config->mdb, 0};
	uint8_t addr_rnw, pec_v;

	if (data->target_config.address == 0) {
		/* the dynamic address of the I3C controller is not been assigned */
		return -ENOTCONN;
	}

	addr_rnw = (data->target_config.address << 1) | 0x1;
	pec_v = crc8_ccitt(0, &addr_rnw, 1);
	pec_v = crc8_ccitt(pec_v, ibi_payload, 1);
	ibi_payload[1] = pec_v;

	ibi.ibi_type = I3C_IBI_TARGET_INTR;
	ibi.payload = ibi_payload;
	ibi.payload_len = 2;
	return i3c_target_pending_read_notify(config->controller, buf, len, &ibi);
}

int i3c_target_mqueue_read(const struct device *dev, uint8_t *buf, int budget)
{
	struct i3c_target_mqueue_data *data = dev->data;
	struct mq_msg *msg;
	int ret;

	if (data->out == data->in) {
		return 0;
	}

	msg = &data->msg_queue[data->out];
	ret = (msg->size > budget) ? budget : msg->size;
	// LOG_DBG("%s: out = %d, msg size = %d", __func__, data->out, ret);

	memcpy(buf, msg->buf, ret);
	data->out = (data->out + 1) & (data->config->num_of_msgs - 1);

	return ret;
}

/**
 * @brief Blocking read - waits for a message to be available
 *
 * @param dev The SMQ device
 * @param buf Buffer to store the message
 * @param budget Maximum bytes to read
 * @param timeout Timeout for waiting (K_FOREVER for indefinite)
 * @return Number of bytes read, 0 if timeout, negative on error
 */
int i3c_target_mqueue_read_blocking(const struct device *dev, uint8_t *buf, int budget,
				    k_timeout_t timeout)
{
	struct i3c_target_mqueue_data *data = dev->data;
	int ret;

	/*
	 * Wait for notification that data is available.
	 * Each k_sem_give() from stop_cb corresponds to one message,
	 * so we always take the semaphore to keep count synchronized.
	 */
	ret = k_sem_take(&data->msg_avail_sem, timeout);
	if (ret != 0) {
		/* Timeout or error - check if data arrived anyway */
		if (data->out != data->in) {
			return i3c_target_mqueue_read(dev, buf, budget);
		}
		return 0;
	}

	/* Read the message */
	return i3c_target_mqueue_read(dev, buf, budget);
}

static int i3c_target_mq_write_requested_cb(struct i3c_target_config *config)
{
	struct i3c_target_mqueue_data *data =
		CONTAINER_OF(config, struct i3c_target_mqueue_data, target_config);

	data->msg_curr = &data->msg_queue[data->in];
	data->msg_curr->size = 0;

	data->in = (data->in + 1) & (data->config->num_of_msgs - 1);

	/* if queue full, drop the oldest un-read message */
	if (data->in == data->out) {
		LOG_WRN("Queue full! Drop the oldest message: in=%d, out=%d", data->in, data->out);
		data->out = (data->out + 1) & (data->config->num_of_msgs - 1);
	}

	return 0;
}

static int i3c_target_mq_write_received_cb(struct i3c_target_config *config, uint8_t val)
{
	struct i3c_target_mqueue_data *data =
		CONTAINER_OF(config, struct i3c_target_mqueue_data, target_config);

	data->msg_curr->buf[data->msg_curr->size++] = val;

	return 0;
}

#ifdef CONFIG_I3C_TARGET_BUFFER_MODE
/**
 * @brief Bulk write received callback - receives entire buffer at once
 *
 * This is much more efficient than byte-by-byte delivery for DMA-based
 * transfers, as it allows a single memcpy instead of per-byte function calls.
 */
static void i3c_target_mq_buf_write_received_cb(struct i3c_target_config *config,
						uint8_t *ptr, uint32_t len)
{
	struct i3c_target_mqueue_data *data =
		CONTAINER_OF(config, struct i3c_target_mqueue_data, target_config);
	uint32_t max_len = data->config->msg_size - data->msg_curr->size;

	if (len > max_len) {
		LOG_WRN("buf_write_received: truncating %u to %u bytes", len, max_len);
		len = max_len;
	}

	memcpy(&data->msg_curr->buf[data->msg_curr->size], ptr, len);
	data->msg_curr->size += len;
}
#endif

static int i3c_target_mq_read_requested_cb(struct i3c_target_config *config, uint8_t *val)
{
	return 0;
}

static int i3c_target_mq_read_processed_cb(struct i3c_target_config *config, uint8_t *val)
{
	return 0;
}

static int i3c_target_mq_stop_cb(struct i3c_target_config *config)
{
	struct i3c_target_mqueue_data *data =
		CONTAINER_OF(config, struct i3c_target_mqueue_data, target_config);

	/* Signal that a message is available for reading */
	k_sem_give(&data->msg_avail_sem);

	return 0;
}

static const struct i3c_target_callbacks i3c_target_mq_callbacks = {
	.write_requested_cb = i3c_target_mq_write_requested_cb,
	.write_received_cb = i3c_target_mq_write_received_cb,
	.read_requested_cb = i3c_target_mq_read_requested_cb,
	.read_processed_cb = i3c_target_mq_read_processed_cb,
	.stop_cb = i3c_target_mq_stop_cb,
#ifdef CONFIG_I3C_TARGET_BUFFER_MODE
	.buf_write_received_cb = i3c_target_mq_buf_write_received_cb,
#endif
};

static int i3c_target_mqueue_register(const struct device *dev)
{
	const struct i3c_target_mqueue_config *config = dev->config;
	struct i3c_target_mqueue_data *data = dev->data;
	uint8_t *buf;
	int i;

	buf = k_calloc(config->msg_size, config->num_of_msgs);
	if (!buf) {
		return -ENOBUFS;
	}

	data->msg_queue = (struct mq_msg *)k_malloc(sizeof(struct mq_msg) * config->num_of_msgs);
	if (!data->msg_queue) {
		return -ENOBUFS;
	}

	for (i = 0; i < config->num_of_msgs; i++) {
		data->msg_queue[i].buf = buf + (i * config->msg_size);
		data->msg_queue[i].size = 0;
	}

	data->out = 0;
	data->in = 0;
	data->msg_curr = &data->msg_queue[data->in];
	data->target_config.callbacks = &i3c_target_mq_callbacks;

	/* Initialize semaphore for message available notification */
	k_sem_init(&data->msg_avail_sem, 0, config->num_of_msgs);

	LOG_INF("register %s to I3C controller %s", dev->name, config->controller->name);

	return i3c_target_register(config->controller, &data->target_config);
}

static int i3c_target_mqueue_unregister(const struct device *dev)
{
	const struct i3c_target_mqueue_config *config = dev->config;
	struct i3c_target_mqueue_data *data = dev->data;

	LOG_INF("unregister %s from I3C controller %s", dev->name, config->controller->name);

	return i3c_target_unregister(config->controller, &data->target_config);
}

struct i3c_target_driver_api i3c_target_mqueue_api = {
	.driver_register = i3c_target_mqueue_register,
	.driver_unregister = i3c_target_mqueue_unregister,
};

static int i3c_target_mqueue_init(const struct device *dev)
{
	const struct i3c_target_mqueue_config *config = dev->config;
	struct i3c_target_mqueue_data *data = dev->data;

	data->config = config;

	LOG_INF("%s bind to controller %s", dev->name, config->controller->name);

	return 0;
}

#define I3C_TARGET_MQUEUE_INIT(n)                                                                  \
	static const struct i3c_target_mqueue_config i3c_target_mqueue_config_##n = {              \
		.controller = DEVICE_DT_GET(DT_INST_BUS(n)),                                       \
		.msg_size = DT_INST_PROP(n, msg_size),                                             \
		.num_of_msgs = DT_INST_PROP(n, num_of_msgs),                                       \
		.mdb = DT_INST_PROP(n, mandatory_data_byte),                                       \
	};                                                                                         \
                                                                                                   \
	static struct i3c_target_mqueue_data i3c_target_mqueue_data_##n;                           \
	DEVICE_DT_INST_DEFINE(n, i3c_target_mqueue_init, NULL, &i3c_target_mqueue_data_##n,        \
			      &i3c_target_mqueue_config_##n, POST_KERNEL,                          \
			      CONFIG_I3C_TARGET_INIT_PRIORITY, &i3c_target_mqueue_api);

DT_INST_FOREACH_STATUS_OKAY(I3C_TARGET_MQUEUE_INIT)
