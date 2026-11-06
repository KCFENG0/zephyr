/*
 * USB OCP OBMF class driver
 *
 * Copyright (c) 2025 Google LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/usb_device.h>
#include <usb_descriptor.h>
#include <zephyr/usb/class/usb_obmf.h>

#define LOG_LEVEL CONFIG_USB_DEVICE_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usb_obmf);

#define OBMF_INTERFACE_CLASS            0xEF /* Miscellaneous Class */
#define OBMF_INTERFACE_SUBCLASS         0x09 /* OCP OBMF-ICP SubClass */
#define OBMF_INTERFACE_PROTOCOL         0x01 /* OBMF over USB binding version 1 */

#define OCP_OBMF_FUNCTIONAL_DESC_TYPE   0x24
#define OCP_OBMF_FUNCTIONAL_DESC_SUBTYPE 0x01

/* Calculate number of endpoints based on config */
#define NUM_BULK_ENDPOINTS 2
#if CONFIG_OBMF_INTERRUPT_IN_EP_SUPPORT && CONFIG_OBMF_INTERRUPT_OUT_EP_SUPPORT
#define NUM_INT_ENDPOINTS 2
#elif CONFIG_OBMF_INTERRUPT_IN_EP_SUPPORT || CONFIG_OBMF_INTERRUPT_OUT_EP_SUPPORT
#define NUM_INT_ENDPOINTS 1
#else
#define NUM_INT_ENDPOINTS 0
#endif
#define NUM_ENDPOINTS (NUM_BULK_ENDPOINTS + NUM_INT_ENDPOINTS)

static uint8_t obmf_buf[CONFIG_OBMF_BULK_EP_MPS];

struct obmf_device_info {
	const struct obmf_ops *ops;
	struct usb_dev_data common;
};

struct ocp_obmf_functional_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bMultimessageSupport;
	uint16_t wMaxWrTransferSize;
	uint16_t wMaxRdTransferSize;
	uint16_t wMaxWrInterruptSize;
	uint16_t wMaxRdInterruptSize;
	uint16_t bcdOCPOBMFVersion;
} __packed;

struct usb_obmf_config {
	struct usb_if_descriptor if0;
	struct ocp_obmf_functional_descriptor if0_obmf_func;
	struct usb_ep_descriptor if0_in_ep;
	struct usb_ep_descriptor if0_out_ep;
#if CONFIG_OBMF_INTERRUPT_IN_EP_SUPPORT
	struct usb_ep_descriptor if0_int_in_ep;
#endif
#if CONFIG_OBMF_INTERRUPT_OUT_EP_SUPPORT
	struct usb_ep_descriptor if0_int_out_ep;
#endif
} __packed;

/* String descriptor for iInterface: "OCP OBMF" */
#define OBMF_IFACE_STR "OCP OBMF"

struct usb_obmf_iface_str_descr {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bString[USB_BSTRING_LENGTH(OBMF_IFACE_STR)];
} __packed;

USBD_STRING_DESCR_DEFINE(primary)
struct usb_obmf_iface_str_descr obmf_iface_str = {
	.bLength = USB_STRING_DESCRIPTOR_LENGTH(OBMF_IFACE_STR),
	.bDescriptorType = USB_DESC_STRING,
	.bString = OBMF_IFACE_STR,
};

#define INITIALIZER_IF \
	{ \
		.bLength = sizeof(struct usb_if_descriptor), \
		.bDescriptorType = USB_DESC_INTERFACE, \
		.bInterfaceNumber = 0, \
		.bAlternateSetting = 0, \
		.bNumEndpoints = NUM_ENDPOINTS, \
		.bInterfaceClass = OBMF_INTERFACE_CLASS, \
		.bInterfaceSubClass = OBMF_INTERFACE_SUBCLASS, \
		.bInterfaceProtocol = OBMF_INTERFACE_PROTOCOL, \
		.iInterface = 0, /* Set at runtime via obmf_interface_config() */ \
	}

/*
 * [1.0rc5] bMultimessageSupport must be 0x00.
 * Multi-message per USB transfer is not permitted.
 */
#define OBMF_MULTIMESSAGE_VAL 0x00

#if CONFIG_OBMF_INTERRUPT_IN_EP_SUPPORT
#define OBMF_INT_IN_EP_SIZE CONFIG_OBMF_INTERRUPT_IN_EP_MPS
#else
#define OBMF_INT_IN_EP_SIZE 0
#endif

#if CONFIG_OBMF_INTERRUPT_OUT_EP_SUPPORT
#define OBMF_INT_OUT_EP_SIZE CONFIG_OBMF_INTERRUPT_OUT_EP_MPS
#else
#define OBMF_INT_OUT_EP_SIZE 0
#endif

#define INITIALIZER_OBMF_FUNC_DESC(wr_size, rd_size, wr_int_size, rd_int_size) \
	{ \
		.bLength = sizeof(struct ocp_obmf_functional_descriptor), \
		.bDescriptorType = OCP_OBMF_FUNCTIONAL_DESC_TYPE, \
		.bDescriptorSubtype = OCP_OBMF_FUNCTIONAL_DESC_SUBTYPE, \
		.bMultimessageSupport = OBMF_MULTIMESSAGE_VAL, \
		.wMaxWrTransferSize = sys_cpu_to_le16(wr_size), \
		.wMaxRdTransferSize = sys_cpu_to_le16(rd_size), \
		.wMaxWrInterruptSize = sys_cpu_to_le16(wr_int_size), \
		.wMaxRdInterruptSize = sys_cpu_to_le16(rd_int_size), \
		.bcdOCPOBMFVersion = sys_cpu_to_le16(0x0100), /* v1.0.0 rc1 */ \
	}

#define INITIALIZER_EP_DESC(addr, attr, mps, interval) \
	{ \
		.bLength = sizeof(struct usb_ep_descriptor), \
		.bDescriptorType = USB_DESC_ENDPOINT, \
		.bEndpointAddress = addr, \
		.bmAttributes = attr, \
		.wMaxPacketSize = sys_cpu_to_le16(mps), \
		.bInterval = interval, \
	}

#if CONFIG_OBMF_INTERRUPT_IN_EP_SUPPORT && CONFIG_OBMF_INTERRUPT_OUT_EP_SUPPORT
/* 4 endpoints: Bulk IN + Bulk OUT + Interrupt IN + Interrupt OUT */
#define DEFINE_OBMF_DESCR(x, _) \
	USBD_CLASS_DESCR_DEFINE(primary, x) \
	struct usb_obmf_config obmf_cfg_##x = { \
		.if0 = INITIALIZER_IF, \
		.if0_obmf_func = INITIALIZER_OBMF_FUNC_DESC(CONFIG_OBMF_BULK_EP_MPS, CONFIG_OBMF_BULK_EP_MPS, OBMF_INT_OUT_EP_SIZE, OBMF_INT_IN_EP_SIZE), \
		.if0_in_ep = INITIALIZER_EP_DESC(AUTO_EP_IN, USB_DC_EP_BULK, CONFIG_OBMF_BULK_EP_MPS, 0), \
		.if0_out_ep = INITIALIZER_EP_DESC(AUTO_EP_OUT, USB_DC_EP_BULK, CONFIG_OBMF_BULK_EP_MPS, 0), \
		.if0_int_in_ep = INITIALIZER_EP_DESC(AUTO_EP_IN, USB_DC_EP_INTERRUPT, CONFIG_OBMF_INTERRUPT_IN_EP_MPS, 0x0A), \
		.if0_int_out_ep = INITIALIZER_EP_DESC(AUTO_EP_OUT, USB_DC_EP_INTERRUPT, CONFIG_OBMF_INTERRUPT_OUT_EP_MPS, 0x0A), \
	};
#elif CONFIG_OBMF_INTERRUPT_IN_EP_SUPPORT
/* 3 endpoints: Bulk IN + Bulk OUT + Interrupt IN */
#define DEFINE_OBMF_DESCR(x, _) \
	USBD_CLASS_DESCR_DEFINE(primary, x) \
	struct usb_obmf_config obmf_cfg_##x = { \
		.if0 = INITIALIZER_IF, \
		.if0_obmf_func = INITIALIZER_OBMF_FUNC_DESC(CONFIG_OBMF_BULK_EP_MPS, CONFIG_OBMF_BULK_EP_MPS, OBMF_INT_OUT_EP_SIZE, OBMF_INT_IN_EP_SIZE), \
		.if0_in_ep = INITIALIZER_EP_DESC(AUTO_EP_IN, USB_DC_EP_BULK, CONFIG_OBMF_BULK_EP_MPS, 0), \
		.if0_out_ep = INITIALIZER_EP_DESC(AUTO_EP_OUT, USB_DC_EP_BULK, CONFIG_OBMF_BULK_EP_MPS, 0), \
		.if0_int_in_ep = INITIALIZER_EP_DESC(AUTO_EP_IN, USB_DC_EP_INTERRUPT, CONFIG_OBMF_INTERRUPT_IN_EP_MPS, 0x0A), \
	};
#elif CONFIG_OBMF_INTERRUPT_OUT_EP_SUPPORT
/* 3 endpoints: Bulk IN + Bulk OUT + Interrupt OUT */
#define DEFINE_OBMF_DESCR(x, _) \
	USBD_CLASS_DESCR_DEFINE(primary, x) \
	struct usb_obmf_config obmf_cfg_##x = { \
		.if0 = INITIALIZER_IF, \
		.if0_obmf_func = INITIALIZER_OBMF_FUNC_DESC(CONFIG_OBMF_BULK_EP_MPS, CONFIG_OBMF_BULK_EP_MPS, OBMF_INT_OUT_EP_SIZE, OBMF_INT_IN_EP_SIZE), \
		.if0_in_ep = INITIALIZER_EP_DESC(AUTO_EP_IN, USB_DC_EP_BULK, CONFIG_OBMF_BULK_EP_MPS, 0), \
		.if0_out_ep = INITIALIZER_EP_DESC(AUTO_EP_OUT, USB_DC_EP_BULK, CONFIG_OBMF_BULK_EP_MPS, 0), \
		.if0_int_out_ep = INITIALIZER_EP_DESC(AUTO_EP_OUT, USB_DC_EP_INTERRUPT, CONFIG_OBMF_INTERRUPT_OUT_EP_MPS, 0x0A), \
	};
#else
/* 2 endpoints: Bulk IN + Bulk OUT only */
#define DEFINE_OBMF_DESCR(x, _) \
	USBD_CLASS_DESCR_DEFINE(primary, x) \
	struct usb_obmf_config obmf_cfg_##x = { \
		.if0 = INITIALIZER_IF, \
		.if0_obmf_func = INITIALIZER_OBMF_FUNC_DESC(CONFIG_OBMF_BULK_EP_MPS, CONFIG_OBMF_BULK_EP_MPS, 0, 0), \
		.if0_in_ep = INITIALIZER_EP_DESC(AUTO_EP_IN, USB_DC_EP_BULK, CONFIG_OBMF_BULK_EP_MPS, 0), \
		.if0_out_ep = INITIALIZER_EP_DESC(AUTO_EP_OUT, USB_DC_EP_BULK, CONFIG_OBMF_BULK_EP_MPS, 0), \
	};
#endif

static sys_slist_t usb_obmf_devlist;

void usb_obmf_register_device(const struct device *dev, const struct obmf_ops *ops)
{
	struct obmf_device_info *dev_data = dev->data;

	dev_data->ops = ops;
	dev_data->common.dev = dev;

	sys_slist_append(&usb_obmf_devlist, &dev_data->common.node);

	LOG_DBG("Added dev_data %p dev %p to devlist %p", dev_data, dev,
		&usb_obmf_devlist);
}

static void obmf_out_cb(uint8_t ep, enum usb_dc_ep_cb_status_code ep_status)
{
	struct obmf_device_info *dev_data;
	struct usb_dev_data *common;
	uint32_t bytes_to_read;

	common = usb_get_dev_data_by_ep(&usb_obmf_devlist, ep);
	if (common == NULL) {
		LOG_WRN("Device data not found for endpoint %u", ep);
		return;
	}

	dev_data = CONTAINER_OF(common, struct obmf_device_info, common);

	if (ep_status != USB_DC_EP_DATA_OUT || dev_data->ops == NULL ||
	    dev_data->ops->read == NULL) {
		return;
	}

	usb_read(ep, NULL, 0, &bytes_to_read);
	LOG_DBG("ep 0x%x, bytes to read %d ", ep, bytes_to_read);

	usb_read(ep, obmf_buf, bytes_to_read, NULL);

	dev_data->ops->read(common->dev, bytes_to_read, obmf_buf);
}

int obmf_usb_ep_write(const struct device *dev, const uint8_t *data,
		      uint32_t data_len, uint32_t *bytes_ret)
{
	const struct usb_cfg_data *cfg = dev->config;
	int ret;

	/* transfer data to host */
	ret = usb_transfer_sync(cfg->endpoint[0].ep_addr,
				(uint8_t *)data, data_len, USB_TRANS_WRITE);
	if (ret != data_len) {
		LOG_ERR("Transfer failure");
		return -EINVAL;
	}

	*bytes_ret = ret;

	return 0;
}

#if CONFIG_OBMF_INTERRUPT_IN_EP_SUPPORT
int obmf_usb_int_ep_write(const struct device *dev, const uint8_t *data,
			  uint32_t data_len, uint32_t *bytes_ret)
{
	const struct usb_cfg_data *cfg = dev->config;
	int ret;

	/* [TR-INT-01] Interrupt message must fit in a single transfer */
	if (data_len > CONFIG_OBMF_INTERRUPT_IN_EP_MPS) {
		LOG_ERR("Interrupt IN data (%u) exceeds max transfer size (%u)",
			data_len, CONFIG_OBMF_INTERRUPT_IN_EP_MPS);
		return -EINVAL;
	}

	/* Interrupt IN endpoint index: after Bulk IN(0) and Bulk OUT(1) */
	ret = usb_transfer_sync(cfg->endpoint[2].ep_addr,
				(uint8_t *)data, data_len, USB_TRANS_WRITE);
	if (ret != data_len) {
		LOG_ERR("Interrupt IN transfer failure");
		return -EINVAL;
	}

	*bytes_ret = ret;

	return 0;
}
#endif

#if CONFIG_OBMF_INTERRUPT_OUT_EP_SUPPORT
static void obmf_int_out_cb(uint8_t ep, enum usb_dc_ep_cb_status_code ep_status)
{
	struct obmf_device_info *dev_data;
	struct usb_dev_data *common;
	uint32_t bytes_to_read;

	common = usb_get_dev_data_by_ep(&usb_obmf_devlist, ep);
	if (common == NULL) {
		LOG_WRN("Device data not found for endpoint %u", ep);
		return;
	}

	dev_data = CONTAINER_OF(common, struct obmf_device_info, common);

	if (ep_status != USB_DC_EP_DATA_OUT || dev_data->ops == NULL ||
	    dev_data->ops->int_read == NULL) {
		return;
	}

	usb_read(ep, NULL, 0, &bytes_to_read);
	LOG_DBG("int out ep 0x%x, bytes to read %d ", ep, bytes_to_read);

	usb_read(ep, obmf_buf, bytes_to_read, NULL);

	dev_data->ops->int_read(common->dev, bytes_to_read, obmf_buf);
}
#endif

#define INITIALIZER_EP_DATA(cb, addr) \
	{ \
		.ep_cb = cb, \
		.ep_addr = addr, \
	}

#if CONFIG_OBMF_INTERRUPT_IN_EP_SUPPORT && CONFIG_OBMF_INTERRUPT_OUT_EP_SUPPORT
/* 4 endpoints: Bulk IN, Bulk OUT, Interrupt IN, Interrupt OUT */
#define DEFINE_OBMF_EP(x, _) \
	static struct usb_ep_cfg_data obmf_ep_data_##x[] = { \
		INITIALIZER_EP_DATA(usb_transfer_ep_callback, AUTO_EP_IN), \
		INITIALIZER_EP_DATA(obmf_out_cb, AUTO_EP_OUT), \
		INITIALIZER_EP_DATA(usb_transfer_ep_callback, AUTO_EP_IN), \
		INITIALIZER_EP_DATA(obmf_int_out_cb, AUTO_EP_OUT), \
	};
#elif CONFIG_OBMF_INTERRUPT_IN_EP_SUPPORT
/* 3 endpoints: Bulk IN, Bulk OUT, Interrupt IN */
#define DEFINE_OBMF_EP(x, _) \
	static struct usb_ep_cfg_data obmf_ep_data_##x[] = { \
		INITIALIZER_EP_DATA(usb_transfer_ep_callback, AUTO_EP_IN), \
		INITIALIZER_EP_DATA(obmf_out_cb, AUTO_EP_OUT), \
		INITIALIZER_EP_DATA(usb_transfer_ep_callback, AUTO_EP_IN), \
	};
#elif CONFIG_OBMF_INTERRUPT_OUT_EP_SUPPORT
/* 3 endpoints: Bulk IN, Bulk OUT, Interrupt OUT */
#define DEFINE_OBMF_EP(x, _) \
	static struct usb_ep_cfg_data obmf_ep_data_##x[] = { \
		INITIALIZER_EP_DATA(usb_transfer_ep_callback, AUTO_EP_IN), \
		INITIALIZER_EP_DATA(obmf_out_cb, AUTO_EP_OUT), \
		INITIALIZER_EP_DATA(obmf_int_out_cb, AUTO_EP_OUT), \
	};
#else
/* 2 endpoints: Bulk IN, Bulk OUT */
#define DEFINE_OBMF_EP(x, _) \
	static struct usb_ep_cfg_data obmf_ep_data_##x[] = { \
		INITIALIZER_EP_DATA(usb_transfer_ep_callback, AUTO_EP_IN), \
		INITIALIZER_EP_DATA(obmf_out_cb, AUTO_EP_OUT), \
	};
#endif

static void obmf_status_cb(struct usb_cfg_data *cfg,
			   enum usb_dc_status_code status,
			   const uint8_t *param)
{
	struct obmf_device_info *dev_data = NULL;
	struct usb_dev_data *common;

	ARG_UNUSED(param);

	common = usb_get_dev_data_by_cfg(&usb_obmf_devlist, cfg);
	if (common != NULL) {
		dev_data = CONTAINER_OF(common, struct obmf_device_info, common);
	}

	switch (status) {
	case USB_DC_RESET:
		/*
		 * [1.0rc5 C7] USB bus reset detected.
		 * Clear USB-layer RX buffer to ensure clean state.
		 * Notify the application so it can reset upper-layer
		 * OBMF-ICP channel state (tags, fragment reassembly, etc.),
		 * e.g. after a host reboot.
		 */
		LOG_WRN("USB reset: clearing OBMF USB-layer RX buffer");
		memset(obmf_buf, 0, sizeof(obmf_buf));
		if (dev_data && dev_data->ops && dev_data->ops->bus_reset) {
			dev_data->ops->bus_reset(common->dev);
		}
		break;
	case USB_DC_DISCONNECTED:
		/*
		 * Host connection lost. Notify the application to reset
		 * upper-layer OBMF-ICP channel state.
		 */
		LOG_WRN("USB disconnected: clearing OBMF USB-layer RX buffer");
		memset(obmf_buf, 0, sizeof(obmf_buf));
		if (dev_data && dev_data->ops && dev_data->ops->bus_reset) {
			dev_data->ops->bus_reset(common->dev);
		}
		break;
	case USB_DC_CLEAR_HALT:
		/*
		 * [1.0rc5 C6] CLEAR_FEATURE(ENDPOINT_HALT) received.
		 * Clear USB-layer RX buffer so the next transfer starts
		 * cleanly. Upper-layer state cleanup is the application's
		 * responsibility.
		 */
		LOG_WRN("Endpoint halt cleared: clearing OBMF USB-layer RX buffer");
		memset(obmf_buf, 0, sizeof(obmf_buf));
		break;
	case USB_DC_SET_HALT:
		LOG_WRN("Endpoint halt set");
		break;
	case USB_DC_CONFIGURED:
		LOG_DBG("USB device configured");
		break;
	case USB_DC_SUSPEND:
		/*
		 * No SOF from the host. With no VBUS pin this is the only
		 * early indication that the host is gone, so let the
		 * application take its channels down. Upper-layer tag state
		 * is left alone: a resume may follow without a bus reset.
		 */
		LOG_WRN("USB suspended: OBMF link down");
		if (dev_data && dev_data->ops && dev_data->ops->link_down) {
			dev_data->ops->link_down(common->dev);
		}
		break;
	case USB_DC_RESUME:
		/* Undo the suspend-time link down; no bus reset is involved here */
		LOG_WRN("USB resumed: OBMF link up");
		if (dev_data && dev_data->ops && dev_data->ops->link_up) {
			dev_data->ops->link_up(common->dev);
		}
		break;
	case USB_DC_ERROR:
		LOG_ERR("USB device error");
		break;
	default:
		LOG_DBG("status_cb status %d", status);
		break;
	}
}

static void obmf_interface_config(struct usb_desc_header *head,
				  uint8_t bInterfaceNumber)
{
	struct usb_if_descriptor *if_desc = (struct usb_if_descriptor *)head;
	struct usb_obmf_config *desc =
		CONTAINER_OF(if_desc, struct usb_obmf_config, if0);
	int idx;

	desc->if0.bInterfaceNumber = bInterfaceNumber;

	/* [DESC-INTF-02] Set iInterface string descriptor index */
	idx = usb_get_str_descriptor_idx(&obmf_iface_str);
	if (idx) {
		desc->if0.iInterface = idx;
	}
}

#define DEFINE_OBMF_CFG_DATA(x, _) \
	USBD_DEFINE_CFG_DATA(obmf_config_##x) = { \
		.usb_device_description = NULL, \
		.interface_config = obmf_interface_config, \
		.interface_descriptor = &obmf_cfg_##x.if0, \
		.cb_usb_status = obmf_status_cb, \
		.interface = { \
			.class_handler = NULL, \
			.custom_handler = NULL, \
		}, \
		.num_endpoints = ARRAY_SIZE(obmf_ep_data_##x), \
		.endpoint = obmf_ep_data_##x, \
	};

static int usb_obmf_device_init(const struct device *dev)
{
	LOG_INF("Init OBMF USB Device: dev %p (%s)", dev, dev->name);
	return 0;
}

#define DEFINE_OBMF_DEV_DATA(x, _) \
	struct obmf_device_info usb_obmf_dev_data_##x;

#define DEFINE_OBMF_DEVICE(x, _) \
	DEVICE_DEFINE(usb_obmf_device_##x, CONFIG_USB_OBMF_DEVICE_NAME "_" #x, \
			    &usb_obmf_device_init, NULL, &usb_obmf_dev_data_##x, \
			    &obmf_config_##x, POST_KERNEL, \
			    CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, NULL);

LISTIFY(CONFIG_USB_OBMF_DEVICE_COUNT, DEFINE_OBMF_DESCR, (;), _);
LISTIFY(CONFIG_USB_OBMF_DEVICE_COUNT, DEFINE_OBMF_EP, (;), _);
LISTIFY(CONFIG_USB_OBMF_DEVICE_COUNT, DEFINE_OBMF_CFG_DATA, (;), _);
LISTIFY(CONFIG_USB_OBMF_DEVICE_COUNT, DEFINE_OBMF_DEV_DATA, (;), _);
LISTIFY(CONFIG_USB_OBMF_DEVICE_COUNT, DEFINE_OBMF_DEVICE, (;), _);
