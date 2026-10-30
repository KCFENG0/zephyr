/*
 * Copyright (c) 2025 Google LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_USB_CLASS_USB_OBMF_H_
#define ZEPHYR_INCLUDE_USB_CLASS_USB_OBMF_H_

#include <stdint.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

struct obmf_ops {
	void (*read)(const struct device *dev, uint32_t len, uint8_t *data);
	/*
	 * Optional. Bytes the application can still accept in read(). When a
	 * Bulk OUT packet does not fit, it is left in the endpoint FIFO (the
	 * endpoint keeps NAKing) until obmf_usb_rx_resume() is called.
	 */
	uint32_t (*rx_space)(const struct device *dev);
#if CONFIG_OBMF_INTERRUPT_OUT_EP_SUPPORT
	void (*int_read)(const struct device *dev, uint32_t len, uint8_t *data);
#endif
	/*
	 * Called when the USB connection to the host is lost (USB bus reset
	 * or disconnect), e.g. on a host reboot. Runs in USB interrupt
	 * context, so the implementation must be ISR-safe (no blocking,
	 * no dynamic memory operations).
	 */
	void (*bus_reset)(const struct device *dev);

	/*
	 * Called when the USB link goes idle (suspend), which on a port
	 * without a VBUS pin is the earliest indication that the host is
	 * gone. Runs in USB interrupt context and must be ISR-safe.
	 */
	void (*link_down)(const struct device *dev);

	/*
	 * Called when the USB link comes back (resume). Runs in USB interrupt
	 * context and must be ISR-safe.
	 */
	void (*link_up)(const struct device *dev);
};

void usb_obmf_register_device(const struct device *dev,
			      const struct obmf_ops *ops);

/* Call after rx_space() grew, to receive a Bulk OUT packet left pending */
void obmf_usb_rx_resume(const struct device *dev);

int obmf_usb_ep_write(const struct device *dev, const uint8_t *data,
		      uint32_t data_len, uint32_t *bytes_ret);

#if CONFIG_OBMF_INTERRUPT_IN_EP_SUPPORT
int obmf_usb_int_ep_write(const struct device *dev, const uint8_t *data,
			  uint32_t data_len, uint32_t *bytes_ret);
#endif

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_USB_CLASS_USB_OBMF_H_ */
