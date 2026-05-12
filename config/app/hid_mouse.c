/*
 * Copyright (c) 2022-2023 XiNGRZ
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>

#include <zmk/usb.h>

#include <app/hid_mouse.h>

/*
 * HID Mouse Report Descriptor with horizontal scroll (AC Pan) support.
 * Report format: { buttons(1), x(1), y(1), vwheel(1), hwheel(1) } = 5 bytes.
 */
static const uint8_t hid_mouse_report_desc[] = {
	/* Generic Desktop Mouse */
	0x05, 0x01,       /* Usage Page (Generic Desktop) */
	0x09, 0x02,       /* Usage (Mouse) */
	0xA1, 0x01,       /* Collection (Application) */
	0x09, 0x01,       /*   Usage (Pointer) */
	0xA1, 0x00,       /*   Collection (Physical) */
	/* Buttons (2 buttons, 6-bit padding) */
	0x05, 0x09,       /*     Usage Page (Button) */
	0x19, 0x01,       /*     Usage Minimum (1) */
	0x29, 0x02,       /*     Usage Maximum (2) */
	0x15, 0x00,       /*     Logical Minimum (0) */
	0x25, 0x01,       /*     Logical Maximum (1) */
	0x95, 0x02,       /*     Report Count (2) */
	0x75, 0x01,       /*     Report Size (1) */
	0x81, 0x02,       /*     Input (Data, Variable, Absolute) */
	0x95, 0x01,       /*     Report Count (1) */
	0x75, 0x06,       /*     Report Size (6) */
	0x81, 0x01,       /*     Input (Constant) */
	/* X, Y */
	0x05, 0x01,       /*     Usage Page (Generic Desktop) */
	0x09, 0x30,       /*     Usage (X) */
	0x09, 0x31,       /*     Usage (Y) */
	0x15, 0x81,       /*     Logical Minimum (-127) */
	0x25, 0x7F,       /*     Logical Maximum (127) */
	0x75, 0x08,       /*     Report Size (8) */
	0x95, 0x02,       /*     Report Count (2) */
	0x81, 0x06,       /*     Input (Data, Variable, Relative) */
	/* Vertical Wheel */
	0x09, 0x38,       /*     Usage (Wheel) */
	0x15, 0x81,       /*     Logical Minimum (-127) */
	0x25, 0x7F,       /*     Logical Maximum (127) */
	0x75, 0x08,       /*     Report Size (8) */
	0x95, 0x01,       /*     Report Count (1) */
	0x81, 0x06,       /*     Input (Data, Variable, Relative) */
	/* Horizontal Wheel (Consumer AC Pan) */
	0x05, 0x0C,       /*     Usage Page (Consumer) */
	0x0A, 0x38, 0x02, /*     Usage (AC Pan) */
	0x15, 0x81,       /*     Logical Minimum (-127) */
	0x25, 0x7F,       /*     Logical Maximum (127) */
	0x75, 0x08,       /*     Report Size (8) */
	0x95, 0x01,       /*     Report Count (1) */
	0x81, 0x06,       /*     Input (Data, Variable, Relative) */
	0xC0,             /*   End Collection */
	0xC0              /* End Collection */
};

static const struct device *hid_dev;

static K_SEM_DEFINE(hid_sem, 1, 1);

static void in_ready_cb(const struct device *dev)
{
	ARG_UNUSED(dev);
	k_sem_give(&hid_sem);
}

static const struct hid_ops ops = {
	.int_in_ready = in_ready_cb,
};

static int hid_mouse_send_report(const uint8_t *report, size_t len)
{
	switch (zmk_usb_get_status()) {
	case USB_DC_SUSPEND:
		return usb_wakeup_request();
	case USB_DC_ERROR:
	case USB_DC_RESET:
	case USB_DC_DISCONNECTED:
	case USB_DC_UNKNOWN:
		return -ENODEV;
	default:
		k_sem_take(&hid_sem, K_MSEC(30));
		int err = hid_int_ep_write(hid_dev, report, len, NULL);

		if (err) {
			k_sem_give(&hid_sem);
		}

		return err;
	}
}

int hid_mouse_wheel_report(int direction, bool pressed)
{
	uint8_t val = pressed ? (uint8_t)(direction & 0xFF) : 0x00;

	uint8_t report[] = { 0x00, 0x00, 0x00, val, 0x00 };
	hid_mouse_send_report(report, sizeof(report));

	return 0;
}

int hid_mouse_hwheel_report(int direction, bool pressed)
{
	uint8_t val = pressed ? (uint8_t)(direction & 0xFF) : 0x00;

	uint8_t report[] = { 0x00, 0x00, 0x00, 0x00, val };
	hid_mouse_send_report(report, sizeof(report));

	return 0;
}

int hid_mouse_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	hid_dev = device_get_binding(CONFIG_HW75_HID_MOUSE_DEVICE_NAME);
	if (hid_dev == NULL) {
		LOG_ERR("Unable to locate HID device");
		return -ENODEV;
	}

	usb_hid_register_device(hid_dev, hid_mouse_report_desc, sizeof(hid_mouse_report_desc),
				&ops);
	usb_hid_init(hid_dev);

	return 0;
}

SYS_INIT(hid_mouse_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
