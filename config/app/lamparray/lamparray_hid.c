/*
 * Copyright (c) 2022-2023 XiNGRZ
 * SPDX-License-Identifier: MIT
 */

#include <stdint.h>
#include <string.h>
#include <stddef.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/slist.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(lamparray, CONFIG_HW75_LAMPARRAY_LOG_LEVEL);

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>

#include "lamparray.h"
#include "lamparray_descriptor.h"
#include "lamparray_reports.h"

/*
 * HACK: directly append HID_2 to the HID class driver's lookup list.
 *
 * The pcapng (usb-new-2.pcapng) showed that usb_get_dev_data_by_iface()
 * returns NULL for bInterfaceNumber=2 at host enumeration time, even
 * though the user's lamparray_hid_init calls usb_hid_register_device().
 * MI_00 and MI_01 work on retry (their init functions land in the list
 * during the host's retry window), but MI_02 never does — so the
 * lookup stays NULL and hid_custom_handle_req returns -EINVAL,
 * which makes the device STALL GET_DESCRIPTOR HID_REPORT for MI_02
 * and Windows shows CM_PROB_FAILED_START.
 *
 * The function usb_hid_register_device() in
 * subsys/usb/device/class/hid/core.c is what normally populates
 * usb_hid_devlist. We can't force a different SYS_INIT priority
 * that races correctly (POST_KERNEL/1 bricked the device, APPLICATION
 * races the host's enumeration). So instead, reach in and append the
 * entry directly. The list is a `static sys_slist_t` inside the HID
 * class driver, but its symbol is exported by the linker.
 */
extern sys_slist_t usb_hid_devlist;

/*
 * Mirror of struct hid_device_info from
 * subsys/usb/device/class/hid/core.c, with the conditional fields
 * (SOF bookkeeping, boot protocol protocol byte) omitted because
 * neither CONFIG_USB_DEVICE_SOF nor CONFIG_USB_HID_BOOT_PROTOCOL
 * is set in this build (we can confirm this from the pcapng: SET_IDLE
 * STALLs on every interface, which is the !SOF behavior). The layout
 * up to and including `common` is the same.
 *
 * We only touch report_desc, report_size, ops, and `common` (node +
 * dev), which is the same set usb_hid_register_device() writes.
 */
struct fake_hid_device_info {
	const uint8_t *report_desc;
	size_t report_size;
	const struct hid_ops *ops;
	bool configured;
	bool suspended;
	uint8_t _pad[2];
	struct usb_dev_data common;
};

static const struct device *hid_dev;

/* Last LampId requested via AttributesRequest report (Report #2). */
static uint16_t requested_lamp_id;

/* Pre-built Attributes response payload (Report #1, 23 bytes). */
static const struct LampArrayAttributesReport attributes_report = {
	.ReportId = LAMPARRAY_REPORT_ATTRIBUTES,
	.LampCount = HW75_LAMPARRAY_LAMP_COUNT,
	.BoundingBoxWidthInMicrometers = HW75_LAMPARRAY_BB_W_MICRONS,
	.BoundingBoxHeightInMicrometers = HW75_LAMPARRAY_BB_H_MICRONS,
	.BoundingBoxDepthInMicrometers = HW75_LAMPARRAY_BB_D_MICRONS,
	.LampArrayKind = HW75_LAMPARRAY_LAMP_KIND,
	.MinUpdateIntervalInMicroseconds = HW75_LAMPARRAY_MIN_UPDATE_US,
};

/*
 * Per-lamp attribute responses are precomputed at startup because
 * position / purpose values are static. We store them as a flat table
 * indexed by LampId and emit one AttributesResponse on demand.
 */

#define _LAMP_X_FOR(i)   HW75_LAMPARRAY_X_FOR(i)
#define _LAMP_Y_FOR(i)   HW75_LAMPARRAY_Y_FOR(i)
#define _LAMP_PURP_FOR(i) HW75_LAMPARRAY_PURPOSE_FOR(i)

#define HW75_LAMPARRAY_DEFINE_ATTR_TABLE(i, _)                                            \
	static const struct LampAttributesResponseReport _attr_resp_##i = {             \
		.ReportId = LAMPARRAY_REPORT_ATTRIBUTES_RESPONSE,                       \
		.LampId = (uint16_t)i,                                                  \
		.PositionXInMicrometers = _LAMP_X_FOR(i),                               \
		.PositionYInMicrometers = _LAMP_Y_FOR(i),                               \
		.PositionZInMicrometers = HW75_LAMPARRAY_Z,                             \
		.LampPurposes = _LAMP_PURP_FOR(i),                                      \
		.UpdateLatencyInMicroseconds = HW75_LAMPARRAY_MIN_UPDATE_US,            \
		.RedLevelCount = 0xFF,                                                 \
		.GreenLevelCount = 0xFF,                                               \
		.BlueLevelCount = 0xFF,                                                \
		.IntensityLevelCount = 0xFF,                                           \
		.IsProgrammable = 0x01,                                                \
		.InputBinding = 0x00,                                                  \
	};

LISTIFY(HW75_LAMPARRAY_LAMP_COUNT, HW75_LAMPARRAY_DEFINE_ATTR_TABLE, (;))

/* Lookup table mapping LampId -> attributes response pointer. Built at
 * startup so we can answer GET_REPORT(LampArrayAttributesResponse) with
 * O(1) indirection without re-running LISTIFY at query time.
 */
static const struct LampAttributesResponseReport *const attr_table[] = {
#define HW75_LAMPARRAY_ATTR_PUT(i, _) &_attr_resp_##i,
	LISTIFY(HW75_LAMPARRAY_LAMP_COUNT, HW75_LAMPARRAY_ATTR_PUT, ())
#undef HW75_LAMPARRAY_ATTR_PUT
};

static const struct LampAttributesResponseReport *attr_response_for(uint16_t id)
{
	if (id >= HW75_LAMPARRAY_LAMP_COUNT) {
		return NULL;
	}
	return attr_table[id];
}

static int hid_lamparray_get_report_cb(const struct device *dev, struct usb_setup_packet *setup,
				       int32_t *len, uint8_t **data)
{
	uint8_t report_id = (uint8_t)(setup->wValue & 0xFF);

	switch (report_id) {
	case LAMPARRAY_REPORT_ATTRIBUTES: {
		size_t copy = MIN(*len, (int32_t)sizeof(attributes_report));
		memcpy(*data, &attributes_report, copy);
		*len = (int32_t)sizeof(attributes_report);
		return 0;
	}
	case LAMPARRAY_REPORT_ATTRIBUTES_RESPONSE: {
		const struct LampAttributesResponseReport *resp =
			attr_response_for(requested_lamp_id);
		if (!resp) {
			return -EINVAL;
		}
		size_t copy = MIN(*len, (int32_t)sizeof(*resp));
		memcpy(*data, resp, copy);
		*len = (int32_t)sizeof(*resp);
		return 0;
	}
	default:
		return -ENOTSUP;
	}
}

static int hid_lamparray_set_report_cb(const struct device *dev, struct usb_setup_packet *setup,
				       int32_t *len, uint8_t **data)
{
	uint8_t report_id = (uint8_t)(setup->wValue & 0xFF);

	switch (report_id) {
	case LAMPARRAY_REPORT_ATTRIBUTES_REQUEST: {
		if (*len < (int32_t)sizeof(struct LampAttributesRequestReport)) {
			return -EINVAL;
		}
		const struct LampAttributesRequestReport *req =
			(const struct LampAttributesRequestReport *)(*data);
		requested_lamp_id = req->LampId;
		return 0;
	}
	case LAMPARRAY_REPORT_MULTI_UPDATE: {
		lamparray_apply_multiupdate(*data, (size_t)*len);
		return 0;
	}
	case LAMPARRAY_REPORT_RANGE_UPDATE: {
		lamparray_apply_range_update(*data, (size_t)*len);
		return 0;
	}
	case LAMPARRAY_REPORT_CONTROL: {
		if (*len < (int32_t)sizeof(struct LampArrayControlReport)) {
			return -EINVAL;
		}
		const struct LampArrayControlReport *ctrl =
			(const struct LampArrayControlReport *)(*data);
		if (ctrl->AutonomousMode) {
			lamparray_enter_autonomous();
		} else {
			lamparray_enter_slave();
		}
		return 0;
	}
	default:
		LOG_WRN("lamparray: set_report unsupported id 0x%02x", report_id);
		return -ENOTSUP;
	}
}

static const struct hid_ops ops = {
	.get_report = hid_lamparray_get_report_cb,
	.set_report = hid_lamparray_set_report_cb,
};

static int lamparray_hid_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	hid_dev = device_get_binding(CONFIG_HW75_LAMPARRAY_DEVICE_NAME);
	if (hid_dev == NULL) {
		LOG_ERR("lamparray: cannot locate %s", CONFIG_HW75_LAMPARRAY_DEVICE_NAME);
		return -ENODEV;
	}

	/* Diagnose: report which HID instance we got, plus its runtime
	 * bInterfaceNumber as the Zephyr USB device stack sees it.
	 */
	const struct usb_cfg_data *cfg = hid_dev->config;
	const struct usb_if_descriptor *if_desc = cfg->interface_descriptor;
	LOG_INF("lamparray hid: dev_name=%s bInterfaceNumber=%u "
		"report_size=%u cfg=%p",
		hid_dev->name, if_desc->bInterfaceNumber,
		sizeof(hw75_lamparray_report_desc), cfg);

	/*
	 * Direct-list-append (HACK) — see the comment at the top of this
	 * file. Functionally equivalent to usb_hid_register_device():
	 * set the same fields and append to the same list, just without
	 * going through the public API. The struct layout is matched by
	 * struct fake_hid_device_info above. If the lookup for
	 * bInterfaceNumber=2 still fails after this, the issue is not
	 * the registration call but something else (the field offsets,
	 * a different list, or a different code path the host takes).
	 */
	struct fake_hid_device_info *dev_data = hid_dev->data;

	dev_data->report_desc = hw75_lamparray_report_desc;
	dev_data->report_size = sizeof(hw75_lamparray_report_desc);
	dev_data->ops = &ops;
	dev_data->common.dev = hid_dev;
	sys_slist_append(&usb_hid_devlist, &dev_data->common.node);

	return usb_hid_init(hid_dev);
}

/*
 * APPLICATION priority. Originally this was at
 * CONFIG_APPLICATION_INIT_PRIORITY, but a brief experiment at
 * POST_KERNEL/1 made the keyboard fail to enumerate at all —
 * POST_KERNEL/1 runs before HID class DEVICE_DEFINEs (which are
 * at CONFIG_KERNEL_INIT_PRIORITY_DEFAULT=50), so
 * device_get_binding("HID_2") returns NULL and the SYS_INIT
 * returns -ENODEV during kernel bring-up.
 *
 * The race we want to fix (MI_02 GET_DESCRIPTOR HID_REPORT
 * STALLing on the first try because usb_hid_devlist is empty
 * for bInterfaceNumber=2 when the host queries it) is real and
 * is documented in the previous commits' pcapng analyses.
 * Solving it requires a deeper change to the Zephyr USB stack
 * (or patching ZMK's usb_enable timing) — the application-level
 * workaround attempted here was reverted. For now the LampArray
 * interface enumerates as Code 10 on Windows until the upstream
 * fix lands.
 */
SYS_INIT(lamparray_hid_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);