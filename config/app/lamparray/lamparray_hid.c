/*
 * Copyright (c) 2022-2023 XiNGRZ
 * SPDX-License-Identifier: MIT
 */

#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(lamparray, CONFIG_HW75_LAMPARRAY_LOG_LEVEL);

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>

#include "lamparray.h"
#include "lamparray_descriptor.h"
#include "lamparray_reports.h"

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

	usb_hid_register_device(hid_dev, hw75_lamparray_report_desc,
				sizeof(hw75_lamparray_report_desc), &ops);

	return usb_hid_init(hid_dev);
}

SYS_INIT(lamparray_hid_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);