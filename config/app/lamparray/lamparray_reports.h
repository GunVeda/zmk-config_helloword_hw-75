/*
 * Copyright (c) 2022-2023 XiNGRZ
 * SPDX-License-Identifier: MIT
 *
 * HID LampArray report structures, ported from
 * https://github.com/taleroangel/zephyr-lamparray (MIT, Angel Talero).
 *
 * Naming uses CamelCase to match the USB convention ("when in Rome...").
 * All structs are __packed to match the over-the-wire byte ordering.
 */

#ifndef HW75_LAMPARRAY_REPORTS_H_
#define HW75_LAMPARRAY_REPORTS_H_

#include <stdint.h>

/* LampArrayKind enum values from the LampArray spec. */
enum LampArrayKind
{
	LAMPARRAY_KIND_KEYBOARD = 0x01,
	LAMPARRAY_KIND_MOUSE = 0x02,
	LAMPARRAY_KIND_GAME_CONTROLLER = 0x03,
	LAMPARRAY_KIND_PERIPHERAL = 0x04,
	LAMPARRAY_KIND_SCENE = 0x05,
	LAMPARRAY_KIND_NOTIFICATION = 0x06,
	LAMPARRAY_KIND_CHASSIS = 0x07,
	LAMPARRAY_KIND_WEARABLE = 0x08,
	LAMPARRAY_KIND_FURNITURE = 0x09,
	LAMPARRAY_KIND_ART = 0x0A,
};

/* LampArrayPurposes flag bits. May be ORed together. */
enum LampArrayPurposes
{
	LAMPARRAY_PURPOSE_CONTROL = 0x01,
	LAMPARRAY_PURPOSE_ACCENT = 0x02,
	LAMPARRAY_PURPOSE_BRANDING = 0x04,
	LAMPARRAY_PURPOSE_STATUS = 0x08,
	LAMPARRAY_PURPOSE_ILLUMINATION = 0x10,
	LAMPARRAY_PURPOSE_PRESENTATION = 0x20,
};

/* LampUpdateFlags bitmask indicating which channels are being updated. */
enum LampUpdateFlags
{
	LAMPARRAY_UPDATE_FLAG_RED = 0x01,
	LAMPARRAY_UPDATE_FLAG_GREEN = 0x02,
	LAMPARRAY_UPDATE_FLAG_BLUE = 0x04,
	LAMPARRAY_UPDATE_FLAG_INTENSITY = 0x08,
};

/* Report IDs. */
enum LampArrayReportId
{
	LAMPARRAY_REPORT_ATTRIBUTES = 0x01,
	LAMPARRAY_REPORT_ATTRIBUTES_REQUEST = 0x02,
	LAMPARRAY_REPORT_ATTRIBUTES_RESPONSE = 0x03,
	LAMPARRAY_REPORT_MULTI_UPDATE = 0x04,
	LAMPARRAY_REPORT_RANGE_UPDATE = 0x05,
	LAMPARRAY_REPORT_CONTROL = 0x06,
};

/* Report #1: LampArrayAttributesReport (host -> device, get only). */
struct LampArrayAttributesReport
{
	uint8_t ReportId;
	uint16_t LampCount;
	uint32_t BoundingBoxWidthInMicrometers;
	uint32_t BoundingBoxHeightInMicrometers;
	uint32_t BoundingBoxDepthInMicrometers;
	uint32_t LampArrayKind;
	uint32_t MinUpdateIntervalInMicroseconds;
} __attribute__((packed));

/* Report #2: LampAttributesRequestReport (host -> device). */
struct LampAttributesRequestReport
{
	uint8_t ReportId;
	uint16_t LampId;
} __attribute__((packed));

/* Report #3: LampAttributesResponseReport (device -> host, get only). */
struct LampAttributesResponseReport
{
	uint8_t ReportId;
	uint16_t LampId;
	uint32_t PositionXInMicrometers;
	uint32_t PositionYInMicrometers;
	uint32_t PositionZInMicrometers;
	uint32_t LampPurposes;
	uint32_t UpdateLatencyInMicroseconds;
	uint8_t RedLevelCount;
	uint8_t GreenLevelCount;
	uint8_t BlueLevelCount;
	uint8_t IntensityLevelCount;
	uint8_t IsProgrammable;
	uint8_t InputBinding;
} __attribute__((packed));

/* Report #4: LampMultiUpdateReport (host -> device).
 *
 * Variable-length body follows the header. See LampMultiUpdateLamp.
 */
struct LampMultiUpdateReportHeader
{
	uint8_t ReportId;
	uint8_t LampCount;
	uint8_t LampUpdateFlags;
} __attribute__((packed));

/* One lamp entry within a MultiUpdate. */
struct LampMultiUpdateLamp
{
	uint16_t LampId;
	uint8_t RedUpdateChannel;
	uint8_t GreenUpdateChannel;
	uint8_t BlueUpdateChannel;
	uint8_t IntensityUpdateChannel;
} __attribute__((packed));

/* Report #5: LampRangeUpdateReport (host -> device). */
struct LampRangeUpdateReport
{
	uint8_t ReportId;
	uint8_t LampUpdateFlags;
	uint16_t LampIdStart;
	uint16_t LampIdEnd;
	uint8_t RedUpdateChannel;
	uint8_t GreenUpdateChannel;
	uint8_t BlueUpdateChannel;
	uint8_t IntensityUpdateChannel;
} __attribute__((packed));

/* Report #6: LampArrayControlReport (host -> device). */
struct LampArrayControlReport
{
	uint8_t ReportId;
	uint8_t AutonomousMode;
} __attribute__((packed));

#endif /* HW75_LAMPARRAY_REPORTS_H_ */