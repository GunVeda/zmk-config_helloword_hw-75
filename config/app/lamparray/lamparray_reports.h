/*
 * Copyright (c) 2022-2023 XiNGRZ
 * SPDX-License-Identifier: MIT
 *
 * HID LampArray report structures, aligned with the Microsoft-published
 * reference descriptor (microsoft/ArduinoHidForWindows LampArrayReportDescriptor.h).
 *
 * The MultiUpdate report (Report #4) is fixed at 8 lamps per frame, per
 * spec. To update more than 8 lamps, the host issues multiple
 * SET_REPORT control transfers; each call is independent.
 *
 * All structs are __attribute__((packed)) to match the over-the-wire byte
 * ordering. Field names use CamelCase to follow the USB convention.
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

/* LampUpdateFlags bitmask indicating which channels are being updated.
 * Per spec, the host sets the bits it is updating in this report.
 */
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

/* Number of lamps per MultiUpdate report, per Microsoft spec. The host
 * fires ⌈HW75_LAMPARRAY_LAMP_COUNT / 8⌉ control transfers to update all
 * lamps on the device; each transfer is self-contained.
 */
#define HW75_LAMPARRAY_MULTI_UPDATE_LAMPS 8

/* Report #1: LampArrayAttributesReport (host reads via GET_REPORT).
 * 1 + 2 + 4*5 = 23 bytes including ReportId, but only 22 feature bytes.
 */
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

/* Report #2: LampAttributesRequestReport (host writes via SET_REPORT). */
struct LampAttributesRequestReport
{
	uint8_t ReportId;
	uint16_t LampId;
} __attribute__((packed));

/* Report #3: LampAttributesResponseReport (host reads via GET_REPORT). */
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

/* Report #4: LampMultiUpdateReport (host writes via SET_REPORT).
 * Fixed 50 bytes: 1 + 1 + 1 + (8 × 2) + (8 × 4) = 50.
 * LampId[] holds 8 little-endian u16 ids; Channels[] holds
 * 8 × {Red, Green, Blue, Intensity} in that order (32 bytes).
 */
struct LampMultiUpdateReport
{
	uint8_t ReportId;
	uint8_t LampCount;
	uint8_t LampUpdateFlags;
	uint16_t LampId[HW75_LAMPARRAY_MULTI_UPDATE_LAMPS];
	uint8_t Channels[HW75_LAMPARRAY_MULTI_UPDATE_LAMPS * 4];
} __attribute__((packed));

/* Report #5: LampRangeUpdateReport (host writes via SET_REPORT). */
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

/* Report #6: LampArrayControlReport (host writes via SET_REPORT). */
struct LampArrayControlReport
{
	uint8_t ReportId;
	uint8_t AutonomousMode;
} __attribute__((packed));

#endif /* HW75_LAMPARRAY_REPORTS_H_ */
