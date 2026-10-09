/*
 * Copyright (c) 2022-2023 XiNGRZ
 * SPDX-License-Identifier: MIT
 */

#ifndef APP_LAMPARRAY_H_
#define APP_LAMPARRAY_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @brief Switch the LampArray device to "slave" mode.
 *
 * In slave mode the host (e.g. Windows 11 Dynamic Lighting) controls every
 * LED; the autonomous ZMK RGB underglow worker and (on hw75_dynamic) the
 * status indicator are paused so the host has exclusive control of the
 * strip.
 *
 * Mode application is performed on ZMK's low-priority work queue.
 */
void lamparray_enter_slave(void);

/**
 * @brief Switch the LampArray device back to "autonomous" mode.
 *
 * Firmware-driven effects (ZMK RGB underglow, status indicator) resume.
 */
void lamparray_enter_autonomous(void);

/**
 * @brief Return whether the LampArray device is currently in slave mode.
 */
bool lamparray_is_slave(void);

/**
 * @brief Parse and apply a LampMultiUpdateReport (Report #4) body.
 *
 * Called from the HID set_report callback. Updates the internal
 * slave_buffer and schedules a render work item if currently in
 * slave mode.
 *
 * @param report Pointer to the report body (ReportId byte first).
 * @param len    Total report length in bytes.
 */
void lamparray_apply_multiupdate(const uint8_t *report, size_t len);

/**
 * @brief Parse and apply a LampRangeUpdateReport (Report #5) body.
 *
 * Convenience helper that reuses lamparray_apply_multiupdate by
 * expanding the [LampIdStart, LampIdEnd] range into per-lamp entries.
 */
void lamparray_apply_range_update(const uint8_t *report, size_t len);

#endif /* APP_LAMPARRAY_H_ */