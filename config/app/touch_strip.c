/*
 * Copyright (c) 2022-2023 XiNGRZ
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/workqueue.h>

#include <app/touch_strip.h>
#include <app/hid_mouse.h>

#define SENSOR_COUNT 6

/* 8.8 fixed-point: 1.0 = 256 */
#define FP_ONE 256

/* Horizontal scroll directions (signed byte) */
#define HWHEEL_RIGHT 1
#define HWHEEL_LEFT (-1)

#ifndef ABS
#define ABS(x) ((x) < 0 ? -(x) : (x))
#endif

enum ts_state {
	TS_IDLE,
	TS_TOUCHING,
	TS_MOMENTUM,
};

static enum ts_state ts_state;
static uint8_t ts_sensor_mask;

/* Position tracking (8.8 fixed-point, range 0 to 5*256) */
static int16_t ts_position;
static int32_t ts_velocity;
static int64_t ts_last_time;

/* Scroll accumulator (8.8 fixed-point) */
static int16_t ts_scroll_accum;

/* Tap detection */
static int64_t ts_touch_start;
static uint8_t ts_start_sensor;
static bool ts_moved;

/* Momentum */
static int32_t ts_momentum_vel;

static void emit_scroll(int16_t delta)
{
	int16_t accum = ts_scroll_accum + delta;

	while (accum >= FP_ONE) {
		hid_mouse_hwheel_report(HWHEEL_RIGHT, true);
		hid_mouse_hwheel_report(HWHEEL_RIGHT, false);
		accum -= FP_ONE;
		LOG_INF("hscroll right");
	}

	while (accum <= -FP_ONE) {
		hid_mouse_hwheel_report(HWHEEL_LEFT, true);
		hid_mouse_hwheel_report(HWHEEL_LEFT, false);
		accum += FP_ONE;
		LOG_INF("hscroll left");
	}

	ts_scroll_accum = accum;
}

static void handle_tap(uint8_t sensor)
{
	LOG_INF("Tap at sensor %d", sensor);

	if (sensor <= 1) {
		hid_mouse_hwheel_report(HWHEEL_LEFT, true);
		hid_mouse_hwheel_report(HWHEEL_LEFT, false);
	} else if (sensor >= 4) {
		hid_mouse_hwheel_report(HWHEEL_RIGHT, true);
		hid_mouse_hwheel_report(HWHEEL_RIGHT, false);
	}
}

static void momentum_tick_work(struct k_work *work);

static K_WORK_DELAYABLE_DEFINE(momentum_work, momentum_tick_work);

static void momentum_tick_work(struct k_work *work)
{
	ARG_UNUSED(work);

	if (ts_state != TS_MOMENTUM) {
		return;
	}

	ts_momentum_vel = ts_momentum_vel * CONFIG_HW75_TOUCH_STRIP_MOMENTUM_DECAY / 100;

	if (ts_momentum_vel > -FP_ONE && ts_momentum_vel < FP_ONE) {
		ts_state = TS_IDLE;
		LOG_INF("Momentum stopped");
		return;
	}

	int16_t scroll_delta =
		(int16_t)(ts_momentum_vel * CONFIG_HW75_TOUCH_STRIP_MOMENTUM_TICK_MS / 1000);
	emit_scroll(scroll_delta);

	k_work_reschedule_for_queue(zmk_workqueue_lowprio_work_q(), &momentum_work,
				    K_MSEC(CONFIG_HW75_TOUCH_STRIP_MOMENTUM_TICK_MS));
}

static int16_t compute_position(uint8_t mask)
{
	if (mask == 0) {
		return 0;
	}

	int32_t weighted_sum = 0;
	int count = 0;

	for (int i = 0; i < SENSOR_COUNT; i++) {
		if (mask & BIT(i)) {
			weighted_sum += i * FP_ONE;
			count++;
		}
	}

	return (int16_t)(weighted_sum / count);
}

void touch_strip_press(uint8_t sensor)
{
	if (sensor >= SENSOR_COUNT) {
		return;
	}

	int64_t now = k_uptime_get();

	ts_sensor_mask |= BIT(sensor);

	if (ts_state == TS_IDLE) {
		ts_state = TS_TOUCHING;
		ts_touch_start = now;
		ts_start_sensor = sensor;
		ts_moved = false;
		ts_velocity = 0;
		ts_position = sensor * FP_ONE;
		ts_last_time = now;

		LOG_INF("Touch start at sensor %d, pos=%d", sensor, ts_position);
	} else if (ts_state == TS_TOUCHING) {
		int16_t new_pos = compute_position(ts_sensor_mask);
		int16_t delta = new_pos - ts_position;
		int64_t dt = now - ts_last_time;

		if (dt > 0) {
			int32_t raw_vel = (int32_t)delta * 1000 / (int32_t)dt;
			ts_velocity = ts_velocity * 7 / 10 + raw_vel * 3 / 10;
		}

		ts_position = new_pos;
		ts_last_time = now;

		if (ABS(new_pos - (int16_t)(ts_start_sensor * FP_ONE)) > FP_ONE) {
			ts_moved = true;
		}

		emit_scroll(delta);
		LOG_INF("Press sensor %d, pos=%d, delta=%d, vel=%d", sensor, new_pos, delta,
			ts_velocity);
	}
}

void touch_strip_release(uint8_t sensor)
{
	if (sensor >= SENSOR_COUNT) {
		return;
	}

	int64_t now = k_uptime_get();

	ts_sensor_mask &= ~BIT(sensor);

	if (ts_state != TS_TOUCHING) {
		return;
	}

	if (ts_sensor_mask == 0) {
		int64_t duration = now - ts_touch_start;

		if (duration < CONFIG_HW75_TOUCH_STRIP_TAP_MS && !ts_moved) {
			handle_tap(ts_start_sensor);
			ts_state = TS_IDLE;
		}
#ifdef CONFIG_HW75_TOUCH_STRIP_MOMENTUM
		else if (ABS(ts_velocity) > FP_ONE) {
			ts_momentum_vel = ts_velocity;
			ts_state = TS_MOMENTUM;
			LOG_INF("Momentum start, vel=%d", ts_momentum_vel);
			k_work_reschedule_for_queue(zmk_workqueue_lowprio_work_q(), &momentum_work,
						    K_MSEC(CONFIG_HW75_TOUCH_STRIP_MOMENTUM_TICK_MS));
		}
#endif
		else {
			ts_state = TS_IDLE;
		}

		LOG_INF("Touch end, duration=%lld, moved=%d, state=%d", duration, ts_moved,
			ts_state);
	} else {
		int16_t new_pos = compute_position(ts_sensor_mask);
		int16_t delta = new_pos - ts_position;
		int64_t dt = now - ts_last_time;

		if (dt > 0) {
			int32_t raw_vel = (int32_t)delta * 1000 / (int32_t)dt;
			ts_velocity = ts_velocity * 7 / 10 + raw_vel * 3 / 10;
		}

		ts_position = new_pos;
		ts_last_time = now;

		emit_scroll(delta);
		LOG_INF("Release sensor %d, pos=%d, delta=%d", sensor, new_pos, delta);
	}
}

static int touch_strip_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	ts_state = TS_IDLE;
	ts_sensor_mask = 0;
	ts_scroll_accum = 0;

	LOG_INF("Touch strip initialized");

	return 0;
}

SYS_INIT(touch_strip_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
