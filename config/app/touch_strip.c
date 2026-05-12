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
#define FP_HALF 128

/* Horizontal scroll directions (signed byte) */
#define HWHEEL_RIGHT 1
#define HWHEEL_LEFT (-1)

#ifndef ABS
#define ABS(x) ((x) < 0 ? -(x) : (x))
#endif

enum ts_state { TS_IDLE, TS_TOUCHING, TS_MOMENTUM };

struct ts_data {
	enum ts_state state;
	uint8_t sensor_mask;

	/* Position tracking (8.8 fixed-point, range 0 to 5*256) */
	int16_t position;
	int16_t prev_position;

	/* Velocity (8.8 fixed-point, sensor-units per second) */
	int32_t velocity;
	int64_t last_time;

	/* Momentum */
	int32_t momentum_vel;
	int16_t scroll_accum;

	/* Tap detection */
	int64_t touch_start;
	uint8_t start_sensor;
	bool moved;

	struct k_work_delayable momentum_work;
};

static struct ts_data ts;

static int16_t compute_position(uint8_t mask)
{
	if (mask == 0) {
		return -1;
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

static void emit_scroll(int16_t delta)
{
	int16_t accum = ts.scroll_accum + delta;

	while (accum >= FP_ONE) {
		hid_mouse_hwheel_report(HWHEEL_RIGHT, true);
		hid_mouse_hwheel_report(HWHEEL_RIGHT, false);
		accum -= FP_ONE;
	}

	while (accum <= -FP_ONE) {
		hid_mouse_hwheel_report(HWHEEL_LEFT, true);
		hid_mouse_hwheel_report(HWHEEL_LEFT, false);
		accum += FP_ONE;
	}

	ts.scroll_accum = accum;
}

static void handle_tap(uint8_t sensor)
{
	LOG_DBG("Tap at sensor %d", sensor);

	if (sensor <= 1) {
		/* Left area: volume down */
		hid_mouse_hwheel_report(HWHEEL_LEFT, true);
		hid_mouse_hwheel_report(HWHEEL_LEFT, false);
	} else if (sensor >= 4) {
		/* Right area: volume up */
		hid_mouse_hwheel_report(HWHEEL_RIGHT, true);
		hid_mouse_hwheel_report(HWHEEL_RIGHT, false);
	}
	/* Middle area: no action for now */
}

static void momentum_tick(struct k_work *work)
{
	ARG_UNUSED(work);

	if (ts.state != TS_MOMENTUM) {
		return;
	}

	ts.momentum_vel = ts.momentum_vel * CONFIG_HW75_TOUCH_STRIP_MOMENTUM_DECAY / 100;

	if (ts.momentum_vel > -FP_ONE && ts.momentum_vel < FP_ONE) {
		ts.state = TS_IDLE;
		LOG_DBG("Momentum stopped");
		return;
	}

	int16_t scroll_delta =
		(int16_t)(ts.momentum_vel * CONFIG_HW75_TOUCH_STRIP_MOMENTUM_TICK_MS / 1000);
	emit_scroll(scroll_delta);

	k_work_reschedule_for_queue(zmk_workqueue_lowprio_work_q(), &ts.momentum_work,
				    K_MSEC(CONFIG_HW75_TOUCH_STRIP_MOMENTUM_TICK_MS));
}

static K_WORK_DELAYABLE_DEFINE(momentum_work, momentum_tick);

void touch_strip_press(uint8_t sensor)
{
	if (sensor >= SENSOR_COUNT) {
		return;
	}

	int64_t now = k_uptime_get();

	ts.sensor_mask |= BIT(sensor);

	if (ts.state == TS_IDLE) {
		ts.state = TS_TOUCHING;
		ts.touch_start = now;
		ts.start_sensor = sensor;
		ts.moved = false;
		ts.scroll_accum = 0;
		ts.velocity = 0;
		ts.position = sensor * FP_ONE;
		ts.prev_position = ts.position;
		ts.last_time = now;

		LOG_DBG("Touch start at sensor %d", sensor);
	} else if (ts.state == TS_TOUCHING) {
		int16_t new_pos = compute_position(ts.sensor_mask);
		int16_t delta = new_pos - ts.position;
		int64_t dt = now - ts.last_time;

		if (dt > 0) {
			int32_t raw_vel = (int32_t)delta * 1000 / (int32_t)dt;
			ts.velocity = ts.velocity * 7 / 10 + raw_vel * 3 / 10;
		}

		ts.prev_position = ts.position;
		ts.position = new_pos;
		ts.last_time = now;

		if (ABS(new_pos - (int16_t)(ts.start_sensor * FP_ONE)) > FP_ONE) {
			ts.moved = true;
		}

		emit_scroll(delta);
		LOG_DBG("Sensor %d pressed, pos=%d, delta=%d, vel=%d", sensor, new_pos, delta,
			ts.velocity);
	}
}

void touch_strip_release(uint8_t sensor)
{
	if (sensor >= SENSOR_COUNT) {
		return;
	}

	int64_t now = k_uptime_get();

	ts.sensor_mask &= ~BIT(sensor);

	if (ts.state != TS_TOUCHING) {
		return;
	}

	if (ts.sensor_mask == 0) {
		/* All sensors released */
		int64_t duration = now - ts.touch_start;

		if (duration < CONFIG_HW75_TOUCH_STRIP_TAP_MS && !ts.moved) {
			handle_tap(ts.start_sensor);
			ts.state = TS_IDLE;
		}
#ifdef CONFIG_HW75_TOUCH_STRIP_MOMENTUM
		else if (ABS(ts.velocity) > FP_ONE) {
			ts.momentum_vel = ts.velocity;
			ts.state = TS_MOMENTUM;
			LOG_DBG("Momentum start, vel=%d", ts.momentum_vel);
			k_work_reschedule_for_queue(zmk_workqueue_lowprio_work_q(),
						    &ts.momentum_work,
						    K_MSEC(CONFIG_HW75_TOUCH_STRIP_MOMENTUM_TICK_MS));
		}
#endif
		else {
			ts.state = TS_IDLE;
		}

		LOG_DBG("Touch end, duration=%lld, moved=%d", duration, ts.moved);
	} else {
		/* Some sensors still active, update position */
		int16_t new_pos = compute_position(ts.sensor_mask);
		int16_t delta = new_pos - ts.position;
		int64_t dt = now - ts.last_time;

		if (dt > 0) {
			int32_t raw_vel = (int32_t)delta * 1000 / (int32_t)dt;
			ts.velocity = ts.velocity * 7 / 10 + raw_vel * 3 / 10;
		}

		ts.prev_position = ts.position;
		ts.position = new_pos;
		ts.last_time = now;

		emit_scroll(delta);
	}
}

static int touch_strip_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	ts.state = TS_IDLE;
	ts.sensor_mask = 0;
	ts.scroll_accum = 0;

	ts.momentum_work = momentum_work;

	return 0;
}

SYS_INIT(touch_strip_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
