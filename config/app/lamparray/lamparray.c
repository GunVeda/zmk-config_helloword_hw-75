/*
 * Copyright (c) 2022-2023 XiNGRZ
 * SPDX-License-Identifier: MIT
 */

#include "lamparray.h"
#include "lamparray_reports.h"

#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/logging/log.h>

#include <zmk/workqueue.h>
#include <zmk/rgb_underglow.h>

#ifdef CONFIG_HW75_INDICATOR
#include <app/indicator.h>
#endif

LOG_MODULE_REGISTER(lamparray, CONFIG_HW75_LAMPARRAY_LOG_LEVEL);

#define STRIP_CHOSEN DT_CHOSEN(zmk_underglow)

static const struct device *led_strip;
static atomic_t slave_mode = ATOMIC_INIT(0);

static struct led_rgb slave_buffer[HW75_LAMPARRAY_LAMP_COUNT];
static struct k_spinlock buffer_lock;

static void mode_apply_fn(struct k_work *w);
static void render_fn(struct k_work *w);

K_WORK_DEFINE(mode_apply_work, mode_apply_fn);
K_WORK_DEFINE(render_work, render_fn);

bool lamparray_is_slave(void)
{
	return atomic_get(&slave_mode) != 0;
}

void lamparray_enter_slave(void)
{
	atomic_set(&slave_mode, 1);
	k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &mode_apply_work);
}

void lamparray_enter_autonomous(void)
{
	atomic_set(&slave_mode, 0);
	k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &mode_apply_work);
}

static void mode_apply_fn(struct k_work *w)
{
	if (atomic_get(&slave_mode)) {
		LOG_INF("lamparray: entering slave mode (host takes control)");
		zmk_rgb_underglow_off();
#ifdef CONFIG_HW75_INDICATOR
		indicator_set_enable(false);
#endif
	} else {
		LOG_INF("lamparray: entering autonomous mode (firmware in control)");
#ifdef CONFIG_HW75_INDICATOR
		indicator_set_enable(true);
#endif
		zmk_rgb_underglow_on();
	}
}

static void render_fn(struct k_work *w)
{
	if (!atomic_get(&slave_mode)) {
		/* Mode was flipped back to autonomous while we were queued. */
		return;
	}

	struct led_rgb snapshot[HW75_LAMPARRAY_LAMP_COUNT];
	k_spinlock_key_t key = k_spin_lock(&buffer_lock);
	memcpy(snapshot, slave_buffer, sizeof(snapshot));
	k_spin_unlock(&buffer_lock, key);

	int ret = led_strip_update_rgb(led_strip, snapshot, HW75_LAMPARRAY_LAMP_COUNT);
	if (ret < 0) {
		LOG_WRN("lamparray: strip update failed: %d", ret);
	}
}

static void schedule_render(void)
{
	k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &render_work);
}

void lamparray_apply_multiupdate(const uint8_t *r, size_t len)
{
	/*
	 * Per Microsoft spec, MultiUpdate carries a fixed 8 lamps per
	 * frame. The host issues ⌈HW75_LAMPARRAY_LAMP_COUNT / 8⌉
	 * SET_REPORT control transfers; each transfer is self-contained.
	 */
	if (!r || len < sizeof(struct LampMultiUpdateReport)) {
		LOG_WRN("lamparray: multiupdate too short (%zu, need %zu)", len,
			sizeof(struct LampMultiUpdateReport));
		return;
	}

	const struct LampMultiUpdateReport *rep = (const struct LampMultiUpdateReport *)r;
	uint8_t count = rep->LampCount;
	uint8_t flags = rep->LampUpdateFlags;

	if (count == 0) {
		return;
	}
	if (count > HW75_LAMPARRAY_MULTI_UPDATE_LAMPS) {
		LOG_WRN("lamparray: multiupdate count %u > spec max %d, clamping", count,
			HW75_LAMPARRAY_MULTI_UPDATE_LAMPS);
		count = HW75_LAMPARRAY_MULTI_UPDATE_LAMPS;
	}

	k_spinlock_key_t key = k_spin_lock(&buffer_lock);
	for (uint8_t i = 0; i < count; i++) {
		uint16_t id = rep->LampId[i];
		if (id >= HW75_LAMPARRAY_LAMP_COUNT) {
			continue;
		}
		struct led_rgb *d = &slave_buffer[id];
		/* Channels are stored lamp-major: lamp0 R, G, B, I, lamp1 R, ... */
		const uint8_t *c = &rep->Channels[i * 4];
		if (flags & LAMPARRAY_UPDATE_FLAG_RED) {
			d->r = c[0];
		}
		if (flags & LAMPARRAY_UPDATE_FLAG_GREEN) {
			d->g = c[1];
		}
		if (flags & LAMPARRAY_UPDATE_FLAG_BLUE) {
			d->b = c[2];
		}
		/* Intensity is dropped — WS2812 has no separate intensity channel. */
	}
	k_spin_unlock(&buffer_lock, key);

	if (atomic_get(&slave_mode)) {
		schedule_render();
	}
}

void lamparray_apply_range_update(const uint8_t *r, size_t len)
{
	if (!r || len < sizeof(struct LampRangeUpdateReport)) {
		LOG_WRN("lamparray: range update too short (%zu)", len);
		return;
	}

	const struct LampRangeUpdateReport *hdr = (const struct LampRangeUpdateReport *)r;
	uint16_t start = hdr->LampIdStart;
	uint16_t end = hdr->LampIdEnd;
	uint8_t flags = hdr->LampUpdateFlags;

	if (start > end || end >= HW75_LAMPARRAY_LAMP_COUNT) {
		LOG_WRN("lamparray: range update bad range [%u,%u]", start, end);
		return;
	}

	k_spinlock_key_t key = k_spin_lock(&buffer_lock);
	for (uint16_t id = start; id <= end; id++) {
		struct led_rgb *d = &slave_buffer[id];
		if (flags & LAMPARRAY_UPDATE_FLAG_RED) {
			d->r = hdr->RedUpdateChannel;
		}
		if (flags & LAMPARRAY_UPDATE_FLAG_GREEN) {
			d->g = hdr->GreenUpdateChannel;
		}
		if (flags & LAMPARRAY_UPDATE_FLAG_BLUE) {
			d->b = hdr->BlueUpdateChannel;
		}
	}
	k_spin_unlock(&buffer_lock, key);

	if (atomic_get(&slave_mode)) {
		schedule_render();
	}
}

static int lamparray_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	led_strip = DEVICE_DT_GET(STRIP_CHOSEN);
	if (!device_is_ready(led_strip)) {
		LOG_ERR("lamparray: chosen underglow strip not ready");
		return -ENODEV;
	}

#ifdef CONFIG_HW75_LAMPARRAY_AUTONOMOUS_ON_BOOT
	atomic_set(&slave_mode, 0);
#else
	atomic_set(&slave_mode, 1);
#endif

	LOG_INF("lamparray: initialized, %d lamps, mode=%s", HW75_LAMPARRAY_LAMP_COUNT,
		lamparray_is_slave() ? "slave" : "autonomous");
	return 0;
}

SYS_INIT(lamparray_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);