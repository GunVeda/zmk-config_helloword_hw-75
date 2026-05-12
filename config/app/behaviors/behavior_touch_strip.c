/*
 * Copyright (c) 2022-2023 XiNGRZ
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_touch_strip

#include <zephyr/device.h>
#include <drivers/behavior.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/behavior.h>
#include <app/touch_strip.h>

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
				     struct zmk_behavior_binding_event event)
{
	LOG_INF("Touch strip press sensor %d", binding->param1);
	touch_strip_press(binding->param1);
	return 0;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
				      struct zmk_behavior_binding_event event)
{
	LOG_INF("Touch strip release sensor %d", binding->param1);
	touch_strip_release(binding->param1);
	return 0;
}

static const struct behavior_driver_api behavior_touch_strip_driver_api = {
	.binding_pressed = on_keymap_binding_pressed,
	.binding_released = on_keymap_binding_released,
};

static int behavior_touch_strip_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
};

DEVICE_DT_INST_DEFINE(0, behavior_touch_strip_init, NULL, NULL, NULL, APPLICATION,
		      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_touch_strip_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
