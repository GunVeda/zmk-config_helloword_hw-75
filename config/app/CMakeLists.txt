# Copyright (c) 2022-2023 XiNGRZ
# SPDX-License-Identifier: MIT

zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# ZMK 4.x: app's own CMakeLists only adds its `include/` directory to the
# `app` target (target_include_directories(... PRIVATE include)), so the
# public <zmk/...> headers aren't on the global include path. Add them
# explicitly for our config sources.
zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)

zephyr_library_sources_ifdef(CONFIG_SETTINGS storage_init.c)
zephyr_library_sources_ifdef(CONFIG_HW75_HID_MOUSE hid_mouse.c)
zephyr_library_sources_ifdef(CONFIG_HW75_INDICATOR indicator.c)

zephyr_library_sources_ifdef(CONFIG_LVGL behaviors/behavior_lvgl_key_press.c)
zephyr_library_sources(behaviors/behavior_mouse_wheel.c)

add_subdirectory_ifdef(CONFIG_HW75_USB_COMM usb_comm)

add_subdirectory_ifdef(CONFIG_HW75_LAMPARRAY lamparray)

zephyr_library_sources_ifdef(CONFIG_HW75_TOUCH_STRIP touch_strip.c)
zephyr_library_sources_ifdef(CONFIG_HW75_TOUCH_STRIP behaviors/behavior_touch_strip.c)

zephyr_library_include_directories(${ZEPHYR_LVGL_MODULE_DIR})
zephyr_library_include_directories(${ZEPHYR_BASE}/lib/gui/lvgl)
