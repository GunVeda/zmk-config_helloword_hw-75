# Copyright (c) 2022-2023 XiNGRZ
# SPDX-License-Identifier: MIT

# usb_comm handler for LampArray mode toggle.

zephyr_library_sources_ifdef(CONFIG_HW75_USB_COMM_FEATURE_LAMPARRAY handler_lamparray.c)
zephyr_library_include_directories_ifdef(
	CONFIG_HW75_USB_COMM_FEATURE_LAMPARRAY ${CMAKE_CURRENT_SOURCE_DIR}/..
)
zephyr_library_include_directories_ifdef(
	CONFIG_HW75_USB_COMM_FEATURE_LAMPARRAY ${CMAKE_CURRENT_SOURCE_DIR}/../../usb_comm/handler
)