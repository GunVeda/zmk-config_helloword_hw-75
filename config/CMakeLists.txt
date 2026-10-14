# Copyright (c) 2022-2023 XiNGRZ
# SPDX-License-Identifier: MIT

add_subdirectory(app)
add_subdirectory(drivers)
add_subdirectory(proto)

# Per-board application sources. The pre-HWMv2 layout had a CMakeLists.txt
# at the board root that did `add_subdirectory_ifdef(CONFIG_BOARD_HW75_X app)`,
# but ZMK now warns that 'config/boards' is deprecated and a top-level
# module CMakeLists.txt is the supported home for any extra sources. Pull
# the per-board app/ subdirectory in here, guarded on BOARD so it only
# applies when the matching keyboard is being built.
if(BOARD STREQUAL "hw75_dynamic" OR BOARD MATCHES "^hw75_dynamic@")
    add_subdirectory(boards/helloword/hw75_dynamic/app)
endif()

if(BOARD STREQUAL "hw75_keyboard" OR BOARD MATCHES "^hw75_keyboard@")
    add_subdirectory(boards/helloword/hw75_keyboard/app)
endif()
