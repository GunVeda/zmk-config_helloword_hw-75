# Copyright (c) 2022-2023 XiNGRZ
# SPDX-License-Identifier: MIT

zephyr_library()
zephyr_library_link_libraries(hw75_proto)

zephyr_library_sources(lamparray.c lamparray_hid.c)

zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR})
zephyr_library_include_directories(${APPLICATION_SOURCE_DIR}/include)

# Per-board static configuration (LampCount, positions, kind, bounding box).
# Force-include the per-board header so the descriptor and reports see the
# macros without an extra #include to maintain. -include is a compiler
# flag, not a -D definition, so it has to go through compile_options.
if(BOARD STREQUAL "hw75_keyboard")
	zephyr_library_compile_options(
		-include "${CMAKE_CURRENT_SOURCE_DIR}/lamparray_config_keyboard.h"
	)
elseif(BOARD STREQUAL "hw75_dynamic")
	zephyr_library_compile_options(
		-include "${CMAKE_CURRENT_SOURCE_DIR}/lamparray_config_dynamic.h"
	)
else()
	message(FATAL_ERROR "HW-75 LampArray: unknown board ${BOARD}, need a per-board config header")
endif()

add_subdirectory(handler)