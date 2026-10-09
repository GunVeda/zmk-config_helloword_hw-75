# Copyright (c) 2022-2023 XiNGRZ
# SPDX-License-Identifier: MIT

# Per-board static configuration (LampCount, positions, kind, bounding box).
# Force-include the per-board header so the descriptor and reports see the
# macros without an extra #include to maintain. -include is a compiler
# flag, not a -D definition, so it has to go through compile_options.
#
# NOTE: We deliberately do NOT call zephyr_library() here. The parent's
# `zephyr_library_sources_ifdef(CONFIG_HW75_LAMPARRAY ...)` adds our .c
# files directly to app/libapp.a, which is the only library guaranteed
# to be linked into the final zephyr.elf. Calling zephyr_library() in
# a subdir creates a separate static library that nothing pulls in, and
# the linker silently drops our SYS_INIT and the descriptor.
zephyr_library_sources_ifdef(CONFIG_HW75_LAMPARRAY ${CMAKE_CURRENT_SOURCE_DIR}/lamparray.c)
zephyr_library_sources_ifdef(CONFIG_HW75_LAMPARRAY ${CMAKE_CURRENT_SOURCE_DIR}/lamparray_hid.c)
zephyr_library_link_libraries_ifdef(CONFIG_HW75_LAMPARRAY hw75_proto)
zephyr_library_include_directories_ifdef(
	CONFIG_HW75_LAMPARRAY ${CMAKE_CURRENT_SOURCE_DIR}
)
zephyr_include_directories_ifdef(CONFIG_HW75_LAMPARRAY ${APPLICATION_SOURCE_DIR}/include)

if(BOARD STREQUAL "hw75_keyboard")
	zephyr_library_compile_options_ifdef(
		CONFIG_HW75_LAMPARRAY
		-include "${CMAKE_CURRENT_SOURCE_DIR}/lamparray_config_keyboard.h"
	)
elseif(BOARD STREQUAL "hw75_dynamic")
	zephyr_library_compile_options_ifdef(
		CONFIG_HW75_LAMPARRAY
		-include "${CMAKE_CURRENT_SOURCE_DIR}/lamparray_config_dynamic.h"
	)
else()
	if(CONFIG_HW75_LAMPARRAY)
		message(FATAL_ERROR "HW-75 LampArray: unknown board ${BOARD}, need a per-board config header")
	endif()
endif()

add_subdirectory(handler)