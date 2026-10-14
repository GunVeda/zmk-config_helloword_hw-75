# Copyright (c) 2022-2023 XiNGRZ
# SPDX-License-Identifier: MIT

zephyr_library_named(hw75_proto)

if(CONFIG_HW75_PROTO)

# Nanopb's FindNanopb.cmake (0.4.6-zephyr) calls
#   target_compile_features(nanopb PUBLIC c_std_11)
# but CMake's feature list for the arm-none-eabi-gcc cross-compiler
# does not include c_std_11, so the find_package call fails with
# "The compiler feature c_std_11 is not known to C compiler GNU 12.2.0".
# Add c_std_11 to the known features list before find_package.
list(APPEND CMAKE_C_COMPILE_FEATURES c_std_11)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ZEPHYR_NANOPB_MODULE_DIR}/extra)
find_package(Nanopb REQUIRED)

set(PROTO_DEFS)

if(CONFIG_HW75_PROTO_USB_COMM)
    list(APPEND PROTO_DEFS usb_comm.proto)
endif()

if(CONFIG_HW75_PROTO_UART_COMM)
    list(APPEND PROTO_DEFS uart_comm.proto)
endif()

nanopb_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_DEFS})
zephyr_library_sources(${PROTO_SRCS} ${PROTO_HDRS})

zephyr_include_directories(${NANOPB_INCLUDE_DIRS})
zephyr_include_directories(${CMAKE_CURRENT_BINARY_DIR})

endif() # CONFIG_HW75_PROTO
