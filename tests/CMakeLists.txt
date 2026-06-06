# Build all test applications in one tree:
#   cd tests && mkdir build && cd build
#   cmake -DPICO_SDK_PATH=/path/to/pico-sdk ..
#   make -j$(nproc)
#
# Each test is its own target, so you can also build just one, e.g.:
#   make loopback_test
#   make gpio_interrupt_test
#
# The .uf2 files land in build/<test>/<target>.uf2.
# Connect via USB serial (115200) to see test output.

cmake_minimum_required(VERSION 3.13)

# Default to Pico 2 (RP2350). Override with -DPICO_BOARD=<board>.
if(NOT DEFINED PICO_BOARD)
    set(PICO_BOARD pico2)
endif()

if(NOT DEFINED ENV{PICO_SDK_PATH})
    message(FATAL_ERROR "PICO_SDK_PATH is not set. "
        "Pass it as -DPICO_SDK_PATH=... or export it as an environment variable.")
endif()

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(lw_mcp251xfd_tests C CXX ASM)

pico_sdk_init()

# Pull in the driver library once for all tests.
add_subdirectory(.. lw_mcp251xfd_build)

# Each subdirectory declares its own test target(s).
add_subdirectory(loopback)
add_subdirectory(gpio)
