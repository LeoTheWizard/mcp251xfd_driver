# Build instructions:
#   cd tests && mkdir build && cd build
#   cmake -DPICO_SDK_PATH=/path/to/pico-sdk ..
#   make -j$(nproc)
#
# The .uf2 file will be at build/loopback_test.uf2.
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

# Pull in the driver library from the parent directory.
add_subdirectory(../.. lw_mcp251xfd_build)

add_executable(loopback_test loopback_test.c)

target_compile_definitions(loopback_test PRIVATE
    MCP251XFD_DEBUG
    MCP251XFD_ENABLE_ERROR_MESSAGES
)

target_link_libraries(loopback_test PRIVATE
    lw_mcp251xfd
    pico_stdlib
    hardware_spi
    pico_bootrom
)

# Output over USB; disable UART to keep the default UART pins free.
pico_enable_stdio_usb(loopback_test 1)
pico_enable_stdio_uart(loopback_test 0)

pico_add_extra_outputs(loopback_test)
