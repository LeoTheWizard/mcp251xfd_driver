# Build this test on its own:
#   cd tests/gpio && mkdir build && cd build
#   cmake -DPICO_SDK_PATH=/path/to/pico-sdk ..
#   make -j$(nproc)
#
# Or build it together with the other tests via tests/CMakeLists.txt.
#
# The .uf2 file will be at build/gpio_interrupt_test.uf2.
# Connect via USB serial (115200) to see test output.

cmake_minimum_required(VERSION 3.13)

# When built standalone the driver target doesn't exist yet, so bootstrap
# the Pico SDK and pull in the library. When built from tests/CMakeLists.txt
# this has already been done, so we skip straight to declaring the target.
if(NOT TARGET lw_mcp251xfd)
    if(NOT DEFINED PICO_BOARD)
        set(PICO_BOARD pico2)
    endif()

    if(NOT DEFINED ENV{PICO_SDK_PATH})
        message(FATAL_ERROR "PICO_SDK_PATH is not set. "
            "Pass it as -DPICO_SDK_PATH=... or export it as an environment variable.")
    endif()

    include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

    project(lw_mcp251xfd_gpio_test C CXX ASM)

    pico_sdk_init()

    add_subdirectory(../.. lw_mcp251xfd_build)
endif()

add_executable(gpio_interrupt_test gpio_interrupt_test.c)

target_compile_definitions(gpio_interrupt_test PRIVATE
    MCP251XFD_DEBUG
    MCP251XFD_ENABLE_ERROR_MESSAGES
)

target_link_libraries(gpio_interrupt_test PRIVATE
    lw_mcp251xfd
    pico_stdlib
    hardware_spi
    pico_bootrom
)

# Output over USB; disable UART to keep the default UART pins free.
pico_enable_stdio_usb(gpio_interrupt_test 1)
pico_enable_stdio_uart(gpio_interrupt_test 0)

pico_add_extra_outputs(gpio_interrupt_test)
