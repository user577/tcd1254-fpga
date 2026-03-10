cmake_minimum_required(VERSION 3.13)

# Pull in Pico SDK (set PICO_SDK_PATH env variable)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(tcd1254_controller C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(tcd1254_controller
    src/main.c
    src/fpga_spi.c
    src/stepper.c
    src/tmc2209.c
    src/sort_sequence.c
    src/config.c
)

# Generate PIO headers
pico_generate_pio_header(tcd1254_controller ${CMAKE_CURRENT_LIST_DIR}/pio/stepper.pio)

target_include_directories(tcd1254_controller PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(tcd1254_controller
    pico_stdlib
    pico_sync
    hardware_spi
    hardware_pio
    hardware_gpio
    hardware_flash
    hardware_timer
    pico_multicore
)

# Enable USB serial for config interface (Python talks to this)
pico_enable_stdio_usb(tcd1254_controller 1)
pico_enable_stdio_uart(tcd1254_controller 0)

pico_add_extra_outputs(tcd1254_controller)
