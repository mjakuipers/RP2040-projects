cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(pico_hp41v1)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project

add_executable(pico_hp41v1
        pico_hp41v1.c
        )

pico_generate_pio_header(pico_hp41v1 ${CMAKE_CURRENT_LIST_DIR}/hp41_pio.pio)

# pull in common dependencies
target_link_libraries(  pico_hp41v1
                        pico_stdlib
                        hardware_pio
                        hardware_clocks
                        pico_multicore
                        )

# define default UART for printf console output, USB is enable, UART is disabled
pico_enable_stdio_usb(pico_hp41v1 1)
pico_enable_stdio_uart(pico_hp41v1 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(pico_hp41v1)

# add url via pico_set_program_url
# example_auto_set_url(pico_hp41v1)
