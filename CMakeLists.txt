cmake_minimum_required(VERSION 3.13)

# Pull in PICO SDK (must be before project)
include(pico_sdk_import.cmake)

project(dds-sweeper C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()


add_executable(dds-sweeper
        dds-sweeper.c
        )

target_link_libraries(dds-sweeper 
        pico_stdlib
        hardware_spi
        hardware_clocks
        )

# enable UART output and disable Serial USB
pico_enable_stdio_usb(dds-sweeper 0)
pico_enable_stdio_uart(dds-sweeper 1)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(dds-sweeper)