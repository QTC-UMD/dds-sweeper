cmake_minimum_required(VERSION 3.13)

# Pull in PICO SDK (must be before project)
include(pico_sdk_import.cmake)

project(dds-sweeper C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_subdirectory(dds-sweeper)