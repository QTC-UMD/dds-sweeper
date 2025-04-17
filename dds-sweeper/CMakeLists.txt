# compute firmware name
set(firmware_name dds-sweeper)
if(PICO_PLATFORM MATCHES "^rp2350")
    set(firmware_name "${firmware_name}_rp2350")
else()
    set(firmware_name "${firmware_name}_${PICO_PLATFORM}")
endif()

add_executable(${firmware_name}
        dds-sweeper.c
        ad9959.c
        ad9959.h
        fast_serial.c
        )

pico_generate_pio_header(${firmware_name} ${CMAKE_CURRENT_LIST_DIR}/trigger_timer.pio)

# pass in max instruction array size as a compiler definition
set(instructions_max_size 249856) # 244kB
if(PICO_PLATFORM MATCHES "^rp2350")
    set(instructions_max_size 512000) # 500kB
endif()
target_compile_definitions(${firmware_name} PUBLIC "SWEEPER_MAX_SIZE=${instructions_max_size}")

# pass in board type as a compiler definition
if (PICO_BOARD STREQUAL "pico")
    target_compile_definitions(${firmware_name} PUBLIC "SWEEPER_PICO_BOARD=1")
elseif (PICO_BOARD STREQUAL "pico2")
    target_compile_definitions(${firmware_name} PUBLIC "SWEEPER_PICO_BOARD=2")
else ()
    message(FATAL_ERROR "Unsupported PICO_BOARD")
endif()

target_link_libraries(${firmware_name}
        pico_stdlib
        pico_multicore
        pico_unique_id
        hardware_spi
        hardware_clocks
        hardware_pio
        hardware_dma
        hardware_flash
        tinyusb_device
        tinyusb_board
        )

target_include_directories(${firmware_name} PRIVATE .)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(${firmware_name})