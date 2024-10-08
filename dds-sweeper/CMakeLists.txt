add_executable(dds-sweeper
        dds-sweeper.c
        ad9959.c
        ad9959.h
        fast_serial.c
        )

pico_generate_pio_header(dds-sweeper ${CMAKE_CURRENT_LIST_DIR}/trigger_timer.pio)

target_link_libraries(dds-sweeper 
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

target_include_directories(dds-sweeper PRIVATE .)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(dds-sweeper)