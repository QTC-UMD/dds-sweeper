add_executable(dds-sweeper
        dds-sweeper.c
        ad9959.c
        ad9959.h
        )

pico_generate_pio_header(dds-sweeper ${CMAKE_CURRENT_LIST_DIR}/trigger_timer.pio)

target_link_libraries(dds-sweeper 
        pico_stdlib
        pico_multicore
        hardware_spi
        hardware_clocks
        hardware_pio
        hardware_dma
        hardware_irq
        )

# UART/USB config
pico_enable_stdio_usb(dds-sweeper 1)
pico_enable_stdio_uart(dds-sweeper 1)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(dds-sweeper)