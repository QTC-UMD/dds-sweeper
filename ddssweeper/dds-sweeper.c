#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/spi.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "trigger.pio.h"

// Pins to use controlling the AD9959
#define PIN_MISO 12
#define PIN_MOSI 15
#define PIN_SCK 14
#define PIN_RESET 20
#define PIN_UPDATE 22
#define P0 16
#define P1 17
#define P2 18
#define P3 19
#define TRIGGER 8

// SPI config
#define SPI_PORT spi1


// helper functions
static void ad9959_reset() {
    sleep_us(1);
    gpio_put(PIN_RESET, 1);
    sleep_us(1);
    gpio_put(PIN_RESET, 0);
    sleep_us(1);
}

static void ad9959_update() {
    gpio_put(PIN_UPDATE, 1);
    sleep_us(1);
    gpio_put(PIN_UPDATE, 0);
}

static void trigger(uint val) {
    pio_sm_put(pio0, 0, val);
    gpio_put(TRIGGER, 1);
    sleep_us(1);
    gpio_put(TRIGGER, 0);
}

void init_pin(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
}


#define INS_SIZE 17
uint8_t instructions[] = {
    // 0 => 100
    0x06, 0x00, 0x00, 0x00,
    0x07, 0x01, 0x01,
    0x08, 0x00, 0x40, 0x00, 0x00,
    0x0a, 0xff, 0xc0, 0x00, 0x00,

    // 0 => 25
    0x06, 0x00, 0x00, 0x00,
    0x07, 0x01, 0x01,
    0x08, 0x00, 0x40, 0x00, 0x00,
    0x0a, 0x40, 0x00, 0x00, 0x00,

    // 0 => 75
    0x06, 0x00, 0x00, 0x00,
    0x07, 0x01, 0x01,
    0x08, 0x00, 0x40, 0x00, 0x00,
    0x0a, 0xc0, 0x00, 0x00, 0x00,
};



void background() {

    // let other core know we ready
    multicore_fifo_push_blocking(0);

    printf("I can do this too...\n");

    spi_write_blocking(SPI_PORT, instructions, 17);

    pio_sm_get_blocking(pio0, 0);

    spi_write_blocking(SPI_PORT, instructions + 17, 17);

}



int main() {
    // set sysclock to default 125 MHz
    set_sys_clock_khz(125 * MHZ / 1000, false);
    clock_gpio_init(21, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, 1);


    // enable output for debugging purposes
    stdio_init_all();
    printf("\n\nhowdy\n");

    // turn on light as indicator that this is working!
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);

    // init SPI
    spi_init(SPI_PORT, 100 * MHZ);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // init reset and update pins
    init_pin(PIN_RESET);
    init_pin(PIN_UPDATE);

    // put chip in a known state
    ad9959_reset();

    uint8_t setup[] = {
        0x00, 0xf2,                     // set serial mode
        0x01, 0x90, 0x00, 0x00,         // set PLL default multiplier
        0x03, 0x40, 0x43, 0x04,         // amplitude freq mode
        0x04, 0x33, 0x33, 0x33, 0x33,   // set freq of 100MHz
        0x06, 0x00, 0x00, 0x00,         // max amp
    };

    spi_write_blocking(SPI_PORT, setup, sizeof(setup));
    ad9959_update();

    // init pio trigger
    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &trigger_program);
    trigger_program_init(pio, sm, offset, TRIGGER, P0, PIN_UPDATE);

    // launch other core
    multicore_launch_core1(background);
    // wait for it
    multicore_fifo_pop_blocking();
    printf("Other core has launched\n");

    // uint8_t ramp_up[] = {
    //     0x06, 0x00, 0x00, 0x00,
    //     0x0a, 0x80, 0x00, 0x00, 0x00,
    //     0x07, 0x01, 0x01,
    //     0x08, 0x00, 0x40, 0x00, 0x00
    // };

    // spi_write_blocking(SPI_PORT, ramp_up, sizeof(ramp_up));
    // ad9959_update();

    sleep_us(50);
    init_pin(TRIGGER);
    trigger(1);
    


    return 0;
}
