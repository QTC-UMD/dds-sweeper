#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ad9959.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/spi.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
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

typedef struct pio_sm {
    PIO pio;
    uint sm;
    uint offset;
} pio_sm;

pio_sm trig;

static void trigger(uint channel, uint val) {
    pio_sm_put(trig.pio, trig.sm, val);
    gpio_put(TRIGGER, 1);
    sleep_us(1);
    gpio_put(TRIGGER, 0);
}

static void wait(uint channel) { pio_sm_get_blocking(trig.pio, trig.sm); }

static void update() { trigger(0, 2); }

void init_pin(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
}

// clang-format off
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

    // no more no dwell mode
    0x03, 0x40, 0x43, 0x10,
    0x02, 0x00, 0x00,
    0x17, 0x00, 0x40, 0x00, 0x00,
    0x18, 0xc0, 0x00, 0x00, 0x00,

    // 0 => 50
    0x06, 0x00, 0x00, 0x00,
    0x07, 0x01, 0x01,
    0x08, 0x00, 0x40, 0x00, 0x00,
    0x0a, 0x80, 0x00, 0x00, 0x00,

    // 50 => 100
    0x06, 0x00, 0x02, 0x00,
    0x07, 0x01, 0x01,
    0x08, 0x00, 0x40, 0x00, 0x00,
    0x0a, 0xff, 0xc0, 0x00, 0x00,

    // no more autoclear
    0x03, 0x40, 0x43, 0x00,
    0x02, 0x00, 0x00,
    0x17, 0x00, 0x40, 0x00, 0x00,
    0x18, 0xc0, 0x00, 0x00, 0x00,

    // 100 => 50
    0x06, 0x00, 0x02, 0x00,
    0x07, 0x01, 0x01,
    0x09, 0x00, 0x40, 0x00, 0x00,
    0x0a, 0xff, 0xc0, 0x00, 0x00,

    // 100 => 25
    0x06, 0x00, 0x01, 0x00,
    0x07, 0x01, 0x01,
    0x08, 0xff, 0xff, 0xff, 0xff,
    0x0a, 0xff, 0xc0, 0x00, 0x00,

    // 50 => 100
    0x06, 0x00, 0x02, 0x00,
    0x07, 0x01, 0x01,
    0x08, 0x00, 0x40, 0x00, 0x00,
    0x0a, 0xff, 0xc0, 0x00, 0x00,

};
// clang-format on

ad9959_config ad9959;

void background() {
    // let other core know we ready
    multicore_fifo_push_blocking(0);

    // wait for signal from the other core
    multicore_fifo_pop_blocking();

    // enter amplitude sweep mode
    ad9959_set_amp_sweep(&ad9959, true);

    // write the first sweep manually since there was no trigger to cause it
    spi_write_blocking(SPI_PORT, instructions, 17);

    // for the first one tell the other core it has been prepped so it knows to
    // start sending the triggers
    multicore_fifo_push_blocking(0);

    for (int i = 1; i < 3; i++) {
        wait(0);

        spi_write_blocking(SPI_PORT, instructions + (i * INS_SIZE), INS_SIZE);
    }

    // test rising to rising ramps
    wait(0);

    // switch ramp modes
    spi_write_blocking(SPI_PORT, instructions + (3 * INS_SIZE), INS_SIZE);
    // 0 => 50
    spi_write_blocking(SPI_PORT, instructions + (4 * INS_SIZE), INS_SIZE);

    wait(0);

    // 50 +> 100
    spi_write_blocking(SPI_PORT, instructions + (5 * INS_SIZE), INS_SIZE);

    wait(0);

    // turn off autoclear
    spi_write_blocking(SPI_PORT, instructions + (6 * INS_SIZE), INS_SIZE);
    update();

    // 100 => 50
    spi_write_blocking(SPI_PORT, instructions + (7 * INS_SIZE), INS_SIZE);

    // wait(0);
    wait(0);

    // 100 => 25
    spi_write_blocking(SPI_PORT, instructions + (8 * INS_SIZE), INS_SIZE);

    wait(0);

    spi_write_blocking(SPI_PORT, instructions + (9 * INS_SIZE), INS_SIZE);

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
    printf("baudrate: %d\n", spi_init(SPI_PORT, 100 * MHZ));
    spi_set_format(SPI_PORT, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // launch other core
    multicore_launch_core1(background);
    multicore_fifo_pop_blocking();
    printf("Other core has launched\n");

    // init the trigger
    trig.pio = pio0;
    trig.sm = pio_claim_unused_sm(pio0, true);
    trig.offset = pio_add_program(pio0, &trigger_program);
    trigger_program_init(pio0, trig.sm, trig.offset, TRIGGER, P0, PIN_UPDATE);

    // init the trigger pin
    init_pin(TRIGGER);


    // put chip in a known state
    init_pin(PIN_RESET);
    ad9959_reset();

    ad9959 = ad9959_get_default_config();
    ad9959_set_spi(&ad9959, SPI_PORT);
    ad9959_send_config(&ad9959);

    update();

    // tell the other core to send the first step
    multicore_fifo_push_blocking(0);
    // wait for it to finish
    multicore_fifo_pop_blocking();
    trigger(0, 1);
    trigger(0, 3);

    sleep_us(5);
    trigger(0, 1);
    trigger(0, 3);

    sleep_us(5);
    trigger(0, 1);
    trigger(0, 3);

    // switch out of no dwell mode
    sleep_us(5);
    sleep_us(5);
    trigger(0, 1);

    sleep_us(5);
    trigger(0, 1);

    sleep_us(5);
    sleep_us(5);
    sleep_us(5);
    trigger(0, 0);

    sleep_us(5);
    trigger(0, 1);
    sleep_us(1);
    trigger(0, 3);

    sleep_us(5);
    trigger(0, 1);

    printf("Fin\n");
    return 0;
}
