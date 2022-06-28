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

#include "tusb.h"

#define VERSION "0.0.0"

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

// ============================================================================
// global variables
// ============================================================================
typedef struct pio_sm {
    PIO pio;
    uint sm;
    uint offset;
} pio_sm;

pio_sm trig;
ad9959_config ad9959;
char readstring[256];

// hard-coded sweeps
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

// ============================================================================
// Helper Functions
// ============================================================================
void ad9959_reset() {
    sleep_us(1);
    gpio_put(PIN_RESET, 1);
    sleep_us(1);
    gpio_put(PIN_RESET, 0);
    sleep_us(1);
}

void trigger(uint channel, uint val) {
    pio_sm_put(trig.pio, trig.sm, val);
    gpio_put(TRIGGER, 1);
    sleep_us(1);
    gpio_put(TRIGGER, 0);
}

void wait(uint channel) { pio_sm_get_blocking(trig.pio, trig.sm); }

void update() { trigger(0, 2); }

void init_pin(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
}

void background() {
    // let other core know we ready
    multicore_fifo_push_blocking(0);

    // wait for signal from the other core
    multicore_fifo_pop_blocking();

    // nothing to do per se
}

void readline() {
    int i = 0;
    char c;
    while (true) {
        c = getchar();
        if (c == '\n') {
            readstring[i] = '\0';
            return;
        } else {
            readstring[i++] = c;
        }
    }
}

void loop() {
    readline();
    // scanf("%s", readstring);
    if (strncmp(readstring, "version", 7) == 0) {
        printf("version: %s\n", VERSION);
    } else if (strncmp(readstring, "status", 6) == 0) {
        printf("Running\n");

    } else {
        printf("Unrecognized command: %s\n", readstring);
    }
}

int main() {
    // set sysclock to default 125 MHz
    set_sys_clock_khz(125 * MHZ / 1000, false);
    clock_gpio_init(21, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, 1);

    // enable output for debugging purposes

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

    // launch other core
    multicore_launch_core1(background);
    multicore_fifo_pop_blocking();

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

    stdio_init_all();
    // not sure why, but there is an extra 0xff that needs to get trashed or it
    // messes everything up
    getchar();

    while (true) {
        loop();
    }

    return 0;
}
