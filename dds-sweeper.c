#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/spi.h"
#include "hardware/structs/clocks.h"
#include "hardware/structs/pll.h"
#include "pico/stdlib.h"

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

// SPI config
#define SPI_PORT spi1

#define READ_BIT 0x80
#define REF_CLK (125 * 1000 * 1000)
#define MULT 4
#define CSR_NIBBLE 0x2

// Serial Register Addresses
#define CSR 0x00
#define FR1 0x01
#define FR2 0x02
#define CFR 0x03
#define CFTW0 0x04

// Register Size
#define CSR_LEN 1
#define FR1_LEN 3
#define FR2_LEN 2
#define CFR_LEN 3
#define CFTW0_LEN 4

uint8_t zeros[10];

// helper functions
static void ad9959_reset() {
    sleep_us(1);
    gpio_put(PIN_RESET, 1);
    sleep_us(1);
    gpio_put(PIN_RESET, 0);
    sleep_us(1);
}

static void ad9959_update() {
    sleep_us(1);
    gpio_put(PIN_UPDATE, 1);
    sleep_us(1);
    gpio_put(PIN_UPDATE, 0);
    sleep_us(1);
}

typedef union word_t {
    uint8_t bytes[4];
    uint32_t word;
} word_t;

static void get_freq_wrd(uint32_t f, word_t* word) {
    uint64_t fout = f;
    uint64_t fsys = REF_CLK * MULT;
    uint32_t val = (uint32_t)round(fout * 4294967296.0 / fsys);
    word->word = (val << 24) | ((val & 0xff00) << 8) | ((val & 0xff0000) >> 8) |
                 (val >> 24);
}

void send(uint8_t reg, uint len, uint8_t* data) {
    spi_write_blocking(SPI_PORT, &reg, 1);
    spi_write_blocking(SPI_PORT, data, len);
}

void read(uint8_t reg, uint len, uint8_t* buf) {
    reg |= READ_BIT;
    spi_write_blocking(SPI_PORT, &reg, 1);
    spi_read_blocking(SPI_PORT, 0, buf, len);
}

void select_channel(uint channel) {
    // set just channel we want
    uint8_t enable_nibbles[] = {0b0001, 0b0010, 0b0100, 0b1000};

    uint8_t mesg = enable_nibbles[channel] << 4 | CSR_NIBBLE;
    send(CSR, CSR_LEN, &mesg);
}

void set_freq(uint channel, uint32_t freq) {
    // turn the Hz into a FTW
    word_t ftw;
    get_freq_wrd(freq, &ftw);

    select_channel(channel);

    // send the freq
    send(CFTW0, CFTW0_LEN, ftw.bytes);
}

void get_delta_word(uint32_t time, uint32_t start, uint32_t end, word_t* word) {
    double min_rate = 0xff * 4.0 / (REF_CLK * MULT);
    double steps = time / min_rate;
    double delta = (end - start) / steps;
    uint64_t fsys = REF_CLK * MULT;
    uint32_t val = (uint32_t)round(delta * 4294967296.0 / fsys);
    val = val == 0 ? 1 : val;
    word->word = (val << 24) | ((val & 0xff00) << 8) | ((val & 0xff0000) >> 8) |
                 (val >> 24);
}

void sweep_setup(uint channel, uint32_t start, uint32_t end, uint32_t time) {
    // prep all values
    word_t s0;
    get_freq_wrd(start, &s0);
    word_t e0;
    get_freq_wrd(end, &e0);

    word_t rdw;
    get_delta_word(time, start, end, &rdw);

    // assume most fine gradient
    uint8_t srr[2] = {0xff, 0xff};

    // the CFR register always gets the same values for a sweep
    uint8_t sweep[3] = {0x80, 0x43, 0x04};

    select_channel(channel);

    send(0x04, 4, s0.bytes);
    send(0x0a, 4, e0.bytes);
    send(CFR, CFR_LEN, sweep);
    send(0x07, 2, srr);
    send(0x08, 4, rdw.bytes);
    send(0x09, 4, rdw.bytes);
}

void init_pin(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
}

void setup() {
    // enable output for debugging purposes
    stdio_init_all();

    // turn on light as indicator that this is working!
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);

    // init SPI
    spi_init(SPI_PORT, 10 * 1000 * 1000);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // init reset and update pins
    init_pin(PIN_RESET);
    init_pin(PIN_UPDATE);
    init_pin(P0);
    init_pin(P1);
    init_pin(P2);
    init_pin(P3);

    // put chip in a known state
    ad9959_reset();

    // put in 3 wire spi mode
    uint8_t command[] = {CSR, 0xf2};
    spi_write_blocking(SPI_PORT, command, 2);
    ad9959_update();
}

int main() {
    // default system clock speed is 125 MHz
    // need to set it before io init or it is bad
    // stdio_init_all();
    set_sys_clock_khz(125 * MHZ / 1000, false);
    clock_gpio_init(21, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, 1);

    memset(zeros, 0, 10);
    setup();
    printf("\n\nHowdy\n");

    // set PLL multiplier
    // int8_t fr1[] = {FR1, 0x28, 0x00, 0x00};
    int8_t fr1[] = {FR1, 0x90, 0x00, 0x00};
    spi_write_blocking(SPI_PORT, fr1, FR1_LEN + 1);
    ad9959_update();

    set_freq(0, 5 * MHZ);
    ad9959_update();

    gpio_put(P0, 0);
    sweep_setup(0, 5 * MHZ, 25 * MHZ, 5);
    ad9959_update();
    gpio_put(P0, 1);

    sleep_ms(5500);
    gpio_put(P0, 0);

    sweep_setup(0, 5 * MHZ, 40 * MHZ, 3);
    sleep_ms(5500);
    ad9959_update();
    gpio_put(P0, 1);

    sweep_setup(0, 15 * MHZ, 40 * MHZ, 2);
    sleep_ms(3500);
    ad9959_update();
    gpio_put(P0, 0);

    return 0;
}
