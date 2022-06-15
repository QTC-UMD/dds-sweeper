/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "hardware/spi.h"
#include "pico/stdlib.h"

// Pins to use controlling the AD9959
#define PIN_MISO 16
#define PIN_MOSI 19
#define PIN_SCK 18
#define PIN_RESET 20
#define PIN_UPDATE 21

// SPI config
#define SPI_PORT spi0

#define READ_BIT 0x80
#define REF_CLK (20 * 1000 * 1000)
#define MULT 10

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

// helper functions
static void ad9959_reset();
static void ad9959_update();

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

typedef union ftw {
    uint8_t bytes[4];
    uint32_t word;
} ftw;

static void get_ftw(uint32_t f, ftw* word) {
    uint64_t fout = 0 | f;
    uint64_t fsys = REF_CLK * MULT;
    word->word = (uint32_t)round(fout * 4294967296.0 / fsys);
}

void send(uint8_t reg, uint len, uint8_t* data) {
    spi_write_blocking(SPI_PORT, &reg, 1);
    spi_write_blocking(SPI_PORT, data, len);
}

void set_freq(uint channel, uint32_t freq) {
    // turn the Hz into a FTW
    ftw channel_freq;
    get_ftw(freq, &channel_freq);

    // set just chanel we want
    uint8_t enable_nibbles[] = {
        0b0001, 
        0b0010,
        0b0100,
        0b1000        
    };

    // because of how get_ftw works, it is easier to send the LSB first
    uint8_t mesg = enable_nibbles[channel] << 4 | 0b0010;
    send(CSR, CSR_LEN, &mesg);
    ad9959_update();

    // send the freq
    send(CFTW0, CFTW0_LEN, channel_freq.bytes);

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
    gpio_init(PIN_RESET);
    gpio_set_dir(PIN_RESET, GPIO_OUT);
    gpio_put(PIN_RESET, 0);

    gpio_init(PIN_UPDATE);
    gpio_set_dir(PIN_UPDATE, GPIO_OUT);
    gpio_put(PIN_UPDATE, 0);

    // put chip in a known state
    ad9959_reset();

    // put in 3 wire spi mode
    uint8_t command[] = {CSR, 0xf2};
    spi_write_blocking(SPI_PORT, command, 2);
    ad9959_update();
}

int main() {
    
    setup();

    // set PLL multiplier
    int8_t fr1[] = {FR1, 0x28, 0x00, 0x00};
    spi_write_blocking(SPI_PORT, fr1, FR1_LEN + 1);
    ad9959_update();

    

    // Output 20Mhz signal on all chanels
    uint8_t command[] = {0x04, 0x19, 0x99, 0x99, 0x9A};
    spi_write_blocking(SPI_PORT, command, 5);
    ad9959_update();

    // try a read
    uint8_t x[] ={0x80};
    spi_write_blocking(SPI_PORT, x, 1);
    uint8_t resp[1];
    spi_read_blocking(SPI_PORT, 0, resp, 1);

    printf("Resp: %x\n", *resp);

    // us my function
    set_freq(0, 15000000);
    ad9959_update();

    printf("\n\n==============================\n");
    return 0;
}
