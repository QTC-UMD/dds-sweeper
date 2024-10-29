
#ifndef _AD9959_H
#define _AD9959_H

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "hardware/clocks.h"
#include "hardware/spi.h"
#include "hardware/structs/watchdog.h"
#include "pico/stdlib.h"

// Default Pins to use
#define PIN_MISO 0  //RX
#define PIN_MOSI 19  //TX
#define PIN_SCK 6
#define PIN_SYNC 3
#define PIN_CLOCK 21
#define PIN_UPDATE 8
#define PIN_RESET 4
#define P0 12
#define P1 11
#define P2 10
#define P3 9
#define PROFILE_ASC false
#define TRIGGER 16
#define INT_TRIGGER 17

#define PIO_TRIG pio0
#define PIO_TIME pio1

#define SPI spi0

typedef struct ad9959_config {
    double ref_clk;
    uint32_t pll_mult;
    int sweep_type;
    uint channels;
} ad9959_config;

// get tuning words
double get_asf(double amp, uint8_t* buf);
double get_ftw(ad9959_config* c, double freq, uint8_t* buf);
double get_pow(double phase, uint8_t* buf);

// send tuning words
void send_channel(uint8_t reg, uint8_t channel, uint8_t* buf, size_t len);
void send(uint8_t reg, uint8_t* buf, size_t len);

// Readback from AD9959
void read_reg(uint8_t reg, size_t len, uint8_t* buf);
void read_all();

// control
void set_pll_mult(ad9959_config* c, uint mult);
void set_ref_clk(ad9959_config* c, uint64_t freq);
void single_step_mode();
void clear();

#endif
