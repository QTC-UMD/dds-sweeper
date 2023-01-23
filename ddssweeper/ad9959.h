
#ifndef _AD9959_H
#define _AD9959_H

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "hardware/clocks.h"
#include "hardware/spi.h"
#include "hardware/structs/watchdog.h"
#include "pico/stdlib.h"

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

#endif