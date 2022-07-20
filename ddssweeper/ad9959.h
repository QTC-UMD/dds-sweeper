
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
    double sys_clk;
    double ref_clk;
    uint32_t pll_mult;
    int sweep_type;
    uint channels;
} ad9959_config;

uint32_t send_freq(ad9959_config* c, uint channel, double freq);
double send_phase(uint channel, double phase);
double send_amp(uint channel, double amp);
void ad9959_default_config();
void read_reg(uint8_t reg, size_t len, uint8_t* buf);
void read_all();

#endif