
#ifndef _AD9959_H
#define _AD9959_H

#include <string.h>
#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/clocks.h"
#include "hardware/structs/watchdog.h"

typedef struct ad9959_config {
    uint8_t csr[2];
    uint8_t fr1[4];
    uint8_t fr2[3];
    uint8_t cfr[4][4];
    uint8_t cftw0[4][5];
    uint8_t cpow0[4][3];
    uint8_t acr[4][4];
    uint8_t lsrr[4][3];
    uint8_t rdw[4][5];
    uint8_t fdw[4][5];
    spi_inst_t* spi;
    uint32_t sys_clk;
    uint32_t pll_mult;
} ad9959_config;

ad9959_config ad9959_get_default_config();

void ad9959_config_spi(ad9959_config* c, spi_inst_t* spi);
void ad9959_config_amp_sweep(ad9959_config* c, uint channel, bool no_dwell);
void ad9959_config_pll_mult(ad9959_config* c, uint32_t val);
void ad9959_config_sys_clk(ad9959_config* c, uint32_t val);
uint32_t ad9959_config_freq(ad9959_config* c, uint channel,  double freq);

void ad9959_send_config(ad9959_config* c);

void ad9959_read_all(ad9959_config* c);

#endif