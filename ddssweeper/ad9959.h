
#ifndef _AD9959_H
#define _AD9959_H

#include <string.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"

typedef struct ad9959_config {
    uint8_t csr[2];
    uint8_t fr1[4];
    uint8_t fr2[3];
    uint8_t cfr[4];
    uint8_t cftw0[5];
    uint8_t cpow0[3];
    uint8_t acr[4];
    uint8_t lsrr[3];
    uint8_t rdw[5];
    uint8_t fdw[5];
    spi_inst_t* spi;
} ad9959_config;

ad9959_config ad9959_get_default_config();

void ad9959_set_spi(ad9959_config* c, spi_inst_t* spi);
void ad9959_set_amp_sweep(ad9959_config* c, bool no_dwell);

void ad9959_send_config(ad9959_config* c);

#endif