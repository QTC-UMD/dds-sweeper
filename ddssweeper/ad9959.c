#include "ad9959.h"

ad9959_config ad9959_get_default_config() {
    ad9959_config c;

    // TODO: find a better way to do this?
    memcpy(c.csr, "\x00\xf2", 2);
    memcpy(c.fr1, "\x01\x90\x00\x00", 4);
    memcpy(c.fr2, "\x02\x00\x00", 3);

    for (int i = 0; i < 4; i++) {
        memcpy(c.cfr[i], "\x03\x00\x03\x00", 4);
        memcpy(c.cftw0[i], "\x04\x66\x66\x66\x66", 5);
        memcpy(c.cpow0[i], "\x05\x00\x00", 3);
        memcpy(c.acr[i], "\x06\x00\x00\x00", 4);
        memcpy(c.lsrr[i], "\x07\x00\x00", 3);
        memcpy(c.rdw[i], "\x08\x00\x00\x00\x00", 5);
        memcpy(c.fdw[i], "\x09\x00\x00\x00\x00", 5);
    }

    c.sys_clk = 125 * MHZ * 4;
    c.pll_mult = 4;

    return c;
}

void ad9959_config_spi(ad9959_config* c, spi_inst_t* spi) { c->spi = spi; }

void ad9959_config_amp_sweep(ad9959_config* c, uint channel, bool no_dwell) {
    c->cfr[channel][1] &= 0x03;
    c->cfr[channel][2] &= 0x0b;
    c->cfr[channel][1] |= 0x40;
    c->cfr[channel][2] |= 0x40 | (no_dwell << 7);
}

void ad9959_config_freq(ad9959_config* c, uint channel, uint32_t freq) {
    uint32_t ftw = (uint32_t) round(freq * 4294967296.l / c->sys_clk);
    volatile uint8_t *bytes = (volatile uint8_t *) &ftw;
    
    for (int i = 0; i < 4; i++) {
        c->cftw0[channel][i + 1] = bytes[3 - i];
    }
}

void ad9959_config_pll_mult(ad9959_config* c, uint32_t val) {
    c->pll_mult = val;
}

void ad9959_config_sys_clk(ad9959_config* c, uint32_t val) {
    c->sys_clk;
    val;
}

void ad9959_send_config(ad9959_config* c) {
    spi_write_blocking(c->spi, c->fr1, 4);
    spi_write_blocking(c->spi, c->fr2, 3);

    // uint32_t csr = 0x0f & c->csr[1];
    for (int i = 0; i < 4; i++) {
        uint8_t csr[] = {
            0x00,
            (0x0f & c->csr[1]) | (1u << (i + 4)),
        };
        spi_write_blocking(c->spi, csr, 2);
        spi_write_blocking(c->spi, c->cfr[i], 4);
        spi_write_blocking(c->spi, c->cftw0[i], 5);
        spi_write_blocking(c->spi, c->cpow0[i], 3);
        spi_write_blocking(c->spi, c->acr[i], 4);
        spi_write_blocking(c->spi, c->lsrr[i], 3);
        spi_write_blocking(c->spi, c->rdw[i], 5);
        spi_write_blocking(c->spi, c->fdw[i], 5);
    }

    spi_write_blocking(c->spi, c->csr, 2);
}