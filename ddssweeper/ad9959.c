#include "ad9959.h"



ad9959_config ad9959_get_default_config() {
    ad9959_config c;

    // TODO: find a better way to do this?
    memcpy(c.csr, "\x00\xf2", 2);                // thing
    memcpy(c.fr1, "\x01\x90\x00\x00", 4);        // thing
    memcpy(c.fr2, "\x02\x00\x00", 3);            // thing
    memcpy(c.cfr, "\x03\x00\x03\x00", 4);        // thing
    memcpy(c.cftw0, "\x04\x66\x66\x66\x66", 5);  // thing
    memcpy(c.cpow0, "\x05\x00\x00", 3);          // thing
    memcpy(c.acr, "\x06\x00\x00\x00", 4);        // thing
    memcpy(c.lsrr, "\x07\x00\x00", 3);           // thing
    memcpy(c.rdw, "\x08\x00\x00\x00\x00", 5);    // thing
    memcpy(c.fdw, "\x09\x00\x00\x00\x00", 5);    // thing

    return c;
}

void ad9959_set_spi(ad9959_config* c, spi_inst_t* spi) {
    c->spi = spi;
}

void ad9959_set_amp_sweep(ad9959_config* c, bool no_dwell) {
    c->cfr[1] &= 0x03;
    c->cfr[2] &= 0x0b;
    c->cfr[1] |= 0x40;
    c->cfr[2] |= 0x40 | (no_dwell << 7);
    
    // printf("%02x %02x %02x %02x\n", (c->cfr[0]), (c->cfr[1]), (c->cfr[2]), (c->cfr[3]));

    spi_write_blocking(c->spi, c->cfr, 4);
}

void ad9959_send_config(ad9959_config* c) {
    spi_write_blocking(c->spi, c->csr, 2);
    spi_write_blocking(c->spi, c->fr1, 4);
    spi_write_blocking(c->spi, c->fr2, 3);
    spi_write_blocking(c->spi, c->cfr, 4);
    spi_write_blocking(c->spi, c->cftw0, 5);
    spi_write_blocking(c->spi, c->cpow0, 3);
    spi_write_blocking(c->spi, c->acr, 4);
    spi_write_blocking(c->spi, c->lsrr, 3);
    spi_write_blocking(c->spi, c->rdw, 5);
    spi_write_blocking(c->spi, c->fdw, 5);
}
