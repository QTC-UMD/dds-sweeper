#include "ad9959.h"

ad9959_config ad9959_get_default_config() {
    ad9959_config c;

    // TODO: find a better way to do this?
    // its probably fin actually

    // 2 wire spi mode with all channels selected
    memcpy(c.csr, "\x00\xf2", 2);

    // PLL multiplier of x4
    memcpy(c.fr1, "\x01\x90\x00\x00", 4);

    // nothing in FR2
    memcpy(c.fr2, "\x02\x00\x00", 3);

    for (int i = 0; i < 4; i++) {
        // select the channel
        memcpy(c.cfr[i], "\x03\x00\x03\x04", 4);

        // all channel registers initialized to zero
        memcpy(c.cftw0[i], "\x04\x00\x00\x00\x00", 5);
        memcpy(c.cpow0[i], "\x05\x00\x00", 3);
        memcpy(c.acr[i], "\x06\x00\x00\x00", 4);
        memcpy(c.lsrr[i], "\x07\x00\x00", 3);
        memcpy(c.rdw[i], "\x08\x00\x00\x00\x00", 5);
        memcpy(c.fdw[i], "\x09\x00\x00\x00\x00", 5);
    }

    c.ref_clk = 125 * MHZ;
    c.pll_mult = 4;
    c.sys_clk = c.ref_clk * c.pll_mult;
    c.sweep_type = -1;
    c.spi = spi1;

    c.channels = 1;

    return c;
}

void ad9959_config_mode(ad9959_config* c, uint type, uint no_dwell) {
    c->sweep_type = type;

    // zero out the modulation mode bits
    c->fr1[2] &= 0b11111100;

    for (int i = 0; i < 4; i++) {
        c->cfr[i][1] = (type << 6);
        c->cfr[i][2] = (no_dwell << 7) | ((type ? 1 : 0) << 6) | 0b100011;
        c->cfr[i][3] = 0b00010000;
    }
}

uint32_t ad9959_config_freq(ad9959_config* c, uint channel, double freq) {
    uint32_t ftw = (uint32_t)round(freq * 4294967296.l / c->sys_clk);

    uint8_t* bytes = (uint8_t*)&ftw;
    for (int i = 0; i < 4; i++) {
        // 1 offset for the register address
        c->cftw0[channel][i + 1] = bytes[3 - i];
    }

    // for debugging
    return ftw;
}

// uint32_t ad9959_config_phase(ad9959_config* c, uint channel, double phase) {
//     uint32_t pow = (uint32_t)round(phase / 360.0 * 16384.0);

//     uint8_t* bytes = (uint8_t*)&pow;
//     printf("PHASE OFFSET: ");
//     for (int i = 0; i < 2; i++) {
//         // 1 offset for the register address
//         c->cpow0[channel][i + 1] = bytes[1 - i];
//         printf("%02x ", bytes[1 - i]);
//     }
//     printf("\n");

//     // for debugging
//     return pow;
// }

uint32_t ad9959_config_amp(ad9959_config* c, uint channel, double amp) {
    uint32_t atw = (uint16_t)round(amp * 1023);

    // amplitude multiplier bit
    atw |= 0x1000;

    memcpy(c->acr[channel] + 1, (uint8_t*)&atw, 3);

    // for debugging
    return atw;
}

void ad9959_config_pll_mult(ad9959_config* c, uint32_t val) {
    c->pll_mult = val;
    c->sys_clk = c->ref_clk * val;

    // zero out the current pll multiplier
    c->fr1[1] &= 0b10000011;
    c->fr1[1] |= val << 2;

    // VCO gain bit
    // maybe should give the user more control?
    if (c->sys_clk > 255) c->fr1[1] |= 0x80;
}

void ad9959_send_config(ad9959_config* c) {
    spi_write_blocking(c->spi, c->fr1, 4);
    spi_write_blocking(c->spi, c->fr2, 3);

    for (int i = 0; i < 4; i++) {
        uint8_t csr[] = {
            0x00,
            (1u << (i + 4)) | 0x02,
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

// could be helpful for debugging
static void read(ad9959_config* c, uint8_t reg, size_t len, uint8_t* buf) {
    reg |= 0x80;
    spi_write_blocking(c->spi, &reg, 1);
    spi_read_blocking(c->spi, 0, buf, len);
}

void ad9959_read_all(ad9959_config* c) {
    spi_set_baudrate(spi1, 1 * MHZ);

    uint8_t resp[20];

    read(c, 0x00, 1, resp);
    printf(" CSR: %02x\n", resp[0]);

    read(c, 0x01, 3, resp);
    printf(" FR1: %02x %02x %02x\n", resp[0], resp[1], resp[2]);

    read(c, 0x02, 2, resp);
    printf(" FR2: %02x %02x\n", resp[0], resp[1]);

    for (int i = 0; i < 4; i++) {
        printf("CHANNEL %d:\n", i);

        uint8_t csr[] = {0x00, (1u << (i + 4)) | 0x02};

        spi_write_blocking(c->spi, csr, 2);

        read(c, 0x03, 3, resp);
        printf(" CFR: %02x %02x %02x\n", resp[0], resp[1], resp[2]);

        read(c, 0x04, 4, resp);
        printf("CFTW: %02x %02x %02x %02x\n", resp[0], resp[1], resp[2],
               resp[3]);

        read(c, 0x05, 2, resp);
        printf("CPOW: %02x %02x\n", resp[0], resp[1]);

        read(c, 0x06, 3, resp);
        printf(" ACR: %02x %02x %02x\n", resp[0], resp[1], resp[2]);

        read(c, 0x07, 2, resp);
        printf("LSRR: %02x %02x\n", resp[0], resp[1]);

        read(c, 0x08, 4, resp);
        printf(" RDW: %02x %02x %02x %02x\n", resp[0], resp[1], resp[2],
               resp[3]);

        read(c, 0x09, 4, resp);
        printf(" FDW: %02x %02x %02x %02x\n", resp[0], resp[1], resp[2],
               resp[3]);
    }
    spi_set_baudrate(spi1, 100 * MHZ);
}