#include "ad9959.h"

void ad9959_default_config() {
    // PLL multiplier of x4
    uint8_t fr1[] = {0x01, 0x90, 0x00, 0x00};
    spi_write_blocking(spi1, fr1, 4);

    // nothing in FR2
    uint8_t fr2[] = {0x02, 0x00, 0x00};
    spi_write_blocking(spi1, fr2, 3);

    uint8_t cfr[] = {0x03, 0x00, 0x03, 0x04};
    uint8_t ftw[] = {0x04, 0x00, 0x00, 0x00, 0x00};
    uint8_t pow[] = {0x05, 0x00, 0x00};
    uint8_t acr[] = {0x06, 0x00, 0x00, 0x00};
    for (int i = 0; i < 4; i++) {
        // select the channel
        uint8_t csr[] = {0x00, (1u << (i + 4)) | 0x2};
        spi_write_blocking(spi1, csr, 2);
        spi_write_blocking(spi1, cfr, 4);
        spi_write_blocking(spi1, ftw, 5);
        spi_write_blocking(spi1, pow, 3);
        spi_write_blocking(spi1, acr, 4);
    }
}

uint32_t send_freq(ad9959_config* c, uint channel, double freq) {
    uint32_t ftw = round(freq * 4294967296.l / c->sys_clk);

    uint8_t buf[5];
    buf[0] = 0x04;

    uint8_t* bytes = (uint8_t*)&ftw;
    for (int i = 0; i < 4; i++) {
        // 1 offset for the register address
        buf[i + 1] = bytes[3 - i];
    }

    uint8_t csr[] = {0x00, (1u << (channel + 4)) | 0x02};
    spi_write_blocking(spi1, csr, 2);
    spi_write_blocking(spi1, buf, 5);

    // for debugging
    return ftw;
}

double send_phase(uint channel, double phase) {
    uint32_t pow = round(phase / 360.0 * 16384.0);

    if (pow > 16383) pow = 16383;

    uint8_t buf[3];
    buf[0] = 0x05;
    buf[1] = (0xff00 & pow) >> 8;
    buf[2] = 0xff & pow;

    uint8_t csr[] = {0x00, (1u << (channel + 4)) | 0x02};
    spi_write_blocking(spi1, csr, 2);
    spi_write_blocking(spi1, buf, 3);

    return pow / 16384.0 * 360.0;
}

double send_amp(uint channel, double amp) {
    uint32_t asf = round(amp * 1024);

    if (asf > 1023) asf = 1023;

    uint8_t buf[4];
    buf[0] = 0x06;
    buf[1] = 0x00;
    buf[2] = ((0x300 & asf) >> 8) | 0x10;
    buf[3] = 0xff & asf;

    uint8_t csr[] = {0x00, (1u << (channel + 4)) | 0x02};
    spi_write_blocking(spi1, csr, 2);
    spi_write_blocking(spi1, buf, 4);

    return asf / 1023.0;
}

void read_reg(uint8_t reg, size_t len, uint8_t* buf) {
    reg |= 0x80;
    spi_write_blocking(spi1, &reg, 1);
    spi_read_blocking(spi1, 0, buf, len);
}

void read_all() {
    spi_set_baudrate(spi1, 1 * MHZ);

    uint8_t resp[20];

    read_reg(0x00, 1, resp);
    printf(" CSR: %02x\n", resp[0]);

    read_reg(0x01, 3, resp);
    printf(" FR1: %02x %02x %02x\n", resp[0], resp[1], resp[2]);

    read_reg(0x02, 2, resp);
    printf(" FR2: %02x %02x\n", resp[0], resp[1]);

    for (int i = 0; i < 4; i++) {
        printf("CHANNEL %d:\n", i);

        uint8_t csr[] = {0x00, (1u << (i + 4)) | 0x02};

        spi_write_blocking(spi1, csr, 2);

        read_reg(0x03, 3, resp);
        printf(" CFR: %02x %02x %02x\n", resp[0], resp[1], resp[2]);

        read_reg(0x04, 4, resp);
        printf("CFTW: %02x %02x %02x %02x\n", resp[0], resp[1], resp[2],
               resp[3]);

        read_reg(0x05, 2, resp);
        printf("CPOW: %02x %02x\n", resp[0], resp[1]);

        read_reg(0x06, 3, resp);
        printf(" ACR: %02x %02x %02x\n", resp[0], resp[1], resp[2]);

        read_reg(0x07, 2, resp);
        printf("LSRR: %02x %02x\n", resp[0], resp[1]);

        read_reg(0x08, 4, resp);
        printf(" RDW: %02x %02x %02x %02x\n", resp[0], resp[1], resp[2],
               resp[3]);

        read_reg(0x09, 4, resp);
        printf(" FDW: %02x %02x %02x %02x\n", resp[0], resp[1], resp[2],
               resp[3]);
    }
    spi_set_baudrate(spi1, 100 * MHZ);
}