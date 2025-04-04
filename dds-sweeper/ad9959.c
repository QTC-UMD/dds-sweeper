#include "ad9959.h"
#include "fast_serial.h"

// =============================================================================
// calculate tuning words
// =============================================================================
double get_asf(double amp, uint16_t* asf) {
    *asf = round(amp * 1024);

    // validation
    if (*asf > 1023) *asf = 1023;
    if (*asf < 0) *asf = 0;

    return *asf / 1023.0;
}

double get_ftw(ad9959_config* c, double freq, uint32_t* ftw) {
    double sys_clk = c->ref_clk * c->pll_mult;
    *ftw = round(freq * 4294967296.l / sys_clk);

    // maybe add some validation here

    // return the frequency that was able to be set
    return *ftw * sys_clk / 4294967296.l;
}

double get_pow(double phase, uint16_t* pow) {
    *pow = round(phase / 360.0 * 16384.0);

    // make sure pow is within range?
    *pow = *pow % 16383;

    return *pow / 16383.0 * 360.0;
}

double get_time(ad9959_config* c, double time, uint32_t* cycles) {
    *cycles = round(time * c->ref_clk);

    // return time that was able to be set
    return *cycles / c->ref_clk;
}

void load_acr(uint16_t asf, uint8_t* buf) {
    buf[0] = 0x00;
    buf[1] = ((0x0300 & asf) >> 8) | 0x10;
    buf[2] = 0xff & asf;
}

void load_ftw(uint32_t ftw, uint8_t* buf) {
    buf[0] = (ftw & 0xFF000000) >> 24;
    buf[1] = (ftw & 0x00FF0000) >> 16;
    buf[2] = (ftw & 0x0000FF00) >> 8;
    buf[3] = (ftw & 0x000000FF);
}

void load_pow(uint16_t pow, uint8_t* buf) {
    buf[0] = (pow & 0xFF00) >> 8;
    buf[1] = pow & 0x00FF;
}

// =============================================================================
// Sending Tuning Words
// =============================================================================
void send_channel(uint8_t reg, uint8_t channel, uint8_t* buf, size_t len) {
    uint8_t csr[] = {0x00, 0x02 | (1u << (channel + 4))};
    spi_write_blocking(SPI, csr, 2);
    spi_write_blocking(SPI, &reg, 1);
    spi_write_blocking(SPI, buf, len);
}

void send(uint8_t reg, uint8_t* buf, size_t len) {
    spi_write_blocking(SPI, &reg, 1);
    spi_write_blocking(SPI, buf, len);
}

// =============================================================================
// Readback
// =============================================================================

void read_reg(uint8_t reg, size_t len, uint8_t* buf) {
    reg |= 0x80;
    spi_write_blocking(SPI, &reg, 1);
    spi_read_blocking(SPI, 0, buf, len);
}

void read_all() {
    spi_set_baudrate(SPI, 1 * MHZ);

    uint8_t resp[20];

    read_reg(0x00, 1, resp);
    fast_serial_printf(" CSR: %02x\n", resp[0]);

    read_reg(0x01, 3, resp);
    fast_serial_printf(" FR1: %02x %02x %02x\n", resp[0], resp[1], resp[2]);

    read_reg(0x02, 2, resp);
    fast_serial_printf(" FR2: %02x %02x\n", resp[0], resp[1]);

    for (int i = 0; i < 4; i++) {
        fast_serial_printf("CHANNEL %d:\n", i);

        uint8_t csr[] = {0x00, (1u << (i + 4)) | 0x02};

        spi_write_blocking(SPI, csr, 2);

        read_reg(0x03, 3, resp);
        fast_serial_printf(" CFR: %02x %02x %02x\n", resp[0], resp[1], resp[2]);

        read_reg(0x04, 4, resp);
        fast_serial_printf("CFTW: %02x %02x %02x %02x\n", resp[0], resp[1], resp[2], resp[3]);

        read_reg(0x05, 2, resp);
        fast_serial_printf("CPOW: %02x %02x\n", resp[0], resp[1]);

        read_reg(0x06, 3, resp);
        fast_serial_printf(" ACR: %02x %02x %02x\n", resp[0], resp[1], resp[2]);

        read_reg(0x07, 2, resp);
        fast_serial_printf("LSRR: %02x %02x\n", resp[0], resp[1]);

        read_reg(0x08, 4, resp);
        fast_serial_printf(" RDW: %02x %02x %02x %02x\n", resp[0], resp[1], resp[2], resp[3]);

        read_reg(0x09, 4, resp);
        fast_serial_printf(" FDW: %02x %02x %02x %02x\n", resp[0], resp[1], resp[2], resp[3]);

        read_reg(0x0a, 4, resp);
        fast_serial_printf(" CW1: %02x %02x %02x %02x\n", resp[0], resp[1], resp[2], resp[3]);
    }
    spi_set_baudrate(SPI, 100 * MHZ);
}

// =============================================================================
// Control
// =============================================================================
void set_pll_mult(ad9959_config* c, uint mult) {
    c->pll_mult = mult;

    uint8_t vco = 0;
    if (mult * c->ref_clk >= 255 * MHZ) {
        vco = 0x80;
    }

    uint8_t fr1[] = {0x01, vco | (mult << 2), 0x00, 0x00};
    spi_write_blocking(SPI, fr1, 4);

    // for (int i = 0; i < 4; i++) {
    //     fast_serial_printf("%02x\n", fr1[i]);
    // }
}

void set_ref_clk(ad9959_config* c, uint64_t freq) { c->ref_clk = freq; }

void single_step_mode() {
    uint8_t csr = 0xf2;
    send(0x00, &csr, 1);
    uint8_t cfr[3] = {0x00, 0x03, 0x00};
    send(0x03, cfr, 3);
}

void clear() {
    uint8_t clear[] = {0x00, 0xf2, 0x03, 0x00, 0x03, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x05,
                       0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x08, 0x00, 0x00,
                       0x00, 0x00, 0x09, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x00,
                       0x02, 0x00, 0x00};

    spi_write_blocking(SPI, clear, sizeof clear);
}

// =============================================================================
// Sweep rate algorithms
// =============================================================================

double approx_double_32(double f, uint32_t * numerator, uint * denominator, uint32_t p_max, uint32_t q_max) {
    // Approximates a double as a ratio of integers 
    // using a modified continued fractions algorithm

    double eps = 0.000000001; // later

    uint32_t a0 = floor(f);
    double r_old = f - a0;
    uint32_t q_double_old = 0;
    uint32_t p_double_old = 1;
    uint32_t p_old = a0;
    uint32_t q_old = 1;

    uint32_t a_new, p_new, q_new;
    double r_new;

    if (p_old == 0) {
        // Returns slowest possible ramp rate
        *numerator = 1;
        *denominator = 255;
        return 1.0/255.0;
    }
    
    uint num_iters = 100; 
    for (uint i = 1; i < num_iters; i++) {

        if (fabs(f - ((double)p_old / (double)q_old)) < eps) {
            // Ratio close to double within eps
            break;
        }
        
        a_new = floor(1/r_old);
        p_new = a_new * p_old + p_double_old;
        q_new = a_new * q_old + q_double_old;

        if (p_new > p_max || q_new > q_max){
            // Either delta or rate out of bounds
            break;
        }

        r_old = 1/r_old - a_new;

        // update all vars
        p_double_old = p_old;
        p_old = p_new;
        q_double_old = q_old;
        q_old = q_new;
        
    }

    *numerator = p_old;
    *denominator = q_old;
    return (double)p_old / (double)q_old;
}

double approx_double_16(double f, uint16_t * numerator, uint * denominator, uint16_t p_max, uint16_t q_max) {
    // Approximates a double as a ratio of integers 
    // using a modified continued fractions algorithm

    double eps = 0.000000001; // later

    uint16_t a0 = floor(f);
    double r_old = f - a0;
    uint16_t q_double_old = 0;
    uint16_t p_double_old = 1;
    uint16_t p_old = a0;
    uint16_t q_old = 1;

    uint16_t a_new, p_new, q_new;
    double r_new;

    if (p_old == 0) {
        // Returns slowest possible ramp rate
        *numerator = 1;
        *denominator = 255;
        return 1.0/255.0;
    }
    
    uint num_iters = 100; 
    for (uint i = 1; i < num_iters; i++) {
        if (fabs(f - ((double)p_old / (double)q_old)) < eps) {
            // Ratio close to double within eps
            break;
        }
        
        a_new = floor(1/r_old);
        p_new = a_new * p_old + p_double_old;
        q_new = a_new * q_old + q_double_old;
        
        if (p_new > p_max || q_new > q_max){
            // Either delta or rate out of bounds
            break;
        }
        
        r_old = 1/r_old - a_new;
        
        // update all vars
        p_double_old = p_old;
        p_old = p_new;
        q_double_old = q_old;
        q_old = q_new;

    }

    *numerator = p_old;
    *denominator = q_old;
    return (double)p_old / (double)q_old;
    
}