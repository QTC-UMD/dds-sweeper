
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

// AD9959 registers
#define AD9959_REG_CSR 0x00
#define AD9959_REG_FR1 0x01
#define AD9959_REG_FR2 0x02
#define AD9959_REG_CFR 0x03
#define AD9959_REG_FTW 0x04
#define AD9959_REG_POW 0x05
#define AD9959_REG_ACR 0x06
#define AD9959_REG_LSRR 0x07
#define AD9959_REG_RDW 0x08
#define AD9959_REG_FDW 0x09
#define AD9959_REG_CW 0x0A

// Instruction memory maps
// Defines offset in instruction array where register address is stored
// Register value starts at offset of address + 1
#define INS_PROFILE 0
#define INS_CSR 1
// For Single Step
#define INS_SS_FTW 3
#define INS_SS_POW 8
#define INS_SS_ACR 11
// For AMP/AMP2 sweep
#define INS_AMP_ACR 3
#define INS_AMP_LSRR 7
#define INS_AMP_RDW 10
#define INS_AMP_FDW 15
#define INS_AMP_CW 20
#define INS_AMP_CFR 25
#define INS_AMP_FTW 29
#define INS_AMP_POW 34
// For FREQ/FREQ2 sweep
#define INS_FREQ_FTW 3
#define INS_FREQ_LSRR 8
#define INS_FREQ_RDW 11
#define INS_FREQ_FDW 16
#define INS_FREQ_CW 21
#define INS_FREQ_CFR 26
#define INS_FREQ_ACR 30
#define INS_FREQ_POW 34
// For PHASE/PHASE2 sweep
#define INS_PHASE_POW 3
#define INS_PHASE_LSRR 6
#define INS_PHASE_RDW 9
#define INS_PHASE_FDW 14
#define INS_PHASE_CW 19
#define INS_PHASE_CFR 24
#define INS_PHASE_FTW 28
#define INS_PHASE_ACR 33

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
