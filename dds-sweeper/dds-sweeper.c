
/*
#######################################################################
#                                                                     #
# dds-sweeper.c                                                       #
#                                                                     #
# Copyright 2023, Ethan Huegler                                       #
#                                                                     #
# Serial communication code based on the PineBlaster and PrawnBlaster #
#   https://github.com/labscript-suite/pineblaster                    #
#   Copyright 2013, Christopher Billington                            #
#   https://github.com/labscript-suite/prawnblaster                   #
#   Copyright 2013, Philip Starkey                                    #
#                                                                     #
# This file is used to flash a Raspberry Pi Pico microcontroller      #
# prototyping board to create a DDS-Sweeper (see readme.txt           #
# This file is licensed under the BSD-2-Clause license.               #
# See the license.txt file for the full license.                      #
#                                                                     #
#######################################################################
*/

#include <string.h>

#include "fast_serial.h"

#include "ad9959.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/flash.h"
#include "hardware/pio.h"
#include "hardware/spi.h"
#include "pico/bootrom.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "trigger_timer.pio.h"

#define VERSION "0.2.1"

// Mutex for status
static mutex_t status_mutex;
static mutex_t wait_mutex;

#define FLASH_TARGET_OFFSET (256 * 1024)

// STATUS flag
#define STOPPED 0
#define RUNNING 1
#define ABORTING 2
int status = STOPPED;

// modes
#define UNDEF_MODE -1
#define SS_MODE 0
#define AMP_MODE 1
#define FREQ_MODE 2
#define PHASE_MODE 3
#define AMP2_MODE 4
#define FREQ2_MODE 5
#define PHASE2_MODE 6

// PIO VALUES IT IS LOOKING FOR
#define UPDATE 0

#define MAX_SIZE 245760
#define TIMERS 5000
#define TIMING_OFFSET (MAX_SIZE - TIMERS * 4)

// minimum wait lengths
#define WAITS_SS_PER 250
#define WAITS_SS_BASE (500 - WAITS_SS_PER)
#define WAITS_SW_PER 500
#define WAITS_SW_BASE (1000 - WAITS_SW_PER)

// For responding OK to successful commands
#define OK() fast_serial_printf("ok\n")

// =============================================================================
// global variables
// =============================================================================
ad9959_config ad9959;
#define READSTRING_SIZE 256
char readstring[READSTRING_SIZE];
bool DEBUG = true;
bool timing = false;

uint triggers;

uint timer_dma;

uint INS_SIZE = 0;
uint8_t instructions[MAX_SIZE];

// bytes to encode an instruction in terms of sweep type
// order is single step, amp, freq, phase, amp2, freq2, phase2
uint BYTES_PER_INS[] = {8, 7, 13, 7, 13, 17, 13};

// =============================================================================
// Utility Functions
// =============================================================================

void init_pin(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
}

void init_pio() {
    pio_clear_instruction_memory(PIO_TRIG);
    uint offset = pio_add_program(PIO_TRIG, &trigger_program);
    uint trigger = timing ? INT_TRIGGER : TRIGGER;
    uint profile_low = PROFILE_ASC ? P0 : P3;
    trigger_program_init(PIO_TRIG, 0, offset, trigger, profile_low, PIN_UPDATE);

    if(timing) {
        pio_clear_instruction_memory(PIO_TIME);
        offset = pio_add_program(PIO_TIME, &timer_program);
        timer_program_init(PIO_TIME, 0, offset, TRIGGER, INT_TRIGGER);
    }
}

int get_status() {
    mutex_enter_blocking(&status_mutex);
    int temp = status;
    mutex_exit(&status_mutex);
    return temp;
}

void set_status(int new_status) {
    mutex_enter_blocking(&status_mutex);
    status = new_status;
    mutex_exit(&status_mutex);
}

void measure_freqs(void) {
    // From https://github.com/raspberrypi/pico-examples under BSD-3-Clause License
    uint f_pll_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
    uint f_pll_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_USB_CLKSRC_PRIMARY);
    uint f_rosc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC);
    uint f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
    uint f_clk_peri = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_PERI);
    uint f_clk_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_USB);
    uint f_clk_adc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_ADC);
    uint f_clk_rtc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_RTC);

    fast_serial_printf("pll_sys = %dkHz\n", f_pll_sys);
    fast_serial_printf("pll_usb = %dkHz\n", f_pll_usb);
    fast_serial_printf("rosc = %dkHz\n", f_rosc);
    fast_serial_printf("clk_sys = %dkHz\n", f_clk_sys);
    fast_serial_printf("clk_peri = %dkHz\n", f_clk_peri);
    fast_serial_printf("clk_usb = %dkHz\n", f_clk_usb);
    fast_serial_printf("clk_adc = %dkHz\n", f_clk_adc);
    fast_serial_printf("clk_rtc = %dkHz\n", f_clk_rtc);
}

void update() { pio_sm_put(PIO_TRIG, 0, UPDATE); }

void sync() {
    gpio_put(PIN_SYNC, 1);
    sleep_ms(1);
    gpio_put(PIN_SYNC, 0);
    sleep_ms(1);
}

void reset() {
    gpio_put(PIN_RESET, 1);
    sleep_ms(1);
    gpio_put(PIN_RESET, 0);
    sleep_ms(1);

    sync();
    ad9959.sweep_type = 1;
    ad9959.channels = 1;
    ad9959.mirror = 0;
    INS_SIZE = 14;

    set_pll_mult(&ad9959, ad9959.pll_mult);

    clear();
    update();
}

void wait(uint channel) {
    pio_sm_get_blocking(PIO_TRIG, 0);
    triggers++;
}

void abort_run() {
    if (get_status() == RUNNING) {
        set_status(ABORTING);

        // Make trigger PIO trigger immediately so background aborts
        pio_sm_exec(PIO_TRIG, 0, pio_encode_push(false, false));

        // reinit PIO
        init_pio();
    }
}

// =============================================================================
// Set Table Instructions
// =============================================================================

void set_time(uint32_t addr, uint32_t time, int sweep_type, uint channels) {
    uint32_t cycles = time;

    if (sweep_type == SS_MODE) {
        // single stepping
        if (cycles < WAITS_SS_BASE + WAITS_SS_PER * channels) {
            cycles = WAITS_SS_BASE + WAITS_SS_PER * channels;
        }
    } else {
        // sweeping
        if (cycles < WAITS_SW_BASE + WAITS_SW_PER * channels) {
            cycles = WAITS_SW_BASE + WAITS_SW_PER * channels;
        }
    }

    if (addr == 0) {
        // the first time through pio takes longer
        cycles -= 18;
    } else {
        cycles -= 10;
    }
    *((uint32_t *)(instructions + TIMING_OFFSET + 4 * addr)) = cycles;
}

/* offset from the beginning of this step to where this channel's instruction goes
*/
uint get_channel_offset(uint channel) {
    return INS_SIZE * channel + 1;
}

int get_offset(uint channel, uint addr) {
    // add one at the end here for this steps profile pin byte
    uint offset = (INS_SIZE * ad9959.channels + 1) * addr;

    // Space needed for timers
    uint tspace = timing ? TIMERS * 4 : 0;

    if (offset + get_channel_offset(channel) + INS_SIZE + tspace >= MAX_SIZE || (timing && addr > TIMERS)) {
        return -1;
    }

    return offset;
}

bool set_repeat_instruction(uint addr){
    int offset = get_offset(0, addr);
    if (offset < 0) {
        return false;
    }
    instructions[offset] = 0x00;
    instructions[offset+1] = 0xff;
    return true;
}

bool set_stop_instruction(uint addr){
    int offset = get_offset(0, addr);
    if (offset < 0) {
        return false;
    }
    instructions[offset] = 0x00;
    instructions[offset+1] = 0x00;
    return true;
}

void set_ins_csr(uint8_t * ins, uint channel){
    // set csr
    ins[INS_CSR] = AD9959_REG_CSR;
    if (ad9959.mirror) {
        ins[INS_CSR+1] = 0xf2;
    } else {
        ins[INS_CSR+1] = (1u << (channel + 4)) | 0x02;
    }
}

void set_ins_sweeps(uint addr, uint channel, bool rising){
    uint offset = get_offset(0, addr);
    // Profile pins are the same for all sweep types
    // Profile pins get updated twice for each trigger
    // the first update puts the profile pin high no matter what
    // then only if it is a downward sweep drop the profile pin low

    // The profile pin directions for a single table step are stored in a single byte
    // The least significant 4 bits in the byte correspond to the first value the
    // profile pin hits during an update. Since the pin should always go high first,
    // that means the least significant nibble should always be 0xf.
    if (rising && ad9959.mirror) {
        // case: upward sweep single channel mode
        instructions[offset] = 0xff;
    } else if (rising) {
        // case: upward sweep on this channel
        if (PROFILE_ASC) {
            instructions[offset] |= (1u << channel) | (1u << (channel + 4));
        }
        else {
            instructions[offset] |= (1u << (3 - channel)) | (1u << (7 - channel));
        }
    } else if (ad9959.mirror) {
        // case: downward sweep single channel mode
        instructions[offset] = 0x0f;
    } else {
        // case: downward sweep on this channel
        if (PROFILE_ASC) {
            instructions[offset] &= ~(1u << (channel + 4));
            instructions[offset] |= 1u << channel;
        }
        else {
            instructions[offset] &= ~(1u << (7 - channel));
            instructions[offset] |= 1u << (3 - channel);
        }
    }
}

/*
  Set single step instruction at offset for channel with
  Frequency Tuning Word
  Phase Offset Word
  Amplitude Control Register
 */
void set_single_step_ins(uint addr, uint channel,
                         uint32_t ftw, uint16_t pow, uint16_t asf){
    uint8_t ins[14];
    set_ins_csr(ins, channel);

    // Memory Map (12 bytes)
    // [ 0x00, CSR                      *Channel Select Register
    //   0x04, FTW3, FTW2, FTW1, FTW0,  *Frequcney Tuning Word
    //   0x05, POW1, POW0,              *Phase Offset Word
    //   0x06, ACR2, ACR1, ACR0         *Amplitude Control Register
    // ]

    ins[INS_SS_FTW] = AD9959_REG_FTW;
    ins[INS_SS_POW] = AD9959_REG_POW;
    ins[INS_SS_ACR] = AD9959_REG_ACR;

    // profile pins do not matter for single tone mode, but the pio program
    // still expects a nonzero value for the profile pin mask
    instructions[get_offset(0, addr)] |= 0x01;

    load_acr(asf, &(ins[INS_SS_ACR+1]));
    load_ftw(ftw, &(ins[INS_SS_FTW+1]));
    load_pow(pow, &(ins[INS_SS_POW+1]));

    memcpy(instructions + get_offset(channel, addr) + get_channel_offset(channel), ins, INS_SIZE);
}

void set_single_step_ins_from_buffer(uint addr, uint channel, char * buffer){
    uint32_t ftw, time;
    uint16_t asf, pow;
    memcpy(&ftw, &(buffer[0]), 4);
    memcpy(&asf, &(buffer[4]), 2);
    memcpy(&pow, &(buffer[6]), 2);
    set_single_step_ins(addr, channel, ftw, pow, asf);
    if (timing) {
        memcpy(&time, &(buffer[8]), 4);
        set_time(addr, time, ad9959.sweep_type, ad9959.channels);
    }
}

/*
  Set amplitude sweep instruction
 */
void set_amp_sweep_ins(uint addr, uint channel, uint16_t asf_start, uint16_t asf_end,
                       uint16_t delta, uint rate, uint32_t ftw, uint16_t pow){
    uint8_t ins[37];
    set_ins_csr(ins, channel);
    set_ins_sweeps(addr, channel, asf_end > asf_start);
    
    // Memory Map
    // [ 0x00, CSR                              *Channel Select Register
    //   0x06, ACR2, ACR1, ACR0,                *Amplitude Control Register
    //   0X07,  FRR,  RRR,                      *Linear Sweep Ramp Rate Register
    //   0x08, RDW3, RDW2, RDW1, RDW1,          *Rising Delta Word Register
    //   0x09, FDW3, FDW2, FDW1, FDW1,          *Falling Delta Word Register
    //   0x0a,  CW3,  CW2,  CW1,  CW0,          *Sweep Endpoint,
    //   0x03, CFR2, CFR1, CFR0                 *Channel Function Register
    //  *0x04, FTW3, FTW2, FTW1, FTW0           *Frequency Tuning Word
    //  *0x05, POW1, POW0                       *Phase Offset Word
    // ]
    ins[INS_AMP_ACR] = AD9959_REG_ACR;
    ins[INS_AMP_LSRR] = AD9959_REG_LSRR;
    ins[INS_AMP_RDW] = AD9959_REG_RDW;
    ins[INS_AMP_FDW] = AD9959_REG_FDW;
    ins[INS_AMP_CW] = AD9959_REG_CW;
    ins[INS_AMP_CFR] = AD9959_REG_CFR;

    if (ad9959.sweep_type == AMP2_MODE) {
        // set freq
        ins[INS_AMP_FTW] = AD9959_REG_FTW;
        load_ftw(ftw, &(ins[INS_AMP_FTW+1]));

        ins[INS_AMP_POW] = AD9959_REG_POW;
        load_pow(pow, &(ins[INS_AMP_POW+1]));
    }

    // bit alignment
    uint32_t rate_word;
    rate_word = ((((uint32_t) delta) & 0x3fc) >> 2) | ((((uint32_t) delta) & 0x3) << 14);

    uint32_t lower;
    uint32_t higher;
    if (asf_end > asf_start) {
        lower = asf_start;
        higher = asf_end;

        ins[INS_AMP_LSRR+1] = 0x01;
        ins[INS_AMP_LSRR+2] = rate;

        memcpy(&(ins[INS_AMP_RDW+1]), &rate_word, 4);
        memcpy(&(ins[INS_AMP_FDW+1]), "\xff\xc0\x00\x00", 4);
    } else {
        lower = asf_end;
        higher = asf_start;

        ins[INS_AMP_LSRR+1] = rate;
        ins[INS_AMP_LSRR+2] = 0x01;

        memcpy(&(ins[INS_AMP_RDW+1]), "\xff\xc0\x00\x00", 4);
        memcpy(&(ins[INS_AMP_FDW+1]), &rate_word, 4);
    }

    // bit alignments
    // the lower point needs to be in the bottom 10 bits of ACR
    lower = ((lower & 0xff) << 16) | (lower & 0xff00);
    memcpy(&(ins[INS_AMP_ACR+1]), (uint8_t *)&lower, 3);
    // higher point goes in the top of CW1
    higher = ((higher & 0x3fc) >> 2) | ((higher & 0x3) << 14);
    memcpy(&(ins[INS_AMP_CW+1]), (uint8_t *)&higher, 4);

    // set CFR for Amplitude Sweep mode with sweep accumulator set to autoclear
    memcpy(&(ins[INS_AMP_CFR+1]), "\x40\x43\x10", 3);

    memcpy(instructions + get_offset(channel, addr) + get_channel_offset(channel), ins, INS_SIZE);
}

void set_amp_sweep_ins_from_buffer(uint addr, uint channel, char * buffer){
    uint32_t ftw, time;
    uint16_t asf_start, asf_end, delta, pow;
    uint8_t rate;
    memcpy(&asf_start, &(buffer[0]), 2);
    memcpy(&asf_end, &(buffer[2]), 2);
    memcpy(&delta, &(buffer[4]), 2);
    rate = buffer[6];
    if (ad9959.sweep_type == AMP2_MODE) {
        memcpy(&ftw, &(buffer[7]), 4);
        memcpy(&pow, &(buffer[11]), 2);
    }
    set_amp_sweep_ins(addr, channel, asf_start, asf_end, delta, rate, ftw, pow);
    if (timing) {
        if (ad9959.sweep_type == AMP2_MODE) {
            memcpy(&time, &(buffer[13]), 4);
        } else {
            memcpy(&time, &(buffer[7]), 4);
        }
        set_time(addr, time, ad9959.sweep_type, ad9959.channels);
    }
}

void parse_amp_sweep_ins(uint addr, uint channel,
                         double start, double end, double sweep_rate, double freq, double phase){
    uint16_t asf_start, asf_end, delta, pow;
    uint32_t ftw;
    uint rate = 1;

    if (ad9959.sweep_type == AMP2_MODE) {
        // Convert others into integer values
        get_ftw(&ad9959, freq, &ftw);
        get_pow(phase, &pow);
    }

    // Convert percentages to integers, check values in range
    start = get_asf(start, &asf_start);
    end = get_asf(end, &asf_end);

    delta = round(sweep_rate * 1024);

    if (delta < 1) {
        if (asf_end > asf_start) {
            // If rising, we can use rate to divide down
            rate = 255;
            delta = round(sweep_rate * 1024 / 255);
            if (delta < 1) {
                delta = 1;
            }
        } else {
            delta = 1;
        }
    } else if (delta > 1023) {
        delta = 1023;
    }

    if (DEBUG) {
        fast_serial_printf(
                           "Set ins #%d for channel %d from %3lf%% to %3lf%% with delta %3lf%% "
                           "and rate of %d\n",
                           addr, channel, start / 10.23, end / 10.23, delta / 10.23, rate);
    }

    set_amp_sweep_ins(addr, channel, asf_start, asf_end, delta, rate, ftw, pow);
}

/*
  Set frequency sweep instruction
 */
void set_freq_sweep_ins(uint addr, uint channel, uint32_t ftw_start, uint32_t ftw_end,
                       uint32_t delta, uint rate, uint16_t asf, uint16_t pow){
    uint8_t ins[37];
    set_ins_csr(ins, channel);
    set_ins_sweeps(addr, channel, ftw_end > ftw_start);

    // Memory Map
    // [ 0x00, CSR                        *Channel Select Register
    //   0x04, FTW3, FTW2, FTW1, FTW0     *Frequency Tuning Word (Start point of sweep)
    //   0X07,  FRR, RRR,                 *Linear Sweep Ramp Rate Register
    //   0x08, RDW3, RDW2, RDW1, RDW1,    *Rising Delta Word Register
    //   0x09, FDW3, FDW2, FDW1, FDW1,    *Falling Delta Word Register
    //   0x0a,  CW3,  CW2,  CW1,  CW0,    *Sweep Endpoint
    //   0x03, CFR2, CFR1, CFR0           *Channel Function Register
    //  *0x06, ACR2, ACR1, ACR0,          *Amplitude Control Register
    //  *0x05, POW1, POW0                 *Phase Offset Word
    // ]
    ins[INS_FREQ_FTW] = AD9959_REG_FTW;
    ins[INS_FREQ_LSRR] = AD9959_REG_LSRR;
    ins[INS_FREQ_LSRR+1] = rate;
    ins[INS_FREQ_LSRR+2] = rate;
    ins[INS_FREQ_RDW] = AD9959_REG_RDW;
    ins[INS_FREQ_FDW] = AD9959_REG_FDW;
    ins[INS_FREQ_CW] = AD9959_REG_CW;
    ins[INS_FREQ_CFR] = AD9959_REG_CFR;

    if (ad9959.sweep_type == FREQ2_MODE) {
        // set amp
        ins[INS_FREQ_ACR] = AD9959_REG_ACR;
        load_acr(asf, &(ins[INS_FREQ_ACR+1]));

        ins[INS_FREQ_POW] = AD9959_REG_POW;
        load_pow(pow, &(ins[INS_FREQ_POW+1]));
    }

    // write instruction
    uint32_t lower, higher;
    if (ftw_start <= ftw_end) {
        // SWEEP UP
        lower = ftw_start;
        higher = ftw_end;

        ins[INS_FREQ_LSRR+1] = 0x01;
        load_ftw(delta, &(ins[INS_FREQ_RDW+1]));
        memcpy(&(ins[INS_FREQ_FDW+1]), "\x00\x00\x00\x00", 4);
    } else {
        // SWEEP DOWN
        ins[INS_FREQ_LSRR+2] = 0x01;
        lower = ftw_end;
        higher = ftw_start;

        memcpy(&(ins[INS_FREQ_RDW+1]), "\xff\xff\xff\xff", 4);
        load_ftw(delta, &(ins[INS_FREQ_FDW+1]));
    }
    // set CFR for Freq Sweep mode with sweep accumulator set to autoclear
    memcpy(&(ins[INS_FREQ_CFR+1]), "\x80\x43\x10", 3);

    load_ftw(lower, &(ins[INS_FREQ_FTW+1]));
    load_ftw(higher, &(ins[INS_FREQ_CW+1]));

    memcpy(instructions + get_offset(channel, addr) + get_channel_offset(channel), ins, INS_SIZE);
}

void set_freq_sweep_ins_from_buffer(uint addr, uint channel, char * buffer){
    uint32_t ftw_start, ftw_end, delta, time;
    uint16_t asf, pow;
    uint8_t rate;
    memcpy(&ftw_start, &(buffer[0]), 4);
    memcpy(&ftw_end, &(buffer[4]), 4);
    memcpy(&delta, &(buffer[8]), 4);
    rate = buffer[12];
    if (ad9959.sweep_type == FREQ2_MODE) {
        memcpy(&asf, &(buffer[13]), 2);
        memcpy(&pow, &(buffer[15]), 2);
    }
    set_freq_sweep_ins(addr, channel, ftw_start, ftw_end, delta, rate, asf, pow);
    if (timing) {
        if (ad9959.sweep_type == FREQ2_MODE) {
            memcpy(&time, &(buffer[17]), 4);
        } else {
            memcpy(&time, &(buffer[13]), 4);
        }
        set_time(addr, time, ad9959.sweep_type, ad9959.channels);
    }
}

void parse_freq_sweep_ins(uint addr, uint channel,
                          double start, double end, double sweep_rate, double amp, double phase){
    uint16_t asf, pow;
    uint32_t ftw_start, ftw_end, delta;
    uint rate = 1;

    if (ad9959.sweep_type == FREQ2_MODE) {
        // Convert others into integer values
        get_asf(amp, &asf);
        get_pow(phase, &pow);
    }

    // Convert percentages to integers, check values in range
    start = get_ftw(&ad9959, start, &ftw_start);
    end = get_ftw(&ad9959, end, &ftw_end);
    sweep_rate = get_ftw(&ad9959, sweep_rate, &delta);

    if (DEBUG) {
        fast_serial_printf(
                           "Set ins #%d for channel %d from %4lf Hz to %4lf Hz with delta %4lf "
                           "Hz and rate of %d\n",
                           addr, channel, start, end, sweep_rate, rate);
    }

    set_freq_sweep_ins(addr, channel, ftw_start, ftw_end, delta, rate, asf, pow);
}

/*
  Set phase sweep instruction
 */
void set_phase_sweep_ins(uint addr, uint channel, uint16_t pow_start, uint16_t pow_end,
                         uint16_t delta, uint rate, uint32_t ftw, uint16_t asf){
    uint8_t ins[37];
    set_ins_csr(ins, channel);
    set_ins_sweeps(addr, channel, pow_end > pow_start);
    // PHASE Sweep
    // Memory Map
    // [ 0x00, CSR                          *Channel Select Register
    //   0x05, POW1, POW0                   *Phase Offset Word (Start point of sweep)
    //   0X07,  FRR, RRR,                   *Linear Sweep Ramp Rate Register
    //   0x08, RDW3, RDW2, RDW1, RDW1,      *Rising Delta Word Register
    //   0x09, FDW3, FDW2, FDW1, FDW1,      *Falling Delta Word Register
    //   0x0a,  CW3,  CW2,  CW1,  CW0,      *Sweep Endpoint
    //   0x03, CFR2, CFR1, CFR0             *Channel Function Register
    //  *0x04, FTW3, FTW2, FTW1, FTW0       *Frequency Tuning Word
    //  *0x06, ACR2, ACR1, ACR0,            *Amplitude Control Register
    // ]
    ins[INS_PHASE_POW] = AD9959_REG_POW;
    ins[INS_PHASE_LSRR] = AD9959_REG_LSRR;
    ins[INS_PHASE_LSRR+1] = rate;
    ins[INS_PHASE_LSRR+2] = rate;
    ins[INS_PHASE_RDW] = AD9959_REG_RDW;
    ins[INS_PHASE_FDW] = AD9959_REG_FDW;
    ins[INS_PHASE_CW] = AD9959_REG_CW;
    ins[INS_PHASE_CFR] = AD9959_REG_CFR;

    if (ad9959.sweep_type == PHASE2_MODE) {
        // set amp
        ins[INS_PHASE_FTW] = AD9959_REG_FTW;
        load_ftw(ftw, &(ins[INS_PHASE_FTW+1]));

        ins[INS_PHASE_ACR] = AD9959_REG_ACR;
        load_acr(asf, &(ins[INS_PHASE_ACR+1]));
    }

    // bit shifting to flip endianness
    uint32_t rate_word;
    rate_word = ((((uint32_t)delta) & 0x3fc0) >> 6) | ((((uint32_t)delta) & 0x3f) << 10);

    uint32_t lower, higher;
    if (pow_start <= pow_end) {
        // sweep up
        ins[INS_PHASE_LSRR+1] = 0x01;
        lower = (uint32_t) pow_start;
        higher = (uint32_t) pow_end;

        memcpy(&(ins[INS_PHASE_RDW+1]), (uint8_t *) &rate_word, 4);
        memcpy(&(ins[INS_PHASE_FDW+1]), "\x00\x00\x00\x00", 4);
    } else {
        // sweep down
        ins[INS_PHASE_LSRR+2] = 0x01;
        lower = (uint32_t) pow_end;
        higher = (uint32_t) pow_start;

        memcpy(&(ins[INS_PHASE_RDW+1]), "\xff\xff\xff\xff", 4);
        memcpy(&(ins[INS_PHASE_FDW+1]), (uint8_t *) &rate_word, 4);
    }

    lower = ((lower & 0xff) << 8) | ((lower & 0xff00) >> 8);
    higher = ((higher & 0x3fc0) >> 6) | ((higher & 0x3f) << 10);

    memcpy(&(ins[INS_PHASE_POW+1]), (uint8_t *) &lower, 2);
    memcpy(&(ins[INS_PHASE_CW+1]), (uint8_t *) &higher, 4);
    memcpy(&(ins[INS_PHASE_CFR+1]), "\xc0\x43\x10", 3);

    memcpy(instructions + get_offset(channel, addr) + get_channel_offset(channel), ins, INS_SIZE);
}

void set_phase_sweep_ins_from_buffer(uint addr, uint channel, char * buffer){
    uint32_t ftw, time;
    uint16_t asf, pow_start, pow_end, delta;
    uint8_t rate;
    memcpy(&pow_start, &(buffer[0]), 2);
    memcpy(&pow_end, &(buffer[2]), 2);
    memcpy(&delta, &(buffer[4]), 2);
    rate = buffer[6];
    if (ad9959.sweep_type == PHASE2_MODE){
        memcpy(&ftw, &(buffer[7]), 4);
        memcpy(&asf, &(buffer[11]), 2);
    }
    set_phase_sweep_ins(addr, channel, pow_start, pow_end, delta, rate, ftw, asf);
    if (timing) {
        if (ad9959.sweep_type == PHASE2_MODE) {
            memcpy(&time, &(buffer[13]), 4);
        } else {
            memcpy(&time, &(buffer[7]), 4);
        }
        set_time(addr, time, ad9959.sweep_type, ad9959.channels);
    }
}

void parse_phase_sweep_ins(uint addr, uint channel,
                           double start, double end, double sweep_rate, double freq, double amp){
    uint16_t asf, pow_start, pow_end, delta;
    uint32_t ftw;
    uint rate = 1;

    if (ad9959.sweep_type == PHASE2_MODE) {
        // Convert others into integer values
        get_ftw(&ad9959, freq, &ftw);
        get_asf(amp, &asf);
    }

    // Convert from degrees to tuning words
    start = get_pow(start, &pow_start);
    end = get_pow(end, &pow_end);
    sweep_rate = get_pow(sweep_rate, &delta);

    if (DEBUG) {
        fast_serial_printf(
            "Set ins #%d for channel %d from %4lf deg to %4lf deg with delta "
            "%4lf deg and rate of %d\n",
            addr, channel, pow_start / 16384.0 * 360, pow_end / 16384.0 * 360, delta / 16384.0 * 360,
            rate);
    }

    set_phase_sweep_ins(addr, channel, pow_start, pow_end, delta, rate, ftw, asf);
}

// =============================================================================
// Table Running Loop
// =============================================================================

void background() {
    // let other core know ready
    multicore_fifo_push_blocking(0);

    int hwstart = 0;
    while (true) {
        // wait for a start command
        hwstart = multicore_fifo_pop_blocking();

        set_status(RUNNING);

        // pre-calculate spacing vars
        uint step = INS_SIZE * ad9959.channels + 1;
        uint offset = 0;

        // count instructions to run
        bool repeat = false;
        int num_ins = 0;
        int i = 0;
        while (true) {
            // If an instruction is empty that means to stop
            if (instructions[offset] == 0x00) {
                if (instructions[offset + 1]) {
                    repeat = true;
                }
                break;
            }
            offset = step * ++i;
        }

        num_ins = i;
        offset = i = 0;
        triggers = 0;

        // sync just to be sure
        sync();

        // if this is hwstart, stell the timer pio core and it will handle that on its own
        if (hwstart) {
            pio_sm_put(PIO_TIME, 0, 0);
        }

        while (status != ABORTING) {
            // check if last instruction
            if (i == num_ins) {
                if (repeat) {
                    i = offset = 0;
                } else {
                    break;
                }
            }

            // prime PIO
            pio_sm_put(PIO_TRIG, 0, instructions[offset]);

            // send new instruciton to AD9959
            spi_write_blocking(SPI, instructions + offset + 1, step - 1);

            // if on the first instruction, begin the timer
            if (i == 0 && timing) {
                dma_channel_transfer_from_buffer_now(timer_dma, instructions + TIMING_OFFSET,
                                                     num_ins);
            }

            wait(0);

            offset = step * ++i;
        }

        // clean up
        dma_channel_abort(timer_dma);
        pio_sm_clear_fifos(PIO_TRIG, 0);
        pio_sm_clear_fifos(PIO_TIME, 0);
        set_status(STOPPED);
    }
}

// =============================================================================
// Serial Communication Loop
// =============================================================================

void loop() {
    bzero(readstring, 256);
    fast_serial_read_until(readstring, 256, '\n');
    int local_status = get_status();

    if (strncmp(readstring, "version", 7) == 0) {
        fast_serial_printf("%s\n", VERSION);
    } else if (strncmp(readstring, "status", 6) == 0) {
        fast_serial_printf("%d\n", local_status);
    } else if (strncmp(readstring, "debug on", 8) == 0) {
        DEBUG = 1;
        OK();
    } else if (strncmp(readstring, "debug off", 9) == 0) {
        DEBUG = 0;
        OK();
    } else if (strncmp(readstring, "getfreqs", 8) == 0) {
        measure_freqs();
		OK();
    } else if (strncmp(readstring, "numtriggers", 11) == 0) {
        fast_serial_printf("%u\n", triggers);
    } else if (strncmp(readstring, "reset", 5) == 0) {
        abort_run();
        reset();
        set_status(STOPPED);
        OK();
    } else if (strncmp(readstring, "abort", 5) == 0) {
        abort_run();
        OK();
    }
    // ====================================================
    // Stuff that cannot be done while the table is running
    // ====================================================
    else if (local_status != STOPPED) {
        fast_serial_printf(
            "Cannot execute command \"%s\" during buffered execution. Check "
            "status first and wait for it to return %d (stopped or aborted).\n",
            readstring, STOPPED);
    } else if (strncmp(readstring, "readregs", 8) == 0) {
        single_step_mode();
        update();
        read_all();
        OK();
    } else if (strncmp(readstring, "load", 4) == 0) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-overread"
        memcpy(instructions, ((uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET)), MAX_SIZE);
#pragma GCC diagnostic pop

        OK();
    } else if (strncmp(readstring, "save", 4) == 0) {
        uint32_t ints = save_and_disable_interrupts();
        // erase sections
        flash_range_erase(FLASH_TARGET_OFFSET, MAX_SIZE);
        // reprogram
        flash_range_program(FLASH_TARGET_OFFSET, instructions, MAX_SIZE);
        restore_interrupts(ints);
        OK();
    } else if (strncmp(readstring, "setchannels", 11) == 0) {
        uint channels;

        int parsed = sscanf(readstring, "%*s %u", &channels);

        if (parsed < 1) {
            fast_serial_printf("Missing Argument - expected: setchannels <num:int>\n");
        } else if (channels < 0 || channels > 4) {
            fast_serial_printf("Invalid Channels - expected: num must be in range 0-4\n");
        } else {
            if (channels == 0) {
                // mirror commands to all channels
                ad9959.channels = 1;
                ad9959.mirror = 1;
            } else {
                ad9959.channels = channels;
                ad9959.mirror = 0;
            }
            OK();
        }
    } else if (strncmp(readstring, "setfreq", 7) == 0) {
        // setfreq <channel:int> <frequency:float>

        uint channel;
        double freq;
        int parsed = sscanf(readstring, "%*s %u %lf", &channel, &freq);
        if (parsed < 2) {
            fast_serial_printf(
                "Missing Argument - too few arguments - expected: setfreq "
                "<channel:int> <frequency:double>\n");
        } else if (channel < 0 || channel > 3) {
            fast_serial_printf("Invalid Channel - num must be in range 0-3\n");
        } else {
            uint32_t ftw;
            freq = get_ftw(&ad9959, freq, &ftw);
            uint8_t ftw_buf[4];
            load_ftw(ftw, ftw_buf);
            send_channel(0x04, channel, ftw_buf, 4);
            update();

            if (DEBUG) {
                fast_serial_printf("set freq: %lf\n", freq);
            }

            OK();
        }
    } else if (strncmp(readstring, "setphase", 8) == 0) {
        // setphase <channel:int> <phase:float>

        uint channel;
        double phase;
        int parsed = sscanf(readstring, "%*s %u %lf", &channel, &phase);
        if (parsed < 2) {
            fast_serial_printf(
                "Missing Argument - too few arguments - expected: setphase "
                "<channel:int> <frequency:double>\n");
        } else if (channel < 0 || channel > 3) {
            fast_serial_printf("Invalid Channel - channel must be in range 0-3\n");
        } else {
            uint16_t pow;
            phase = get_pow(phase, &pow);
            uint8_t pow_buf[2];
            load_pow(pow, pow_buf);
            send_channel(0x05, channel, pow_buf, 2);
            update();

            if (DEBUG) {
                fast_serial_printf("Phase: %12lf\n", phase);
            }

            OK();
        }
    } else if (strncmp(readstring, "setamp", 6) == 0) {
        // setamp <channel:int> <amp:float>

        uint channel;
        double amp;
        int parsed = sscanf(readstring, "%*s %u %lf", &channel, &amp);
        if (parsed < 2) {
            fast_serial_printf(
                "Missing Argument - expected: setamp <channel:int> "
                "<amp:double>\n");
        } else if (channel < 0 || channel > 3) {
            fast_serial_printf("Invalid Channel - channel must be in range 0-3\n");
        } else {
            uint16_t asf;
            amp = get_asf(amp, &asf);
            uint8_t acr[3];
            load_acr(asf, acr);
            send_channel(0x06, channel, acr, 3);
            update();

            if (DEBUG) {
                fast_serial_printf("Amp: %12lf\n", amp);
            }

            OK();
        }
    } else if (strncmp(readstring, "setmult", 7) == 0) {
        uint mult;

        int parsed = sscanf(readstring, "%*s %u", &mult);

        if (parsed < 1) {
            fast_serial_printf("Missing Argument - expected: setmult <pll_mult:int>\n");
        } else if (mult != 1 && !(mult >= 4 && mult <= 20)) {
            fast_serial_printf("Invalid Multiplier: multiplier must be 1 or in range 4-20\n");
        } else {
            // could do more validation to make sure it is a valid
            // multiply/system clock freq
            set_pll_mult(&ad9959, mult);
            update();

            OK();
        }
    } else if (strncmp(readstring, "setclock", 8) == 0) {
        uint src;   // 0 = internal, 1 = external
        uint freq;  // in Hz (up to 133 MHz)
        int parsed = sscanf(readstring, "%*s %u %u", &src, &freq);
        if (parsed < 2) {
            fast_serial_printf("Missing Argument - expected: setclock <mode:int> <freq:int>\n");
        } else {
            if (src > 1) {
                fast_serial_printf("Invalid Mode - mode must be in range 0-1\n");
            } else {
                // Set new clock frequency
                if (src == 0) {
                    if (set_sys_clock_khz(freq / 1000, false)) {
                        set_ref_clk(&ad9959, freq);
                        clock_configure(clk_peri, 0,
                                        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, 125 * MHZ,
                                        125 * MHZ);
                        clock_gpio_init(PIN_CLOCK, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, 1);
                        OK();
                    } else {
                        fast_serial_printf("Failure. Cannot exactly achieve that clock frequency.\n");
                    }
                } else {
                    set_ref_clk(&ad9959, freq);
                    gpio_deinit(PIN_CLOCK);
                    if (DEBUG) fast_serial_printf("AD9959 requires external reference clock\n");
                    OK();
                }
            }
        }
    } else if (strncmp(readstring, "mode", 4) == 0) {
        // mode <type:int> <timing:int>

        uint type, _timing;
        int parsed = sscanf(readstring, "%*s %u %u", &type, &_timing);

        if (parsed < 2) {
            fast_serial_printf("Missing Argument - expected: mode <type:int> <timing:int>\n");
        } else if (type > PHASE2_MODE) {
            fast_serial_printf("Invalid Type - table type must be in range 0-6\n");
        } else {
            uint8_t sizes[] = {14, 28, 29, 27, 36, 36, 36};
            INS_SIZE = sizes[type];
            ad9959.sweep_type = type;
            timing = _timing;

            if (ad9959.sweep_type == SS_MODE) {
                single_step_mode();
                update();
            }

            // Re-initialize PIO in case timing has changed.
            init_pio();

            OK();
        }
    } else if (strncmp(readstring, "set ", 4) == 0) {
        if (ad9959.sweep_type == SS_MODE) {
            // SINGLE TONE MODE
            uint32_t channel, addr, time;
            double freq, amp, phase;
            int parsed = sscanf(readstring, "%*s %u %u %lf %lf %lf %u", &channel, &addr, &freq,
                                &amp, &phase, &time);

            if (parsed == 1) {
                fast_serial_printf("Missing Argument - expected: set <channel:int> ... \n");
            } else if (channel > 5) {
                fast_serial_printf(
                    "Invalid Channel - expected 0-3 for channels or 4/5 for stop/repeat "
                    "instruction\n");
            } else if (channel > 3 && parsed < 2) {
                fast_serial_printf("Missing Argument - expected: set <channel:int> <addr:int> \n");
            } else if (channel < 4 && ((timing && parsed < 6) || (!timing && parsed < 5))) {
                fast_serial_printf(
                    "Missing Argument - expected: set <channel:int> <addr:int> <frequency:double> "
                    "<amplitude:double> <phase:double> (<time:int>)\n");
            } else {
                // At this point, parsing tests have been passed
                if (channel == 4) {
                    if (!set_stop_instruction(addr)) {
                        fast_serial_printf("Insufficient space for stop instruction\n");
                    }
                } else if (channel == 5) {
                    if (!set_repeat_instruction(addr)) {
                        fast_serial_printf("Insufficient space for repeat instruction\n");
                    }
                } else {
                    int ins_offset = get_offset(channel, addr);
                    if(ins_offset < 0) {
                        fast_serial_printf("Insufficient space for instruction\n");
                    } else {
                        uint16_t asf, pow;
                        uint32_t ftw;
                        amp = get_asf(amp, &asf);
                        freq = get_ftw(&ad9959, freq, &ftw);
                        phase = get_pow(phase, &pow);

                        if (DEBUG) {
                            fast_serial_printf(
                                "Set ins #%d for channel %d with amp: %3lf %% "
                                "freq: %3lf Hz phase: %3lf deg\n",
                                addr, channel, amp, freq, phase);
                        }

                        set_single_step_ins(addr, channel, ftw, pow, asf);
                        if (timing) {
                            set_time(addr, time, ad9959.sweep_type, ad9959.channels);
                        }
                    }
                }
            }

            OK();
        } else {
            // Sweep mode
            uint32_t channel, addr, time;
            double start, end, sweep_rate, other1, other2;
            int parsed = 0;

            bool sweep_step_mode = ad9959.sweep_type == AMP2_MODE
                || ad9959.sweep_type == FREQ2_MODE
                || ad9959.sweep_type == PHASE2_MODE;

            if (sweep_step_mode) {
                parsed = sscanf(readstring, "%*s %u %u %lf %lf %lf %lf %lf %u", &channel, &addr, &start,
                                &end, &sweep_rate, &other1, &other2, &time);
            } else {
                parsed = sscanf(readstring, "%*s %u %u %lf %lf %lf %u", &channel, &addr, &start,
                                &end, &sweep_rate, &time);
            }

            if (parsed == 1) {
                fast_serial_printf("Missing Argument - expected: set <channel:int> ... \n");
            } else if (channel > 5) {
                fast_serial_printf(
                    "Invalid Channel - expected 0-3 for channels or 4/5 for stop/repeat "
                    "instruction\n");
            } else if (channel > 3 && parsed < 2) {
                fast_serial_printf("Missing Argument - expected: set <channel:int> <addr:int> \n");
            } else if (!timing && !sweep_step_mode && channel < 4 && parsed < 5) {
                fast_serial_printf(
                    "Missing Argument - expected: set <channel:int> <addr:int> "
                    "<start_point:double> <end_point:double> <sweep rate:double>\n");
            } else if (timing && !sweep_step_mode && channel < 4 && parsed < 6) {
                fast_serial_printf(
                    "Missing Argument - expected: set <channel:int> <addr:int> "
                    "<start_point:double> <end_point:double> <sweep rate:double> "
                    "<time:int>\n");
            } else if (!timing && sweep_step_mode && channel < 4 && parsed < 7) {
                fast_serial_printf(
                    "Missing Argument - expected: set <channel:int> <addr:int> "
                    "<start_point:double> <end_point:double> "
                    "<sweep rate:double> <other1> <other2>\n");
            } else if (timing && sweep_step_mode && channel < 4 && parsed < 8) {
                fast_serial_printf(
                    "Missing Argument - expected: set <channel:int> <addr:int> "
                    "<start_point:double> <end_point:double> "
                    "<sweep rate:double> <other1> <other2> <time:int>\n");
            } else {
                // At this point, parsing tests have been passed

                if (channel == 4) {
                    if (!set_stop_instruction(addr)) {
                        fast_serial_printf("Insufficient space for stop instruction\n");
                    }
                } else if (channel == 5) {
                    if (!set_repeat_instruction(addr)) {
                        fast_serial_printf("Insufficient space for repeat instruction\n");
                    }
                } else {
                    int ins_offset = get_offset(channel, addr);
                    if(ins_offset < 0) {
                        fast_serial_printf("Insufficient space for instruction\n");
                    } else {
                        if (ad9959.sweep_type == AMP_MODE || ad9959.sweep_type == AMP2_MODE) {
                            parse_amp_sweep_ins(addr, channel,
                                                start, end, sweep_rate, other1, other2);
                        } else if (ad9959.sweep_type == FREQ_MODE || ad9959.sweep_type == FREQ2_MODE) {
                            parse_freq_sweep_ins(addr, channel,
                                                 start, end, sweep_rate, other1, other2);
                        } else if (ad9959.sweep_type == PHASE_MODE || ad9959.sweep_type == PHASE2_MODE) {
                            parse_phase_sweep_ins(addr, channel,
                                                  start, end, sweep_rate, other1, other2);
                        } else {
                            fast_serial_printf(
                                "Invalid Command - \'mode\' must be defined before "
                                "instructions can be set\n");
                        }
                        if (timing) {
                            set_time(addr, time, ad9959.sweep_type, ad9959.channels);
                        }
                    }
                }
            }

            OK();
        } 
    } else if (strncmp(readstring, "seti ", 5) == 0) {
        // Set instructions from integers
        if (ad9959.sweep_type == SS_MODE) {
            // SINGLE TONE MODE
            uint32_t channel, addr, time, ftw;
            uint16_t asf, pow;
            int parsed = sscanf(readstring, "%*s %u %u %u %u %u %u", &channel, &addr, &ftw,
                                &asf, &pow, &time);

            if (parsed == 1) {
                fast_serial_printf("Missing Argument - expected: set <channel:int> ... \n");
            } else if (channel > 5) {
                fast_serial_printf(
                    "Invalid Channel - expected 0-3 for channels or 4/5 for stop/repeat "
                    "instruction\n");
            } else if (channel > 3 && parsed < 2) {
                fast_serial_printf("Missing Argument - expected: set <channel:int> <addr:int> \n");
            } else if (channel < 4 && ((timing && parsed < 6) || (!timing && parsed < 5))) {
                fast_serial_printf(
                    "Missing Argument - expected: set <channel:int> <addr:int> <ftw:int32> "
                    "<asf:int16> <pow:int16> (<time:int>)\n");
            } else {
                // At this point, parsing tests have been passed
                if (channel == 4) {
                    if (!set_stop_instruction(addr)) {
                        fast_serial_printf("Insufficient space for stop instruction\n");
                    }
                } else if (channel == 5) {
                    if (!set_repeat_instruction(addr)) {
                        fast_serial_printf("Insufficient space for repeat instruction\n");
                    }
                } else {
                    int ins_offset = get_offset(channel, addr);
                    if(ins_offset < 0) {
                        fast_serial_printf("Insufficient space for instruction\n");
                    } else {
                        if (DEBUG) {
                            fast_serial_printf(
                                "Set ins #%d for channel %d with asf: %d "
                                "ftw: %d pow: %d\n",
                                addr, channel, asf, ftw, pow);
                        }

                        set_single_step_ins(addr, channel, ftw, pow, asf);
                        if (timing) {
                            set_time(addr, time, ad9959.sweep_type, ad9959.channels);
                        }
                    }
                }
            }

            OK();
        } else {
            // Sweep mode
            uint32_t channel, addr, time, start, end, delta, rate, other1, other2;
            int parsed = 0;

            bool sweep_step_mode = ad9959.sweep_type == AMP2_MODE
                || ad9959.sweep_type == FREQ2_MODE
                || ad9959.sweep_type == PHASE2_MODE;

            if (sweep_step_mode) {
                parsed = sscanf(readstring, "%*s %u %u %u %u %u %u %u %u %u", &channel, &addr, &start,
                                &end, &delta, &rate, &other1, &other2, &time);
            } else {
                parsed = sscanf(readstring, "%*s %u %u %u %u %u %u %u", &channel, &addr, &start,
                                &end, &delta, &rate, &time);
            }
            fast_serial_printf("%d\n", parsed);

            if (parsed == 1) {
                fast_serial_printf("Missing Argument - expected: set <channel:int> ... \n");
            } else if (channel > 5) {
                fast_serial_printf(
                    "Invalid Channel - expected 0-3 for channels or 4/5 for stop/repeat "
                    "instruction\n");
            } else if (channel > 3 && parsed < 2) {
                fast_serial_printf("Missing Argument - expected: set <channel:int> <addr:int> \n");
            } else if (!timing && !sweep_step_mode && channel < 4 && parsed < 6) {
                fast_serial_printf(
                    "Missing Argument - expected: set <channel:int> <addr:int> "
                    "<start_point:int> <end_point:int> <delta:int> <rate:int>\n");
            } else if (timing && !sweep_step_mode && channel < 4 && parsed < 7) {
                fast_serial_printf(
                    "Missing Argument - expected: set <channel:int> <addr:int> "
                    "<start_point:int> <end_point:int> <delta:int> <rate:int> "
                    "<time:int>\n");
            } else if (!timing && sweep_step_mode && channel < 4 && parsed < 8) {
                fast_serial_printf(
                    "Missing Argument - expected: set <channel:int> <addr:int> "
                    "<start_point:int> <end_point:int> <delta:int> "
                    "<rate:double> <other1:int> <other2:int>\n");
            } else if (timing && sweep_step_mode && channel < 4 && parsed < 9) {
                fast_serial_printf(
                    "Missing Argument - expected: set <channel:int> <addr:int> "
                    "<start_point:int> <end_point:int> <delta:int> "
                    "<rate:int> <other1:int> <other2:int> <time:int>\n");
            } else {
                // At this point, parsing tests have been passed

                if (channel == 4) {
                    if (!set_stop_instruction(addr)) {
                        fast_serial_printf("Insufficient space for stop instruction\n");
                    }
                } else if (channel == 5) {
                    if (!set_repeat_instruction(addr)) {
                        fast_serial_printf("Insufficient space for repeat instruction\n");
                    }
                } else {
                    int ins_offset = get_offset(channel, addr);
                    if(ins_offset < 0) {
                        fast_serial_printf("Insufficient space for instruction\n");
                    } else {
                        if (ad9959.sweep_type == AMP_MODE || ad9959.sweep_type == AMP2_MODE) {
                            set_amp_sweep_ins(addr, channel,
                                              start, end, delta, rate, other1, other2);
                        } else if (ad9959.sweep_type == FREQ_MODE || ad9959.sweep_type == FREQ2_MODE) {
                            set_freq_sweep_ins(addr, channel,
                                               start, end, delta, rate, other1, other2);
                        } else if (ad9959.sweep_type == PHASE_MODE || ad9959.sweep_type == PHASE2_MODE) {
                            set_phase_sweep_ins(addr, channel,
                                                start, end, delta, rate, other1, other2);
                        } else {
                            fast_serial_printf(
                                "Invalid Command - \'mode\' must be defined before "
                                "instructions can be set\n");
                        }
                        if (timing) {
                            set_time(addr, time, ad9959.sweep_type, ad9959.channels);
                        }
                    }
                }
            }

            OK();
        }
    } else if (strncmp(readstring, "setb ", 5) == 0) {
        // Set instructions, bulk/binary
        uint32_t start_addr, ins_count;
        int parsed = sscanf(readstring, "%*s %u %u", &start_addr, &ins_count);

        if (parsed < 2) {
            fast_serial_printf("Missing Argument - expected: setb <start addr:int> <instruction count:int>\n");
        } else if (get_offset(0, start_addr + ins_count) < 0) {
            fast_serial_printf("Insufficient space for instructions\n");
        } else {

            uint bytes_per_ins = BYTES_PER_INS[ad9959.sweep_type];

            if (bytes_per_ins > 0) {
                if (timing) {
                    bytes_per_ins += 4;
                }

                uint32_t ins_per_buffer = READSTRING_SIZE / (bytes_per_ins * ad9959.channels);
                // Data will fit, mode is valid, tell the computer to send it
                fast_serial_printf("ready for %d bytes\n",
                                   bytes_per_ins * ad9959.channels * ins_count);

                uint32_t addr = start_addr;

                // In this loop, we read nearly full serial buffers and load them.
                while (ins_count > ins_per_buffer) {
                    fast_serial_read(readstring, ins_per_buffer*bytes_per_ins*ad9959.channels);

                    for (int i = 0; i < ins_per_buffer; i++) {
                        for(int j = 0; j < ad9959.channels; j++) {
                            uint byte_offset = bytes_per_ins*(i*ad9959.channels + j);
                            if (ad9959.sweep_type == SS_MODE) {
                                set_single_step_ins_from_buffer(addr, j,
                                                                &(readstring[byte_offset]));
                            } else if (ad9959.sweep_type == AMP_MODE || ad9959.sweep_type == AMP2_MODE) {
                                set_amp_sweep_ins_from_buffer(addr, j,
                                                              &(readstring[byte_offset]));
                            } else if (ad9959.sweep_type == FREQ_MODE || ad9959.sweep_type == FREQ2_MODE) {
                                set_freq_sweep_ins_from_buffer(addr, j,
                                                               &(readstring[byte_offset]));
                            } else if (ad9959.sweep_type == PHASE_MODE || ad9959.sweep_type == PHASE2_MODE) {
                                set_phase_sweep_ins_from_buffer(addr, j,
                                                                &(readstring[byte_offset]));
                            }
                        }
                        addr++;
                    }
                    ins_count -= ins_per_buffer;
                }

                // In this if statement, we read a final serial buffer and load it.
                if(ins_count > 0){
                    fast_serial_read(readstring, ins_count*bytes_per_ins*ad9959.channels);

                    for (int i = 0; i < ins_per_buffer; i++) {
                        for(int j = 0; j < ad9959.channels; j++) {
                            uint byte_offset = bytes_per_ins*(i*ad9959.channels + j);
                            if (ad9959.sweep_type == SS_MODE) {
                                set_single_step_ins_from_buffer(addr, j,
                                                                &(readstring[byte_offset]));
                            } else if (ad9959.sweep_type == AMP_MODE || ad9959.sweep_type == AMP2_MODE) {
                                set_amp_sweep_ins_from_buffer(addr, j,
                                                              &(readstring[byte_offset]));
                            } else if (ad9959.sweep_type == FREQ_MODE || ad9959.sweep_type == FREQ2_MODE) {
                                set_freq_sweep_ins_from_buffer(addr, j,
                                                               &(readstring[byte_offset]));
                            } else if (ad9959.sweep_type == PHASE_MODE || ad9959.sweep_type == PHASE2_MODE) {
                                set_phase_sweep_ins_from_buffer(addr, j,
                                                                &(readstring[byte_offset]));
                            }
                        }
                        addr++;
                    }
                } 

                OK();
            }
        }
    } else if (strncmp(readstring, "start", 5) == 0) {
        if (ad9959.sweep_type == UNDEF_MODE) {
            fast_serial_printf(
                "Invalid Command - \'mode\' must be defined before "
                "a table can be started\n");
        } else {
            pio_sm_clear_fifos(PIO_TRIG, 0);
            pio_sm_clear_fifos(PIO_TIME, 0);

            // start the other core
            multicore_fifo_push_blocking(0);
            OK();
        }
    } else if (strncmp(readstring, "hwstart", 7) == 0) {
        if (ad9959.sweep_type == UNDEF_MODE) {
            fast_serial_printf(
                "Invalid Command - \'mode\' must be defined before "
                "a table can be started\n");
        } else {
            pio_sm_clear_fifos(PIO_TRIG, 0);
            pio_sm_clear_fifos(PIO_TIME, 0);

            // start the other core
            multicore_fifo_push_blocking(1);
            OK();
        }
    } else if (strncmp(readstring, "program", 7) == 0) {
        reset_usb_boot(0, 0);
    } else {
        fast_serial_printf("Unrecognized Command: \"%s\"\n", readstring);
    }
}

// =============================================================================
// Initial Setup
// =============================================================================

int main() {
    init_pin(PICO_DEFAULT_LED_PIN);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);

    fast_serial_init();

    set_sys_clock_khz(125 * MHZ / 1000, false);

    // output sys clock on a gpio pin to be used as REF_CLK for AD9959
    clock_gpio_init(PIN_CLOCK, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, 1);

    // attatch spi to system clock so it runs at max rate
    clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, 125 * MHZ,
                    125 * MHZ);

    // init SPI
    spi_init(SPI, 100 * MHZ);
    spi_set_format(SPI, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // launch other core
    multicore_launch_core1(background);
    multicore_fifo_pop_blocking();

    // initialise the status mutex
    mutex_init(&status_mutex);
    mutex_init(&wait_mutex);

    // init the PIO
    init_pio();

    // setup dma
    timer_dma = dma_claim_unused_channel(true);

    // if pico is timing itself, it will use dma to send all the wait
    // lengths to the timer pio program
    dma_channel_config c = dma_channel_get_default_config(timer_dma);
    channel_config_set_dreq(&c, DREQ_PIO1_TX0);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    dma_channel_configure(timer_dma, &c, &PIO_TIME->txf[0], instructions + TIMING_OFFSET, 0, false);

    // put AD9959 in default state
    init_pin(PIN_SYNC);
    init_pin(PIN_RESET);
    set_ref_clk(&ad9959, 125 * MHZ);
    set_pll_mult(&ad9959, 4);
    reset();

    while (true) {
        loop();
    }
    return 0;
}
