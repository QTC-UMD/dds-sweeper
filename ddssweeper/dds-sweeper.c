#include <stdio.h>
#include <string.h>

#include "ad9959.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/flash.h"
#include "hardware/pio.h"
#include "hardware/spi.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "trigger_timer.pio.h"

#define VERSION "0.0.0"

// Default Pins to use
#define PIN_MISO 12
#define PIN_MOSI 15
#define PIN_SCK 14
#define PIN_RESET 10
#define PIN_CLOCK 21
#define PIN_UPDATE 22
#define P0 16
#define P1 17
#define P2 18
#define P3 19
#define TRIGGER 8

#define PIO_TRIG pio0
#define PIO_TIME pio1

// Mutex for status
static mutex_t status_mutex;
static mutex_t wait_mutex;

#define FLASH_TARGET_OFFSET (256 * 1024)

// STATUS flag
#define STOPPED 0
#define RUNNING 1
#define ABORTING 2
int status = STOPPED;

// PIO VALUES IT IS LOOKING FOR
#define UPDATE 0

#define MAX_SIZE 249856
#define TIMERS 5000
#define TIMING_OFFSET (MAX_SIZE - TIMERS * 4)

// ============================================================================
// global variables
// ============================================================================
ad9959_config ad9959;
char readstring[256];
bool DEBUG = true;
bool timing = false;

uint triggers;

uint dma;

uint INS_SIZE = 0;
uint8_t instructions[MAX_SIZE];

// ============================================================================
// Setup
// ============================================================================

void init_pin(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
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
    // From https://github.com/raspberrypi/pico-examples under BSD-3-Clause
    // License
    uint f_pll_sys =
        frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
    uint f_pll_usb =
        frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_USB_CLKSRC_PRIMARY);
    uint f_rosc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC);
    uint f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
    uint f_clk_peri = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_PERI);
    uint f_clk_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_USB);
    uint f_clk_adc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_ADC);
    uint f_clk_rtc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_RTC);

    printf("pll_sys = %dkHz\n", f_pll_sys);
    printf("pll_usb = %dkHz\n", f_pll_usb);
    printf("rosc = %dkHz\n", f_rosc);
    printf("clk_sys = %dkHz\n", f_clk_sys);
    printf("clk_peri = %dkHz\n", f_clk_peri);
    printf("clk_usb = %dkHz\n", f_clk_usb);
    printf("clk_adc = %dkHz\n", f_clk_adc);
    printf("clk_rtc = %dkHz\n", f_clk_rtc);
}

void readline() {
    int i = 0;
    char c;
    while (true) {
        c = getchar();
        if (c == '\n') {
            readstring[i] = '\0';
            return;
        } else {
            readstring[i++] = c;
        }
    }
}

// ============================================================================
// Interact with AD9959
// ============================================================================

void update() { pio_sm_put(PIO_TRIG, 0, UPDATE); }

void reset() {
    gpio_put(PIN_RESET, 1);
    sleep_ms(1);
    gpio_put(PIN_RESET, 0);
    sleep_ms(1);

    ad9959.pll_mult = 4;
    ad9959.sys_clk = ad9959.ref_clk * ad9959.pll_mult;
    ad9959.sweep_type = -1;
    ad9959.channels = 1;

    ad9959_default_config();

    update();
}

void wait(uint channel) {
    pio_sm_get_blocking(PIO_TRIG, 0);
    triggers++;
}

void abort_run() {
    if (get_status() == RUNNING) {
        set_status(ABORTING);

        // use the time pio to hit the trigger
        pio_sm_put(PIO_TIME, 0, 10);
    }
}

// ============================================================================
// Set Table Instructions
// ============================================================================

void set_ins(uint type, uint channel, uint addr, double s0, double e0,
             double rate, uint div) {
    uint8_t ins[30];

    uint csrs = ad9959.channels == 1 ? 0 : ad9959.channels;
    uint offset = (INS_SIZE * ad9959.channels + csrs * 2 + 1) * addr + 1;
    uint channel_offset = (INS_SIZE + (csrs ? 2 : 0)) * channel;

    uint tspace = timing ? TIMERS * 4 : 0;
    if (offset + channel_offset + INS_SIZE + 2 + tspace >= MAX_SIZE)
        printf("Invalid Address\n");
        return;

    if (channel == 4 || channel == 5) {
        instructions[offset - 1] = 0x00;
        if (channel == 5)
            instructions[offset] = 0xff;
        else
            instructions[offset] = 0x00;
        return;
    }

    if (type == 0) {
        // SINGLE TONE
        uint32_t ftw, asf, pow;

        instructions[offset - 1] |= 0x01;

        ins[0] = 0x04;
        ins[5] = 0x05;
        ins[8] = 0x06;

        ftw = round(s0 / ad9959.sys_clk * 4294967296.0);
        ftw = ((ftw & 0xff) << 24) | ((ftw & 0xff00) << 8) |
              ((ftw & 0xff0000) >> 8) | ((ftw & 0xff000000) >> 24);

        asf = round(e0 * 1023);
        if (asf < 1) asf = 1;
        asf = ((asf & 0xff) << 16) | (asf & 0xff00) | 0x1000;

        pow = round(rate / 360.0 * 16384.0);
        if (pow > 16384 - 1) pow = 16384 - 1;
        pow = ((pow & 0xff) << 8) | ((pow & 0xff00) >> 8);

        memcpy(ins + 1, (uint8_t *)&ftw, 4);
        memcpy(ins + 6, (uint8_t *)&pow, 2);
        memcpy(ins + 9, (uint8_t *)&asf, 3);
    } else {
        // Not a single tone
        uint32_t lower, higher;

        if (type == 1) {
            // AMP sweep
            uint32_t asf;

            // register addresses
            ins[0] = 0x06;
            ins[4] = 0x07;
            ins[7] = 0x08;
            ins[12] = 0x09;
            ins[17] = 0x0a;
            ins[22] = 0x03;

            // calculations
            s0 = round(s0 * 1023);
            e0 = round(e0 * 1023);
            asf = round(rate * 1023);

            // validation
            if (asf < 1) asf = 1;

            // bit alignment
            asf = ((asf & 0x3fc) >> 2) | ((asf & 0x3) << 14);

            if (s0 <= e0) {
                // UP SWEEP
                lower = (uint32_t)s0;
                higher = (uint32_t)e0;

                ins[5] = 0x01;
                ins[6] = div;

                memcpy(ins + 8, (uint8_t *)&asf, 4);
                memcpy(ins + 13, "\xff\xc0\x00\x00", 4);
                memcpy(ins + 23, "\x40\x43\x10", 3);
            } else {
                // SWEEP DOWN
                lower = (uint32_t)e0;
                higher = (uint32_t)s0;

                ins[5] = div;
                ins[6] = 0x01;

                memcpy(ins + 23, "\x40\x43\x00", 3);
                memcpy(ins + 8, "\xff\xc0\x00\x00", 4);
                memcpy(ins + 13, (uint8_t *)&asf, 4);
            }

            // bit alignments
            lower = ((lower & 0xff) << 16) | (lower & 0xff00);
            higher = ((higher & 0x3fc) >> 2) | ((higher & 0x3) << 14);

            memcpy(ins + 1, (uint8_t *)&lower, 3);
            memcpy(ins + 18, (uint8_t *)&higher, 4);

        } else if (type == 2) {
            // FREQ Sweep

            ins[0] = 0x04;
            ins[5] = 0x07;
            // ??
            ins[6] = div;
            ins[7] = div;
            ins[8] = 0x08;
            ins[13] = 0x09;
            ins[18] = 0x0a;
            ins[23] = 0x03;

            // convert from frequencies to tuning words
            double conv = 4294967296.0 / ad9959.sys_clk;
            uint32_t sword = round(s0 * conv);
            uint32_t eword = round(e0 * conv);
            uint32_t rword = round(rate * conv);

            if (rword < 1) rword = 1;

            // gross bit shifting to force big endianness
            sword = ((sword & 0xff) << 24) | ((sword & 0xff00) << 8) |
                    ((sword & 0xff0000) >> 8) | ((sword & 0xff000000) >> 24);
            eword = ((eword & 0xff) << 24) | ((eword & 0xff00) << 8) |
                    ((eword & 0xff0000) >> 8) | ((eword & 0xff000000) >> 24);
            rword = ((rword & 0xff) << 24) | ((rword & 0xff00) << 8) |
                    ((rword & 0xff0000) >> 8) | ((rword & 0xff000000) >> 24);

            if (s0 <= e0) {
                // sweep up
                lower = sword;
                higher = eword;

                memcpy(ins + 9, (uint8_t *)&rword, 4);
                memcpy(ins + 14, "\x00\x00\x00\x00", 4);
                memcpy(ins + 24, "\x80\x43\x10", 3);
            } else {
                // sweep down
                ins[7] = 0x01;
                lower = eword;
                higher = sword;

                memcpy(ins + 9, "\xff\xff\xff\xff", 4);
                memcpy(ins + 14, (uint8_t *)&rword, 4);
                memcpy(ins + 24, "\x80\x43\x00", 3);
            }

            memcpy(ins + 1, (uint8_t *)&lower, 4);
            memcpy(ins + 19, (uint8_t *)&higher, 4);
        } else if (type == 3) {
            // PHASE Sweep
            uint32_t pow;

            ins[0] = 0x05;
            ins[3] = 0x07;
            // ??
            ins[4] = div;
            ins[5] = div;
            ins[6] = 0x08;
            ins[11] = 0x09;
            ins[16] = 0x0a;
            ins[21] = 0x03;

            // convert from degrees to tuning words
            s0 = round(s0 / 360.0 * 16384.0);
            e0 = round(e0 / 360.0 * 16384.0);
            pow = round(rate / 360.0 * 16384.0);

            if (pow > 16384 - 1) pow = 16384 - 1;
            if (s0 > 16384 - 1) s0 = 16384 - 1;
            if (e0 > 16384 - 1) e0 = 16384 - 1;

            // gross bit shifting to force big endianness
            pow = ((pow & 0x3fc0) >> 6) | ((pow & 0x3f) << 10);

            if (s0 <= e0) {
                // sweep up
                lower = (uint32_t)s0;
                higher = (uint32_t)e0;

                memcpy(ins + 7, (uint8_t *)&pow, 4);
                memcpy(ins + 12, "\x00\x00\x00\x00", 4);
                memcpy(ins + 22, "\xc0\x43\x10", 3);
            } else {
                // sweep down
                ins[7] = 0x01;
                lower = (uint32_t)e0;
                higher = (uint32_t)s0;

                memcpy(ins + 7, "\xff\xff\xff\xff", 4);
                memcpy(ins + 12, (uint8_t *)&pow, 4);
                memcpy(ins + 22, "\xc0\x43\x00", 3);
            }

            lower = ((lower & 0xff) << 8) | ((lower & 0xff00) >> 8);
            higher = ((higher & 0x3fc0) >> 6) | ((higher & 0x3f) << 10);

            memcpy(ins + 1, (uint8_t *)&lower, 2);
            memcpy(ins + 17, (uint8_t *)&higher, 4);
        }

        // setting profile pin trigger bits
        if (s0 <= e0 && ad9959.channels == 1) {
            instructions[offset - 1] |= 0xff;
        } else if (s0 <= e0) {
            instructions[offset - 1] |= (1u << channel) | (1u << (channel + 4));

        } else if (ad9959.channels == 1) {
            instructions[offset - 1] |= 0x0f;
        } else {
            instructions[offset - 1] &= ~(1u << (channel + 4));
            instructions[offset - 1] |= 1u << channel;
        }
    }

    memcpy(instructions + offset + channel_offset, ins, INS_SIZE);

    if (ad9959.channels > 1) {
        uint8_t csr[] = {
            0x00, 0x02 | (1u << (((channel + 1) % ad9959.channels) + 4))};
        memcpy(instructions + offset + channel_offset + INS_SIZE, csr, 2);
    }

    // printf("Instruction #%d - offset %u - channel_offset %d: ", addr, offset,
    //        channel_offset);
    // for (int i = 0; i < INS_SIZE; i++) {
    //     printf("%02x ", ins[i]);
    // }
    // printf("\n");
}

// ============================================================================
// Main Tasks
// ============================================================================

void background() {
    // let other core know we ready
    multicore_fifo_push_blocking(0);

    init_pin(10);

    while (true) {
        // wait for the bat-signal
        multicore_fifo_pop_blocking();

        set_status(RUNNING);

        int i = 0;
        uint csrs = ad9959.channels == 1 ? 0 : ad9959.channels;
        uint offset = (INS_SIZE * ad9959.channels + csrs * 2 + 1) * (i++);

        // set the initial channel select bits
        uint8_t csr[] = {0x00, ad9959.channels == 1 ? 0xf2 : 0x12};
        spi_write_blocking(spi1, csr, 2);

        // work aorund fake trigger
        pio_sm_put(PIO_TRIG, 0, 0x0f);
        pio_sm_put(PIO_TIME, 0, 10);
        wait(0);
        triggers = 0;

        while (status != ABORTING) {
            // If an instruction is empty that means to stop
            if (instructions[offset] == 0x00) {
                if (instructions[offset + 1]) {
                    i = 0;
                    offset =
                        (INS_SIZE * ad9959.channels + csrs * 2 + 1) * (i++);
                } else
                    break;
            }

            if (ad9959.channels > 1) {
                spi_write_blocking(spi1, instructions + offset + 1,
                                   (INS_SIZE + 2) * ad9959.channels);
            } else {
                spi_write_blocking(spi1, instructions + offset + 1, INS_SIZE);
            }

            pio_sm_put(PIO_TRIG, 0, instructions[offset]);

            if (i == 1 && timing) {
                multicore_fifo_push_blocking(1);
            }

            offset = (INS_SIZE * ad9959.channels + csrs * 2 + 1) * (i++);

            wait(0);
        }
        dma_channel_abort(dma);
        pio_sm_clear_fifos(PIO_TRIG, 0);
        pio_sm_clear_fifos(PIO_TIME, 0);
        set_status(STOPPED);
    }
}

void loop() {
    readline();
    int local_status = get_status();

    if (DEBUG) printf("%s\n", readstring);

    if (strncmp(readstring, "version", 7) == 0) {
        printf("version: %s\n", VERSION);
    } else if (strncmp(readstring, "status", 6) == 0) {
        printf("%d\n", local_status);
    } else if (strncmp(readstring, "debug on", 8) == 0) {
        DEBUG = 1;
        printf("ok\n");
    } else if (strncmp(readstring, "debug off", 9) == 0) {
        DEBUG = 0;
        printf("ok\n");
    } else if (strncmp(readstring, "getfreqs", 8) == 0) {
        measure_freqs();
        printf("ok\n");
    } else if (strncmp(readstring, "numtriggers", 11) == 0) {
        printf("%u triggers recieved in last run\n", triggers);
        printf("ok\n");
    } else if (strncmp(readstring, "reset", 5) == 0) {
        abort_run();
        reset();
        set_status(STOPPED);
        printf("ok\n");
    } else if (strncmp(readstring, "abort", 5) == 0) {
        abort_run();
        printf("ok\n");
    } else if (strncmp(readstring, "save", 4) == 0) {
        uint32_t ints = save_and_disable_interrupts();
        printf("Erasing...\n");
        flash_range_erase(FLASH_TARGET_OFFSET, MAX_SIZE);
        printf("Programming...\n");
        flash_range_program(FLASH_TARGET_OFFSET, instructions, MAX_SIZE);
        restore_interrupts(ints);
        printf("ok\n");
    }
    // ====================================================
    // Stuff that cannot be done while the table is running
    // ====================================================
    else if (local_status != STOPPED) {
        printf(
            "Cannot execute command \"%s\" during buffered execution. Check "
            "status first and wait for it to return %d (stopped or "
            "aborted).\n",
            readstring, STOPPED);
    } else if (strncmp(readstring, "readregs", 8) == 0) {
        read_all();
        printf("ok\n");
    } else if (strncmp(readstring, "load", 4) == 0) {
        for (int j = 0; j < MAX_SIZE; j++) {
            instructions[j] = ((uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET))[j];
        }
        for (int j = 0; j < 100; j++) {
            printf("%02x ", instructions[j]);
        }
        printf("\n");

        printf("ok\n");
    } else if (strncmp(readstring, "setchannels", 11) == 0) {
        uint channels;

        int parsed = sscanf(readstring, "%*s %u", &channels);

        if (parsed < 1) {
            printf("Invalid Command - expected: setchannels <num:int>\n");
        } else if (channels < 1 || channels > 4) {
            printf("Invalid Command - expected: num must be in range 0-3\n");
        } else {
            ad9959.channels = channels;
            printf("ok\n");
        }
    } else if (strncmp(readstring, "setfreq", 7) == 0) {
        // setfreq <channel:int> <frequency:float>

        uint channel;
        double freq;
        int parsed = sscanf(readstring, "%*s %u %lf", &channel, &freq);
        if (parsed < 2) {
            printf(
                "Invalid Command - too few arguments - expected: setfreq "
                "<channel:int> <frequency:double>\n");
        } else if (channel < 0 || channel > 3) {
            printf("Invalid Command - num must be in range 0-3\n");
        } else {
            uint64_t word = send_freq(&ad9959, channel, freq);
            update();

            if (DEBUG) {
                double f = word * ad9959.sys_clk / 4294967296.l;
                printf("Freq: %12lf\n", f);
            }

            printf("ok\n");
        }
    } else if (strncmp(readstring, "setphase", 8) == 0) {
        // setphase <channel:int> <phase:float>

        uint channel;
        double phase;
        int parsed = sscanf(readstring, "%*s %u %lf", &channel, &phase);
        if (parsed < 2) {
            printf(
                "Invalid Command - too few arguments - expected: setphase "
                "<channel:int> <frequency:double>\n");
        } else if (channel < 0 || channel > 3) {
            printf("Invalid Command - channel must be in range 0-3\n");
        } else {
            double resp = send_phase(channel, phase);
            update();

            if (DEBUG) {
                printf("Phase: %12lf\n", resp);
            }

            printf("ok\n");
        }
    } else if (strncmp(readstring, "setamp", 6) == 0) {
        // setamp <channel:int> <amp:float>

        uint channel;
        double amp;
        int parsed = sscanf(readstring, "%*s %u %lf", &channel, &amp);
        if (parsed < 2) {
            printf(
                "Invalid Command - too few arguments - expected: setamp "
                "<channel:int> <amp:double>\n");
        } else if (channel < 0 || channel > 3) {
            printf("Invalid Command - channel must be in range 0-3\n");
        } else {
            double resp = send_amp(channel, amp);
            update();

            if (DEBUG) {
                printf("Amp: %12lf\n", resp);
            }

            printf("ok\n");
        }
    } else if (strncmp(readstring, "setmult", 7) == 0) {
        uint mult;

        int parsed = sscanf(readstring, "%*s %u", &mult);

        if (parsed < 1) {
            printf("Invalid Command - expected: setmult <pll_mult:int>\n");
        } else {
            // could do more validation to make sure it is a valid
            // multiply/system clock freq
            uint8_t fr1[] = {0x01, mult << 2, 0x00, 0x00};
            spi_write_blocking(spi1, fr1, 4);
            update();

            printf("ok\n");
        }
    } else if (strncmp(readstring, "setclock", 8) == 0) {
        uint src;   // 0 = internal, 1 = external
        uint freq;  // in Hz (up to 133 MHz)
        int parsed = sscanf(readstring, "%*s %u %u", &src, &freq);
        if (parsed < 2) {
            printf(
                "Invalid Command - expected: setclock <mode:int> <freq:int>\n");
        } else {
            if (src > 1) {
                printf("Invalid Command - mode must be in range 0-1\n");
            } else {
                // Set new clock frequency
                if (src == 0) {
                    if (set_sys_clock_khz(freq / 1000, false)) {
                        ad9959.ref_clk = freq;
                        ad9959.sys_clk = freq * ad9959.pll_mult;
                        clock_configure(
                            clk_peri, 0,
                            CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                            125 * MHZ, 125 * MHZ);
                        clock_gpio_init(
                            PIN_CLOCK,
                            CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, 1);
                        stdio_init_all();
                        printf("ok\n");
                        // clock_status = INTERNAL;
                    } else {
                        printf(
                            "Failure. Cannot exactly achieve that clock "
                            "frequency.");
                    }
                } else {
                    ad9959.ref_clk = freq;
                    ad9959.sys_clk = freq * ad9959.pll_mult;
                    gpio_deinit(PIN_CLOCK);
                    if (DEBUG)
                        printf(
                            "Pico no longer providing ref_clock to AD9959\n");
                    printf("ok\n");
                }
            }
        }
    } else if (strncmp(readstring, "mode", 4) == 0) {
        // mode <type:int> <timing:int>

        uint type, _timing;
        int parsed = sscanf(readstring, "%*s %u %u %u", &type, &_timing);

        if (parsed < 2) {
            printf(
                "Invalid Command - too few arguments - expected: mode "
                "<type:int> <timing:int>\n");
        } else if (type > 3) {
            printf("Invalid Command - table type must be in range 0-3\n");
        } else {
            uint8_t sizes[] = {12, 26, 27, 25};
            INS_SIZE = sizes[type];
            ad9959.sweep_type = type;
            timing = _timing;
            printf("OK\n");
        }

    } else if (strncmp(readstring, "set ", 4) == 0) {
        // set <channel:int> <addr:int> <start_point:double> <end_point:double>
        // <rate:double> <time:int>

        if (ad9959.sweep_type == 0) {
            // SINGLE TONE MODE
            uint channel, addr;
            uint32_t time;
            double freq, amp, phase;
            int parsed = sscanf(readstring, "%*s %u %u %lf %lf %lf %u",
                                &channel, &addr, &freq, &amp, &phase, &time);

            if (!timing && parsed < 5) {
                printf("Invalid Command - expected: set <> <>\n");
            } else if (timing && parsed < 6) {
                printf("Invalid Command - expected: set <> <> nbeed a time!\n");
            } else {
                set_ins(ad9959.sweep_type, channel, addr, freq, amp, phase, 0);
                if (timing) {
                    *((uint32_t *)(instructions + TIMING_OFFSET + 4 * addr)) =
                        time - 10;
                }
            }

        } else if (ad9959.sweep_type <= 3) {
            // AMP SWEEP
            // set <channel:int> <addr:int> <start_point:double>
            // <end_point:double> <rate:double> <div:int>
            uint channel, addr, div;
            uint32_t time;
            double start, end, rate;
            int parsed =
                sscanf(readstring, "%*s %u %u %lf %lf %lf %u %u", &channel,
                       &addr, &start, &end, &rate, &div, &time);

            if (parsed < 6) {
                printf(
                    "Invalid Command - expected: set <channel:int> <addr:int> "
                    "<start_point:double> <end_point:double> <rate:double> "
                    "(<time:int>)\n");
            } else if (timing && parsed < 7) {
                printf(
                    "Invalid Command - expected: set <channel:int> <addr:int>  "
                    "<start_point:double> <end_point:double> <rate:double> "
                    "<time:int>\n");
            } else {
                set_ins(ad9959.sweep_type, channel, addr, start, end, rate,
                        div);
                if (timing) {
                    *((uint32_t *)(instructions + TIMING_OFFSET + 4 * addr)) =
                        time - 10;
                }
            }

        } else {
            printf(
                "Invalid Command - Please set operation type using \'mode\' "
                "first.\n");
        }

    } else if (strncmp(readstring, "start", 5) == 0) {
        if (ad9959.sweep_type == -1) {
            printf("Please select an operating mode using \'mode\' first.\n");
        } else {
            multicore_fifo_push_blocking(0);

            if (timing) {
                dma_channel_config c = dma_channel_get_default_config(dma);
                channel_config_set_dreq(&c, DREQ_PIO1_TX0);
                channel_config_set_transfer_data_size(&c, DMA_SIZE_32);

                multicore_fifo_pop_blocking();

                dma_channel_configure(dma, &c, &PIO_TIME->txf[0],
                                      instructions + TIMING_OFFSET, TIMERS,
                                      true);
            }

            printf("OK\n");
        }
    } else {
        printf("Unrecognized command: \"%s\"\n", readstring);
    }
}

int main() {
    // set sysclock to default 125 MHz and output it
    set_sys_clock_khz(125 * MHZ / 1000, false);
    clock_gpio_init(PIN_CLOCK, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, 1);

    // attatch spi to system clock
    clock_configure(clk_peri, 0,
                    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, 125 * MHZ,
                    125 * MHZ);

    // turn on LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);

    // init SPI
    spi_init(spi1, 100 * MHZ);
    // AD9959 seems to have support for all SPI modes
    spi_set_format(spi1, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
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
    uint offset = pio_add_program(PIO_TRIG, &trigger_program);
    trigger_program_init(PIO_TRIG, 0, offset, TRIGGER, 9, P0, PIN_UPDATE);
    offset = pio_add_program(PIO_TIME, &timer_program);
    timer_program_init(PIO_TIME, 0, offset, TRIGGER);

    // setup dma
    dma = dma_claim_unused_channel(true);

    // put chip in a known state
    ad9959.ref_clk = 125 * MHZ;
    init_pin(PIN_RESET);
    reset();

    // second init is a workaround for the first char getting corrupted
    stdio_init_all();
    stdio_init_all();

    // zeroing out the table is not necessary, but could help debugging tables
    memset(instructions, 0, MAX_SIZE);

    // for debugging over uart only
    printf("\n==================================\n");

    while (true) {
        loop();
    }
    return 0;
}
