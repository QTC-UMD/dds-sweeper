#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ad9959.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/pll.h"
#include "hardware/spi.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "trigger.pio.h"
#include "tusb.h"

// Default Pins to use
uint PIN_MISO = 12;
uint PIN_MOSI = 15;
uint PIN_SCK = 14;
uint PIN_RESET = 20;
uint PIN_UPDATE = 22;
uint P0 = 16;
uint P1 = 17;
uint P2 = 18;
uint P3 = 19;
uint TRIGGER = 8;

// SPI config
spi_inst_t* SPI_PORT = spi1;

// Mutex for status
static mutex_t status_mutex;
static mutex_t wait_mutex;

// STATUS flag
#define STOPPED 0
#define RUNNING 1
#define ABORTED 2
int status = STOPPED;

// PIO VALUES IT IS LOOKING FOR
#define UPDATE 0
#define TRIG_UP 3
#define TRIG_DOWN 5

// ============================================================================
// global variables
// ============================================================================
typedef struct pio_sm {
    PIO pio;
    uint sm;
    uint offset;
} pio_sm;

pio_sm trig;
ad9959_config ad9959;
char readstring[256];
#define VERSION "0.0.0"
bool DEBUG = true;

// clang-format off
uint INS_SIZE = 23;
uint8_t instructions[20000];
// clang-format on

// ============================================================================
// Helper Functions
// ============================================================================
void ad9959_reset() {
    sleep_us(1);
    gpio_put(PIN_RESET, 1);
    sleep_us(1);
    gpio_put(PIN_RESET, 0);
    sleep_us(1);
}

void trigger(uint channel, uint val) {
    pio_sm_put(trig.pio, trig.sm, val);
    gpio_put(TRIGGER, 1);
    // sleep_us(1);
    gpio_put(TRIGGER, 0);
}

void wait(uint channel) { pio_sm_get_blocking(trig.pio, trig.sm); }

void update() { pio_sm_put(trig.pio, trig.sm, 0); }

void init_pin(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
}

void set_ins(uint channel, uint addr, double s0, double e0, double rr) {
    uint8_t ins[30];

    if (rr == 0) {
        // if the ramp rate is zero, assume that means the end of forever
        memset(ins, 0, sizeof(ins));
    } else if (ad9959.sweep_type == 0) {
        // Single Tone

    } else if (ad9959.sweep_type == 1) {
        // AMP SWEEP

        if (s0 < 0) s0 = 0;
        if (s0 > 1) s0 = 1;

        if (e0 < 0) e0 = 0;
        if (e0 > 1) e0 = 1;

        s0 = round(s0 * 1023);
        e0 = round(e0 * 1023);

        uint32_t higher, lower;
        if (s0 > e0) {
            // sweep down
            lower = (uint32_t)e0;
            higher = (uint32_t)s0;

            ins[0] = TRIG_DOWN;

            // purposefully big
            memcpy(ins + 8, "\x08\xff\xc0\x00\x00", 5);
            memcpy(ins + 5, "\x07\x02\x01", 3);

            memcpy(ins + 13, "\x09\x00\x40\x00\x00", 5);

        } else {
            // sweep up

            lower = (uint32_t)s0;
            higher = (uint32_t)e0;

            ins[0] = TRIG_UP;

            memcpy(ins + 13, "\x09\x00\x40\x00\x00", 5);
            memcpy(ins + 8, "\x08\x00\x40\x00\x00", 5);

            memcpy(ins + 5, "\x07\x04\x04", 3);
        }

        ins[1] = 0x06;
        ins[18] = 0x0a;

        lower = ((lower & 0xff) << 16) | (lower & 0xff00);
        higher = ((higher & 0x3fc) >> 2) | ((higher & 0x3) << 14);
        memcpy(ins + 2, (uint8_t*)&lower, 3);
        memcpy(ins + 19, (uint8_t*)&higher, 4);

    } else if (ad9959.sweep_type == 2) {
        // freq Sweep
    } else if (ad9959.sweep_type == 3) {
        // phase Sweep
    }

    // printf("Instruction #%d: ", addr);
    // for (int i = 0; i < INS_SIZE; i++) {
    //     printf("%02x ", ins[i]);
    // }
    // printf("\n");

    memcpy(instructions + (addr * INS_SIZE), ins, INS_SIZE);
}

// Thread safe functions for getting/setting status
int get_status() {
    mutex_enter_blocking(&status_mutex);
    int status_copy = status;
    mutex_exit(&status_mutex);
    return status_copy;
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

void resus_callback(void) {
    // Reconfigure PLL sys back to the default state of 1500 / 6 / 2 = 125MHz
    pll_init(pll_sys, 1, 1500 * MHZ, 6, 2);

    // CLK SYS = PLL SYS (125MHz) / 1 = 125MHz
    clock_configure(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, 125 * MHZ,
                    125 * MHZ);

    // kill external clock pins
    gpio_set_function(2, GPIO_FUNC_NULL);

    // Reconfigure IO
    stdio_init_all();
    printf("Resus event fired\n");

    // Wait for uart output to finish
    uart_default_tx_wait_blocking();
}

void background() {
    // let other core know we ready
    multicore_fifo_push_blocking(0);

    while (true) {
        // wait for the batsignal
        uint hwstart = multicore_fifo_pop_blocking();

        ad9959_send_config(&ad9959);
        update();

        set_status(RUNNING);

        // make sure this will not overflow with real big numbers of
        // instructions that would be pretty silly though
        int i = 0;

        while (true) {
            uint offset = INS_SIZE * (i++);

            // If an instruction is empty that means to stop
            if (instructions[offset] == 0x00) {
                set_status(ABORTED);
                break;
            }

            spi_write_blocking(ad9959.spi, instructions + offset + 1,
                               INS_SIZE - 1);
            pio_sm_put(trig.pio, trig.sm, instructions[offset]);
            wait(0);
        }
    }
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

void loop() {
    readline();
    int local_status = get_status();

    if (DEBUG) printf("%s\n", readstring);

    if (strncmp(readstring, "version", 7) == 0) {
        printf("version: %s\n", VERSION);
    } else if (strncmp(readstring, "status", 6) == 0) {
        printf("Running\n");
    } else if (strncmp(readstring, "debug on", 8) == 0) {
        DEBUG = 1;
        printf("ok\n");
    } else if (strncmp(readstring, "debug off", 9) == 0) {
        DEBUG = 0;
        printf("ok\n");
    } else if (strncmp(readstring, "getfreqs", 8) == 0) {
        measure_freqs();
        printf("ok\n");
    }
    // ====================================================
    // Stuff that cannot be done while the table is running
    // ====================================================
    else if (local_status != ABORTED && local_status != STOPPED) {
        printf(
            "Cannot execute command \"%s\" during buffered execution. Check "
            "status first and wait for it to return 0 or 5 (stopped or "
            "aborted).\n",
            readstring);
    } else if (strncmp(readstring, "readregs", 8) == 0) {
        ad9959_read_all(&ad9959);
        printf("ok\n");
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
            printf("Invalid Command - channel must be in range 0-3\n");
        } else {
            uint64_t word = ad9959_config_freq(&ad9959, channel, freq);
            ad9959_send_config(&ad9959);
            update();

            if (DEBUG) {
                double f = word * ad9959.sys_clk / 4294967296.l;
                printf("%12lf\n", f);
            }

            printf("ok\n");
        }
    } else if (strncmp(readstring, "config", 6) == 0) {
        // configtable <type:int> <no dwell:int>

        uint type, no_dwell;
        int parsed = sscanf(readstring, "%*s %u %u", &type, &no_dwell);

        if (parsed < 2) {
            printf(
                "Invalid Command - too few arguments - expected: configtable "
                "<type:int> <no dwell:int>\n");
        } else if (type > 3) {
            printf("Invalid Command - table type must be in range 0-3\n");
        } else if (type != 0 && no_dwell > 1) {
            printf(
                "Invalid Command - no dwell must be in range 0-1 if not in "
                "single tone mode\n");
        } else {
            ad9959_config_table(&ad9959, type, no_dwell);
            printf("OK\n");
        }

    } else if (strncmp(readstring, "set", 3) == 0) {
        // set <channel:int> <addr:int> <start_point:double> <end_point:double>
        // <rate:double>

        uint channel, addr;
        double s0, e0, rr;
        int parsed = sscanf(readstring, "%*s %u %u %lf %lf %lf", &channel,
                            &addr, &s0, &e0, &rr);

        if (parsed < 5) {
            printf(
                "Invalid Command - too few arguments - expected: set "
                "<channel:int> <addr:int> <start_point:double> "
                "<end_point:double> <rate:double>\n");

            // TODO:
            // lots more validation
            // make sure to check what sweep type the table is configured for
        } else {
            set_ins(channel, addr, s0, e0, rr);

            printf("OK\n");
        }

    } else if (strncmp(readstring, "start", 5) == 0) {
        multicore_fifo_push_blocking(0);
        set_status(RUNNING);
        printf("OK\n");
    } else if (strncmp(readstring, "hwstart", 5) == 0) {
        multicore_fifo_push_blocking(1);
        set_status(RUNNING);
        printf("OK\n");
    } else {
        printf("Unrecognized command: \"%s\"\n", readstring);
    }
}

int main() {
    // set sysclock to default 125 MHz
    set_sys_clock_khz(125 * MHZ / 1000, false);
    // have SPI clock follow the system clock for max speed
    clock_configure(clk_peri, 0,
                    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, 125 * MHZ,
                    125 * MHZ);
    // output system clock on pin 21
    clock_gpio_init(21, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, 1);
    // enable clock resus if external sys clock is lost
    clocks_enable_resus(&resus_callback);

    // turn on light as indicator that this is working!
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);

    // init SPI
    spi_init(SPI_PORT, 100 * MHZ);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // launch other core
    multicore_launch_core1(background);
    multicore_fifo_pop_blocking();

    // initialise the status mutex
    mutex_init(&status_mutex);
    mutex_init(&wait_mutex);

    // init the trigger
    trig.pio = pio0;
    trig.sm = pio_claim_unused_sm(pio0, true);
    trig.offset = pio_add_program(pio0, &trigger_program);
    trigger_program_init(pio0, trig.sm, trig.offset, TRIGGER, P3, PIN_UPDATE);

    // put chip in a known state
    init_pin(PIN_RESET);
    ad9959_reset();

    ad9959 = ad9959_get_default_config();
    ad9959_config_spi(&ad9959, SPI_PORT);
    ad9959_send_config(&ad9959);

    update();

    stdio_init_all();
    stdio_init_all();

    printf("\n==================================\n");

    while (true) {
        loop();
    }
    return 0;
}
