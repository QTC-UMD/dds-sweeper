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

// Pins to use controlling the AD9959
#define PIN_MISO 12
#define PIN_MOSI 15
#define PIN_SCK 14
#define PIN_RESET 20
#define PIN_UPDATE 22
#define P0 16
#define P1 17
#define P2 18
#define P3 19
#define TRIGGER 8

// SPI config
#define SPI_PORT spi1

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
#define INS_SIZE 22
uint8_t test[] = {
    // 100 => 50
    0x06, 0x00, 0x02, 0x00,
    0x07, 0x01, 0x01,
    0x08, 0xff, 0xc0, 0x00, 0x00,
    0x09, 0x00, 0x40, 0x00, 0x00,
    0x0a, 0xff, 0xc0, 0x00, 0x00,

    // 50 => 0
    0x06, 0x00, 0x00, 0x00,
    0x07, 0x01, 0x01,
    0x08, 0xff, 0xc0, 0x00, 0x00,
    0x09, 0x00, 0x40, 0x00, 0x00,
    0x0a, 0x80, 0x00, 0x00, 0x00,

    // 0 => 50
    0x06, 0x00, 0x00, 0x00,
    0x07, 0x01, 0x01,
    0x08, 0x00, 0x40, 0x00, 0x00,
    0x09, 0x00, 0x40, 0x00, 0x00,
    0x0a, 0x80, 0x00, 0x00, 0x00,

    // 50 => 100
    0x06, 0x00, 0x02, 0x00,
    0x07, 0x01, 0x01,
    0x08, 0x00, 0x40, 0x00, 0x00,
    0x09, 0x00, 0x40, 0x00, 0x00,
    0x0a, 0xff, 0xc0, 0x00, 0x00,

};
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

    // wait for the batsignal
    multicore_fifo_pop_blocking();


    ad9959_config_amp_sweep(&ad9959, 3, false);
    ad9959_send_config(&ad9959);
    update();


    spi_write_blocking(ad9959.spi, instructions, INS_SIZE);
    pio_sm_put(trig.pio, trig.sm, TRIG_DOWN);
    wait(0);

    spi_write_blocking(ad9959.spi, instructions + (1 * INS_SIZE), INS_SIZE);
    pio_sm_put(trig.pio, trig.sm, TRIG_DOWN);
    wait(0);

    spi_write_blocking(ad9959.spi, instructions + (2 * INS_SIZE), INS_SIZE);
    pio_sm_put(trig.pio, trig.sm, TRIG_UP);
    wait(0);

    spi_write_blocking(ad9959.spi, instructions + (3 * INS_SIZE), INS_SIZE);
    pio_sm_put(trig.pio, trig.sm, TRIG_UP);
    wait(0);

    printf("fin\n");

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
    } else if (strncmp(readstring, "readregs", 8) == 0) {
        ad9959_read_all(&ad9959);
        printf("ok\n");
    } else if (strncmp(readstring, "setfreq", 7) == 0) {
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
    } else {
        printf("Unrecognized command: %s\n", readstring);
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

    // init the trigger
    trig.pio = pio0;
    trig.sm = pio_claim_unused_sm(pio0, true);
    trig.offset = pio_add_program(pio0, &trigger_program);
    trigger_program_init(pio0, trig.sm, trig.offset, TRIGGER, P3, PIN_UPDATE);

    // init the trigger pin
    init_pin(TRIGGER);

    // put chip in a known state
    init_pin(PIN_RESET);
    ad9959_reset();

    ad9959 = ad9959_get_default_config();
    ad9959_config_spi(&ad9959, SPI_PORT);
    ad9959_send_config(&ad9959);

    update();

    stdio_init_all();
    stdio_init_all();

    // let the other guy loose
    multicore_fifo_push_blocking(0);

    while (true) {
        loop();
    }
    return 0;
}
