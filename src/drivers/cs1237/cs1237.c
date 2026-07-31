/**
 * @file cs1237.c
 * @brief CS1237 driver using bit-banging with shared SCLK.
 */
#include "cs1237.h"
#include "board_config.h"
#include "hal_time.h"

#define TC_SCAN_TIMEOUT_MS 250

static inline void delay_1us(void) {
    for (volatile int i = 0; i < 15; i++) {}
}

static inline void tc_clock_pulse(void) {
    gpio_bit_write(CS1237_SCLK_PORT, CS1237_SCLK_PIN, SET);
    delay_1us();
    gpio_bit_write(CS1237_SCLK_PORT, CS1237_SCLK_PIN, RESET);
    delay_1us();
}

static uint32_t get_dout_port(uint8_t ch) {
    switch (ch) {
        case 0: return CS1237_DOUT1_PORT;
        case 1: return CS1237_DOUT2_PORT;
        case 2: return CS1237_DOUT3_PORT;
        case 3: return CS1237_DOUT4_PORT;
        case 4: return CS1237_DOUT5_PORT;
        case 5: return CS1237_DOUT6_PORT;
        case 6: return CS1237_DOUT7_PORT;
        case 7: return CS1237_DOUT8_PORT;
        default: return CS1237_DOUT1_PORT;
    }
}

static uint32_t get_dout_pin(uint8_t ch) {
    switch (ch) {
        case 0: return CS1237_DOUT1_PIN;
        case 1: return CS1237_DOUT2_PIN;
        case 2: return CS1237_DOUT3_PIN;
        case 3: return CS1237_DOUT4_PIN;
        case 4: return CS1237_DOUT5_PIN;
        case 5: return CS1237_DOUT6_PIN;
        case 6: return CS1237_DOUT7_PIN;
        case 7: return CS1237_DOUT8_PIN;
        default: return CS1237_DOUT1_PIN;
    }
}

void cs1237_init(void)
{
    /* Assume RCU clocks for GPIOs are enabled in hal_init or similar */
    gpio_init(CS1237_SCLK_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, CS1237_SCLK_PIN);
    gpio_bit_write(CS1237_SCLK_PORT, CS1237_SCLK_PIN, RESET);
    
    for (uint8_t i = 0; i < 8; i++) {
        gpio_init(get_dout_port(i), GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, get_dout_pin(i));
    }
}

static int32_t tc_shift_in(uint32_t port, uint32_t pin) {
    int32_t v = 0;
    for (uint8_t i = 0; i < 24; i++) {
        gpio_bit_write(CS1237_SCLK_PORT, CS1237_SCLK_PIN, SET);
        delay_1us();
        v = (v << 1) | (gpio_input_bit_get(port, pin) == SET ? 1 : 0);
        gpio_bit_write(CS1237_SCLK_PORT, CS1237_SCLK_PIN, RESET);
        delay_1us();
    }
    if (v & 0x800000L) {
        v -= 0x1000000L; /* sign extension */
    }
    return v;
}

static bool tc_wait_edge(uint32_t port, uint32_t pin) {
    gpio_bit_write(CS1237_SCLK_PORT, CS1237_SCLK_PIN, RESET);
    uint32_t start = hal_millis();
    
    /* Wait for stale low to pass */
    while (gpio_input_bit_get(port, pin) == RESET) {
        if (hal_millis() - start > TC_SCAN_TIMEOUT_MS) return false;
    }
    /* Wait for fresh falling edge */
    while (gpio_input_bit_get(port, pin) == SET) {
        if (hal_millis() - start > TC_SCAN_TIMEOUT_MS) return false;
    }
    return true;
}

bool cs1237_configure_one(uint8_t ch, uint8_t gain, uint8_t speed, uint8_t channel)
{
    if (ch >= 8) return false;
    uint32_t port = get_dout_port(ch);
    uint32_t pin = get_dout_pin(ch);

    if (!tc_wait_edge(port, pin)) return false;
    
    (void)tc_shift_in(port, pin); /* clocks 1-24 */
    tc_clock_pulse(); /* 25 */
    tc_clock_pulse(); /* 26 */
    tc_clock_pulse(); /* 27 */

    /* clocks 28-29 turnaround */
    gpio_init(port, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, pin);
    tc_clock_pulse();
    tc_clock_pulse();

    /* clocks 30-36: write config command 0x65 */
    for (int8_t i = 6; i >= 0; i--) {
        gpio_bit_write(CS1237_SCLK_PORT, CS1237_SCLK_PIN, SET);
        delay_1us();
        gpio_bit_write(port, pin, ((0x65 >> i) & 1) ? SET : RESET);
        gpio_bit_write(CS1237_SCLK_PORT, CS1237_SCLK_PIN, RESET);
        delay_1us();
    }

    tc_clock_pulse(); /* 37 turnaround */

    /* clocks 38-45: write register */
    uint8_t reg = gain | speed | channel;
    for (int8_t i = 7; i >= 0; i--) {
        gpio_bit_write(CS1237_SCLK_PORT, CS1237_SCLK_PIN, SET);
        delay_1us();
        gpio_bit_write(port, pin, ((reg >> i) & 1) ? SET : RESET);
        gpio_bit_write(CS1237_SCLK_PORT, CS1237_SCLK_PIN, RESET);
        delay_1us();
    }

    tc_clock_pulse(); /* clock 46 */

    /* Give data line back */
    gpio_init(port, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, pin);
    return true;
}

bool cs1237_scan_all(int32_t raw_out[])
{
    gpio_bit_write(CS1237_SCLK_PORT, CS1237_SCLK_PIN, RESET);

    for (uint8_t i = 0; i < 8; i++) {
        uint32_t port = get_dout_port(i);
        uint32_t pin = get_dout_pin(i);
        
        uint32_t start = hal_millis();
        
        while (gpio_input_bit_get(port, pin) == RESET) {
            if (hal_millis() - start > TC_SCAN_TIMEOUT_MS) return false;
        }
        
        while (gpio_input_bit_get(port, pin) == SET) {
            if (hal_millis() - start > TC_SCAN_TIMEOUT_MS) return false;
        }
        
        raw_out[i] = tc_shift_in(port, pin);
    }
    return true;
}
