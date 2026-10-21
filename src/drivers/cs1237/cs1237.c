/**
 * @file cs1237.c
 * @brief CS1237 24-bit ADC driver using sequential wait-and-read on shared SCLK.
 *        Ported from gd32-cs1237-test with cooperative anti-timeout yielding.
 *
 * Implements the proven sequential wait-and-read protocol:
 *  - Shared SCLK on PB12 with 8 individual DOUT lines.
 *  - Each chip is sampled immediately after its own fresh DRDY falling edge.
 *  - During all DRDY wait loops, cs1237_yield() calls modbus_task() so
 *    incoming Modbus register writes and requests are serviced immediately.
 *  - Generous per-channel timeout (TC_SCAN_TIMEOUT_MS = 250ms) ensures asynchronous
 *    oscillator drift between chips never causes false timeouts.
 */
#include "cs1237.h"
#include "board_config.h"
#include "hal_time.h"
#include "gd32f30x.h"
#include <stddef.h>

static const uint32_t DOUT_PORTS[CS1237_CHANNELS] = {
    CS1237_DOUT1_PORT,
    CS1237_DOUT2_PORT,
    CS1237_DOUT3_PORT,
    CS1237_DOUT4_PORT,
    CS1237_DOUT5_PORT,
    CS1237_DOUT6_PORT,
    CS1237_DOUT7_PORT,
    CS1237_DOUT8_PORT};

static const uint32_t DOUT_PINS[CS1237_CHANNELS] = {
    CS1237_DOUT1_PIN,
    CS1237_DOUT2_PIN,
    CS1237_DOUT3_PIN,
    CS1237_DOUT4_PIN,
    CS1237_DOUT5_PIN,
    CS1237_DOUT6_PIN,
    CS1237_DOUT7_PIN,
    CS1237_DOUT8_PIN};

static cs1237_yield_fn_t s_yield_cb = NULL;

void cs1237_set_yield_callback(cs1237_yield_fn_t fn)
{
    s_yield_cb = fn;
}

static inline void cs1237_yield(void)
{
    if (s_yield_cb)
    {
        s_yield_cb();
    }
}

static inline void delay_1us(void)
{
    /* At 120 MHz CPU frequency, 30 NOP loop yields ~1.0-1.2 us */
    for (volatile int i = 0; i < 30; i++)
    {
        __NOP();
    }
}

static inline void cs1237_clock_pulse(void)
{
    gpio_bit_set(CS1237_SCLK_PORT, CS1237_SCLK_PIN);
    delay_1us();
    gpio_bit_reset(CS1237_SCLK_PORT, CS1237_SCLK_PIN);
    delay_1us();
}

static int32_t cs1237_shift_in(uint32_t port, uint32_t pin)
{
    int32_t v = 0;
    for (uint8_t i = 0; i < 24; i++)
    {
        gpio_bit_set(CS1237_SCLK_PORT, CS1237_SCLK_PIN);
        delay_1us();
        v = (v << 1) | (gpio_input_bit_get(port, pin) == SET ? 1 : 0);
        gpio_bit_reset(CS1237_SCLK_PORT, CS1237_SCLK_PIN);
        delay_1us();
    }
    /* Sign-extend 24-bit two's complement to 32-bit signed integer */
    if (v & 0x800000L)
    {
        v -= 0x1000000L;
    }
    return v;
}

static bool cs1237_wait_edge(uint32_t port, uint32_t pin, uint32_t timeout_ms)
{
    gpio_bit_reset(CS1237_SCLK_PORT, CS1237_SCLK_PIN);
    uint32_t start = hal_millis();

    /* Skip stale-ready low state: wait for high */
    while (gpio_input_bit_get(port, pin) == RESET)
    {
        cs1237_yield();
        if (hal_millis() - start > timeout_ms)
        {
            return false;
        }
        delay_1us();
    }

    /* Wait for fresh falling edge: high -> low */
    while (gpio_input_bit_get(port, pin) == SET)
    {
        cs1237_yield();
        if (hal_millis() - start > timeout_ms)
        {
            return false;
        }
        delay_1us();
    }
    return true;
}

bool cs1237_configure_one(uint8_t channel, uint8_t gain, uint8_t speed, uint8_t ch_cfg)
{
    if (channel >= CS1237_CHANNELS)
    {
        return false;
    }

    uint32_t port = DOUT_PORTS[channel];
    uint32_t pin = DOUT_PINS[channel];

    if (!cs1237_wait_edge(port, pin, TC_SCAN_TIMEOUT_MS))
    {
        return false;
    }

    /* Clocks 1-24: flush pending conversion */
    (void)cs1237_shift_in(port, pin);

    /* Clocks 25-27: 3 turnaround clocks */
    cs1237_clock_pulse();
    cs1237_clock_pulse();
    cs1237_clock_pulse();

    /* Clocks 28-29: chip switches its data pin to input, MCU switches DOUT to OUTPUT */
    gpio_init(port, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, pin);
    cs1237_clock_pulse();
    cs1237_clock_pulse();

    /* Clocks 30-36: 7-bit write-config command (0x65), MSB first */
    for (int8_t i = 6; i >= 0; i--)
    {
        gpio_bit_set(CS1237_SCLK_PORT, CS1237_SCLK_PIN);
        delay_1us();
        if ((CS1237_WRITE_CONFIG >> i) & 0x01)
        {
            gpio_bit_set(port, pin);
        }
        else
        {
            gpio_bit_reset(port, pin);
        }
        gpio_bit_reset(CS1237_SCLK_PORT, CS1237_SCLK_PIN);
        delay_1us();
    }

    /* Clock 37: turnaround clock */
    cs1237_clock_pulse();

    /* Clocks 38-45: 8-bit register value, MSB first */
    uint8_t reg = gain | speed | ch_cfg;
    for (int8_t i = 7; i >= 0; i--)
    {
        gpio_bit_set(CS1237_SCLK_PORT, CS1237_SCLK_PIN);
        delay_1us();
        if ((reg >> i) & 0x01)
        {
            gpio_bit_set(port, pin);
        }
        else
        {
            gpio_bit_reset(port, pin);
        }
        gpio_bit_reset(CS1237_SCLK_PORT, CS1237_SCLK_PIN);
        delay_1us();
    }

    /* Clock 46: chip takes back its data pin */
    cs1237_clock_pulse();

    /* Return DOUT pin to input mode (pull-up) */
    gpio_init(port, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, pin);
    return true;
}

bool cs1237_configure_all(uint8_t gain, uint8_t speed, uint8_t ch_cfg)
{
    bool all_ok = true;

    for (uint8_t i = 0; i < CS1237_CHANNELS; i++)
    {
        bool ok = false;
        for (uint8_t retry = 0; retry < 3 && !ok; retry++)
        {
            ok = cs1237_configure_one(i, gain, speed, ch_cfg);
            if (!ok)
            {
                hal_delay_ms(30);
            }
        }
        if (!ok)
        {
            all_ok = false;
        }
    }

    /* Flush initial conversion after configuration */
    int32_t dummy_raw[CS1237_CHANNELS];
    bool dummy_valid[CS1237_CHANNELS];
    cs1237_scan_all(dummy_raw, dummy_valid, TC_SCAN_TIMEOUT_MS);

    return all_ok;
}

bool cs1237_scan_channel(uint8_t channel, int32_t *raw_out, uint32_t timeout_ms)
{
    if (channel >= CS1237_CHANNELS)
    {
        return false;
    }

    uint32_t port = DOUT_PORTS[channel];
    uint32_t pin = DOUT_PINS[channel];

    if (!cs1237_wait_edge(port, pin, timeout_ms))
    {
        return false;
    }

    int32_t val = cs1237_shift_in(port, pin);
    /* Clocks 25-27: 3 turnaround clocks; clock 27 forces DOUT/DRDY back HIGH to prepare for next conversion */
    cs1237_clock_pulse();
    cs1237_clock_pulse();
    cs1237_clock_pulse();

    if (raw_out)
    {
        *raw_out = val;
    }
    return true;
}

uint8_t cs1237_scan_all(int32_t raw_out[CS1237_CHANNELS], bool valid_out[CS1237_CHANNELS], uint32_t timeout_ms)
{
    gpio_bit_reset(CS1237_SCLK_PORT, CS1237_SCLK_PIN); /* Ensure awake/converting */

    uint8_t count = 0;
    for (uint8_t i = 0; i < CS1237_CHANNELS; i++)
    {
        valid_out[i] = false;
        raw_out[i] = 0;

        uint32_t port = DOUT_PORTS[i];
        uint32_t pin = DOUT_PINS[i];

        if (cs1237_wait_edge(port, pin, timeout_ms))
        {
            raw_out[i] = cs1237_shift_in(port, pin);
            /* Clocks 25-27: 3 turnaround clocks; clock 27 forces DOUT/DRDY back HIGH */
            cs1237_clock_pulse();
            cs1237_clock_pulse();
            cs1237_clock_pulse();
            valid_out[i] = true;
            count++;
        }
        cs1237_yield();
    }
    return count;
}

bool cs1237_read_averaged(int32_t avg[CS1237_CHANNELS], bool open[CS1237_CHANNELS])
{
    int64_t sum[CS1237_CHANNELS] = {0};
    int32_t raw[CS1237_CHANNELS];
    bool valid[CS1237_CHANNELS];
    uint8_t valid_samples[CS1237_CHANNELS] = {0};

    for (uint8_t i = 0; i < CS1237_CHANNELS; i++)
    {
        open[i] = false;
        avg[i] = 0;
    }

    for (uint8_t s = 0; s < TC_AVG_SAMPLES; s++)
    {
        cs1237_scan_all(raw, valid, TC_SCAN_TIMEOUT_MS);
        for (uint8_t i = 0; i < CS1237_CHANNELS; i++)
        {
            if (valid[i])
            {
                if (cs1237_is_open(raw[i]))
                {
                    open[i] = true;
                }
                sum[i] += raw[i];
                valid_samples[i]++;
            }
        }
    }

    bool any_valid = false;
    for (uint8_t i = 0; i < CS1237_CHANNELS; i++)
    {
        if (valid_samples[i] > 0)
        {
            avg[i] = (int32_t)(sum[i] / valid_samples[i]);
            any_valid = true;
        }
        else
        {
            open[i] = true;
        }
    }
    return any_valid;
}

float cs1237_raw_to_microvolts(int32_t raw)
{
    return (float)raw * (0.5f * CS1237_VREF / TC_GAIN_F) / 8388608.0f * 1e6f;
}

bool cs1237_is_open(int32_t raw)
{
    return (raw > TC_OPEN_THRESHOLD) || (raw < -TC_OPEN_THRESHOLD);
}

bool cs1237_init(void)
{
    /* Enable GPIO A, B, and C clocks */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);

    /* SCLK (PB12): Output Push-Pull, initial state LOW */
    gpio_init(CS1237_SCLK_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, CS1237_SCLK_PIN);
    gpio_bit_reset(CS1237_SCLK_PORT, CS1237_SCLK_PIN);

    /* DOUT1..8: Input with Pull-Up */
    for (uint8_t i = 0; i < CS1237_CHANNELS; i++)
    {
        gpio_init(DOUT_PORTS[i], GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, DOUT_PINS[i]);
    }

    /* Allow chips to complete initial power-on reset */
    hal_delay_ms(50);

    return cs1237_configure_all(TC_GAIN, TC_SPEED, CS1237_CHANNEL_A | CS1237_REF_EXTERNAL);
}
