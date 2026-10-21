/**
 * @file cs1237.h
 * @brief CS1237 24-bit ADC driver for 8-channel thermocouple acquisition.
 *        Ported from gd32-cs1237-test with cooperative anti-timeout yielding.
 */
#ifndef CS1237_H
#define CS1237_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef CS1237_CHANNELS
#define CS1237_CHANNELS 8
#endif
#ifndef CS1237_NUM_CHANNELS
#define CS1237_NUM_CHANNELS CS1237_CHANNELS
#endif

/* CS1237 SPI Command bytes (7-bit) */
#define CS1237_CMD_READ_CONFIG 0x56
#define CS1237_CMD_WRITE_CONFIG 0x65
#define CS1237_READ_CONFIG 0x56
#define CS1237_WRITE_CONFIG 0x65

/* Sample rates */
#define CS1237_SPEED_10 0b00000000   /* 10 Hz */
#define CS1237_SPEED_40 0b00010000   /* 40 Hz */
#define CS1237_SPEED_640 0b00100000  /* 640 Hz */
#define CS1237_SPEED_1280 0b00110000 /* 1280 Hz */

/* PGA Gains */
#define CS1237_PGA_1 0b00000000
#define CS1237_PGA_2 0b00000100
#define CS1237_PGA_64 0b00001000
#define CS1237_PGA_128 0b00001100

/* Channels */
#define CS1237_CHANNEL_A 0b00000000    /* External differential input */
#define CS1237_CHANNEL_TEMP 0b00000010 /* Internal temperature sensor */

/* Reference Selection */
#define CS1237_REF_INTERNAL 0b00000000
#define CS1237_REF_EXTERNAL 0b01000000

/* Board Reference Voltage (TL431 shunt provides ~2.50V on VREF net) */
#define CS1237_VREF 2.5f

/* Active operating parameters */
#define TC_GAIN CS1237_PGA_128
#define TC_GAIN_F 128.0f
#define TC_SPEED CS1237_SPEED_40
#define TC_SPS 40
#define TC_AVG_SAMPLES 4
#define TC_SCAN_TIMEOUT_MS 250

/* Open-thermocouple threshold (~98% of full scale 2^23 - 1) */
#define TC_OPEN_THRESHOLD 8220000L

    /** Callback hook to service communications (e.g. modbus_task) during waits */
    typedef void (*cs1237_yield_fn_t)(void);

    /**
     * @brief Register a cooperative yield function to be called during ADC wait loops.
     * @param fn Pointer to function (e.g. modbus_task)
     */
    void cs1237_set_yield_callback(cs1237_yield_fn_t fn);

    /**
     * @brief Initialize GPIOs and configure all 8 CS1237 ADC chips.
     * @return true if all chips configured successfully, false otherwise
     */
    bool cs1237_init(void);

    /**
     * @brief Configure one CS1237 chip register (46 clocks sequence).
     * @param channel Channel index (0..7)
     * @param gain Gain setting (e.g. CS1237_PGA_128)
     * @param speed Sample rate setting (e.g. CS1237_SPEED_40)
     * @param ch_cfg Channel & Reference setting (e.g. CS1237_CHANNEL_A | CS1237_REF_EXTERNAL)
     * @return true if successful, false on timeout
     */
    bool cs1237_configure_one(uint8_t channel, uint8_t gain, uint8_t speed, uint8_t ch_cfg);

    /**
     * @brief Configure all 8 CS1237 chips sequentially with retries.
     * @param gain Gain setting (e.g. CS1237_PGA_128)
     * @param speed Sample rate setting (e.g. CS1237_SPEED_40)
     * @param ch_cfg Channel & Reference setting (e.g. CS1237_CHANNEL_A | CS1237_REF_EXTERNAL)
     * @return true if all chips acknowledged configuration
     */
    bool cs1237_configure_all(uint8_t gain, uint8_t speed, uint8_t ch_cfg);

    /**
     * @brief Read a single CS1237 channel sequentially with cooperative yield.
     * @param channel Channel index (0..7)
     * @param raw_out Pointer to receive signed 24-bit raw ADC code
     * @param timeout_ms Maximum time to wait for fresh DRDY falling edge
     * @return true if fresh sample read within timeout, false otherwise
     */
    bool cs1237_scan_channel(uint8_t channel, int32_t *raw_out, uint32_t timeout_ms);

    /**
     * @brief Sequentially scan all 8 channels with cooperative yielding.
     * @param raw_out Array to store 24-bit sign-extended raw readings [0..7]
     * @param valid_out Array of flags indicating which channels completed within timeout
     * @param timeout_ms Timeout per channel in milliseconds (e.g. 250ms)
     * @return Number of channels successfully read (0..8)
     */
    uint8_t cs1237_scan_all(int32_t raw_out[CS1237_CHANNELS], bool valid_out[CS1237_CHANNELS], uint32_t timeout_ms);

    /**
     * @brief Read averaged samples across all 8 channels with open-circuit detection.
     * @param avg Array to store averaged raw values
     * @param open Array returning true if channel is open / burned out
     * @return true if all averaged reads succeeded
     */
    bool cs1237_read_averaged(int32_t avg[CS1237_CHANNELS], bool open[CS1237_CHANNELS]);

    /**
     * @brief Convert raw 24-bit ADC code to input microvolts.
     * @param raw Raw signed 24-bit ADC code
     * @return Microvolts at ADC differential input
     */
    float cs1237_raw_to_microvolts(int32_t raw);

    /**
     * @brief Check if raw ADC reading indicates an open thermocouple circuit.
     * @param raw Raw signed 24-bit ADC code
     * @return true if open, false if normal
     */
    bool cs1237_is_open(int32_t raw);

#ifdef __cplusplus
}
#endif

#endif /* CS1237_H */
