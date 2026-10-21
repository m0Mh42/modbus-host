/**
 * @file    ads1220.h
 * @brief   Driver for the TI ADS1220 24-bit delta-sigma ADC.
 *
 * Two devices share one SPI bus; each is selected by its own CS and signals
 * conversion-ready on its own DRDY line. One device serves thermocouple
 * channels 0..3, the other 4..7. Channels are read single-ended vs AVSS
 * (MUX = AINx-AVSS), one per ADS1220 input pin.
 *
 * Register-level behavior is modeled on the TI datasheet and the open-source
 * Arduino libraries (ADS1220_WE / Protocentral), re-implemented on top of the
 * project HAL so it builds without the Arduino framework.
 */
#ifndef ADS1220_H
#define ADS1220_H

#include <stdint.h>
#include <stdbool.h>

/* ---- SPI command set ---- */
#define ADS1220_CMD_RESET 0x06
#define ADS1220_CMD_START 0x08 /* START/SYNC */
#define ADS1220_CMD_POWERDOWN 0x02
#define ADS1220_CMD_RDATA 0x10
#define ADS1220_CMD_RREG 0x20 /* | (reg<<2) | (count-1) */
#define ADS1220_CMD_WREG 0x40 /* | (reg<<2) | (count-1) */

/* ---- Configuration registers ---- */
#define ADS1220_REG_CONFIG0 0x00 /* MUX[7:4] GAIN[3:1] PGA_BYPASS[0] */
#define ADS1220_REG_CONFIG1 0x01 /* DR[7:5] MODE[4:3] CM[2] TS[1] BCS[0] */
#define ADS1220_REG_CONFIG2 0x02 /* VREF[7:6] FIR[5:4] PSW[3] IDAC[2:0] */
#define ADS1220_REG_CONFIG3 0x03 /* I1MUX[7:5] I2MUX[4:2] DRDYM[1] */

/* ---- CONFIG0 field helpers ---- */
/* MUX codes for single-ended AINx vs AVSS (used for the 4 channels/chip). */
#define ADS1220_MUX_AIN0_AVSS 0x08
#define ADS1220_MUX_AIN1_AVSS 0x09
#define ADS1220_MUX_AIN2_AVSS 0x0A
#define ADS1220_MUX_AIN3_AVSS 0x0B
/* PGA gain field (bits 3:1). Gain 1..128 in powers of two. */
#define ADS1220_GAIN_1 0x00
#define ADS1220_GAIN_2 0x02
#define ADS1220_GAIN_4 0x04
#define ADS1220_GAIN_8 0x06

/* Handle describing one physical ADS1220 and its register shadow. */
typedef struct
{
    uint32_t cs_port;
    uint32_t cs_pin;
    uint32_t drdy_port;
    uint32_t drdy_pin;
    uint8_t config[4]; /* shadow of CONFIG0..3 */
} ads1220_t;

/** Initialize device pins, reset the chip, and write the default config. */
bool ads1220_init(ads1220_t *dev);

/** Write a configuration register (also updates the shadow copy). */
void ads1220_write_reg(ads1220_t *dev, uint8_t reg, uint8_t value);

/** Read a configuration register from the chip. */
uint8_t ads1220_read_reg(ads1220_t *dev, uint8_t reg);

/** Point the input mux at logical channel 0..3 (AINx-AVSS). */
void ads1220_select_channel(ads1220_t *dev, uint8_t channel);

/** Issue START/SYNC to begin a (single-shot) conversion. */
void ads1220_start(ads1220_t *dev);

/** True once DRDY (active low) indicates a fresh result. */
bool ads1220_data_ready(ads1220_t *dev);

/** Read the latest 24-bit signed conversion, sign-extended to int32. */
int32_t ads1220_read_data(ads1220_t *dev);

#endif /* ADS1220_H */
