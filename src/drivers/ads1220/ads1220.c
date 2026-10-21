/**
 * @file ads1220.c
 * @brief ADS1220 driver implementation.
 *
 * SPI access pattern (datasheet):
 *   - CS low for the whole command/transfer, high afterwards.
 *   - WREG: 0x40 | (reg<<2) | (count-1), then `count` data bytes.
 *   - RREG: 0x20 | (reg<<2) | (count-1), then read `count` bytes.
 *   - RDATA: 0x10, then read 3 bytes (MSB first), 24-bit two's complement.
 *
 * Edit the default register block in ads1220_init() to match your sensor
 * (gain, data rate, reference, conversion mode).
 */
#include "ads1220.h"
#include "hal_spi.h"
#include "hal_gpio.h"
#include "hal_time.h"

/* ---- CS helpers ---- */
static inline void cs_low(ads1220_t *dev) { hal_gpio_write(dev->cs_port, dev->cs_pin, false); }
static inline void cs_high(ads1220_t *dev) { hal_gpio_write(dev->cs_port, dev->cs_pin, true); }

bool ads1220_init(ads1220_t *dev)
{
    /* Pins: CS output (idle high), DRDY input (active low, external pull-up
     * on the board or use internal pull-up here). */
    hal_gpio_init(dev->cs_port, dev->cs_pin, HAL_GPIO_OUTPUT);
    cs_high(dev);
    hal_gpio_init(dev->drdy_port, dev->drdy_pin, HAL_GPIO_INPUT_PULLUP);

    /* Reset and let the device settle. */
    cs_low(dev);
    hal_spi_transfer(ADS1220_CMD_RESET);
    cs_high(dev);
    hal_delay_ms(1);

    /* ---- Default configuration (edit for your thermocouple setup) ----
     * CONFIG0: AIN0-AVSS, gain 1, PGA enabled.
     * CONFIG1: 20 SPS, single-shot (MODE/CM), normal.
     * CONFIG2: internal 2.048 V reference, 50/60 Hz rejection.
     * CONFIG3: IDACs off, DRDY only on dedicated pin.
     */
    dev->config[0] = ADS1220_MUX_AIN0_AVSS | ADS1220_GAIN_1; /* 0x08 */
    dev->config[1] = 0x00;                                   /* DR=20SPS, single-shot, normal mode */
    dev->config[2] = 0x00;                                   /* internal 2.048V ref (00), no filter, IDAC off */
    dev->config[3] = 0x00;                                   /* no IDAC routing, DRDY pin only */

    /* Burst-write all four config registers in one WREG. */
    cs_low(dev);
    hal_spi_transfer(ADS1220_CMD_WREG | (ADS1220_REG_CONFIG0 << 2) | (4 - 1));
    for (int i = 0; i < 4; i++)
    {
        hal_spi_transfer(dev->config[i]);
    }
    cs_high(dev);

    /* Read CONFIG0 back as a presence check. */
    return ads1220_read_reg(dev, ADS1220_REG_CONFIG0) == dev->config[0];
}

void ads1220_write_reg(ads1220_t *dev, uint8_t reg, uint8_t value)
{
    cs_low(dev);
    hal_spi_transfer(ADS1220_CMD_WREG | (reg << 2) | 0x00); /* count-1 = 0 */
    hal_spi_transfer(value);
    cs_high(dev);
    if (reg < 4)
        dev->config[reg] = value;
}

uint8_t ads1220_read_reg(ads1220_t *dev, uint8_t reg)
{
    cs_low(dev);
    hal_spi_transfer(ADS1220_CMD_RREG | (reg << 2) | 0x00);
    uint8_t v = hal_spi_transfer(0xFF);
    cs_high(dev);
    return v;
}

void ads1220_select_channel(ads1220_t *dev, uint8_t channel)
{
    /* Map logical channel 0..3 to AINx-AVSS mux code in CONFIG0[7:4]. */
    static const uint8_t mux[4] = {
        ADS1220_MUX_AIN0_AVSS,
        ADS1220_MUX_AIN1_AVSS,
        ADS1220_MUX_AIN2_AVSS,
        ADS1220_MUX_AIN3_AVSS,
    };
    if (channel > 3)
        channel = 3;

    /* Preserve gain/PGA bits, replace only MUX[7:4]. */
    uint8_t cfg0 = (dev->config[0] & 0x0F) | (mux[channel] << 4);
    ads1220_write_reg(dev, ADS1220_REG_CONFIG0, cfg0);
}

void ads1220_start(ads1220_t *dev)
{
    cs_low(dev);
    hal_spi_transfer(ADS1220_CMD_START);
    cs_high(dev);
}

bool ads1220_data_ready(ads1220_t *dev)
{
    /* DRDY is active low. */
    return hal_gpio_read(dev->drdy_port, dev->drdy_pin) == false;
}

int32_t ads1220_read_data(ads1220_t *dev)
{
    uint8_t b[3];
    cs_low(dev);
    hal_spi_transfer(ADS1220_CMD_RDATA);
    b[0] = hal_spi_transfer(0xFF); /* MSB */
    b[1] = hal_spi_transfer(0xFF);
    b[2] = hal_spi_transfer(0xFF); /* LSB */
    cs_high(dev);

    /* Assemble 24-bit value and sign-extend to 32 bits. */
    int32_t raw = ((int32_t)b[0] << 16) | ((int32_t)b[1] << 8) | b[2];
    if (raw & 0x00800000)
    {
        raw |= (int32_t)0xFF000000; /* negative */
    }
    return raw;
}
