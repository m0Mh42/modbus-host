/**
 * @file hal_spi.c
 * @brief SPI master abstraction for the ADS1220 bus (GD32F30x SPL).
 *
 * Configured for ADS1220: SPI mode 1 (CPOL=0, CPHA=1), MSB first, 8-bit.
 * Chip-select is handled by the ADS1220 driver, not here, so the same bus
 * can serve both devices.
 */
#include "hal_spi.h"
#include "hal_gpio.h"
#include "board_config.h"
#include "gd32f30x.h"

void hal_spi_init(void)
{
    /* 1) Clocks: GPIO port for the SPI pins + the SPI peripheral + AF. */
    rcu_periph_clock_enable(ADS_SPI_RCU);
    rcu_periph_clock_enable(RCU_AF);

    /* 2) Pins: SCK + MOSI as AF push-pull, MISO as floating input. */
    hal_gpio_init(ADS_SPI_PORT, ADS_SPI_SCK_PIN,  HAL_GPIO_ALTERNATE);
    hal_gpio_init(ADS_SPI_PORT, ADS_SPI_MOSI_PIN, HAL_GPIO_ALTERNATE);
    hal_gpio_init(ADS_SPI_PORT, ADS_SPI_MISO_PIN, HAL_GPIO_INPUT);

    /* 3) Peripheral configuration. */
    spi_parameter_struct cfg;
    spi_struct_para_init(&cfg);
    cfg.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    cfg.device_mode          = SPI_MASTER;
    cfg.frame_size           = SPI_FRAMESIZE_8BIT;
    cfg.clock_polarity_phase = SPI_CK_PL_LOW_PH_2EDGE;  /* mode 1 for ADS1220 */
    cfg.nss                  = SPI_NSS_SOFT;
    cfg.prescale             = SPI_PSC_64;   /* keep SCLK well under ADS1220 max */
    cfg.endian               = SPI_ENDIAN_MSB;
    spi_init(ADS_SPI_PERIPH, &cfg);

    spi_enable(ADS_SPI_PERIPH);
}

uint8_t hal_spi_transfer(uint8_t tx)
{
    /* Wait for TX buffer empty, push the byte. */
    while (spi_i2s_flag_get(ADS_SPI_PERIPH, SPI_FLAG_TBE) == RESET) { }
    spi_i2s_data_transmit(ADS_SPI_PERIPH, tx);

    /* Wait for RX buffer not empty, read the byte clocked back. */
    while (spi_i2s_flag_get(ADS_SPI_PERIPH, SPI_FLAG_RBNE) == RESET) { }
    return (uint8_t)spi_i2s_data_receive(ADS_SPI_PERIPH);
}

void hal_spi_transfer_buf(const uint8_t *tx, uint8_t *rx, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uint8_t out = tx ? tx[i] : 0xFF;     /* send dummy 0xFF on read-only */
        uint8_t in  = hal_spi_transfer(out);
        if (rx) rx[i] = in;
    }
}
