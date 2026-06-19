/**
 * @file    hal_spi.h
 * @brief   SPI master abstraction for the ADS1220 bus (SPI1).
 */
#ifndef HAL_SPI_H
#define HAL_SPI_H

#include <stdint.h>
#include <stddef.h>

/** Initialize SPI1 in master mode (ADS1220: SPI mode 1, MSB first). */
void hal_spi_init(void);

/** Full-duplex byte transfer. */
uint8_t hal_spi_transfer(uint8_t tx);

/** Block transfer; rx or tx may be NULL. */
void hal_spi_transfer_buf(const uint8_t *tx, uint8_t *rx, size_t len);

#endif /* HAL_SPI_H */
