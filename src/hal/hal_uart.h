/**
 * @file    hal_uart.h
 * @brief   UART abstraction for USART2 (RS-485 / Modbus link).
 */
#ifndef HAL_UART_H
#define HAL_UART_H

#include <stdint.h>
#include <stddef.h>

/** Initialize USART2 at the given baud (8 data bits, parity per app_config). */
void hal_uart_init(uint32_t baudrate);

/** Transmit a buffer (blocking). RS-485 DE handled by caller/modbus layer. */
void hal_uart_write(const uint8_t *data, size_t len);

/** Non-blocking single-byte read; returns 1 if a byte was available. */
int hal_uart_read_byte(uint8_t *out);

/** Drive the RS-485 transceiver direction (true = transmit). */
void hal_uart_set_tx_enable(int enable);

#endif /* HAL_UART_H */
