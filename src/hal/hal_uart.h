/**
 * @file    hal_uart.h
 * @brief   UART abstraction for USART2 (RS-485 / Modbus link).
 */
#ifndef HAL_UART_H
#define HAL_UART_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/** Initialize USART2 at the given baud (8 data bits, parity per app_config). */
void hal_uart_init(uint32_t baudrate);

/** Transmit a buffer (blocking). Includes turnaround delay and TC flag synchronization. */
void hal_uart_write(const uint8_t *data, size_t len);

/** Non-blocking single-byte read; returns 1 if a byte was available. */
int hal_uart_read_byte(uint8_t *out);

/** Check how many bytes are currently available in RX buffer. */
size_t hal_uart_rx_available(void);

/** Peek at a byte in the RX buffer without removing it. */
int hal_uart_peek_byte(size_t offset, uint8_t *out);

/** Flush / clear the RX buffer. */
void hal_uart_flush_rx(void);

/** 1ms tick called from SysTick_Handler for T3.5 framing. */
void hal_uart_timer_1ms_tick(void);

/** Check if a complete Modbus RTU frame has been detected (T3.5 elapsed). */
bool hal_uart_is_frame_ready(void);

/** Drive the RS-485 transceiver direction (true = transmit). */
void hal_uart_set_tx_enable(int enable);

#endif /* HAL_UART_H */
