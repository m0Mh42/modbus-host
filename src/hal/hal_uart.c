/**
 * @file hal_uart.c
 * @brief UART abstraction for the Modbus link (GD32F30x SPL).
 *
 * Interrupt-driven receive into a ring buffer; blocking transmit. RS-485
 * direction (DE/RE) is toggled around transmits via MODBUS_DE_PIN.
 *
 * NOTE: board_config defines MODBUS_UART as "USART2" (schematic net name).
 * On GD32 the peripheral handles are USART0/1/2 and UART3/4. Make sure the
 * macros below (peripheral, RCU, IRQ, pins) all refer to the SAME peripheral.
 */
#include "hal_uart.h"
#include "hal_gpio.h"
#include "board_config.h"
#include "app_config.h"
#include "gd32f30x.h"

/* ---- Pick the concrete peripheral here (must match board routing). ---- */
#define UART_PERIPH   USART1            /* TODO confirm: GD32 USART1 == "USART2" net? */
#define UART_RCU      RCU_USART1
#define UART_IRQ      USART1_IRQn
#define UART_IRQ_FN   USART1_IRQHandler

/* ---- RX ring buffer ---- */
#define RX_BUF_SIZE   256
static volatile uint8_t  rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head;       /* written by ISR */
static volatile uint16_t rx_tail;       /* read by application */

void hal_uart_init(uint32_t baudrate)
{
    /* Clocks for the UART pins, the peripheral, and AF. */
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(UART_RCU);

    /* TX = AF push-pull, RX = floating input. */
    hal_gpio_init(MODBUS_UART_PORT, MODBUS_UART_TX_PIN, HAL_GPIO_ALTERNATE);
    hal_gpio_init(MODBUS_UART_PORT, MODBUS_UART_RX_PIN, HAL_GPIO_INPUT);

    /* RS-485 direction pin: output, default to receive (low). */
    hal_gpio_init(MODBUS_DE_PORT, MODBUS_DE_PIN, HAL_GPIO_OUTPUT);
    hal_gpio_write(MODBUS_DE_PORT, MODBUS_DE_PIN, false);

    /* Frame: 8 data bits, no parity, 1 stop (Modbus 8N1 default). Edit to
     * match MODBUS_PARITY_* / MODBUS_STOP_BITS in app_config if needed. */
    usart_deinit(UART_PERIPH);
    usart_baudrate_set(UART_PERIPH, baudrate);
    usart_word_length_set(UART_PERIPH, USART_WL_8BIT);
    usart_stop_bit_set(UART_PERIPH, USART_STB_1BIT);
    usart_parity_config(UART_PERIPH, USART_PM_NONE);
    usart_transmit_config(UART_PERIPH, USART_TRANSMIT_ENABLE);
    usart_receive_config(UART_PERIPH, USART_RECEIVE_ENABLE);

    /* RX-not-empty interrupt for the ring buffer. */
    rx_head = rx_tail = 0;
    nvic_irq_enable(UART_IRQ, 1U, 0U);
    usart_interrupt_enable(UART_PERIPH, USART_INT_RBNE);

    usart_enable(UART_PERIPH);
}

void hal_uart_write(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        while (usart_flag_get(UART_PERIPH, USART_FLAG_TBE) == RESET) { }
        usart_data_transmit(UART_PERIPH, data[i]);
    }
    /* Wait for the last byte to fully shift out before releasing the bus. */
    while (usart_flag_get(UART_PERIPH, USART_FLAG_TC) == RESET) { }
}

int hal_uart_read_byte(uint8_t *out)
{
    if (rx_head == rx_tail) {
        return 0;                       /* empty */
    }
    *out = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUF_SIZE;
    return 1;
}

void hal_uart_set_tx_enable(int enable)
{
    /* DE/RE high = driver enabled (transmit), low = receiver enabled. */
    hal_gpio_write(MODBUS_DE_PORT, MODBUS_DE_PIN, enable ? true : false);
}

/* USART RX ISR: push each received byte into the ring buffer. */
void UART_IRQ_FN(void)
{
    if (usart_interrupt_flag_get(UART_PERIPH, USART_INT_FLAG_RBNE) != RESET) {
        uint8_t b = (uint8_t)usart_data_receive(UART_PERIPH);
        uint16_t next = (rx_head + 1) % RX_BUF_SIZE;
        if (next != rx_tail) {          /* drop on overflow */
            rx_buf[rx_head] = b;
            rx_head = next;
        }
    }
}
