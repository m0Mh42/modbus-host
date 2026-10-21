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

/* ---- Concrete peripheral mapped in board_config.h (USART2 on PB10/PB11) ---- */
#define UART_PERIPH MODBUS_UART
#define UART_RCU MODBUS_UART_RCU
#define UART_IRQ MODBUS_UART_IRQ
#define UART_IRQ_FN MODBUS_UART_IRQ_HANDLER

/* ---- RX ring buffer and framing ---- */
#define RX_BUF_SIZE 256
static volatile uint8_t rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head; /* written by ISR */
static volatile uint16_t rx_tail; /* read by application */
static volatile uint16_t s_rx_timer = 0;
static volatile bool s_rx_active = false;
static volatile bool s_frame_ready = false;
static uint16_t s_t35_ms = 2; /* T3.5 silent timeout in ms */

void hal_uart_init(uint32_t baudrate)
{
    /* Determine T3.5 silent timeout according to baudrate */
    if (baudrate <= 9600U)
    {
        s_t35_ms = 4;
    }
    else if (baudrate <= 19200U)
    {
        s_t35_ms = 3;
    }
    else
    {
        s_t35_ms = 2;
    }

    /* Clocks for the UART pins, the peripheral, and AF. */
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(UART_RCU);

    /* TX = AF push-pull, RX = input pull-up (prevents RS-485 floating line noise). */
    hal_gpio_init(MODBUS_UART_PORT, MODBUS_UART_TX_PIN, HAL_GPIO_ALTERNATE);
    hal_gpio_init(MODBUS_UART_PORT, MODBUS_UART_RX_PIN, HAL_GPIO_INPUT_PULLUP);

#if defined(MODBUS_USE_HW_DE) && MODBUS_USE_HW_DE
    /* RS-485 direction pin: output, default to receive (low). */
    hal_gpio_init(MODBUS_DE_PORT, MODBUS_DE_PIN, HAL_GPIO_OUTPUT);
    hal_gpio_write(MODBUS_DE_PORT, MODBUS_DE_PIN, false);
#endif

    /* Frame: 8 data bits, no parity, 1 stop (Modbus 8N1 default). */
    usart_deinit(UART_PERIPH);
    usart_baudrate_set(UART_PERIPH, baudrate);
    usart_word_length_set(UART_PERIPH, USART_WL_8BIT);
    usart_stop_bit_set(UART_PERIPH, USART_STB_1BIT);
    usart_parity_config(UART_PERIPH, USART_PM_NONE);
    usart_hardware_flow_rts_config(UART_PERIPH, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(UART_PERIPH, USART_CTS_DISABLE);
    usart_transmit_config(UART_PERIPH, USART_TRANSMIT_ENABLE);
    usart_receive_config(UART_PERIPH, USART_RECEIVE_ENABLE);

    /* Reset buffer & framing states */
    rx_head = rx_tail = 0;
    s_rx_timer = 0;
    s_rx_active = false;
    s_frame_ready = false;

    /* RX interrupt configuration: highest preemption priority for RS-485 */
    nvic_irq_enable(UART_IRQ, 0U, 0U);
    usart_interrupt_enable(UART_PERIPH, USART_INT_RBNE);

    usart_enable(UART_PERIPH);
}

void hal_uart_write(const uint8_t *data, size_t len)
{
    if (!data || len == 0)
    {
        return;
    }

    /* Short turnaround delay (~1.5ms at 120MHz CPU clock) allowing the external
     * RS-485 auto-direction circuit (TM74HC04 + RC delay) and master transceiver
     * to switch cleanly to receive mode. */
    for (volatile uint32_t i = 0; i < 30000; i++)
    {
        __NOP();
    }

#if defined(MODBUS_USE_HW_DE) && MODBUS_USE_HW_DE
    hal_gpio_write(MODBUS_DE_PORT, MODBUS_DE_PIN, true);
    for (volatile int i = 0; i < 80; i++)
        __NOP();
#endif

    for (size_t i = 0; i < len; i++)
    {
        while (usart_flag_get(UART_PERIPH, USART_FLAG_TBE) == RESET)
        {
        }
        /* Clear TC flag before sending so we can accurately detect the physical completion */
        usart_flag_clear(UART_PERIPH, USART_FLAG_TC);
        usart_data_transmit(UART_PERIPH, data[i]);
    }

    /* Wait until the last byte has completely exited the shift register onto the wire */
    while (usart_flag_get(UART_PERIPH, USART_FLAG_TC) == RESET)
    {
    }

#if defined(MODBUS_USE_HW_DE) && MODBUS_USE_HW_DE
    for (volatile int i = 0; i < 80; i++)
        __NOP();
    hal_gpio_write(MODBUS_DE_PORT, MODBUS_DE_PIN, false);
#endif
}

int hal_uart_read_byte(uint8_t *out)
{
    if (rx_head == rx_tail)
    {
        return 0; /* empty */
    }
    *out = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUF_SIZE;
    if (rx_head == rx_tail)
    {
        s_frame_ready = false;
    }
    return 1;
}

size_t hal_uart_rx_available(void)
{
    if (rx_head >= rx_tail)
    {
        return (size_t)(rx_head - rx_tail);
    }
    else
    {
        return (size_t)(RX_BUF_SIZE - rx_tail + rx_head);
    }
}

int hal_uart_peek_byte(size_t offset, uint8_t *out)
{
    if (offset >= hal_uart_rx_available())
    {
        return 0;
    }
    size_t idx = (rx_tail + offset) % RX_BUF_SIZE;
    *out = rx_buf[idx];
    return 1;
}

void hal_uart_flush_rx(void)
{
    rx_tail = rx_head;
    s_rx_timer = 0;
    s_rx_active = false;
    s_frame_ready = false;
}

void hal_uart_timer_1ms_tick(void)
{
    if (s_rx_active && !s_frame_ready)
    {
        s_rx_timer++;
        if (s_rx_timer >= s_t35_ms)
        {
            s_rx_active = false;
            s_frame_ready = true;
        }
    }
}

bool hal_uart_is_frame_ready(void)
{
    return s_frame_ready;
}

void hal_uart_set_tx_enable(int enable)
{
#if defined(MODBUS_USE_HW_DE) && MODBUS_USE_HW_DE
    /* DE/RE high = driver enabled (transmit), low = receiver enabled. */
    hal_gpio_write(MODBUS_DE_PORT, MODBUS_DE_PIN, enable ? true : false);
#else
    (void)enable;
#endif
}

/* USART RX ISR: push each received byte into the ring buffer.
 * Reading STAT0 then DATA clears both RBNE and ORERR cleanly. */
void UART_IRQ_FN(void)
{
    uint32_t stat = USART_STAT0(UART_PERIPH);

    if (stat & (USART_STAT0_RBNE | USART_STAT0_ORERR))
    {
        uint8_t b = (uint8_t)(USART_DATA(UART_PERIPH) & 0xFF);

        if (stat & USART_STAT0_RBNE)
        {
            uint16_t next = (rx_head + 1) % RX_BUF_SIZE;
            if (next != rx_tail)
            { /* drop on overflow */
                rx_buf[rx_head] = b;
                rx_head = next;
            }
            s_rx_timer = 0;
            s_rx_active = true;
            s_frame_ready = false;
        }
    }
}
