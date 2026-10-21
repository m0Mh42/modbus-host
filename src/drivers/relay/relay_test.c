/**
 * @file    relay_test.c
 * @brief   Test routine: Verifies that odd-numbered relay digital outputs are ON.
 *
 * Board:   GD32F303RCT6 Thermocontroller
 * Relays:  16 outputs via ULN2803
 *
 * Hardware Pin Mapping:
 *   Odd-numbered Relays (ON) -> Mask: 0x5555
 *     - Relay 1  -> PD2   (Channel 0)  -> ON
 *     - Relay 3  -> PB3   (Channel 2)  -> ON  [SWJ Remapped: JTAG JTDO released]
 *     - Relay 5  -> PB4   (Channel 4)  -> ON  [SWJ Remapped: JTAG NJTRST released]
 *     - Relay 7  -> PB5   (Channel 6)  -> ON
 *     - Relay 9  -> PC12  (Channel 8)  -> ON
 *     - Relay 11 -> PC11  (Channel 10) -> ON
 *     - Relay 13 -> PC10  (Channel 12) -> ON
 *     - Relay 15 -> PA15  (Channel 14) -> ON  [SWJ Remapped: JTAG JTDI released]
 *
 *   Even-numbered Relays (OFF)
 *     - Relay 2  -> PB9   (Channel 1)  -> OFF
 *     - Relay 4  -> PB8   (Channel 3)  -> OFF
 *     - Relay 6  -> PB7   (Channel 5)  -> OFF
 *     - Relay 8  -> PB6   (Channel 7)  -> OFF
 *     - Relay 10 -> PA12  (Channel 9)  -> OFF
 *     - Relay 12 -> PA11  (Channel 11) -> OFF
 *     - Relay 14 -> PA10  (Channel 13) -> OFF
 *     - Relay 16 -> PA9   (Channel 15) -> OFF
 *
 * Diagnostic UART:
 *   USART2 @ 115200 8N1 (PB10 TX, PB11 RX)
 */

#include "relay_test.h"
#include "relay.h"
#include "hal_time.h"
#include "hal_uart.h"
#include <stdio.h>
#include <string.h>

static void uart_print(const char *str)
{
    hal_uart_write((const uint8_t *)str, strlen(str));
}

static void print_banner(void)
{
    uart_print("\r\n========================================\r\n");
    uart_print(" GD32F303RCT6 Thermocontroller\r\n");
    uart_print(" RELAY DIGITAL OUTPUT TEST PROGRAM\r\n");
    uart_print("========================================\r\n");
    uart_print("Target: Odd-numbered relays ON (1, 3, 5, 7, 9, 11, 13, 15)\r\n");
    uart_print("Even relays OFF (2, 4, 6, 8, 10, 12, 14, 16)\r\n");
    uart_print("Relay mask: 0x5555 (0b0101010101010101)\r\n");
    uart_print("----------------------------------------\r\n");
    uart_print("Pin Status:\r\n");
    uart_print("  Relay  1 [PD2 ]  : ON\r\n");
    uart_print("  Relay  2 [PB9 ]  : OFF\r\n");
    uart_print("  Relay  3 [PB3 ]  : ON  (JTAG Remapped)\r\n");
    uart_print("  Relay  4 [PB8 ]  : OFF\r\n");
    uart_print("  Relay  5 [PB4 ]  : ON  (JTAG Remapped)\r\n");
    uart_print("  Relay  6 [PB7 ]  : OFF\r\n");
    uart_print("  Relay  7 [PB5 ]  : ON\r\n");
    uart_print("  Relay  8 [PB6 ]  : OFF\r\n");
    uart_print("  Relay  9 [PC12]  : ON\r\n");
    uart_print("  Relay 10 [PA12]  : OFF\r\n");
    uart_print("  Relay 11 [PC11]  : ON\r\n");
    uart_print("  Relay 12 [PA11]  : OFF\r\n");
    uart_print("  Relay 13 [PC10]  : ON\r\n");
    uart_print("  Relay 14 [PA10]  : OFF\r\n");
    uart_print("  Relay 15 [PA15]  : ON  (JTAG Remapped)\r\n");
    uart_print("  Relay 16 [PA9 ]  : OFF\r\n");
    uart_print("========================================\r\n\r\n");
}

void relay_test_run(void)
{
    /* 1. Initialize SysTick 1 ms timer */
    hal_time_init();

    /* 2. Initialize diagnostic UART (USART2 @ 115200) */
    hal_uart_init(115200);

    /* 3. Initialize 16 relay outputs and configure SWJ remap (frees PB3, PB4, PA15) */
    relay_init();

    /* Print test banner to UART */
    print_banner();

    /* Energize odd relays: Relay 1, 3, 5, 7, 9, 11, 13, 15 */
    relay_set_mask(RELAY_MASK_ODD_RELAYS);
    uart_print("[TEST] Odd relays (0x5555) energized and held ON.\r\n");

    uint32_t last_heartbeat = hal_millis();
    uint32_t seconds = 0;

    for (;;)
    {
        uint32_t now = hal_millis();
        if (now - last_heartbeat >= 2000)
        {
            last_heartbeat = now;
            seconds += 2;
            char msg[64];
            snprintf(msg, sizeof(msg), "[HEARTBEAT] Up %lu s | Mask=0x%04X (Odd ON)\r\n",
                     (unsigned long)seconds, (unsigned int)relay_get_mask());
            uart_print(msg);
        }
    }
}
