/**
 * @file    board_config.h
 * @brief   Hardware pin map for the GD32F303RCT6 thermocontroller board.
 *
 * Centralizes every MCU pin and peripheral assignment so the rest of the
 * firmware never hard-codes a port/pin.
 * All pin assignments are confirmed against docs/pins.csv (Altium Pin Report).
 */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#ifndef UNIT_TEST
#include "gd32f30x.h"
#else
#include <stdint.h>
#define GPIOA 0x40010800U
#define GPIOB 0x40010C00U
#define GPIOC 0x40011000U
#define GPIOD 0x40011400U
#define GPIO_PIN_0 (1U << 0)
#define GPIO_PIN_1 (1U << 1)
#define GPIO_PIN_2 (1U << 2)
#define GPIO_PIN_3 (1U << 3)
#define GPIO_PIN_4 (1U << 4)
#define GPIO_PIN_5 (1U << 5)
#define GPIO_PIN_6 (1U << 6)
#define GPIO_PIN_7 (1U << 7)
#define GPIO_PIN_8 (1U << 8)
#define GPIO_PIN_9 (1U << 9)
#define GPIO_PIN_10 (1U << 10)
#define GPIO_PIN_11 (1U << 11)
#define GPIO_PIN_12 (1U << 12)
#define GPIO_PIN_13 (1U << 13)
#define GPIO_PIN_14 (1U << 14)
#define GPIO_PIN_15 (1U << 15)
#endif

/* =========================================================================
 * CS1237 - 8x 24-bit ADCs for thermocouples
 * Nets in CSV / Netlist:
 *   SCLK  -> Pin 33 (PB12)
 *   DOUT1 -> Pin 41 (PA8)
 *   DOUT2 -> Pin 40 (PC9)
 *   DOUT3 -> Pin 39 (PC8)
 *   DOUT4 -> Pin 38 (PC7)
 *   DOUT5 -> Pin 37 (PC6)
 *   DOUT6 -> Pin 36 (PB15)
 *   DOUT7 -> Pin 35 (PB14)
 *   DOUT8 -> Pin 34 (PB13)
 * ========================================================================= */
#define TC_CHANNELS 8
#ifndef CS1237_CHANNELS
#define CS1237_CHANNELS TC_CHANNELS
#endif

#define CS1237_SCLK_PORT GPIOB
#define CS1237_SCLK_PIN GPIO_PIN_12

#define CS1237_DOUT1_PORT GPIOA
#define CS1237_DOUT1_PIN GPIO_PIN_8

#define CS1237_DOUT2_PORT GPIOC
#define CS1237_DOUT2_PIN GPIO_PIN_9

#define CS1237_DOUT3_PORT GPIOC
#define CS1237_DOUT3_PIN GPIO_PIN_8

#define CS1237_DOUT4_PORT GPIOC
#define CS1237_DOUT4_PIN GPIO_PIN_7

#define CS1237_DOUT5_PORT GPIOC
#define CS1237_DOUT5_PIN GPIO_PIN_6

#define CS1237_DOUT6_PORT GPIOB
#define CS1237_DOUT6_PIN GPIO_PIN_15

#define CS1237_DOUT7_PORT GPIOB
#define CS1237_DOUT7_PIN GPIO_PIN_14

#define CS1237_DOUT8_PORT GPIOB
#define CS1237_DOUT8_PIN GPIO_PIN_13

/* =========================================================================
 * Current transformers - 8x analog inputs into on-chip ADC0
 * Nets in CSV:
 *   CT_ADC_1 -> Pin 20 (PA4) -> ADC0_IN4
 *   CT_ADC_2 -> Pin 21 (PA5) -> ADC0_IN5
 *   CT_ADC_3 -> Pin 22 (PA6) -> ADC0_IN6
 *   CT_ADC_4 -> Pin 23 (PA7) -> ADC0_IN7
 *   CT_ADC_5 -> Pin 24 (PC4) -> ADC0_IN14
 *   CT_ADC_6 -> Pin 25 (PC5) -> ADC0_IN15
 *   CT_ADC_7 -> Pin 26 (PB0) -> ADC0_IN8
 *   CT_ADC_8 -> Pin 27 (PB1) -> ADC0_IN9
 * ========================================================================= */
#define CT_ADC_CHANNELS 8

#define CT_ADC_1_PORT GPIOA
#define CT_ADC_1_PIN GPIO_PIN_4
#define CT_ADC_1_CHANNEL 4

#define CT_ADC_2_PORT GPIOA
#define CT_ADC_2_PIN GPIO_PIN_5
#define CT_ADC_2_CHANNEL 5

#define CT_ADC_3_PORT GPIOA
#define CT_ADC_3_PIN GPIO_PIN_6
#define CT_ADC_3_CHANNEL 6

#define CT_ADC_4_PORT GPIOA
#define CT_ADC_4_PIN GPIO_PIN_7
#define CT_ADC_4_CHANNEL 7

#define CT_ADC_5_PORT GPIOC
#define CT_ADC_5_PIN GPIO_PIN_4
#define CT_ADC_5_CHANNEL 14

#define CT_ADC_6_PORT GPIOC
#define CT_ADC_6_PIN GPIO_PIN_5
#define CT_ADC_6_CHANNEL 15

#define CT_ADC_7_PORT GPIOB
#define CT_ADC_7_PIN GPIO_PIN_0
#define CT_ADC_7_CHANNEL 8

#define CT_ADC_8_PORT GPIOB
#define CT_ADC_8_PIN GPIO_PIN_1
#define CT_ADC_8_CHANNEL 9

/* Convenience aliases for CT channel numbers */
#define CT_ADC_CH_1 CT_ADC_1_CHANNEL
#define CT_ADC_CH_2 CT_ADC_2_CHANNEL
#define CT_ADC_CH_3 CT_ADC_3_CHANNEL
#define CT_ADC_CH_4 CT_ADC_4_CHANNEL
#define CT_ADC_CH_5 CT_ADC_5_CHANNEL
#define CT_ADC_CH_6 CT_ADC_6_CHANNEL
#define CT_ADC_CH_7 CT_ADC_7_CHANNEL
#define CT_ADC_CH_8 CT_ADC_8_CHANNEL

/* =========================================================================
 * General analog inputs - 2x, each behind a resistor voltage divider
 * Nets in CSV:
 *   ADC1 -> Pin 10 (PC2) -> ADC0_IN12
 *   ADC2 -> Pin 11 (PC3) -> ADC0_IN13
 * ========================================================================= */
#define AIN_COUNT 2
#define AIN1_PORT GPIOC
#define AIN1_PIN GPIO_PIN_2
#define AIN1_ADC_CHANNEL 12

#define AIN2_PORT GPIOC
#define AIN2_PIN GPIO_PIN_3
#define AIN2_ADC_CHANNEL 13

/* Divider ratio = (R_top + R_bottom) / R_bottom
 * Schematic / Netlist:
 *   AIN1: R41 = 4.7k, R44 = 1.2k -> (4.7 + 1.2) / 1.2 = 5.9 / 1.2 = 4.9167
 *   AIN2: R39 = 4.7k, R43 = 1.2k -> (4.7 + 1.2) / 1.2 = 5.9 / 1.2 = 4.9167
 */
#define AIN1_DIVIDER_RATIO (5.9f / 1.2f)
#define AIN2_DIVIDER_RATIO (5.9f / 1.2f)
#define AIN_VREF_MV 3300
#define AIN_ADC_FULL_SCALE 4095

/* =========================================================================
 * PWM outputs - 2x
 * Nets in CSV:
 *   PWM1 -> Pin 16 (PA2) -> TIMER1_CH2
 *   PWM2 -> Pin 17 (PA3) -> TIMER1_CH3
 * ========================================================================= */
#define PWM_COUNT 2
#define PWM_TIMER TIMER1
#define PWM_TIMER_RCU RCU_TIMER1

#define PWM1_PORT GPIOA
#define PWM1_PIN GPIO_PIN_2
#define PWM1_TIMER_CHANNEL TIMER_CH_2

#define PWM2_PORT GPIOA
#define PWM2_PIN GPIO_PIN_3
#define PWM2_TIMER_CHANNEL TIMER_CH_3

#define PWM_PERIOD 999
#define PWM_MAX_DUTY 1000

/* =========================================================================
 * Proximity / Counter inputs - 2x digital inputs (meterage / speed)
 * Nets in CSV:
 *   COUNTER_1 -> Pin 14 (PA0-WKUP) -> EXTI0
 *   COUNTER_2 -> Pin 15 (PA1)      -> EXTI1
 * ========================================================================= */
#define PROX1_PORT GPIOA
#define PROX1_PIN GPIO_PIN_0
#define PROX1_EXTI_LINE EXTI_0
#define PROX1_PORT_SOURCE GPIO_PORT_SOURCE_GPIOA
#define PROX1_PIN_SOURCE GPIO_PIN_SOURCE_0
#define PROX1_IRQn EXTI0_IRQn

#define PROX2_PORT GPIOA
#define PROX2_PIN GPIO_PIN_1
#define PROX2_EXTI_LINE EXTI_1
#define PROX2_PORT_SOURCE GPIO_PORT_SOURCE_GPIOA
#define PROX2_PIN_SOURCE GPIO_PIN_SOURCE_1
#define PROX2_IRQn EXTI1_IRQn

/* =========================================================================
 * Relay outputs - 16x (heater / blower), driven through ULN2803
 * Nets in CSV:
 *   RELAY_1  -> Pin 54 (PD2)
 *   RELAY_2  -> Pin 62 (PB9)
 *   RELAY_3  -> Pin 55 (PB3)   [SWJ remap required: JTAG JTDO]
 *   RELAY_4  -> Pin 61 (PB8)
 *   RELAY_5  -> Pin 56 (PB4)   [SWJ remap required: JTAG NJTRST]
 *   RELAY_6  -> Pin 59 (PB7)
 *   RELAY_7  -> Pin 57 (PB5)
 *   RELAY_8  -> Pin 58 (PB6)
 *   RELAY_9  -> Pin 53 (PC12)
 *   RELAY_10 -> Pin 45 (PA12)
 *   RELAY_11 -> Pin 52 (PC11)
 *   RELAY_12 -> Pin 44 (PA11)
 *   RELAY_13 -> Pin 51 (PC10)
 *   RELAY_14 -> Pin 43 (PA10)
 *   RELAY_15 -> Pin 50 (PA15)  [SWJ remap required: JTAG JTDI]
 *   RELAY_16 -> Pin 42 (PA9)
 * ========================================================================= */
#define RELAY_COUNT 16

#define RELAY_1_PORT GPIOD
#define RELAY_1_PIN GPIO_PIN_2

#define RELAY_2_PORT GPIOB
#define RELAY_2_PIN GPIO_PIN_9

#define RELAY_3_PORT GPIOB
#define RELAY_3_PIN GPIO_PIN_3 /* JTAG JTDO */

#define RELAY_4_PORT GPIOB
#define RELAY_4_PIN GPIO_PIN_8

#define RELAY_5_PORT GPIOB
#define RELAY_5_PIN GPIO_PIN_4 /* JTAG NJTRST */

#define RELAY_6_PORT GPIOB
#define RELAY_6_PIN GPIO_PIN_7

#define RELAY_7_PORT GPIOB
#define RELAY_7_PIN GPIO_PIN_5

#define RELAY_8_PORT GPIOB
#define RELAY_8_PIN GPIO_PIN_6

#define RELAY_9_PORT GPIOC
#define RELAY_9_PIN GPIO_PIN_12

#define RELAY_10_PORT GPIOA
#define RELAY_10_PIN GPIO_PIN_12

#define RELAY_11_PORT GPIOC
#define RELAY_11_PIN GPIO_PIN_11

#define RELAY_12_PORT GPIOA
#define RELAY_12_PIN GPIO_PIN_11

#define RELAY_13_PORT GPIOC
#define RELAY_13_PIN GPIO_PIN_10

#define RELAY_14_PORT GPIOA
#define RELAY_14_PIN GPIO_PIN_10

#define RELAY_15_PORT GPIOA
#define RELAY_15_PIN GPIO_PIN_15 /* JTAG JTDI */

#define RELAY_16_PORT GPIOA
#define RELAY_16_PIN GPIO_PIN_9

/* =========================================================================
 * Ready / Fault Dual-LED indicator
 * Nets in CSV/Netlist:
 *   LED2 (Red = Fault, anode to VDD via R85) -> Pin 28 (PB2)
 *   LED3 (Green = Ready, cathode to GND via R58) -> Pin 28 (PB2)
 * Driving PB2 LOW illuminates Red LED (Fault).
 * Driving PB2 HIGH illuminates Green LED (Ready / Run).
 * ========================================================================= */
#define STATUS_LED_PORT GPIOB
#define STATUS_LED_PIN GPIO_PIN_2

/* =========================================================================
 * Modbus RTU over RS-485 (external transceiver)
 * Nets in CSV:
 *   USART2_TX -> Pin 29 (PB10)
 *   USART2_RX -> Pin 30 (PB11)
 * Direction control: hardware auto-direction (no dedicated MCU DE pin in CSV)
 * ========================================================================= */
#define MODBUS_UART USART2
#define MODBUS_UART_RCU RCU_USART2
#define MODBUS_UART_IRQ USART2_IRQn
#define MODBUS_UART_IRQ_HANDLER USART2_IRQHandler
#define MODBUS_UART_PORT GPIOB
#define MODBUS_UART_TX_PIN GPIO_PIN_10
#define MODBUS_UART_RX_PIN GPIO_PIN_11
#define MODBUS_USE_HW_DE 0

/* =========================================================================
 * DS18B20 - Cold Junction Compensation (1-Wire)
 * Nets in CSV:
 *   DS_ONELINE -> Pin 9 (PC1)
 * ========================================================================= */
#define DS18B20_PORT GPIOC
#define DS18B20_PIN GPIO_PIN_1

/* =========================================================================
 * Modbus Address DIP Switch
 * Nets in CSV:
 *   MODBUS1 -> Pin 2 (PC13) - Address bit 0
 *   MODBUS2 -> Pin 3 (PC14) - Address bit 1
 *   MODBUS3 -> Pin 4 (PC15) - Address bit 2
 *   MODBUS4 -> Pin 8 (PC0)  - Toggle HW (ON) / SW (OFF) address selector
 * ========================================================================= */
#define MODBUS_DIP1_PORT GPIOC
#define MODBUS_DIP1_PIN GPIO_PIN_13

#define MODBUS_DIP2_PORT GPIOC
#define MODBUS_DIP2_PIN GPIO_PIN_14

#define MODBUS_DIP3_PORT GPIOC
#define MODBUS_DIP3_PIN GPIO_PIN_15

#define MODBUS_DIP4_PORT GPIOC
#define MODBUS_DIP4_PIN GPIO_PIN_0

#endif /* BOARD_CONFIG_H */
