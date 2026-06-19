/**
 * @file    board_config.h
 * @brief   Hardware pin map for the GD32F303RCT6 thermocontroller board.
 *
 * Centralizes every MCU pin and peripheral assignment so the rest of the
 * firmware never hard-codes a port/pin. Values are derived from MainBoard.SchDoc
 * (Altium). Pins marked TODO must be confirmed against the schematic netlist
 * before bring-up.
 */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* SPL header: provides GPIOA/B/C, GPIO_PIN_x, SPIx, TIMERx, USARTx, RCU_* etc.
 * used by the pin/peripheral macros below. */
#include "gd32f30x.h"

/* =========================================================================
 * SPI1 - shared bus for the two ADS1220 thermocouple front-ends
 * Nets: SPI1_SCK, SPI1_MOSI, SPI1_MISO
 * ========================================================================= */
#define ADS_SPI_PERIPH        SPI0           /* GD32 SPI peripheral. TODO confirm (SPI0=PA5/6/7) */
#define ADS_SPI_RCU           RCU_SPI0       /* must match ADS_SPI_PERIPH */
#define ADS_SPI_PORT          GPIOA          /* TODO confirm */
#define ADS_SPI_SCK_PIN       GPIO_PIN_5     /* PA5  - SPI1_SCK  TODO */
#define ADS_SPI_MISO_PIN      GPIO_PIN_6     /* PA6  - SPI1_MISO TODO */
#define ADS_SPI_MOSI_PIN      GPIO_PIN_7     /* PA7  - SPI1_MOSI TODO */

/* ADS1220 #1 (thermocouple channels 0..3) */
#define ADS1_CS_PORT          GPIOB          /* TODO */
#define ADS1_CS_PIN           GPIO_PIN_0     /* ADS1_CS   TODO */
#define ADS1_DRDY_PORT        GPIOB          /* TODO */
#define ADS1_DRDY_PIN         GPIO_PIN_1     /* ADS1_DRDY TODO */

/* ADS1220 #2 (thermocouple channels 4..7) */
#define ADS2_CS_PORT          GPIOB          /* TODO */
#define ADS2_CS_PIN           GPIO_PIN_2     /* ADS2_CS   TODO */
#define ADS2_DRDY_PORT        GPIOB          /* TODO */
#define ADS2_DRDY_PIN         GPIO_PIN_10    /* ADS2_DRDY TODO */

/* =========================================================================
 * Current transformers - 8x analog inputs straight into on-chip ADC
 * Nets: CT_ADC_1 .. CT_ADC_8
 * ========================================================================= */
#define CT_ADC_PORT           GPIOC          /* TODO */
#define CT_ADC_CHANNELS       8
/* Map each logical CT channel to an ADC input channel number. TODO confirm. */
#define CT_ADC_CH_1           0
#define CT_ADC_CH_2           1
#define CT_ADC_CH_3           2
#define CT_ADC_CH_4           3
#define CT_ADC_CH_5           4
#define CT_ADC_CH_6           5
#define CT_ADC_CH_7           6
#define CT_ADC_CH_8           7

/* =========================================================================
 * General analog inputs - 2x, each behind a resistor voltage divider
 * Nets: AIN1, AIN2  (read -> Modbus input registers)
 * ========================================================================= */
#define AIN_COUNT             2
#define AIN1_ADC_CHANNEL      8     /* TODO confirm ADC channel for AIN1 */
#define AIN2_ADC_CHANNEL      9     /* TODO confirm ADC channel for AIN2 */
/* Divider ratio = (R_top + R_bottom) / R_bottom. Used to reconstruct the
 * real input voltage from the voltage seen at the ADC pin. TODO set from BOM. */
#define AIN1_DIVIDER_RATIO    11.0f /* e.g. 100k/10k -> 11.0 */
#define AIN2_DIVIDER_RATIO    11.0f
#define AIN_VREF_MV           3300  /* ADC reference in millivolts */
#define AIN_ADC_FULL_SCALE    4095  /* 12-bit ADC */

/* =========================================================================
 * PWM outputs - 2x, duty driven directly from Modbus holding registers
 * Nets: PWM1, PWM2
 * Pick a timer whose channels are routed to the PWM1/PWM2 pins. TODO confirm.
 * ========================================================================= */
#define PWM_COUNT             2
#define PWM_TIMER             TIMER2          /* TODO confirm timer */
#define PWM1_PORT             GPIOA           /* TODO */
#define PWM1_PIN              GPIO_PIN_6      /* PWM1 -> e.g. TIMER2_CH0 TODO */
#define PWM1_TIMER_CHANNEL    TIMER_CH_0      /* TODO */
#define PWM2_PORT             GPIOA           /* TODO */
#define PWM2_PIN              GPIO_PIN_7      /* PWM2 -> e.g. TIMER2_CH1 TODO */
#define PWM2_TIMER_CHANNEL    TIMER_CH_1      /* TODO */
/* PWM resolution: compare value runs 0..PWM_PERIOD. A Modbus duty register of
 * 0..PWM_MAX_DUTY is rescaled onto 0..PWM_PERIOD. */
#define PWM_PERIOD            999             /* timer auto-reload (period-1) */
#define PWM_MAX_DUTY          1000            /* Modbus duty full-scale */

/* =========================================================================
 * Proximity sensors - 2x digital inputs (meterage counter + speed)
 * Nets: PROX1, PROX2
 * ========================================================================= */
#define PROX1_PORT            GPIOC          /* TODO */
#define PROX1_PIN             GPIO_PIN_8     /* PROX1 TODO */
#define PROX2_PORT            GPIOC          /* TODO */
#define PROX2_PIN             GPIO_PIN_9     /* PROX2 TODO */

/* =========================================================================
 * Relay outputs - 16x (heater / blower), driven through ULN2803
 * Nets: RELAY_1 .. RELAY_16
 * Pins below are placeholders; fill from schematic. TODO confirm all.
 * ========================================================================= */
#define RELAY_COUNT           16
/* Each entry: { port, pin }. See relay.c for the table definition. */

/* =========================================================================
 * Modbus RTU over RS-485 (external MAX485)
 * Nets: USART2_TX, USART2_RX  (+ DE/RE direction control)
 * ========================================================================= */
#define MODBUS_UART           USART2
#define MODBUS_UART_PORT      GPIOA
#define MODBUS_UART_TX_PIN    GPIO_PIN_2     /* PA2 - USART2_TX */
#define MODBUS_UART_RX_PIN    GPIO_PIN_3     /* PA3 - USART2_RX */
/* RS-485 driver direction (DE/RE tied). TODO confirm pin, or NC if auto. */
#define MODBUS_DE_PORT        GPIOA
#define MODBUS_DE_PIN         GPIO_PIN_1     /* TODO confirm */

#endif /* BOARD_CONFIG_H */
