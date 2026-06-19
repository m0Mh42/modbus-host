/**
 * @file    analog_input.h
 * @brief   2 general analog inputs, each behind a resistor voltage divider.
 *
 * Reads the divided voltage on the on-chip ADC and reconstructs the real
 * input voltage (in millivolts) using the divider ratio from board_config.
 * The result is exposed to Modbus as input registers.
 */
#ifndef ANALOG_INPUT_H
#define ANALOG_INPUT_H

#include <stdint.h>

/** Configure the 2 analog-input pins. */
void analog_input_init(void);

/** Periodic task: sample both channels and update cached values. */
void analog_input_task(void);

/** Reconstructed input voltage for channel 0..1, in millivolts. */
uint16_t analog_input_get_mv(uint8_t channel);

/** Raw 12-bit ADC code for channel 0..1 (diagnostics). */
uint16_t analog_input_get_raw(uint8_t channel);

#endif /* ANALOG_INPUT_H */
