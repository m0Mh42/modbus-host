/**
 * @file    hal_adc.h
 * @brief   On-chip ADC abstraction for the 8 current-transformer inputs.
 */
#ifndef HAL_ADC_H
#define HAL_ADC_H

#include <stdint.h>

/** Initialize the ADC and its input channels (CT_ADC_1..8). */
void hal_adc_init(void);

/** Read a single raw conversion (0..4095) from an ADC channel. */
uint16_t hal_adc_read(uint8_t channel);

#endif /* HAL_ADC_H */
