/**
 * @file ds18b20.h
 * @brief DS18B20 1-Wire temperature sensor driver.
 */
#ifndef DS18B20_H
#define DS18B20_H

#include <stdint.h>
#include <stdbool.h>

/** Initialize the DS18B20 pin */
void ds18b20_init(void);

/** Start a temperature conversion. Non-blocking. */
bool ds18b20_start_conversion(void);

/** 
 * Read the converted temperature. 
 * Call this at least 750ms after ds18b20_start_conversion().
 * @param temp_c Pointer to store the read temperature in Celsius.
 * @return true if successful, false if sensor not found or CRC failed.
 */
bool ds18b20_read_temp(float *temp_c);

#endif /* DS18B20_H */
