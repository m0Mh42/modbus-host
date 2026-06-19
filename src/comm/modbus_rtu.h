/**
 * @file    modbus_rtu.h
 * @brief   Modbus RTU slave (server) over RS-485, built on nanoMODBUS.
 *
 * nanoMODBUS handles framing, CRC and function-code dispatch. This module
 * supplies the transport (UART read/write via the HAL) and the register-access
 * callbacks (which defer to register_map).
 *
 * Dependency: nanoMODBUS (https://github.com/debevv/nanoMODBUS), pulled in via
 * platformio.ini lib_deps.
 */
#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include <stdint.h>

/** Initialize the UART link and create the Modbus server instance. */
void modbus_init(uint8_t slave_address, uint32_t baudrate);

/** Poll the Modbus server once (call frequently from the super-loop). */
void modbus_task(void);

#endif /* MODBUS_RTU_H */
