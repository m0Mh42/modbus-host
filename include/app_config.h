/**
 * @file    app_config.h
 * @brief   Application-level constants and tunables (no hardware pins here).
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ---- System ---- */
#define FW_VERSION_MAJOR 0
#define FW_VERSION_MINOR 1
#define FW_VERSION_PATCH 0

/* ---- Channel counts ---- */
#define NUM_THERMOCOUPLES 8
#define NUM_CURRENT_CHANNELS 8
#define NUM_PROXIMITY_INPUTS 2
#define NUM_RELAYS 16
#define NUM_ANALOG_INPUTS 2
#define NUM_PWM_OUTPUTS 2

/* ---- Modbus RTU ---- */
#define MODBUS_SLAVE_ADDRESS 1
#define MODBUS_BAUDRATE 115200
#define MODBUS_PARITY_NONE 0
#define MODBUS_STOP_BITS 1

/* ---- Sampling / control loop ---- */
#define CONTROL_LOOP_PERIOD_MS 100 /* main periodic task period */
#define TC_SAMPLE_PERIOD_MS 250    /* thermocouple refresh */
#define CT_SAMPLE_PERIOD_MS 50     /* current refresh */

#endif /* APP_CONFIG_H */
