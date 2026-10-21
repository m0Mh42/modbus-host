/**
 * @file    status_led.h
 * @brief   Ready / Fault Dual-LED indicator driver (PB2).
 *
 * Hardware connection:
 *  - LED2 (RED = Fault, anode to 3.3V via R85) -> PB2
 *  - LED3 (GREEN = Ready/Run, cathode to GND via R58) -> PB2
 *
 * Driving PB2 LOW energizes Red LED (Fault).
 * Driving PB2 HIGH energizes Green LED (Ready / Run).
 * Setting PB2 to Input / Tri-state turns both LEDs OFF.
 */
#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    STATUS_LED_OFF = 0,
    STATUS_LED_READY_SOLID,     /* Solid Green */
    STATUS_LED_READY_HEARTBEAT, /* Green heartbeat flash */
    STATUS_LED_FAULT_SOLID,     /* Solid Red */
    STATUS_LED_FAULT_FLASH,     /* Flashing Red */
} status_led_mode_t;

/** Initialize the status LED GPIO (PB2) and set initial state. */
void status_led_init(void);

/** Set the operational mode of the status LED. */
void status_led_set_mode(status_led_mode_t mode);

/**
 * @brief Update the status LED based on active system fault and acknowledgment status.
 * @param has_fault True if any fault condition is active, false if system is healthy.
 * @param is_acknowledged True if active faults have been acknowledged (solid red instead of flash).
 */
void status_led_set_fault_state(bool has_fault, bool is_acknowledged);

/**
 * @brief Periodic service task for LED blinking / heartbeat.
 *        Call periodically (e.g. every 50ms or 100ms in app_run).
 */
void status_led_task(void);

#endif /* STATUS_LED_H */
