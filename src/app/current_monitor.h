/**
 * @file    current_monitor.h
 * @brief   8-channel heater-current monitoring via current transformers.
 *
 * Reads the 8 CT analog inputs through the on-chip ADC and converts each to
 * an RMS / scaled current value.
 */
#ifndef CURRENT_MONITOR_H
#define CURRENT_MONITOR_H

#include <stdint.h>

/** Initialize the ADC channels for the 8 CT inputs. */
void current_monitor_init(void);

/** Periodic task: sample channels and update current readings. */
void current_monitor_task(void);

/** Latest current for channel 0..7, in milliamps (scaled). */
uint16_t current_monitor_get(uint8_t channel);

#endif /* CURRENT_MONITOR_H */
