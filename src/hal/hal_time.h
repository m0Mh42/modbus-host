/**
 * @file    hal_time.h
 * @brief   System tick / timing abstraction (SysTick based).
 */
#ifndef HAL_TIME_H
#define HAL_TIME_H

#include <stdint.h>

/** Start the 1 kHz system tick. */
void hal_time_init(void);

/** Milliseconds since boot. */
uint32_t hal_millis(void);

/** Busy-wait delay. */
void hal_delay_ms(uint32_t ms);

#endif /* HAL_TIME_H */
