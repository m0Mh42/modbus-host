/**
 * @file    hal_clock.h
 * @brief   Clock monitoring and crystal failure recovery for GD32F303.
 *
 * Supports running on external crystal (8 MHz HXTAL -> 120 MHz PLL)
 * with automatic fallback to internal oscillator (IRC8M -> 120 MHz PLL)
 * on startup timeout or runtime crystal error (via Clock Security System & NMI).
 */
#ifndef HAL_CLOCK_H
#define HAL_CLOCK_H

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    HAL_CLOCK_SOURCE_HXTAL, /* Running on external crystal (8 MHz) via PLL (120 MHz) */
    HAL_CLOCK_SOURCE_IRC8M  /* Running on internal oscillator (8 MHz) via PLL (120 MHz) */
} hal_clock_source_t;

/**
 * @brief Initialize clock monitoring and verify external crystal / internal oscillator configuration.
 *
 * Checks current clock configuration, ensures Clock Security System (CKM) is armed
 * when running on external crystal, and synchronizes status flags.
 */
void hal_clock_init(void);

/**
 * @brief Get current system clock source.
 * @return HAL_CLOCK_SOURCE_HXTAL or HAL_CLOCK_SOURCE_IRC8M
 */
hal_clock_source_t hal_clock_get_source(void);

/**
 * @brief Check if currently running on external crystal.
 * @return true if using external crystal, false if using internal oscillator
 */
bool hal_clock_is_using_crystal(void);

/**
 * @brief Check if the external crystal has encountered an error (startup timeout or runtime CKM fault).
 * @return true if crystal error was encountered and system is using internal oscillator
 */
bool hal_clock_has_crystal_fault(void);

/**
 * @brief Record a crystal failure (called by startup fallback or NMI handler).
 */
void hal_clock_record_crystal_fault(void);

/**
 * @brief Clear the latched crystal fault flag.
 */
void hal_clock_clear_fault(void);

#endif /* HAL_CLOCK_H */
