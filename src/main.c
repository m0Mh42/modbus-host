/**
 * @file    main.c
 * @brief   Firmware entry point for the GD32F303RCT6 thermocontroller.
 *
 * Keeps main() trivial: all wiring and the run-loop live in app.c.
 */
#include "app.h"

int main(void)
{
    app_init();
    app_run();
    return 0; /* never reached */
}
