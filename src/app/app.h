/**
 * @file    app.h
 * @brief   Top-level application: wires modules together and runs the
 *          super-loop. main() only calls app_init() then app_run().
 */
#ifndef APP_H
#define APP_H

/** Initialize HAL, drivers, services and comms. */
void app_init(void);

/** Run the cooperative super-loop (never returns). */
void app_run(void);

#endif /* APP_H */
