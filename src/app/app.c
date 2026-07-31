/**
 * @file app.c
 * @brief Top-level application wiring + cooperative super-loop.
 *
 * The Modbus server is polled as fast as possible; sensor sampling and the
 * input-register snapshot run on fixed periods driven by hal_millis().
 * PWM and relay outputs are applied immediately when their Modbus registers
 * are written (inside the register_map callbacks), so they need no periodic task.
 */
#include "app.h"
#include "app_config.h"

#include "hal_time.h"
#include "hal_gpio.h"
#include "hal_spi.h"
#include "hal_adc.h"

#include "thermocouple.h"
#include "current_monitor.h"
#include "analog_input.h"
#include "pwm_output.h"
#include "proximity.h"
#include "relay.h"
#include "modbus_rtu.h"
#include "register_map.h"
#include "pid_control.h"

void app_init(void)
{
    /* --- HAL (shared peripherals first) --- */
    hal_time_init();
    hal_spi_init();
    hal_adc_init();

    /* --- Output actuators (safe state) --- */
    relay_init();          /* all relays de-energized */
    pwm_output_init();     /* both PWM outputs at 0% */

    /* --- Input drivers / services --- */
    proximity_init();
    thermocouple_init();
    pid_control_init();
    current_monitor_init();
    analog_input_init();

    /* --- Comms --- */
    modbus_init(MODBUS_SLAVE_ADDRESS, MODBUS_BAUDRATE);
}

void app_run(void)
{
    uint32_t t_tc = 0, t_ct = 0, t_ctrl = 0;

    for (;;) {
        uint32_t now = hal_millis();

        /* Service Modbus every iteration for low latency. */
        modbus_task();

        /* Thermocouple scan step. */
        if ((now - t_tc) >= TC_SAMPLE_PERIOD_MS) {
            t_tc = now;
            thermocouple_task();
        }

        /* Current + analog input sampling. */
        if ((now - t_ct) >= CT_SAMPLE_PERIOD_MS) {
            t_ct = now;
            current_monitor_task();
            analog_input_task();
        }

        /* Periodic housekeeping + refresh the Modbus input snapshot. */
        if ((now - t_ctrl) >= CONTROL_LOOP_PERIOD_MS) {
            t_ctrl = now;
            proximity_task();
            pid_control_process();
            register_map_update_inputs();
        }
    }
}
