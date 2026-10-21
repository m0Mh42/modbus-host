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
#include "board_config.h"

#include "hal_time.h"
#include "hal_gpio.h"
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
#include "pwm_pid.h"
#include "flash_eeprom.h"
#include "status_led.h"

static uint8_t read_modbus_address(void)
{
    /* Initialize DIP switch pins as pull-up inputs */
    hal_gpio_init(MODBUS_DIP1_PORT, MODBUS_DIP1_PIN, HAL_GPIO_INPUT_PULLUP);
    hal_gpio_init(MODBUS_DIP2_PORT, MODBUS_DIP2_PIN, HAL_GPIO_INPUT_PULLUP);
    hal_gpio_init(MODBUS_DIP3_PORT, MODBUS_DIP3_PIN, HAL_GPIO_INPUT_PULLUP);
    hal_gpio_init(MODBUS_DIP4_PORT, MODBUS_DIP4_PIN, HAL_GPIO_INPUT_PULLUP);

    /* Allow pull-up resistors and pin lines to stabilize */
    for (volatile int i = 0; i < 5000; i++)
    {
        __NOP();
    }

    /* Read switch inputs: active LOW when switch is ON (connected to GND) */
    bool sw1 = !hal_gpio_read(MODBUS_DIP1_PORT, MODBUS_DIP1_PIN);
    bool sw2 = !hal_gpio_read(MODBUS_DIP2_PORT, MODBUS_DIP2_PIN);
    bool sw3 = !hal_gpio_read(MODBUS_DIP3_PORT, MODBUS_DIP3_PIN);
    bool sw4 = !hal_gpio_read(MODBUS_DIP4_PORT, MODBUS_DIP4_PIN);

    /* Per note.txt:
     * "Toggle hardware or software modbus address usage using the 4th pin on dip switch modbus address selector"
     * ALL OFF (open/high): use configured software address (MODBUS_SLAVE_ADDRESS)
     * ONE OF SW++ ON (closed/low): use 3-bit hardware address from SW1..3
     */
    if (!(sw1 || sw2 || sw3 || sw4))
    {
        return MODBUS_SLAVE_ADDRESS;
    }

    uint8_t hw_addr = (sw1 ? 1 : 0) | ((sw2 ? 1 : 0) << 1) | ((sw3 ? 1 : 0) << 2);
    return (hw_addr > 0) ? hw_addr : 1;
}

void app_init(void)
{
    /* --- Early Safety State (release JTAG pins to GPIO and force all relays OFF immediately) --- */
    relay_early_init();

    /* --- HAL (shared peripherals first) --- */
    hal_time_init();
    hal_adc_init();

    /* --- Persistent Storage (load configs or save defaults) --- */
    flash_eeprom_init();

    /* --- Comms (initialize early so yield callbacks and interrupts are active) --- */
    uint8_t slave_addr = read_modbus_address();
    modbus_init(slave_addr, MODBUS_BAUDRATE);

    /* --- Output actuators (safe state) --- */
    relay_init();      /* all relays de-energized */
    pwm_output_init(); /* both PWM outputs at 0% */

    /* --- Input drivers / services (loads persisted config) --- */
    proximity_init();
    thermocouple_init();
    pid_control_init();
    pwm_pid_init();
    current_monitor_init();
    analog_input_init();
    status_led_init();
    register_map_update_inputs();
}

void app_run(void)
{
    uint32_t t_tc = 0, t_ct = 0, t_ctrl = 0;

    for (;;)
    {
        uint32_t now = hal_millis();

        /* Service Modbus every iteration for low latency. */
        modbus_task();

        /* Service Flash EEPROM debounced auto-save timer. */
        flash_eeprom_task();

        /* Service Ready / Fault LED indicator phase. */
        status_led_task();

        /* Thermocouple scan step. */
        if ((now - t_tc) >= TC_SAMPLE_PERIOD_MS)
        {
            thermocouple_task();
            t_tc = hal_millis(); /* Record timestamp AFTER task completes to ensure loop spacing */
        }

        /* Current + analog input sampling. */
        if ((now - t_ct) >= CT_SAMPLE_PERIOD_MS)
        {
            t_ct = now;
            current_monitor_task();
            analog_input_task();
        }

        /* Periodic housekeeping + refresh the Modbus input snapshot. */
        if ((now - t_ctrl) >= CONTROL_LOOP_PERIOD_MS)
        {
            t_ctrl = now;
            proximity_task();
            pid_control_process();
            pwm_pid_process();
            register_map_update_inputs();
            uint16_t f_overall = register_map_get_fault_overall();
            status_led_set_fault_state((f_overall & FAULT_OVERALL_MASTER) != 0,
                                       register_map_is_fault_acknowledged());
        }
    }
}
