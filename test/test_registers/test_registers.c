#include <unity.h>
#include "flash_eeprom.h"
#include "../../src/drivers/flash_eeprom/flash_eeprom.c"
#include "proximity.h"
#include "../../src/drivers/proximity/proximity.c"
#include "pwm_pid.h"
#include "../../src/app/pwm_pid.c"
#include "register_map.h"
#include "../../src/comm/register_map.c"
#include <string.h>

/* Mock stubs for services referenced by register_map.c */
static uint16_t mock_tc_enable = 0x00FF;
static uint16_t mock_tc_faults = 0x0000;
static bool mock_tc_valid[8] = {true, true, true, true, true, true, true, true};

void thermocouple_init(void) {}
int16_t thermocouple_get_temp(uint8_t ch) { (void)ch; return 250; }
bool thermocouple_is_valid(uint8_t ch) { return (ch < 8) ? mock_tc_valid[ch] : false; }
int16_t thermocouple_get_cjc_temp_c10(void) { return 250; }
uint16_t thermocouple_get_faults(void) { return mock_tc_faults; }
void thermocouple_clear_faults(void) { mock_tc_faults = 0; }
uint16_t thermocouple_get_enable_mask(void) { return mock_tc_enable; }
void thermocouple_set_enable_mask(uint16_t mask) { mock_tc_enable = mask; }
int16_t thermocouple_get_offset(uint8_t ch) { (void)ch; return 0; }
void thermocouple_set_offset(uint8_t ch, int16_t off) { (void)ch; (void)off; }

void current_monitor_init(void) {}
uint16_t current_monitor_get(uint8_t ch) { (void)ch; return 0; }
uint16_t current_monitor_get_faults(void) { return 0; }
void current_monitor_clear_faults(void) {}
uint16_t current_monitor_get_enable_mask(void) { return 0xFF; }
void current_monitor_set_enable_mask(uint16_t mask) { (void)mask; }
uint16_t current_monitor_get_bias_mv(void) { return 2495; }
void current_monitor_set_bias_mv(uint16_t mv) { (void)mv; }
uint16_t current_monitor_get_turns_ratio(void) { return 2375; }
void current_monitor_set_turns_ratio(uint16_t r) { (void)r; }
uint16_t current_monitor_get_burden_ohm_x10(void) { return 620; }
void current_monitor_set_burden_ohm_x10(uint16_t b) { (void)b; }

static uint16_t mock_ain_mv[2] = {0, 0};
static uint16_t mock_pwm_duty[2] = {0, 0};

void analog_input_init(void) {}
uint16_t analog_input_get_mv(uint8_t ch) { return (ch < 2) ? mock_ain_mv[ch] : 0; }
uint16_t analog_input_get_faults(void) { return 0; }
void analog_input_clear_faults(void) {}
uint16_t analog_input_get_enable_mask(void) { return 3; }
void analog_input_set_enable_mask(uint16_t mask) { (void)mask; }
uint16_t analog_input_get_divider_ratio_x1000(uint8_t ch) { (void)ch; return 4917; }
void analog_input_set_divider_ratio_x1000(uint8_t ch, uint16_t r) { (void)ch; (void)r; }

void pwm_output_init(void) {}
uint16_t pwm_output_get(uint8_t ch) { return (ch < 2) ? mock_pwm_duty[ch] : 0; }
void pwm_output_set(uint8_t ch, uint16_t duty) { if (ch < 2) mock_pwm_duty[ch] = duty; }
uint16_t pwm_output_get_enable_mask(void) { return 3; }
void pwm_output_set_enable_mask(uint16_t mask) { (void)mask; }

uint16_t relay_get_mask(void) { return 0; }
void relay_set_mask(uint16_t mask) { (void)mask; }
bool relay_get(uint8_t ch) { (void)ch; return false; }
void relay_set(uint8_t ch, bool val) { (void)ch; (void)val; }

void pid_control_init(void) {}
uint16_t pid_get_mode(uint8_t ch) { (void)ch; return 0; }
void pid_set_mode(uint8_t ch, uint16_t m) { (void)ch; (void)m; }
uint16_t pid_get_setpoint(uint8_t ch) { (void)ch; return 0; }
void pid_set_setpoint(uint8_t ch, uint16_t sp) { (void)ch; (void)sp; }
uint16_t pid_get_kp(uint8_t ch) { (void)ch; return 0; }
void pid_set_kp(uint8_t ch, uint16_t v) { (void)ch; (void)v; }
uint16_t pid_get_ki(uint8_t ch) { (void)ch; return 0; }
void pid_set_ki(uint8_t ch, uint16_t v) { (void)ch; (void)v; }
uint16_t pid_get_kd(uint8_t ch) { (void)ch; return 0; }
void pid_set_kd(uint8_t ch, uint16_t v) { (void)ch; (void)v; }
uint16_t pid_get_deadzone(uint8_t ch) { (void)ch; return 0; }
void pid_set_deadzone(uint8_t ch, uint16_t v) { (void)ch; (void)v; }
uint16_t pid_get_heater_relay(uint8_t ch) { (void)ch; return 0; }
bool pid_set_heater_relay(uint8_t ch, uint16_t v) { (void)ch; (void)v; return true; }
uint16_t pid_get_blower_relay(uint8_t ch) { (void)ch; return 0; }
bool pid_set_blower_relay(uint8_t ch, uint16_t v) { (void)ch; (void)v; return true; }
uint16_t pid_get_manual_duty(uint8_t ch) { (void)ch; return 0; }
void pid_set_manual_duty(uint8_t ch, uint16_t v) { (void)ch; (void)v; }

uint16_t pid_get_action(uint8_t ch) { (void)ch; return 0; }
void pid_set_action(uint8_t ch, uint16_t v) { (void)ch; (void)v; }
uint16_t pid_get_hysteresis(uint8_t ch) { (void)ch; return 20; }
void pid_set_hysteresis(uint8_t ch, uint16_t v) { (void)ch; (void)v; }
uint16_t pid_get_cycle_time(uint8_t ch) { (void)ch; return 2; }
void pid_set_cycle_time(uint8_t ch, uint16_t v) { (void)ch; (void)v; }
int16_t pid_get_sv_high_limit(uint8_t ch) { (void)ch; return 13700; }
void pid_set_sv_high_limit(uint8_t ch, int16_t v) { (void)ch; (void)v; }
int16_t pid_get_sv_low_limit(uint8_t ch) { (void)ch; return -2000; }
void pid_set_sv_low_limit(uint8_t ch, int16_t v) { (void)ch; (void)v; }
int16_t pid_get_pv_offset(uint8_t ch) { (void)ch; return 0; }
void pid_set_pv_offset(uint8_t ch, int16_t v) { (void)ch; (void)v; }
uint16_t pid_get_fail_safe_duty(uint8_t ch) { (void)ch; return 0; }
void pid_set_fail_safe_duty(uint8_t ch, uint16_t v) { (void)ch; (void)v; }
int16_t pid_get_alarm_high(uint8_t ch) { (void)ch; return 13700; }
void pid_set_alarm_high(uint8_t ch, int16_t v) { (void)ch; (void)v; }
int16_t pid_get_alarm_low(uint8_t ch) { (void)ch; return -2000; }
void pid_set_alarm_low(uint8_t ch, int16_t v) { (void)ch; (void)v; }

void hal_gpio_init(uint32_t port, uint32_t pin, hal_gpio_mode_t mode) { (void)port; (void)pin; (void)mode; }
bool hal_gpio_read(uint32_t port, uint32_t pin) { (void)port; (void)pin; return false; }
bool hal_clock_has_crystal_fault(void) { return false; }
void hal_clock_clear_fault(void) {}

void setUp(void)
{
    flash_eeprom_factory_reset();
    mock_tc_enable = 0x00FF;
    mock_tc_faults = 0x0000;
    for (int i = 0; i < 8; i++) mock_tc_valid[i] = true;
    proximity_init();
}

void tearDown(void)
{
}

void test_zone_status_registers(void)
{
    /* Zone 0: Working */
    /* Zone 1: Disabled (bit 1 cleared in enable mask) */
    /* Zone 2: Fault (bit 2 set in faults) */
    /* Zone 3: Invalid probe */
    mock_tc_enable = 0x00FD; /* bit 1 = 0 -> Disabled */
    mock_tc_faults = (1U << 2); /* Zone 2 fault */
    mock_tc_valid[3] = false; /* Zone 3 invalid */

    register_map_update_inputs();

    uint16_t status = 999;
    /* Zone 0 (channel 0) -> WORKING */
    TEST_ASSERT_TRUE(register_map_read_input(REG_IN_ZONE_STATUS_BASE + 0, &status));
    TEST_ASSERT_EQUAL_UINT16(ZONE_STATUS_WORKING, status);

    /* Zone 1 (channel 1) -> DISABLED */
    TEST_ASSERT_TRUE(register_map_read_input(REG_IN_ZONE_STATUS_BASE + 1, &status));
    TEST_ASSERT_EQUAL_UINT16(ZONE_STATUS_DISABLED, status);

    /* Zone 2 (channel 2) -> FAULT */
    TEST_ASSERT_TRUE(register_map_read_input(REG_IN_ZONE_STATUS_BASE + 2, &status));
    TEST_ASSERT_EQUAL_UINT16(ZONE_STATUS_FAULT, status);

    /* Zone 3 (channel 3) -> FAULT */
    TEST_ASSERT_TRUE(register_map_read_input(REG_IN_ZONE_STATUS_BASE + 3, &status));
    TEST_ASSERT_EQUAL_UINT16(ZONE_STATUS_FAULT, status);
}

void test_speed_source_and_conversion_factor(void)
{
    /* Set mock speeds directly into internal variables */
    prox_speed[0] = 100; /* PROX1 = 100 Hz */
    prox_speed[1] = 200; /* PROX2 = 200 Hz */

    /* 1. Source = PROX1 (0) */
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_SPEED_PROX_SRC, SPEED_SRC_PROX1));
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PULSES_PER_METER, 100)); /* 100 pulses/m */

    register_map_update_inputs();

    uint16_t eff_hz = 0;
    uint16_t line_speed = 0;
    TEST_ASSERT_TRUE(register_map_read_input(REG_IN_SPEED_HZ, &eff_hz));
    TEST_ASSERT_EQUAL_UINT16(100, eff_hz);

    /* Line speed (0.1 m/min) = (Hz * 600) / ppm = (100 * 600) / 100 = 600 (60.0 m/min) */
    TEST_ASSERT_TRUE(register_map_read_input(REG_IN_LINE_SPEED, &line_speed));
    TEST_ASSERT_EQUAL_UINT16(600, line_speed);

    /* 2. Source = PROX2 (1) */
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_SPEED_PROX_SRC, SPEED_SRC_PROX2));
    register_map_update_inputs();

    TEST_ASSERT_TRUE(register_map_read_input(REG_IN_SPEED_HZ, &eff_hz));
    TEST_ASSERT_EQUAL_UINT16(200, eff_hz);
    TEST_ASSERT_TRUE(register_map_read_input(REG_IN_LINE_SPEED, &line_speed));
    TEST_ASSERT_EQUAL_UINT16(1200, line_speed);

    /* 3. Source = BOTH (2) -> Average = (100 + 200) / 2 = 150 Hz */
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_SPEED_PROX_SRC, SPEED_SRC_BOTH));
    register_map_update_inputs();

    TEST_ASSERT_TRUE(register_map_read_input(REG_IN_SPEED_HZ, &eff_hz));
    TEST_ASSERT_EQUAL_UINT16(150, eff_hz);
    TEST_ASSERT_TRUE(register_map_read_input(REG_IN_LINE_SPEED, &line_speed));
    TEST_ASSERT_EQUAL_UINT16(900, line_speed);

    /* 4. Custom PPM = 200 pulses/m -> Line speed = (150 * 600) / 200 = 450 (45.0 m/min) */
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PULSES_PER_METER, 200));
    register_map_update_inputs();

    TEST_ASSERT_TRUE(register_map_read_input(REG_IN_LINE_SPEED, &line_speed));
    TEST_ASSERT_EQUAL_UINT16(450, line_speed);

    /* 5. Set speed input register (0.1 m/min) */
    uint16_t set_spd = 0;
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_SPEED_SETPOINT, 1200)); /* 120.0 m/min */
    TEST_ASSERT_TRUE(register_map_read_holding(REG_HOLD_SPEED_SETPOINT, &set_spd));
    TEST_ASSERT_EQUAL_UINT16(1200, set_spd);
}

void test_speed_config_persistence(void)
{
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_SPEED_PROX_SRC, SPEED_SRC_BOTH));
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PULSES_PER_METER, 250));
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_SPEED_SETPOINT, 1450));

    /* Commit to flash EEPROM */
    TEST_ASSERT_TRUE(flash_eeprom_save());

    /* Re-init proximity service from flash EEPROM */
    proximity_init();

    TEST_ASSERT_EQUAL_UINT16(SPEED_SRC_BOTH, proximity_get_speed_src());
    TEST_ASSERT_EQUAL_UINT16(250, proximity_get_pulses_per_meter());
    TEST_ASSERT_EQUAL_UINT16(1450, proximity_get_speed_setpoint());
}

void test_machine_run_register_and_coil(void)
{
    /* Machine should default to stopped */
    TEST_ASSERT_FALSE(register_map_is_machine_running());

    uint16_t run_val = 999;
    bool coil_val = true;
    TEST_ASSERT_TRUE(register_map_read_holding(REG_HOLD_MACHINE_RUN, &run_val));
    TEST_ASSERT_EQUAL_UINT16(0, run_val);
    TEST_ASSERT_TRUE(register_map_read_coil(COIL_MACHINE_RUN, &coil_val));
    TEST_ASSERT_FALSE(coil_val);

    /* 1. Start machine via holding register */
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_MACHINE_RUN, 1));
    TEST_ASSERT_TRUE(register_map_is_machine_running());
    TEST_ASSERT_TRUE(register_map_read_holding(REG_HOLD_MACHINE_RUN, &run_val));
    TEST_ASSERT_EQUAL_UINT16(1, run_val);
    TEST_ASSERT_TRUE(register_map_read_coil(COIL_MACHINE_RUN, &coil_val));
    TEST_ASSERT_TRUE(coil_val);

    /* 2. Stop machine via coil */
    TEST_ASSERT_TRUE(register_map_write_coil(COIL_MACHINE_RUN, false));
    TEST_ASSERT_FALSE(register_map_is_machine_running());
    TEST_ASSERT_TRUE(register_map_read_holding(REG_HOLD_MACHINE_RUN, &run_val));
    TEST_ASSERT_EQUAL_UINT16(0, run_val);
}

void test_meterage_counting_and_reset_coil(void)
{
    flash_eeprom_init();
    proximity_init();

    /* Set calibration: 100 pulses per meter */
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PULSES_PER_METER, 100));
    uint16_t ppm_read = 0;
    TEST_ASSERT_TRUE(register_map_read_holding(REG_HOLD_PULSES_PER_METER, &ppm_read));
    TEST_ASSERT_EQUAL_UINT16(100, ppm_read);

    /* Reset meterage first */
    TEST_ASSERT_TRUE(register_map_write_coil(COIL_RESET_METERAGE, true));
    register_map_update_inputs();

    uint16_t m_hi = 0, m_lo = 0;
    TEST_ASSERT_TRUE(register_map_read_input(REG_IN_METERAGE_HI, &m_hi));
    TEST_ASSERT_TRUE(register_map_read_input(REG_IN_METERAGE_LO, &m_lo));
    uint32_t meterage_c10 = ((uint32_t)m_hi << 16) | m_lo;
    TEST_ASSERT_EQUAL_UINT32(0, meterage_c10);

    /* Generate 250 pulses on PROX 1: at 100 ppm, 250 pulses = 2.5 meters = 25 in 0.1m units */
    proximity_set_speed_src(0); /* PROX1 */
    for (int i = 0; i < 250; i++)
    {
        proximity_on_edge(PROX_CHANNEL_1);
    }
    register_map_update_inputs();

    TEST_ASSERT_TRUE(register_map_read_input(REG_IN_METERAGE_HI, &m_hi));
    TEST_ASSERT_TRUE(register_map_read_input(REG_IN_METERAGE_LO, &m_lo));
    meterage_c10 = ((uint32_t)m_hi << 16) | m_lo;
    TEST_ASSERT_EQUAL_UINT32(25, meterage_c10); /* 2.5 meters = 25 */

    uint16_t m_m_hi = 0, m_m_lo = 0;
    TEST_ASSERT_TRUE(register_map_read_input(REG_IN_METERAGE_M_HI, &m_m_hi));
    TEST_ASSERT_TRUE(register_map_read_input(REG_IN_METERAGE_M_LO, &m_m_lo));
    uint32_t meterage_m = ((uint32_t)m_m_hi << 16) | m_m_lo;
    TEST_ASSERT_EQUAL_UINT32(2, meterage_m); /* 2 whole meters */

    /* Zero meterage via coil */
    TEST_ASSERT_TRUE(register_map_write_coil(COIL_RESET_METERAGE, true));
    register_map_update_inputs();

    TEST_ASSERT_TRUE(register_map_read_input(REG_IN_METERAGE_HI, &m_hi));
    TEST_ASSERT_TRUE(register_map_read_input(REG_IN_METERAGE_LO, &m_lo));
    meterage_c10 = ((uint32_t)m_hi << 16) | m_lo;
    TEST_ASSERT_EQUAL_UINT32(0, meterage_c10);

    /* Read coil returns false */
    bool c_val = true;
    TEST_ASSERT_TRUE(register_map_read_coil(COIL_RESET_METERAGE, &c_val));
    TEST_ASSERT_FALSE(c_val);
}

void test_pwm_pid_registers(void)
{
    flash_eeprom_init();
    pwm_pid_init();

    /* Write PWM1 PID parameters */
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM1_PID_MODE, 1));
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM1_PID_SRC, 1)); /* AIN1 */
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM1_PID_SETPOINT, 3500)); /* 3500 mV */
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM1_PID_KP, 250));
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM1_PID_KI, 40));
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM1_PID_KD, 15));
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM1_PID_MIN_DUTY, 100));
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM1_PID_MAX_DUTY, 900));
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM1_PID_SCALE_MUL, 1500));
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM1_PID_SCALE_DIV, 10000));

    /* Read back and verify */
    uint16_t val = 0;
    TEST_ASSERT_TRUE(register_map_read_holding(REG_HOLD_PWM1_PID_MODE, &val));
    TEST_ASSERT_EQUAL_UINT16(1, val);
    TEST_ASSERT_TRUE(register_map_read_holding(REG_HOLD_PWM1_PID_SRC, &val));
    TEST_ASSERT_EQUAL_UINT16(1, val);
    TEST_ASSERT_TRUE(register_map_read_holding(REG_HOLD_PWM1_PID_SETPOINT, &val));
    TEST_ASSERT_EQUAL_UINT16(3500, val);
    TEST_ASSERT_TRUE(register_map_read_holding(REG_HOLD_PWM1_PID_KP, &val));
    TEST_ASSERT_EQUAL_UINT16(250, val);
    TEST_ASSERT_TRUE(register_map_read_holding(REG_HOLD_PWM1_PID_KI, &val));
    TEST_ASSERT_EQUAL_UINT16(40, val);
    TEST_ASSERT_TRUE(register_map_read_holding(REG_HOLD_PWM1_PID_KD, &val));
    TEST_ASSERT_EQUAL_UINT16(15, val);
    TEST_ASSERT_TRUE(register_map_read_holding(REG_HOLD_PWM1_PID_MIN_DUTY, &val));
    TEST_ASSERT_EQUAL_UINT16(100, val);
    TEST_ASSERT_TRUE(register_map_read_holding(REG_HOLD_PWM1_PID_MAX_DUTY, &val));
    TEST_ASSERT_EQUAL_UINT16(900, val);
    TEST_ASSERT_TRUE(register_map_read_holding(REG_HOLD_PWM1_PID_SCALE_MUL, &val));
    TEST_ASSERT_EQUAL_UINT16(1500, val);
    TEST_ASSERT_TRUE(register_map_read_holding(REG_HOLD_PWM1_PID_SCALE_DIV, &val));
    TEST_ASSERT_EQUAL_UINT16(10000, val);

    /* PWM2 scale default and write */
    TEST_ASSERT_TRUE(register_map_read_holding(REG_HOLD_PWM2_PID_SCALE_MUL, &val));
    TEST_ASSERT_EQUAL_UINT16(1000, val);
    TEST_ASSERT_TRUE(register_map_read_holding(REG_HOLD_PWM2_PID_SCALE_DIV, &val));
    TEST_ASSERT_EQUAL_UINT16(1000, val);
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM2_PID_SCALE_MUL, 500));
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM2_PID_SCALE_DIV, 250));
    TEST_ASSERT_TRUE(register_map_read_holding(REG_HOLD_PWM2_PID_SCALE_MUL, &val));
    TEST_ASSERT_EQUAL_UINT16(500, val);
    TEST_ASSERT_TRUE(register_map_read_holding(REG_HOLD_PWM2_PID_SCALE_DIV, &val));
    TEST_ASSERT_EQUAL_UINT16(250, val);
}

void test_pwm_pid_closed_loop_execution(void)
{
    flash_eeprom_init();
    pwm_pid_init();
    mock_pwm_duty[0] = mock_pwm_duty[1] = 0;
    mock_ain_mv[0] = mock_ain_mv[1] = 0;

    /* 1. Setup PWM1 with AIN1 feedback and Scale: mul=1500, div=10000 (10000 mV = 1500 RPM) */
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM1_PID_MODE, 1)); /* Auto PID */
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM1_PID_SRC, 1));  /* AIN1 */
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM1_PID_SCALE_MUL, 1500));
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM1_PID_SCALE_DIV, 10000));
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM1_PID_SETPOINT, 1200)); /* 1200 RPM target */
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM1_PID_KP, 100)); /* Kp = 1.00 */
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM1_PID_KI, 0));
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM1_PID_KD, 0));
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM1_PID_MIN_DUTY, 50));  /* 5.0% min clamp */
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM1_PID_MAX_DUTY, 950)); /* 95.0% max clamp */

    /* 2. When machine is STOPPED: duty MUST be 0 */
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_MACHINE_RUN, 0));
    mock_ain_mv[0] = 5000; /* 5.0V input */
    pwm_pid_process();
    TEST_ASSERT_EQUAL_UINT16(0, pwm_output_get(0));

    /* 3. When machine is RUNNING:
     * AIN1 = 5000 mV (5.00V) -> PV = (5000 * 1500 + 5000) / 10000 = 750 RPM
     * SP = 1200 RPM -> Error = 1200 - 750 = 450 RPM
     * P_term = (450 * 100) / 100 = 450 duty
     */
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_MACHINE_RUN, 1));
    pwm_pid_process();
    TEST_ASSERT_EQUAL_UINT16(450, pwm_output_get(0));

    /* 4. When actual feedback reaches target setpoint (8000 mV = 8.00V = 1200 RPM):
     * Error = 1200 - 1200 = 0 -> clamped to MIN_DUTY = 50
     */
    mock_ain_mv[0] = 8000;
    pwm_pid_process();
    TEST_ASSERT_EQUAL_UINT16(50, pwm_output_get(0));

    /* 5. Direct 1:1 feedback mode on PWM2 (Scale: mul=1000, div=1000):
     * SP = 6000 mV (6.00V), AIN2 = 4000 mV (4.00V) -> Error = 2000 mV
     * With Kp = 20 (0.20): P_term = (2000 * 20) / 100 = 400 duty
     */
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM2_PID_MODE, 1));
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM2_PID_SRC, 2)); /* AIN2 */
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM2_PID_SCALE_MUL, 1000));
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM2_PID_SCALE_DIV, 1000));
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM2_PID_SETPOINT, 6000));
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM2_PID_KP, 20));
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM2_PID_KI, 0));
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM2_PID_KD, 0));
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM2_PID_MIN_DUTY, 0));
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM2_PID_MAX_DUTY, 1000));
    mock_ain_mv[1] = 4000;
    pwm_pid_process();
    TEST_ASSERT_EQUAL_UINT16(400, pwm_output_get(1));

    /* 6. Line Speed auto-tracking when PWM setpoint == 0:
     * PROX1 = 50 Hz, PPM = 100 -> Line Speed = (50 * 600) / 100 = 300 (30.0 m/min)
     * Master Speed Setpoint = 500 (50.0 m/min)
     * PWM1 setpoint = 0 -> tracks Master Setpoint (500)
     * Error = 500 - 300 = 200 -> Duty = (200 * 100) / 100 = 200
     */
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM1_PID_SRC, 0));      /* Line Speed */
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM1_PID_SCALE_MUL, 1000));
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM1_PID_SCALE_DIV, 1000));
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM1_PID_SETPOINT, 0)); /* Track master */
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_SPEED_PROX_SRC, 0));    /* PROX1 */
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PULSES_PER_METER, 100));
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_SPEED_SETPOINT, 500));
    prox_speed[0] = 50;
    register_map_update_inputs();
    pwm_pid_process();
    TEST_ASSERT_EQUAL_UINT16(200, pwm_output_get(0));

    /* 7. Frequency feedback scaling on PROX1 (src=3):
     * 100 pulses/rev -> RPM = Hz * 60 / 100 = Hz * 6 / 10.
     * scale_mul = 6, scale_div = 10.
     * PROX1 = 50 Hz -> Scaled PV = (50 * 6 + 5) / 10 = 30 RPM.
     * SP = 50 RPM -> Error = 20 RPM -> Kp = 100 -> Duty = 20.
     */
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM1_PID_SRC, 3)); /* PROX1_HZ */
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM1_PID_SCALE_MUL, 6));
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM1_PID_SCALE_DIV, 10));
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM1_PID_SETPOINT, 50));
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM1_PID_MIN_DUTY, 0));
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_PWM1_PID_MAX_DUTY, 1000));
    prox_speed[0] = 50;
    pwm_pid_process();
    TEST_ASSERT_EQUAL_UINT16(20, pwm_output_get(0));
}

void test_fault_ack_and_clear_registers_and_coils(void)
{
    mock_tc_faults = 0;
    register_map_update_inputs();

    uint16_t overall = 0;
    uint16_t ack_in = 0;
    bool coil_ack = false;
    TEST_ASSERT_TRUE(register_map_read_input(REG_IN_FAULT_OVERALL, &overall));
    TEST_ASSERT_EQUAL_UINT16(0, overall);
    TEST_ASSERT_TRUE(register_map_read_input(REG_IN_FAULT_ACK, &ack_in));
    TEST_ASSERT_EQUAL_UINT16(0, ack_in);
    TEST_ASSERT_TRUE(register_map_read_coil(COIL_FAULT_ACK, &coil_ack));
    TEST_ASSERT_FALSE(coil_ack);

    /* 1. Simulate active TC fault */
    mock_tc_faults = 0x0001; /* TC1 open */
    register_map_update_inputs();
    TEST_ASSERT_TRUE(register_map_read_input(REG_IN_FAULT_OVERALL, &overall));
    TEST_ASSERT_TRUE((overall & FAULT_OVERALL_MASTER) != 0);
    TEST_ASSERT_TRUE((overall & FAULT_OVERALL_TC) != 0);
    TEST_ASSERT_FALSE((overall & FAULT_OVERALL_ACK) != 0);
    TEST_ASSERT_TRUE(register_map_read_input(REG_IN_FAULT_ACK, &ack_in));
    TEST_ASSERT_EQUAL_UINT16(0, ack_in);

    /* 2. Acknowledge fault via Holding Register REG_HOLD_FAULT_ACK */
    TEST_ASSERT_TRUE(register_map_write_holding(REG_HOLD_FAULT_ACK, 1));
    register_map_update_inputs();
    TEST_ASSERT_TRUE(register_map_read_input(REG_IN_FAULT_OVERALL, &overall));
    TEST_ASSERT_TRUE((overall & FAULT_OVERALL_MASTER) != 0);
    TEST_ASSERT_TRUE((overall & FAULT_OVERALL_ACK) != 0);
    TEST_ASSERT_TRUE(register_map_read_input(REG_IN_FAULT_ACK, &ack_in));
    TEST_ASSERT_EQUAL_UINT16(1, ack_in);
    TEST_ASSERT_TRUE(register_map_read_coil(COIL_FAULT_ACK, &coil_ack));
    TEST_ASSERT_TRUE(coil_ack);

    /* 3. Clear faults via Coil COIL_FAULT_CLEAR */
    TEST_ASSERT_TRUE(register_map_write_coil(COIL_FAULT_CLEAR, true));
    register_map_update_inputs();
    TEST_ASSERT_TRUE(register_map_read_input(REG_IN_FAULT_OVERALL, &overall));
    TEST_ASSERT_EQUAL_UINT16(0, overall);
    TEST_ASSERT_TRUE(register_map_read_input(REG_IN_FAULT_ACK, &ack_in));
    TEST_ASSERT_EQUAL_UINT16(0, ack_in);
    TEST_ASSERT_TRUE(register_map_read_coil(COIL_FAULT_ACK, &coil_ack));
    TEST_ASSERT_FALSE(coil_ack);
}

void test_name_registers_unmapped(void)
{
    uint16_t val = 0;
    /* Former name register space 0x0210..0x02DF must return false on read and write */
    for (uint16_t addr = 0x0210; addr <= 0x02DF; addr++)
    {
        TEST_ASSERT_FALSE(register_map_read_holding(addr, &val));
        TEST_ASSERT_FALSE(register_map_write_holding(addr, 0x1234));
    }
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_zone_status_registers);
    RUN_TEST(test_speed_source_and_conversion_factor);
    RUN_TEST(test_speed_config_persistence);
    RUN_TEST(test_machine_run_register_and_coil);
    RUN_TEST(test_meterage_counting_and_reset_coil);
    RUN_TEST(test_pwm_pid_registers);
    RUN_TEST(test_pwm_pid_closed_loop_execution);
    RUN_TEST(test_fault_ack_and_clear_registers_and_coils);
    RUN_TEST(test_name_registers_unmapped);
    return UNITY_END();
}
