#include <unity.h>
#include "flash_eeprom.h"
#include "../../src/drivers/flash_eeprom/flash_eeprom.c"
#include "pwm_pid.h"
#include "../../src/app/pwm_pid.c"
#include "register_map.h"
#include "../../src/comm/register_map.c"
#include <string.h>

/* Mock stubs for services referenced by register_map.c */
void thermocouple_init(void) {}
int16_t thermocouple_get_temp(uint8_t ch) { (void)ch; return 0; }
bool thermocouple_is_valid(uint8_t ch) { (void)ch; return true; }
int16_t thermocouple_get_cjc_temp_c10(void) { return 250; }
uint16_t thermocouple_get_faults(void) { return 0; }
void thermocouple_clear_faults(void) {}
uint16_t thermocouple_get_enable_mask(void) { return 0xFF; }
void thermocouple_set_enable_mask(uint16_t mask) { (void)mask; }
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

void analog_input_init(void) {}
uint16_t analog_input_get_mv(uint8_t ch) { (void)ch; return 0; }
uint16_t analog_input_get_faults(void) { return 0; }
void analog_input_clear_faults(void) {}
uint16_t analog_input_get_enable_mask(void) { return 3; }
void analog_input_set_enable_mask(uint16_t mask) { (void)mask; }
uint16_t analog_input_get_divider_ratio_x1000(uint8_t ch) { (void)ch; return 4917; }
void analog_input_set_divider_ratio_x1000(uint8_t ch, uint16_t r) { (void)ch; (void)r; }

void pwm_output_init(void) {}
uint16_t pwm_output_get(uint8_t ch) { (void)ch; return 0; }
void pwm_output_set(uint8_t ch, uint16_t duty) { (void)ch; (void)duty; }
uint16_t pwm_output_get_enable_mask(void) { return 3; }
void pwm_output_set_enable_mask(uint16_t mask) { (void)mask; }

void proximity_init(void) {}
uint32_t proximity_get_count(prox_channel_t ch) { (void)ch; return 0; }
uint32_t proximity_get_speed(prox_channel_t ch) { (void)ch; return 0; }
uint16_t proximity_get_faults(void) { return 0; }
void proximity_clear_faults(void) {}
uint16_t proximity_get_enable_mask(void) { return 3; }
void proximity_set_enable_mask(uint16_t mask) { (void)mask; }
static uint16_t mock_prox_speed_src = 0;
void proximity_set_speed_src(uint16_t src) { mock_prox_speed_src = src; }
uint16_t proximity_get_speed_src(void) { return mock_prox_speed_src; }
static uint16_t mock_prox_speed_setpoint = 0;
void proximity_set_speed_setpoint(uint16_t sp) { mock_prox_speed_setpoint = sp; }
uint16_t proximity_get_speed_setpoint(void) { return mock_prox_speed_setpoint; }
uint32_t proximity_get_effective_speed_hz(void) { return 0; }
uint16_t proximity_get_line_speed_c10(void) { return 0; }
uint32_t proximity_get_meterage_c10(void) { return 0; }
uint32_t proximity_get_meterage_m(void) { return 0; }
void proximity_reset_meterage(void) {}
uint16_t proximity_get_pulses_per_meter(void) { return 100; }
void proximity_set_pulses_per_meter(uint16_t ppm) { (void)ppm; }

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
void pid_set_deadzone(uint8_t ch, uint16_t dz) { (void)ch; (void)dz; }
uint16_t pid_get_heater_relay(uint8_t ch) { (void)ch; return 0xFFFF; }
bool pid_set_heater_relay(uint8_t ch, uint16_t r) { (void)ch; (void)r; return true; }
uint16_t pid_get_blower_relay(uint8_t ch) { (void)ch; return 0xFFFF; }
bool pid_set_blower_relay(uint8_t ch, uint16_t r) { (void)ch; (void)r; return true; }
uint16_t pid_get_manual_duty(uint8_t ch) { (void)ch; return 0; }
void pid_set_manual_duty(uint8_t ch, uint16_t d) { (void)ch; (void)d; }
uint16_t pid_get_action(uint8_t ch) { (void)ch; return 0; }
void pid_set_action(uint8_t ch, uint16_t a) { (void)ch; (void)a; }
uint16_t pid_get_hysteresis(uint8_t ch) { (void)ch; return 0; }
void pid_set_hysteresis(uint8_t ch, uint16_t h) { (void)ch; (void)h; }
uint16_t pid_get_cycle_time(uint8_t ch) { (void)ch; return 20; }
void pid_set_cycle_time(uint8_t ch, uint16_t c) { (void)ch; (void)c; }
int16_t pid_get_sv_high_limit(uint8_t ch) { (void)ch; return 4000; }
void pid_set_sv_high_limit(uint8_t ch, int16_t l) { (void)ch; (void)l; }
int16_t pid_get_sv_low_limit(uint8_t ch) { (void)ch; return 0; }
void pid_set_sv_low_limit(uint8_t ch, int16_t l) { (void)ch; (void)l; }
int16_t pid_get_pv_offset(uint8_t ch) { (void)ch; return 0; }
void pid_set_pv_offset(uint8_t ch, int16_t o) { (void)ch; (void)o; }
uint16_t pid_get_fail_safe_duty(uint8_t ch) { (void)ch; return 0; }
void pid_set_fail_safe_duty(uint8_t ch, uint16_t d) { (void)ch; (void)d; }
int16_t pid_get_alarm_high(uint8_t ch) { (void)ch; return 4200; }
void pid_set_alarm_high(uint8_t ch, int16_t a) { (void)ch; (void)a; }
int16_t pid_get_alarm_low(uint8_t ch) { (void)ch; return 500; }
void pid_set_alarm_low(uint8_t ch, int16_t a) { (void)ch; (void)a; }

bool hal_clock_has_crystal_fault(void) { return false; }
void hal_clock_clear_fault(void) {}
bool hal_gpio_read(uint32_t port, uint32_t pin) { (void)port; (void)pin; return false; }

void setUp(void)
{
    flash_eeprom_factory_reset();
}

void tearDown(void) {}

void test_default_device_names(void)
{
    TEST_ASSERT_EQUAL_STRING("Node-1", flash_eeprom_get_node_name());
    TEST_ASSERT_EQUAL_STRING("Zone-1", flash_eeprom_get_zone_name(0));
    TEST_ASSERT_EQUAL_STRING("Zone-8", flash_eeprom_get_zone_name(7));
    TEST_ASSERT_EQUAL_STRING("AIN-1", flash_eeprom_get_ain_name(0));
    TEST_ASSERT_EQUAL_STRING("PWM-1", flash_eeprom_get_pwm_name(0));
}

void test_persian_naming_direct_api(void)
{
    /* Persian strings in UTF-8 */
    const char *persian_node = "کوره اصلی ۱";
    const char *persian_zone1 = "منطقه پیش‌گرم";
    const char *persian_zone8 = "اتاقک خنک‌کننده";
    const char *persian_ain = "فشار گاز ورودی";
    const char *persian_pwm = "دمنده هوای تازه";

    flash_eeprom_set_node_name(persian_node);
    flash_eeprom_set_zone_name(0, persian_zone1);
    flash_eeprom_set_zone_name(7, persian_zone8);
    flash_eeprom_set_ain_name(0, persian_ain);
    flash_eeprom_set_pwm_name(0, persian_pwm);

    TEST_ASSERT_EQUAL_STRING(persian_node, flash_eeprom_get_node_name());
    TEST_ASSERT_EQUAL_STRING(persian_zone1, flash_eeprom_get_zone_name(0));
    TEST_ASSERT_EQUAL_STRING(persian_zone8, flash_eeprom_get_zone_name(7));
    TEST_ASSERT_EQUAL_STRING(persian_ain, flash_eeprom_get_ain_name(0));
    TEST_ASSERT_EQUAL_STRING(persian_pwm, flash_eeprom_get_pwm_name(0));
}

void test_persian_naming_modbus_registers(void)
{
    const char *persian_zone = "مشعل شماره ۱";
    flash_eeprom_set_zone_name(2, persian_zone);

    /* Read back via Modbus holding registers (16 registers = 32 bytes) */
    char buf[33] = {0};
    for (uint8_t i = 0; i < REG_HOLD_NAME_REGS_PER_NAME; i++)
    {
        uint16_t reg = 0;
        bool ok = register_map_read_holding(REG_HOLD_NAME_ZONE_BASE + (2 * REG_HOLD_NAME_REGS_PER_NAME) + i, &reg);
        TEST_ASSERT_TRUE(ok);
        buf[i * 2] = (char)((reg >> 8) & 0xFF);
        buf[i * 2 + 1] = (char)(reg & 0xFF);
    }
    buf[32] = '\0';
    TEST_ASSERT_EQUAL_STRING(persian_zone, buf);

    /* Now write a new Persian name via Modbus holding registers */
    const char *new_persian_node = "دستگاه آزمون";
    size_t new_len = strlen(new_persian_node);
    for (uint8_t i = 0; i < REG_HOLD_NAME_REGS_PER_NAME; i++)
    {
        uint8_t b0 = (i * 2 < new_len) ? (uint8_t)new_persian_node[i * 2] : 0;
        uint8_t b1 = (i * 2 + 1 < new_len) ? (uint8_t)new_persian_node[i * 2 + 1] : 0;
        uint16_t val = ((uint16_t)b0 << 8) | b1;
        bool ok = register_map_write_holding(REG_HOLD_NAME_NODE_BASE + i, val);
        TEST_ASSERT_TRUE(ok);
    }

    /* Verify written node name via flash_eeprom API */
    TEST_ASSERT_EQUAL_STRING(new_persian_node, flash_eeprom_get_node_name());
}

void test_persian_naming_persistence_and_crc(void)
{
    const char *persian_node = "خط تولید ۲";
    const char *persian_zone = "گرم‌کن اکسترودر";

    flash_eeprom_set_node_name(persian_node);
    flash_eeprom_set_zone_name(3, persian_zone);
    bool saved = flash_eeprom_save();
    TEST_ASSERT_TRUE(saved);

    /* Reinitialize from mock flash */
    bool init_ok = flash_eeprom_init();
    TEST_ASSERT_TRUE(init_ok);
    TEST_ASSERT_EQUAL_STRING(persian_node, flash_eeprom_get_node_name());
    TEST_ASSERT_EQUAL_STRING(persian_zone, flash_eeprom_get_zone_name(3));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_default_device_names);
    RUN_TEST(test_persian_naming_direct_api);
    RUN_TEST(test_persian_naming_modbus_registers);
    RUN_TEST(test_persian_naming_persistence_and_crc);
    return UNITY_END();
}
