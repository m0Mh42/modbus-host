#include <unity.h>
#include "flash_eeprom.h"
#include "../../src/drivers/flash_eeprom/flash_eeprom.c"
#include "pid_control.h"
#include "../../src/app/pid_control.c"
#include <stdbool.h>
#include <stdint.h>

/* Mock dependencies for native testing */
static int16_t mock_temp[8];
static bool mock_valid[8];
static bool mock_relay[16];

int16_t thermocouple_get_temp(uint8_t channel) { return mock_temp[channel]; }
bool thermocouple_is_valid(uint8_t channel) { return mock_valid[channel]; }
void relay_set(uint8_t ch, bool val)
{
    if (ch < 16)
        mock_relay[ch] = val;
}
void pwm_output_set(uint8_t ch, uint16_t duty)
{
    (void)ch;
    (void)duty;
}

void setUp(void)
{
    pid_control_init();
    for (int i = 0; i < 8; i++)
    {
        mock_temp[i] = 250; /* 25.0 C */
        mock_valid[i] = true;
    }
    for (int i = 0; i < 16; i++)
    {
        mock_relay[i] = false;
    }
}

void tearDown(void) {}

void test_pid_initialization(void)
{
    TEST_ASSERT_EQUAL(0, pid_get_mode(0));
    TEST_ASSERT_EQUAL(250, pid_get_setpoint(0));
    TEST_ASSERT_EQUAL(20, pid_get_deadzone(0));
    TEST_ASSERT_EQUAL(0, pid_get_heater_relay(0));
    TEST_ASSERT_EQUAL(8, pid_get_blower_relay(0));
}

void test_pid_manual_mode(void)
{
    pid_set_mode(0, 0);          /* Manual */
    pid_set_manual_duty(0, 500); /* 50% duty */

    /* Initially soft pwm counter is 0, so output should trigger heater */
    pid_control_process();
    TEST_ASSERT_TRUE(mock_relay[0]);
    TEST_ASSERT_FALSE(mock_relay[8]);
}

void test_pid_auto_mode_heating(void)
{
    pid_set_mode(0, 1);       /* Auto */
    pid_set_setpoint(0, 300); /* Target 30.0 C */
    mock_temp[0] = 250;       /* Current 25.0 C */

    /* 50 error * kp(10) = 500 output. That's 50% duty. */
    pid_control_process();

    TEST_ASSERT_TRUE(mock_relay[0]);
    TEST_ASSERT_FALSE(mock_relay[8]);
}

void test_pid_auto_mode_cooling(void)
{
    pid_set_mode(0, 1);       /* Auto */
    pid_set_setpoint(0, 200); /* Target 20.0 C */
    mock_temp[0] = 250;       /* Current 25.0 C */

    pid_control_process();

    /* Error is negative, should trigger blower (relay 8) */
    TEST_ASSERT_FALSE(mock_relay[0]);
    TEST_ASSERT_TRUE(mock_relay[8]);
}

void test_pid_deadzone(void)
{
    pid_set_mode(0, 1); /* Auto */
    pid_set_setpoint(0, 250);
    pid_set_deadzone(0, 20); /* +/- 2.0 C deadzone */

    /* Within deadzone */
    mock_temp[0] = 240;
    pid_control_process();
    TEST_ASSERT_FALSE(mock_relay[0]);
    TEST_ASSERT_FALSE(mock_relay[8]);
}

void test_pid_relay_exclusivity(void)
{
    /* By default: Zone 0 heater = relay 0, Zone 1 heater = relay 1 */
    TEST_ASSERT_EQUAL(0, pid_get_heater_relay(0));
    TEST_ASSERT_EQUAL(1, pid_get_heater_relay(1));

    /* Setting Zone 1 heater to relay 0 must be rejected (duplicate) */
    TEST_ASSERT_FALSE(pid_set_heater_relay(1, 0));
    TEST_ASSERT_EQUAL(1, pid_get_heater_relay(1));

    /* Setting Zone 1 blower to relay 0 must be rejected (duplicate) */
    TEST_ASSERT_FALSE(pid_set_blower_relay(1, 0));
    TEST_ASSERT_EQUAL(9, pid_get_blower_relay(1));

    /* Unassigning Zone 0 heater */
    TEST_ASSERT_TRUE(pid_set_heater_relay(0, 0xFFFF));
    TEST_ASSERT_EQUAL(0xFFFF, pid_get_heater_relay(0));

    /* Now Zone 1 can claim relay 0 */
    TEST_ASSERT_TRUE(pid_set_heater_relay(1, 0));
    TEST_ASSERT_EQUAL(0, pid_get_heater_relay(1));

    /* Reclaiming relay 0 on Zone 0 must now fail */
    TEST_ASSERT_FALSE(pid_set_heater_relay(0, 0));
}

void test_pid_relay_invalid_pwm_rejected(void)
{
    /* PWM actuators (0x0020, 0x0021) and out-of-range relays (>15) must be rejected */
    TEST_ASSERT_FALSE(pid_set_heater_relay(0, 0x0020));
    TEST_ASSERT_FALSE(pid_set_heater_relay(0, 0x0021));
    TEST_ASSERT_FALSE(pid_set_heater_relay(0, 16));
    TEST_ASSERT_FALSE(pid_set_blower_relay(0, 0x0020));
    TEST_ASSERT_FALSE(pid_set_blower_relay(0, 16));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_pid_initialization);
    RUN_TEST(test_pid_manual_mode);
    RUN_TEST(test_pid_auto_mode_heating);
    RUN_TEST(test_pid_auto_mode_cooling);
    RUN_TEST(test_pid_deadzone);
    RUN_TEST(test_pid_relay_exclusivity);
    RUN_TEST(test_pid_relay_invalid_pwm_rejected);
    return UNITY_END();
}
