#include <unity.h>
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
void relay_set(uint8_t ch, bool val) { if (ch < 16) mock_relay[ch] = val; }

void setUp(void) {
    pid_control_init();
    for(int i=0; i<8; i++) {
        mock_temp[i] = 250; /* 25.0 C */
        mock_valid[i] = true;
    }
    for(int i=0; i<16; i++) {
        mock_relay[i] = false;
    }
}

void tearDown(void) {}

void test_pid_initialization(void) {
    TEST_ASSERT_EQUAL(0, pid_get_mode(0));
    TEST_ASSERT_EQUAL(250, pid_get_setpoint(0));
    TEST_ASSERT_EQUAL(20, pid_get_deadzone(0));
    TEST_ASSERT_EQUAL(0, pid_get_heater_relay(0));
    TEST_ASSERT_EQUAL(8, pid_get_blower_relay(0));
}

void test_pid_manual_mode(void) {
    pid_set_mode(0, 0); /* Manual */
    pid_set_manual_duty(0, 500); /* 50% duty */
    
    /* Initially soft pwm counter is 0, so output should trigger heater */
    pid_control_process();
    TEST_ASSERT_TRUE(mock_relay[0]);
    TEST_ASSERT_FALSE(mock_relay[8]);
}

void test_pid_auto_mode_heating(void) {
    pid_set_mode(0, 1); /* Auto */
    pid_set_setpoint(0, 300); /* Target 30.0 C */
    mock_temp[0] = 250; /* Current 25.0 C */
    
    /* 50 error * kp(10) = 500 output. That's 50% duty. */
    pid_control_process();
    
    TEST_ASSERT_TRUE(mock_relay[0]);
    TEST_ASSERT_FALSE(mock_relay[8]);
}

void test_pid_auto_mode_cooling(void) {
    pid_set_mode(0, 1); /* Auto */
    pid_set_setpoint(0, 200); /* Target 20.0 C */
    mock_temp[0] = 250; /* Current 25.0 C */
    
    pid_control_process();
    
    /* Error is negative, should trigger blower (relay 8) */
    TEST_ASSERT_FALSE(mock_relay[0]);
    TEST_ASSERT_TRUE(mock_relay[8]);
}

void test_pid_deadzone(void) {
    pid_set_mode(0, 1); /* Auto */
    pid_set_setpoint(0, 250); 
    pid_set_deadzone(0, 20); /* +/- 2.0 C deadzone */
    
    /* Within deadzone */
    mock_temp[0] = 240; 
    pid_control_process();
    TEST_ASSERT_FALSE(mock_relay[0]);
    TEST_ASSERT_FALSE(mock_relay[8]);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_pid_initialization);
    RUN_TEST(test_pid_manual_mode);
    RUN_TEST(test_pid_auto_mode_heating);
    RUN_TEST(test_pid_auto_mode_cooling);
    RUN_TEST(test_pid_deadzone);
    return UNITY_END();
}
