#include <unity.h>
#include <stdbool.h>
#include <stdint.h>
#include "hal_clock.h"
#include "register_map.h"

/* Include hal_clock.c directly for native testing */
#include "../../src/hal/hal_clock.c"

void setUp(void)
{
    hal_clock_init();
}

void tearDown(void) {}

void test_clock_initial_state_crystal(void)
{
    TEST_ASSERT_TRUE(hal_clock_is_using_crystal());
    TEST_ASSERT_EQUAL(HAL_CLOCK_SOURCE_HXTAL, hal_clock_get_source());
    TEST_ASSERT_FALSE(hal_clock_has_crystal_fault());
}

void test_clock_record_crystal_fault(void)
{
    /* Simulate a crystal error at startup or runtime */
    hal_clock_record_crystal_fault();

    TEST_ASSERT_FALSE(hal_clock_is_using_crystal());
    TEST_ASSERT_EQUAL(HAL_CLOCK_SOURCE_IRC8M, hal_clock_get_source());
    TEST_ASSERT_TRUE(hal_clock_has_crystal_fault());
}

void test_clock_clear_fault(void)
{
    hal_clock_record_crystal_fault();
    TEST_ASSERT_TRUE(hal_clock_has_crystal_fault());

    hal_clock_clear_fault();
    TEST_ASSERT_FALSE(hal_clock_has_crystal_fault());
}

void test_clock_fault_bit_mask(void)
{
    TEST_ASSERT_EQUAL_HEX16(0x0004, FAULT_SYS_CRYSTAL_ERROR);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_clock_initial_state_crystal);
    RUN_TEST(test_clock_record_crystal_fault);
    RUN_TEST(test_clock_clear_fault);
    RUN_TEST(test_clock_fault_bit_mask);
    return UNITY_END();
}
