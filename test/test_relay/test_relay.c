#include <unity.h>
#include <stdbool.h>
#include <stdint.h>
#include "relay.h"
#include "hal_gpio.h"

/* Mock hal_gpio */
static uint32_t last_port;
static uint32_t last_pin;
static bool last_level;

void hal_gpio_init(uint32_t port, uint32_t pin, hal_gpio_mode_t mode)
{
    (void)port;
    (void)pin;
    (void)mode;
}

void hal_gpio_write(uint32_t port, uint32_t pin, bool level)
{
    last_port = port;
    last_pin = pin;
    last_level = level;
}

bool hal_gpio_read(uint32_t port, uint32_t pin)
{
    (void)port;
    (void)pin;
    return false;
}

void hal_gpio_toggle(uint32_t port, uint32_t pin)
{
    (void)port;
    (void)pin;
}

/* Include relay.c directly for native test */
#include "../../src/drivers/relay/relay.c"

void setUp(void)
{
    relay_init();
}

void tearDown(void) {}

void test_relay_initialization(void)
{
    TEST_ASSERT_EQUAL_HEX16(0x0000, relay_get_mask());
    for (uint8_t i = 0; i < RELAY_COUNT; i++)
    {
        TEST_ASSERT_FALSE(relay_get(i));
    }
}

void test_relay_set_odd_relays_mask(void)
{
    /* Set odd relays on: Relay 1, 3, 5, 7, 9, 11, 13, 15 (Channels 0, 2, 4, 6, 8, 10, 12, 14) */
    relay_set_mask(RELAY_MASK_ODD_RELAYS);
    TEST_ASSERT_EQUAL_HEX16(0x5555, relay_get_mask());

    /* Odd numbered relays (1-based) should be ON */
    TEST_ASSERT_TRUE(relay_get(0));   /* Relay 1  (PD2)  */
    TEST_ASSERT_FALSE(relay_get(1));  /* Relay 2  (PB9)  */
    TEST_ASSERT_TRUE(relay_get(2));   /* Relay 3  (PB3)  */
    TEST_ASSERT_FALSE(relay_get(3));  /* Relay 4  (PB8)  */
    TEST_ASSERT_TRUE(relay_get(4));   /* Relay 5  (PB4)  */
    TEST_ASSERT_FALSE(relay_get(5));  /* Relay 6  (PB7)  */
    TEST_ASSERT_TRUE(relay_get(6));   /* Relay 7  (PB5)  */
    TEST_ASSERT_FALSE(relay_get(7));  /* Relay 8  (PB6)  */
    TEST_ASSERT_TRUE(relay_get(8));   /* Relay 9  (PC12) */
    TEST_ASSERT_FALSE(relay_get(9));  /* Relay 10 (PA12) */
    TEST_ASSERT_TRUE(relay_get(10));  /* Relay 11 (PC11) */
    TEST_ASSERT_FALSE(relay_get(11)); /* Relay 12 (PA11) */
    TEST_ASSERT_TRUE(relay_get(12));  /* Relay 13 (PC10) */
    TEST_ASSERT_FALSE(relay_get(13)); /* Relay 14 (PA10) */
    TEST_ASSERT_TRUE(relay_get(14));  /* Relay 15 (PA15) */
    TEST_ASSERT_FALSE(relay_get(15)); /* Relay 16 (PA9)  */
}

void test_relay_all_off(void)
{
    relay_set_mask(RELAY_MASK_ODD_RELAYS);
    TEST_ASSERT_EQUAL_HEX16(0x5555, relay_get_mask());
    relay_all_off();
    TEST_ASSERT_EQUAL_HEX16(0x0000, relay_get_mask());
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_relay_initialization);
    RUN_TEST(test_relay_set_odd_relays_mask);
    RUN_TEST(test_relay_all_off);
    return UNITY_END();
}
