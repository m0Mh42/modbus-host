/**
 * @file ds18b20.c
 * @brief DS18B20 1-Wire temperature sensor driver.
 */
#include "ds18b20.h"
#include "board_config.h"
#include "hal_time.h"

#define DS18B20_CMD_SKIP_ROM 0xCC
#define DS18B20_CMD_CONVERT_T 0x44
#define DS18B20_CMD_READ_SCRATCHPAD 0xBE

static inline void delay_us(uint32_t us)
{
    /* Simple loop delay. Adjust for your clock speed.
       Assuming 120MHz, roughly 120 cycles per us. */
    uint32_t count = us * (120 / 4);
    while (count--)
    {
        __asm__ volatile("nop");
    }
}

static void ds18b20_out(void)
{
    gpio_init(DS18B20_PORT, GPIO_MODE_OUT_OD, GPIO_OSPEED_50MHZ, DS18B20_PIN);
}

static void ds18b20_in(void)
{
    gpio_init(DS18B20_PORT, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, DS18B20_PIN);
}

static void ds18b20_write_bit(uint8_t bit)
{
    ds18b20_out();
    gpio_bit_write(DS18B20_PORT, DS18B20_PIN, RESET);
    delay_us(2);
    if (bit)
    {
        ds18b20_in(); /* Release line early for logic 1 */
    }
    delay_us(60);
    ds18b20_in(); /* Release line */
    delay_us(2);
}

static uint8_t ds18b20_read_bit(void)
{
    uint8_t bit = 0;
    ds18b20_out();
    gpio_bit_write(DS18B20_PORT, DS18B20_PIN, RESET);
    delay_us(2);
    ds18b20_in(); /* Release line to let sensor drive it */
    delay_us(10);
    if (gpio_input_bit_get(DS18B20_PORT, DS18B20_PIN) == SET)
    {
        bit = 1;
    }
    delay_us(50);
    return bit;
}

static void ds18b20_write_byte(uint8_t data)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        ds18b20_write_bit(data & 0x01);
        data >>= 1;
    }
}

static uint8_t ds18b20_read_byte(void)
{
    uint8_t data = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        if (ds18b20_read_bit())
        {
            data |= (1 << i);
        }
    }
    return data;
}

static bool ds18b20_reset(void)
{
    ds18b20_out();
    gpio_bit_write(DS18B20_PORT, DS18B20_PIN, RESET);
    delay_us(500);
    ds18b20_in();
    delay_us(60);

    /* Check for presence pulse */
    bool present = (gpio_input_bit_get(DS18B20_PORT, DS18B20_PIN) == RESET);
    delay_us(440);

    return present;
}

void ds18b20_init(void)
{
    switch (DS18B20_PORT)
    {
    case GPIOA:
        rcu_periph_clock_enable(RCU_GPIOA);
        break;
    case GPIOB:
        rcu_periph_clock_enable(RCU_GPIOB);
        break;
    case GPIOC:
        rcu_periph_clock_enable(RCU_GPIOC);
        break;
    case GPIOD:
        rcu_periph_clock_enable(RCU_GPIOD);
        break;
    default:
        break;
    }
    ds18b20_in();
}

bool ds18b20_start_conversion(void)
{
    if (!ds18b20_reset())
        return false;
    ds18b20_write_byte(DS18B20_CMD_SKIP_ROM);
    ds18b20_write_byte(DS18B20_CMD_CONVERT_T);
    return true;
}

bool ds18b20_read_temp(float *temp_c)
{
    if (!ds18b20_reset())
        return false;
    ds18b20_write_byte(DS18B20_CMD_SKIP_ROM);
    ds18b20_write_byte(DS18B20_CMD_READ_SCRATCHPAD);

    uint8_t scratchpad[9];
    for (uint8_t i = 0; i < 9; i++)
    {
        scratchpad[i] = ds18b20_read_byte();
    }

    /* CRC Check */
    uint8_t crc = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        uint8_t inbyte = scratchpad[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix)
                crc ^= 0x8C;
            inbyte >>= 1;
        }
    }

    if (crc != scratchpad[8])
    {
        return false;
    }

    int16_t raw_temp = (scratchpad[1] << 8) | scratchpad[0];
    *temp_c = (float)raw_temp / 16.0f;

    return true;
}
