/**
 * @file hal_adc.c
 * @brief On-chip ADC abstraction (GD32F30x SPL).
 *
 * Single-channel, software-triggered conversions on ADC0. Used for the 8
 * current-transformer inputs and the 2 divided analog inputs. The caller is
 * responsible for having configured the corresponding GPIO pin as analog
 * (see current_monitor / analog_input init).
 */
#include "hal_adc.h"
#include "gd32f30x.h"

void hal_adc_init(void)
{
    /* ADC clock must be <= 14 MHz (GD32F303): derive from APB2 with a divider.
     * APB2 is 120MHz, DIV12 gives 10MHz (DIV8 gave 15MHz, exceeding 14MHz max). */
    rcu_periph_clock_enable(RCU_ADC0);
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV12);

    adc_deinit(ADC0);

    /* Independent mode, single regular channel, right-aligned data. */
    adc_mode_config(ADC_MODE_FREE);
    adc_special_function_config(ADC0, ADC_SCAN_MODE, DISABLE);
    adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, DISABLE);
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
    adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL, 1U);

    /* Software trigger only. */
    adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL,
                                       ADC0_1_2_EXTTRIG_REGULAR_NONE);
    adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);

    adc_enable(ADC0);

    /* Self-calibration (must run after enable + short stabilization). */
    for (volatile int i = 0; i < 10000; i++)
    {
    } /* ~stabilization delay */
    adc_calibration_enable(ADC0);
    adc_enable(ADC0);
}

uint16_t hal_adc_read(uint8_t channel)
{
    /* Clear any stale end-of-conversion flag before starting new conversion */
    adc_flag_clear(ADC0, ADC_FLAG_EOC);

    /* Program the single regular channel with a generous sample time. */
    adc_regular_channel_config(ADC0, 0U, channel, ADC_SAMPLETIME_55POINT5);

    /* Software-trigger one conversion and wait for end-of-conversion. */
    adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);

    uint32_t timeout = 100000U;
    while ((adc_flag_get(ADC0, ADC_FLAG_EOC) == RESET) && --timeout)
    {
    }

    uint16_t val = adc_regular_data_read(ADC0);
    adc_flag_clear(ADC0, ADC_FLAG_EOC);

    return val;
}
