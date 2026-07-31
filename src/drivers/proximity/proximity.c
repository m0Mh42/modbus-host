/**
 * @file proximity.c
 * @brief Proximity-sensor input implementation.
 *
 * Each sensor produces a pulse train. We count edges (meterage) and estimate
 * speed from the time between edges. proximity_on_edge() is meant to be called
 * from the EXTI ISR for the corresponding pin; proximity_task() ages out the
 * speed to zero when pulses stop.
 *
 * NOTE: EXTI line setup (exti_init / nvic) is board-specific and left as TODO
 * in proximity_init() - wire it to PROX1_PIN / PROX2_PIN.
 */
#include "proximity.h"
#include "board_config.h"
#include "app_config.h"
#include "hal_gpio.h"
#include "hal_time.h"

/* Speed is reported in pulses-per-second (scale to me/min in the app layer
 * using the encoder's pulses-per-meter constant). */
#define PROX_SPEED_TIMEOUT_MS  2000   /* zero the speed after this idle gap */

static volatile uint32_t prox_count[NUM_PROXIMITY_INPUTS];
static volatile uint32_t prox_speed[NUM_PROXIMITY_INPUTS];  /* pulses/sec */
static volatile uint32_t prox_last_ms[NUM_PROXIMITY_INPUTS];

void proximity_init(void)
{
    hal_gpio_init(PROX1_PORT, PROX1_PIN, HAL_GPIO_INPUT_PULLUP);
    hal_gpio_init(PROX2_PORT, PROX2_PIN, HAL_GPIO_INPUT_PULLUP);

    for (int i = 0; i < NUM_PROXIMITY_INPUTS; i++) {
        prox_count[i] = prox_speed[i] = prox_last_ms[i] = 0;
    }

    /* Configure EXTI lines for PROX1/PROX2 (rising edge) */
    rcu_periph_clock_enable(RCU_AF);
    
    /* Assuming PROX1 is PC8 and PROX2 is PC9, map EXTI lines.
     * Note: If pins change, these hardcoded AFIO source selections
     * might need adjusting. */
    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOC, GPIO_PIN_SOURCE_8);
    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOC, GPIO_PIN_SOURCE_9);
    
    exti_init(EXTI_8, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    exti_init(EXTI_9, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    
    nvic_irq_enable(EXTI5_9_IRQn, 2, 0);
}

void EXTI5_9_IRQHandler(void)
{
    if(exti_interrupt_flag_get(EXTI_8) != RESET) {
        exti_interrupt_flag_clear(EXTI_8);
        proximity_on_edge(PROX_CHANNEL_1);
    }
    if(exti_interrupt_flag_get(EXTI_9) != RESET) {
        exti_interrupt_flag_clear(EXTI_9);
        proximity_on_edge(PROX_CHANNEL_2);
    }
}

void proximity_on_edge(prox_channel_t ch)
{
    if (ch >= NUM_PROXIMITY_INPUTS) return;

    uint32_t now = hal_millis();
    uint32_t dt  = now - prox_last_ms[ch];

    prox_count[ch]++;

    /* Instantaneous speed from the inter-pulse interval. */
    if (dt > 0) {
        prox_speed[ch] = 1000u / dt;     /* pulses per second */
    }
    prox_last_ms[ch] = now;
}

void proximity_task(void)
{
    /* Decay speed to zero if no pulses arrive for a while (object stopped). */
    uint32_t now = hal_millis();
    for (int i = 0; i < NUM_PROXIMITY_INPUTS; i++) {
        if ((now - prox_last_ms[i]) > PROX_SPEED_TIMEOUT_MS) {
            prox_speed[i] = 0;
        }
    }
}

uint32_t proximity_get_count(prox_channel_t ch)
{
    if (ch >= NUM_PROXIMITY_INPUTS) return 0;
    return prox_count[ch];
}

uint32_t proximity_get_speed(prox_channel_t ch)
{
    if (ch >= NUM_PROXIMITY_INPUTS) return 0;
    return prox_speed[ch];
}

void proximity_reset_count(prox_channel_t ch)
{
    if (ch < NUM_PROXIMITY_INPUTS) prox_count[ch] = 0;
}
