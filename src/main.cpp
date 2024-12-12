#include <errno.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dma.h>

static volatile uint64_t _millis = 0;

extern "C" void sys_tick_handler(void) {
    _millis++;
}

static void systick_setup(void) {
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_clear();
    systick_set_reload(rcc_ahb_frequency / 1000 - 1);
    systick_interrupt_enable();
    systick_counter_enable();
}

void delay(uint32_t ms) {
    const uint64_t until = _millis + ms;
    while (_millis < until);
}

static void gpio_setup(void) {
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
}

int pwm_timer_period = 500; // Set period here. freq = (10^6/pwm_timer_period)Hz
// ex: pwm_timer_period=1000 -> freq=1kHz, pwm_timer_period=2000 -> freq=500Hz, pwm_timer_period=500 -> freq=2kHz
void set_pwm(int channel, float duty_cycle) {
    int value = int(duty_cycle * (pwm_timer_period/100.0));
    switch (channel) {
        case 1:
            timer_set_oc_value(TIM1, TIM_OC1, value);
            break;
        case 2:
            timer_set_oc_value(TIM1, TIM_OC2, value);
            break;
        case 3:
            timer_set_oc_value(TIM1, TIM_OC3, value);
            break;
        case 4:
            timer_set_oc_value(TIM1, TIM_OC4, value);
            break;
        default:
            break;
    }
}


void pwm_setup(void) {
    // 1. Enable clocks for GPIOA and Timer1
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_TIM1);

    // 2. Configure GPIO pins PA8, PA9, PA10, and PA11 as alternate function for PWM output
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO8 | GPIO9 | GPIO10 | GPIO11);

    // 3. Set up Timer1 for PWM mode
    timer_set_prescaler(TIM1, 72 - 1);  // Prescaler = 72, for 1 MHz timer frequency
    timer_set_period(TIM1, pwm_timer_period-1);

    // Set the PWM mode to mode 1 (active when counter < CCRx) for each channel
    // Channel 1 on PA8
    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM1, TIM_OC1);  // Enable output compare on channel 1
    // timer_set_oc_value(TIM1, TIM_OC1, int(0 * (freq/100.0)));  // duty cycle
    set_pwm(1, 0);

    // Channel 2 on PA9
    timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM1, TIM_OC2);  // Enable output compare on channel 2
    set_pwm(2, 0);  // duty cycle

    // Channel 3 on PA10
    timer_set_oc_mode(TIM1, TIM_OC3, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM1, TIM_OC3);  // Enable output compare on channel 3
    set_pwm(3, 0);  // duty cycle

    // Channel 4 on PA11
    timer_set_oc_mode(TIM1, TIM_OC4, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM1, TIM_OC4);  // Enable output compare on channel 4
    set_pwm(4, 0);  // duty cycle

    // Enable Timer1 counter and output
    timer_enable_preload(TIM1);
    timer_enable_break_main_output(TIM1);  // Required for advanced timers like TIM1
    timer_enable_counter(TIM1);
}

// #define DIRO GPIO12
// #define TACHO GPIO15
#define DIR GPIO13
#define BRAKE GPIO14

void set_gpio(int pin, int state) {
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, pin);
    if (state) {
        gpio_set(GPIOB, pin);
    } else {
        gpio_clear(GPIOB, pin);
    }
}


int main(void) {
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
    // rcc_clock_setup_in_hse_8mhz_out_72mhz();
    systick_setup();
    gpio_setup();
    pwm_setup();

    rcc_periph_clock_enable(RCC_GPIOB);
    
    set_gpio(DIR, 0);
    set_gpio(BRAKE, 1);

    set_pwm(1, 50);

    while (1) {
        gpio_toggle(GPIOC, GPIO13);
        delay(100);
    }
    return 0;
}