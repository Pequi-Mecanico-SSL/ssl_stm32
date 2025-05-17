#include <errno.h>
#include <stdio.h>
#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>
#include <string>
#include <cmath>
#include <array>

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

int pwm_timer_period = 1028; // for 70khz
//int pwm_timer_period = 400; // Period = 1000, for 1 kHz PWM frequency
void set_pwm(int channel, float duty_cycle) {
    if (duty_cycle < 0 || duty_cycle > 100) {
        return;
    }
    int value = int((duty_cycle/100.0) * pwm_timer_period);
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
    timer_set_prescaler(TIM1, 0);  // 72 MHz frequency
    timer_set_period(TIM1, pwm_timer_period-1);

    // Set the PWM mode to mode 1 (active when counter < CCRx) for each channel
    // Channel 1 on PA8
    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM1, TIM_OC1);  // Enable output compare on channel 1
    // timer_set_oc_value(TIM1, TIM_OC1, int(0 * (freq/100.0)));  // duty cycle
    set_pwm(1, 50);

    // Channel 2 on PA9
    timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM1, TIM_OC2);  // Enable output compare on channel 2
    set_pwm(2, 50);  // duty cycle

    // Channel 3 on PA10
    timer_set_oc_mode(TIM1, TIM_OC3, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM1, TIM_OC3);  // Enable output compare on channel 3
    set_pwm(3, 50);  // duty cycle

    // Channel 4 on PA11
    timer_set_oc_mode(TIM1, TIM_OC4, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM1, TIM_OC4);  // Enable output compare on channel 4
    set_pwm(4, 50);  // duty cycle

    // Enable Timer1 counter and output
    timer_enable_preload(TIM1);
    timer_enable_break_main_output(TIM1);  // Required for advanced timers like TIM1
    timer_enable_counter(TIM1);
}

void spi_slave_setup(void) {
    // Enable clocks for GPIO and SPI peripherals
    rcc_periph_clock_enable(RCC_GPIOA);  // GPIO clock for SPI pins (PA5, PA6, PA7 on STM32F103)
    rcc_periph_clock_enable(RCC_SPI1);   // SPI1 clock

    // Set up GPIO pins for SPI (assuming PA5 for SCK, PA6 for MISO, PA7 for MOSI)
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO5);  // SCK
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO6);  // MISO
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO7);  // MOSI
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO4);  // NSS

    // Configure SPI1 for slave mode
    spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_256, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

    spi_set_slave_mode(SPI1);  // Set SPI1 to slave mode

    //spi_enable_software_slave_management(SPI1);  // Enable software NSS management
    //spi_set_nss_low(SPI1);  // Set NSS to low
    spi_disable_software_slave_management(SPI1);  // Disable software NSS management
    spi_disable_ss_output(SPI1);  // Disable slave select output

    // spi_enable_rx_buffer_not_empty_interrupt(SPI1);
    // nvic_enable_irq(NVIC_SPI1_IRQ);

    spi_enable_rx_dma(SPI1);
    spi_enable_tx_dma(SPI1);

    // Enable SPI
    spi_enable(SPI1);
}

// uint8_t spi_receive_data(void) {
//     while (!(SPI_SR(SPI1) & SPI_SR_RXNE));  // Wait until receive buffer is not empty
//     return spi_read(SPI1);
// }

// void spi_send_data(uint8_t data) {
//     while (!(SPI_SR(SPI1) & SPI_SR_TXE));   // Wait until transmit buffer is empty
//     spi_write(SPI1, data);
// }

// extern "C" void spi1_isr(void) {
//     if (SPI_SR(SPI1) & SPI_SR_RXNE) {  // Check if the RXNE flag is set
//         uint8_t received_data = spi_read(SPI1);  // Read the received byte

//         // Optional: Send a response back immediately
//         spi_write(SPI1, received_data * 4);  // Echo back incremented data, for example

//         // Clear the RXNE flag by reading the data register (already done above)
//     }
// }


#define BUFFER_SIZE 4  // Define the number of bytes to transfer
float tx_buffer[BUFFER_SIZE];  // Buffer to hold data to send
float rx_buffer[BUFFER_SIZE];  // Buffer to hold received data
// volatile uint32_t velocity_cmd[4];  // Buffer to hold data to send
// volatile uint32_t current_velocity[4];  // Buffer to hold received data

void dma_setup(void) {
    /* Enable DMA1 clock */
    rcc_periph_clock_enable(RCC_DMA1);

    /* Configure DMA channel for SPI1 RX (DMA1 Channel 2) */
    dma_channel_reset(DMA1, DMA_CHANNEL2);
    dma_set_peripheral_address(DMA1, DMA_CHANNEL2, (uint32_t)&SPI1_DR); // SPI data register
    dma_set_memory_address(DMA1, DMA_CHANNEL2, (uint32_t)rx_buffer); // RX buffer
    dma_set_number_of_data(DMA1, DMA_CHANNEL2, sizeof(rx_buffer)); // Transfer size
    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL2);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL2);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL2, DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL2, DMA_CCR_MSIZE_8BIT);
    dma_set_priority(DMA1, DMA_CHANNEL2, DMA_CCR_PL_HIGH);
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL2);

    /* Configure DMA channel for SPI1 TX (DMA1 Channel 3) */
    dma_channel_reset(DMA1, DMA_CHANNEL3);
    dma_set_peripheral_address(DMA1, DMA_CHANNEL3, (uint32_t)&SPI1_DR); // SPI data register
    dma_set_memory_address(DMA1, DMA_CHANNEL3, (uint32_t)tx_buffer); // TX buffer
    dma_set_number_of_data(DMA1, DMA_CHANNEL3, sizeof(tx_buffer)); // Transfer size
    dma_set_read_from_memory(DMA1, DMA_CHANNEL3);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL3);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL3, DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL3, DMA_CCR_MSIZE_8BIT);
    dma_set_priority(DMA1, DMA_CHANNEL3, DMA_CCR_PL_HIGH);
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL3);

    nvic_enable_irq(NVIC_DMA1_CHANNEL2_IRQ);  // Enable DMA1 Channel 2 interrupt
    nvic_enable_irq(NVIC_DMA1_CHANNEL3_IRQ);  // Enable DMA1 Channel 3 interrupt

    /* Enable DMA channels */
    dma_enable_channel(DMA1, DMA_CHANNEL2); // RX
    dma_enable_channel(DMA1, DMA_CHANNEL3); // TX
}

extern "C" void dma1_channel2_isr(void) {  // RX complete
    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL2, DMA_TCIF)) {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL2, DMA_TCIF);

        /* Handle received data */
        // Process or store data in rx_buffer here
        // for (int i = 0; i < BUFFER_SIZE; i++) {
        //     tx_buffer[i] = rx_buffer[i] * 2;
        // }

        /* Re-enable the DMA channel for the next transfer */
        // spi_enable_rx_dma(SPI1);
        dma_disable_channel(DMA1, DMA_CHANNEL2);
        dma_set_memory_address(DMA1, DMA_CHANNEL2, (uint32_t)rx_buffer);
        dma_set_number_of_data(DMA1, DMA_CHANNEL2, sizeof(rx_buffer));
        dma_enable_channel(DMA1, DMA_CHANNEL2);
        
        // dma_clear_interrupt_flags(DMA1, DMA_CHANNEL2, DMA_TCIF);
    }
}

// volatile int sent = 10000;
extern "C" void dma1_channel3_isr(void) {  // TX complete
    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL3, DMA_TCIF)) {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL3, DMA_TCIF);

        /* Transmission is complete, prepare for next transfer */
        // Prepare new data in tx_buffer if needed
        // sent += 1000;

        /* Re-enable the DMA channel for the next transfer */
        dma_disable_channel(DMA1, DMA_CHANNEL3);
        dma_set_memory_address(DMA1, DMA_CHANNEL3, (uint32_t)tx_buffer);
        dma_set_number_of_data(DMA1, DMA_CHANNEL3, sizeof(tx_buffer));
        dma_enable_channel(DMA1, DMA_CHANNEL3);
        // dma_clear_interrupt_flags(DMA1, DMA_CHANNEL3, DMA_TCIF);
    }
}

#define DIRO GPIO12
// #define TACHO GPIO15
#define DIR GPIO13

void set_gpio(int pin, int port, int state) {
    gpio_set_mode(port, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, pin);
    if (state) {
        gpio_set(port, pin);
    } else {
        gpio_clear(port, pin);
    }
}

// float period = 0;
// uint64_t prev_measure = 0;
std::array<uint64_t, 4> prev_measures = {0, 0, 0, 0};
std::array<float, 4> periods = {0, 0, 0, 0};
float alpha = 0.95;

static uint32_t read_32bit_time(void) {
    uint16_t high = timer_get_counter(TIM3);
    uint16_t low  = timer_get_counter(TIM2);
    return ((uint32_t)high << 16) | (uint32_t)low;
}

void update_period(int index) {
    volatile uint64_t now = read_32bit_time();
    float measured_period = (now - prev_measures[index]);
    prev_measures[index] = now;
    if (measured_period >= 0) {
        //int direction = gpio_get(GPIOB, DIRO) ? 1 : -1;
        periods[index] = alpha * measured_period + (1 - alpha) * periods[index];
        tx_buffer[index] = periods[index];
    }
}

// EXTI interrupt handler for tacho
extern "C" void exti15_10_isr(void) {
    if (exti_get_flag_status(EXTI15)) {
        update_period(0);
        exti_reset_request(EXTI15); // Clear the interrupt flag
    }
    spi_write(SPI1, tx_buffer[0]);
}

extern "C" void exti4_isr(void) {
    if (exti_get_flag_status(EXTI4)) {
        update_period(1);
        exti_reset_request(EXTI4); // Clear the interrupt flag
    }
    spi_write(SPI1, tx_buffer[0]);
}

extern "C" void exti9_5_isr(void) {
    if (exti_get_flag_status(EXTI6)) {
        update_period(2);
        exti_reset_request(EXTI6); // Clear the interrupt flag
    }
    spi_write(SPI1, tx_buffer[0]);
}

extern "C" void exti1_isr(void) {
    if (exti_get_flag_status(EXTI1)) {
        update_period(3);
        exti_reset_request(EXTI1); // Clear the interrupt flag
    }
    spi_write(SPI1, tx_buffer[0]);
}

void setup_interrupt(uint32_t port, uint32_t pin, uint32_t exti_line, uint32_t irq) {
    gpio_set_mode(port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, pin);
    exti_select_source(exti_line, port);
    exti_set_trigger(exti_line, EXTI_TRIGGER_BOTH);
    exti_enable_request(exti_line);
    nvic_enable_irq(irq);
}

void setup_tacho_diro_timers_and_interrupts() {
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_TIM2);
    rcc_periph_clock_enable(RCC_TIM3);
    rcc_periph_clock_enable(RCC_AFIO);

    // gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, DIRO);

    // gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO15);
    // exti_select_source(EXTI15, GPIOB);
    // exti_set_trigger(EXTI15, EXTI_TRIGGER_BOTH);
    // exti_enable_request(EXTI15);
    // nvic_enable_irq(NVIC_EXTI15_10_IRQ);

    //setup_interrupt(GPIOB, GPIO15, EXTI15, NVIC_EXTI15_10_IRQ);
    setup_interrupt(GPIOB, GPIO4, EXTI4, NVIC_EXTI4_IRQ);
    //setup_interrupt(GPIOB, GPIO6, EXTI6, NVIC_EXTI9_5_IRQ);
    //setup_interrupt(GPIOA, GPIO1, EXTI1, NVIC_EXTI1_IRQ);
    
    timer_set_prescaler(TIM2, 72 - 1);
    timer_set_period(TIM2, 0xFFFF);
    timer_set_master_mode(TIM2, TIM_CR2_MMS_UPDATE);


    timer_set_prescaler(TIM3, 0);
    timer_set_period(TIM3, 0xFFFF);
    timer_slave_set_trigger(TIM3, TIM_SMCR_TS_ITR1);
    timer_slave_set_mode(TIM3, TIM_SMCR_SMS_ECM1);

    timer_enable_counter(TIM3);
    timer_enable_counter(TIM2);
}

static void usart_setup(void) {
    /* Enable clocks for GPIOA and USART1 */
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_USART3);

    /* 
     * Set up GPIOA pin9 (USART1 TX) as alternate function push-pull.
     * Set up GPIOA pin10 (USART1 RX) as input floating (or pull-up).
     */
    gpio_set_mode(GPIOB,
                  GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                  GPIO10);
    gpio_set_mode(GPIOB,
                  GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_FLOAT,
                  GPIO11);

    /* 
     * Configure USART1:
     *  - Baud rate: 115200
     *  - 8 data bits, no parity, 1 stop bit
     *  - Enable transmitter and receiver
     */
    usart_set_baudrate(USART3, 115200);
    usart_set_databits(USART3, 8);
    usart_set_stopbits(USART3, USART_STOPBITS_1);
    usart_set_mode(USART3, USART_MODE_TX_RX);
    usart_set_parity(USART3, USART_PARITY_NONE);
    usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);

    /* Finally, enable the USART. */
    usart_enable(USART3);
}

static void send_uint32_decimal(uint32_t val) {
    /* Convert number to decimal string. */
    char buffer[32];
    int len = snprintf(buffer, sizeof(buffer), "%lu\r\n", (unsigned long)val);

    /* Transmit each character. */
    for (int i = 0; i < len; i++) {
        usart_send_blocking(USART3, buffer[i]);
    }
}

static void send(std::string msg) {
    for (int i = 0; i < msg.length(); i++) {
        usart_send_blocking(USART3, msg[i]);
    }
}

static void send(char *msg) {
    for (int i = 0; msg[i] != '\0'; i++) {
        usart_send_blocking(USART3, msg[i]);
    }
}


int main(void) {
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
    // rcc_clock_setup_in_hse_8mhz_out_72mhz();
    systick_setup();
    gpio_setup();
    pwm_setup();
    dma_setup();
    spi_slave_setup();
    usart_setup();
    // tacho_diro_interrupt_setup();
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    setup_tacho_diro_timers_and_interrupts();
    for (int i = 0; i < BUFFER_SIZE; i++) {
        rx_buffer[i] = 0.0;
        tx_buffer[i] = 0.0;
    }

    rcc_periph_clock_enable(RCC_GPIOB);
    
    //set_gpio(DIR, 0);
    set_gpio(GPIO14, GPIOB, 1);
    set_gpio(GPIO3, GPIOB, 1);
    set_gpio(GPIO7, GPIOB, 1);
    set_gpio(GPIO2, GPIOA, 1);

    set_pwm(1, 50);
    set_pwm(2, 50);
    set_pwm(3, 50);
    set_pwm(4, 50);
    rx_buffer[0] = 50;
    rx_buffer[1] = 50;
    rx_buffer[2] = 50;
    rx_buffer[3] = 50;

    // int vals[] = {0, 200, 400, 600};
    // int j = 1;
    // float amplitude = 20;
    // float offset = 47;
    // float freq = 0.05;
    while (1) {
        gpio_toggle(GPIOC, GPIO13);
        //gpio_toggle(GPIOB, BRAKE);
        delay(50);

        // float pwm_signal = offset + amplitude * std::sin(2 * 3.14159 * freq * _millis / 1000.0);
        // set_pwm(1, pwm_signal);
        // timer_set_oc_value(TIM1, TIM_OC1, 100);
        // for (int i = 0; i < 4; i++) {
        //     vals[i] += 100;
        //     if (vals[i] > 1000) {
        //         vals[i] = 0;
        //     }
        // }
        // timer_set_oc_value(TIM1, TIM_OC1, vals[0]);
        // timer_set_oc_value(TIM1, TIM_OC2, vals[1]);
        // timer_set_oc_value(TIM1, TIM_OC3, vals[2]);
        // timer_set_oc_value(TIM1, TIM_OC4, vals[3]);

        // timer_set_oc_value(TIM1, TIM_OC1, velocity_cmd[0]);
        // timer_set_oc_value(TIM1, TIM_OC2, velocity_cmd[1]);
        // timer_set_oc_value(TIM1, TIM_OC3, velocity_cmd[2]);
        // timer_set_oc_value(TIM1, TIM_OC4, velocity_cmd[3]);
        // current_velocity[0] = velocity_cmd[0];
        // current_velocity[1] = velocity_cmd[1];
        // current_velocity[2] = velocity_cmd[2];
        // current_velocity[3] = velocity_cmd[3];

        // for (int i = 0; i < BUFFER_SIZE; i++) {
        //     tx_buffer[i] = rx_buffer[i];
        // }
        // while(dma_get_interrupt_flag(DMA1, DMA_CHANNEL3, DMA_TCIF));
        // if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL2, DMA_TCIF)) {
        //     gpio_set(GPIOC, GPIO13);
        // } else {
        //     gpio_clear(GPIOC, GPIO13);
        // }

        // char buffer[200];
        // snprintf(buffer, sizeof(buffer), "Period: %d\n", int(period));
        // send(buffer);

        // only write if the ss is high
        if (gpio_get(GPIOA, GPIO4)) {
           static_assert(BUFFER_SIZE == 4);
            set_pwm(1, int(rx_buffer[0]));
            set_pwm(2, int(rx_buffer[1]));
            set_pwm(3, int(rx_buffer[2]));
            set_pwm(4, int(rx_buffer[3]));
            for (int i = 0; i < BUFFER_SIZE; i++) {
                set_pwm(i + 1, rx_buffer[i]);
            }
            // for (int i = 0; i < BUFFER_SIZE; i++) {
            //   tx_buffer[i] = rx_buffer[i];
            // }
            spi_write(SPI1, tx_buffer[0]);
        }
        //for (int i = 0; i < BUFFER_SIZE; i++) {
        //    tx_buffer[i] = j + i;
        //}
        //spi_write(SPI1, tx_buffer[0]);
        //spi_enable_tx_dma(SPI1);
        // j++;
    }
    return 0;
}