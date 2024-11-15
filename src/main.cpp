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


void pwm_setup(void) {
    // 1. Enable clocks for GPIOA and Timer1
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_TIM1);

    // 2. Configure GPIO pins PA8, PA9, PA10, and PA11 as alternate function for PWM output
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO8 | GPIO9 | GPIO10 | GPIO11);

    // 3. Set up Timer1 for PWM mode
    timer_set_prescaler(TIM1, 72 - 1);  // Prescaler = 72, for 1 MHz timer frequency
    timer_set_period(TIM1, 1000);  // Period = 1000, for 1 kHz PWM frequency

    // Set the PWM mode to mode 1 (active when counter < CCRx) for each channel
    // Channel 1 on PA8
    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM1, TIM_OC1);  // Enable output compare on channel 1
    timer_set_oc_value(TIM1, TIM_OC1, 0);  // duty cycle

    // Channel 2 on PA9
    timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM1, TIM_OC2);  // Enable output compare on channel 2
    timer_set_oc_value(TIM1, TIM_OC2, 0);  // duty cycle

    // Channel 3 on PA10
    timer_set_oc_mode(TIM1, TIM_OC3, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM1, TIM_OC3);  // Enable output compare on channel 3
    timer_set_oc_value(TIM1, TIM_OC3, 0);  // duty cycle

    // Channel 4 on PA11
    timer_set_oc_mode(TIM1, TIM_OC4, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM1, TIM_OC4);  // Enable output compare on channel 4
    timer_set_oc_value(TIM1, TIM_OC4, 0);  // duty cycle

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

    // Configure SPI1 for slave mode
    spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_256, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

    spi_set_slave_mode(SPI1);  // Set SPI1 to slave mode
    spi_enable_software_slave_management(SPI1);  // Enable software NSS management
    spi_set_nss_low(SPI1);  // Set NSS to low
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


// #define BUFFER_SIZE 16  // Define the number of bytes to transfer
// uint8_t tx_buffer[BUFFER_SIZE];  // Buffer to hold data to send
// uint8_t rx_buffer[BUFFER_SIZE];  // Buffer to hold received data
uint32_t velocity_cmd[4];  // Buffer to hold data to send
uint32_t current_velocity[4];  // Buffer to hold received data

void dma_setup(void) {
    /* Enable DMA1 clock */
    rcc_periph_clock_enable(RCC_DMA1);

    /* Configure DMA channel for SPI1 RX (DMA1 Channel 2) */
    dma_channel_reset(DMA1, DMA_CHANNEL2);
    dma_set_peripheral_address(DMA1, DMA_CHANNEL2, (uint32_t)&SPI1_DR); // SPI data register
    dma_set_memory_address(DMA1, DMA_CHANNEL2, (uint32_t)velocity_cmd); // RX buffer
    dma_set_number_of_data(DMA1, DMA_CHANNEL2, sizeof(velocity_cmd)); // Transfer size
    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL2);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL2);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL2, DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL2, DMA_CCR_MSIZE_8BIT);
    dma_set_priority(DMA1, DMA_CHANNEL2, DMA_CCR_PL_HIGH);
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL2);

    /* Configure DMA channel for SPI1 TX (DMA1 Channel 3) */
    dma_channel_reset(DMA1, DMA_CHANNEL3);
    dma_set_peripheral_address(DMA1, DMA_CHANNEL3, (uint32_t)&SPI1_DR); // SPI data register
    dma_set_memory_address(DMA1, DMA_CHANNEL3, (uint32_t)current_velocity); // TX buffer
    dma_set_number_of_data(DMA1, DMA_CHANNEL3, sizeof(current_velocity)); // Transfer size
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
        dma_set_memory_address(DMA1, DMA_CHANNEL2, (uint32_t)velocity_cmd);
        dma_set_number_of_data(DMA1, DMA_CHANNEL2, sizeof(velocity_cmd));
        dma_enable_channel(DMA1, DMA_CHANNEL2);
    }
}

extern "C" void dma1_channel3_isr(void) {  // TX complete
    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL3, DMA_TCIF)) {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL3, DMA_TCIF);

        /* Transmission is complete, prepare for next transfer */
        // Prepare new data in tx_buffer if needed

        /* Re-enable the DMA channel for the next transfer */
        dma_disable_channel(DMA1, DMA_CHANNEL3);
        dma_set_memory_address(DMA1, DMA_CHANNEL3, (uint32_t)current_velocity);
        dma_set_number_of_data(DMA1, DMA_CHANNEL3, sizeof(current_velocity));
        dma_enable_channel(DMA1, DMA_CHANNEL3);
    }
}


int main(void) {
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
    systick_setup();
    gpio_setup();
    pwm_setup();
    dma_setup();
    spi_slave_setup();
    for (int i = 0; i < 4; i++) {
        velocity_cmd[i] = 0;
        current_velocity[i] = 0;
    }

    // int vals[] = {0, 200, 400, 600};
    while (1) {
        gpio_toggle(GPIOC, GPIO13);
        delay(500);
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

        timer_set_oc_value(TIM1, TIM_OC1, velocity_cmd[0]);
        timer_set_oc_value(TIM1, TIM_OC2, velocity_cmd[1]);
        timer_set_oc_value(TIM1, TIM_OC3, velocity_cmd[2]);
        timer_set_oc_value(TIM1, TIM_OC4, velocity_cmd[3]);
        current_velocity[0] = velocity_cmd[0];
        current_velocity[1] = velocity_cmd[1];
        current_velocity[2] = velocity_cmd[2];
        current_velocity[3] = velocity_cmd[3];
    }
    return 0;
}