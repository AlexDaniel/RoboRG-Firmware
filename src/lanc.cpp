/*
 * Copyright © 2018-2019
 *     Aleks-Daniel Jakimenko-Aleksejev <alex.jakimenko@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#define PIN_LANC_OUT    GPIO2
#define PIN_LANC_IN     GPIO3
#define PIN_LANC_DETECT GPIO4

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>

#include "cdcacm.hpp"
#include "lanc.hpp"
#include "timing.hpp"

volatile uint8_t datagram_byte_counter = 0;
uint32_t datagram_last_byte_ms = 0;

// Define this to use hardware UART for TX (not recommended)
//#define LANC_OUT_UART

// Define this when the output is inverted (just one NPN transistor)
#define LANC_OUT_INVERTED

// First two bytes are output, the rest is input
volatile uint8_t data[8] = {};

// To make sure we don't overwrite the command during transmission
volatile uint8_t output_data_staging[2] = {};
// To make sure we don't overwrite the command during transmission
volatile uint8_t input_data_staging[8] = {};

void lanc_setup(void) {
    // LANC-detect
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_PULL_UPDOWN, PIN_LANC_DETECT);
    gpio_set(GPIOA, PIN_LANC_DETECT); // pull-up


    // start bit interrupt
    nvic_enable_irq(NVIC_EXTI3_IRQ);
    exti_select_source(EXTI3, GPIOA);
    exti_set_trigger(EXTI3, EXTI_TRIGGER_FALLING);

    // uart
    rcc_periph_clock_enable(RCC_USART2);
    nvic_enable_irq(NVIC_USART2_IRQ);

    gpio_set_mode(GPIO_BANK_USART2_RX, GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RX);


#ifdef LANC_OUT_UART

#ifdef LANC_OUT_INVERTED
#error Cannot invert the output of hardware UART (please undefine LANC_OUT_INVERTED)
#endif

    gpio_set(GPIOA, PIN_LANC_OUT);
    gpio_set_mode(GPIO_BANK_USART2_TX, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
#else
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, PIN_LANC_OUT);
#ifdef LANC_OUT_INVERTED
    gpio_clear(GPIOA, PIN_LANC_OUT);
#else
    gpio_set(GPIOA, PIN_LANC_OUT);
#endif


    nvic_enable_irq(NVIC_TIM4_IRQ);
    nvic_set_priority(NVIC_TIM4_IRQ, 1);

    rcc_periph_clock_enable(RCC_TIM4);

    TIM_PSC(TIM4) = 72000000 / 1000000; // 1µs per tick
    TIM_ARR(TIM4) = 104; // 104µs per bit
    TIM_DIER(TIM4) |= TIM_DIER_UIE; // enable interrupt
#endif


    // UART parameters
    usart_set_baudrate(USART2, 9600);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART2, USART_MODE_TX_RX);

    // receive interrupt
    USART_CR1(USART2) |= USART_CR1_RXNEIE;

    usart_enable(USART2);
}

/// This is called after every datagram.
static void lanc_datagram_tick(void) {
    // TODO memcpy?
    data[0] = output_data_staging[0];
    data[1] = output_data_staging[1];
    for (int i = 0; i < 8; i++)
        input_data_staging[i] = data[i];
}

/// This is called after every byte.
static void process_lanc_in(uint8_t byte) {
    if (milliseconds - datagram_last_byte_ms > 5) {
        datagram_byte_counter = 0;
    } else {
        datagram_byte_counter++;
    }
    if (datagram_byte_counter == 7 ||
        datagram_byte_counter == 0) { // for output bytes only (0th and 1st)
        // enable start bit trigger for the next byte
        exti_enable_request(EXTI3);
    }
    datagram_last_byte_ms = milliseconds;
    if (datagram_byte_counter >= 2 and datagram_byte_counter <= 7)
        data[datagram_byte_counter] = byte;
    if (datagram_byte_counter == 7)
        lanc_datagram_tick();
}

#ifndef LANC_OUT_UART
volatile uint8_t current_byte = 0;
uint8_t bit_counter = 0;

void tim4_isr(void) {
    if (bit_counter <= 7) {
        TIM_CNT(TIM4) = 1;
        if (current_byte & (1 << bit_counter)) {
#ifdef LANC_OUT_INVERTED
            gpio_set(GPIOA, PIN_LANC_OUT);
#else
            gpio_clear(GPIOA, PIN_LANC_OUT);
#endif
        } else {
#ifdef LANC_OUT_INVERTED
            gpio_clear(GPIOA, PIN_LANC_OUT);
#else
            gpio_set(GPIOA, PIN_LANC_OUT);
#endif
        }
        TIM_CNT(TIM4) = 1;
        bit_counter++;
    } else {
        bit_counter = 0;
#ifdef LANC_OUT_INVERTED
        gpio_clear(GPIOA, PIN_LANC_OUT);
#else
        gpio_set(GPIOA, PIN_LANC_OUT);
#endif
        TIM_CR1(TIM4) &= ~TIM_CR1_CEN; // stop timer
    }
    TIM_SR(TIM4) &= ~TIM_SR_UIF; // clear interrupt flag
}
#endif

void exti3_isr(void) {
    if (data[0] != 0) {
        uint8_t byte_to_send;
        bool skip = false;
        switch (datagram_byte_counter) {
        case 7: // ahead of time by a byte tick
            byte_to_send = data[0];
            break;
        case 0:
            byte_to_send = data[1];
            break;
        default:
            skip = true;
            break;
        }
        if (!skip) {
#ifdef LANC_OUT_UART
            usart_send(USART2, ~byte_to_send);
            USART_CR1(USART2) |= USART_CR1_TXEIE;
#else
            current_byte = byte_to_send;
            TIM_CNT(TIM4) = 1;
            TIM_CR1(TIM4) |= TIM_CR1_CEN; // start timer
#endif
        }
    }

    // disable the interrupt for now
    exti_disable_request(EXTI3);
    exti_reset_request(EXTI3);
}

void usart2_isr(void) {
    // Check if we were called because of RXNE
    if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
        ((USART_SR(USART2)  & USART_SR_RXNE)    != 0)) {

        uint8_t data = usart_recv(USART2);
        // invert so that the data corresponds to protocol description
        process_lanc_in(~data);
    }

    // Check if we were called because of TXE
    if (((USART_CR1(USART2) & USART_CR1_TXEIE) != 0) &&
        ((USART_SR(USART2)  & USART_SR_TXE)    != 0)) {

        /* Disable the TXE interrupt as we don't need it anymore. */
        USART_CR1(USART2) &= ~USART_CR1_TXEIE;
    }
}

/// Stage the command to the device.
void lanc_write(uint8_t byte0, uint8_t byte1) {
    output_data_staging[0] = byte0;
    output_data_staging[1] = byte1;
}

/// Get the last lanc datagram.
volatile uint8_t* lanc_read(void) {
    return input_data_staging;
}

void lanc_tick(void) {
}
