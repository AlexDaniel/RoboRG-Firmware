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

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/scb.h>

#include "cdcacm.hpp"
#include "input.hpp"
#include "lanc.hpp"
#include "motors.hpp"
#include "timing.hpp"

static void clock_setup(void) {
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_AFIO);

    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
    systick_set_reload(72000000/8/1000 - 1);
    systick_interrupt_enable();
    systick_counter_enable();
}

static void gpio_setup(void) {
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                 GPIO_CNF_OUTPUT_PUSHPULL, GPIO6);

    AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON;
}

void sys_tick_handler(void) {
    timing_tick();
}

int main(void) {
    clock_setup();
    cdcacm_setup();
    gpio_setup();
    lanc_setup();
    motors_setup();
    input_setup();

    while (1) {
        motors_tick();
        lanc_tick();
        cdcacm_tick();
        input_tick();
    }

    return 0;
}
