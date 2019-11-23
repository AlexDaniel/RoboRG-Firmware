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
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/timer.h>
#include <stdlib.h>

#include "input.hpp"
#include "motors.hpp"
#include "timing.hpp"

#define ACCELERATION_FREQUENCY 50
uint32_t last_update = 0;

// TODO use a dynamic prescaler?
#define BASE_FREQUENCY 72000000
#define PRESCALER (BASE_FREQUENCY / 1000000) // 1µs per tick

class Driver { // TODO refactor

public:
    uint32_t pin_enable;
    uint32_t pin_direction;
    uint32_t pin_step;
    uint32_t pin_chipselect;

    Driver (
            uint32_t _pin_enable,
            uint32_t _pin_direction,
            uint32_t _pin_step,
            uint32_t _pin_chipselect
            ) :
        pin_enable(_pin_enable),
        pin_direction(_pin_direction),
        pin_step(_pin_step),
        pin_chipselect(_pin_chipselect)
    { }
};

Driver axis_drivers[axis_total] =
    {
     {  PAN_EN_PIN,  PAN_DIR_PIN,  PAN_STEP_PIN,  PAN_CS_PIN },
     { TILT_EN_PIN, TILT_DIR_PIN, TILT_STEP_PIN, TILT_CS_PIN },
    };

motors_t motors = {};

int32_t step_counter[axis_total] = {};


static void setup_timers() {
    nvic_enable_irq(NVIC_TIM2_IRQ);
    nvic_set_priority(NVIC_TIM2_IRQ, 1);
    rcc_periph_clock_enable(RCC_TIM2);
    TIM_PSC(TIM2) = PRESCALER;
    TIM_ARR(TIM2) = 10000; // some value
    TIM_DIER(TIM2) |= TIM_DIER_UIE; // enable interrupt
    TIM_CR1(TIM2) |= TIM_CR1_CEN; // start timer

    nvic_enable_irq(NVIC_TIM3_IRQ);
    nvic_set_priority(NVIC_TIM3_IRQ, 1);
    rcc_periph_clock_enable(RCC_TIM3);
    TIM_PSC(TIM3) = PRESCALER;
    TIM_ARR(TIM3) = 10000; // some value
    TIM_DIER(TIM3) |= TIM_DIER_UIE; // enable interrupt
    TIM_CR1(TIM3) |= TIM_CR1_CEN; // start timer
}

static void update_timer(motors_axis_t axis) {
    int32_t speed = motors.speed_current[axis];
    int32_t absolute = speed < 0 ? -speed : speed;

    uint32_t compare_match = 255;
    if (speed != 0)
        compare_match = BASE_FREQUENCY / PRESCALER / absolute;
    if (compare_match > 0xFFFF) // oversaturated 16-bit timer
        compare_match = 0xFFFF;

    switch (axis) {
    case pan:
        TIM_CNT(TIM2) = 0;
        TIM_ARR(TIM2) = compare_match;
        break;
    case tilt:
        TIM_CNT(TIM3) = 0;;
        TIM_ARR(TIM3) = compare_match;
        break;
    default:
        return;
    }
}


// TODO refactor and check asm
static void do_step_pan(void) {
    int32_t speed = motors.speed_current[pan];
    if        (speed > 0) {
        gpio_clear(GPIOB, PAN_DIR_PIN);
        gpio_set(  GPIOA, PAN_STEP_PIN);
        step_counter[pan]++;
        gpio_clear(GPIOA, PAN_STEP_PIN);
    } else if (speed < 0) {
        gpio_set(  GPIOB, PAN_DIR_PIN);
        gpio_set(  GPIOA, PAN_STEP_PIN);
        step_counter[pan]--;
        gpio_clear(GPIOA, PAN_STEP_PIN);
    } else { } // zero speed, no stepping
}

static void do_step_tilt(void) {
    int32_t speed = motors.speed_current[tilt];
    if        (speed > 0) {
        gpio_clear(GPIOB, TILT_DIR_PIN);
        gpio_set(  GPIOA, TILT_STEP_PIN);
        step_counter[tilt]++;
        gpio_clear(GPIOA, TILT_STEP_PIN);
    } else if (speed < 0) {
        gpio_set(  GPIOB, TILT_DIR_PIN);
        gpio_set(  GPIOA, TILT_STEP_PIN);
        step_counter[tilt]++;
        gpio_clear(GPIOA, TILT_STEP_PIN);
    } else { } // zero speed, no stepping
}

void tim2_isr(void) {
    do_step_pan();
    TIM_SR(TIM2) &= ~TIM_SR_UIF; // clear interrupt flag
}

void tim3_isr(void) {
    do_step_tilt();
    TIM_SR(TIM3) &= ~TIM_SR_UIF; // clear interrupt flag
}

static void update_speed(motors_axis_t axis) {
    int32_t acc  = motors.acceleration[axis];
    int32_t diff = motors.speed_goal[axis] - motors.speed_current[axis];

    if (-acc < diff && diff < +acc) {
        // already or almost there
        motors.speed_current[axis] = motors.speed_goal[axis];
    } else {
        if (diff > 0) {
            motors.speed_current[axis] += acc;
        } else {
            motors.speed_current[axis] -= acc;
        }
    }
    update_timer(axis);
}

static void tick_speeds(void) {
    for (size_t i = 0; i < axis_total; i++)
        update_speed(static_cast<motors_axis_t>(i));
}

static void driver_setup(Driver driver) {
    gpio_set(GPIOB, driver.pin_enable);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, driver.pin_enable);
    gpio_set(GPIOB, driver.pin_chipselect);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, driver.pin_chipselect);
    gpio_set(GPIOB, driver.pin_direction);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, driver.pin_direction);

    gpio_clear(GPIOA, driver.pin_step);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, driver.pin_step);
}

static void motors_enable(void) {
    for (int i = 0; i < axis_total; i++)
        gpio_clear(GPIOB, axis_drivers[i].pin_enable);
}

///* currently unused
static void motors_disable(void) {
    for (int i = 0; i < axis_total; i++)
        gpio_set(GPIOB, axis_drivers[i].pin_enable);
}
//*/

static void driver_spi_setup(uint32_t cs_pin) {
    // It achieves something like this:
    /*
      driver.begin();
      driver.rms_current(800);
      driver.stealthChop(1); // enable, otherwise noisy
      driver.stealth_autoscale(1); // enable, otherwise works horribly
      driver.microsteps(MICROSTEPS);
      driver.intpol(1); // if MICROSTEPS ≠ 256 it makes the motion smoother
      driver.hold_current(10);
      driver.hold_delay(2);
      driver.power_down_delay(32); // relatively high to prevent vibration when switching to hold current
    */

    // TODO I mean… it works, but it is horrible.
    // GSTAT
    gpio_clear(GPIOB, cs_pin);
    spi_xfer(SPI2, 0x01);
    spi_xfer(SPI2, 0x00);
    spi_xfer(SPI2, 0x00);
    spi_xfer(SPI2, 0x00);
    spi_xfer(SPI2, 0x00);
    gpio_set(GPIOB, cs_pin);
    for (int i = 0; i < 0x400; i++)
        __asm__("nop");

    // PWMCONF
    gpio_clear(GPIOB, cs_pin);
    spi_xfer(SPI2, 0xF0);
    spi_xfer(SPI2, 0x00);
    spi_xfer(SPI2, 0x05);
    spi_xfer(SPI2, 0x04);
    spi_xfer(SPI2, 0x80);
    gpio_set(GPIOB, cs_pin);
    for (int i = 0; i < 0x400; i++)
        __asm__("nop");

    // CHOPCONF
    gpio_clear(GPIOB, cs_pin);
    spi_xfer(SPI2, 0xEC);
    spi_xfer(SPI2, 0x10);
    spi_xfer(SPI2, 0x02);
    spi_xfer(SPI2, 0x80);
    spi_xfer(SPI2, 0x08);
    gpio_set(GPIOB, cs_pin);
    for (int i = 0; i < 0x400; i++)
        __asm__("nop");


    // GCONF
    gpio_clear(GPIOB, cs_pin);
    spi_xfer(SPI2, 0x80);
    spi_xfer(SPI2, 0x00);
    spi_xfer(SPI2, 0x00);
    spi_xfer(SPI2, 0x00);
    spi_xfer(SPI2, 0x04);
    gpio_set(GPIOB, cs_pin);
    for (int i = 0; i < 0x400; i++)
        __asm__("nop");

    // IHOLD_IRUN
    gpio_clear(GPIOB, cs_pin);
    spi_xfer(SPI2, 0x90);
    spi_xfer(SPI2, 0x00);
    spi_xfer(SPI2, 0x02); // IHOLDDELAY
    spi_xfer(SPI2, 0x19); // IRUN
    spi_xfer(SPI2, 0x08); // IHOLD
    gpio_set(GPIOB, cs_pin);
    for (int i = 0; i < 0x400; i++)
        __asm__("nop");


    // TPOWERDOWN=10: Delay before power down in stand still
    gpio_clear(GPIOB, cs_pin);
    spi_xfer(SPI2, 0x91);
    spi_xfer(SPI2, 0x00);
    spi_xfer(SPI2, 0x00);
    spi_xfer(SPI2, 0x00);
    spi_xfer(SPI2, 0x20);
    gpio_set(GPIOB, cs_pin);
    for (int i = 0; i < 0x400; i++)
        __asm__("nop");
}

static void spi_setup(void) {
    rcc_periph_clock_enable(RCC_SPI2);

    // SCK and MOSI
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO13);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO15);

    // MISO
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO14);
    gpio_set(GPIOB, GPIO14);


    spi_reset(SPI2);
    spi_init_master(SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_256,
                    0, 0, // set later in spi_set_standard_mode
                    SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
    spi_set_standard_mode(SPI2, 3);

    spi_enable_software_slave_management(SPI2);
    spi_set_nss_high(SPI2);

    spi_enable(SPI2);

    driver_spi_setup(GPIO1);
    driver_spi_setup(GPIO4);
}

void motors_setup(void) {
    for (int i = 0; i < axis_total; i++)
        driver_setup(axis_drivers[i]);

    spi_setup();
    motors_enable();
    //motors_disable(); // TODO
    setup_timers();

    // init speeds
    for (size_t i = 0; i < axis_total; i++)
        update_speed(static_cast<motors_axis_t>(i));
}

void motors_tick(void) {
    uint32_t once_per = 1000 / ACCELERATION_FREQUENCY;
    if (milliseconds - last_update > once_per) {
        last_update += once_per;
        tick_speeds();
    }
    /*
    motors.acceleration[pan] = 100;
    motors.acceleration[tilt] = 100;
    if (milliseconds % 2000 < 1000) {
        motors.speed_goal[pan] = +500;
        motors.speed_goal[tilt] = +500;
    } else {
        motors.speed_goal[pan] = -500;
        motors.speed_goal[tilt] = -500;
    }
    */
}


motors_t* motors_get_data(void) {
    return &motors;
}

void motors_set_speed_goals(int32_t pan_speed, int32_t tilt_speed) {
    motors.speed_goal[pan]  =  pan_speed;
    motors.speed_goal[tilt] = tilt_speed;
}

void motors_set_accelerations(int32_t pan_acceleration, int32_t tilt_acceleration) {
    motors.acceleration[pan]  =  pan_acceleration;
    motors.acceleration[tilt] = tilt_acceleration;
}
