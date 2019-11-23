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

#pragma once

#define PAN_STEP_PIN   GPIO8
#define PAN_DIR_PIN    GPIO0
#define PAN_CS_PIN     GPIO1
#define PAN_EN_PIN     GPIO2

#define TILT_STEP_PIN  GPIO0
#define TILT_DIR_PIN   GPIO3
#define TILT_CS_PIN    GPIO4
#define TILT_EN_PIN    GPIO5

#define NATIVE_STEPS_PER_REV 200
#define MY_MULT 1
#define MICROSTEPS (256 * (MY_MULT))
// #define STEPS_PER_REVOLUTION ((NATIVE_STEPS_PER_REV) * (MICROSTEPS))

void motors_setup(void);
void motors_tick(void);

enum motors_axis_t { pan, tilt, axis_total };
struct motors_t {
    volatile int32_t acceleration[axis_total];
    volatile int32_t speed_goal[axis_total];
    volatile int32_t speed_current[axis_total];
};
motors_t* motors_get_data(void);
void motors_set_speed_goals(int32_t pan_speed, int32_t tilt_speed);
void motors_set_accelerations(int32_t pan_acceleration, int32_t tilt_acceleration);
