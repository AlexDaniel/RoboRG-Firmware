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

#include <stdint.h>

/// Angular  pan acceleration to be achieved (steps/s²)
extern volatile int32_t acceleration[2];

/// Angular speed to be achieved (steps/s)
extern volatile int32_t speed_goal[2];

/// Zoom speed to be achieved (camera-specific units, -8 … +8 with LANC)
extern int8_t zoom_speed_goal;

void input_setup();
void input_tick();
void input_protocol_state(char incomingByte);
