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

// TODO speeds and acceleration are awkwardly in *steps*, they should
// be in degrees

// TODO Temporary protocol (will be removed after USB is fully embraced)
// For both input and output:
// (4 bytes) 0xAA × 4
// (4 bytes) Pan  speed
// (4 bytes) Tilt speed
// (4 bytes) Pan  acceleration
// (4 bytes) Tilt acceleration
// (8 bytes) LANC pass-thru (for input only 2 bytes are used, rest is ignored)
struct packet {
    int32_t start_sequence; /// 0xAB 0xAB 0xAB 0xAB
    int32_t  pan_speed_current; /// Current angular speed (steps/s)
    int32_t tilt_speed_current; /// Current angular speed (steps/s)
    int32_t  pan_speed_goal; /// Angular speed to be achieved (steps/s)
    int32_t tilt_speed_goal; /// Angular speed to be achieved (steps/s)
    int32_t  pan_acceleration; /// Angular acceleration to be achieved (steps/s²)
    int32_t tilt_acceleration; /// Angular acceleration to be achieved (steps/s²)
    uint8_t lanc_data[8]; /// LANC pass-thru
} __attribute__((packed));

void input_setup(void);
void input_tick(void);
void input_protocol_state(char incomingByte);
packet* input_make_output_packet(void);
