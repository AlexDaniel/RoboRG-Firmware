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

#include <stdlib.h>
#include <string.h>

#include "input.hpp"

#include "cdcacm.hpp"
#include "lanc.hpp"
#include "motors.hpp"
#include "timing.hpp"

uint8_t state = 0;
uint8_t byte_count = 0;

packet  input_data = {};
packet output_data = {};

/// Processing of one line of input.
static void process_input_packet(void) {
    // TODO temporary solution
    motors_set_speed_goals(input_data.pan_speed_goal,
                           input_data.tilt_speed_goal);
    motors_set_accelerations(input_data.pan_acceleration,
                             input_data.tilt_acceleration);
    lanc_write(input_data.lanc_data[0], input_data.lanc_data[1]);
}

packet* input_make_output_packet(void) {
    // TODO refactor once the protocol is changed
    motors_t* motors = motors_get_data();
    output_data.pan_speed_current  = motors->speed_current[0];
    output_data.tilt_speed_current = motors->speed_current[1];
    output_data.pan_speed_goal  = motors->speed_goal[0];
    output_data.tilt_speed_goal = motors->speed_goal[1];
    output_data.pan_acceleration  = motors->acceleration[0];
    output_data.tilt_acceleration = motors->acceleration[1];

    volatile uint8_t* lanc = lanc_read();
    for (int i = 0; i < 8; i++) // TODO memcpy
        output_data.lanc_data[i] = lanc[i];

    return &output_data;
}

/// State machine for input processing.
void input_protocol_state(char incoming_byte) {
    memset(((unsigned char*) &input_data) + byte_count++, incoming_byte, 1);
    if (byte_count <= 4 && incoming_byte != 0xAA) {
        byte_count = 0;
    } else if (byte_count >= sizeof(packet)) {
        byte_count = 0;
        process_input_packet();
    }
}

void input_setup(void) {
    output_data.start_sequence = 0xAAAAAAAA;
}

uint32_t last_ms = 0;
void input_tick() {
    if (milliseconds < 5000) // TODO this delay shouldn't exist
        return;
    // TODO this should be synced with lanc
    if (milliseconds % 20 == 0 && last_ms != milliseconds) {
        packet* data = input_make_output_packet();
        cdcacm_write(data, sizeof(packet));
        last_ms = milliseconds;
    }
}
