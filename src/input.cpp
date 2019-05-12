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

#include "input.hpp"

volatile int32_t acceleration[2] = {1000, 1000};
volatile int32_t speed_goal[2] = {0, 0};

int8_t zoom_speed_goal = 0;

uint8_t state = 0;
uint8_t axis = 0;

uint8_t argument_length = 0;
char argument[64] = {0};

int type = 0; // 0 – speed, 1 – acceleration

void input_setup() {
    // TODO: print "Start..."
}

/// Processing of speed commands.
void do_speed() {
    argument[argument_length++] = 0;
    int32_t value = atoi(argument);

    if        (axis == 0) { // X = tilt
        speed_goal[1]   = value;
    } else if (axis == 1) { // Y = zoom
        zoom_speed_goal = value;
    } else if (axis == 2) { // Z = pan
        speed_goal[0]   = value;
    }
}

/// Processing of acceleration commands.
void do_acceleration() {
    argument[argument_length++] = 0;
    int32_t value = atoi(argument);
    if        (axis == 0) { // X = tilt
        acceleration[1] = value;
    } else if (axis == 2) { // Z = pan
        acceleration[0] = value;
    }
}

/// Processing of one line of input.
void do_line() {
    if (type == 0)
        do_speed();
    else
        do_acceleration();
}


/// State machine for gcode-like input processing.
void input_protocol_state(char incoming_byte) {
    switch (state) {
    case 0:
        if        (incoming_byte == 'G')
            state++;
        else if   (incoming_byte == 'M')
            state = 10;
        else
            state = 0;
        break;
    case 1:
        if        (incoming_byte == '0') {
            type = 0;
            state++;
        } else
            state = 0;
        break;
    case 2:
        if        (incoming_byte == ' ')
            state++;
        else
            state = 0;
        break;
    case 3:
        if        (incoming_byte == 'X') {
            axis = 0;
            state++;
        } else if (incoming_byte == 'Y') {
            axis = 1;
            state++;
        } else if (incoming_byte == 'Z') {
            axis = 2;
            state++;
        } else
            state = 0;
        break;
    case 4:
        if        (incoming_byte == '\n') {
            state = 0;
            do_line();
            argument_length = 0;
            argument[0] = 0;
        } else if (('0' <= incoming_byte && incoming_byte <= '9') || incoming_byte == '-') {
            argument[argument_length++] = incoming_byte;
        } else {
            argument_length = 0;
            argument[0] = 0;
            state = 0;
        }
        break;
    case 10:
        if        (incoming_byte == '2')
            state++;
        else
            state = 0;
        break;
    case 11:
        if        (incoming_byte == '0')
            state++;
        else
            state = 0;
        break;
    case 12:
        if        (incoming_byte == '1') {
            type = 1;
            state = 2;
        } else
            state = 0;
        break;
    default:
        state = 0;
    }
}


void input_tick() {
}
