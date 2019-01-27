#pragma once

#include <stdint.h>

/// Angular  pan acceleration to be achieved (steps/s²)
extern volatile int16_t acceleration[2];

/// Angular speed to be achieved (steps/s)
extern volatile int16_t speed_goal[2];

/// Zoom speed to be achieved (camera-specific units, -8 … +8 with LANC)
extern int8_t zoom_speed_goal;

void input_setup();
void input_tick();
