#pragma once

extern volatile int  pan_acceleration;
extern volatile int tilt_acceleration;

extern volatile int  pan_speed_goal;
extern volatile int tilt_speed_goal;

extern int target_zoom_speed;

void input_step();
void input_setup();
