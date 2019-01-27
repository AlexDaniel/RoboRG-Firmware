#pragma once

#define PAN_EN_PIN      8
#define PAN_DIR_PIN    14
#define PAN_STEP_PIN   15
#define PAN_CS_PIN     16

#define TILT_EN_PIN     7
#define TILT_DIR_PIN   17
#define TILT_STEP_PIN  18
#define TILT_CS_PIN    19

#define NATIVE_STEPS_PER_REV 200
#define MY_MULT 1
#define MICROSTEPS (256 * (MY_MULT))
// #define STEPS_PER_REVOLUTION ((NATIVE_STEPS_PER_REV) * (MICROSTEPS))

void motors_setup();
void motors_tick();
