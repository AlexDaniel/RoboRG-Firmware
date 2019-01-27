#include "input.hpp"

#include <Arduino.h>


volatile int16_t acceleration[2];
volatile int16_t speed_goal[2];

int8_t zoom_speed_goal = 0;

uint8_t state = 0;
uint8_t axis = 0;
String argument = "";

int type = 0; // 0 – speed, 1 – acceleration

void input_setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Start...");
}

/// Processing of speed commands.
void do_speed() {
  int16_t value = argument.toInt();

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
  int16_t value = argument.toInt();
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
void gcode_state(int incomingByte) {
  switch (state) {
  case 0:
    if        (incomingByte == 'G')
      state++;
    else if   (incomingByte == 'M')
      state = 10;
    else
      state = 0;
    break;
  case 1:
    if        (incomingByte == '0') {
      type = 0;
      state++;
    } else
      state = 0;
    break;
  case 2:
    if        (incomingByte == ' ')
      state++;
    else
      state = 0;
    break;
  case 3:
    if        (incomingByte == 'X') {
      axis = 0;
      state++;
    } else if (incomingByte == 'Y') {
      axis = 1;
      state++;
    } else if (incomingByte == 'Z') {
      axis = 2;
      state++;
    } else
      state = 0;
    break;
  case 4:
    if        (incomingByte == '\n') {
      state = 0;
      do_line();
      argument = "";
    } else if (('0' <= incomingByte && incomingByte <= '9') || incomingByte == '-') {
      argument += (char) incomingByte;
    } else {
      argument = "";
      state = 0;
    }
    break;
  case 10:
    if        (incomingByte == '2')
      state++;
    else
      state = 0;
    break;
  case 11:
    if        (incomingByte == '0')
      state++;
    else
      state = 0;
    break;
  case 12:
    if        (incomingByte == '1') {
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
  if (Serial.available() > 0)
    gcode_state(Serial.read());
}
