#include "input.hpp"

#include <Arduino.h>


volatile int  pan_acceleration = 1;
volatile int tilt_acceleration = 1;

volatile int  pan_speed_goal = 0;
volatile int tilt_speed_goal = 0;

int target_zoom_speed = 0;

int dir = 1;

int state = 0;
int axis = 0;
String inString = "";

int type = 0; // 0 – speed, 1 – acceleration

void input_setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Start...");
}

void doSpeed() {
  int value = inString.toInt();

  if        (axis == 0) { // X = tilt
    tilt_speed_goal   = value;
  } else if (axis == 2) { // Z = pan
    pan_speed_goal    = value;
  } else if (axis == 1) { // Y = zoom
    target_zoom_speed = value;
  }
}

void doAcceleration() {
  int value = inString.toInt();
  if        (axis == 0) { // X = tilt
    tilt_acceleration = value;
  } else if (axis == 2) { // Z = pan
    pan_acceleration = value;
  }
}

void doLine() {
  //Serial.print("I received: ");
  //Serial.println(value, DEC);
  //Serial.print("Axis: ");
  //Serial.println(axis, DEC);

  if (type == 0)
    doSpeed();
  else
    doAcceleration();
}


void gcodeState(int incomingByte) {
  switch (state) {
  case 0:
    if (incomingByte == 'G')
      state++;
    else if (incomingByte == 'M')
      state = 10;
    else
      state = 0;
    break;
  case 1:
    if (incomingByte == '0') {
      type = 0;
      state++;
    } else
      state = 0;
    break;
  case 2:
    if (incomingByte == ' ')
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
    if (incomingByte == '\n') {
      state = 0;
      doLine();
      inString = "";
    } else if (('0' <= incomingByte && incomingByte <= '9') || incomingByte == '-') {
      inString += (char) incomingByte;
    } else {
      inString = "";
      state = 0;
    }
    break;
  case 10:
    if (incomingByte == '2')
      state++;
    else
      state = 0;
    break;
  case 11:
    if (incomingByte == '0')
      state++;
    else
      state = 0;
    break;
  case 12:
    if (incomingByte == '1') {
      type = 1;
      state = 2;
    } else
      state = 0;
    break;
  default:
    state = 0;
  }
}

void input_step() {
  if (Serial.available() > 0)
    gcodeState(Serial.read());
}
