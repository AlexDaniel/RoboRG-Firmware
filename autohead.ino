#include "input.hpp"
#include "motors.hpp"
#include "lanc.hpp"

void setup() {
  input_setup();
  motors_setup();
  lanc_setup();

  sei(); // allow interrupts
}

void loop() {
  motors_step();
  input_step();
  lanc_step();
}
