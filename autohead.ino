#include "input.hpp"
#include "lanc.hpp"
#include "motors.hpp"

void setup() {
  input_setup();
  motors_setup();
  lanc_setup();

  sei(); // allow interrupts
}

void loop() {
  motors_tick();
  input_tick();
  lanc_tick();
}
