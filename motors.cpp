#include "motors.hpp"

#include "config.hpp"
#include "input.hpp"

#include <TMC2130Stepper.h>

#define ACCELERATION_FREQUENCY 50

class Driver {
  friend class TMC2130Stepper;

public:
  uint16_t pin_enable;
  uint16_t pin_direction;
  uint16_t pin_step;
  uint16_t pin_chipselect;
  TMC2130Stepper stepper;

  Driver (
          uint16_t _pin_enable,
          uint16_t _pin_direction,
          uint16_t _pin_step,
          uint16_t _pin_chipselect
          ) :
    pin_enable(_pin_enable),
    pin_direction(_pin_direction),
    pin_step(_pin_step),
    pin_chipselect(_pin_chipselect),
    stepper(_pin_enable, _pin_direction, _pin_step, _pin_chipselect)
  { }
};


enum motors_axis_t { pan, tilt, axis_total };

Driver axis_drivers[axis_total] = {
  {  PAN_EN_PIN,  PAN_DIR_PIN,  PAN_STEP_PIN,  PAN_CS_PIN },
  { TILT_EN_PIN, TILT_DIR_PIN, TILT_STEP_PIN, TILT_CS_PIN },
};


/// Lack of timers forces us to do timekeeping even though the
/// prescaler is changing.
uint32_t time = 0;

volatile int16_t speed_current[axis_total];
uint16_t prescaler_current[axis_total];


void motors_setup_timer() {
  // using Timer0 and Timer2 (8-bit timers)

  TCCR0A = 0; // Counter Control Register A
  TCCR2A = 0; // Counter Control Register A

  TCCR0B = 0; // Counter Control Register B
  TCCR2B = 0; // Counter Control Register B

  TCCR0A |= 1 << WGM01;  // WGMx1 – Waveform Generation CTC mode
  TCCR2A |= 1 << WGM21;  // WGMx1 – Waveform Generation CTC mode

  TIMSK0 |= 1 << OCIE0A; // Timer Mask
  TIMSK2 |= 1 << OCIE2A; // Timer Mask

  OCR0A = 255; // Compare Match
  OCR2A = 255; // Compare Match

  TCNT0 = 0; // Counter Value
  TCNT2 = 0; // Counter Value

  // any prescaler, just trying to trigger the ISR
  TCCR0B |= 1 << CS00;
  TCCR2B |= 1 << CS20;
}

//  TCNT0 = 0;
//  TCNT2 = 0;
//  TCCR2B |= 1 << CS22;
//  TCCR2B |= 1 << CS21;

uint16_t calculate_prescaler(int16_t speed) {
  // Useful prescalers that are available on both timers:
  // clk /64 /256 /1024

  if        (speed >= BASE_FREQUENCY / 256 /  64) {
    return   64;
  } else if (speed >= BASE_FREQUENCY / 256 / 256) {
    return  256;
  } else { // slowest range
    return 1024;
  }
}

void axis_ISR(motors_axis_t axis) {
  int16_t speed = speed_current[axis];
  int16_t absolute = speed < 0 ? -speed : speed;

  uint16_t prescaler = calculate_prescaler(absolute);
  uint16_t compare_match = 255;
  if (speed != 0)
    compare_match = BASE_FREQUENCY / prescaler / absolute;
  if (compare_match > 255)
    compare_match = 255;

  uint8_t dir_pin;
  uint8_t step_pin;
  switch (axis) {
  case pan:
    TCCR0B = 0;
    TCNT0 = 0;
    dir_pin  = PAN_DIR_PIN;
    step_pin = PAN_STEP_PIN;
    OCR0A = compare_match;
    switch (prescaler) {
    case   64:
      TCCR0B |= (1 << CS01) | (1 << CS00);
      break;
    case  256:
      TCCR0B |= 1 << CS02;
      break;
    case 1024:
      TCCR0B |= (1 << CS02) | (1 << CS00);
      break;
    }
    break;
  case tilt:
    TCCR2B = 0;
    TCNT2 = 0;
    dir_pin  = TILT_DIR_PIN;
    step_pin = TILT_STEP_PIN;
    OCR2A = compare_match;
    switch (prescaler) {
    case   64:
      TCCR2B |= 1 << CS22;
      break;
    case  256:
      TCCR2B |= (1 << CS22) | (1 << CS21);
      break;
    case 1024:
      TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
      break;
    }
    break;
  default:
    return;
  }
  prescaler_current[axis] = prescaler;

  if        (speed > 0) {
    digitalWrite(dir_pin,   LOW);
    digitalWrite(step_pin, HIGH);
  } else if (speed < 0) {
    digitalWrite(dir_pin,  HIGH);
    digitalWrite(step_pin, HIGH);
  } else { } // zero speed, no stepping

  digitalWrite(step_pin, LOW);
}

void timekeeping() {
  time += OCR2A * prescaler_current[tilt];
}

ISR(TIMER0_COMPA_vect) {
  axis_ISR(pan);
}
ISR(TIMER2_COMPA_vect) {
  timekeeping();
  axis_ISR(tilt);
}

void update_speed(motors_axis_t axis) {
  int16_t acc  = acceleration[axis];
  int16_t diff = speed_goal[axis] - speed_current[axis];

  if (-acc < diff && diff < +acc) {
    // already or almost there
    speed_current[axis] = speed_goal[axis];
    return;
  }

  if (diff > 0) {
    speed_current[axis] += acc;
  } else {
    speed_current[axis] -= acc;
  }
}

void tick_speeds() {
  uint32_t ticks_required = BASE_FREQUENCY / ACCELERATION_FREQUENCY;
  if (time < ticks_required)
    return;
  time -= ticks_required;

  for (size_t i = 0; i < axis_total; i++)
    update_speed(static_cast<motors_axis_t>(i));
}

void motors_driver_setup(TMC2130Stepper &driver) {
  driver.begin();
  driver.rms_current(800);
  driver.stealthChop(1); // enable, otherwise noisy
  driver.stealth_autoscale(1); // enable, otherwise works horribly
  driver.microsteps(MICROSTEPS);
  driver.intpol(1); // if MICROSTEPS ≠ 256 it makes the motion smoother
  driver.hold_current(10);
  driver.hold_delay(2);
  driver.power_down_delay(32); // relatively high to prevent vibration when switching to hold current
}

void motors_enable() {
  digitalWrite( PAN_EN_PIN, LOW);
  digitalWrite(TILT_EN_PIN, LOW);
}

void motors_disable() {
  digitalWrite( PAN_EN_PIN, HIGH);
  digitalWrite(TILT_EN_PIN, HIGH);
}

void motors_setup() {
  for (int i = 0; i < axis_total; i++)
    motors_driver_setup(axis_drivers[i].stepper);

  motors_enable();
  motors_setup_timer();
}

void motors_tick() {
  tick_speeds();
}
