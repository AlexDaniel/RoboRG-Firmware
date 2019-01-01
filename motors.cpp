#include "motors.hpp"

#include "input.hpp"

#include <TMC2130Stepper.h>


// min steps/s =   clock    ÷ prescaler ÷ max value
//             = 16_000_000 ÷    1024   ÷   256
//             ≈ 61




TMC2130Stepper  pan_driver = TMC2130Stepper( PAN_EN_PIN,  PAN_DIR_PIN,  PAN_STEP_PIN,  PAN_CS_PIN);
TMC2130Stepper tilt_driver = TMC2130Stepper(TILT_EN_PIN, TILT_DIR_PIN, TILT_STEP_PIN, TILT_CS_PIN);

volatile int pan_speed = 0;
volatile int tilt_speed = 0;

// Using Timer0 and Timer2 (8-bit timers)
void motors_setup_timer() {
  TCCR0A = 0; // Counter Control Register A
  TCCR0B = 0; // Counter Control Register B
  TCNT0 = 0; // Counter Value

  TCCR0A |= (1 << WGM01);  // WGMx1 – Waveform Generation CTC mode
  TIMSK0 |= (1 << OCIE0A); // Timer Mask


  TCCR2A = 0; // Counter Control Register A
  TCCR2B = 0; // Counter Control Register B
  TCNT2 = 0; // Counter Value

  TCCR2A |= (1 << WGM21);  // WGMx1 – Waveform Generation CTC mode
  TIMSK2 |= (1 << OCIE2A); // Timer Mask
}

void motors_kickstart() {
  TCNT0 = 0;
  OCR0A = 255; // Compare Match
  // enable interrupt
  // CS02 = /256
  TCCR0B |= 1 << CS02;
  //TCCR0B |= 1 << CS00;

  TCNT2 = 0;
  OCR2A = 255; // Compare Match
  // enable interrupt
  // CS22 + CS21 = /256
  TCCR2B |= 1 << CS22;
  TCCR2B |= 1 << CS21;
}

void motors_knockout() {
  TCCR0B = 0;
  TCCR2B = 0;
}

int pan_linearize = 0;
#define CUTOFF 250
#define LINEARIZE_FACTOR 50
void motors_step_pan_speed() {
  if (pan_linearize > 0) {
    pan_linearize--;
    return;
  }

  pan_linearize = pan_speed / LINEARIZE_FACTOR;
  pan_linearize *= pan_linearize;
  pan_linearize *= pan_linearize;

  int diff_pan = pan_speed_goal - pan_speed;
  if (-pan_acceleration < diff_pan && diff_pan < +pan_acceleration) {
    // already or almost there
    pan_speed = pan_speed_goal;
    return;
  }

  /*
  if (pan_speed > 0)
    pan_linearize = +(pan_speed * pan_speed) / LINEARIZE_FACTOR;
  else
    pan_linearize = -(pan_speed * pan_speed) / LINEARIZE_FACTOR;
  */

  if (diff_pan > 0) {
    if (pan_speed + pan_acceleration > +CUTOFF)
      pan_speed = +CUTOFF;
    else
      pan_speed += pan_acceleration;
  } else {
    if (pan_speed - pan_acceleration < -CUTOFF)
      pan_speed = -CUTOFF;
    else
      pan_speed -= pan_acceleration;
  }
}

void motors_step_tilt_speed() {
  int diff_tilt = tilt_speed_goal - tilt_speed;
  if (-tilt_acceleration < diff_tilt && diff_tilt < +tilt_acceleration) {
    // already or almost there
    tilt_speed = tilt_speed_goal;
  } else if (diff_tilt > 0) {
    if (tilt_speed + tilt_acceleration > CUTOFF)
      tilt_speed = +CUTOFF;
    else
      tilt_speed += tilt_acceleration;
  } else {
    if (tilt_speed - tilt_acceleration < -CUTOFF)
      tilt_speed = -CUTOFF;
    else
      tilt_speed -= tilt_acceleration;
  }
}

ISR(TIMER0_COMPA_vect) {
  TCNT0 = 0;

  if        (pan_speed > 0) {
    digitalWrite(PAN_STEP_PIN, HIGH);
    digitalWrite(PAN_DIR_PIN,   LOW);
    OCR0A = 255 - pan_speed; // Compare Match
  } else if (pan_speed < 0) {
    digitalWrite(PAN_STEP_PIN, HIGH);
    digitalWrite(PAN_DIR_PIN,  HIGH);
    OCR0A = 255 + pan_speed; // Compare Match
  }

  motors_step_pan_speed();

  digitalWrite(PAN_STEP_PIN, LOW);
}

ISR(TIMER2_COMPA_vect) {
  TCNT2 = 0;

  if        (tilt_speed > 0) {
    digitalWrite(TILT_STEP_PIN, HIGH);
    digitalWrite(TILT_DIR_PIN,   LOW);
    OCR2A = 255 - tilt_speed; // Compare Match
  } else if (tilt_speed < 0) {
    digitalWrite(TILT_STEP_PIN, HIGH);
    digitalWrite(TILT_DIR_PIN,  HIGH);
    OCR2A = 255 + tilt_speed; // Compare Match
  }

  motors_step_tilt_speed();

  digitalWrite(TILT_STEP_PIN, LOW);
}

void motors_driver_setup(TMC2130Stepper &driver) {
  driver.begin();
  driver.rms_current(800);
  driver.stealthChop(1);
  driver.stealth_autoscale(1);
  driver.microsteps(MICROSTEPS);
  driver.intpol(1);
  driver.hold_current(10);
  driver.hold_delay(2);
  driver.power_down_delay(32);
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
  motors_driver_setup(pan_driver);
  motors_driver_setup(tilt_driver);
  motors_enable();
  motors_setup_timer();
  motors_kickstart();
}

void motors_step() {
}

void motors_step_speeds() {
  // TODO unused?
  motors_step_pan_speed();
  motors_step_tilt_speed();
}
