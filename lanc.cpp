#include "lanc.hpp"

#include "input.hpp"
#include "motors.hpp"

#include <Arduino.h>

// TODO move to .h

// ticks per µs =  1µs ×   clock    ÷ prescaler
//              = 10⁻⁶ × 16_000_000 ÷ 8
//              = 2
uint8_t ticks_per_us = 2;
uint8_t bit_duration = 104 * ticks_per_us;

// timing compensation
uint8_t timing_delay =   0 * ticks_per_us;
uint8_t timing_shift =  13 * ticks_per_us;

void lanc_setup_external_interrupt() {
  EIMSK |= (1 << INT0); // External Interrupt Mask
  // 10 in ISC01:ISC00 for falling edge
  EICRA |= (1 << ISC01);
  EICRA |= (0 << ISC00); // no-op
}

void lanc_setup_timer() {
  TCCR1A = 0; // Counter Control Register A
  TCCR1B = 0; // Counter Control Register B
  TCNT1  = 0; // Counter Value

  TCCR1A |= (1 << WGM11);  // WGM21 – Waveform Generation CTC mode
  // TCCR2B |= 1 << CS21;   // CS21 – /8 clock select
  TIMSK1 |= (1 << OCIE1A); // Timer 2 Mask
}

void lanc_setup() {
  pinMode(LANC_PIN_IN, INPUT); // listens to the LANC line
  pinMode(LANC_PIN_OUT, OUTPUT); // writes to the LANC line

  lanc_setup_external_interrupt();
  lanc_setup_timer();
}

void lanc_kickstart() {
  EIMSK &= ~(1 << INT0); // disable pin interrupt
  TCNT1 = 0;
  OCR1A = bit_duration - timing_delay - timing_shift; // Compare Match
  // enable interrupt
  TCCR1B |= 1 << CS11; // CS21 – /8 clock select
}

void lanc_knockout() {
  TCCR1B = 0;
  EIFR = (1 << INTF0); // External Interrupt Flag Register
  EIMSK |= (1 << INT0); // re-enable pin interrupt
}


// ⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠧⠯⠯⠯⠯⠯⠯⠯⠯⠏⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠧⠯⠯⠯⠯⠯⠯⠯⠯⠏⠉⠉⠉⠉⠉⠉⠉⠉⠉
//   start-bit ↑   cmd1        knockout      start-bit ↑  cmd2


volatile uint8_t cmd1 = CAM_TYPE;
volatile uint8_t cmd2 = 0b00011000;

void sendCMD(uint8_t cmd1_, uint8_t cmd2_) {
  cmd1 = cmd1_;
  cmd2 = cmd2_;
}

volatile uint8_t byte_counter = 0;
volatile uint8_t bit_counter = 0;

volatile bool zooming = false;

ISR(INT0_vect) {
  if (byte_counter != 0 || zooming)
    lanc_kickstart();
  zooming = false;
}

ISR(TIMER1_COMPA_vect) {
  if (bit_counter <= 7) {
    if      (byte_counter == 0)
      digitalWrite(LANC_PIN_OUT, (cmd1 & (1 << bit_counter)) ? HIGH : LOW); // bang
    else if (byte_counter == 1)
      digitalWrite(LANC_PIN_OUT, (cmd2 & (1 << bit_counter)) ? HIGH : LOW); // bang

    TCNT1 = 0;
    OCR1A = bit_duration - timing_delay; // Compare Match
    bit_counter++;
  } else {
    digitalWrite(LANC_PIN_OUT, LOW); // idle
    // TODO ↑ won't we retrigger ourself?
    bit_counter = 0;
    if (byte_counter++ == 7)
      byte_counter = 0;
    lanc_knockout();
  }
}


void zoom(int speed) {
  switch (speed) {
  case 0: //speed 1
    sendCMD(CAM_TYPE, B00010000);
    break;
  case -1: //speed 2
    sendCMD(CAM_TYPE, B00010010);
    break;
  case -2: //speed 3
    sendCMD(CAM_TYPE, B00010100);
    break;
  case -3: //speed 4
    sendCMD(CAM_TYPE, B00010110);
    break;
  case -4: //speed 5
    sendCMD(CAM_TYPE, B00011000);
    break;
  case -5: //speed 6
    sendCMD(CAM_TYPE, B00011010);
    break;
  case -6: //speed 7
    sendCMD(CAM_TYPE, B00011100);
    break;
  case -7: //speed 8
    sendCMD(CAM_TYPE, B00011110);
    break;

  case 1: //speed 1
    sendCMD(CAM_TYPE, B00000000);
    break;
  case 2: //speed 2
    sendCMD(CAM_TYPE, B00000010);
    break;
  case 3: //speed 3
    sendCMD(CAM_TYPE, B00000100);
    break;
  case 4: //speed 4
    sendCMD(CAM_TYPE, B00000110);
    break;
  case 5: //speed 5
    sendCMD(CAM_TYPE, B00001000);
    break;
  case 6: //speed 6
    sendCMD(CAM_TYPE, B00001010);
    break;
  case 7: //speed 7
    sendCMD(CAM_TYPE, B00001100);
    break;
  case 8: //speed 8
    sendCMD(CAM_TYPE, B00001110);
    break;
  }
}

// some zoom ramping
uint8_t easing_hack = 1;
uint8_t zoom_multiplier = 8 * easing_hack;
int16_t zoom_speed = 0;
uint8_t lanc_rate_counter = 0;
uint16_t lanc_silence_counter = 0;

void lanc_step() {
  int lanc_5v = digitalRead(LANC_PIN_IN);
  if (!zooming && lanc_5v) {
    if (lanc_silence_counter++ > 500) {
      int16_t target_multiplied = target_zoom_speed * zoom_multiplier;
      if (zoom_speed != target_multiplied) {
        int8_t inc = target_multiplied - zoom_speed > 0 ? +1 : -1;
        if (-zoom_multiplier * 2 < zoom_speed && zoom_speed < +zoom_multiplier * 2) {
          // first zoom level, do it slowly
        } else {
          inc *= easing_hack;
        }
        zoom_speed += inc;
      }
      int8_t zoom_command = zoom_speed / zoom_multiplier;
      if (zoom_command) {
        zoom(zoom_command);
        zooming = true;
      }

      lanc_silence_counter = 0;
      //motors_step_speeds();
    }
  } else {
    lanc_silence_counter = 0;
  }
}
