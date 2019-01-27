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

/// Configures external interrupt for start bit detection.
void lanc_setup_external_interrupt() {
  EIMSK |= (1 << INT0); // External Interrupt Mask
  // 10 in ISC01:ISC00 for falling edge
  EICRA |= (1 << ISC01);
  EICRA |= (0 << ISC00); // no-op
}

/// Configures the timer for LANC.
void lanc_setup_timer() {
  TCCR1A = 0; // Counter Control Register A
  TCCR1B = 0; // Counter Control Register B
  TCNT1  = 0; // Counter Value

  TCCR1A |= (1 << WGM11);  // WGM21 – Waveform Generation CTC mode
  TIMSK1 |= (1 << OCIE1A); // Timer 2 Mask
  // prescaler is configured later in lanc_kickstart()
}

/// Configures pins and calls lanc_setup_external_interrupt() and
/// lanc_setup_timer().
void lanc_setup() {
  pinMode(LANC_PIN_IN,   INPUT);
  pinMode(LANC_PIN_OUT, OUTPUT);

  lanc_setup_external_interrupt();
  lanc_setup_timer();
}

/// Starts timed output of bits.
///
/// /verbatim
///
/// ⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠧⠯⠯⠯⠯⠯⠯⠯⠯⠏⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠧⠯⠯⠯⠯⠯⠯⠯⠯⠏⠉⠉⠉⠉⠉⠉⠉⠉⠉
///              ⬈ ↑↑↑↑↑↑↑↑ ⬉                            ⬈ ↑↑↑↑↑↑↑↑ ⬉
///     start bit    cmd1    stop bit           start bit    cmd1    stop bit
///    (kickstart)          (knockout)         (kickstart)          (knockout)
///
/// \endverbatim
void lanc_kickstart() {
  EIMSK &= ~(1 << INT0); // disable pin interrupt
  TCNT1 = 0;
  OCR1A = bit_duration - timing_delay - timing_shift; // Compare Match
  // enable interrupt
  TCCR1B |= 1 << CS11; // CS21 – /8 clock select
}

/// Stops the timer and re-enables external interrupt.
void lanc_knockout() {
  TCCR1B = 0;
  EIFR = (1 << INTF0); // External Interrupt Flag Register
  EIMSK |= (1 << INT0); // re-enable pin interrupt
}


volatile uint8_t cmd1 = 0;
volatile uint8_t cmd2 = 0;

/// Schedules a lanc command (the data will be sent later, if ever).
void send_command(uint8_t cmd1_, uint8_t cmd2_) {
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

/// Send a zoom command to the camera (-8 … +8).
///
/// Per http://www.boehmel.de/lanc.htm:
/// ① 0b0001_1000 – Normal  command to VTR or video camera
/// ② 0b0010_1000 – Special command to        video camera
/// ③ 0b0011_1000 – Special command to VTR
/// ④ 0b0001_1110 – Normal  command to still  video camera
///
/// There are zooming commands in both ② and ④, and we will use ②.
void zoom(int8_t speed) {
  uint8_t absolute = speed > 0 ? +speed : -speed;
  if (absolute > 8)
    return; // no such LANC command
  if (absolute == 0)
    return; // nothing to do

  uint8_t byte1 = 0b00101000;
  uint8_t byte2 = ((speed < 0) << 4) | ((absolute - 1) << 1);
  send_command(byte1, byte2);
}

// some zoom ramping
uint8_t easing_hack = 1;
uint8_t zoom_multiplier = 8 * easing_hack;
int16_t zoom_speed = 0;
uint8_t lanc_rate_counter = 0;
uint16_t lanc_silence_counter = 0;

void lanc_tick() {
  int lanc_5v = digitalRead(LANC_PIN_IN);
  if (!zooming && lanc_5v) {
    if (lanc_silence_counter++ > 500) {
      int16_t target_multiplied = zoom_speed_goal * zoom_multiplier;
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
