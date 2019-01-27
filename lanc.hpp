#pragma once

#define LANC_PIN_IN       2 // 5V limited input signal from LANC data
#define LANC_PIN_OUT      4 // Command to send to LANC


void lanc_setup();
void lanc_tick();
