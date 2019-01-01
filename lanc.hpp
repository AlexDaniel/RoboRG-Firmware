#pragma once

#define LANC_PIN_IN       2 // 5V limited input signal from LANC data
#define LANC_PIN_OUT      4 // Command to send to LANC
#define CAM_TYPE 0b00101000 // Should work in most cases


void lanc_step();
void lanc_setup();
