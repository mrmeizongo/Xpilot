#include "SysConfig.h"
// ATmega328p pin definitions
/*
 * ISR vectors
 * All input pins use pin change interrupts
 * Depending on airplane type selected, input interrupt pins must be defined
 * Best to keep these unchanged unless absolutely necessary
 * Changing any XXXXPIN_INT or XXXXPIN_INPUT value requires modifications to PinChangeInterruptSettings.h
 */

// Input pins
#define THROTTLEPIN_INPUT 2
#define AILPIN_INPUT 3
#define ELEVPIN_INPUT 4
#define RUDDPIN_INPUT 5
#define AUX1PIN_INPUT 6
#define AUX2PIN_INPUT 7

// Output pins
#define THROTTLEPIN_OUTPUT 8
#define AIL1PIN_OUTPUT 9
#define AIL2PIN_OUTPUT 10
#define ELEVPIN_OUTPUT 11
#define RUDDPIN_OUTPUT 12

// Interrupt pins
// See lib/PinChangeInterrupt/src/PinChangeInterruptSettings.h
#define THROTTLEPIN_INT 18
#define AILPIN_INT 19
#define ELEVPIN_INT 20
#define RUDDPIN_INT 21
#define AUX1PIN_INT 22
#define AUX2PIN_INT 23