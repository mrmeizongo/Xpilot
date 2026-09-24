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
#define AILPIN_INPUT 2
#define ELEVPIN_INPUT 3
#define RUDDPIN_INPUT 4
#define AUX1PIN_INPUT 5
#define AUX2PIN_INPUT 6
#define THROTTLEPIN_INPUT 7

// Output pins
#define AIL1PIN_OUTPUT 8
#define AIL2PIN_OUTPUT 9
#define ELEVPIN_OUTPUT 10
#define RUDDPIN_OUTPUT 11
#define THROTTLEPIN_OUTPUT 12

// Interrupt pins
// See lib/PinChangeInterrupt/src/PinChangeInterruptSettings.h
#define AILPIN_INT 18
#define ELEVPIN_INT 19
#define RUDDPIN_INT 20
#define AUX1PIN_INT 21
#define AUX2PIN_INT 22
#define THROTTLEPIN_INT 23