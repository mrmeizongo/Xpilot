#pragma once

// Pin interrupt connections
// Aileron and elevator inputs use hardware interrupts
// Mode switch uses pin change interrupts
#define AILPIN_INT 2
#define ELEVPIN_INT 3
#define MODEPIN_INT 4
// #define THROTTLEPIN_INT 5
// #define RUDDPIN_INT 6

#define AILPIN_OUT 9
#define ELEVPIN_OUT 10
// #define RUDDPIN_OUT 11

#define INPUT_THRESHOLD 200
#define RECEIVER_LOW 1000
#define RECEIVER_MID 1500
#define RECEIVER_HIGH 2000

// In degree radians
#define ROLL_LIMIT 45
#define PITCH_LIMIT 45

// In angles
#define AILERON_DEFLECTION_LIM 35
#define ELEVATOR_DEFLECTION_LIM 35

#define CENTER_DEFLECTION_POS 90

// Forbid copying of object
#define CLASS_NO_COPY(c)        \
    c(const c &other) = delete; \
    c &operator=(const c &) = delete;

// Set to 1 to enable the respective debugging, zero otherwise
// To enable any of the XX_DEBUG, set DEBUG to 1
#define DEBUG 1
#define LOOP_DEBUG 0
#define IO_DEBUG 1