#ifndef _BOARD_CONFIG_H
#define _BOARD_CONFIG_H
// ATmega328p pin definitions

/*
 * ISR vectors
 * All input pins use pin change interrupts
 * Depending on airplane type selected, input interrupt pins must be defined
 * Change these values to match your selected input pins
 * Best to keep these unchanged unless absolutely necessary
 * Changing any XXXXPIN_INT or XXXXPIN_INPUT value require modifications made to PinChangeInterrupt library
 *
 * | PCINT |  Uno/Nano/Mini  |
 * | ----- | --------------- |
 * |     0 |  8       (PB0)  |
 * |     1 |  9       (PB1)  |
 * |     2 | 10 SS    (PB2)  |
 * |     3 | 11 MISO  (PB3)  |
 * |     4 | 12 MOSI  (PB4)  |
 * |     5 | 13 SCK   (PB5)  |
 * |     6 |    XTAL1 (PB6)* |
 * |     7 |    XTAL2 (PB7)* |
 * | ----- | --------------- |
 * |     8 | A0       (PC0)  |
 * |     9 | A1       (PC1)  |
 * |    10 | A2       (PC2)  |
 * |    11 | A3       (PC3)  |
 * |    12 | A4 SDA   (PC4)  |
 * |    13 | A5 SDC   (PC5)  |
 * |    14 |    RST   (PC6)* |
 * |    15 |                 |
 * | ----- | --------------- |
 * |    16 |  0 RX    (PD0)  |
 * |    17 |  1 TX    (PD1)  |
 * |    18 |  2 INT0  (PD2)  |
 * |    19 |  3 INT1  (PD3)  |
 * |    20 |  4       (PD4)  |
 * |    21 |  5       (PD5)  |
 * |    22 |  6       (PD6)  |
 * |    23 |  7       (PD7)  |
 * | ----- | --------------- |
 */

#define AILPIN_INT 18
#define ELEVPIN_INT 19
#define RUDDPIN_INT 20
#define AUXPIN_INT 21

// Input pins
#define AILPIN_INPUT 2
#define ELEVPIN_INPUT 3
#define RUDDPIN_INPUT 4
#define AUXPIN_INPUT 5

// Output pins
#define AILPIN1_OUTPUT 8
#define AILPIN2_OUTPUT 9
#define ELEVPIN_OUTPUT 10
#define RUDDPIN_OUTPUT 11
// ------------------------------------------------------------------------------------------------------
#endif // _BOARD_CONFIG_H