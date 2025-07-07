#ifndef _SYSTEM_CONFIG_H
#define _SYSTEM_CONFIG_H
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

// System variables
#define BAUD_RATE 9600               // Serial baud rate
#define INPUT_REFRESH_RATE_US 20000U // Radio input refresh rate in microseconds (20ms)
#define ONEHZ_LOOP_US 1000000U       // 1Hz loop rate in microseconds
#define IMU_WARMUP_LOOP 1000U        // Number of IMU warmup loops before initial use
#define MPU6050_ADDRESS 0x68         // I2C address of MPU6050
#define I2C_CLOCK_400KHZ 400000U     // I2C clock speed in Hz
#define CALIBRATE_MEMORY_OFFSET 0    // EEPROM offset for calibration data. Change only if you have data in EEPROM that you want to keep
#define FAILSAFE_TIMEOUT_MS 1500U    // Failsafe timeout in milliseconds
#define FAILSAFE_TOLERANCE 200U      // Failsafe tolerance in microseconds
// ------------------------------------------------------------------------------------------------------
#endif // _SYSTEM_CONFIG_H