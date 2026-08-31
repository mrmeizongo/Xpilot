#ifndef _SYSTEM_CONFIG_H
#define _SYSTEM_CONFIG_H

#define SYSTEM_CONFIG_VERSION "3"

// Uncomment to use flaperons
// #define USE_FLAPERONS

#if defined(USE_FLAPERONS)
#define USE_AUXIN2
#endif

// Uncomment to enable auxiliary input 3
// #define USE_AUXIN3

// Uncomment to enable auxiliary output channel 1
// #define USE_AUXOUT1

// Enable communication with xp_serial.py
#define USE_SERIAL_TASK

// Task scheduler config
#define CONTROL_LOOP_RATE_HZ 250                     // Control loop period in hz
#define IMU_UPDATE_RATE_HZ CONTROL_LOOP_RATE_HZ      // IMU update period in hz
#define FLIGHT_MODE_RUN_RATE_HZ CONTROL_LOOP_RATE_HZ // Flight mode run period in hz
#define FLIGHT_MODE_UPDATE_RATE_HZ 25                // Flight mode update period in hz
#define RADIO_INPUT_PROCESS_RATE_HZ 45.45            // Radio input period in hz, Spektrum uses 22ms frame rates
#define WRITE_SERVO_RATE_HZ 50                       // Write servo output period in hz
#define IMU_PRINT_RATE_HZ 4                          // IMU debug print period in hz
#define TASK_PRINT_RATE_HZ 2                         // Task rate debug print period in hz
#define IO_PRINT_RATE_HZ 2                           // IO debug print period in hz
#define SERIAL_TASK_RATE_HZ 20                       // Serial task period in hz

// Debug config

/*
 * Uncomment to enable the respective debugging
 * CAUTION: Only uncomment one debug option at a time
 */
// #define SCHEDULER_RATE_DEBUG
// #define IMU_DEBUG
// #define IO_DEBUG

// #define PRINT_IMU_TASK_STAT
// #define PRINT_RADIO_TASK_STAT
// #define PRINT_FM_RUN_TASK_STAT
// #define PRINT_FM_UPDATE_TASK_STAT
// #define PRINT_SERVO_TASK_STAT

// Any debugging should disable serial communication with xp_serial.py
#if defined(SCHEDULER_RATE_DEBUG) || defined(IMU_DEBUG) || defined(IO_DEBUG) || defined(PRINT_IMU_TASK_STAT) || defined(PRINT_RADIO_TASK_STAT) || defined(PRINT_FM_RUN_TASK_STAT) || defined(PRINT_FM_UPDATE_TASK_STAT) || defined(PRINT_SERVO_TASK_STAT)
#if defined(USE_SERIAL_TASK)
#undef USE_SERIAL_TASK
#endif
#endif
// ------------------------------------------------------------------------------------------------------
#endif // _SYSTEM_CONFIG_H