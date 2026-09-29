#ifndef _SYSTEM_CONFIG_H
#define _SYSTEM_CONFIG_H

#define SYSTEM_CONFIG_VERSION "4"

/*
 * Uncomment to use flaperons
 * For flap/flaperon control, a 3 position switch on the TX is used
 * The same output can be used to directly control flap servos
 * 
 * Positions
 * 0-------1------2
 * Low---Trim---High
 * 
 * High = flap/flaperon flush with wing
 * Trim = 50% down
 * Low = 100% down
 */
// #define ENABLE_FLAPERONS 1

// #define ENABLE_LED_NOTIFIER 1

// Enable communication with xp_serial.py
#define USE_SERIAL_TASK 1

// Task scheduler config
#define CONTROL_LOOP_HZ 250       // Control loop period in hz
#define STATE_UPDATE_HZ 25        // Flight mode change period in hz
#define FLIGHT_MODE_UPDATE_HZ 50  // Flight mode update period in hz
#define FLIGHT_MODE_OUTPUT_HZ 50  // Flight mode output period in hz
#define RADIO_INPUT_PROCESS_HZ 50 // Radio input period in hz
#define TASK_PRINT_HZ 2           // Task rate debug print period in hz
#define SERIAL_TASK_HZ 20         // Serial task period in hz

// Debug config

/*
 * Uncomment to enable the respective debugging
 * CAUTION: Only uncomment one debug option at a time
 */
// #define PRINT_SCHEDULER_RATE 1
// #define PRINT_IO 1

// #define PRINT_IMU 1

// #define PRINT_IMU_TASK_STAT 1
// #define PRINT_RADIO_TASK_STAT 1
// #define PRINT_STATE_UPDATE_TASK_STAT 1
// #define PRINT_FM_UPDATE_TASK_STAT 1
// #define PRINT_FM_RUN_TASK_STAT 1
// #define PRINT_FM_OUTPUT_TASK_STAT 1

// Any debugging should disable serial communication with xp_serial.py
#if defined(PRINT_SCHEDULER_RATE) || defined(PRINT_IO) || defined(PRINT_IMU) || defined(PRINT_IMU_TASK_STAT) ||             \
    defined(PRINT_RADIO_TASK_STAT) || defined(PRINT_STATE_UPDATE_TASK_STAT) || defined(PRINT_FM_UPDATE_TASK_STAT) ||        \
    defined(PRINT_FM_RUN_TASK_STAT) || defined(PRINT_FM_OUTPUT_TASK_STAT)
#if defined(USE_SERIAL_TASK)
#undef USE_SERIAL_TASK
#endif
#endif
// ------------------------------------------------------------------------------------------------------
#endif // _SYSTEM_CONFIG_H