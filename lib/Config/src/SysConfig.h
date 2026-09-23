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
// #define USE_FLAPERONS

// Enable communication with xp_serial.py
#define USE_SERIAL_TASK

// Task scheduler config
#define CONTROL_LOOP_HZ 250                // Control loop period in hz
#define IMU_UPDATE_HZ CONTROL_LOOP_HZ      // IMU update period in hz
#define FLIGHT_MODE_RUN_HZ CONTROL_LOOP_HZ // Flight mode run period in hz
#define FLIGHT_MODE_CHANGE_HZ 25           // Flight mode change period in hz
#define FLIGHT_MODE_UPDATE_HZ 50           // Flight mode update period in hz
#define FLIGHT_MODE_OUTPUT_HZ 50           // Flight mode output period in hz
#define RADIO_INPUT_PROCESS_HZ 50          // Radio input period in hz
#define TASK_PRINT_HZ 2                    // Task rate debug print period in hz
#define SERIAL_TASK_HZ 20                  // Serial task period in hz

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
// #define PRINT_FM_UPDATE_TASK_STAT
// #define PRINT_FM_MODE_INPUT_UPDATE_TASK_STAT
// #define PRINT_FM_RUN_TASK_STAT
// #define PRINT_FM_OUTPUT_TASK_STAT

// Any debugging should disable serial communication with xp_serial.py
#if defined(SCHEDULER_RATE_DEBUG) || defined(IMU_DEBUG) || defined(IO_DEBUG) || defined(PRINT_IMU_TASK_STAT) ||             \
    defined(PRINT_RADIO_TASK_STAT) || defined(PRINT_FM_RUN_TASK_STAT) || defined(PRINT_FM_UPDATE_TASK_STAT) ||              \
    defined(PRINT_FM_OUTPUT_TASK_STAT)
#if defined(USE_SERIAL_TASK)
#undef USE_SERIAL_TASK
#endif
#endif
// ------------------------------------------------------------------------------------------------------
#endif // _SYSTEM_CONFIG_H