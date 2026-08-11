#include <Arduino.h>
#include "Xpilot.h"
#include "PlaneConfig.h"
#include "SystemConfig.h"
#include "IMU.h"
#include "Radio.h"
#include "Actuators.h"
#include "Debug.h"

// Task handlers for the scheduler to manage periodic tasks
uint8_t Xpilot::imuTaskId = 0;
uint8_t Xpilot::radioTaskId = 0;
uint8_t Xpilot::flightModeUpdateTaskId = 0;
uint8_t Xpilot::flightModeRunTaskId = 0;
uint8_t Xpilot::actuatorTaskId = 0;

void Xpilot::setup(void)
{
    sysInit();

    /*
     * Initialize the scheduler and add system tasks
     * The order tasks are added determines priority, with the highest priority first.
     */
    imuTaskId = scheduler.addTask(&IMU::getLatestReadingsTask, &imu, IMU_UPDATE_RATE_HZ);
    radioTaskId = scheduler.addTask(&Radio::processInputTask, &radio, RADIO_INPUT_PROCESS_RATE_HZ);
    flightModeRunTaskId = scheduler.addTask(&Mode::runTask, &currentMode, FLIGHT_MODE_RUN_RATE_HZ);
    actuatorTaskId = scheduler.addTask(&Actuators::writeServosTask, &actuators, WRITE_SERVO_RATE_HZ);
    flightModeUpdateTaskId = scheduler.addTask(&Xpilot::updateFlightModeTask, this, FLIGHT_MODE_UPDATE_RATE_HZ);
#if defined(IO_DEBUG)
    (void)scheduler.addTask(&Xpilot::printIOTask, this, IO_PRINT_RATE_HZ);
#endif
#if defined(IMU_DEBUG) || defined(CALIBRATE_DEBUG)
    (void)scheduler.addTask(&Xpilot::printIMUTask, this, IMU_PRINT_RATE_HZ);
#endif
#if defined(SCHEDULER_RATE_DEBUG)
    (void)scheduler.addTask(&Xpilot::printSchedulerRateTask, this, TASK_PRINT_RATE_HZ);
#endif
#if defined(PRINT_IMU_TASK_STAT)
    (void)scheduler.addTask(&Xpilot::printIMUTaskStatTask, this, TASK_PRINT_RATE_HZ);
#endif
#if defined(PRINT_RADIO_TASK_STAT)
    (void)scheduler.addTask(&Xpilot::printRadioTaskStatTask, this, TASK_PRINT_RATE_HZ);
#endif
#if defined(PRINT_FM_RUN_TASK_STAT)
    (void)scheduler.addTask(&Xpilot::printFlightModeRunTaskStatTask, this, TASK_PRINT_RATE_HZ);
#endif
#if defined(PRINT_FM_UPDATE_TASK_STAT)
    (void)scheduler.addTask(&Xpilot::printFlightModeUpdateTaskStatTask, this, TASK_PRINT_RATE_HZ);
#endif
#if defined(PRINT_SERVO_TASK_STAT)
    (void)scheduler.addTask(&Xpilot::printServoTaskStatTask, this, TASK_PRINT_RATE_HZ);
#endif
    scheduler.init();
}

// Main Xpilot execution loop
void Xpilot::loop(void)
{
    scheduler.runTasks();
}

void Xpilot::sysInit(void)
{
#if defined(DEBUG)
    Serial.begin(BAUD_RATE);
    while (!Serial)
        ; // Wait for Serial port to open
#endif

    // Specify the mode switch position for each mode
    passthroughMode.setModeSwitchPosition(Radio::THREE_POS_SW::HIGH_POS);
    rateMode.setModeSwitchPosition(Radio::THREE_POS_SW::MID_POS);
    stabilizeMode.setModeSwitchPosition(Radio::THREE_POS_SW::LOW_POS);

#if defined(DEFAULT_TO_PASSTHROUGH_MODE)
    currentMode = &passthroughMode;
#elif defined(DEFAULT_TO_RATE_MODE)
    currentMode = &rateMode;
#elif defined(DEFAULT_TO_STABILIZE_MODE)
    currentMode = &stabilizeMode;
#endif
    previousMode = currentMode;

    failSafeActive = false;
    imuFaultActive = false;

    // Initialize systems
    imu.init();
    radio.init();
    actuators.init();
}

void Xpilot::updateFlightMode(void)
{
    bool radioInFailSafe = radio.inFailsafe();
    /*
     * If system failsafe has been activated and transmitter is still in fail safe,
     * or there is an active imu fault, prevent flight mode switching until cleared
     */
    if ((failSafeActive && radioInFailSafe) || imuFaultActive)
        return;

    // First time detecting radio in failsafe
    if (radioInFailSafe)
    {
        failSafeActive = true; // Set failsafe active flag
#if defined(FAILSAFE_TO_STABILIZE)
        currentMode = &stabilizeMode;
#elif defined(FAILSAFE_TO_RATE)
        currentMode = &rateMode;
#elif defined(FAILSAFE_TO_PASSTHROUGH)
        currentMode = &passthroughMode;
#endif
    }
    else if (currentMode->getFaultState())
    {
        /*
         * If imu faulted and stopped returning ahrs values, default to passthrough until reset
         * if already in a non-imu assisted mode, simply set imu fault active
         * This will prevent user from switching flight modes until system reset is performed
         */
        if (currentMode->imuAssisted() && !imuFaultActive)
        {
            currentMode = &passthroughMode;
            imuFaultActive = true;
        }
        else if (!imuFaultActive)
        {
            imuFaultActive = true;
            return;
        }
    }
    else
    {
        // Both system and radio are not in failsafe
        failSafeActive = false; // Reset failsafe active flag
        Radio::THREE_POS_SW radioModeSwitchPos = radio.getRxAux1Pos();
        // Radio mode switch position has not changed
        if (radioModeSwitchPos == currentMode->getModeSwitchPosition())
            return;

        // Process new mode if mode switch position has changed
        if (radioModeSwitchPos == passthroughMode.getModeSwitchPosition())
            currentMode = &passthroughMode;
        else if (radioModeSwitchPos == stabilizeMode.getModeSwitchPosition())
            currentMode = &stabilizeMode;
        else if (radioModeSwitchPos == rateMode.getModeSwitchPosition())
            currentMode = &rateMode;
    }

    // Failsafe detected, mode switch position has changed, or imu has faulted perform mode transition
    previousMode->exit();
    currentMode->enter();
    previousMode = currentMode;
}
// ---------------------------