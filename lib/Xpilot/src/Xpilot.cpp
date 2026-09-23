#include <Arduino.h>
#include "IMU.h"
#include "Radio.h"
#include "Debug.h"
#include "Actuators.h"
#include "SysConfig.h"
#include "Scheduler.h"
#include "FlightConfigAccess.h"

static constexpr uint32_t SERIAL_BAUD_RATE = 250000; // Serial baud rate

// Task handlers for the scheduler to manage periodic tasks
uint8_t Xpilot::imuTaskId = 0;
uint8_t Xpilot::radioTaskId = 0;
uint8_t Xpilot::flightModeChangeTaskId = 0;
uint8_t Xpilot::flightModeUpdateTaskId = 0;
uint8_t Xpilot::flightModeRunTaskId = 0;
uint8_t Xpilot::flightModeOutputTaskId = 0;

Xpilot::Xpilot()
    : serialConfigTask{Serial}
{
}

void Xpilot::setup(void)
{
    sysInit();

    /*
     * Tasks added to the scheduler list are the only ones executed in the loop function
     * The order tasks are added determines priority and is critical.
     * Priority is in descending order
     */
    imuTaskId = scheduler.addTask(&IMU::getLatestReadingsTask, &imu, IMU_UPDATE_HZ);
    radioTaskId = scheduler.addTask(&Radio::processInputTask, &radio, RADIO_INPUT_PROCESS_HZ);
    flightModeChangeTaskId = scheduler.addTask(&Xpilot::changeFlightModeTask, this, FLIGHT_MODE_CHANGE_HZ);
    flightModeUpdateTaskId = scheduler.addTask(&Mode::updateInput, &currentMode, FLIGHT_MODE_UPDATE_HZ);
    flightModeRunTaskId = scheduler.addTask(&Mode::runControllers, &currentMode, FLIGHT_MODE_RUN_HZ);
    flightModeOutputTaskId = scheduler.addTask(&Mode::processOutput, &currentMode, FLIGHT_MODE_OUTPUT_HZ);

#if defined(PRINT_SCHEDULER_RATE)
    (void)scheduler.addTask(&Xpilot::printSchedulerRateTask, this, TASK_PRINT_HZ);
#endif
#if defined(PRINT_IO)
    (void)scheduler.addTask(&Xpilot::printIOTask, this, TASK_PRINT_HZ);
#endif
#if defined(PRINT_IMU)
    (void)scheduler.addTask(&IMU::printIMUTask, &imu, TASK_PRINT_HZ);
#endif
#if defined(PRINT_IMU_TASK_STAT)
    (void)scheduler.addTask(&Xpilot::printIMUTaskStatTask, this, TASK_PRINT_HZ);
#endif
#if defined(PRINT_RADIO_TASK_STAT)
    (void)scheduler.addTask(&Xpilot::printRadioTaskStatTask, this, TASK_PRINT_HZ);
#endif
#if defined(PRINT_FM_CHANGE_TASK_STAT)
    (void)scheduler.addTask(&Xpilot::printFlightModeChangeTaskStatTask, this, TASK_PRINT_HZ);
#endif
#if defined(PRINT_FM_UPDATE_TASK_STAT)
    (void)scheduler.addTask(&Xpilot::printFlightModeUpdateTaskStatTask, this, TASK_PRINT_HZ);
#endif
#if defined(PRINT_FM_RUN_TASK_STAT)
    (void)scheduler.addTask(&Xpilot::printFlightModeRunTaskStatTask, this, TASK_PRINT_HZ);
#endif
#if defined(PRINT_FM_OUTPUT_TASK_STAT)
    (void)scheduler.addTask(&Xpilot::printFlightModeOutputTaskStatTask, this, TASK_PRINT_HZ);
#endif

#if defined(USE_SERIAL_TASK)
    scheduler.addTask(&Xpilot::runSerialConfigTask, this, SERIAL_TASK_HZ);
#endif

    scheduler.init();
}

// Main Xpilot execution loop
void Xpilot::loop(void) { scheduler.runTasks(); }

void Xpilot::sysInit(void)
{
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial)
        ; // Wait for Serial port to open

    configManager.init(); // Initiailize configuration manager

    // Initialize systems
    imu.init();
    radio.init();
    actuators.init();
    currentMode->init();

    currentMode = &rateMode; // Rate mode is the default mode of operation on startup
}

void Xpilot::changeFlightMode(void)
{
    Mode* requestedMode = currentMode;

    if (radio.inFailsafe())
    {
        sysFailsafeActive = true;

        if (currentMode == &stabilizeMode)
            return;

        // Stabilize mode is the default failsafe mode
        requestedMode = &stabilizeMode;
    }
    else
    {
        sysFailsafeActive = false;

        const Radio::THREE_POS_SW switchPos =
            radio.getThreeSwitchPos(Radio::CHANNEL::AUX1, config().aux1RxConfig.trim, THREE_POS_SW_SEP);

        if (switchPos == Radio::THREE_POS_SW::UNDEFINED)
            return;

        // Mode select switch position has not changed
        if (switchPos == currentMode->getModeSwitchPosition())
            return;

        if (switchPos == passthroughMode.getModeSwitchPosition())
            requestedMode = &passthroughMode;

        else if (switchPos == rateMode.getModeSwitchPosition())
            requestedMode = &rateMode;

        else if (switchPos == stabilizeMode.getModeSwitchPosition())
            requestedMode = &stabilizeMode;

        else
            return;
    }

    currentMode->exit();

    currentMode = requestedMode;
    currentMode->enter();
}
// ---------------------------