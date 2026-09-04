#include <Arduino.h>
#include "IMU.h"
#include "Radio.h"
#include "Debug.h"
#include "Actuators.h"
#include "SystemConfig.h"
#include "FlightConfigAccess.h"

static constexpr uint32_t SERIAL_BAUD_RATE = 250000; // Serial baud rate

// Task handlers for the scheduler to manage periodic tasks
uint8_t Xpilot::imuTaskId = 0;
uint8_t Xpilot::radioTaskId = 0;
uint8_t Xpilot::flightModeUpdateTaskId = 0;
uint8_t Xpilot::flightModeInputUpdateTaskId = 0;
uint8_t Xpilot::flightModeRunTaskId = 0;
uint8_t Xpilot::actuatorTaskId = 0;
uint8_t Xpilot::serialConfigTaskId = 0;

Xpilot::Xpilot()
    : serialConfigTask{Serial, configManager}
{
}

void Xpilot::setup(void)
{
    sysInit();

    /*
     * Initialize the scheduler and add system tasks
     * The order tasks are added determines priority, with the highest priority first.
     */
    imuTaskId = scheduler.addTask(&IMU::getLatestReadingsTask, &imu, IMU_UPDATE_RATE_HZ);
    radioTaskId = scheduler.addTask(&Radio::processInputTask, &radio, RADIO_INPUT_PROCESS_RATE_HZ);
    flightModeUpdateTaskId = scheduler.addTask(&Xpilot::updateFlightModeTask, this, FLIGHT_MODE_UPDATE_RATE_HZ);
    flightModeInputUpdateTaskId = scheduler.addTask(&Mode::updateInput, &currentMode, FLIGHT_MODE_RUN_RATE_HZ);
    flightModeRunTaskId = scheduler.addTask(&Mode::runTask, &currentMode, FLIGHT_MODE_RUN_RATE_HZ);
    actuatorTaskId = scheduler.addTask(&Actuators::writeServosTask, &actuators, WRITE_SERVO_RATE_HZ);

#if defined(IO_DEBUG)
    (void)scheduler.addTask(&Xpilot::printIOTask, this, IO_PRINT_RATE_HZ);
#endif
#if defined(IMU_DEBUG)
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
    (void)scheduler.addTask(&Xpilot::printActuatorTaskStatTask, this, TASK_PRINT_RATE_HZ);
#endif

#if defined(USE_SERIAL_TASK)
    serialConfigTaskId = scheduler.addTask(&Xpilot::runSerialConfigTask, this, SERIAL_TASK_RATE_HZ);
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

    // Specify the mode switch position for each mode
    passthroughMode.setModeSwitchPosition(Radio::THREE_POS_SW::HIGH_POS);
    rateMode.setModeSwitchPosition(Radio::THREE_POS_SW::MID_POS);
    stabilizeMode.setModeSwitchPosition(Radio::THREE_POS_SW::LOW_POS);

    currentMode = &rateMode; // Rate mode is the default mode of operation on startup
}

void Xpilot::updateFlightMode(void)
{
    bool radioInFailSafe = radio.inFailsafe();

    // Failsafe already processed
    if (radioInFailSafe && sysFailsafeActive)
        return;

    Mode* requestedMode = currentMode;

    if (radioInFailSafe)
    {
        sysFailsafeActive = true;
        requestedMode = &stabilizeMode;
    }
    else
    {
        sysFailsafeActive = false;

        const Radio::THREE_POS_SW switchPos = radio.getThreeSwitchPos(Radio::CHANNEL::AUX1);

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