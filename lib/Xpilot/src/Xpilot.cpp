#include <Arduino.h>
#include "IMU.h"
#include "Radio.h"
#include "Actuators.h"
#include "SysConfig.h"
#include "Scheduler.h"
#include "LEDnotifier.h"
#include "XPDebug.h"
#include "FlightConfigAccess.h"

constexpr uint32_t SERIAL_BAUD_RATE = 250000UL; // Serial baud rate

constexpr uint16_t ARM_STATE_HOLD_TIME_MS = 1000U;

uint32_t armStateStartTime = 0;

bool armStateTimerStarted = false;

Xpilot::Xpilot()
    : armState{ArmState::DISARMED}
    , sysFailsafeActive{true}
    , xpInterface{Serial}
{
}

void Xpilot::setup(void)
{
    sysInit();

    /*
     * The order tasks are added determines priority and is critical.
     * Priority is in descending order
     */
    imuTaskId = scheduler.addTask(&IMU::getLatestReadingsTask, &imu, CONTROL_LOOP_HZ);
    radioTaskId = scheduler.addTask(&Radio::processInputTask, &radio, RADIO_INPUT_PROCESS_HZ);
    stateUpdateTaskId = scheduler.addTask(&Xpilot::stateUpdateTask, this, STATE_UPDATE_HZ);
    flightModeUpdateTaskId = scheduler.addTask(&Mode::updateInput, &currentMode, FLIGHT_MODE_UPDATE_HZ);
    flightModeRunTaskId = scheduler.addTask(&Mode::runControllers, &currentMode, CONTROL_LOOP_HZ);
    flightModeOutputTaskId = scheduler.addTask(&Mode::processOutput, &currentMode, FLIGHT_MODE_OUTPUT_HZ);
    ledNotifierTaskId = scheduler.addTask(&LEDNotifier::update, nullptr, STATE_UPDATE_HZ);

#if defined(USE_SERIAL_TASK)
    (void)scheduler.addTask(&Xpilot::xpInterfaceTask, this, SERIAL_TASK_HZ);
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
    xpDebug.init();       // Initialize debug functionality

    // Initialize systems
    imu.init();
    radio.init();
    actuators.init();
    currentMode->init();

    currentMode = &rateMode; // Rate mode is the default mode of operation on startup
}

void Xpilot::stateUpdate(void)
{
    updateArmState();
    updateFlightMode();
}

void Xpilot::updateFlightMode(void)
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

        const Radio::THREE_POS_SW switchPos = radio.getThreeSwitchPos(Radio::CHANNEL::AUX1, config().aux1RxConfig.trim);

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

bool Xpilot::armDisarmInput(bool requireThrottleCut)
{
    if (requireThrottleCut && !radio.inThrottleCut())
    {
        armStateTimerStarted = false;
        return false;
    }

    const bool inputSatisfied =
        abs(config().rollRxConfig.min - static_cast<int16_t>(radio.getPWM(Radio::CHANNEL::ROLL))) <= INPUT_THRESHOLD &&
        abs(config().pitchRxConfig.max - static_cast<int16_t>(radio.getPWM(Radio::CHANNEL::PITCH))) <= INPUT_THRESHOLD &&
        abs(config().yawRxConfig.max - static_cast<int16_t>(radio.getPWM(Radio::CHANNEL::YAW))) <= INPUT_THRESHOLD;

    // Gesture must remain continuously held
    if (!inputSatisfied)
    {
        armStateTimerStarted = false;
        return false;
    }

    const uint32_t now = millis();

    if (!armStateTimerStarted)
    {
        armStateTimerStarted = true;
        armStateStartTime = now;
        return false;
    }

    if (now - armStateStartTime < ARM_STATE_HOLD_TIME_MS)
        return false;

    armStateTimerStarted = false;
    return true;
}

void Xpilot::updateArmState(void)
{
    bool requireThrottleCut = true;

    switch (armState)
    {
        case ArmState::ARMED:
        {
            if (armDisarmInput(!requireThrottleCut))
                armState = ArmState::WAITING_FOR_DISARM_RELEASE;

            break;
        }

        case ArmState::WAITING_FOR_DISARM_RELEASE:
        {
            if (radio.primarySticksCentered())
                armState = ArmState::DISARMED;

            break;
        }

        case ArmState::DISARMED:
        {
            if (armDisarmInput(requireThrottleCut))
                armState = ArmState::WAITING_FOR_ARM_RELEASE;

            break;
        }

        case ArmState::WAITING_FOR_ARM_RELEASE:
        {
            if (radio.primarySticksCentered())
            {
                armState = ArmState::ARMED;
                currentMode->resetControllers();
            }

            break;
        }
    }
}