#include <Arduino.h>
#include <PlaneConfig.h>
#include <SystemConfig.h>
#include "Xpilot.h"
#include "Radio.h"
#include "Actuators.h"
#include "IMU.h"

// Task handlers for the scheduler to manage periodic tasks
static uint8_t imuTaskId;
static uint8_t radioTaskId;
static uint8_t flightModeUpdateTaskId;
static uint8_t flightModeRunTaskId;
static uint8_t writeServoTaskId;

Xpilot::Xpilot(void)
{
}

void Xpilot::setup(void)
{
    sysInit();

    // Initialize the scheduler and add system tasks
    imuTaskId = scheduler.addTask(imu.getLatestReadingsTask, &imu, IMU_UPDATE_RATE_HZ);
    radioTaskId = scheduler.addTask(radio.processInputTask, &radio, RADIO_INPUT_PROCESS_RATE_HZ);
    flightModeUpdateTaskId = scheduler.addTask(updateFlightModeTask, this, FLIGHT_MODE_UPDATE_RATE_HZ);
    flightModeRunTaskId = scheduler.addTask(currentMode->runTask, &currentMode, FLIGHT_MODE_RUN_RATE_HZ);
    writeServoTaskId = scheduler.addTask(Mode::servoOut, nullptr, WRITE_SERVO_RATE_HZ);
#if defined(IO_DEBUG)
    (void)scheduler.addTask(printIOTask, this, IO_PRINT_RATE_HZ);
#endif
#if defined(IMU_DEBUG) || defined(CALIBRATE_DEBUG)
    (void)scheduler.addTask(printIMUTask, this, IMU_PRINT_RATE_HZ);
#endif
#if defined(SCHEDULER_DEBUG)
    (void)scheduler.addTask(printSchedulerRateTask, this, TASK_PRINT_RATE_HZ);
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
    passthroughMode.setModeSwitchPosition(THREE_POS_SW::HIGH_POS);
    rateMode.setModeSwitchPosition(THREE_POS_SW::MID_POS);
    stabilizeMode.setModeSwitchPosition(THREE_POS_SW::LOW_POS);

#if defined(DEFAULT_TO_PASSTHROUGH_MODE)
    currentMode = &passthroughMode;
#elif defined(DEFAULT_TO_RATE_MODE)
    currentMode = &rateMode;
#elif defined(DEFAULT_TO_STABILIZE_MODE)
    currentMode = &stabilizeMode;
#endif
    previousMode = currentMode;
    failSafeActive = false;

    // Initialize systems
    imu.init();
    radio.init();
    actuators.init();
}

void Xpilot::updateFlightMode(void)
{
    bool radioFailSafe = radio.inFailsafe();
    // If system failsafe has been activated and radio is still in fail safe
    if (failSafeActive && radioFailSafe)
        return;

    // First time detecting radio in failsafe
    // Set the current mode to selected failsafe mode
    if (radioFailSafe)
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
    else
    {
        // Both system and radio are not in failsafe
        // Set current flight mode based on the radio mode switch position
        failSafeActive = false; // Reset failsafe active flag
        THREE_POS_SW radioModeSwitchPos = radio.getRxAux1Pos();
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

    // Failsafe detected or mode switch position has changed, perform mode transition
    previousMode->exit();
    currentMode->enter();
    previousMode = currentMode;
}

static void clearTerminal()
{
    Serial.print("\033[2J"); // Clear screen
    Serial.print("\033[H");  // Move cursor to home
}

void Xpilot::printIO(void)
{
    clearTerminal();
    Serial.print("\t\t\t\t");
    Serial.print("Flight Mode: ");
    Serial.println(xpilot.getFlightMode()->modeName4());
    Serial.print("\t\t\t\t");
    Serial.print("Failsafe: ");
    Serial.println(xpilot.inFailsafe() ? "Active" : "Inactive");
    Serial.println();
    Serial.print("Radio Input PWM");
    Serial.print("\t\t\t");
    Serial.print("Mode Input");
    Serial.print("\t\t\t");
    Serial.println("Output PWM");

    Serial.print("Aileron 1: ");
    Serial.print(radio.getRxRollPWM());
    Serial.print("\t\t\t");
    Serial.print("Aileron 1: ");
    Serial.print(currentMode->getRoll());
    Serial.print("\t\t\t");
    Serial.print("Aileron 1: ");
    Serial.println(actuators.getServoOut(Actuators::Channel::CH1));

    Serial.print("Aileron 2: ");
    Serial.print(radio.getRxRollPWM());
    Serial.print("\t\t\t");
    Serial.print("Aileron 2: ");
    Serial.print(currentMode->getRoll());
    Serial.print("\t\t\t");
    Serial.print("Aileron 2: ");
    Serial.println(actuators.getServoOut(Actuators::Channel::CH2));

    Serial.print("Elevator: ");
    Serial.print(radio.getRxPitchPWM());
    Serial.print("\t\t\t");
    Serial.print("Elevator: ");
    Serial.print(currentMode->getPitch());
    Serial.print("\t\t\t");
    Serial.print("Elevator: ");
    Serial.println(actuators.getServoOut(Actuators::Channel::CH3));

    Serial.print("Rudder: ");
    Serial.print(radio.getRxYawPWM());
    Serial.print("\t\t\t");
    Serial.print("Rudder: ");
    Serial.print(currentMode->getYaw());
    Serial.print("\t\t\t");
    Serial.print("Rudder: ");
    Serial.println(actuators.getServoOut(Actuators::Channel::CH4));

#if defined(USE_FLAPERONS)
    Serial.print("Flaperon: ");
    Serial.print(radio.getRxAux2PWM());
    Serial.print("\t\t\t");
    Serial.print("Flaperon: ");
    Serial.print(currentMode->getFlaperon());
    Serial.print("\t\t\t");
    Serial.print("Flap Position: ");
    Serial.println((int16_t)radio.getRxAux2Pos());
#endif

#if defined(USE_AUX3)
    Serial.print("Aux3 PWM: ");
    Serial.print(radio.getRxAux3PWM());
    Serial.print("\t\t\t");
    Serial.print("Aux3 Position: ");
    Serial.println((int16_t)radio.getRxAux3Pos());
#endif
}

void Xpilot::printIMU(void)
{
    clearTerminal();
    Serial.println(">");
    Serial.print("Roll: ");
    Serial.println(static_cast<int16_t>(imu.getRoll()));
    Serial.print("Pitch: ");
    Serial.println(static_cast<int16_t>(imu.getPitch()));
    Serial.print("Yaw: ");
    Serial.println(static_cast<int16_t>(imu.getYaw()));
}

void Xpilot::printSchedulerRate(void)
{
    clearTerminal();
    Serial.println(">");

    Scheduler::TaskStats taskStats;
    if (scheduler.getStats(imuTaskId, taskStats))
    {
        Serial.print("IMU Loop Rate:\t\t\t");
        Serial.print(taskStats.loopRateHz);
        Serial.println();
    }
    if (scheduler.getStats(flightModeRunTaskId, taskStats))
    {
        Serial.print("Mode Run Loop Rate:\t\t");
        Serial.print(taskStats.loopRateHz);
        Serial.println();
    }
    if (scheduler.getStats(flightModeUpdateTaskId, taskStats))
    {
        Serial.print("Mode Update Loop Rate:\t\t");
        Serial.print(taskStats.loopRateHz);
        Serial.println();
    }
    if (scheduler.getStats(radioTaskId, taskStats))
    {
        Serial.print("Radio Loop Rate:\t\t");
        Serial.print(taskStats.loopRateHz);
        Serial.println();
    }
    if (scheduler.getStats(writeServoTaskId, taskStats))
    {
        Serial.print("Write Servo Loop Rate:\t\t");
        Serial.print(taskStats.loopRateHz);
        Serial.println();
    }
}
// ---------------------------