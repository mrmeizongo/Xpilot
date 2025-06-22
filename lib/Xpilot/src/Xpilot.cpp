#include <Arduino.h>
#include <PlaneConfig.h>
#include <SystemConfig.h>
#include "Xpilot.h"
#include "Radio.h"
#include "Actuators.h"
#include "IMU.h"

// Timer variables
static unsigned long nowUs, inputLastUs = 0;
// -------------------------

// Debug helper functions
static void printDebug(void) __attribute__((unused));
static void printIO(void) __attribute__((unused));
static void printIMU(void) __attribute__((unused));
static void printLoopRate(void) __attribute__((unused));
// -------------------------

Xpilot::Xpilot(void)
{
    // Specify the mode switch position for each mode
    passthroughMode.setModeSwitchPosition(THREE_POS_SW::HIGH_POS);
    rateMode.setModeSwitchPosition(THREE_POS_SW::MID_POS);
    stabilizeMode.setModeSwitchPosition(THREE_POS_SW::LOW_POS);
}

void Xpilot::setup(void)
{
#if defined(IO_DEBUG) || defined(LOOP_DEBUG) || defined(IMU_DEBUG) || defined(CALIBRATE_DEBUG) || defined(SELF_TEST_ACCEL_GYRO)
    Serial.begin(BAUD_RATE);
    while (!Serial)
        ; // Wait for Serial port to open
#endif

    imu.init();       // Initialize IMU
    radio.init();     // Initialize radio
    actuators.init(); // Initialize servos
}

/*
    Main Xpilot execution loop
    Read imu data, read input and process output to servos
*/
void Xpilot::loop(void)
{
    nowUs = micros();
    // Process radio input
    if (nowUs - inputLastUs >= INPUT_REFRESH_RATE_US)
    {
        radio.processInput();
        inputLastUs = nowUs;
    }

    updateFlightMode();      // Update flight mode based on radio mode switch position
    imu.processIMU();        // Grab new sensor data if available
    currentMode->run();      // Run the high level processing for the current flight mode
    actuators.writeServos(); // Write servo outputs to the actuators object

#if defined(IO_DEBUG) || defined(LOOP_DEBUG) || defined(IMU_DEBUG) || defined(CALIBRATE_DEBUG)
    printDebug();
#endif
}

void Xpilot::updateFlightMode(void)
{
    bool radioFailSafe = radio.inFailsafe();
    // If failsafe is active and radio is still in fail safe mode, simply return
    if (failSafeActive && radioFailSafe)
        return;

    // Switch to selected failsafe mode if radio is in failsafe
    // Otherwise switch to the mode corresponding to the new mode switch position
    if (radioFailSafe)
    {
#if defined(FAIL_SAFE_TO_STABILIZE)
        currentMode = &stabilizeMode;
#elif defined(FAIL_SAFE_TO_RATE)
        currentMode = &rateMode;
#elif defined(FAIL_SAFE_TO_PASSTHROUGH)
        currentMode = &passthroughMode;
#endif
        failSafeActive = true; // Set failsafe active flag
    }
    else
    {
        THREE_POS_SW auxPos = radio.getRxAuxPos();
        // Radio mode switch position has not changed, simply return
        if (auxPos == currentMode->getModeSwitchPosition())
            return;

        // Process new mode if mode switch position has changed
        if (auxPos == passthroughMode.getModeSwitchPosition())
            currentMode = &passthroughMode;
        else if (auxPos == stabilizeMode.getModeSwitchPosition())
            currentMode = &stabilizeMode;
        else if (auxPos == rateMode.getModeSwitchPosition())
            currentMode = &rateMode;

        failSafeActive = false; // Reset failsafe active flag
    }

    previousMode->exit();
    currentMode->enter();
    previousMode = currentMode;
}

// Debug helper functions
static void printDebug(void)
{
#if defined(IO_DEBUG)
    static unsigned long debugLastUs = 0;
    if (nowUs - debugLastUs >= ONEHZ_LOOP_US)
    {
        printIO();
        debugLastUs = nowUs;
    }
#endif

#if defined(IMU_DEBUG) || defined(CALIBRATE_DEBUG)
    printIMU();
#endif
#if defined(LOOP_DEBUG)
    printLoopRate();
#endif
}

static void printIO(void)
{
    Serial.print("\t\t");
    Serial.print("Flight Mode: ");
    Serial.println(xpilot.getFlightMode()->modeName4());
    Serial.print("Input PWM");
    Serial.print("\t\t\t\t");
    Serial.println("Output PWM");

    Serial.print("Aileron 1 PWM: ");
    Serial.print(radio.getRxRollPWM());
    Serial.print("\t\t\t");
    Serial.print("Aileron 1 PWM: ");
    Serial.println(actuators.getServoOut(Actuators::Channel::AILERON1));

    Serial.print("Aileron 2 PWM: ");
    Serial.print(radio.getRxRollPWM());
    Serial.print("\t\t\t");
    Serial.print("Aileron 2 PWM: ");
    Serial.println(actuators.getServoOut(Actuators::Channel::AILERON2));

    Serial.print("Elevator PWM: ");
    Serial.print(radio.getRxPitchPWM());
    Serial.print("\t\t\t");
    Serial.print("Elevator PWM: ");
    Serial.println(actuators.getServoOut(Actuators::Channel::ELEVATOR));

    Serial.print("Rudder PWM: ");
    Serial.print(radio.getRxYawPWM());
    Serial.print("\t\t\t");
    Serial.print("Rudder PWM: ");
    Serial.println(actuators.getServoOut(Actuators::Channel::RUDDER));
    Serial.println();
}

static void printIMU(void)
{
    Serial.print(">");
    Serial.print("Roll: ");
    Serial.print(imu.getRoll());
    Serial.print(", ");
    Serial.print("Pitch: ");
    Serial.print(imu.getPitch());
    Serial.print(", ");
    Serial.print("Yaw: ");
    Serial.print(imu.getYaw());
    Serial.println();
}

static void printLoopRate(void)
{
    unsigned long loopUs = micros() - nowUs;
    unsigned long loopMs = loopUs / 1000;
    loopMs = loopMs <= 0 ? 1 : loopMs;
    Serial.print(">");
    Serial.print("Loop time: ");
    Serial.print(loopMs);
    Serial.print("ms");
    Serial.print(", ");
    Serial.print("Loop rate: ");
    Serial.print(1000 / loopMs);
    Serial.print("Hz");
    Serial.println();
}

Xpilot xpilot;
// ---------------------------