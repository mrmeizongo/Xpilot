#include <Arduino.h>
#include <PlaneConfig.h>
#include "Xpilot.h"
#include "Radio.h"
#include "Actuators.h"
#include "IMU.h"

#define INPUT_REFRESH_RATE_US 20000U
#define ONEHZ_LOOP_US 1000000U

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
}

void Xpilot::setup(void)
{
#if defined(IO_DEBUG) || defined(LOOP_DEBUG) || defined(IMU_DEBUG) || defined(CALIBRATE_DEBUG) || defined(SELF_TEST_ACCEL_GYRO)
    Serial.begin(115200);
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
    imu.processIMU();
    // Process radio input
    if (nowUs - inputLastUs >= INPUT_REFRESH_RATE_US)
    {
        radio.processInput();
        inputLastUs = nowUs;
    }
    updateFlightMode();
    currentMode->run();
    currentMode->setServoOut();
    actuators.writeServos();

#if defined(IO_DEBUG) || defined(LOOP_DEBUG) || defined(IMU_DEBUG) || defined(CALIBRATE_DEBUG)
    printDebug();
#endif
}

// Update flight mode based on radio mode switch position
void Xpilot::updateFlightMode(void)
{
    Control::MODEPOS modePos = radio.getRxModePos();
    if (modePos == currentMode->modePos())
        return;

    // Process new mode if mode switch position has changed
    if (modePos == passthroughMode.modePos())
        currentMode = &passthroughMode;
    else if (modePos == stabilizeMode.modePos())
        currentMode = &stabilizeMode;
    else if (modePos == rateMode.modePos())
        currentMode = &rateMode;

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
    Serial.println(xpilot.getFlightModeName());
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