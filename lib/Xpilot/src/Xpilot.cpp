#include <Arduino.h>
#include <PlaneConfig.h>
#include <SystemConfig.h>
#include "Xpilot.h"
#include "Radio.h"
#include "Actuators.h"
#include "IMU.h"

// Timer variables
static unsigned long nowUs = 0, inputLastUs = 0, imuLastUs = 0;
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
#if defined(IO_DEBUG) || defined(LOOP_DEBUG) || defined(IMU_DEBUG) || defined(CALIBRATE_DEBUG) || defined(SELF_TEST_ACCEL_GYRO) || defined(READ_CALIBRATION_FROM_EEPROM)
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
    previousMode = currentMode; // Set previous mode to current mode
    failSafeActive = false;

    imu.init();       // Initialize IMU
    radio.init();     // Initialize radio
    actuators.init(); // Initialize servos
}

// Main Xpilot execution loop
void Xpilot::loop(void)
{
    nowUs = micros();
    // Process radio input
    if (nowUs - inputLastUs >= INPUT_REFRESH_RATE_US)
    {
        radio.processInput();
        inputLastUs = nowUs;
    }

    // Update IMU readings and AHRS values
    if (nowUs - imuLastUs >= IMU_REFRESH_RATE_US)
    {
        imu.getLatestReadings();
        imuLastUs = nowUs;
    }

    updateFlightMode(); // Update flight mode based on radio mode switch position
    currentMode->run(); // Run the high level processing for the current flight mode

#if defined(IO_DEBUG) || defined(LOOP_DEBUG) || defined(IMU_DEBUG) || defined(CALIBRATE_DEBUG)
    printDebug();
#endif
}

void Xpilot::updateFlightMode(void)
{
    bool radioFailSafe = radio.inFailsafe();
    // If system failsafe has been activated and radio is still in fail safe, simply return
    if (failSafeActive && radioFailSafe)
        return;

    // First time detecting radio in failsafe, activate system failsafe
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
        // Radio mode switch position has not changed, simply return
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
    Serial.print("\t\t");
    Serial.print("Failsafe: ");
    Serial.println(xpilot.inFailsafe() ? "Active" : "Inactive");
    Serial.print("Input PWM");
    Serial.print("\t\t\t\t");
    Serial.println("Output PWM");

    Serial.print("Aileron 1 PWM: ");
    Serial.print(radio.getRxRollPWM());
    Serial.print("\t\t\t");
    Serial.print("Aileron 1 PWM: ");
    Serial.println(actuators.getServoOut(Actuators::Channel::CH1));

    Serial.print("Aileron 2 PWM: ");
    Serial.print(radio.getRxRollPWM());
    Serial.print("\t\t\t");
    Serial.print("Aileron 2 PWM: ");
    Serial.println(actuators.getServoOut(Actuators::Channel::CH2));

    Serial.print("Elevator PWM: ");
    Serial.print(radio.getRxPitchPWM());
    Serial.print("\t\t\t");
    Serial.print("Elevator PWM: ");
    Serial.println(actuators.getServoOut(Actuators::Channel::CH3));

    Serial.print("Rudder PWM: ");
    Serial.print(radio.getRxYawPWM());
    Serial.print("\t\t\t");
    Serial.print("Rudder PWM: ");
    Serial.println(actuators.getServoOut(Actuators::Channel::CH4));

#if defined(USE_FLAPERONS)
    Serial.print("Flaperon PWM: ");
    Serial.print(radio.getRxAux2PWM());
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
    loopUs = loopUs <= 0 ? 1 : loopUs;
    Serial.print(">");
    Serial.print("Loop time: ");
    Serial.print(loopUs);
    Serial.print("us");
    Serial.print(", ");
    Serial.print("Loop rate: ");
    Serial.print(1000000 / loopUs);
    Serial.print("Hz");
    Serial.println();
}
// ---------------------------