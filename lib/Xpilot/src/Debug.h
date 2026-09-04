#include "Xpilot.h"
#include "IMU.h"

#define CLEAR_TERMINAL()                                                                                               \
    do                                                                                                                 \
    {                                                                                                                  \
        Serial.print("\033[2J");                                                                                       \
        Serial.print("\033[H");                                                                                        \
    } while (0)

void Xpilot::printIO(void)
{
    CLEAR_TERMINAL();
    Serial.print(F("\t\t\t\t\t\t"));
    Serial.print(F("Flight Mode: "));
    Serial.println(xpilot.getCurrentFlightMode()->modeName4());
    Serial.print(F("\t\t\t\t\t\t"));
    Serial.print(F("Failsafe: "));
    Serial.println(xpilot.inFailsafe() ? F("Active") : F("Inactive"));
    Serial.println();
    Serial.print(F("Radio Input PWM"));
    Serial.print(F("\t\t\t"));
    Serial.print(F("Mode Input"));
    Serial.print(F("\t\t\t"));
    Serial.print(F("Mode Output"));
    Serial.print(F("\t\t\t"));
    Serial.println(F("Servo Output PWM"));

    Serial.print(F("Aileron 1: "));
    Serial.print(radio.getPWM(Radio::CHANNEL::ROLL));
    Serial.print(F("\t\t\t"));
    Serial.print(F("Aileron 1: "));
    Serial.print(currentMode->getRollInput());
    Serial.print(F("\t\t\t"));
    Serial.print(F("Aileron 1: "));
    Serial.print(currentMode->getRollOutput());
    Serial.print(F("\t\t\t"));
    Serial.print(F("Aileron 1: "));
    Serial.println(actuators.getServoOut(Actuators::Channel::CH1));

    Serial.print(F("Aileron 2: "));
    Serial.print(radio.getPWM(Radio::CHANNEL::ROLL));
    Serial.print(F("\t\t\t"));
    Serial.print(F("Aileron 2: "));
    Serial.print(currentMode->getRollInput());
    Serial.print(F("\t\t\t"));
    Serial.print(F("Aileron 2: "));
    Serial.print(currentMode->getRollOutput());
    Serial.print(F("\t\t\t"));
    Serial.print(F("Aileron 2: "));
    Serial.println(actuators.getServoOut(Actuators::Channel::CH2));

    Serial.print(F("Elevator: "));
    Serial.print(radio.getPWM(Radio::CHANNEL::PITCH));
    Serial.print(F("\t\t\t"));
    Serial.print(F("Elevator: "));
    Serial.print(currentMode->getPitchInput());
    Serial.print(F("\t\t\t"));
    Serial.print(F("Elevator: "));
    Serial.print(currentMode->getPitchOutput());
    Serial.print(F("\t\t\t"));
    Serial.print(F("Elevator: "));
    Serial.println(actuators.getServoOut(Actuators::Channel::CH3));

    Serial.print(F("Rudder: "));
    Serial.print(radio.getPWM(Radio::CHANNEL::YAW));
    Serial.print(F("\t\t\t"));
    Serial.print(F("Rudder: "));
    Serial.print(currentMode->getYawInput());
    Serial.print(F("\t\t\t"));
    Serial.print(F("Rudder: "));
    Serial.print(currentMode->getYawOutput());
    Serial.print(F("\t\t\t"));
    Serial.print(F("Rudder: "));
    Serial.println(actuators.getServoOut(Actuators::Channel::CH4));

#if defined(USE_FLAPERONS)
    Serial.print(F("Flaperon: "));
    Serial.print(radio.getPWM(Radio::CHANNEL::AUX2));
    Serial.print(F("\t\t\t"));
    Serial.print(F("Flaperon: "));
    Serial.print(currentMode->getFlaperon());
    Serial.print(F("\t\t\t"));
    Serial.print(F("Flaperon Position: "));
    Serial.println((int16_t)radio.getThreeSwitchPos(Radio::CHANNEL::AUX2));
#endif
}

void Xpilot::printIMU(void)
{
    CLEAR_TERMINAL();
    Serial.print(F("Roll: "));
    Serial.println(imu.getRoll());
    Serial.print(F("Pitch: "));
    Serial.println(imu.getPitch());
    Serial.print(F("Yaw: "));
    Serial.println(imu.getYaw());
}

void Xpilot::printSchedulerRate(void)
{
    CLEAR_TERMINAL();
    Scheduler::TaskStats taskStats;
    if (scheduler.getStats(imuTaskId, taskStats))
    {
        Serial.print(F("IMU Task Loop Rate:\t\t\t"));
        Serial.print(taskStats.loopRateHz);
        Serial.println();
    }
    if (scheduler.getStats(flightModeRunTaskId, taskStats))
    {
        Serial.print(F("Mode Run Task Loop Rate:\t\t"));
        Serial.print(taskStats.loopRateHz);
        Serial.println();
    }
    if (scheduler.getStats(flightModeInputUpdateTaskId, taskStats))
    {
        Serial.print(F("Mode Input Update Task Loop Rate:\t"));
        Serial.print(taskStats.loopRateHz);
        Serial.println();
    }
    if (scheduler.getStats(flightModeUpdateTaskId, taskStats))
    {
        Serial.print(F("Mode Update Task Loop Rate:\t\t"));
        Serial.print(taskStats.loopRateHz);
        Serial.println();
    }
    if (scheduler.getStats(radioTaskId, taskStats))
    {
        Serial.print(F("Radio Task Loop Rate:\t\t\t"));
        Serial.print(taskStats.loopRateHz);
        Serial.println();
    }
    if (scheduler.getStats(actuatorTaskId, taskStats))
    {
        Serial.print(F("Actuator Task Loop Rate:\t\t"));
        Serial.print(taskStats.loopRateHz);
        Serial.println();
    }
}

void Xpilot::printIMUTaskStats(void)
{
    CLEAR_TERMINAL();
    Scheduler::TaskStats taskStats;
    if (scheduler.getStats(imuTaskId, taskStats))
    {
        Serial.println(F("IMU Task Stats"));
        Serial.print(F("Run count: "));
        Serial.println(taskStats.runCount);
        Serial.print(F("Missed periods: "));
        Serial.println(taskStats.missedPeriods);
        Serial.print(F("Overrun count: "));
        Serial.println(taskStats.overrunCount);
        Serial.print(F("Last run time(us): "));
        Serial.println(taskStats.lastRuntimeUs);
        Serial.print(F("Max run time(us) : "));
        Serial.println(taskStats.maxRuntimeUs);
        Serial.print(F("Last loop rate update(us): "));
        Serial.println(taskStats.lastLoopRateUpdateUs);
        Serial.print(F("Loop rate(hz): "));
        Serial.println(taskStats.loopRateHz);
        Serial.print(F("Loop counter: "));
        Serial.println(taskStats.loopCounter);
    }
}

void Xpilot::printRadioTaskStats(void)
{
    CLEAR_TERMINAL();
    Scheduler::TaskStats taskStats;
    if (scheduler.getStats(radioTaskId, taskStats))
    {
        Serial.println(F("Radio Task Stats"));
        Serial.print(F("Run count: "));
        Serial.println(taskStats.runCount);
        Serial.print(F("Missed periods: "));
        Serial.println(taskStats.missedPeriods);
        Serial.print(F("Overrun count: "));
        Serial.println(taskStats.overrunCount);
        Serial.print(F("Last run time(us): "));
        Serial.println(taskStats.lastRuntimeUs);
        Serial.print(F("Max run time(us) : "));
        Serial.println(taskStats.maxRuntimeUs);
        Serial.print(F("Last loop rate update(us): "));
        Serial.println(taskStats.lastLoopRateUpdateUs);
        Serial.print(F("Loop rate(hz): "));
        Serial.println(taskStats.loopRateHz);
        Serial.print(F("Loop counter: "));
        Serial.println(taskStats.loopCounter);
    }
}

void Xpilot::printFlightModeRunTaskStats(void)
{
    CLEAR_TERMINAL();
    Scheduler::TaskStats taskStats;
    if (scheduler.getStats(flightModeRunTaskId, taskStats))
    {
        Serial.println(F("FM Run Task Stats"));
        Serial.print(F("Run count: "));
        Serial.println(taskStats.runCount);
        Serial.print(F("Missed periods: "));
        Serial.println(taskStats.missedPeriods);
        Serial.print(F("Overrun count: "));
        Serial.println(taskStats.overrunCount);
        Serial.print(F("Last run time(us): "));
        Serial.println(taskStats.lastRuntimeUs);
        Serial.print(F("Max run time(us) : "));
        Serial.println(taskStats.maxRuntimeUs);
        Serial.print(F("Last loop rate update(us): "));
        Serial.println(taskStats.lastLoopRateUpdateUs);
        Serial.print(F("Loop rate(hz): "));
        Serial.println(taskStats.loopRateHz);
        Serial.print(F("Loop counter: "));
        Serial.println(taskStats.loopCounter);
    }
}

void Xpilot::printFlightModeUpdateTaskStats(void)
{
    CLEAR_TERMINAL();
    Scheduler::TaskStats taskStats;
    if (scheduler.getStats(flightModeUpdateTaskId, taskStats))
    {
        Serial.println(F("FM Update Task Stats"));
        Serial.print(F("Run count: "));
        Serial.println(taskStats.runCount);
        Serial.print(F("Missed periods: "));
        Serial.println(taskStats.missedPeriods);
        Serial.print(F("Overrun count: "));
        Serial.println(taskStats.overrunCount);
        Serial.print(F("Last run time(us): "));
        Serial.println(taskStats.lastRuntimeUs);
        Serial.print(F("Max run time(us) : "));
        Serial.println(taskStats.maxRuntimeUs);
        Serial.print(F("Last loop rate update(us): "));
        Serial.println(taskStats.lastLoopRateUpdateUs);
        Serial.print(F("Loop rate(hz): "));
        Serial.println(taskStats.loopRateHz);
        Serial.print(F("Loop counter: "));
        Serial.println(taskStats.loopCounter);
    }
}

void Xpilot::printFlightModeInputUpdateTaskStats(void)
{
    CLEAR_TERMINAL();
    Scheduler::TaskStats taskStats;
    if (scheduler.getStats(flightModeInputUpdateTaskId, taskStats))
    {
        Serial.println(F("FM Input Update Task Stats"));
        Serial.print(F("Run count: "));
        Serial.println(taskStats.runCount);
        Serial.print(F("Missed periods: "));
        Serial.println(taskStats.missedPeriods);
        Serial.print(F("Overrun count: "));
        Serial.println(taskStats.overrunCount);
        Serial.print(F("Last run time(us): "));
        Serial.println(taskStats.lastRuntimeUs);
        Serial.print(F("Max run time(us) : "));
        Serial.println(taskStats.maxRuntimeUs);
        Serial.print(F("Last loop rate update(us): "));
        Serial.println(taskStats.lastLoopRateUpdateUs);
        Serial.print(F("Loop rate(hz): "));
        Serial.println(taskStats.loopRateHz);
        Serial.print(F("Loop counter: "));
        Serial.println(taskStats.loopCounter);
    }
}

void Xpilot::printActuatorTaskStats(void)
{
    CLEAR_TERMINAL();
    Scheduler::TaskStats taskStats;
    if (scheduler.getStats(actuatorTaskId, taskStats))
    {
        Serial.println(F("Actuator Task Stats"));
        Serial.print(F("Run count: "));
        Serial.println(taskStats.runCount);
        Serial.print(F("Missed periods: "));
        Serial.println(taskStats.missedPeriods);
        Serial.print(F("Overrun count: "));
        Serial.println(taskStats.overrunCount);
        Serial.print(F("Last run time(us): "));
        Serial.println(taskStats.lastRuntimeUs);
        Serial.print(F("Max run time(us) : "));
        Serial.println(taskStats.maxRuntimeUs);
        Serial.print(F("Last loop rate update(us): "));
        Serial.println(taskStats.lastLoopRateUpdateUs);
        Serial.print(F("Loop rate(hz): "));
        Serial.println(taskStats.loopRateHz);
        Serial.print(F("Loop counter: "));
        Serial.println(taskStats.loopCounter);
    }
}