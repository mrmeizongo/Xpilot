#include "Xpilot.h"
#include "IMU.h"

#define CLEAR_TERMINAL()         \
    do                           \
    {                            \
        Serial.print("\033[2J"); \
        Serial.print("\033[H");  \
    } while (0)

void Xpilot::printIO(void)
{
    CLEAR_TERMINAL();
    Serial.print("\t\t\t\t\t\t");
    Serial.print("Flight Mode: ");
    Serial.println(xpilot.getCurrentFlightMode()->modeName4());
    Serial.print("\t\t\t\t\t\t");
    Serial.print("Failsafe: ");
    Serial.println(xpilot.inFailsafe() ? "Active" : "Inactive");
    Serial.println();
    Serial.print("Radio Input PWM");
    Serial.print("\t\t\t");
    Serial.print("Mode Input");
    Serial.print("\t\t\t");
    Serial.print("Mode Output");
    Serial.print("\t\t\t");
    Serial.println("Servo Output PWM");

    Serial.print("Aileron 1: ");
    Serial.print(radio.getPWM(Radio::CHANNELS::ROLL));
    Serial.print("\t\t\t");
    Serial.print("Aileron 1: ");
    Serial.print(currentMode->getRollInput());
    Serial.print("\t\t\t");
    Serial.print("Aileron 1: ");
    Serial.print(currentMode->getRollOutput());
    Serial.print("\t\t\t");
    Serial.print("Aileron 1: ");
    Serial.println(actuators.getServoOut(Actuators::Channel::CH1));

    Serial.print("Aileron 2: ");
    Serial.print(radio.getPWM(Radio::CHANNELS::ROLL));
    Serial.print("\t\t\t");
    Serial.print("Aileron 2: ");
    Serial.print(currentMode->getRollInput());
    Serial.print("\t\t\t");
    Serial.print("Aileron 2: ");
    Serial.print(currentMode->getRollOutput());
    Serial.print("\t\t\t");
    Serial.print("Aileron 2: ");
    Serial.println(actuators.getServoOut(Actuators::Channel::CH2));

    Serial.print("Elevator: ");
    Serial.print(radio.getPWM(Radio::CHANNELS::PITCH));
    Serial.print("\t\t\t");
    Serial.print("Elevator: ");
    Serial.print(currentMode->getPitchInput());
    Serial.print("\t\t\t");
    Serial.print("Elevator: ");
    Serial.print(currentMode->getPitchOutput());
    Serial.print("\t\t\t");
    Serial.print("Elevator: ");
    Serial.println(actuators.getServoOut(Actuators::Channel::CH3));

    Serial.print("Rudder: ");
    Serial.print(radio.getPWM(Radio::CHANNELS::YAW));
    Serial.print("\t\t\t");
    Serial.print("Rudder: ");
    Serial.print(currentMode->getYawInput());
    Serial.print("\t\t\t");
    Serial.print("Rudder: ");
    Serial.print(currentMode->getYawOutput());
    Serial.print("\t\t\t");
    Serial.print("Rudder: ");
    Serial.println(actuators.getServoOut(Actuators::Channel::CH4));

#if defined(USE_FLAPERONS)
    Serial.print("Flaperon: ");
    Serial.print(radio.getPWM(Radio::CHANNELS::AUX2));
    Serial.print("\t\t\t");
    Serial.print("Flaperon: ");
    Serial.print(currentMode->getFlaperon());
    Serial.print("\t\t\t");
    Serial.print("Flaperon Position: ");
    Serial.println((int16_t)radio.getThreeSwitchPos(Radio::CHANNELS::AUX2));
#endif
}

void Xpilot::printIMU(void)
{
    CLEAR_TERMINAL();
    Serial.println(">");
    Serial.print("Roll: ");
    Serial.println(imu.getRoll());
    Serial.print("Pitch: ");
    Serial.println(imu.getPitch());
    Serial.print("Yaw: ");
    Serial.println(imu.getYaw());
}

void Xpilot::printSchedulerRate(void)
{
    CLEAR_TERMINAL();
    Scheduler::TaskStats taskStats;
    if (scheduler.getStats(imuTaskId, taskStats))
    {
        Serial.print("IMU Task Loop Rate:\t\t\t");
        Serial.print(taskStats.loopRateHz);
        Serial.println();
    }
    if (scheduler.getStats(flightModeRunTaskId, taskStats))
    {
        Serial.print("Mode Run Task Loop Rate:\t\t");
        Serial.print(taskStats.loopRateHz);
        Serial.println();
    }
    if (scheduler.getStats(flightModeInputUpdateTaskId, taskStats))
    {
        Serial.print("Mode Input Update Task Loop Rate:\t");
        Serial.print(taskStats.loopRateHz);
        Serial.println();
    }
    if (scheduler.getStats(flightModeUpdateTaskId, taskStats))
    {
        Serial.print("Mode Update Task Loop Rate:\t\t");
        Serial.print(taskStats.loopRateHz);
        Serial.println();
    }
    if (scheduler.getStats(radioTaskId, taskStats))
    {
        Serial.print("Radio Task Loop Rate:\t\t\t");
        Serial.print(taskStats.loopRateHz);
        Serial.println();
    }
    if (scheduler.getStats(actuatorTaskId, taskStats))
    {
        Serial.print("Actuator Task Loop Rate:\t\t");
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
        Serial.println("IMU Task Stats");
        Serial.print("Run count: ");
        Serial.println(taskStats.runCount);
        Serial.print("Missed periods: ");
        Serial.println(taskStats.missedPeriods);
        Serial.print("Overrun count: ");
        Serial.println(taskStats.overrunCount);
        Serial.print("Last run time(us): ");
        Serial.println(taskStats.lastRuntimeUs);
        Serial.print("Max run time(us) : ");
        Serial.println(taskStats.maxRuntimeUs);
        Serial.print("Last loop rate update(us): ");
        Serial.println(taskStats.lastLoopRateUpdateUs);
        Serial.print("Loop rate(hz): ");
        Serial.println(taskStats.loopRateHz);
        Serial.print("Loop counter: ");
        Serial.println(taskStats.loopCounter);
    }
}

void Xpilot::printRadioTaskStats(void)
{
    CLEAR_TERMINAL();
    Scheduler::TaskStats taskStats;
    if (scheduler.getStats(radioTaskId, taskStats))
    {
        Serial.println("Radio Task Stats");
        Serial.print("Run count: ");
        Serial.println(taskStats.runCount);
        Serial.print("Missed periods: ");
        Serial.println(taskStats.missedPeriods);
        Serial.print("Overrun count: ");
        Serial.println(taskStats.overrunCount);
        Serial.print("Last run time(us): ");
        Serial.println(taskStats.lastRuntimeUs);
        Serial.print("Max run time(us) : ");
        Serial.println(taskStats.maxRuntimeUs);
        Serial.print("Last loop rate update(us): ");
        Serial.println(taskStats.lastLoopRateUpdateUs);
        Serial.print("Loop rate(hz): ");
        Serial.println(taskStats.loopRateHz);
        Serial.print("Loop counter: ");
        Serial.println(taskStats.loopCounter);
    }
}

void Xpilot::printFlightModeRunTaskStats(void)
{
    CLEAR_TERMINAL();
    Scheduler::TaskStats taskStats;
    if (scheduler.getStats(flightModeRunTaskId, taskStats))
    {
        Serial.println("FM Run Task Stats");
        Serial.print("Run count: ");
        Serial.println(taskStats.runCount);
        Serial.print("Missed periods: ");
        Serial.println(taskStats.missedPeriods);
        Serial.print("Overrun count: ");
        Serial.println(taskStats.overrunCount);
        Serial.print("Last run time(us): ");
        Serial.println(taskStats.lastRuntimeUs);
        Serial.print("Max run time(us) : ");
        Serial.println(taskStats.maxRuntimeUs);
        Serial.print("Last loop rate update(us): ");
        Serial.println(taskStats.lastLoopRateUpdateUs);
        Serial.print("Loop rate(hz): ");
        Serial.println(taskStats.loopRateHz);
        Serial.print("Loop counter: ");
        Serial.println(taskStats.loopCounter);
    }
}

void Xpilot::printFlightModeUpdateTaskStats(void)
{
    CLEAR_TERMINAL();
    Scheduler::TaskStats taskStats;
    if (scheduler.getStats(flightModeUpdateTaskId, taskStats))
    {
        Serial.println("FM Update Task Stats");
        Serial.print("Run count: ");
        Serial.println(taskStats.runCount);
        Serial.print("Missed periods: ");
        Serial.println(taskStats.missedPeriods);
        Serial.print("Overrun count: ");
        Serial.println(taskStats.overrunCount);
        Serial.print("Last run time(us): ");
        Serial.println(taskStats.lastRuntimeUs);
        Serial.print("Max run time(us) : ");
        Serial.println(taskStats.maxRuntimeUs);
        Serial.print("Last loop rate update(us): ");
        Serial.println(taskStats.lastLoopRateUpdateUs);
        Serial.print("Loop rate(hz): ");
        Serial.println(taskStats.loopRateHz);
        Serial.print("Loop counter: ");
        Serial.println(taskStats.loopCounter);
    }
}

void Xpilot::printFlightModeInputUpdateTaskStats(void)
{
    CLEAR_TERMINAL();
    Scheduler::TaskStats taskStats;
    if (scheduler.getStats(flightModeInputUpdateTaskId, taskStats))
    {
        Serial.println("FM Input Update Task Stats");
        Serial.print("Run count: ");
        Serial.println(taskStats.runCount);
        Serial.print("Missed periods: ");
        Serial.println(taskStats.missedPeriods);
        Serial.print("Overrun count: ");
        Serial.println(taskStats.overrunCount);
        Serial.print("Last run time(us): ");
        Serial.println(taskStats.lastRuntimeUs);
        Serial.print("Max run time(us) : ");
        Serial.println(taskStats.maxRuntimeUs);
        Serial.print("Last loop rate update(us): ");
        Serial.println(taskStats.lastLoopRateUpdateUs);
        Serial.print("Loop rate(hz): ");
        Serial.println(taskStats.loopRateHz);
        Serial.print("Loop counter: ");
        Serial.println(taskStats.loopCounter);
    }
}

void Xpilot::printActuatorTaskStats(void)
{
    CLEAR_TERMINAL();
    Scheduler::TaskStats taskStats;
    if (scheduler.getStats(actuatorTaskId, taskStats))
    {
        Serial.println("Actuator Task Stats");
        Serial.print("Run count: ");
        Serial.println(taskStats.runCount);
        Serial.print("Missed periods: ");
        Serial.println(taskStats.missedPeriods);
        Serial.print("Overrun count: ");
        Serial.println(taskStats.overrunCount);
        Serial.print("Last run time(us): ");
        Serial.println(taskStats.lastRuntimeUs);
        Serial.print("Max run time(us) : ");
        Serial.println(taskStats.maxRuntimeUs);
        Serial.print("Last loop rate update(us): ");
        Serial.println(taskStats.lastLoopRateUpdateUs);
        Serial.print("Loop rate(hz): ");
        Serial.println(taskStats.loopRateHz);
        Serial.print("Loop counter: ");
        Serial.println(taskStats.loopCounter);
    }
}