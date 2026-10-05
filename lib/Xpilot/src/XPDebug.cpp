#include "XPDebug.h"
#include "IMU.h"
#include "Scheduler.h"
#include "SysConfig.h"

#define CLEAR_TERMINAL()                                                                                                    \
    do                                                                                                                      \
    {                                                                                                                       \
        Serial.print("\033[2J");                                                                                            \
        Serial.print("\033[H");                                                                                             \
    } while (0)

void XPDebug::init(void)
{
#if defined(PRINT_SCHEDULER_RATE)
    (void)scheduler.addTask(&XPDebug::printSchedulerRateTask, this, TASK_PRINT_HZ);
#endif
#if defined(PRINT_IO)
    (void)scheduler.addTask(&XPDebug::printIOTask, this, TASK_PRINT_HZ);
#endif
#if defined(PRINT_IMU)
    (void)scheduler.addTask(&IMU::printIMUTask, &imu, TASK_PRINT_HZ);
#endif
#if defined(PRINT_IMU_TASK_STAT)
    (void)scheduler.addTask(&XPDebug::printIMUTaskStatTask, this, TASK_PRINT_HZ);
#endif
#if defined(PRINT_RADIO_TASK_STAT)
    (void)scheduler.addTask(&XPDebug::printRadioTaskStatTask, this, TASK_PRINT_HZ);
#endif
#if defined(PRINT_STATE_UPDATE_TASK_STAT)
    (void)scheduler.addTask(&XPDebug::printStateUpdateTaskStatTask, this, TASK_PRINT_HZ);
#endif
#if defined(PRINT_FM_UPDATE_TASK_STAT)
    (void)scheduler.addTask(&XPDebug::printFMUpdateTaskStatTask, this, TASK_PRINT_HZ);
#endif
#if defined(PRINT_FM_RUN_TASK_STAT)
    (void)scheduler.addTask(&XPDebug::printFMRunTaskStatTask, this, TASK_PRINT_HZ);
#endif
#if defined(PRINT_FM_OUTPUT_TASK_STAT)
    (void)scheduler.addTask(&XPDebug::printFMOutputTaskStatTask, this, TASK_PRINT_HZ);
#endif
#if defined(PRINT_LED_NOTIFIER_TASK_STAT)
    (void)scheduler.addTask(&XPDebug::printLEDNotifierTaskStatTask, this, TASK_PRINT_HZ);
#endif
}

void XPDebug::printSchedulerRate(void)
{
    CLEAR_TERMINAL();
    Scheduler::TaskStats taskStats;
    if (scheduler.getStats(xpilot.imuTaskId, taskStats))
    {
        Serial.print(F("IMU Task Loop Rate:\t\t\t"));
        Serial.print(taskStats.loopRateHz);
        Serial.println();
    }
    if (scheduler.getStats(xpilot.radioTaskId, taskStats))
    {
        Serial.print(F("Radio Task Loop Rate:\t\t\t"));
        Serial.print(taskStats.loopRateHz);
        Serial.println();
    }
    if (scheduler.getStats(xpilot.stateUpdateTaskId, taskStats))
    {
        Serial.print(F("State Update Task Loop Rate:\t\t"));
        Serial.print(taskStats.loopRateHz);
        Serial.println();
    }
    if (scheduler.getStats(xpilot.flightModeUpdateTaskId, taskStats))
    {
        Serial.print(F("Mode Update Task Loop Rate:\t\t"));
        Serial.print(taskStats.loopRateHz);
        Serial.println();
    }
    if (scheduler.getStats(xpilot.flightModeRunTaskId, taskStats))
    {
        Serial.print(F("Mode Run Task Loop Rate:\t\t"));
        Serial.print(taskStats.loopRateHz);
        Serial.println();
    }
    if (scheduler.getStats(xpilot.flightModeOutputTaskId, taskStats))
    {
        Serial.print(F("FM Output Task Loop Rate:\t\t"));
        Serial.print(taskStats.loopRateHz);
        Serial.println();
    }
    if (scheduler.getStats(xpilot.ledNotifierTaskId, taskStats))
    {
        Serial.print(F("LED Notifier Task Loop Rate:\t\t"));
        Serial.print(taskStats.loopRateHz);
        Serial.println();
    }
}

void XPDebug::printIO(void)
{
    CLEAR_TERMINAL();
    Serial.print(F("\t\t\t\t\t\t"));
    Serial.print(F("Flight Mode: "));
    Serial.println(xpilot.getCurrentFlightMode()->modeName4());
    Serial.print(F("\t\t"));
    Serial.print(F("Armed: "));
    Serial.print(xpilot.isArmed() ? F("Yes") : F("No"));
    Serial.print(F("\t\t"));
    Serial.print(F("Failsafe: "));
    Serial.print(xpilot.inFailsafe() ? F("Active") : F("Inactive"));
    Serial.print(F("\t\t"));
    Serial.print(F("Throttle cut: "));
    Serial.println(radio.inThrottleCut() ? F("Active") : F("Inactive"));
    Serial.println();
    Serial.print(F("Radio Input PWM"));
    Serial.print(F("\t\t\t"));
    Serial.print(F("Mode Input"));
    Serial.print(F("\t\t\t"));
    Serial.print(F("Mode Output"));
    Serial.print(F("\t\t\t"));
    Serial.println(F("Servo Output PWM"));

    Serial.print(F("Throttle: "));
    Serial.print(radio.getPWM(Radio::CHANNEL::THROTTLE));
    Serial.print(F("\t\t\t"));
    Serial.print(F("Throttle: "));
    Serial.print(xpilot.currentMode->getThrottleInput());
    Serial.print(F("\t\t\t"));
    Serial.print(F("Throttle: "));
    Serial.print(xpilot.currentMode->getThrottleOutput());
    Serial.print(F("\t\t\t"));
    Serial.print(F("Throttle: "));
    Serial.println(actuators.getServoOut(Actuators::CHANNEL::CH1));

    Serial.print(F("Aileron 1: "));
    Serial.print(radio.getPWM(Radio::CHANNEL::ROLL));
    Serial.print(F("\t\t\t"));
    Serial.print(F("Aileron 1: "));
    Serial.print(xpilot.currentMode->getRollInput());
    Serial.print(F("\t\t\t"));
    Serial.print(F("Aileron 1: "));
    Serial.print(xpilot.currentMode->getLeftRollOutput());
    Serial.print(F("\t\t\t"));
    Serial.print(F("Aileron 1: "));
    Serial.println(actuators.getServoOut(Actuators::CHANNEL::CH2));

    Serial.print(F("Aileron 2: "));
    Serial.print(radio.getPWM(Radio::CHANNEL::ROLL));
    Serial.print(F("\t\t\t"));
    Serial.print(F("Aileron 2: "));
    Serial.print(xpilot.currentMode->getRollInput());
    Serial.print(F("\t\t\t"));
    Serial.print(F("Aileron 2: "));
    Serial.print(xpilot.currentMode->getRightRollOutput());
    Serial.print(F("\t\t\t"));
    Serial.print(F("Aileron 2: "));
    Serial.println(actuators.getServoOut(Actuators::CHANNEL::CH3));

    Serial.print(F("Elevator: "));
    Serial.print(radio.getPWM(Radio::CHANNEL::PITCH));
    Serial.print(F("\t\t\t"));
    Serial.print(F("Elevator: "));
    Serial.print(xpilot.currentMode->getPitchInput());
    Serial.print(F("\t\t\t"));
    Serial.print(F("Elevator: "));
    Serial.print(xpilot.currentMode->getPitchOutput());
    Serial.print(F("\t\t\t"));
    Serial.print(F("Elevator: "));
    Serial.println(actuators.getServoOut(Actuators::CHANNEL::CH4));

    Serial.print(F("Rudder: "));
    Serial.print(radio.getPWM(Radio::CHANNEL::YAW));
    Serial.print(F("\t\t\t"));
    Serial.print(F("Rudder: "));
    Serial.print(xpilot.currentMode->getYawInput());
    Serial.print(F("\t\t\t"));
    Serial.print(F("Rudder: "));
    Serial.print(xpilot.currentMode->getYawOutput());
    Serial.print(F("\t\t\t"));
    Serial.print(F("Rudder: "));
    Serial.println(actuators.getServoOut(Actuators::CHANNEL::CH5));

#if defined(USE_FLAPERONS)
    Serial.print(F("Flaperon: "));
    Serial.print(xpilot.radio.getPWM(Radio::CHANNEL::AUX2));
    Serial.print(F("\t\t\t"));
    Serial.print(F("Flaperon: "));
    Serial.print(xpilot.currentMode->getFlaperon());
    Serial.print(F("\t\t\t"));
    Serial.print(F("Flaperon Position: "));
    Serial.println((int16_t)radio.getThreeSwitchPos(Radio::CHANNEL::AUX2, config().aux2RxConfig.trim, 68));
#endif
}

void XPDebug::printIMUTaskStats(void)
{
    CLEAR_TERMINAL();
    Scheduler::TaskStats taskStats;
    if (scheduler.getStats(xpilot.imuTaskId, taskStats))
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

void XPDebug::printRadioTaskStats(void)
{
    CLEAR_TERMINAL();
    Scheduler::TaskStats taskStats;
    if (scheduler.getStats(xpilot.radioTaskId, taskStats))
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

void XPDebug::printStateUpdateTaskStats(void)
{
    CLEAR_TERMINAL();
    Scheduler::TaskStats taskStats;
    if (scheduler.getStats(xpilot.stateUpdateTaskId, taskStats))
    {
        Serial.println(F("State Update Task Stats"));
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

void XPDebug::printFMUpdateTaskStats(void)
{
    CLEAR_TERMINAL();
    Scheduler::TaskStats taskStats;
    if (scheduler.getStats(xpilot.flightModeUpdateTaskId, taskStats))
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

void XPDebug::printFMRunTaskStats(void)
{
    CLEAR_TERMINAL();
    Scheduler::TaskStats taskStats;
    if (scheduler.getStats(xpilot.flightModeRunTaskId, taskStats))
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

void XPDebug::printFMOutputTaskStats(void)
{
    CLEAR_TERMINAL();
    Scheduler::TaskStats taskStats;
    if (scheduler.getStats(xpilot.flightModeOutputTaskId, taskStats))
    {
        Serial.println(F("FM Output Task Stats"));
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

void XPDebug::printLEDNotifierTaskStats(void)
{
    CLEAR_TERMINAL();
    Scheduler::TaskStats taskStats;
    if (scheduler.getStats(xpilot.ledNotifierTaskId, taskStats))
    {
        Serial.println(F("LED Notifier Task Stats"));
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

XPDebug xpDebug;