#include "Mode.h"
#include "IMU.h"

void RateMode::enter(void)
{
    // Reset PIDF controllers
    Mode::rollPIDF.resetPIDF();
    Mode::pitchPIDF.resetPIDF();
}

void RateMode::process(void)
{
    if (!radio.inFailsafe())
    {
        Mode::rollOut = SETINPUT(radio.getRxRollPWM(), ROLL_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -MAX_ROLL_RATE_DEGS, 0, MAX_ROLL_RATE_DEGS);
        Mode::pitchOut = SETINPUT(radio.getRxPitchPWM(), PITCH_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -MAX_ROLL_RATE_DEGS, 0, MAX_ROLL_RATE_DEGS);
        Mode::yawOut = SETINPUT(radio.getRxYawPWM(), YAW_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -MAX_ROLL_RATE_DEGS, 0, MAX_ROLL_RATE_DEGS);
    }
    else
        controlFailsafe();
}

void RateMode::run(void)
{
    process();
#if defined(RUDDER_MIX_IN_RATE)
    rudderMixer();
#endif
    yawController();
    int16_t roll = Mode::rollPIDF.Compute(Mode::rollOut, imu.getGyroX());
    int16_t pitch = Mode::pitchPIDF.Compute(Mode::pitchOut, imu.getGyroY());
    int16_t yaw = Mode::yawPIDF.Compute(Mode::yawOut, imu.getGyroZ());

    Mode::planeMixer(roll, pitch, yaw);
    Mode::SRVout[Actuators::Channel::AILERON1] = constrain(Mode::SRVout[Actuators::Channel::AILERON1], -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
    Mode::SRVout[Actuators::Channel::AILERON2] = constrain(Mode::SRVout[Actuators::Channel::AILERON2], -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
    Mode::SRVout[Actuators::Channel::ELEVATOR] = constrain(Mode::SRVout[Actuators::Channel::ELEVATOR], -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
    Mode::SRVout[Actuators::Channel::RUDDER] = constrain(Mode::SRVout[Actuators::Channel::RUDDER], -MAX_PID_OUTPUT, MAX_PID_OUTPUT);

    Mode::SRVout[Actuators::Channel::AILERON1] = map(Mode::SRVout[Actuators::Channel::AILERON1], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::AILERON2] = map(Mode::SRVout[Actuators::Channel::AILERON2], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::ELEVATOR] = map(Mode::SRVout[Actuators::Channel::ELEVATOR], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::RUDDER] = map(Mode::SRVout[Actuators::Channel::RUDDER], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::setServoOut();
}

void RateMode::yawController(void)
{
    Mode::yawPIDF.resetPIDF();
}

void RateMode::controlFailsafe(void)
{
    Mode::rollOut = 0;
    Mode::pitchOut = 0;
    Mode::yawOut = 0;
}