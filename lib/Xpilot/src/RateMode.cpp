#include "Mode.h"
#include "IMU.h"

void RateMode::enter(void)
{
    // Reset PIDF controllers
    Mode::rollPIDF.resetPIDF();
    Mode::pitchPIDF.resetPIDF();
    Mode::yawPIDF.resetPIDF();
}

void RateMode::process(void)
{
    if (!radio.inFailsafe())
    {
        Mode::rollOut = GETINPUT(radio.getRxRollPWM(), ROLL_INPUT_DEADBAND, -MAX_ROLL_RATE_DEGS, MAX_ROLL_RATE_DEGS);
        Mode::pitchOut = GETINPUT(radio.getRxPitchPWM(), PITCH_INPUT_DEADBAND, -MAX_PITCH_RATE_DEGS, MAX_PITCH_RATE_DEGS);
        Mode::yawOut = GETINPUT(radio.getRxYawPWM(), YAW_INPUT_DEADBAND, -MAX_YAW_RATE_DEGS, MAX_YAW_RATE_DEGS);
#if defined(USE_AUX2)
        Mode::auxOut = radio.getRxAux2Pos();
#endif
    }
    else
        Mode::controlFailsafe();
}

void RateMode::run(void)
{
    process();
#if defined(RUDDER_MIX_IN_RATE)
    Mode::rudderMixer();
#endif

    yawController();
    int16_t roll = Mode::rollPIDF.Compute(Mode::rollOut, imu.getGyroX());
    int16_t pitch = Mode::pitchPIDF.Compute(Mode::pitchOut, imu.getGyroY());
    int16_t yaw = Mode::yawPIDF.Compute(Mode::yawOut, imu.getGyroZ());

    Mode::planeMixer(roll, pitch, yaw);
    Mode::SRVout[Actuators::Channel::CH1] = map(Mode::SRVout[Actuators::Channel::CH1], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::CH2] = map(Mode::SRVout[Actuators::Channel::CH2], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::CH3] = map(Mode::SRVout[Actuators::Channel::CH3], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::CH4] = map(Mode::SRVout[Actuators::Channel::CH4], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::setServoOut();
}

void RateMode::yawController(void)
{
    Mode::yawPIDF.resetPIDF();
}