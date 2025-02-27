#include "Mode.h"
#include "IMU.h"

void RateMode::enter(void)
{
    // Reset PIDF controllers
    rollPIDF.resetPIDF();
    pitchPIDF.resetPIDF();
}

void RateMode::process(void)
{
    rollInput = SETINPUT(radio.getRxRollPWM(), ROLL_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -MAX_ROLL_RATE_DEGS, MAX_ROLL_RATE_DEGS);
    pitchInput = SETINPUT(radio.getRxPitchPWM(), PITCH_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -MAX_ROLL_RATE_DEGS, MAX_ROLL_RATE_DEGS);
    yawInput = SETINPUT(radio.getRxYawPWM(), YAW_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -MAX_ROLL_RATE_DEGS, MAX_ROLL_RATE_DEGS);
}

void RateMode::run(void)
{
    process();
#if defined(RUDDER_MIX_IN_RATE)
    rudderMixer();
#endif
    yawController();
    int16_t roll = rollPIDF.Compute(rollInput, imu.getGyroX());
    int16_t pitch = pitchPIDF.Compute(pitchInput, imu.getGyroY());
    int16_t yaw = yawPIDF.Compute(yawInput, imu.getGyroZ());

    planeMixer(roll, pitch, yaw);
    SRVout[Actuators::Channel::AILERON1] = constrain(SRVout[Actuators::Channel::AILERON1], -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
    SRVout[Actuators::Channel::AILERON2] = constrain(SRVout[Actuators::Channel::AILERON2], -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
    SRVout[Actuators::Channel::ELEVATOR] = constrain(SRVout[Actuators::Channel::ELEVATOR], -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
    SRVout[Actuators::Channel::RUDDER] = constrain(SRVout[Actuators::Channel::RUDDER], -MAX_PID_OUTPUT, MAX_PID_OUTPUT);

    SRVout[Actuators::Channel::AILERON1] = map(SRVout[Actuators::Channel::AILERON1], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    SRVout[Actuators::Channel::AILERON2] = map(SRVout[Actuators::Channel::AILERON2], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    SRVout[Actuators::Channel::ELEVATOR] = map(SRVout[Actuators::Channel::ELEVATOR], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    SRVout[Actuators::Channel::RUDDER] = map(SRVout[Actuators::Channel::RUDDER], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
}