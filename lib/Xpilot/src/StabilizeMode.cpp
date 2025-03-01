#include "Mode.h"
#include "IMU.h"

void StabilizeMode::enter(void)
{
    // Reset PIDF controllers
    Mode::rollPIDF.resetPIDF();
    Mode::pitchPIDF.resetPIDF();
}

void StabilizeMode::process(void)
{
    if (!radio.inFailsafe())
    {
        Mode::rollOut = SETINPUT(radio.getRxRollPWM(), ROLL_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -MAX_ROLL_ANGLE_DEGS, 0, MAX_ROLL_ANGLE_DEGS);
        Mode::pitchOut = SETINPUT(radio.getRxPitchPWM(), PITCH_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -MAX_ROLL_ANGLE_DEGS, 0, MAX_ROLL_ANGLE_DEGS);
        Mode::yawOut = SETINPUT(radio.getRxYawPWM(), YAW_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -MAX_ROLL_RATE_DEGS, 0, MAX_ROLL_RATE_DEGS);
    }
    else
        controlFailsafe();
}

void StabilizeMode::run(void)
{
    process();
#if defined(RUDDER_MIX_IN_STABILIZE)
    Mode::rudderMixer();
#endif
    yawController();
    float rollDemand = Mode::rollOut - imu.getRoll();
    float pitchDemand = Mode::pitchOut - imu.getPitch();
    rollDemand = map(rollDemand, -MAX_ROLL_ANGLE_DEGS, MAX_ROLL_ANGLE_DEGS, -MAX_ROLL_RATE_DEGS, MAX_ROLL_RATE_DEGS);
    pitchDemand = map(pitchDemand, -MAX_PITCH_ANGLE_DEGS, MAX_PITCH_ANGLE_DEGS, -MAX_PITCH_RATE_DEGS, MAX_PITCH_RATE_DEGS);

    int16_t roll = Mode::rollPIDF.Compute(rollDemand, imu.getGyroX());
    int16_t pitch = Mode::pitchPIDF.Compute(pitchDemand, imu.getGyroY());
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

void StabilizeMode::yawController(void)
{
#if defined(USE_HEADING_HOLD)
    if (yawOut != 0)
        Mode::yawPIDF.resetPIDF();
#else
    Mode::yawPIDF.resetPIDF();
#endif
}

void StabilizeMode::controlFailsafe(void)
{
    Mode::rollOut = 0;
    Mode::pitchOut = 0;
    Mode::yawOut = 0;
}