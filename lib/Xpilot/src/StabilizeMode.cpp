#include "Mode.h"
#include "IMU.h"

void StabilizeMode::enter(void)
{
    // Reset PIDF controllers
    Mode::rollPIDF.resetPIDF();
    Mode::pitchPIDF.resetPIDF();
    Mode::yawPIDF.resetPIDF();
}

void StabilizeMode::process(void)
{
    if (!radio.inFailsafe())
    {
        Mode::rollOut = GETINPUT(radio.getRxRollPWM(), ROLL_INPUT_DEADBAND, -MAX_ROLL_ANGLE_DEGS, MAX_ROLL_ANGLE_DEGS);
        Mode::pitchOut = GETINPUT(radio.getRxPitchPWM(), PITCH_INPUT_DEADBAND, -MAX_PITCH_ANGLE_DEGS, MAX_PITCH_ANGLE_DEGS);
        Mode::yawOut = GETINPUT(radio.getRxYawPWM(), YAW_INPUT_DEADBAND, -MAX_YAW_RATE_DEGS, MAX_YAW_RATE_DEGS);
        Mode::auxOut = 0; // Default aux output to mid position for now
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

    float rollDemand = Mode::rollOut - imu.getRoll();
    float pitchDemand = Mode::pitchOut - imu.getPitch();
    rollDemand = map(rollDemand, -MAX_ROLL_ANGLE_DEGS, MAX_ROLL_ANGLE_DEGS, -MAX_ROLL_RATE_DEGS, MAX_ROLL_RATE_DEGS);
    pitchDemand = map(pitchDemand, -MAX_PITCH_ANGLE_DEGS, MAX_PITCH_ANGLE_DEGS, -MAX_PITCH_RATE_DEGS, MAX_PITCH_RATE_DEGS);
    yawController();

    int16_t roll = Mode::rollPIDF.Compute(rollDemand, imu.getGyroX());
    int16_t pitch = Mode::pitchPIDF.Compute(pitchDemand, imu.getGyroY());
    int16_t yaw = Mode::yawPIDF.Compute(Mode::yawOut, imu.getGyroZ());

    Mode::planeMixer(roll, pitch, yaw);
    Mode::SRVout[Actuators::Channel::CH1] = map(Mode::SRVout[Actuators::Channel::CH1], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::CH2] = map(Mode::SRVout[Actuators::Channel::CH2], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::CH3] = map(Mode::SRVout[Actuators::Channel::CH3], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::CH4] = map(Mode::SRVout[Actuators::Channel::CH4], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::CH5] = map(Mode::SRVout[Actuators::Channel::CH5], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
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
    Mode::rollOut = 5;  // 5 degree roll to the right
    Mode::pitchOut = 0;
    Mode::yawOut = 0;
    Mode::auxOut = 0;
}