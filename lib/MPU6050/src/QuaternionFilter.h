#pragma once
#ifndef _QUATERNIONFILTER_H
#define _QUATERNIONFILTER_H

class QuaternionFilter
{
    // for madgwick
    float GyroMeasError = PI * (40.0f / 180.0f);    // gyroscope measurement error in rads/s (start at 40 deg/s)
    float beta = sqrt(3.0f / 4.0f) * GyroMeasError; // compute beta

    float deltaT{0.};
    uint32_t newTime{0}, oldTime{0};

public:
    void update(float ax, float ay, float az, float gx, float gy, float gz, float *q)
    {
        newTime = micros();
        deltaT = newTime - oldTime;
        oldTime = newTime;
        deltaT = (deltaT * 0.001 * 0.001);

        madgwick6DOF(ax, ay, az, gx, gy, gz, deltaT, q);
    }

    // Madgwick Quaternion Update
    void madgwick6DOF(float ax, float ay, float az, float gx, float gy, float gz, float dt, float *q)
    {
        float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3]; // short name local variable for readability
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        float a_norm = (ax * ax) + (ay * ay) + (az * az);
        if (a_norm != 0.)
        {
            // Normalize accelerometer measurement
            recipNorm = 1.0 / sqrt(a_norm);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Auxiliary variables to avoid repeated arithmetic
            _2q0 = 2.0f * q0;
            _2q1 = 2.0f * q1;
            _2q2 = 2.0f * q2;
            _2q3 = 2.0f * q3;
            _4q0 = 4.0f * q0;
            _4q1 = 4.0f * q1;
            _4q2 = 4.0f * q2;
            _8q1 = 8.0f * q1;
            _8q2 = 8.0f * q2;
            q0q0 = q0 * q0;
            q1q1 = q1 * q1;
            q2q2 = q2 * q2;
            q3q3 = q3 * q3;

            // Gradient decent algorithm corrective step
            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
            recipNorm = 1.0 / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            // Apply feedback step
            qDot1 -= beta * s0;
            qDot2 -= beta * s1;
            qDot3 -= beta * s2;
            qDot4 -= beta * s3;
        }

        // Integrate rate of change of quaternion to yield quaternion
        q0 += qDot1 * dt;
        q1 += qDot2 * dt;
        q2 += qDot3 * dt;
        q3 += qDot4 * dt;

        // Normalize quaternion
        recipNorm = 1.0 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;

        q[0] = q0;
        q[1] = q1;
        q[2] = q2;
        q[3] = q3;
    }
};

#endif // _QUATERNIONFILTER_H
