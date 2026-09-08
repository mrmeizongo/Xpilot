#pragma once
#ifndef _QUATERNIONFILTER_H
#define _QUATERNIONFILTER_H

class QuaternionFilter
{
    // for madgwick
    float GyroMeasError = PI * (40.0f / 180.0f); // gyroscope measurement error in rads/s (start at 40 deg/s)
    float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;

    float deltaT;

    // for mahony
    float Kp = 30.0;
    float Ki = 0.0;

public:
    QuaternionFilter(float dt = 0.001f)
        : deltaT(dt)
    {
    }

    void mahony6DOF(float ax, float ay, float az, float gx, float gy, float gz, float* q)
    {
        float recipNorm;
        float vx, vy, vz;
        float ex, ey, ez; //error terms
        float qa, qb, qc;
        static float ix = 0.0f, iy = 0.0f, iz = 0.0f; //integral feedback terms

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        float tmp = ax * ax + ay * ay + az * az;
        if (tmp > 0.0f)
        {
            // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
            recipNorm = 1.0f / sqrtf(tmp);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Estimated direction of gravity in the body frame (factor of two divided out)
            vx = q[1] * q[3] - q[0] * q[2];
            vy = q[0] * q[1] + q[2] * q[3];
            vz = q[0] * q[0] - 0.5f + q[3] * q[3];

            // Error is cross product between estimated and measured direction of gravity in body frame
            // (half the actual magnitude)
            ex = (ay * vz - az * vy);
            ey = (az * vx - ax * vz);
            ez = (ax * vy - ay * vx);

            // Apply proportional feedback to gyro term
            gx += Kp * ex;
            gy += Kp * ey;
            gz += Kp * ez;

            // Compute and apply to gyro term the integral feedback, if enabled
            if (Ki > 0.0f)
            {
                ix += Ki * ex * deltaT; // integral error scaled by Ki
                iy += Ki * ey * deltaT;
                iz += Ki * ez * deltaT;
                gx += ix; // apply integral feedback
                gy += iy;
                gz += iz;
            }
        }

        // Integrate rate of change of quaternion, q cross gyro term
        const float halfDt = 0.5f * deltaT;

        gx *= halfDt; // pre-multiply common factors
        gy *= halfDt;
        gz *= halfDt;
        qa = q[0];
        qb = q[1];
        qc = q[2];
        q[0] += (-qb * gx - qc * gy - q[3] * gz);
        q[1] += (qa * gx + qc * gz - q[3] * gy);
        q[2] += (qa * gy - qb * gz + q[3] * gx);
        q[3] += (qa * gz + qb * gy - qc * gx);

        // renormalise quaternion
        recipNorm = 1.0f / sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        q[0] = q[0] * recipNorm;
        q[1] = q[1] * recipNorm;
        q[2] = q[2] * recipNorm;
        q[3] = q[3] * recipNorm;
    }

    void madgwick6DOF(float ax, float ay, float az, float gx, float gy, float gz, float* q)
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
        if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
        {
            // Normalize accelerometer measurement
            recipNorm = 1 / sqrtf((ax * ax) + (ay * ay) + (az * az));
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
            recipNorm = 1 / sqrtf((s0 * s0) + (s1 * s1) + (s2 * s2) + (s3 * s3)); // normalise step magnitude
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
        q0 += qDot1 * deltaT;
        q1 += qDot2 * deltaT;
        q2 += qDot3 * deltaT;
        q3 += qDot4 * deltaT;

        // Normalize quaternion
        recipNorm = 1 / sqrtf((q0 * q0) + (q1 * q1) + (q2 * q2) + (q3 * q3));
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
