#include <Arduino.h>

#define gScale (250. / 32768.0) * (PI / 180.0) // gyro default 250 LSB per d/s -> rad/s
#define vectorDot(a, b) (((a[0]) * (b[0])) + ((a[1]) * (b[1])) + ((a[2]) * (b[2])))

void vectorNormalize(float a[3])
{
    float mag = sqrt(vectorDot(a, a));
    a[0] /= mag;
    a[1] /= mag;
    a[2] /= mag;
}

// simple pid controller
// int pid_ctrl(int error, float *integral, int *previous, float kp, float ki, float kd, float deltaT)
// {
//     int proportional = error;
//     *integral += (error * deltaT);
//     float derivative = (float)((error - (*previous)) / deltaT);
//     *previous = error;
//     int output = (int)((kp * proportional) + (ki * (*integral)) + (kd * derivative));
//     return output;
// }