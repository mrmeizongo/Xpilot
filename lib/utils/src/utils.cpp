#include <Arduino.h>

#define gscale (250. / 32768.0) * (PI / 180.0) // gyro default 250 LSB per d/s -> rad/s

float vector_dot(float a[3], float b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_normalize(float a[3])
{
    float mag = sqrt(vector_dot(a, a));
    a[0] /= mag;
    a[1] /= mag;
    a[2] /= mag;
}

// Replaces arduino default map() function which uses long type, consuming more resources than is required
int16_t map16(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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