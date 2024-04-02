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

// Default Arduino map function returns value of type long
// For this project, that is overkill so to save some space, we return uint8_t instead(max 255)
uint8_t map8(int x, uint16_t in_min, uint16_t in_max, uint8_t out_min, uint8_t out_max)
{
    return ((x - in_min) * (out_max - out_min)) / ((in_max - in_min) + out_min);
}