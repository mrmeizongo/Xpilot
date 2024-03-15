// #include <Arduino.h>
// #include <Xpilot.h>
// #include <I2Cdev.h>
// #include "srv_config.h"

// class SRV_Channel
// {
// public:
//     enum Channel : uint8_t
//     {
//         AILERON = 1,
//         ELEVATOR
//     };

//     enum OPERATION : uint8_t
//     {
//         ADD_TO = 1,
//         SUB_FROM,
//         SET
//     };

//     SRV_Channel(void);
//     CLASS_NO_COPY(SRV_Channel);

//     void setOutputConstrained(Channel channel, unsigned char outputValue, OPERATION op, Xpilot &xpilot = xpilot);
// };