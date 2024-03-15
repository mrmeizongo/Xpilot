// #include "SRV_Channel.h"
// #include "srv_config.h"

// SRV_Channel::SRV_Channel(void) {}

// void SRV_Channel::setOutputConstrained(Channel channel, unsigned char outputValue, OPERATION op, Xpilot &xpilot)
// {
//     outputValue = constrain(outputValue, 0, 180);

//     switch (channel)
//     {
//     case AILERON:
//         switch (op)
//         {
//         case ADD_TO:
//             xpilot.aileron_out += outputValue;
//             break;
//         case SUB_FROM:
//             xpilot.aileron_out -= outputValue;
//             break;
//         case SET:
//             xpilot.aileron_out = outputValue;
//             break;
//         default:
//             break;
//         }
//         break;

//     case ELEVATOR:
//         switch (op)
//         {
//         case ADD_TO:
//             xpilot.elevator_out += outputValue;
//             break;
//         case SUB_FROM:
//             xpilot.elevator_out -= outputValue;
//             break;
//         case SET:
//             xpilot.elevator_out = outputValue;
//             break;
//         default:
//             break;
//         }
//         break;
//     default:
//         break;
//     }
// }