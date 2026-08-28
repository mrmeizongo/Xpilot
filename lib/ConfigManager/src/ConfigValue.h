#ifndef _CONFIG_VALUE_H
#define _CONFIG_VALUE_H
#include <stdint.h>

enum class ConfigValueType : uint8_t
{
    FLOAT = 0x00,
    UINT16 = 0x01,
    INT16 = 0x02,
    UINT8 = 0x03,
    INT8 = 0x04,
    BOOL = 0x05
};

union ConfigValue
{
    uint8_t u8;
    int8_t i8;
    uint16_t u16;
    int16_t i16;
    uint32_t raw; // Not used as a config value type, holds the raw data bytes
    float f;
};

#endif //_CONFIG_VALUE_H