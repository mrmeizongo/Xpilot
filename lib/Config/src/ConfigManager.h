#ifndef _CONFIG_MANAGER_H
#define _CONFIG_MANAGER_H
#include <stdint.h>

#include "Config.h"
#include "ConfigID.h"
#include "ConfigValue.h"

class ConfigManager
{
public:
    using Subscriber = void (*)(ConfigID);

    ConfigManager();

    void init();

    const Config& config() const;

    bool get(ConfigID id, ConfigValue& value, ConfigValueType& type) const;

    bool set(ConfigID id, const ConfigValue& value);

    bool save();
    bool load();

    void loadDefaults();

    void registerSubscriber(Subscriber);

    void setIMUCalibration(const float (&)[3], const float (&)[3]);

    bool isDirty() const;

private:
    static constexpr uint16_t EEPROM_MAGIC = 0x5850; // XP - XPilot firmware signature

    static constexpr uint8_t EEPROM_VERSION = 0x02;

    static constexpr uint16_t EEPROM_ADDRESS = 0x0;

    static constexpr int EEPROM_MAGIC_ADDR = EEPROM_ADDRESS;

    static constexpr int EEPROM_VERSION_ADDR = EEPROM_MAGIC_ADDR + sizeof(uint16_t);

    static constexpr int EEPROM_CONFIG_ADDR = EEPROM_VERSION_ADDR + sizeof(uint8_t);

    static constexpr int EEPROM_CHECKSUM_ADDR = EEPROM_CONFIG_ADDR + sizeof(Config);

    static constexpr uint8_t MAX_SUBSCRIBERS = 1;

    Config _config;

    bool _dirty;

    Subscriber _subscriber = nullptr;

    bool validateSet(ConfigID id, const ConfigValue& value) const;

    static uint16_t calculateChecksum(const uint8_t* data, uint16_t length);
    static uint16_t calculateEEPROMChecksum(int address, uint16_t length);
};
#endif //_CONFIG_MANAGER_H