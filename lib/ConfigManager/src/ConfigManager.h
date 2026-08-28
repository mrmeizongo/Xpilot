#ifndef _CONFIG_MANAGER_H
#define _CONFIG_MANAGER_H
#include <stdint.h>

#include "Config.h"
#include "ConfigID.h"
#include "ConfigValue.h"

class ConfigManager
{
public:
    using Callback = void (*)(ConfigID, void *);

    ConfigManager();

    void init();

    const Config &config() const;

    bool get(
        ConfigID id,
        ConfigValue &value,
        ConfigValueType &type) const;

    bool set(
        ConfigID id,
        const ConfigValue &value);

    bool save();
    bool load();

    void loadDefaults();

    void registerSubscriber(Callback, void *);

    void setIMUCalibration(const float (&)[3], const float (&)[3]);

    bool isDirty() const;

private:
    static constexpr uint16_t EEPROM_MAGIC =
        0x5850; // XP - XPilot firmware signature

    static constexpr uint8_t EEPROM_VERSION =
        0x01;

    static constexpr uint16_t EEPROM_ADDRESS =
        0x0;

    static constexpr uint8_t MAX_SUBSCRIBERS =
        1;

    struct StoredConfig
    {
        uint16_t magic;
        uint8_t version;

        Config config;

        uint16_t checksum;
    };

    Config _config;

    bool _dirty;

    struct Subscriber
    {
        Callback cb;
        void *ctx;
    };

    Subscriber subscribers[MAX_SUBSCRIBERS];
    uint8_t subscriberCount =
        0;

    bool validate(
        ConfigID id,
        const ConfigValue &value) const;

    static uint16_t calculateChecksum(
        const uint8_t *data,
        uint16_t length);
};
#endif //_CONFIG_MANAGER_H