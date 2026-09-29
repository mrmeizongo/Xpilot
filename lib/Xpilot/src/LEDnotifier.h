#ifndef _LED_NOTIFIER
#define _LED_NOTIFIER
#include "Arduino.h"
#include "SysConfig.h"

constexpr uint8_t SUCCESS_BLINK_COUNT = 2;
constexpr uint32_t SUCCESS_BLINK_DURATION = 250;

#if defined(ENABLE_LED_NOTIFIER)

class LEDNotifier
{
public:
    static void begin(void)
    {
        pinMode(LED_BUILTIN, OUTPUT);
        digitalWrite(LED_BUILTIN, LOW);
    }

    static void blinkLED(uint8_t count, uint32_t onDuration, uint32_t offDuration = 0)
    {
        if (count == 0)
            return;

        if (offDuration == 0)
            offDuration = onDuration;

        blinkCount_ = count;
        onDuration_ = onDuration;
        offDuration_ = offDuration;

        ledOn_ = true;
        active_ = true;
        stateStartTime_ = millis();

        digitalWrite(LED_BUILTIN, HIGH);
    }

    static void update(void* ctx)
    {
        if (!active_)
            return;

        const uint32_t now = millis();

        if (ledOn_)
        {
            if (now - stateStartTime_ < onDuration_)
                return;

            digitalWrite(LED_BUILTIN, LOW);
            ledOn_ = false;
            stateStartTime_ = now;

            if (--blinkCount_ == 0)
                active_ = false;
        }
        else
        {
            if (now - stateStartTime_ < offDuration_)
                return;

            digitalWrite(LED_BUILTIN, HIGH);
            ledOn_ = true;
            stateStartTime_ = now;
        }
    }

    static bool active(void) { return active_; }

private:
    static uint32_t stateStartTime_;
    static uint32_t onDuration_;
    static uint32_t offDuration_;

    static uint8_t blinkCount_;

    static bool ledOn_;
    static bool active_;
};

#else
class LEDNotifier
{
public:
    static void begin(void) {}

    static void blinkLED(uint8_t, uint32_t, uint32_t = 0) {}

    static void update(void*) {}

    static bool active(void) { return false; }
};
#endif

#endif // _LED_NOTIFIER