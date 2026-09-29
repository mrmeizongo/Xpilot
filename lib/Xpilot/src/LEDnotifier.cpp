#include "LEDnotifier.h"

#if defined(ENABLE_LED_NOTIFIER)
uint32_t LEDNotifier::stateStartTime_ = 0;
uint32_t LEDNotifier::onDuration_ = 0;
uint32_t LEDNotifier::offDuration_ = 0;

uint8_t LEDNotifier::blinkCount_ = 0;

bool LEDNotifier::ledOn_ = false;
bool LEDNotifier::active_ = false;
#endif