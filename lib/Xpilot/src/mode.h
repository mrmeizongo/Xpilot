#include <Arduino.h>
#include "Xpilot.h"

class Mode
{
public:
    Mode();
    void init();
    void update();
    void set();

private:
    unsigned char modePinInt; // Interrupt pin to read mode
    void manualMode();
    void FBWMode();
    void stabilizeMode();
};

extern Mode mode;