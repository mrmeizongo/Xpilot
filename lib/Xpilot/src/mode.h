#include <Arduino.h>
#include "Xpilot.h"

class Mode
{
public:
    Mode();
    void init();
    void setMode();
    void updateMode();

private:
    unsigned char modePinInt; // Interrupt pin to read mode
    void manualMode(Xpilot &xpilot);
    void FBWMode(Xpilot &xpilot);
    void stabilizeMode(Xpilot &xpilot);
};

extern Mode mode;