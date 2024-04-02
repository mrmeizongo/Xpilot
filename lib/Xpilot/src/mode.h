#pragma once

#include "Xpilot.h"

class Mode
{
public:
    Mode();
    void init();
    void update();
    void process();

private:
    unsigned char modePinInt; // Interrupt pin to read mode
    void manualMode();
    void FBWMode();
    void stabilizeMode();
};

extern Mode mode;