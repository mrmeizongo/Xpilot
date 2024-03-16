#include <Arduino.h>
#include "Xpilot.h"

class Mode
{
public:
    Mode();
    void init(Xpilot &xpilot);
    void setMode();
    void updateMode();
};

extern Mode mode;