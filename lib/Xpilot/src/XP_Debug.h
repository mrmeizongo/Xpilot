#ifndef _XP_DEBUG_H
#define _XP_DEBUG_H
#include "Xpilot.h"

class XP_Debug
{
public:
    XP_Debug(void) {}

    void init(void);

    static void printSchedulerRateTask(void* ctx) { static_cast<XP_Debug*>(ctx)->printSchedulerRate(); }
    static void printIOTask(void* ctx) { static_cast<XP_Debug*>(ctx)->printIO(); }

    static void printIMUTaskStatTask(void* ctx) { static_cast<XP_Debug*>(ctx)->printIMUTaskStats(); }
    static void printRadioTaskStatTask(void* ctx) { static_cast<XP_Debug*>(ctx)->printRadioTaskStats(); }
    static void printStateUpdateTaskStatTask(void* ctx) { static_cast<XP_Debug*>(ctx)->printStateUpdateTaskStats(); }
    static void printFMUpdateTaskStatTask(void* ctx) { static_cast<XP_Debug*>(ctx)->printFMUpdateTaskStats(); }
    static void printFMRunTaskStatTask(void* ctx) { static_cast<XP_Debug*>(ctx)->printFMRunTaskStats(); }
    static void printFMOutputTaskStatTask(void* ctx) { static_cast<XP_Debug*>(ctx)->printFMOutputTaskStats(); }
    static void printLEDNotifierTaskStatTask(void* ctx) { static_cast<XP_Debug*>(ctx)->printLEDNotifierTaskStats(); }

    void printSchedulerRate(void);
    void printIO(void);

    void printIMUTaskStats(void);
    void printRadioTaskStats(void);
    void printStateUpdateTaskStats(void);
    void printFMUpdateTaskStats(void);
    void printFMRunTaskStats(void);
    void printFMOutputTaskStats(void);
    void printLEDNotifierTaskStats(void);
};

extern XP_Debug xpDebug;

#endif // _XP_DEBUG_H