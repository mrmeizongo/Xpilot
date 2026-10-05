#ifndef _XP_DEBUG_H
#define _XP_DEBUG_H
#include "Xpilot.h"

class XPDebug
{
public:
    XPDebug(void) {}

    void init(void);

    static void printSchedulerRateTask(void* ctx) { static_cast<XPDebug*>(ctx)->printSchedulerRate(); }
    static void printIOTask(void* ctx) { static_cast<XPDebug*>(ctx)->printIO(); }

    static void printIMUTaskStatTask(void* ctx) { static_cast<XPDebug*>(ctx)->printIMUTaskStats(); }
    static void printRadioTaskStatTask(void* ctx) { static_cast<XPDebug*>(ctx)->printRadioTaskStats(); }
    static void printStateUpdateTaskStatTask(void* ctx) { static_cast<XPDebug*>(ctx)->printStateUpdateTaskStats(); }
    static void printFMUpdateTaskStatTask(void* ctx) { static_cast<XPDebug*>(ctx)->printFMUpdateTaskStats(); }
    static void printFMRunTaskStatTask(void* ctx) { static_cast<XPDebug*>(ctx)->printFMRunTaskStats(); }
    static void printFMOutputTaskStatTask(void* ctx) { static_cast<XPDebug*>(ctx)->printFMOutputTaskStats(); }
    static void printLEDNotifierTaskStatTask(void* ctx) { static_cast<XPDebug*>(ctx)->printLEDNotifierTaskStats(); }

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

extern XPDebug xpDebug;

#endif // _XP_DEBUG_H