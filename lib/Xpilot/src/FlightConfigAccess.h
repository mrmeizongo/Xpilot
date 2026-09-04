#ifndef _CONFIG_ACCESS_H
#define _CONFIG_ACCESS_H
#include "ConfigManager.h"

extern ConfigManager configManager;

inline const Config& config() { return configManager.config(); }

#endif //_CONFIG_ACCESS_H