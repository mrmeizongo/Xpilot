#ifndef _PLANE_CONFIG_H
#define _PLANE_CONFIG_H
#include "PlaneSelector.h"

/*
 * Use conditional compilation based on airplane definitions in PlaneSelector
 * Exmample
 * #if defined(USE_GLIDER) || defined(USE_CUB) || defined(USE_ACRO)
 * #error Only one airplane need to be defined at a time
 * #endif
 *
 * #if defined(USE_GLIDER)
 * #include "Glider.h"
 * #undef USE_GLIDER
 * #elif defined(USE_CUB)
 * #include "Cub.h"
 * #undef USE_CUB
 * #elif defined(USE_ACRO)
 * #include "Acro.h"
 * #undef USE_ACRO
 * #else
 * #define USE_DEFAULT
 * #endif
 */

/*
 * NOTE
 * If you intend to modify and use only the default settings in config.h you can leave this file as is
 */
#define USE_DEFAULT

#endif // PLANE_CONFIG_H