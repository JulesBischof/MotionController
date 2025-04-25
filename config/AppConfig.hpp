#ifndef APPCONFIG_H
#define APPCONFIG_H

#include "LoggerService.hpp"

/*  ================================== */
/*  ===========  App MODE  =========== */
/*  ================================== */
// 0: run device tests
// 1: run prodictive code

#define APP_MODE (1)

#define SIMULATE_GRIPCONTROLL (0)

constexpr services::logLevel APPCONFIG_LOGLEVEL = services::logLevel::Error;

#endif // APPCONFIG_H