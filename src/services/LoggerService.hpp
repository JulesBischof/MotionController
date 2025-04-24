#pragma once

#include <stdio.h>
#include <stdarg.h>

namespace services
{
    enum class logLevel
    {
        Debug,
        Info,
        Warn,
        Error,
        Fatal, 
        None,
    };

    class LoggerService
    {
        private:
            static void _print(const char *prfx, const char *call, const char *msg, va_list args);
            static logLevel _logLevel;

        public:

            static void setLogLevel(logLevel logLevel);

            static void debug(const char *call, const char *msg, ...);
            static void info(const char *call, const char *msg, ...);
            static void warn(const char *call, const char *msg, ...);
            static void error(const char *call, const char *msg, ...);
            static void fatal(const char *call, const char *msg, ...);
    };
}