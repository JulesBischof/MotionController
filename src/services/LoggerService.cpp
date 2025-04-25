#include "LoggerService.hpp"

namespace services
{
    logLevel LoggerService::_logLevel = logLevel::None;

    void services::LoggerService::_print(const char *prfx, const char *call, const char *msg, va_list args)
    {
        printf("%s", prfx);
        printf("in call ");
        printf("%s", call);
        printf(" --- ");
        vprintf(msg, args);
        printf("\n");
    }

    void LoggerService::setLogLevel(logLevel logLevel)
    {
        _logLevel = logLevel;
    }

    void LoggerService::debug(const char *call, const char *msg, ...)
    {
        if (_logLevel <= logLevel::Debug)
        {
            va_list args;
            va_start(args, msg);
            _print("[DEBUG] ", call, msg, args);
            va_end(args);
        }
    }

    void LoggerService::info(const char *call, const char *msg, ...)
    {
        if (_logLevel <= logLevel::Info)
        {
            va_list args;
            va_start(args, msg);
            _print("[INFO] ", call, msg, args);
            va_end(args);
        }
    }

    void LoggerService::warn(const char *call, const char *msg, ...)
    {
        if (_logLevel <= logLevel::Warn)
        {
            va_list args;
            va_start(args, msg);
            _print("[WARN] ", call, msg, args);
            va_end(args);
        }
    }

    void LoggerService::error(const char *call, const char *msg, ...)
    {
        if (_logLevel <= logLevel::Error)
        {
            va_list args;
            va_start(args, msg);
            _print("[ERROR] ", call, msg, args);
            va_end(args);
        }
    }

    void LoggerService::fatal(const char *call, const char *msg, ...)
    {
        if (_logLevel <= logLevel::Fatal)
        {
            va_list args;
            va_start(args, msg);
            _print("[FATAL] ", call, msg, args);
            va_end(args);
        }
    }
}