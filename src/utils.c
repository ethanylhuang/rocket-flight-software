#include "utils.h"
#include <stdarg.h>
#include <stdio.h>

void logMessage(LogLevel level, const char *message) {
    const char *levelStr;
    switch (level) {
        case LOG_LEVEL_INFO:  levelStr = "[INFO]"; break;
        case LOG_LEVEL_WARN:  levelStr = "[WARN]"; break;
        case LOG_LEVEL_ERROR: levelStr = "[ERROR]"; break;
        case LOG_LEVEL_DEBUG: levelStr = "[DEBUG]"; break;
        default:               levelStr = "[UNKNOWN]"; break;
    }
    printf("%s %s\n", levelStr, message);
}

void logInfo(const char *format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    logMessage(LOG_LEVEL_INFO, buffer);
}

void logWarn(const char *format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    logMessage(LOG_LEVEL_WARN, buffer);
}

void logError(const char *format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    logMessage(LOG_LEVEL_ERROR, buffer);
}

void logDebug(const char *format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    logMessage(LOG_LEVEL_DEBUG, buffer);
}