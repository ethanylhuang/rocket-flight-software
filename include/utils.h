#ifndef UTILS_H
#define UTILS_H

#include <stdio.h>
#include <time.h>

/**
 * @brief Simple logging levels.
 */
typedef enum {
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARN,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_DEBUG
} LogLevel;

/**
 * @brief Logs a message with a given level.
 * 
 * @param level The log level.
 * @param message The message to log.
 */
void logMessage(LogLevel level, const char *message);

/**
 * @brief Formatted logging for easier use.
 */
void logInfo(const char *format, ...);
void logWarn(const char *format, ...);
void logError(const char *format, ...);
void logDebug(const char *format, ...);

#endif // UTILS_H
