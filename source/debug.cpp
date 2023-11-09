/**
 * @file debug.cpp
 * @brief Debug printing macros.
 * @author Wilkins White
 * @copyright 2023 Nova Dynamics LLC
 */

#include <cstdio>
#include <stdarg.h>
#include "debug.h"

#if (NOVAGSM_DEBUG && NOVAGSM_DEBUG > 0)

/**
 * @brief User defined function to print debug strings.
 * @param [in] level the log level of the message.
 * @param [in] str message c-string.
 */
extern void gsm_debug(int level, const char *str);

/**
 * @brief Handles debug formatting and passes string to user defined function.
 * @param [in] level the log level of the message.
 * @param [in] format standard c formatting string.
 * @see gsm_debug
 * @private
 */
void gsm_debug_print(int level, const char *format, ...)
{
    va_list argp;
    char buffer[256];

    va_start(argp, format);
    vsnprintf(buffer, sizeof(buffer), format, argp);
    va_end(argp);

    gsm_debug(level, buffer);
}

#endif // (NOVAGSM_DEBUG && NOVAGSM_DEBUG > 0)
