/**
 * @file debug.cpp
 * @brief Debug printing macros.
 * @author Wilkins White
 * @copyright 2019-2021 Nova Dynamics LLC
 */

#include <cstdio>
#include <stdarg.h>
#include "debug.h"

#if NOVAGSM_DEBUG

/**
 * @brief User defined function to print debug strings.
 * @param [in] level the log level of the message.
 * @param [in] file where in the source the message originated.
 * @param [in] line where in 'file' the message originated.
 * @param [in] str message c-string.
 */
extern void gsm_debug(int level, const char *file, int line, const char *str);

/**
 * @brief Handles debug formatting and passes string to user defined function.
 * @param [in] level the log level of the message.
 * @param [in] file where in the source the message originated.
 * @param [in] line where in 'file' the message originated.
 * @param [in] format standard c formatting string.
 * @see gsm_debug
 * @private
 */
void gsm_debug_print(
    int level, const char *file, int line, const char *format, ...)
{
    va_list argp;
    char buffer[256];

    va_start(argp, format);
    vsnprintf(buffer, sizeof(buffer), format, argp );
    va_end( argp );

    gsm_debug(level, file, line, buffer);
}
#endif
