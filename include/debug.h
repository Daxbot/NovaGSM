/**
 * @file debug.h
 * @brief Debug printing macros.
 * @author Wilkins White
 * @copyright 2019-2021 Nova Dynamics LLC
 */

#ifndef NOVAGSM_DEBUG_H_
#define NOVAGSM_DEBUG_H_

#if NOVAGSM_DEBUG
void gsm_debug_print(int level, const char *file, int line, const char *format, ...);
#endif

/**@{*/
/** Log problems that need to be resolved manually. */
#if NOVAGSM_DEBUG >= 1
    #define LOG_ERROR(...) gsm_debug_print(1, __FILE__, __LINE__, __VA_ARGS__)
#else
    #define LOG_ERROR(...) do {} while(0)
#endif
/**@}*/

/**@{*/
/** Log problems that will be resolved automatically. */
#if NOVAGSM_DEBUG >= 2
    #define LOG_WARN(...) gsm_debug_print(2, __FILE__, __LINE__, __VA_ARGS__)
#else
    #define LOG_WARN(...) do {} while(0)
#endif
/**@}*/

/**@{*/
/** Log one-shot informational messages. */
#if NOVAGSM_DEBUG >= 3
    #define LOG_INFO(...) gsm_debug_print(3, __FILE__, __LINE__, __VA_ARGS__)
#else
    #define LOG_INFO(...) do {} while(0)
#endif
/**@}*/

/**@{*/
/** Log verbose informational messages about internal driver operation. */
#if NOVAGSM_DEBUG >= 4
    #define LOG_VERBOSE(...) gsm_debug_print(4, __FILE__, __LINE__, __VA_ARGS__)
#else
    #define LOG_VERBOSE(...) do{} while(0)
#endif
/**@}*/

#endif /* NOVAGSM_DEBUG_H_ */
