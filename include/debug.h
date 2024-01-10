/**
 * @file debug.h
 * @brief Debug printing macros.
 * @author Wilkins White
 * @copyright 2024 Nova Dynamics LLC
 */

#ifndef NOVAGSM_DEBUG_H_
#define NOVAGSM_DEBUG_H_

#define NOVAGSM_DEBUG_ERROR (1)
#define NOVAGSM_DEBUG_WARN (2)
#define NOVAGSM_DEBUG_INFO (3)
#define NOVAGSM_DEBUG_VERBOSE (4)
#define NOVAGSM_DEBUG_TRACE (5)

#ifndef NOVAGSM_DEBUG
#define NOVAGSM_DEBUG (0)
#endif

#if (NOVAGSM_DEBUG > 0)
void gsm_debug_print(int level, const char *format, ...);
#endif

/**@{*/
/** Log problems that need to be resolved manually. */
#if (NOVAGSM_DEBUG >= NOVAGSM_DEBUG_ERROR)
#define LOG_ERROR(...) gsm_debug_print(NOVAGSM_DEBUG_ERROR, __VA_ARGS__)
#else
#define LOG_ERROR(...) do {} while(0)
#endif
/**@}*/

/**@{*/
/** Log problems that will be resolved automatically. */
#if (NOVAGSM_DEBUG >= NOVAGSM_DEBUG_WARN)
#define LOG_WARN(...) gsm_debug_print(NOVAGSM_DEBUG_WARN, __VA_ARGS__)
#else
#define LOG_WARN(...) do {} while(0)
#endif
/**@}*/

/**@{*/
/** Log one-shot informational messages. */
#if (NOVAGSM_DEBUG >= NOVAGSM_DEBUG_INFO)
#define LOG_INFO(...) gsm_debug_print(NOVAGSM_DEBUG_INFO, __VA_ARGS__)
#else
#define LOG_INFO(...) do {} while(0)
#endif
/**@}*/

/**@{*/
/** Log verbose informational messages about internal operation. */
#if (NOVAGSM_DEBUG >= NOVAGSM_DEBUG_VERBOSE)
#define LOG_VERBOSE(...) gsm_debug_print(NOVAGSM_DEBUG_VERBOSE, __VA_ARGS__)
#else
#define LOG_VERBOSE(...) do{} while(0)
#endif
/**@}*/

/**@{*/
/** Log everything. */
#if (NOVAGSM_DEBUG >= NOVAGSM_DEBUG_TRACE)
#define LOG_TRACE(...) gsm_debug_print(NOVAGSM_DEBUG_TRACE, __VA_ARGS__)
#else
#define LOG_TRACE(...) do{} while(0)
#endif
/**@}*/

#endif // NOVAGSM_DEBUG_H_
