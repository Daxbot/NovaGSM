/** Debug printing macros.
 *
 * @file debug.h
 * @author Wilkins White
 * @copyright 2019 Nova Dynamics LLC
 * @version 1.0
 */

#ifndef _GSM_DEBUG_H_
#define _GSM_DEBUG_H_

#ifdef GSM_DEBUG
void gsm_debug_print(int level, const char *file, int line, const char *format, ...);
#endif

/**@{*/
/** Log problems that may need to be resolved by the user. */
#if GSM_DEBUG >= 1
    #define GSM_ERROR(...) gsm_debug_print(1, __FILE__, __LINE__, __VA_ARGS__)
#else
    #define GSM_ERROR(...) do {} while(0)
#endif
/**@}*/

/**@{*/
/** Log problems that will be resolved automatically. */
#if GSM_DEBUG >= 2
    #define GSM_WARN(...) gsm_debug_print(2, __FILE__, __LINE__, __VA_ARGS__)
#else
    #define GSM_WARN(...) do {} while(0)
#endif
/**@}*/

/**@{*/
/** Log one-shot informational messages about state changes and connection status. */
#if GSM_DEBUG >= 3
    #define GSM_INFO(...) gsm_debug_print(3, __FILE__, __LINE__, __VA_ARGS__)
#else
    #define GSM_INFO(...) do {} while(0)
#endif
/**@}*/

/**@{*/
/** Log verbose informational messages about internal driver operation. */
#if GSM_DEBUG >= 4
    #define GSM_VERBOSE(...) gsm_debug_print(4, __FILE__, __LINE__, __VA_ARGS__)
#else
    #define GSM_VERBOSE(...) do{} while(0)
#endif
/**@}*/

#endif /* _GSM_DEBUG_H_ */
