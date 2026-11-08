/** @file robot_debug.h
 *
 * @brief Console filtering only: sensor processing, safety and MQTT remain
 *        active whatever BUDDY3_DEBUG is set to.
 */
#ifndef ROBOT_DEBUG_H
#define ROBOT_DEBUG_H

#ifndef BUDDY3_DEBUG
#define BUDDY3_DEBUG 0
#endif

#if (BUDDY3_DEBUG != 0) && (BUDDY3_DEBUG != 1)
#error BUDDY3_DEBUG must be 0 or 1
#endif

/* Keep errors, safety events and Buddy 3 logs as direct tm_printf calls.
 * NOTE: ROBOT_ROUTINE_LOG stays a variadic macro (a deviation from 6.3.a):
 * tm_printf has no va_list variant, so a function could not forward the
 * arguments. It expands to a single expression with no control flow. */
#if BUDDY3_DEBUG
#define ROBOT_ROUTINE_LOG(...) ((void) 0)
#else
#define ROBOT_ROUTINE_LOG(...) tm_printf(__VA_ARGS__)
#endif

#endif /* ROBOT_DEBUG_H */

/*** end of file ***/

