/* Console filtering only: sensor processing, safety and MQTT remain active. */
#ifndef ROBOT_DEBUG_H
#define ROBOT_DEBUG_H

#ifndef BUDDY3_DEBUG
#define BUDDY3_DEBUG 0
#endif

#if (BUDDY3_DEBUG != 0) && (BUDDY3_DEBUG != 1)
#error BUDDY3_DEBUG must be 0 or 1
#endif

/* Keep errors, safety events and Buddy 3 logs as direct tm_printf calls. */
#if BUDDY3_DEBUG
#define ROBOT_ROUTINE_LOG(...)                                                 \
    do                                                                         \
    {                                                                          \
    } while (0)
#else
#define ROBOT_ROUTINE_LOG(...) tm_printf(__VA_ARGS__)
#endif

#endif /* ROBOT_DEBUG_H */

/*** end of file ***/

