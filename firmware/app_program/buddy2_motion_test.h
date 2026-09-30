/**
 * @file    buddy2_motion_test.h
 * @brief   Buddy 2: button-started motion test (for calibration).
 */

#ifndef BUDDY2_MOTION_TEST_H
#define BUDDY2_MOTION_TEST_H

#include <tk/tkernel.h>

/** Task body: waits for the GP20 button, then runs forward / left / right /
 *  backward and prints encoder data for each step. */
void buddy2_motion_test_task(INT stacd, void *p_exinf);

#endif /* BUDDY2_MOTION_TEST_H */
