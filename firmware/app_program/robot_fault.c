/** @file robot_fault.c
 *
 * @brief Buddy 1 (shared): one fault byte for the whole robot.
 *
 * See robot_fault.h for the locking rule.
 */
#include <tk/tkernel.h>

#include "robot_fault.h"

/* Changed only inside DI()/EI() below; volatile so readers always load the
 * current value. */
static volatile uint8_t g_faults = 0u;

void robot_fault_set (uint8_t mask)
{
    UINT interrupt_state = 0u;

    DI(interrupt_state);
    /* Both operands are bytes, so the result fits in one. */
    g_faults = (uint8_t) (g_faults | (mask & ROBOT_FAULT_ALL));
    EI(interrupt_state);
}

void robot_fault_clear (uint8_t mask)
{
    UINT interrupt_state = 0u;

    DI(interrupt_state);
    /* The complement is masked back to the low byte. */
    g_faults = (uint8_t) (g_faults & (uint8_t) (~mask & 0xFFu));
    EI(interrupt_state);
}

void robot_fault_update (uint8_t clear_mask, uint8_t set_mask)
{
    UINT     interrupt_state = 0u;
    uint32_t faults          = 0u;

    DI(interrupt_state);
    faults   = ((uint32_t) g_faults & ~(uint32_t) clear_mask) |
               ((uint32_t) set_mask & ROBOT_FAULT_ALL);
    g_faults = (uint8_t) (faults & 0xFFu); /* Masked to one byte. */
    EI(interrupt_state);
}

uint8_t robot_fault_get (void)
{
    return g_faults;
}

/*** end of file ***/

