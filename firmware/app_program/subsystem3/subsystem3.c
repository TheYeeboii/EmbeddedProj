/** Subsystem 3 barcode input and worker lifecycle. */
#include "subsystem3.h"
#include "barcode_reading.h"
#include "../pin_config.h"
#include "../robot_hal.h"

#define BARCODE_TASK_PRIORITY (8)
#define BARCODE_TASK_STACK    (4096)

ER subsystem3_init (void)
{
    hal_gpio_input_pullup(BARCODE_DIGITAL_PIN);
    return barcode_reading_init();
}

ER subsystem3_start (void)
{
    T_CTSK settings = {
        .tskatr = TA_HLNG | TA_RNG3,
        .task = barcode_reading_task,
        .itskpri = BARCODE_TASK_PRIORITY,
        .stksz = BARCODE_TASK_STACK,
    };
#if TK_SUPPORT_SMP
    settings.tskatr |= TA_ASSPRC;
    settings.assprc = TP_PRC1;
#endif
    const ID task_id = tk_cre_tsk(&settings);
    if (task_id < E_OK)
    {
        return task_id;
    }

    const ER result = tk_sta_tsk(task_id, 0);
    if (result != E_OK)
    {
        (void) tk_del_tsk(task_id);
    }
    return result;
}
