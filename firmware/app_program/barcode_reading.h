#ifndef BARCODE_READING_H
#define BARCODE_READING_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

typedef struct
{
    uint32_t sequence;
    char symbol;
    bool reversed;
} barcode_reading_result_t;

/** Create the reader event and register the side pin after Buddy 3 setup. */
ER barcode_reading_init (void);

/** Consume the GPIO IRQ FIFO on core 0. */
void barcode_reading_task (INT start_code, void * p_context);

void barcode_reading_reset (void);
void barcode_reading_set_enabled (bool enabled);
bool barcode_reading_latest (barcode_reading_result_t * p_result);

#endif
