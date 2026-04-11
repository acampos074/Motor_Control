/*
 * DebugLog.c
 *
 * Single-producer (ISR) / single-consumer (main loop) ring buffer.
 *
 * Concurrency model (Cortex-M4, single core):
 *   - The ISR is the only writer; it owns 'wr'.
 *   - main() is the only reader; it owns 'rd'.
 *   - 'wr' is written by the ISR only after vsnprintf completes, so the
 *     reader never observes a partial string.
 *   - 'rd' is advanced by main() only after printf completes, so the slot
 *     is never reused while it is being printed.
 *   - No mutex is needed because only one side modifies each index.
 */

#include "DebugLog.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

static char              buf[DBG_SLOTS][DBG_SLOT_SIZE];
static volatile uint32_t wr = 0;
static volatile uint32_t rd = 0;

void dbg_log(const char *fmt, ...)
{
    uint32_t next = (wr + 1u) % DBG_SLOTS;
    if(next == rd)
    {
        return; /* buffer full — drop this message rather than block */
    }

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf[wr], DBG_SLOT_SIZE, fmt, args);
    va_end(args);

    wr = next; /* publish slot only after data is fully written */
}

void dbg_flush(void)
{
    while(rd != wr)
    {
        printf("%s", buf[rd]);
        rd = (rd + 1u) % DBG_SLOTS;
    }
}
