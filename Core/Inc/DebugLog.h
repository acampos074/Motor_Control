/*
 * DebugLog.h
 *
 * Lock-free single-producer / single-consumer ring buffer for debug output.
 *
 * Problem: printf() over UART blocks for ~87 µs per character. Calling it
 * from the 40 kHz TIM1 ISR (25 µs budget) causes ISR overrun and stalls
 * the motor control loop.
 *
 * Solution:
 *   - ISR calls dbg_log() — formats into a ring buffer slot via vsnprintf
 *     (pure computation, no waiting).  Drops silently when the buffer is full.
 *   - main loop calls dbg_flush() — drains the buffer via printf at whatever
 *     rate the UART can sustain, without affecting the ISR.
 *
 * Usage:
 *   ISR / ISR-called functions : dbg_log("fmt", ...);
 *   main() / ES_Run hook       : dbg_flush();
 */

#ifndef INC_DEBUGLOG_H_
#define INC_DEBUGLOG_H_

/* Number of slots in the ring buffer.  Each slot holds one formatted line.
 * Increase if the buffer fills faster than main() can drain it. */
#define DBG_SLOTS     64u

/* Maximum bytes per formatted message including the null terminator. */
#define DBG_SLOT_SIZE 128u

/* Queue a formatted debug message from ISR context.
 * Drops the message silently when the buffer is full (never blocks). */
void dbg_log(const char *fmt, ...);

/* Drain all pending messages to stdout (printf).
 * Call from the main loop — never from an ISR. */
void dbg_flush(void);

#endif /* INC_DEBUGLOG_H_ */
