/* harness.h — thin shim for the standalone test harness.
 * Pulls in the real Saturn headers (which compile fine with lua5.4)
 * and adds only what the harness-specific files need beyond that.
 */
#ifndef HARNESS_H
#define HARNESS_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

/* The real Saturn headers — order matters */
#include "types.h"
#include "memory.h"   /* declares bus_fetch_nibble, bus_write_nibble, read_nibble_crc, nibble_masks */
#include "emulate.h"  /* declares saturn_t, saturn, step_instruction, etc. */

/* Timer stubs type (used in stubs.c only) */
typedef struct { long t1_ticks; long t2_ticks; } t1_t2_ticks;

/* Symbolic names for timer IDs used in emulate.c */
#define RUN_TIMER   0
#define IDLE_TIMER  1
#define T1_TIMER    2

/* x48ng_config type — must match options.h exactly so emulate.c's field
 * reads (esp. inhibit_shutdown) land at the correct offset. */
typedef struct {
    bool  enable_wire;
    bool  enable_ir;
    bool  enable_debugger;
    bool  throttle;
    bool  reset;
    bool  inhibit_shutdown;
    char* ir_serial_device;
} config_t;
extern config_t x48ng_config;

/* extern debugger flags (emulate.c references these) */
extern int  enter_debugger;
extern bool in_debugger;

/* Saturn global (defined in emulate.c) */
/* extern saturn_t saturn;  — already declared in emulate.h */

/* flat_memory.c helpers */
void flat_memory_init(void);
void mem_write_hex(Address addr, const char *hex);
void mem_clear(void);
void mem_read_range(Address addr, unsigned char *buf, int count);

/* generate_tests.c helpers */
void print_reg64(FILE *f, const char *name, const Nibble *r);
void print_state(FILE *f, const char *tag);
void run_test(FILE *f, const char *name,
              Address pc_start, const char *prog_hex, int n_steps);

#endif /* HARNESS_H */
