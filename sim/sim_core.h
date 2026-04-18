/* sim_core.h — C-callable interface that x48_main.c and step_dispatch.c
 * use to talk to the core-selector. Kept separate so the Verilog integration
 * (C++) can live in its own file without leaking Vsaturn_cpu types.
 */
#ifndef SIM_CORE_H
#define SIM_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

typedef enum {
    CORE_C = 0,
    CORE_RTL,
    CORE_LOCKSTEP,
} sim_core_mode_t;

/* Called by x48_main.c while scanning argv. Returns false on unknown mode. */
bool            select_core( const char* name );
sim_core_mode_t current_core( void );

/* --auto-keys flag: inject random key press/release every N instructions
 * to exercise the keyboard interrupt path (mostly useful for lockstep soak). */
void sim_enable_auto_keys( unsigned period );

/* --trace=<file> / --max-steps=N: dump one STEP line per instruction, exit
 * after N instructions. Both modes use the same deterministic auto-key
 * schedule, so running --core=c and --core=rtl and diffing the traces pins
 * the exact instruction where rtl first diverges from the C reference. */
void sim_enable_trace( const char* path, unsigned long max_steps );

/* Called by x48_main.c after ui4x_start()/init_emulator(), once the
 * saturn_t global is fully initialised. RTL/lockstep use this to push
 * the startup state into the Verilog model. */
void sim_core_after_init( void );

/* Linker-wrap shims (see step_dispatch.c). */
void __real_step_instruction( void );
void __wrap_step_instruction( void );

/* Implemented in verilog_core.cpp. Safe no-op when compiled without the
 * Verilog model (then RTL/lockstep fall back to C). */
void verilog_core_reset( void );
void verilog_core_sync_from_saturn( void );
void verilog_core_sync_to_saturn( void );
void verilog_core_step_one( void );

#ifdef __cplusplus
}
#endif

#endif
