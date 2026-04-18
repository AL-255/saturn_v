/* x48_main.c — replacement for x48ng/src/main.c with --core=<c|rtl|lockstep>.
 *
 * Keeps x48ng's peripheral model (memory.c ROM/RAM/I-O mapping, timers.c,
 * serial.c, SDL UI) untouched. The only behavioural substitution is that the
 * CPU's step_instruction() is linker-wrapped (sim/step_dispatch.c) so that:
 *   --core=c         → original x48ng emulate.c step_instruction (reference)
 *   --core=rtl       → Verilator model of saturn_cpu.v
 *   --core=lockstep  → both run, state compared each step, divergences abort
 *
 * Everything below the #include list is copied near-verbatim from
 * x48ng/src/main.c; only the argv scan and the one call to select_core()
 * before config_init() differ.
 */
#include <fcntl.h>
#include <langinfo.h>
#include <locale.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/time.h>

#include <glib.h>

#include "core/debugger.h"
#include "core/emulate.h"
#include "core/init.h"
#include "core/timers.h"

#include "ui4x/src/api.h"

#include "emulator_api.h"
#include "options.h"
#include "romio.h"

#include "sim_core.h"    /* select_core() prototype */

config_t x48ng_config;

void signal_handler( int sig )
{
    switch ( sig ) {
        case SIGINT:
            enter_debugger |= USER_INTERRUPT;
            break;
        case SIGALRM:
            sigalarm_triggered = true;
            break;
        case SIGPIPE:
            ui4x_exit();
            stop_emulator();
            exit( EXIT_SUCCESS );
        default:
            break;
    }
}

/* Strip a --core=<mode> argument out of argv in place, setting the mode
 * via select_core() and squashing the slot so x48ng's config_init() sees
 * a clean argv. Returns true on success, false on unrecognised mode. */
static bool extract_core_arg( int* argc, char** argv )
{
    const char* requested = "c";
    unsigned    autokey_period = 0;
    const char* trace_path = NULL;
    unsigned long max_steps = 0;
    int w = 1;
    for ( int i = 1; i < *argc; i++ ) {
        const char* a = argv[ i ];
        const char* eq = NULL;
        if ( strncmp( a, "--core=", 7 ) == 0 )      eq = a + 7;
        else if ( strncmp( a, "-core=", 6 ) == 0 )  eq = a + 6;
        if ( eq != NULL ) {
            requested = eq;
            continue;    /* drop this slot */
        }
        if ( strncmp( a, "--auto-keys", 11 ) == 0 ) {
            autokey_period = ( a[ 11 ] == '=' ) ? (unsigned)atoi( a + 12 ) : 50000u;
            continue;
        }
        if ( strncmp( a, "--trace=", 8 ) == 0 ) {
            trace_path = a + 8;
            continue;
        }
        if ( strncmp( a, "--max-steps=", 12 ) == 0 ) {
            max_steps = strtoul( a + 12, NULL, 0 );
            continue;
        }
        argv[ w++ ] = argv[ i ];
    }
    *argc = w;
    if ( autokey_period )
        sim_enable_auto_keys( autokey_period );
    if ( trace_path && max_steps )
        sim_enable_trace( trace_path, max_steps );
    else if ( trace_path || max_steps ) {
        fprintf( stderr, "--trace=<file> requires --max-steps=N (and vice versa)\n" );
        return false;
    }
    return select_core( requested );
}

int main( int argc, char** argv )
{
    setlocale( LC_ALL, "C" );

    if ( !extract_core_arg( &argc, argv ) ) {
        fprintf( stderr, "usage: %s [--core=c|rtl|lockstep] [x48ng args]\n", argv[ 0 ] );
        return 2;
    }

    char* progname = g_path_get_basename( argv[ 0 ] );
    char* progpath = g_path_get_dirname( argv[ 0 ] );
    char* default_datadir = g_build_filename( g_get_user_config_dir(), progname, NULL );

    x48ng_config = *config_init( argc, argv );

    ui4x_emulator_api_t emulator_api = {
        .press_key = press_key,
        .release_key = release_key,
        .is_key_pressed = is_key_pressed,
        .is_display_on = get_display_state,
        .get_annunciators = get_annunciators,
        .get_lcd_buffer = get_lcd_buffer,
        .get_contrast = get_contrast,
        .do_mount_sd = NULL,
        .do_unmount_sd = NULL,
        .is_sd_mounted = NULL,
        .get_sd_path = NULL,
        .do_reset = NULL,
        .do_stop = exit_emulator,
        .do_sleep = NULL,
        .do_wake = NULL,
        .do_debug = NULL,
    };
    ui4x_init( argc, argv, progname, progpath, default_datadir, &emulator_api );

    init_emulator();

    ui4x_config.model = opt_gx ? MODEL_48GX : MODEL_48SX;
    if ( ui4x_config.style_filename == NULL )
        ui4x_config.style_filename = opt_gx ? "style-48gx.css" : "style-48sx.css";

    ui4x_start();

    /* Give the Verilog core (if active) a chance to sync starting state. */
    sim_core_after_init();

    sigset_t set;
    struct sigaction sa;
    sigemptyset( &set );
    sigaddset( &set, SIGALRM );
    sa.sa_handler = signal_handler;
    sa.sa_mask = set;
#ifdef SA_RESTART
    sa.sa_flags = SA_RESTART;
#endif
    sigaction( SIGALRM, &sa, NULL );

    sigemptyset( &set );
    sigaddset( &set, SIGINT );
    sa.sa_handler = signal_handler;
    sa.sa_mask = set;
#ifdef SA_RESTART
    sa.sa_flags = SA_RESTART;
#endif
    sigaction( SIGINT, &sa, NULL );

    sigemptyset( &set );
    sigaddset( &set, SIGPIPE );
    sa.sa_handler = signal_handler;
    sa.sa_mask = set;
#ifdef SA_RESTART
    sa.sa_flags = SA_RESTART;
#endif
    sigaction( SIGPIPE, &sa, NULL );

    struct itimerval it;
    it.it_interval.tv_sec = 0;
    it.it_interval.tv_usec = USEC_PER_FRAME;
    it.it_value.tv_sec = it.it_interval.tv_sec;
    it.it_value.tv_usec = it.it_interval.tv_usec;
    setitimer( ITIMER_REAL, &it, NULL );

    long flags;
    flags = fcntl( STDIN_FILENO, F_GETFL, 0 );
    flags &= ~O_NDELAY;
    flags &= ~O_NONBLOCK;
    fcntl( STDIN_FILENO, F_SETFL, flags );

    struct timeval tv;
    struct timeval tv2;
    struct timezone tz;
    do {
        reset_timer( T1_TIMER );
        reset_timer( RUN_TIMER );
        reset_timer( IDLE_TIMER );

        set_accesstime();

        start_timer( T1_TIMER );
        start_timer( RUN_TIMER );

        sched_timer1 = t1_i_per_tick = saturn.t1_tick;
        sched_timer2 = t2_i_per_tick = saturn.t2_tick;

        set_t1 = saturn.timer1;

        do {
            step_instruction();   /* → linker-wrapped by step_dispatch.c */

            if ( x48ng_config.enable_debugger && ( exec_flags & EXEC_BKPT ) &&
                 check_breakpoint( BP_EXEC, saturn.pc ) ) {
                enter_debugger |= BREAKPOINT_HIT;
                break;
            }

            for ( int i = 0; i < KEYS_BUFFER_SIZE; i++ )
                if ( saturn.keybuf[ i ] || x48ng_config.throttle ) {
                    gettimeofday( &tv, &tz );
                    gettimeofday( &tv2, &tz );
                    while ( ( tv.tv_sec == tv2.tv_sec ) && ( ( tv.tv_usec - tv2.tv_usec ) < 2 ) )
                        gettimeofday( &tv, &tz );
                    break;
                }

            if ( schedule_event < 0 )
                schedule_event = 0;

            if ( schedule_event-- <= 0 )
                schedule();
        } while ( !enter_debugger );

        if ( enter_debugger )
            debug();
    } while ( true );

    ui4x_exit();
    stop_emulator();

    return 0;
}
