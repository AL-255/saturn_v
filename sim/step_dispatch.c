/* step_dispatch.c — linker-wrap step_instruction so the core selector
 * decides whether to use the C reference or the Verilog RTL.
 *
 * Linked with -Wl,--wrap=step_instruction.
 *   __real_step_instruction()  = x48ng's original (emulate.c)
 *   __wrap_step_instruction()  = what every caller actually gets
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "core/emulate.h"
#include "core/memory.h"
#include "romio.h"      /* opt_gx */
#include "sim_core.h"

static sim_core_mode_t g_mode = CORE_C;
static unsigned long   g_step = 0;
static int             g_diverged = 0;

static FILE*         g_trace_fp   = NULL;
static unsigned long g_trace_max  = 0;

unsigned long sim_step_num( void ) { return g_step; }

/* Override x48ng's set_accesstime with a deterministic no-op-ish version.
 * Rationale: the real set_accesstime reads gettimeofday() and writes the
 * current wall-clock time directly into saturn.ram[ACCESSTIME..]. This
 * bypasses bus_write_nibble, so every run of x48_sim starts with different
 * RAM contents at 0x80058..0x80076. That makes --core=c vs --core=rtl
 * trace diffing useless — the traces diverge just from timestamps.
 *
 * We still need to write *something* there: the ROM checks the CRC and
 * panics (or runs a boot self-test) if the region is inconsistent. Writing
 * zeros for both time and crc satisfies the ROM. */
#define ACCESSTIME_GX_OFF  0x58
#define ACCESSCRC_GX_OFF   0x65
#define TIMEOUT_GX_OFF     0x69
#define TIMEOUTCLK_GX_OFF  0x76
void __real_set_accesstime( void );
void __wrap_set_accesstime( void )
{
    if ( !getenv( "SIM_DETERMINISTIC" ) ) { __real_set_accesstime(); return; }
    if ( !saturn.ram ) return;
    for ( int i = 0; i < 13; i++ ) saturn.ram[ ACCESSTIME_GX_OFF + i ] = 0;
    for ( int i = 0; i <  4; i++ ) saturn.ram[ ACCESSCRC_GX_OFF  + i ] = 0;
    for ( int i = 0; i < 13; i++ ) saturn.ram[ TIMEOUT_GX_OFF    + i ] = 0;
    saturn.ram[ TIMEOUTCLK_GX_OFF ] = 0xf;
}

void sim_enable_trace( const char* path, unsigned long max_steps )
{
    g_trace_fp  = fopen( path, "w" );
    if ( !g_trace_fp ) {
        fprintf( stderr, "[sim] cannot open trace %s\n", path );
        exit( 1 );
    }
    setvbuf( g_trace_fp, NULL, _IOLBF, 0 );
    g_trace_max = max_steps;
    fprintf( stderr, "[sim] trace → %s, stop at step %lu\n", path, max_steps );
}

static void emit_trace_line( void )
{
    if ( !g_trace_fp ) return;
    /* MSN-first register hex so it matches sim/csim / sim/vsim trace format. */
    char a[17], b[17], c[17], d[17];
    for ( int i = 0; i < 16; i++ ) {
        a[ 15 - i ] = "0123456789ABCDEF"[ saturn.reg[ 0 ][ i ] & 0xf ];
        b[ 15 - i ] = "0123456789ABCDEF"[ saturn.reg[ 1 ][ i ] & 0xf ];
        c[ 15 - i ] = "0123456789ABCDEF"[ saturn.reg[ 2 ][ i ] & 0xf ];
        d[ 15 - i ] = "0123456789ABCDEF"[ saturn.reg[ 3 ][ i ] & 0xf ];
    }
    a[ 16 ] = b[ 16 ] = c[ 16 ] = d[ 16 ] = 0;
    unsigned pstat = 0;
    for ( int i = 0; i < 16; i++ ) if ( saturn.pstat[ i ] ) pstat |= 1u << i;
    unsigned st = ( saturn.st[ 0 ] & 1 ) | ( ( saturn.st[ 1 ] & 1 ) << 1 ) |
                  ( ( saturn.st[ 2 ] & 1 ) << 2 ) | ( ( saturn.st[ 3 ] & 1 ) << 3 );
    fprintf( g_trace_fp,
             "STEP %lu PC=%05lX A=%s B=%s C=%s D=%s D0=%05lX D1=%05lX "
             "P=%X ST=%X PSTAT=%04X CARRY=%d HEX=%d\n",
             g_step, (unsigned long)saturn.pc, a, b, c, d,
             (unsigned long)saturn.d[ 0 ], (unsigned long)saturn.d[ 1 ],
             saturn.p & 0xf, st, pstat,
             saturn.carry & 1, ( saturn.hexmode == HEX ) ? 1 : 0 );
}

/* ── auto-keys: random key press/release every N instructions ────────── */
#define NB_HP48_KEYS 49    /* must match ui4x/src/api.h enum */
extern void press_key( int hpkey );
extern void release_key( int hpkey );

static unsigned      g_autokey_period = 0;   /* 0 = disabled */
static int           g_autokey_held   = -1;  /* which hpkey is currently down, -1 = none */
static unsigned long g_autokey_next   = 0;

void sim_enable_auto_keys( unsigned period )
{
    g_autokey_period = period;
    g_autokey_next   = period;
    /* deterministic pseudo-random sequence — reproducible bugs */
    srand( 0xC0FFEEUL );
}

static void autokey_tick( void )
{
    if ( g_autokey_period == 0 || g_step < g_autokey_next ) return;
    if ( g_autokey_held >= 0 ) {
        release_key( g_autokey_held );
        g_autokey_held = -1;
        g_autokey_next = g_step + g_autokey_period;
    } else {
        g_autokey_held = rand() % NB_HP48_KEYS;
        press_key( g_autokey_held );
        /* hold for ~10% of the period so the ROM's debounce sees it */
        g_autokey_next = g_step + ( g_autokey_period / 10 + 1 );
    }
}

/* Some group-8 I/O opcodes (OUT=C[S], A/C=IN, C=ID, CONFIG, UNCNFG, RESET,
 * BUSCx, SHUTDN, SREQ?, PC=(A)/PC=(C), INTON/INTOFF/RSI) depend on state
 * that lives only in saturn_t (keybuf, mem_cntl[], interruptable, ST[SR],
 * ...) and is not modeled in the Verilog CPU. For those instructions we
 * must run the C reference so the architectural state progresses
 * correctly; the RTL is a pc += N stub (or a stale-data read for A/C=IN).
 *
 * Returns non-zero if the instruction at saturn.pc should go through the C
 * core in rtl/lockstep modes. */
static int should_use_c_fallback( void )
{
    unsigned char n0 = bus_fetch_nibble( saturn.pc );
    /* Group 0, n1=0xF is RTI — jumps to interrupt vector (PC=0x0F) when
     * saturn.int_pending, else acts like RTN. The RTL always acts like RTN. */
    if ( n0 == 0x0 ) {
        unsigned char n1 = bus_fetch_nibble( ( saturn.pc + 1 ) & 0xFFFFF );
        return n1 == 0xF;
    }
    if ( n0 != 0x8 ) return 0;
    unsigned char n1 = bus_fetch_nibble( ( saturn.pc + 1 ) & 0xFFFFF );
    if ( n1 != 0 ) return 0;
    unsigned char n2 = bus_fetch_nibble( ( saturn.pc + 2 ) & 0xFFFFF );
    switch ( n2 ) {
        case 0x0:  /* OUT=CS */
        case 0x1:  /* OUT=C  */
        case 0x2:  /* A=IN — needs do_in() keyboard scan */
        case 0x3:  /* C=IN — needs do_in() keyboard scan */
        case 0x4:  /* UNCNFG */
        case 0x5:  /* CONFIG */
        case 0x6:  /* C=ID   */
        case 0x7:  /* SHUTDN */
        case 0x8:  /* 0808 subdispatch (INTON/RSI/LA/BUSCB/ABIT/CBIT/PC=(A)/BUSCD/PC=(C)/INTOFF) */
        case 0xA:  /* RESET  */
        case 0xB:  /* BUSCC  */
        case 0xE:  /* SREQ?  — clears ST[SR] alongside regC[0] */
            return 1;
        default:
            return 0;
    }
}

bool select_core( const char* name )
{
    if ( !name ) return false;
    if ( !strcmp( name, "c" ) )        g_mode = CORE_C;
    else if ( !strcmp( name, "rtl" ) ) g_mode = CORE_RTL;
    else if ( !strcmp( name, "lockstep" ) ) g_mode = CORE_LOCKSTEP;
    else return false;
    return true;
}

sim_core_mode_t current_core( void ) { return g_mode; }

/* Wrap bus_write_nibble so we can log C-core writes too (see VTRACE_WR env). */
static void ( *g_real_bus_write )( Address addr, Nibble val ) = NULL;
static void sim_bus_write_wrap( Address addr, Nibble val )
{
    g_real_bus_write( addr, val );
    if ( getenv( "VTRACE_WR" ) )
        fprintf( stderr, "[wr] step=%lu pc=%05lX  mem[%05X]<=%X\n",
                 g_step, (unsigned long)saturn.pc, addr, val );
}

void sim_core_after_init( void )
{
    /* x48ng's read_files() allocates saturn.ram via malloc() and then returns
     * 0 if the saved state file is missing; init_saturn() preserves the
     * pointer rather than zeroing its target. Result: saturn.ram contains
     * heap garbage and the emulator's behavior is non-deterministic across
     * runs. Zero it explicitly so trace comparison is meaningful. */
    {
        int ram_size = opt_gx ? 0x40000 : 0x10000;
        if ( saturn.ram ) memset( saturn.ram, 0, ram_size );
    }

    if ( g_mode == CORE_RTL || g_mode == CORE_LOCKSTEP ) {
        verilog_core_reset();
        verilog_core_sync_from_saturn();
    }
    if ( getenv( "VTRACE_WR" ) ) {
        /* Wrap in every mode so --core=rtl fallbacks and --core=c both log. */
        g_real_bus_write = bus_write_nibble;
        bus_write_nibble = sim_bus_write_wrap;
    }
    fprintf( stderr, "[sim] core = %s\n",
             g_mode == CORE_C ? "c" :
             g_mode == CORE_RTL ? "rtl" : "lockstep" );
}

/* Tiny copy of the bits of saturn_t we compare in lockstep. Memory writes
 * are not captured here — we rely on both cores performing identical
 * bus_write_nibble() calls, and the shared x48ng memory catches the
 * first-writer's update. In CORE_LOCKSTEP we always run C first, then V
 * with V's view of memory already matching. Any divergence in the
 * architectural state field below causes an abort before memory can
 * diverge further. */
typedef struct {
    unsigned long pc;
    unsigned char reg[ 4 ][ 16 ];
    unsigned char reg_r[ 5 ][ 16 ];
    unsigned long d[ 2 ];
    unsigned char p;
    unsigned char st[ 4 ];
    unsigned char pstat[ 16 ];
    unsigned char carry;
    unsigned char hex;
    short rstk_ptr;
    unsigned long rstk[ 8 ];
} snap_t;

static void snap_take_from( snap_t* s, const saturn_t* src )
{
    memset( s, 0, sizeof( *s ) );
    s->pc = src->pc;
    memcpy( s->reg,   src->reg,   sizeof( s->reg ) );
    memcpy( s->reg_r, src->reg_r, sizeof( s->reg_r ) );
    s->d[ 0 ] = src->d[ 0 ];
    s->d[ 1 ] = src->d[ 1 ];
    s->p = src->p;
    memcpy( s->st,    src->st,    sizeof( s->st ) );
    memcpy( s->pstat, src->pstat, sizeof( s->pstat ) );
    s->carry = src->carry;
    s->hex   = ( src->hexmode == HEX );
    s->rstk_ptr = src->rstk_ptr;
    memcpy( s->rstk, src->rstk, sizeof( s->rstk ) );
}

static void snap_take( snap_t* s ) { snap_take_from( s, &saturn ); }

static int snap_eq( const snap_t* a, const snap_t* b )
{
    if ( a->pc != b->pc ) return 0;
    if ( memcmp( a->reg,   b->reg,   sizeof( a->reg ) ) != 0 )   return 0;
    if ( memcmp( a->reg_r, b->reg_r, sizeof( a->reg_r ) ) != 0 ) return 0;
    if ( a->d[ 0 ] != b->d[ 0 ] ) return 0;
    if ( a->d[ 1 ] != b->d[ 1 ] ) return 0;
    if ( a->p != b->p ) return 0;
    if ( memcmp( a->st,    b->st,    sizeof( a->st ) ) != 0 )    return 0;
    if ( memcmp( a->pstat, b->pstat, sizeof( a->pstat ) ) != 0 ) return 0;
    if ( a->carry != b->carry ) return 0;
    if ( a->hex   != b->hex )   return 0;
    if ( a->rstk_ptr != b->rstk_ptr ) return 0;
    if ( memcmp( a->rstk, b->rstk, sizeof( a->rstk ) ) != 0 ) return 0;
    return 1;
}

static void snap_dump( const char* tag, const snap_t* s )
{
    fprintf( stderr, "  %s: pc=%05lX p=%X st=%X%X%X%X carry=%d hex=%d rstkp=%d\n",
             tag, s->pc, s->p, s->st[ 3 ], s->st[ 2 ], s->st[ 1 ], s->st[ 0 ],
             s->carry, s->hex, s->rstk_ptr );
    for ( int r = 0; r < 4; r++ ) {
        fprintf( stderr, "  %s: %c=", tag, "ABCD"[ r ] );
        for ( int i = 15; i >= 0; i-- ) fprintf( stderr, "%X", s->reg[ r ][ i ] & 0xf );
        fprintf( stderr, "\n" );
    }
    fprintf( stderr, "  %s: D0=%05lX D1=%05lX\n", tag, s->d[ 0 ], s->d[ 1 ] );
    for ( int r = 0; r < 5; r++ ) {
        fprintf( stderr, "  %s: R%d=", tag, r );
        for ( int i = 15; i >= 0; i-- ) fprintf( stderr, "%X", s->reg_r[ r ][ i ] & 0xf );
        fprintf( stderr, "\n" );
    }
    fprintf( stderr, "  %s: PSTAT=", tag );
    for ( int i = 15; i >= 0; i-- ) fprintf( stderr, "%d", s->pstat[ i ] & 1 );
    fprintf( stderr, "\n  %s: RSTK=", tag );
    for ( int i = 0; i < 8; i++ ) fprintf( stderr, "%05lX%s", s->rstk[ i ], i < 7 ? "," : "" );
    fprintf( stderr, "\n" );
}

static void snap_diff( const snap_t* a, const snap_t* b )
{
    if ( a->pc != b->pc ) fprintf( stderr, "    DIFF PC %05lX vs %05lX\n", a->pc, b->pc );
    for ( int r = 0; r < 4; r++ )
        if ( memcmp( a->reg[ r ], b->reg[ r ], 16 ) != 0 )
            fprintf( stderr, "    DIFF reg[%c]\n", "ABCD"[ r ] );
    for ( int r = 0; r < 5; r++ )
        if ( memcmp( a->reg_r[ r ], b->reg_r[ r ], 16 ) != 0 )
            fprintf( stderr, "    DIFF reg_r[%d]\n", r );
    if ( a->d[ 0 ] != b->d[ 0 ] ) fprintf( stderr, "    DIFF D0 %05lX vs %05lX\n", a->d[ 0 ], b->d[ 0 ] );
    if ( a->d[ 1 ] != b->d[ 1 ] ) fprintf( stderr, "    DIFF D1 %05lX vs %05lX\n", a->d[ 1 ], b->d[ 1 ] );
    if ( a->p != b->p ) fprintf( stderr, "    DIFF P %X vs %X\n", a->p, b->p );
    if ( memcmp( a->st, b->st, 4 ) != 0 ) fprintf( stderr, "    DIFF ST\n" );
    if ( memcmp( a->pstat, b->pstat, 16 ) != 0 ) fprintf( stderr, "    DIFF PSTAT\n" );
    if ( a->carry != b->carry ) fprintf( stderr, "    DIFF CARRY %d vs %d\n", a->carry, b->carry );
    if ( a->hex != b->hex ) fprintf( stderr, "    DIFF HEX %d vs %d\n", a->hex, b->hex );
    if ( a->rstk_ptr != b->rstk_ptr ) fprintf( stderr, "    DIFF RSTKP %d vs %d\n", a->rstk_ptr, b->rstk_ptr );
    if ( memcmp( a->rstk, b->rstk, sizeof( a->rstk ) ) != 0 ) fprintf( stderr, "    DIFF RSTK\n" );
}

void __real_step_instruction( void );   /* provided by linker --wrap */

void __wrap_step_instruction( void )
{
    switch ( g_mode ) {
        case CORE_C:
            __real_step_instruction();
            break;

        case CORE_RTL:
            if ( should_use_c_fallback() ) {
                __real_step_instruction();
            } else {
                verilog_core_sync_from_saturn();
                verilog_core_step_one();
                verilog_core_sync_to_saturn();
                extern unsigned long instructions;
                instructions++;   /* keep schedule() timer2 ticking */
            }
            break;

        case CORE_LOCKSTEP: {
            /* Opcodes the RTL doesn't model — run C only, skip comparison. */
            if ( should_use_c_fallback() ) {
                __real_step_instruction();
                break;
            }

            /* Snapshot the FULL saturn struct, not just CPU regs: MMIO reads
             * in memory.c have device-state side effects, so we need to
             * restore it bit-perfect before the V step gets to see memory. */
            static saturn_t pre;           /* static so we don't blow the stack */
            pre = saturn;

            __real_step_instruction();     /* C core advances saturn */

            snap_t cpu_c;
            snap_take( &cpu_c );

            static saturn_t post_c_full;
            post_c_full = saturn;

            saturn = pre;                  /* rewind every side-effect (device, crc, ...) */

            verilog_core_sync_from_saturn();
            verilog_core_step_one();
            verilog_core_sync_to_saturn();

            snap_t cpu_v;
            snap_take( &cpu_v );

            if ( !snap_eq( &cpu_c, &cpu_v ) ) {
                snap_t pre_snap; snap_take_from( &pre_snap, &pre );
                fprintf( stderr, "\n[LOCKSTEP DIVERGENCE at step %lu, pc_pre=%05lX]\n",
                         g_step, pre.pc );
                snap_dump( "pre  ", &pre_snap );
                snap_dump( "c-out", &cpu_c );
                snap_dump( "v-out", &cpu_v );
                snap_diff( &cpu_c, &cpu_v );
                g_diverged = 1;
                saturn = post_c_full;      /* keep C as canonical */
                exit( 1 );
            }

            saturn = post_c_full;          /* carry on with C's authoritative state */
            break;
        }
    }
    emit_trace_line();
    if ( getenv( "TRACE_RAM" ) ) {
        static unsigned char last[ 0x100 ] = { 0 };
        static int           init = 0;
        if ( !init ) {
            for ( Address a = 0; a < 0x100; a++ ) last[ a ] = bus_fetch_nibble( 0x80000 + a );
            init = 1;
        }
        for ( Address a = 0; a < 0x100; a++ ) {
            unsigned char cur = bus_fetch_nibble( 0x80000 + a );
            if ( cur != last[ a ] ) {
                fprintf( stderr, "[ramw] step=%lu pc=%05lX  mem[%05X] %X->%X\n",
                         g_step, (unsigned long)saturn.pc, 0x80000 + a, last[ a ], cur );
                last[ a ] = cur;
            }
        }
    }
    if ( getenv( "SNAP_STEP" ) && g_step == (unsigned long)atoi( getenv( "SNAP_STEP" ) ) ) {
        const char* snap = getenv( "SNAP_FILE" );
        if ( snap ) {
            FILE* fp = fopen( snap, "w" );
            if ( fp ) {
                fprintf( fp, "# mem_cntl: " );
                for ( int i = 0; i < 6; i++ )
                    fprintf( fp, "[%d]cfg0=%05X cfg1=%05X un=%d  ",
                             i, saturn.mem_cntl[ i ].config[ 0 ],
                             saturn.mem_cntl[ i ].config[ 1 ],
                             saturn.mem_cntl[ i ].unconfigured );
                fprintf( fp, "\n" );
                fprintf( fp, "# ram[0..0xFF] (raw saturn.ram): " );
                for ( int i = 0; i < 0x100; i++ )
                    fprintf( fp, "%X", saturn.ram ? ( saturn.ram[ i ] & 0xf ) : 0 );
                fprintf( fp, "\n" );
                for ( Address a = 0x80000; a < 0x80100; a++ )
                    fprintf( fp, "%05X %X\n", a, bus_fetch_nibble( a ) );
                fclose( fp );
                fprintf( stderr, "[snap] wrote %s at step %lu\n", snap, g_step );
            }
        }
    }
    g_step++;
    autokey_tick();
    if ( g_trace_max && g_step >= g_trace_max ) {
        if ( g_trace_fp ) { fflush( g_trace_fp ); fclose( g_trace_fp ); g_trace_fp = NULL; }
        fprintf( stderr, "[sim] reached max_steps=%lu, exiting\n", g_trace_max );
        exit( 0 );
    }
}
