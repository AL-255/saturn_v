// verilog_core.cpp — C++ shim that wraps the Verilated saturn_cpu model.
//
// Exposes a tiny C-callable API (see sim_core.h) so step_dispatch.c can drive
// it without pulling in Verilator headers. Each call to verilog_core_step_one
// copies saturn_t → Verilog regs, runs the CPU for exactly one instruction,
// and copies Verilog regs → saturn_t. The memory bus is serviced each clock
// by delegating to x48ng's bus_fetch_nibble / bus_write_nibble / read_nibble_crc.
//
// Built with --public-flat-rw so all internal signals are accessible via
// top->rootp->saturn_cpu__DOT__<sig>.

#include "Vsaturn_cpu.h"
#include "Vsaturn_cpu___024root.h"    // access internal state
#include "verilated.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

extern "C" {
    #include "core/emulate.h"
    #include "core/memory.h"
    #include "sim_core.h"
    unsigned long sim_step_num( void );
}

namespace {

VerilatedContext* g_ctx = nullptr;
Vsaturn_cpu*      g_top = nullptr;

// FSM state constants matching saturn_cpu.v
// (localparam S_IDLE=0 S_FETCH=1 S_EXEC=2 S_MEMR=3 S_MEMW=4 S_DONE=5)
static constexpr uint8_t S_FETCH = 1;
static constexpr uint8_t S_MEMR  = 3;

// A read during instruction fetch is semantically `bus_fetch_nibble` — no
// CRC side-effect. A read during DAT recall is `read_nibble_crc`. Using the
// wrong one silently advances saturn.crc and breaks lockstep against the
// C core (which reads mem[0x104..0x107] = saturn.crc as an MMIO register).
Nibble bus_read_for_state( uint8_t st, Address addr )
{
    Nibble v;
    if ( st == S_MEMR ) v = static_cast<Nibble>( read_nibble_crc( addr ) );
    else                v = bus_fetch_nibble( addr );
    if ( getenv( "VTRACE_MEM" ) && st == S_MEMR ) {
        auto* r = g_top->rootp;
        fprintf( stderr, "[vmem] pc=%05lX memr[%05X]=%X  mem_i=%u regC=%016lX\n",
                 (unsigned long)saturn.pc, addr, v,
                 (unsigned)r->saturn_cpu__DOT__mem_i,
                 (unsigned long)r->saturn_cpu__DOT__regC );
    }
    return v;
}

void tick()
{
    auto* r = g_top->rootp;
    uint8_t st = r->saturn_cpu__DOT__state;

    /* Service the bus read ONCE per cycle. Calling bus_read_for_state() on
     * both clk=0 and clk=1 evals double-ticks saturn.crc via calc_crc(),
     * so the ROM's MMIO reads of 0x104..0x107 (which return saturn.crc
     * nibbles) see corrupted values. mem_addr is a posedge-clocked reg, so
     * it's stable across the two evals within one cycle; one read is
     * sufficient. */
    g_top->mem_rdata = bus_read_for_state( st, r->saturn_cpu__DOT__mem_addr );
    g_top->clk = 0;
    g_top->eval();

    g_top->clk = 1;
    g_top->eval();

    if ( r->saturn_cpu__DOT__mem_we ) {
        Address waddr = r->saturn_cpu__DOT__mem_addr;
        Nibble  wdat  = r->saturn_cpu__DOT__mem_wdata & 0xf;
        bus_write_nibble( waddr, wdat );
        /* VTRACE_WR logging is centralized in step_dispatch.c's bus_write_nibble
         * wrapper so both RTL and C-fallback writes funnel through one place. */
    }
}

// Pack a 16-nibble register (saturn.reg[*]) into a 64-bit word where nibble 0
// sits at bits [3:0].
uint64_t pack_reg64( const unsigned char nib[ 16 ] )
{
    uint64_t v = 0;
    for ( int i = 15; i >= 0; i-- )
        v = ( v << 4 ) | ( nib[ i ] & 0xf );
    return v;
}

void unpack_reg64( uint64_t v, unsigned char nib[ 16 ] )
{
    for ( int i = 0; i < 16; i++ )
        nib[ i ] = ( v >> ( i * 4 ) ) & 0xf;
}

uint8_t pack_st4( const unsigned char s[ 4 ] )
{
    // s[0]=XM, s[1]=SB, s[2]=SR, s[3]=MP → ST reg bits 0..3
    return ( ( s[ 0 ] & 1 ) << 0 ) |
           ( ( s[ 1 ] & 1 ) << 1 ) |
           ( ( s[ 2 ] & 1 ) << 2 ) |
           ( ( s[ 3 ] & 1 ) << 3 );
}

void unpack_st4( uint8_t v, unsigned char s[ 4 ] )
{
    s[ 0 ] = ( v >> 0 ) & 1;
    s[ 1 ] = ( v >> 1 ) & 1;
    s[ 2 ] = ( v >> 2 ) & 1;
    s[ 3 ] = ( v >> 3 ) & 1;
}

uint16_t pack_pstat16( const unsigned char p[ 16 ] )
{
    uint16_t v = 0;
    for ( int i = 0; i < 16; i++ )
        if ( p[ i ] ) v |= uint16_t( 1u << i );
    return v;
}

void unpack_pstat16( uint16_t v, unsigned char p[ 16 ] )
{
    for ( int i = 0; i < 16; i++ ) p[ i ] = ( v >> i ) & 1;
}

} // namespace

extern "C" {

void verilog_core_reset( void )
{
    if ( !g_ctx ) {
        g_ctx = new VerilatedContext;
        g_top = new Vsaturn_cpu{ g_ctx };
    }
    g_top->clk = 0;
    g_top->rst_n = 0;
    g_top->step = 0;
    g_top->mem_rdata = 0;
    g_top->eval();
    for ( int i = 0; i < 4; i++ ) tick();
    g_top->rst_n = 1;
    tick();
}

void verilog_core_sync_from_saturn( void )
{
    if ( !g_top ) verilog_core_reset();
    auto* r = g_top->rootp;
    r->saturn_cpu__DOT__PC      = saturn.pc & 0xFFFFF;
    r->saturn_cpu__DOT__regA    = pack_reg64( saturn.reg[ 0 ] );
    r->saturn_cpu__DOT__regB    = pack_reg64( saturn.reg[ 1 ] );
    r->saturn_cpu__DOT__regC    = pack_reg64( saturn.reg[ 2 ] );
    r->saturn_cpu__DOT__regD    = pack_reg64( saturn.reg[ 3 ] );
    for ( int i = 0; i < 5; i++ )
        r->saturn_cpu__DOT__regR[ i ] = pack_reg64( saturn.reg_r[ i ] );
    r->saturn_cpu__DOT__D0      = saturn.d[ 0 ] & 0xFFFFF;
    r->saturn_cpu__DOT__D1      = saturn.d[ 1 ] & 0xFFFFF;
    r->saturn_cpu__DOT__P       = saturn.p & 0xF;
    r->saturn_cpu__DOT__ST      = pack_st4( saturn.st );
    r->saturn_cpu__DOT__PSTAT   = pack_pstat16( saturn.pstat );
    r->saturn_cpu__DOT__CARRY   = saturn.carry & 1;
    r->saturn_cpu__DOT__HEXMODE = ( saturn.hexmode == HEX ) ? 1 : 0;
    r->saturn_cpu__DOT__RSTKP   = ( saturn.rstk_ptr < 0 ) ? 0xF : ( saturn.rstk_ptr & 0xF );
    for ( int i = 0; i < 8; i++ )
        r->saturn_cpu__DOT__RSTK[ i ] = saturn.rstk[ i ] & 0xFFFFF;
    // IN / OUT — saturn.in[0..3] are 4 nibbles → low 16 bits; out[0..2] → low 12.
    uint64_t in = 0, out = 0;
    for ( int i = 3; i >= 0; i-- ) in = ( in << 4 ) | ( saturn.in[ i ] & 0xf );
    for ( int i = 2; i >= 0; i-- ) out = ( out << 4 ) | ( saturn.out[ i ] & 0xf );
    r->saturn_cpu__DOT__IN_REG  = in;
    r->saturn_cpu__DOT__OUT_REG = out;
}

void verilog_core_sync_to_saturn( void )
{
    auto* r = g_top->rootp;
    saturn.pc = r->saturn_cpu__DOT__PC & 0xFFFFF;
    unpack_reg64( r->saturn_cpu__DOT__regA, saturn.reg[ 0 ] );
    unpack_reg64( r->saturn_cpu__DOT__regB, saturn.reg[ 1 ] );
    unpack_reg64( r->saturn_cpu__DOT__regC, saturn.reg[ 2 ] );
    unpack_reg64( r->saturn_cpu__DOT__regD, saturn.reg[ 3 ] );
    for ( int i = 0; i < 5; i++ )
        unpack_reg64( r->saturn_cpu__DOT__regR[ i ], saturn.reg_r[ i ] );
    saturn.d[ 0 ] = r->saturn_cpu__DOT__D0 & 0xFFFFF;
    saturn.d[ 1 ] = r->saturn_cpu__DOT__D1 & 0xFFFFF;
    saturn.p = r->saturn_cpu__DOT__P & 0xF;
    unpack_st4( r->saturn_cpu__DOT__ST, saturn.st );
    unpack_pstat16( r->saturn_cpu__DOT__PSTAT, saturn.pstat );
    saturn.carry = r->saturn_cpu__DOT__CARRY & 1;
    saturn.hexmode = r->saturn_cpu__DOT__HEXMODE ? HEX : DEC;
    {
        uint8_t rp = r->saturn_cpu__DOT__RSTKP & 0xF;
        saturn.rstk_ptr = ( rp == 0xF ) ? -1 : rp;
    }
    for ( int i = 0; i < 8; i++ )
        saturn.rstk[ i ] = r->saturn_cpu__DOT__RSTK[ i ] & 0xFFFFF;
    // Propagate A=IN side-effect: update saturn.in to latest bus view too.
    uint64_t in = r->saturn_cpu__DOT__IN_REG;
    for ( int i = 0; i < 4; i++ ) saturn.in[ i ] = ( in >> ( i * 4 ) ) & 0xf;
}

void verilog_core_step_one( void )
{
    g_top->step = 1;
    tick();
    g_top->step = 0;
    int guard = 0;
    while ( !g_top->step_done && guard < 2000 ) {
        tick();
        guard++;
    }
    tick();    // let S_DONE → S_IDLE
    if ( guard >= 2000 )
        fprintf( stderr, "[verilog_core] step timeout (pc=%05lX)\n", (unsigned long)saturn.pc );
}

} // extern "C"
