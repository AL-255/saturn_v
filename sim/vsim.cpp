// vsim.cpp — Verilog saturn_cpu harness.
// Instantiates the Verilated model, loads a program, and traces one line
// per executed instruction in the same format as csim.c.
//
// Usage:  vsim <prog.hex> <steps> [trace_out.log]
//
// The saturn_cpu module's external interface is a simple nibble memory bus
// (mem_addr, mem_we, mem_re, mem_wdata, mem_rdata). This C++ driver owns the
// memory and services bus reads/writes every clock edge.

#include "Vsaturn_cpu.h"
#include "verilated.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cstdint>

extern "C" {
#include "load_prog.h"
}

#define MEM_SIZE (1u << 20)   // 1M nibbles, 20-bit address
static uint8_t mem[MEM_SIZE];

static void store_mem(int idx, unsigned char nib) {
    if ((uint32_t)idx < MEM_SIZE) mem[idx] = nib & 0xf;
}

// One clock period: falling + rising edge, with memory bus serviced.
// The saturn_cpu registers memory writes on the rising edge, and reads
// mem_rdata combinationally (driven by us from mem[mem_addr]).
static void tick(Vsaturn_cpu *top) {
    // negedge
    top->mem_rdata = mem[top->mem_addr & (MEM_SIZE - 1)];
    top->clk = 0;
    top->eval();
    // posedge
    top->mem_rdata = mem[top->mem_addr & (MEM_SIZE - 1)];
    top->clk = 1;
    top->eval();
    // After posedge, the CPU's NBAs commit. Check mem_we and perform the write.
    if (top->mem_we) {
        mem[top->mem_addr & (MEM_SIZE - 1)] = top->mem_wdata & 0xf;
    }
}

static void dump_state(FILE *tr, int step, Vsaturn_cpu *top) {
    fprintf(tr,
        "STEP %3d PC=%05X A=%016lX B=%016lX C=%016lX D=%016lX "
        "D0=%05X D1=%05X P=%X ST=%X PSTAT=%04X CARRY=%d HEX=%d\n",
        step,
        (unsigned)(top->dbg_pc & 0xFFFFF),
        (unsigned long)top->dbg_A,
        (unsigned long)top->dbg_B,
        (unsigned long)top->dbg_C,
        (unsigned long)top->dbg_D,
        (unsigned)(top->dbg_D0 & 0xFFFFF),
        (unsigned)(top->dbg_D1 & 0xFFFFF),
        (unsigned)(top->dbg_P & 0xf),
        (unsigned)(top->dbg_ST & 0xf),
        (unsigned)(top->dbg_PSTAT & 0xffff),
        (int)(top->dbg_CARRY & 1),
        (int)(top->dbg_HEXMODE & 1));
}

int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr, "usage: %s [--rom] <file> <steps> [trace.log]\n", argv[0]);
        return 1;
    }
    int ai = 1;
    int is_rom = 0;
    if (std::strcmp(argv[ai], "--rom") == 0) { is_rom = 1; ai++; }
    const char *prog = argv[ai++];
    int steps = atoi(argv[ai++]);
    const char *out = (argc > ai) ? argv[ai] : "trace_v.log";

    std::memset(mem, 0, sizeof(mem));
    int n = is_rom ? load_rom_bin(prog, store_mem) : load_prog(prog, store_mem);
    if (n < 0) return 2;
    fprintf(stderr, "[vsim] loaded %d nibbles from %s (%s)\n",
            n, prog, is_rom ? "packed ROM" : "hex text");

    VerilatedContext *ctx = new VerilatedContext;
    ctx->commandArgs(argc, argv);
    Vsaturn_cpu *top = new Vsaturn_cpu{ctx};

    // Initial state: reset low, step low
    top->clk = 0;
    top->rst_n = 0;
    top->step = 0;
    top->mem_rdata = 0;
    top->eval();
    // Hold reset for a few cycles
    for (int i = 0; i < 4; i++) tick(top);
    top->rst_n = 1;
    tick(top);

    FILE *tr = std::fopen(out, "w");
    if (!tr) { perror(out); return 3; }
    std::fprintf(tr, "# Verilog saturn_cpu trace of %s\n", prog);

    for (int s = 0; s < steps; s++) {
        // Pulse step for one cycle
        top->step = 1;
        tick(top);
        top->step = 0;

        // Run until step_done (or timeout)
        int guard = 0;
        while (!top->step_done && guard < 1000) {
            tick(top);
            guard++;
        }
        // Extra tick to let the state settle (S_DONE -> S_IDLE)
        tick(top);

        dump_state(tr, s, top);

        if (guard >= 1000) {
            std::fprintf(stderr, "[vsim] timeout at step %d\n", s);
            break;
        }
    }
    std::fclose(tr);
    std::fprintf(stderr, "[vsim] wrote %s (%d steps)\n", out, steps);

    delete top;
    delete ctx;
    return 0;
}
