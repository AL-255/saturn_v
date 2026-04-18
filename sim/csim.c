/* csim.c — trace a Saturn program using x48ng's original emulate.c.
 *
 * Produces one-line-per-instruction traces identical in format to vsim.cpp,
 * so side-by-side diff verifies Verilog-vs-C equivalence.
 *
 * Usage:  csim <prog.hex> <steps> [trace_out.log]
 */
#include "harness.h"
#include "load_prog.h"
#include <stdio.h>
#include <stdlib.h>

extern void mem_write_nibbles(Address addr, const unsigned char *nibs, int count);

static void store_mem(int idx, unsigned char nib) {
    unsigned char nibs[1] = { nib };
    mem_write_nibbles((Address)idx, nibs, 1);
}

static void dump_state(FILE *tr, int step) {
    /* Build MSN-first 64-bit word from saturn.reg[x][0..15] */
    unsigned long long A=0, B=0, C=0, D=0;
    for (int i = 15; i >= 0; i--) {
        A = (A << 4) | (saturn.reg[0][i] & 0xf);
        B = (B << 4) | (saturn.reg[1][i] & 0xf);
        C = (C << 4) | (saturn.reg[2][i] & 0xf);
        D = (D << 4) | (saturn.reg[3][i] & 0xf);
    }
    /* ST nibbles → 4-bit mask {MP,SR,SB,XM} (bit 0 = XM) */
    int st4 = ((saturn.st[3] & 1) << 3) |
              ((saturn.st[2] & 1) << 2) |
              ((saturn.st[1] & 1) << 1) |
              ( saturn.st[0] & 1);
    /* PSTAT bit array → 16-bit (bit i = pstat[i]) */
    int pstat16 = 0;
    for (int i = 0; i < 16; i++)
        if (saturn.pstat[i]) pstat16 |= (1 << i);

    fprintf(tr,
        "STEP %3d PC=%05lX A=%016llX B=%016llX C=%016llX D=%016llX "
        "D0=%05lX D1=%05lX P=%X ST=%X PSTAT=%04X CARRY=%d HEX=%d\n",
        step, saturn.pc, A, B, C, D,
        saturn.d[0], saturn.d[1],
        saturn.p & 0xf, st4, pstat16,
        saturn.carry & 1, saturn.hexmode == HEX ? 1 : 0);
}

int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr, "usage: %s [--rom] <file> <steps> [trace.log]\n", argv[0]);
        return 1;
    }
    int ai = 1;
    int is_rom = 0;
    if (strcmp(argv[ai], "--rom") == 0) { is_rom = 1; ai++; }
    const char *prog = argv[ai++];
    int steps = atoi(argv[ai++]);
    const char *out = (argc > ai) ? argv[ai] : "trace_c.log";

    flat_memory_init();
    mem_clear();
    memset(&saturn, 0, sizeof(saturn));
    saturn.rstk_ptr = -1;
    saturn.hexmode  = HEX;

    int n = is_rom ? load_rom_bin(prog, store_mem) : load_prog(prog, store_mem);
    if (n < 0) return 2;
    fprintf(stderr, "[csim] loaded %d nibbles from %s (%s)\n",
            n, prog, is_rom ? "packed ROM" : "hex text");

    fprintf(stderr, "[csim] mem[0x80000..F]:");
    for (int k = 0; k < 16; k++) fprintf(stderr, " %X", bus_fetch_nibble(0x80000 + k));
    fprintf(stderr, "\n");

    FILE *tr = fopen(out, "w");
    if (!tr) { perror(out); return 3; }
    fprintf(tr, "# C-core (x48ng emulate.c) trace of %s\n", prog);

    for (int s = 0; s < steps; s++) {
        step_instruction();
        dump_state(tr, s);
        if ((s % 1000) == 999) {
            fflush(tr);
            fprintf(stderr, "[csim] step %d pc=%05lX\n", s+1, saturn.pc);
        }
    }
    fclose(tr);
    fprintf(stderr, "[csim] wrote %s (%d steps)\n", out, steps);
    return 0;
}
