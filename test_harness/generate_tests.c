/* generate_tests.c — Saturn CPU test-vector generator.
 *
 * Builds small instruction programs, runs them through the original
 * x48ng CPU core, and emits one test-case record per instruction step.
 *
 * Output format (one block per step):
 *   TEST <name>
 *   PRE  PC=XXXXX A=XXXXXXXXXXXXXXXX ... CARRY=X HEXMODE=X
 *   INST <hex nibbles of instruction bytes at that PC>
 *   POST PC=XXXXX A=XXXXXXXXXXXXXXXX ... CARRY=X HEXMODE=X
 *   END
 *
 * Nibble order: nibble[0] printed first (LSN), nibble[15] last (MSN).
 * A 20-bit address is printed as 5 hex digits, LSN first.
 */

#include "harness.h"
#include <stdlib.h>
#include <assert.h>

/* ──────────────────────────────────────────────
 * Printing helpers
 * ────────────────────────────────────────────── */

void print_reg64(FILE *f, const char *name, const Nibble *r) {
    fprintf(f, "%s=", name);
    for (int i = 0; i < 16; i++) fprintf(f, "%X", r[i] & 0xf);
}

static void print_addr5(FILE *f, const char *name, Address a) {
    /* LSN first, matching Saturn nibble-address layout */
    fprintf(f, "%s=", name);
    for (int i = 0; i < 5; i++) fprintf(f, "%X", (int)((a >> (i*4)) & 0xf));
}

void print_state(FILE *f, const char *tag) {
    fprintf(f, "%s ", tag);
    print_addr5(f, "PC", saturn.pc);
    fprintf(f, " ");
    print_reg64(f, "A", saturn.reg[A]);
    fprintf(f, " ");
    print_reg64(f, "B", saturn.reg[B]);
    fprintf(f, " ");
    print_reg64(f, "C", saturn.reg[C]);
    fprintf(f, " ");
    print_reg64(f, "D", saturn.reg[D]);
    fprintf(f, " ");
    print_addr5(f, "D0", saturn.d[0]);
    fprintf(f, " ");
    print_addr5(f, "D1", saturn.d[1]);
    fprintf(f, " P=%X", saturn.p & 0xf);
    fprintf(f, " ST=%X%X%X%X",
            saturn.st[XM]&1, saturn.st[SB]&1, saturn.st[SR]&1, saturn.st[MP]&1);
    /* pstat as 16-char bitstring */
    fprintf(f, " PSTAT=");
    for (int i = 0; i < 16; i++) fprintf(f, "%d", saturn.pstat[i] ? 1 : 0);
    fprintf(f, " CARRY=%d HEXMODE=%d", saturn.carry & 1, saturn.hexmode == HEX ? 1 : 0);
    /* Return stack */
    fprintf(f, " RSTK_PTR=%d", saturn.rstk_ptr);
    fprintf(f, " RSTK=");
    for (int i = 0; i <= 7; i++) {
        for (int j = 0; j < 5; j++) fprintf(f, "%X", (int)((saturn.rstk[i] >> (j*4)) & 0xf));
        if (i < 7) fprintf(f, ",");
    }
    /* R0..R4 scratch */
    for (int i = 0; i < 5; i++) {
        fprintf(f, " R%d=", i);
        for (int j = 0; j < 16; j++) fprintf(f, "%X", saturn.reg_r[i][j] & 0xf);
    }
    /* IN/OUT/HST/HEX counter */
    fprintf(f, " IN=");
    for (int j = 0; j < 4; j++) fprintf(f, "%X", saturn.in[j] & 0xf);
    fprintf(f, " OUT=");
    for (int j = 0; j < 3; j++) fprintf(f, "%X", saturn.out[j] & 0xf);
    fprintf(f, "\n");
}

/* Print up to 8 nibbles of the instruction at the given PC */
static void print_inst(FILE *f, Address pc) {
    fprintf(f, "INST");
    for (int i = 0; i < 8; i++) fprintf(f, " %X", (int)bus_fetch_nibble((pc + i) & 0xFFFFF));
    fprintf(f, "\n");
}

/* Reset saturn state to a known baseline */
static void reset_saturn(void) {
    memset(&saturn, 0, sizeof(saturn));
    saturn.rstk_ptr = -1;
    saturn.hexmode  = HEX;
    saturn.carry    = 0;
}

/* Set a 16-nibble register from a hex string (nibble[0] = hex[0]) */
static void set_reg(Nibble *r, const char *hex) {
    for (int i = 0; i < 16 && hex[i]; i++) {
        char c = hex[i];
        r[i] = (c >= '0' && c <= '9') ? c - '0'
             : (c >= 'a' && c <= 'f') ? c - 'a' + 10
             : c - 'A' + 10;
    }
}

/* Dump 16 nibbles of memory starting at addr as a "MEM addr hex16" line,
 * only if any nibble is non-zero and the region doesn't overlap the instruction. */
static void maybe_print_mem(FILE *f, Address addr, Address pc) {
    if (addr == 0 && pc == 0) return;
    if (addr >= pc && addr < (pc + 8)) return;
    unsigned char buf[16];
    mem_read_range(addr, buf, 16);
    int any = 0;
    for (int i = 0; i < 16; i++) if (buf[i]) { any = 1; break; }
    if (!any) return;
    fprintf(f, "MEM %05X ", addr);
    for (int i = 0; i < 16; i++) fprintf(f, "%X", buf[i]);
    fprintf(f, "\n");
}

/* Run one instruction, printing PRE/[MEM]/INST/POST */
static void step_and_print(FILE *f, const char *name) {
    fprintf(f, "TEST %s\n", name);
    print_state(f, "PRE ");
    /* dump any non-zero data at D0/D1 so the RTL tb can preload it */
    maybe_print_mem(f, saturn.d[0], saturn.pc);
    if (saturn.d[1] != saturn.d[0])
        maybe_print_mem(f, saturn.d[1], saturn.pc);
    print_inst(f, saturn.pc);
    step_instruction();
    print_state(f, "POST");
    fprintf(f, "END\n\n");
}

/* ──────────────────────────────────────────────
 * Helper: load a program and run n_steps, emitting a test per step.
 * prog_hex: nibbles of instruction stream placed at pc_start.
 * ────────────────────────────────────────────── */
void run_test(FILE *f, const char *name,
              Address pc_start, const char *prog_hex, int n_steps) {
    mem_write_hex(pc_start, prog_hex);
    saturn.pc = pc_start;
    for (int s = 0; s < n_steps; s++) {
        char tname[128];
        snprintf(tname, sizeof(tname), "%s_step%d", name, s);
        step_and_print(f, tname);
    }
}

/* ──────────────────────────────────────────────
 * Individual test suites
 * ────────────────────────────────────────────── */

/* ── Group 0x0C/0x0D/0x0E/0x0F: A-field (5-nibble) arithmetic ── */
static void test_cd_ef_group(FILE *f) {
    /* A=A+B  (0xC0) */
    reset_saturn();
    set_reg(saturn.reg[A], "1234500000000000");
    set_reg(saturn.reg[B], "ABCDE00000000000");
    mem_write_hex(0, "C0");
    saturn.pc = 0;
    step_and_print(f, "C0_AeqAplusB_Afield");

    /* A=A-B  (0xE0) */
    reset_saturn();
    set_reg(saturn.reg[A], "56789ABCDE000000");
    set_reg(saturn.reg[B], "12345ABCDE000000");
    mem_write_hex(0, "E0");
    saturn.pc = 0;
    step_and_print(f, "E0_AeqAminusB_Afield");

    /* A=A-B with borrow (0xE0) */
    reset_saturn();
    set_reg(saturn.reg[A], "00000ABCDE000000");
    set_reg(saturn.reg[B], "10000ABCDE000000");
    mem_write_hex(0, "E0");
    saturn.pc = 0;
    step_and_print(f, "E0_AeqAminusB_borrow");

    /* C=C+1  (0xE6) */
    reset_saturn();
    set_reg(saturn.reg[C], "FFFFF00000000000");
    mem_write_hex(0, "E6");
    saturn.pc = 0;
    step_and_print(f, "E6_CeqCinc_rollover");

    /* A=-A   2's complement (0xF8) */
    reset_saturn();
    set_reg(saturn.reg[A], "12300ABCDE000000");
    mem_write_hex(0, "F8");
    saturn.pc = 0;
    step_and_print(f, "F8_AeqNeg2A_Afield");

    /* A=-A-1 1's complement (0xFC) */
    reset_saturn();
    set_reg(saturn.reg[A], "12300ABCDE000000");
    mem_write_hex(0, "FC");
    saturn.pc = 0;
    step_and_print(f, "FC_AeqNeg1A_Afield");

    /* ASL shift left A-field (0xF0) */
    reset_saturn();
    set_reg(saturn.reg[A], "1234500000000000");
    mem_write_hex(0, "F0");
    saturn.pc = 0;
    step_and_print(f, "F0_ASL_Afield");

    /* ASR shift right A-field (0xF4) */
    reset_saturn();
    set_reg(saturn.reg[A], "ABCDE00000000000");
    mem_write_hex(0, "F4");
    saturn.pc = 0;
    step_and_print(f, "F4_ASR_Afield");

    /* A=0 (0xD0) */
    reset_saturn();
    set_reg(saturn.reg[A], "FEDCBA9876543210");
    mem_write_hex(0, "D0");
    saturn.pc = 0;
    step_and_print(f, "D0_AeqZero_Afield");

    /* ABEX exchange (0xDC) */
    reset_saturn();
    set_reg(saturn.reg[A], "1111100000000000");
    set_reg(saturn.reg[B], "AAAAA00000000000");
    mem_write_hex(0, "DC");
    saturn.pc = 0;
    step_and_print(f, "DC_ABEX_Afield");

    /* A=B copy (0xD4) */
    reset_saturn();
    set_reg(saturn.reg[A], "1111100000000000");
    set_reg(saturn.reg[B], "BBBBB00000000000");
    mem_write_hex(0, "D4");
    saturn.pc = 0;
    step_and_print(f, "D4_AeqB_Afield");
}

/* ── Group 0x0A/0x0B: arithmetic with field specifier ── */
static void test_ab_group(FILE *f) {
    /* A=A+B W-field (0xA 7 0): all 16 nibbles, hex mode */
    reset_saturn();
    set_reg(saturn.reg[A], "FEDCBA9876543210");
    set_reg(saturn.reg[B], "0000000000000001");
    mem_write_hex(0, "A70");   /* op=0xA, field=7(W), op2=0(A=A+B) */
    saturn.pc = 0;
    step_and_print(f, "A70_AeqAplusB_Wfield");

    /* A=A+B W-field with carry out */
    reset_saturn();
    set_reg(saturn.reg[A], "FFFFFFFFFFFFFFFF");
    set_reg(saturn.reg[B], "1000000000000000");
    mem_write_hex(0, "A70");
    saturn.pc = 0;
    step_and_print(f, "A70_AeqAplusB_carryout");

    /* B=B+C W-field (0xA 7 1) */
    reset_saturn();
    set_reg(saturn.reg[B], "0000000000000001");
    set_reg(saturn.reg[C], "FEDCBA9876543210");
    mem_write_hex(0, "A71");
    saturn.pc = 0;
    step_and_print(f, "A71_BeqBplusC_Wfield");

    /* A=A-1 W-field (0xA 7 C) */
    reset_saturn();
    set_reg(saturn.reg[A], "0000000000000000");
    mem_write_hex(0, "A7C");
    saturn.pc = 0;
    step_and_print(f, "A7C_AeqAdec_Wfield_borrow");

    /* A=A+B B-field (0xA 6 0): nibbles 0-1 only */
    reset_saturn();
    set_reg(saturn.reg[A], "FF00000000000000");
    set_reg(saturn.reg[B], "0100000000000000");
    mem_write_hex(0, "A60");
    saturn.pc = 0;
    step_and_print(f, "A60_AeqAplusB_Bfield");

    /* A=A+B P-field (0xA 0 0): nibble P only, P=3 */
    reset_saturn();
    saturn.p = 3;
    set_reg(saturn.reg[A], "0000F00000000000");
    set_reg(saturn.reg[B], "00001000000000000");  /* nibble[3] = 1 */
    /* B nibble[3] = '1' in string position 3 */
    set_reg(saturn.reg[B], "0001000000000000");
    mem_write_hex(0, "A00");
    saturn.pc = 0;
    step_and_print(f, "A00_AeqAplusB_Pfield_P3");

    /* A=A+B W-field DECIMAL mode */
    reset_saturn();
    saturn.hexmode = DEC;
    set_reg(saturn.reg[A], "9999900000000000");
    set_reg(saturn.reg[B], "1000000000000000");
    mem_write_hex(0, "A70");
    saturn.pc = 0;
    step_and_print(f, "A70_AeqAplusB_Wfield_DEC");

    /* ── 0x0B group (subtract and shift) ── */

    /* A=A-B W-field (0xB 7 0) */
    reset_saturn();
    set_reg(saturn.reg[A], "2000000000000000");
    set_reg(saturn.reg[B], "1000000000000000");
    mem_write_hex(0, "B70");
    saturn.pc = 0;
    step_and_print(f, "B70_AeqAminusB_Wfield");

    /* A=A+1 W-field (0xB 7 4) */
    reset_saturn();
    set_reg(saturn.reg[A], "0000000000000000");
    mem_write_hex(0, "B74");
    saturn.pc = 0;
    step_and_print(f, "B74_AeqAinc_Wfield");

    /* ASL W-field (0xB 8 0): shift left, top nibble lost */
    reset_saturn();
    set_reg(saturn.reg[A], "FEDCBA9876543210");
    mem_write_hex(0, "B80");
    saturn.pc = 0;
    step_and_print(f, "B80_ASL_Wfield");

    /* ASR W-field (0xB 8 4): shift right, SB flag */
    reset_saturn();
    set_reg(saturn.reg[A], "FEDCBA9876543211");
    mem_write_hex(0, "B84");
    saturn.pc = 0;
    step_and_print(f, "B84_ASR_Wfield_SB");

    /* A=-A 2's comp W-field (0xB 8 8) */
    reset_saturn();
    set_reg(saturn.reg[A], "1000000000000000");
    mem_write_hex(0, "B88");
    saturn.pc = 0;
    step_and_print(f, "B88_AeqNeg2A_Wfield");

    /* A=-A-1 1's comp W-field (0xB 8 C) */
    reset_saturn();
    set_reg(saturn.reg[A], "FEDCBA9876543210");
    mem_write_hex(0, "B8C");
    saturn.pc = 0;
    step_and_print(f, "B8C_AeqNeg1A_Wfield");

    /* A=0 W-field via 0xA 8 0 */
    reset_saturn();
    set_reg(saturn.reg[A], "FEDCBA9876543210");
    mem_write_hex(0, "A80");   /* op1=8(>=8 → zero/copy/exch), op2=0 (A=0, W) */
    saturn.pc = 0;
    step_and_print(f, "A80_AeqZero_Wfield");

    /* ABEX W-field (0xA 8 C) */
    reset_saturn();
    set_reg(saturn.reg[A], "1111111111111111");
    set_reg(saturn.reg[B], "AAAAAAAAAAAAAAAA");
    mem_write_hex(0, "A8C");
    saturn.pc = 0;
    step_and_print(f, "A8C_ABEX_Wfield");
}

/* ── Group 0x09: conditional tests with field specifier ── */
static void test_09_group(FILE *f) {
    /* ?A=B W-field, true → jump forward */
    reset_saturn();
    set_reg(saturn.reg[A], "AAAAAAAAAAAAAAAA");
    set_reg(saturn.reg[B], "AAAAAAAAAAAAAAAA");
    /* 0x09 op1=7(W) op2=0(?A=B) + 2-nibble offset 03 00 → offset=3 (jump 3 forward) */
    mem_write_hex(0x100, "97003");   /* instruction + offset 0x03 */
    saturn.pc = 0x100;
    step_and_print(f, "9700_AeqB_Wfield_true");

    /* ?A=B W-field, false → skip (pc+=5) */
    reset_saturn();
    set_reg(saturn.reg[A], "AAAAAAAAAAAAAAAA");
    set_reg(saturn.reg[B], "BBBBBBBBBBBBBBBB");
    mem_write_hex(0x100, "97003");
    saturn.pc = 0x100;
    step_and_print(f, "9700_AeqB_Wfield_false");

    /* ?A#0 B-field, true */
    reset_saturn();
    set_reg(saturn.reg[A], "1200000000000000");
    mem_write_hex(0x100, "96C03");   /* op1=6(B), op2=C(?A#0) offset 03 */
    saturn.pc = 0x100;
    step_and_print(f, "96C0_AneZero_Bfield_true");

    /* ?A#0 W-field, false */
    reset_saturn();
    set_reg(saturn.reg[A], "0000000000000000");
    mem_write_hex(0x100, "97C03");
    saturn.pc = 0x100;
    step_and_print(f, "97C0_AneZero_Wfield_false");

    /* ?A<B W-field (op1=0xF=7+8, op2=4) */
    reset_saturn();
    set_reg(saturn.reg[A], "1000000000000000");
    set_reg(saturn.reg[B], "2000000000000000");
    mem_write_hex(0x100, "9F403");   /* op1=F(7+8→compare), op2=4(?A<B) */
    saturn.pc = 0x100;
    step_and_print(f, "9F40_AltB_Wfield_true");

    /* ?A>B W-field (op1=F, op2=0) */
    reset_saturn();
    set_reg(saturn.reg[A], "2000000000000000");
    set_reg(saturn.reg[B], "1000000000000000");
    mem_write_hex(0x100, "9F003");
    saturn.pc = 0x100;
    step_and_print(f, "9F00_AgtB_Wfield_true");
}

/* ── Group 0x02: P=n ── */
static void test_02_group(FILE *f) {
    reset_saturn(); saturn.p = 0;
    mem_write_hex(0, "2A");   /* P = 0xA */
    saturn.pc = 0;
    step_and_print(f, "2A_PeqA");

    reset_saturn(); saturn.p = 5;
    mem_write_hex(0, "20");   /* P = 0 */
    saturn.pc = 0;
    step_and_print(f, "20_Peq0");
}

/* ── Group 0x03: LC load constant to C ── */
static void test_03_group(FILE *f) {
    /* LC(1): 0x03 0x00 0x7 — load 1 nibble (0x7) into C starting at P */
    reset_saturn(); saturn.p = 0;
    mem_write_hex(0, "3007");
    saturn.pc = 0;
    step_and_print(f, "3007_LC1_P0");

    /* LC(4): 0x03 0x03 ABCD — load 4 nibbles into C starting at P=2 */
    reset_saturn(); saturn.p = 2;
    mem_write_hex(0, "303ABCD");
    saturn.pc = 0;
    step_and_print(f, "303ABCD_LC4_P2");

    /* LC(4) correct: opcode=3, count=3 (→ n=4 nibbles), data=A,B,C,D at P=2 */
    reset_saturn(); saturn.p = 2;
    mem_write_hex(0, "33ABCD");   /* 3, count=3, data=A,B,C,D */
    saturn.pc = 0;
    step_and_print(f, "33ABCD_LC4_P2_correct");

    /* LC wraps: P=14, LC(3) nibbles wrap past 15 */
    reset_saturn(); saturn.p = 14;
    memset(saturn.reg[C], 0, 16);
    mem_write_hex(0, "32123");    /* 3, count=2 (n=3), data=1,2,3 */
    saturn.pc = 0;
    step_and_print(f, "32123_LC3_P14_wrap");
}

/* ── Group 0x04/0x05: JC / JNC ── */
static void test_jmp_group(FILE *f) {
    /* JC forward: carry=1, offset=3 (jumps 4 forward from opcode = after instruction) */
    reset_saturn(); saturn.carry = 1;
    mem_write_hex(0x200, "440");  /* 0x04 0x04 0x00: offset = 0x04 → pc = 0x200+4+1 = 0x205 */
    saturn.pc = 0x200;
    step_and_print(f, "4_JC_taken");

    /* JC not taken: carry=0, pc advances by 3 */
    reset_saturn(); saturn.carry = 0;
    mem_write_hex(0x200, "440");
    saturn.pc = 0x200;
    step_and_print(f, "4_JC_nottaken");

    /* JNC taken: carry=0 */
    reset_saturn(); saturn.carry = 0;
    mem_write_hex(0x200, "540");  /* JNC, offset=4 */
    saturn.pc = 0x200;
    step_and_print(f, "5_JNC_taken");

    /* JNC backward: offset = 0xFD = -3 signed, so pc goes back */
    reset_saturn(); saturn.carry = 0;
    mem_write_hex(0x200, "5FD");  /* 0x05 0x0F 0x0D: 2-nib LE = D|F<<4 = 0xFD = -3 */
    saturn.pc = 0x200;
    step_and_print(f, "5_JNC_backward");
}

/* ── Group 0x06/0x07: unconditional jump and GOSUB ── */
static void test_gosub_group(FILE *f) {
    /* JMP forward: 0x06 followed by 3-nibble offset */
    reset_saturn();
    /* offset 0x010 → pc = 0x200 + 0x010 + 1 = 0x211 */
    mem_write_hex(0x200, "6010");
    saturn.pc = 0x200;
    step_and_print(f, "6_JMP_forward");

    /* GOSUB: push pc+4, jump */
    reset_saturn(); saturn.rstk_ptr = -1;
    /* offset 0x010 → target = 0x300 + 0x010 + 4 = 0x314; return = 0x300+4=0x304 */
    mem_write_hex(0x300, "7010");
    saturn.pc = 0x300;
    step_and_print(f, "7_GOSUB_forward");
}

/* ── Group 0x00: control (RTN, SETHEX/SETDEC, P±1, CLRST, etc.) ── */
static void test_00_group(FILE *f) {
    /* RTN */
    reset_saturn(); saturn.rstk_ptr = 0; saturn.rstk[0] = 0x12345;
    mem_write_hex(0, "01");
    saturn.pc = 0;
    step_and_print(f, "001_RTN");

    /* RTNSC: return, set carry */
    reset_saturn(); saturn.rstk_ptr = 0; saturn.rstk[0] = 0xABCDE; saturn.carry = 0;
    mem_write_hex(0, "02");
    saturn.pc = 0;
    step_and_print(f, "002_RTNSC");

    /* RTNCC: return, clear carry */
    reset_saturn(); saturn.rstk_ptr = 0; saturn.rstk[0] = 0xABCDE; saturn.carry = 1;
    mem_write_hex(0, "03");
    saturn.pc = 0;
    step_and_print(f, "003_RTNCC");

    /* SETHEX */
    reset_saturn(); saturn.hexmode = DEC;
    mem_write_hex(0, "04");
    saturn.pc = 0;
    step_and_print(f, "004_SETHEX");

    /* SETDEC */
    reset_saturn(); saturn.hexmode = HEX;
    mem_write_hex(0, "05");
    saturn.pc = 0;
    step_and_print(f, "005_SETDEC");

    /* RSTK=C: push C as address onto return stack */
    reset_saturn(); saturn.rstk_ptr = -1;
    set_reg(saturn.reg[C], "ABCDE00000000000");
    mem_write_hex(0, "06");
    saturn.pc = 0;
    step_and_print(f, "006_RSTKeqC");

    /* C=RSTK: pop return stack into C */
    reset_saturn(); saturn.rstk_ptr = 0; saturn.rstk[0] = 0x12345;
    mem_write_hex(0, "07");
    saturn.pc = 0;
    step_and_print(f, "007_CeqRSTK");

    /* CLRST: clear pstat[0..11] */
    reset_saturn();
    for (int i = 0; i < 12; i++) saturn.pstat[i] = 1;
    mem_write_hex(0, "08");
    saturn.pc = 0;
    step_and_print(f, "008_CLRST");

    /* C=ST */
    reset_saturn();
    for (int i = 0; i < 12; i++) saturn.pstat[i] = (i & 1);
    mem_write_hex(0, "09");
    saturn.pc = 0;
    step_and_print(f, "009_CeqST");

    /* ST=C */
    reset_saturn();
    set_reg(saturn.reg[C], "A5A00000000000000");
    set_reg(saturn.reg[C], "A5A000000000000");  /* 16 nibbles, 0xA=1010, 0x5=0101, 0xA=1010 */
    mem_write_hex(0, "0A");
    saturn.pc = 0;
    step_and_print(f, "00A_STeqC");

    /* P=P+1 */
    reset_saturn(); saturn.p = 14;
    mem_write_hex(0, "0C");
    saturn.pc = 0;
    step_and_print(f, "00C_PplusPlus");

    /* P=P+1 rollover at 15 */
    reset_saturn(); saturn.p = 15;
    mem_write_hex(0, "0C");
    saturn.pc = 0;
    step_and_print(f, "00C_PplusPlus_rollover");

    /* P=P-1 */
    reset_saturn(); saturn.p = 1;
    mem_write_hex(0, "0D");
    saturn.pc = 0;
    step_and_print(f, "00D_PminusMinus");

    /* P=P-1 underflow at 0 */
    reset_saturn(); saturn.p = 0;
    mem_write_hex(0, "0D");
    saturn.pc = 0;
    step_and_print(f, "00D_PminusMinus_underflow");

    /* RTNSXM: return + set XM */
    reset_saturn(); saturn.rstk_ptr = 0; saturn.rstk[0] = 0x1000; saturn.st[XM] = 0;
    mem_write_hex(0, "00");
    saturn.pc = 0;
    step_and_print(f, "000_RTNSXM");
}

/* ── Group 0x00 0x0E: AND/OR with field ── */
static void test_00e_group(FILE *f) {
    /* A=A&B W-field (0x000E 7 0) */
    reset_saturn();
    set_reg(saturn.reg[A], "FFFFFFFFFFFFFFFF");
    set_reg(saturn.reg[B], "0F0F0F0F0F0F0F0F");
    mem_write_hex(0, "0E70");
    saturn.pc = 0;
    step_and_print(f, "0E70_AeqAandB_Wfield");

    /* A=A|B W-field (0x000E 7 8) */
    reset_saturn();
    set_reg(saturn.reg[A], "0F0F0F0F0F0F0F0F");
    set_reg(saturn.reg[B], "F0F0F0F0F0F0F0F0");
    mem_write_hex(0, "0E78");
    saturn.pc = 0;
    step_and_print(f, "0E78_AeqAorB_Wfield");

    /* C=C&A B-field (0x000E 6 2) */
    reset_saturn();
    set_reg(saturn.reg[C], "FF00000000000000");
    set_reg(saturn.reg[A], "F0F0000000000000");
    mem_write_hex(0, "0E62");
    saturn.pc = 0;
    step_and_print(f, "0E62_CeqCandA_Bfield");
}

/* ── Group 0x01: register/address operations ── */
static void test_01_group(FILE *f) {
    /* R0=A (0x010 0) */
    reset_saturn();
    set_reg(saturn.reg[A], "FEDCBA9876543210");
    mem_write_hex(0, "010 0");
    /* nibbles: 0,1,0,0 */
    mem_write_hex(0, "0100");
    saturn.pc = 0;
    step_and_print(f, "0100_R0eqA");

    /* A=R0 (0x011 0) */
    reset_saturn();
    set_reg(saturn.reg_r[0], "FEDCBA9876543210");
    mem_write_hex(0, "0110");
    saturn.pc = 0;
    step_and_print(f, "0110_AeqR0");

    /* AR0EX (0x012 0) */
    reset_saturn();
    set_reg(saturn.reg[A], "1111111111111111");
    set_reg(saturn.reg_r[0], "AAAAAAAAAAAAAAAA");
    mem_write_hex(0, "0120");
    saturn.pc = 0;
    step_and_print(f, "0120_AR0EX");

    /* D0=A (0x013 0) */
    reset_saturn();
    set_reg(saturn.reg[A], "12345ABCDE000000");
    mem_write_hex(0, "0130");
    saturn.pc = 0;
    step_and_print(f, "0130_D0eqA");

    /* D1=C (0x013 5) */
    reset_saturn();
    set_reg(saturn.reg[C], "ABCDE12345000000");
    mem_write_hex(0, "0135");
    saturn.pc = 0;
    step_and_print(f, "0135_D1eqC");

    /* AD0EX (0x013 2) */
    reset_saturn();
    set_reg(saturn.reg[A], "12345ABCDE000000");
    saturn.d[0] = 0xFEDCB;
    mem_write_hex(0, "0132");
    saturn.pc = 0;
    step_and_print(f, "0132_AD0EX");

    /* D0=D0+n: n+1=4, so D0+=4 (0x016 3) */
    reset_saturn(); saturn.d[0] = 0x10000;
    mem_write_hex(0, "0163");
    saturn.pc = 0;
    step_and_print(f, "0163_D0pluseq4");

    /* D0=D0-n: n+1=2, so D0-=2 (0x018 1) */
    reset_saturn(); saturn.d[0] = 0x10005;
    mem_write_hex(0, "0181");
    saturn.pc = 0;
    step_and_print(f, "0181_D0minuseq2");

    /* D0=(2) load 2-nibble address (0x019 XY) */
    reset_saturn(); saturn.d[0] = 0;
    mem_write_hex(0, "019AB");   /* load D0 = 0xBA (2 nibbles) */
    saturn.pc = 0;
    step_and_print(f, "019AB_D0load2");

    /* D0=(5) load 5-nibble address (0x01B XXXXX) */
    reset_saturn(); saturn.d[0] = 0;
    mem_write_hex(0, "01BABCDE");  /* nibbles: 0,1,B + 5 nibbles A,B,C,D,E */
    mem_write_hex(0, "01BABCDE");
    saturn.pc = 0;
    step_and_print(f, "01B_D0load5");
}

/* ── Group 0x01 0x4/0x5: DAT memory operations ── */
static void test_dat_group(FILE *f) {
    /* DAT0=A W-field (0x014 7): store A[0..15] to D0 */
    reset_saturn();
    set_reg(saturn.reg[A], "FEDCBA9876543210");
    saturn.d[0] = 0x10000;
    mem_write_hex(0, "0147");
    saturn.pc = 0;
    step_and_print(f, "0147_DAT0eqA_Wfield");
    /* verify: memory at 0x10000..0x1000F should be F,E,D,...,0 */
    fprintf(f, "# MEM[10000..1000F]=");
    for (int i = 0; i < 16; i++)
        fprintf(f, "%X", (int)bus_fetch_nibble(0x10000 + i));
    fprintf(f, "\n\n");

    /* A=DAT0 W-field (0x014 2): recall A from D0 */
    reset_saturn();
    saturn.d[0] = 0x10000;
    /* pre-load memory */
    mem_write_hex(0x10000, "0123456789ABCDEF");
    set_reg(saturn.reg[A], "0000000000000000");
    mem_write_hex(0, "0142");
    saturn.pc = 0;
    step_and_print(f, "0142_AeqDAT0_Wfield");

    /* DAT0=A A-field (0x014 0): store A[0..4] to D0 */
    reset_saturn();
    set_reg(saturn.reg[A], "12345ABCDE000000");
    saturn.d[0] = 0x20000;
    mem_write_hex(0, "0140");
    saturn.pc = 0;
    step_and_print(f, "0140_DAT0eqA_Afield");
    fprintf(f, "# MEM[20000..20004]=");
    for (int i = 0; i < 5; i++)
        fprintf(f, "%X", (int)bus_fetch_nibble(0x20000 + i));
    fprintf(f, "\n\n");

    /* DAT1=C B-field (0x014 D): store C[0..1] to D1 */
    reset_saturn();
    set_reg(saturn.reg[C], "ABCDE12345FEDCBA");
    saturn.d[1] = 0x30000;
    mem_write_hex(0, "014D");
    saturn.pc = 0;
    step_and_print(f, "014D_DAT1eqC_Bfield");

    /* C=DAT1 W-field (0x014 E): recall C from D1 */
    reset_saturn();
    saturn.d[1] = 0x30000;
    mem_write_hex(0x30000, "FEDCBA9876543210");
    mem_write_hex(0, "014E");
    saturn.pc = 0;
    step_and_print(f, "014E_CeqDAT1_Wfield");

    /* DAT0=A n+1=3 (0x015 8 2): store 3 nibbles of A to D0 */
    reset_saturn();
    set_reg(saturn.reg[A], "ABCDE00000000000");
    saturn.d[0] = 0x40000;
    mem_write_hex(0, "01582");   /* op=8(store A, variable len), len-nib=2 → 3 nibbles */
    saturn.pc = 0;
    step_and_print(f, "01582_DAT0eqA_n3");
}

/* ── Group 0x08: misc I/O and jumps ── */
static void test_08_group(FILE *f) {
    /* OUT=C (0x0801) */
    reset_saturn();
    set_reg(saturn.reg[C], "ABC0000000000000");
    mem_write_hex(0, "081");
    saturn.pc = 0;
    step_and_print(f, "081_OUTeqC");

    /* 0x0804 PSB=0 n=5 */
    reset_saturn();
    for (int i=0;i<16;i++) saturn.pstat[i]=1;
    mem_write_hex(0, "0845");
    saturn.pc = 0;
    step_and_print(f, "0845_PSBclr5");

    /* 0x0805 PSB=1 n=3 */
    reset_saturn();
    saturn.pstat[3] = 0;
    mem_write_hex(0, "0853");
    saturn.pc = 0;
    step_and_print(f, "0853_PSBset3");

    /* 0x0882 ST XM clear */
    reset_saturn();
    saturn.st[XM]=1; saturn.st[SB]=1; saturn.st[SR]=0; saturn.st[MP]=0;
    mem_write_hex(0, "0821");  /* mask=1 → clear XM */
    saturn.pc = 0;
    step_and_print(f, "0821_CLRSTmask_XM");

    /* 0x08C relative GOTO: 4-nibble offset */
    reset_saturn();
    /* offset 0x0004 → pc = 0x500 + 0x0004 + 2 = 0x506 */
    mem_write_hex(0x500, "08C4000");
    saturn.pc = 0x500;
    step_and_print(f, "08C_GOTO_rel");

    /* 0x08D absolute GOTO: 5-nibble address */
    reset_saturn();
    mem_write_hex(0x500, "08D12345");  /* jump to 0x54321 */
    saturn.pc = 0x500;
    step_and_print(f, "08D_GOTO_abs");

    /* 0x08E relative GOSUB */
    reset_saturn(); saturn.rstk_ptr = -1;
    /* offset 0x0004 → target = 0x600 + 0x0004 + 6 = 0x60A; return = 0x606 */
    mem_write_hex(0x600, "08E4000");
    saturn.pc = 0x600;
    step_and_print(f, "08E_GOSUB_rel");

    /* 0x08F absolute GOSUB */
    reset_saturn(); saturn.rstk_ptr = -1;
    mem_write_hex(0x600, "08FABCDE");  /* GOSUB to 0xEDCBA; return = 0x607 */
    saturn.pc = 0x600;
    step_and_print(f, "08F_GOSUB_abs");

    /* 0x080C C=P n: C[3] = P */
    reset_saturn(); saturn.p = 7;
    mem_write_hex(0, "080C3");
    saturn.pc = 0;
    step_and_print(f, "080C3_CeqPn");

    /* 0x080D P=C n: P = C[2] */
    reset_saturn(); saturn.p = 0;
    set_reg(saturn.reg[C], "00A000000000000"); /* C[2]=A */
    mem_write_hex(0, "080D2");
    saturn.pc = 0;
    step_and_print(f, "080D2_PeqCn");

    /* 0x0809 C+P+1: add P+1 to C[0..4] */
    reset_saturn(); saturn.p = 4;
    set_reg(saturn.reg[C], "FFFFF00000000000");
    mem_write_hex(0, "089");
    saturn.pc = 0;
    step_and_print(f, "089_CplusP1");
}

/* ── Group 0x08A/?= comparisons (A-field) with jump ── */
static void test_08a_group(FILE *f) {
    /* ?A=B A-field, true */
    reset_saturn();
    set_reg(saturn.reg[A], "12345FFFFFFFFFFF");
    set_reg(saturn.reg[B], "12345FFFFFFFFFFF");
    /* 0x08A 0 + 2-nibble offset 03 */
    mem_write_hex(0x700, "08A003");
    saturn.pc = 0x700;
    step_and_print(f, "08A0_AeqB_Afield_true");

    /* ?C#D A-field, false */
    reset_saturn();
    set_reg(saturn.reg[C], "12345FFFFFFFFFFF");
    set_reg(saturn.reg[D], "12346FFFFFFFFFFF");
    mem_write_hex(0x700, "08A703");
    saturn.pc = 0x700;
    step_and_print(f, "08A7_CneD_Afield_false");
}

/* ── Group 0x08 1: shift circular + SRB ── */
static void test_081_group(FILE *f) {
    /* ASLC (0x0810) */
    reset_saturn();
    set_reg(saturn.reg[A], "FEDCBA9876543210");
    mem_write_hex(0, "081 0");
    mem_write_hex(0, "0810");
    saturn.pc = 0;
    step_and_print(f, "0810_ASLC");

    /* ASRC (0x0814) */
    reset_saturn();
    set_reg(saturn.reg[A], "FEDCBA9876543210");
    mem_write_hex(0, "0814");
    saturn.pc = 0;
    step_and_print(f, "0814_ASRC");

    /* ASRB W-field (0x081C) */
    reset_saturn();
    set_reg(saturn.reg[A], "FEDCBA9876543211");
    mem_write_hex(0, "081C");
    saturn.pc = 0;
    step_and_print(f, "081C_ASRB_Wfield");

    /* PC=A (0x081B 2) */
    reset_saturn();
    set_reg(saturn.reg[A], "ABCDE00000000000");
    mem_write_hex(0, "081B2");
    saturn.pc = 0;
    step_and_print(f, "081B2_PCeqA");

    /* A=PC (0x081B 4): stores current PC into A */
    reset_saturn();
    mem_write_hex(0, "081B4");
    saturn.pc = 0;
    step_and_print(f, "081B4_AeqPC");
}

/* ── Interrupt / RTI ── */
static void test_interrupt(FILE *f) {
    /* do_interupt then RTI */
    reset_saturn();
    saturn.interruptable = 1;
    saturn.pc = 0x5000;
    /* place RTI at 0x000F */
    mem_write_hex(0x000F, "0F");
    do_interupt();
    /* cpu should now be at 0x000F with return on stack */
    step_and_print(f, "000F_RTI_after_interupt");
}

/* ── Decimal mode arithmetic ── */
static void test_decimal(FILE *f) {
    /* A=A+B W-field decimal */
    reset_saturn();
    saturn.hexmode = DEC;
    set_reg(saturn.reg[A], "9876543210000000");
    set_reg(saturn.reg[B], "1234567890000000");
    mem_write_hex(0, "A70");
    saturn.pc = 0;
    step_and_print(f, "A70_AeqAplusB_DEC_carry");

    /* A=A-B W-field decimal with borrow */
    reset_saturn();
    saturn.hexmode = DEC;
    set_reg(saturn.reg[A], "0000000001000000");
    set_reg(saturn.reg[B], "1000000001000000");
    mem_write_hex(0, "B70");
    saturn.pc = 0;
    step_and_print(f, "B70_AeqAminusB_DEC_borrow");
}

/* ── CSTEX, full status register exchange ── */
static void test_status(FILE *f) {
    reset_saturn();
    for (int i = 0; i < 12; i++) saturn.pstat[i] = (i & 1);
    set_reg(saturn.reg[C], "A5A000000000000");
    mem_write_hex(0, "0B");  /* CSTEX */
    saturn.pc = 0;
    step_and_print(f, "00B_CSTEX");
}

/* ── Additional field specifier coverage ── */
static void test_field_specs(FILE *f) {
    /* M-field (code=5): nibbles 3..14 */
    reset_saturn();
    set_reg(saturn.reg[A], "000FFFFFFFFFFFF0");   /* M nibbles all F */
    set_reg(saturn.reg[B], "0001000000000010");   /* M nibble[3]=1 */
    mem_write_hex(0, "A50");   /* A=A+B, field=5 (M) */
    saturn.pc = 0;
    step_and_print(f, "A50_AeqAplusB_Mfield");

    /* S-field (code=4): nibble 15 only */
    reset_saturn();
    set_reg(saturn.reg[A], "000000000000000F");   /* A[15]=F */
    set_reg(saturn.reg[B], "0000000000000002");   /* B[15]=2 */
    mem_write_hex(0, "A40");   /* A=A+B, field=4 (S) */
    saturn.pc = 0;
    step_and_print(f, "A40_AeqAplusB_Sfield_carry");

    /* XS-field (code=2): nibble 2 only */
    reset_saturn();
    set_reg(saturn.reg[A], "00F0000000000000");   /* A[2]=F */
    set_reg(saturn.reg[B], "0010000000000000");   /* B[2]=1 */
    mem_write_hex(0, "A20");   /* A=A+B, field=2 (XS) */
    saturn.pc = 0;
    step_and_print(f, "A20_AeqAplusB_XSfield_carry");

    /* WP-field (code=1): nibbles 0..P, P=4 */
    reset_saturn(); saturn.p = 4;
    set_reg(saturn.reg[A], "FFFFF00000000000");   /* A[0..4]=F */
    set_reg(saturn.reg[B], "10000000000000000");  /* too long, use 16 chars */
    set_reg(saturn.reg[B], "1000000000000000");   /* B[0]=1 */
    mem_write_hex(0, "A10");   /* A=A+B, field=1 (WP), P=4 */
    saturn.pc = 0;
    step_and_print(f, "A10_AeqAplusB_WPfield_P4");

    /* X-field (code=3): nibbles 0..2 only */
    reset_saturn();
    set_reg(saturn.reg[C], "FFF0000000000000");   /* C[0..2]=F */
    set_reg(saturn.reg[A], "1000000000000000");   /* A[0]=1 */
    mem_write_hex(0, "A32");   /* C=C+A, field=3 (X), op=2 */
    saturn.pc = 0;
    step_and_print(f, "A32_CeqCplusA_Xfield");

    /* Zero S-field (nibble 15 only) */
    reset_saturn();
    set_reg(saturn.reg[A], "FEDCBA9876543218");
    mem_write_hex(0, "A84");   /* A=0, field=4 (S) */
    saturn.pc = 0;
    step_and_print(f, "A84_AeqZero_Sfield");

    /* ?A=0 M-field (nibbles 3..14), zero check */
    reset_saturn();
    set_reg(saturn.reg[A], "FFF0000000000FFF");   /* M nibbles all 0 */
    mem_write_hex(0x100, "958" "03");   /* op1=5(M), op2=8(?A=0), offset=3 */
    saturn.pc = 0x100;
    step_and_print(f, "958_AeqZero_Mfield_true");

    /* C=C+1 WP-field (P=2, nibbles 0..2) carry chain */
    reset_saturn(); saturn.p = 2;
    set_reg(saturn.reg[C], "FFF0000000000000");   /* C[0..2]=F */
    mem_write_hex(0, "B16");   /* C=C+1, field=1 (WP) */
    saturn.pc = 0;
    step_and_print(f, "B16_CeqCinc_WPfield_P2_carry");

    /* B=-B 1's complement S-field (nibble 15) */
    reset_saturn();
    set_reg(saturn.reg[B], "000000000000000A");
    mem_write_hex(0, "B4D");   /* B=-B-1, field=4(S), op=D */
    saturn.pc = 0;
    step_and_print(f, "B4D_BeqNeg1B_Sfield");
}

/* ── DAT operations with address pointer auto-advance ── */
static void test_dat_addr_advance(FILE *f) {
    /* A=DAT0 W-field: recall all 16 nibbles from D0 into A.
     * Encoding: 1,4,2 (grp1 sub-04, op3=2=A=DAT0, opX=0xf=W), PC+=3 */
    reset_saturn();
    mem_write_hex(0x50000, "ABCDE");   /* 5 nibbles at 0x50000; rest are 0 */
    saturn.d[0] = 0x50000;
    mem_write_hex(0, "142");           /* A=DAT0 W-field */
    saturn.pc = 0;
    step_and_print(f, "142_AeqDAT0_Wfield");
}

/* ── CPEX: exchange C nibble n with P ── */
static void test_cpex(FILE *f) {
    reset_saturn(); saturn.p = 7;
    set_reg(saturn.reg[C], "00000003000000000");  /* too long */
    set_reg(saturn.reg[C], "0000000300000000");   /* C[7]=3 */
    mem_write_hex(0, "80F7");   /* CPEX n=7: grp8 sub-0 case-F n=7, swap C[7] with P */
    saturn.pc = 0;
    step_and_print(f, "80F7_CPEX_n7");
}

/* ──────────────────────────────────────────────
 * Comprehensive coverage — every opcode family exercised at least once.
 * Named test_full_* to stay separate from the earlier suites.
 * ────────────────────────────────────────────── */
static void test_full_group1_r_regs(FILE *f) {
    /* 10x: R{n}=A (n=0..4) and R{n}=C (n=8..12) */
    for (int i = 0; i < 5; i++) {
        reset_saturn();
        set_reg(saturn.reg[A], "123456789ABCDEF0");
        char buf[8]; snprintf(buf, sizeof(buf), "10%X", i);
        mem_write_hex(0, buf);
        saturn.pc = 0;
        char tn[32]; snprintf(tn, sizeof(tn), "10%X_R%d_eq_A", i, i);
        step_and_print(f, tn);
    }
    for (int i = 0; i < 5; i++) {
        reset_saturn();
        set_reg(saturn.reg[C], "FEDCBA9876543210");
        char buf[8]; snprintf(buf, sizeof(buf), "10%X", 8+i);
        mem_write_hex(0, buf);
        saturn.pc = 0;
        char tn[32]; snprintf(tn, sizeof(tn), "10%X_R%d_eq_C", 8+i, i);
        step_and_print(f, tn);
    }
    /* 11x: A=R{n} / C=R{n} */
    for (int i = 0; i < 5; i++) {
        reset_saturn();
        set_reg(saturn.reg_r[i], "AAAAAAAAAAAAAAAA");
        char buf[8]; snprintf(buf, sizeof(buf), "11%X", i);
        mem_write_hex(0, buf);
        saturn.pc = 0;
        char tn[32]; snprintf(tn, sizeof(tn), "11%X_A_eq_R%d", i, i);
        step_and_print(f, tn);
    }
    /* 12x: AR{n}EX / CR{n}EX */
    for (int i = 0; i < 5; i++) {
        reset_saturn();
        set_reg(saturn.reg[A],   "1111111111111111");
        set_reg(saturn.reg_r[i], "2222222222222222");
        char buf[8]; snprintf(buf, sizeof(buf), "12%X", i);
        mem_write_hex(0, buf);
        saturn.pc = 0;
        char tn[32]; snprintf(tn, sizeof(tn), "12%X_AR%dEX", i, i);
        step_and_print(f, tn);
    }
}

static void test_full_group13(FILE *f) {
    /* 13 0..F : D0/D1 assignments and exchanges */
    for (int op = 0; op <= 0xF; op++) {
        reset_saturn();
        set_reg(saturn.reg[A], "12345ABCDEF01234");
        set_reg(saturn.reg[C], "FEDCBA9876543210");
        saturn.d[0] = 0xAABBC;
        saturn.d[1] = 0x11223;
        char buf[8]; snprintf(buf, sizeof(buf), "13%X", op);
        mem_write_hex(0, buf);
        saturn.pc = 0;
        char tn[48]; snprintf(tn, sizeof(tn), "13%X_dreg_op", op);
        step_and_print(f, tn);
    }
}

static void test_full_group0e_andor(FILE *f) {
    /* 0E fs op : AND and OR across all 16 ops at fs=W(7) */
    for (int op = 0; op <= 0xF; op++) {
        reset_saturn();
        set_reg(saturn.reg[A], "1111111111111111");
        set_reg(saturn.reg[B], "2222222222222222");
        set_reg(saturn.reg[C], "3333333333333333");
        set_reg(saturn.reg[D], "4444444444444444");
        char buf[8]; snprintf(buf, sizeof(buf), "0E7%X", op);
        mem_write_hex(0, buf);
        saturn.pc = 0;
        char tn[48]; snprintf(tn, sizeof(tn), "0E7%X_andor_W", op);
        step_and_print(f, tn);
    }
}

static void test_full_group8_branches(FILE *f) {
    /* 83x ?ST mask */
    reset_saturn();
    saturn.st[XM]=1;
    mem_write_hex(0, "8301" "03");      /* ?ST=1 mask=1 (XM set) → branch */
    saturn.pc = 0;
    step_and_print(f, "8301_STmask_XM_branch");

    /* 86n ?ST=0 n + branch */
    reset_saturn();
    /* pstat[5]=0 → ?ST=0 5 → carry=1 → branch */
    mem_write_hex(0, "865" "03");
    saturn.pc = 0;
    step_and_print(f, "865_PST5_zero_branch");

    /* 87n ?ST=1 n + branch */
    reset_saturn();
    saturn.pstat[5] = 1;
    mem_write_hex(0, "875" "03");
    saturn.pc = 0;
    step_and_print(f, "875_PST5_one_branch");

    /* 88n ?P#n + branch (taken when p != n) */
    reset_saturn();
    saturn.p = 3;
    mem_write_hex(0, "885" "03");      /* 3 != 5 → branch */
    saturn.pc = 0;
    step_and_print(f, "885_Pne5_branch");

    /* 89n ?P=n + branch */
    reset_saturn();
    saturn.p = 4;
    mem_write_hex(0, "894" "03");
    saturn.pc = 0;
    step_and_print(f, "894_Peq4_branch");

    /* 8A0 ?A=B A-field + branch */
    reset_saturn();
    set_reg(saturn.reg[A], "12345AAAAAAAAAAA");
    set_reg(saturn.reg[B], "12345BBBBBBBBBBB");
    mem_write_hex(0, "8A0" "03");      /* A-field equal → branch */
    saturn.pc = 0;
    step_and_print(f, "8A0_AeqB_Afield_branch");

    /* 8B0 ?A>B A-field + branch */
    reset_saturn();
    set_reg(saturn.reg[A], "99999000000000000");
    set_reg(saturn.reg[B], "11111000000000000");
    mem_write_hex(0, "8B0" "03");
    saturn.pc = 0;
    step_and_print(f, "8B0_AgtB_Afield_branch");

    /* 8C GOTO_LONG (4-nib signed rel) — forward jump */
    reset_saturn();
    mem_write_hex(0, "8C" "0A00");     /* disp = 0x000A → pc = pc + 0x000A + 2 = 0x0C */
    saturn.pc = 0;
    step_and_print(f, "8C_GOTOLONG_fwd");

    /* 8D GOTO_ABS (5-nib absolute) */
    reset_saturn();
    mem_write_hex(0, "8D" "ABCDE");
    saturn.pc = 0;
    step_and_print(f, "8D_GOTOABS");

    /* 8E GOSUB_LONG */
    reset_saturn();
    mem_write_hex(0, "8E" "0800");     /* disp=0x0008 → call PC=0+8+6=0xE, rstk=PC+6=6 */
    saturn.pc = 0;
    step_and_print(f, "8E_GOSUBLONG_fwd");

    /* 8F GOSUB_ABS */
    reset_saturn();
    mem_write_hex(0, "8F" "13579");
    saturn.pc = 0;
    step_and_print(f, "8F_GOSUBABS");
}

static void test_full_0808_group(FILE *f) {
    /* 08080 INTON (no carry/state change) */
    reset_saturn();
    mem_write_hex(0, "08080");
    saturn.pc = 0;
    step_and_print(f, "08080_INTON");

    /* 0808F INTOFF */
    reset_saturn();
    mem_write_hex(0, "0808F");
    saturn.pc = 0;
    step_and_print(f, "0808F_INTOFF");

    /* 08084n ABIT=0 n (clear bit n of A) */
    reset_saturn();
    set_reg(saturn.reg[A], "FFFFFFFFFFFFFFFF");
    mem_write_hex(0, "08084" "5");        /* clear bit 5 */
    saturn.pc = 0;
    step_and_print(f, "080845_ABIT_clr5");

    /* 08085n ABIT=1 n */
    reset_saturn();
    mem_write_hex(0, "08085" "3");        /* set bit 3 */
    saturn.pc = 0;
    step_and_print(f, "080853_ABIT_set3");

    /* 08088n CBIT=0 n */
    reset_saturn();
    set_reg(saturn.reg[C], "FFFFFFFFFFFFFFFF");
    mem_write_hex(0, "08088" "7");
    saturn.pc = 0;
    step_and_print(f, "080887_CBIT_clr7");

    /* 08089n CBIT=1 n */
    reset_saturn();
    mem_write_hex(0, "08089" "2");
    saturn.pc = 0;
    step_and_print(f, "080892_CBIT_set2");

    /* 080862 ?ABIT=0 + branch (taken: bit 2 is 0) */
    reset_saturn();
    mem_write_hex(0, "08086" "2" "03");   /* carry=1, branch to PC+5+3=8 */
    saturn.pc = 0;
    step_and_print(f, "080862_ABIT0_branch");

    /* 0808C LA1 : load 2 nibbles into A starting at P */
    reset_saturn();
    mem_write_hex(0, "08082" "1" "AB");   /* LA (n4+1)=2 nibbles A,B */
    saturn.pc = 0;
    step_and_print(f, "080821_LA_n2");
}

static void test_full_081_group(FILE *f) {
    /* 810 ASLC, 814 ASRC: whole-W circular shifts */
    reset_saturn();
    set_reg(saturn.reg[A], "123456789ABCDEF0");
    mem_write_hex(0, "810");
    saturn.pc = 0;
    step_and_print(f, "810_ASLC");

    reset_saturn();
    set_reg(saturn.reg[A], "123456789ABCDEF0");
    mem_write_hex(0, "814");
    saturn.pc = 0;
    step_and_print(f, "814_ASRC");

    /* 811B ASRB (W-field bit shift right) */
    reset_saturn();
    set_reg(saturn.reg[A], "0000000000000007");  /* bit 0 = 1 */
    mem_write_hex(0, "81C");
    saturn.pc = 0;
    step_and_print(f, "81C_ASRB");

    /* 0819 SRB field : 819 fs op (5 nib) */
    reset_saturn();
    set_reg(saturn.reg[A], "0000000000000007");
    mem_write_hex(0, "819" "7" "0");     /* field=W, op=0 (A) */
    saturn.pc = 0;
    step_and_print(f, "8197_SRB_A_W");

    /* 0818 A=A+5 A-field */
    reset_saturn();
    set_reg(saturn.reg[A], "0000000000000010");  /* 0x10 */
    mem_write_hex(0, "818" "F" "0" "4");  /* fs=A(15?not, F=A_FIELD), op=+A, const=5 */
    saturn.pc = 0;
    step_and_print(f, "818F04_AaddCON5");

    /* 081A1 A=R0 field W */
    reset_saturn();
    set_reg(saturn.reg_r[0], "ABCDEFFEDCBA9876");
    mem_write_hex(0, "81A" "7" "1" "0");
    saturn.pc = 0;
    step_and_print(f, "81A710_AeqR0_W");

    /* 081B2 PC=A */
    reset_saturn();
    set_reg(saturn.reg[A], "BCDEF00000000000");  /* A[19:0]=0xEFBCD... actually A[0..4]=B,C,D,E,F→addr BCDEF→LSN */
    mem_write_hex(0, "81B" "2");
    saturn.pc = 0;
    step_and_print(f, "81B2_PCeqA");

    /* 081B4 A=PC */
    reset_saturn();
    mem_write_hex(0x100, "81B" "4");
    saturn.pc = 0x100;
    step_and_print(f, "81B4_AeqPC");
}

/* ──────────────────────────────────────────────
 * main
 * ────────────────────────────────────────────── */
int main(void) {
    FILE *f = stdout;

    /* Install flat memory bus */
    flat_memory_init();
    mem_clear();

    fprintf(f, "# Saturn CPU test vectors\n");
    fprintf(f, "# Generated by generate_tests using x48ng CPU core\n");
    fprintf(f, "# Format: TEST name / PRE state / INST nibbles / POST state / END\n\n");

    test_00_group(f);
    test_00e_group(f);
    test_01_group(f);
    test_02_group(f);
    test_03_group(f);
    test_jmp_group(f);
    test_gosub_group(f);
    test_08_group(f);
    test_08a_group(f);
    test_081_group(f);
    test_ab_group(f);
    test_cd_ef_group(f);
    test_09_group(f);
    test_dat_group(f);
    test_decimal(f);
    test_status(f);
    test_interrupt(f);
    test_field_specs(f);
    test_dat_addr_advance(f);
    test_cpex(f);

    /* expanded coverage */
    test_full_group1_r_regs(f);
    test_full_group13(f);
    test_full_group0e_andor(f);
    test_full_group8_branches(f);
    test_full_0808_group(f);
    test_full_081_group(f);

    fprintf(f, "# Done.\n");
    return 0;
}
