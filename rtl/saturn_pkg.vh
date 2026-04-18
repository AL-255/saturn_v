// saturn_pkg.vh — HP Saturn CPU RTL package
// Reference: x48ng source (emulate.c, registers.c, emulate.h)

`ifndef SATURN_PKG_VH
`define SATURN_PKG_VH

// ──────────────────────────────────────────────
// Register indices for A/B/C/D
// ──────────────────────────────────────────────
`define RA  2'd0
`define RB  2'd1
`define RC  2'd2
`define RD  2'd3

// ──────────────────────────────────────────────
// Status register bit positions (st[3:0])
// ──────────────────────────────────────────────
`define XM  0   // hex-mode flag
`define SB  1   // shift bit
`define SR  2   // saturn reserved
`define MP  3   // module-port flag

// ──────────────────────────────────────────────
// Field codes (5-bit, match registers.c indices)
// ──────────────────────────────────────────────
// start_fields: {P,0,2,0,15,3,0,0, P,0,2,0,15,3,0,0, 0,0,0}
// end_fields:   {P,P,2,2,15,14,1,15, P,P,2,2,15,14,1,4, 3,2,0}
`define FC_P     5'd0   // nibble P only
`define FC_WP    5'd1   // nibbles 0..P
`define FC_XS    5'd2   // nibble 2 (exponent sign)
`define FC_X     5'd3   // nibbles 0..2
`define FC_S     5'd4   // nibble 15
`define FC_M     5'd5   // nibbles 3..14
`define FC_B     5'd6   // nibbles 0..1
`define FC_W     5'd7   // nibbles 0..15 (W field)
// codes 8..14 mirror 0..6 (same start/end)
`define FC_A     5'd15  // nibbles 0..4  (A_FIELD)
`define FC_IN    5'd16  // nibbles 0..3  (IN_FIELD)
`define FC_OUT   5'd17  // nibbles 0..2  (OUT_FIELD)
`define FC_OUTS  5'd18  // nibble 0      (OUTS_FIELD)

// ──────────────────────────────────────────────
// ALU operation codes (6-bit)
// ──────────────────────────────────────────────
`define ALUOP_ADD    6'd0   // res = a + b  (field)
`define ALUOP_SUB    6'd1   // res = a - b  (field)
`define ALUOP_INC    6'd2   // res = a + 1  (field)
`define ALUOP_DEC    6'd3   // res = a - 1  (field)
`define ALUOP_NEG2   6'd4   // res = -a     2's-comp (field)
`define ALUOP_NEG1   6'd5   // res = -a-1   1's-comp (field)
`define ALUOP_AND    6'd6   // res = a & b  (field)
`define ALUOP_OR     6'd7   // res = a | b  (field)
`define ALUOP_SHL    6'd8   // shift left nibble (field)
`define ALUOP_SHR    6'd9   // shift right nibble, sets SB (field)
`define ALUOP_SHLC   6'd10  // shift left circular (field)
`define ALUOP_SHRC   6'd11  // shift right circular, sets SB (field)
`define ALUOP_SHRB   6'd12  // shift right 1 bit through field, sets SB
`define ALUOP_ZERO   6'd13  // res = 0      (field)
`define ALUOP_COPY   6'd14  // res = b      (field)
`define ALUOP_EXCH   6'd15  // res=b, resb=a (field)
`define ALUOP_EQ     6'd16  // carry = (a == b) (field)
`define ALUOP_NE     6'd17  // carry = (a != b) (field)
`define ALUOP_LT     6'd18  // carry = (a < b)  (field)
`define ALUOP_GT     6'd19  // carry = (a > b)  (field)
`define ALUOP_LE     6'd20  // carry = (a <= b) (field)
`define ALUOP_GE     6'd21  // carry = (a >= b) (field)
`define ALUOP_ZERQ   6'd22  // carry = (a == 0) (field)
`define ALUOP_NZRQ   6'd23  // carry = (a != 0) (field)
`define ALUOP_ADDCON 6'd24  // res = a + const; const in b[3:0], always hex
`define ALUOP_SUBCON 6'd25  // res = a - const; const in b[3:0], always hex
`define ALUOP_PASS   6'd63  // res = a (passthrough, no carry change)

// ──────────────────────────────────────────────
// CPU FSM states
// ──────────────────────────────────────────────
`define S_RESET   5'd0
`define S_IF0     5'd1   // fetch nib[0]
`define S_IF1     5'd2   // fetch nib[1]
`define S_IF2     5'd3
`define S_IF3     5'd4
`define S_IF4     5'd5
`define S_IF5     5'd6
`define S_IF6     5'd7
`define S_IF7     5'd8
`define S_LCLOAD  5'd9   // load constant into reg nibble by nibble
`define S_EXEC    5'd10  // execute decoded instruction
`define S_MEMRD   5'd11  // nibble read  for recall (DAT0/DAT1→reg)
`define S_MEMWR   5'd12  // nibble write for store  (reg→DAT0/DAT1)
`define S_INT     5'd13  // interrupt entry

`endif // SATURN_PKG_VH
