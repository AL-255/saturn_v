// saturn_alu.v — field-based 64-bit ALU for HP Saturn
// Operands are packed 16 x 4-bit nibbles, [3:0]=nibble0 (LSN).
// Ref: x48ng registers.c.
`include "saturn_pkg.vh"

module saturn_alu (
    input  wire [63:0] a,
    input  wire [63:0] b,
    input  wire [3:0]  p,         // P register (for FC_P / FC_WP / SHL/SHR-n etc.)
    input  wire [4:0]  field,     // field code
    input  wire [5:0]  op,        // ALU op code (see saturn_pkg.vh)
    input  wire        hex_mode,  // 1 = hex (base 16), 0 = decimal (base 10)
    input  wire        carry_in,  // current saturn.carry (for compares we don't use it)
    output reg  [63:0] res,       // primary result
    output reg  [63:0] res_b,     // secondary result (for EXCH)
    output reg         carry_out
);
    // Field decode
    wire [3:0] fs, fe;
    saturn_field u_fd (.code(field), .p(p), .start_nib(fs), .end_nib(fe));

    integer i;
    reg [4:0] sum;                // 5-bit to catch carry
    reg [3:0] na, nb, nr;
    reg       c;                  // rolling carry / borrow
    reg [7:0] base;
    reg [3:0] tmp_nib;
    reg [63:0] shifted;
    reg        any_nonzero;
    reg        eq_flag, lt_flag;

    // helper wire: which nibbles are in-field?
    function in_field;
        input [3:0] idx;
        input [3:0] s, e;
        begin
            in_field = (idx >= s) && (idx <= e);
        end
    endfunction

    // Extract nibble i from 64-bit reg
    function [3:0] getn;
        input [63:0] r;
        input [3:0]  idx;
        begin
            getn = r[ idx*4 +: 4 ];
        end
    endfunction

    always @(*) begin
        // defaults — preserve input register outside field
        res       = a;
        res_b     = b;
        carry_out = carry_in;
        base      = hex_mode ? 8'd16 : 8'd10;

        case (op)
        // ──────────────────────────────────────────────
        // ADD
        // ──────────────────────────────────────────────
        `ALUOP_ADD: begin
            c = 1'b0;
            res = a;
            for (i = 0; i < 16; i = i + 1) begin
                if (in_field(i[3:0], fs, fe)) begin
                    na  = getn(a, i[3:0]);
                    nb  = getn(b, i[3:0]);
                    sum = {1'b0, na} + {1'b0, nb} + {4'b0, c};
                    if (sum < base) begin
                        res[i*4 +: 4] = sum[3:0];
                        c = 1'b0;
                    end else begin
                        res[i*4 +: 4] = sum[3:0] - base[3:0];
                        c = 1'b1;
                    end
                end
            end
            carry_out = c;
        end

        // ──────────────────────────────────────────────
        // SUB: res = a - b, carry=1 on borrow
        // ──────────────────────────────────────────────
        `ALUOP_SUB: begin
            c = 1'b0;
            res = a;
            for (i = 0; i < 16; i = i + 1) begin
                if (in_field(i[3:0], fs, fe)) begin
                    na  = getn(a, i[3:0]);
                    nb  = getn(b, i[3:0]);
                    sum = {1'b0, na} - {1'b0, nb} - {4'b0, c};
                    if (sum[4] == 1'b0) begin
                        res[i*4 +: 4] = sum[3:0];
                        c = 1'b0;
                    end else begin
                        res[i*4 +: 4] = sum[3:0] + base[3:0];
                        c = 1'b1;
                    end
                end
            end
            carry_out = c;
        end

        // ──────────────────────────────────────────────
        // INC
        // ──────────────────────────────────────────────
        `ALUOP_INC: begin
            c = 1'b1;  // +1
            res = a;
            for (i = 0; i < 16; i = i + 1) begin
                if (in_field(i[3:0], fs, fe)) begin
                    na  = getn(a, i[3:0]);
                    sum = {1'b0, na} + {4'b0, c};
                    if (sum < base) begin
                        res[i*4 +: 4] = sum[3:0];
                        c = 1'b0;
                    end else begin
                        res[i*4 +: 4] = sum[3:0] - base[3:0];
                        c = 1'b1;
                    end
                end
            end
            carry_out = c;
        end

        // ──────────────────────────────────────────────
        // DEC
        // ──────────────────────────────────────────────
        `ALUOP_DEC: begin
            c = 1'b1;  // -1
            res = a;
            for (i = 0; i < 16; i = i + 1) begin
                if (in_field(i[3:0], fs, fe)) begin
                    na  = getn(a, i[3:0]);
                    sum = {1'b0, na} - {4'b0, c};
                    if (sum[4] == 1'b0) begin
                        res[i*4 +: 4] = sum[3:0];
                        c = 1'b0;
                    end else begin
                        res[i*4 +: 4] = sum[3:0] + base[3:0];
                        c = 1'b1;
                    end
                end
            end
            carry_out = c;
        end

        // ──────────────────────────────────────────────
        // NEG2: 2's complement over field (res = -a)
        // carry = 1 if any result nibble non-zero
        // ──────────────────────────────────────────────
        `ALUOP_NEG2: begin
            c = 1'b1;  // start with +1 (two's complement = ~a + 1)
            res = a;
            any_nonzero = 1'b0;
            for (i = 0; i < 16; i = i + 1) begin
                if (in_field(i[3:0], fs, fe)) begin
                    na  = getn(a, i[3:0]);
                    // base-complement digit: (base-1) - na  → then +1 rolled through c
                    sum = {1'b0, (base[3:0] - 4'd1) - na} + {4'b0, c};
                    if (sum < base) begin
                        nr = sum[3:0];
                        c = 1'b0;
                    end else begin
                        nr = sum[3:0] - base[3:0];
                        c = 1'b1;
                    end
                    res[i*4 +: 4] = nr;
                    if (nr != 4'd0) any_nonzero = 1'b1;
                end
            end
            carry_out = any_nonzero;
        end

        // ──────────────────────────────────────────────
        // NEG1: 1's complement over field (res = ~a in base)
        // carry always cleared
        // ──────────────────────────────────────────────
        `ALUOP_NEG1: begin
            res = a;
            for (i = 0; i < 16; i = i + 1) begin
                if (in_field(i[3:0], fs, fe)) begin
                    na  = getn(a, i[3:0]);
                    res[i*4 +: 4] = (base[3:0] - 4'd1) - na;
                end
            end
            carry_out = 1'b0;
        end

        // ──────────────────────────────────────────────
        // AND / OR (bitwise per nibble, field-masked)
        // ──────────────────────────────────────────────
        `ALUOP_AND: begin
            res = a;
            for (i = 0; i < 16; i = i + 1)
                if (in_field(i[3:0], fs, fe))
                    res[i*4 +: 4] = getn(a, i[3:0]) & getn(b, i[3:0]);
        end

        `ALUOP_OR: begin
            res = a;
            for (i = 0; i < 16; i = i + 1)
                if (in_field(i[3:0], fs, fe))
                    res[i*4 +: 4] = getn(a, i[3:0]) | getn(b, i[3:0]);
        end

        // ──────────────────────────────────────────────
        // SHL: shift left by 1 nibble within field (zero-fill low)
        // ──────────────────────────────────────────────
        `ALUOP_SHL: begin
            res = a;
            // walk from high→low so we read old values
            for (i = 15; i >= 0; i = i - 1) begin
                if (in_field(i[3:0], fs, fe)) begin
                    if (i[3:0] == fs)
                        res[i*4 +: 4] = 4'd0;
                    else
                        res[i*4 +: 4] = getn(a, i[3:0] - 4'd1);
                end
            end
        end

        // ──────────────────────────────────────────────
        // SHR: shift right by 1 nibble within field, SB = old LSN of field
        // ──────────────────────────────────────────────
        `ALUOP_SHR: begin
            res = a;
            // SB flagging handled in top-level via carry_out passthrough? No—
            // The Saturn sets st[SB] if shifted-out nibble was non-zero.
            // We expose it via carry_out; caller maps to SB.
            carry_out = (getn(a, fs) != 4'd0);
            for (i = 0; i < 16; i = i + 1) begin
                if (in_field(i[3:0], fs, fe)) begin
                    if (i[3:0] == fe)
                        res[i*4 +: 4] = 4'd0;
                    else
                        res[i*4 +: 4] = getn(a, i[3:0] + 4'd1);
                end
            end
        end

        // ──────────────────────────────────────────────
        // SHLC: circular shift left by 1 nibble within field
        // ──────────────────────────────────────────────
        `ALUOP_SHLC: begin
            res = a;
            tmp_nib = getn(a, fe);  // saved top
            for (i = 15; i >= 0; i = i - 1) begin
                if (in_field(i[3:0], fs, fe)) begin
                    if (i[3:0] == fs)
                        res[i*4 +: 4] = tmp_nib;
                    else
                        res[i*4 +: 4] = getn(a, i[3:0] - 4'd1);
                end
            end
        end

        // ──────────────────────────────────────────────
        // SHRC: circular shift right by 1 nibble within field
        // ──────────────────────────────────────────────
        `ALUOP_SHRC: begin
            res = a;
            tmp_nib = getn(a, fs);  // saved bottom
            carry_out = (tmp_nib != 4'd0);
            for (i = 0; i < 16; i = i + 1) begin
                if (in_field(i[3:0], fs, fe)) begin
                    if (i[3:0] == fe)
                        res[i*4 +: 4] = tmp_nib;
                    else
                        res[i*4 +: 4] = getn(a, i[3:0] + 4'd1);
                end
            end
        end

        // ──────────────────────────────────────────────
        // SHRB: shift right 1 BIT through field, SB = old bit 0 of field LSN
        // ──────────────────────────────────────────────
        `ALUOP_SHRB: begin
            shifted = a;
            na = getn(a, fs);
            carry_out = na[0];
            for (i = 0; i < 16; i = i + 1) begin
                if (in_field(i[3:0], fs, fe)) begin
                    na = getn(a, i[3:0]);          // this nibble
                    nb = getn(a, i[3:0] + 4'd1);   // next-higher nibble (for LSB inject)
                    if (i[3:0] == fe)
                        shifted[i*4 +: 4] = {1'b0, na[3:1]};
                    else
                        shifted[i*4 +: 4] = {nb[0], na[3:1]};
                end
            end
            res = shifted;
        end

        // ──────────────────────────────────────────────
        // ZERO: res[field] = 0
        // ──────────────────────────────────────────────
        `ALUOP_ZERO: begin
            res = a;
            for (i = 0; i < 16; i = i + 1)
                if (in_field(i[3:0], fs, fe))
                    res[i*4 +: 4] = 4'd0;
        end

        // ──────────────────────────────────────────────
        // COPY: res[field] = b[field]
        // ──────────────────────────────────────────────
        `ALUOP_COPY: begin
            res = a;
            for (i = 0; i < 16; i = i + 1)
                if (in_field(i[3:0], fs, fe))
                    res[i*4 +: 4] = getn(b, i[3:0]);
        end

        // ──────────────────────────────────────────────
        // EXCH: swap a[field] <-> b[field]
        // ──────────────────────────────────────────────
        `ALUOP_EXCH: begin
            res   = a;
            res_b = b;
            for (i = 0; i < 16; i = i + 1) begin
                if (in_field(i[3:0], fs, fe)) begin
                    res[i*4 +: 4]   = getn(b, i[3:0]);
                    res_b[i*4 +: 4] = getn(a, i[3:0]);
                end
            end
        end

        // ──────────────────────────────────────────────
        // Compares (set carry only). HEX/DEC base doesn't matter for EQ/NE;
        // for LT/GT etc. we compare as unsigned nibble sequence.
        // ──────────────────────────────────────────────
        `ALUOP_EQ, `ALUOP_NE, `ALUOP_LT, `ALUOP_GT, `ALUOP_LE, `ALUOP_GE: begin
            eq_flag = 1'b1;
            lt_flag = 1'b0;
            // from HIGH nibble down: first difference decides lt/gt
            for (i = 15; i >= 0; i = i - 1) begin
                if (in_field(i[3:0], fs, fe)) begin
                    na = getn(a, i[3:0]);
                    nb = getn(b, i[3:0]);
                    if (eq_flag) begin
                        if (na != nb) begin
                            eq_flag = 1'b0;
                            lt_flag = (na < nb);
                        end
                    end
                end
            end
            case (op)
                `ALUOP_EQ: carry_out =  eq_flag;
                `ALUOP_NE: carry_out = ~eq_flag;
                `ALUOP_LT: carry_out = ~eq_flag &  lt_flag;
                `ALUOP_GT: carry_out = ~eq_flag & ~lt_flag;
                `ALUOP_LE: carry_out =  eq_flag |  lt_flag;
                `ALUOP_GE: carry_out =  eq_flag | ~lt_flag;
                default:   carry_out = 1'b0;
            endcase
        end

        // ──────────────────────────────────────────────
        // ZERQ / NZRQ — compare to zero
        // ──────────────────────────────────────────────
        `ALUOP_ZERQ, `ALUOP_NZRQ: begin
            any_nonzero = 1'b0;
            for (i = 0; i < 16; i = i + 1)
                if (in_field(i[3:0], fs, fe))
                    if (getn(a, i[3:0]) != 4'd0) any_nonzero = 1'b1;
            carry_out = (op == `ALUOP_ZERQ) ? ~any_nonzero : any_nonzero;
        end

        // ──────────────────────────────────────────────
        // ADDCON / SUBCON — always hex, const in b[3:0], propagate through field
        // ──────────────────────────────────────────────
        `ALUOP_ADDCON: begin
            // Caller places the full constant (1..16) in b[4:0], not b[3:0].
            // Saturn opcode 818x / 81Ax encodes val = n5+1, which is 1..16
            // and therefore won't fit in 4 bits when n5 == 0xF.
            c = 1'b0;
            res = a;
            for (i = 0; i < 16; i = i + 1) begin
                if (in_field(i[3:0], fs, fe)) begin
                    na  = getn(a, i[3:0]);
                    sum = (i[3:0] == fs) ? ({3'b0, na} + {3'b0, b[4:0]})
                                         : ({3'b0, na} + {7'b0, c});
                    if (sum < 8'd16) begin
                        res[i*4 +: 4] = sum[3:0];
                        c = 1'b0;
                    end else begin
                        res[i*4 +: 4] = sum[3:0];  // natural hex wrap
                        c = 1'b1;
                    end
                end
            end
            carry_out = c;
        end

        `ALUOP_SUBCON: begin
            c = 1'b0;
            res = a;
            for (i = 0; i < 16; i = i + 1) begin
                if (in_field(i[3:0], fs, fe)) begin
                    na  = getn(a, i[3:0]);
                    sum = (i[3:0] == fs) ? ({3'b0, na} - {3'b0, b[4:0]})
                                         : ({3'b0, na} - {7'b0, c});
                    if (sum[4] == 1'b0) begin
                        res[i*4 +: 4] = sum[3:0];
                        c = 1'b0;
                    end else begin
                        res[i*4 +: 4] = sum[3:0];  // natural hex wrap
                        c = 1'b1;
                    end
                end
            end
            carry_out = c;
        end

        `ALUOP_PASS: begin
            res = a;
        end

        default: begin
            res = a;
        end
        endcase
    end
endmodule
