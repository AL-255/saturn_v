// tb_alu.v — smoke test for saturn_alu
`include "saturn_pkg.vh"

module tb_alu;
    reg  [63:0] a, b;
    reg  [3:0]  p;
    reg  [4:0]  field;
    reg  [5:0]  op;
    reg         hex_mode, carry_in;
    wire [63:0] res, res_b;
    wire        carry_out;

    saturn_alu dut (
        .a(a), .b(b), .p(p), .field(field), .op(op),
        .hex_mode(hex_mode), .carry_in(carry_in),
        .res(res), .res_b(res_b), .carry_out(carry_out)
    );

    integer err = 0;
    task check(input [255:0] tag,
               input [63:0]  got,
               input [63:0]  expv,
               input         got_c,
               input         expect_c);
        begin
            if (got !== expv || got_c !== expect_c) begin
                $display("FAIL %0s  got=%h c=%b  exp=%h c=%b", tag, got, got_c, expv, expect_c);
                err = err + 1;
            end else $display("PASS %0s", tag);
        end
    endtask

    initial begin
        // ADD A field (5 nibbles), hex:  A=00001FFFFF, B=0000000001 → A=0000200000, c=0
        a = 64'h000000000000FFFF; b = 64'h0000000000000001;
        p = 0; field = `FC_A; op = `ALUOP_ADD; hex_mode = 1; carry_in = 0; #1;
        check("ADD_A_hex",          res, 64'h0000000000010000, carry_out, 1'b0);

        // ADD A field, dec:  A=99, B=01 → A=00, c=1
        a = 64'h0000000000000099; b = 64'h0000000000000001;
        p = 0; field = `FC_A; op = `ALUOP_ADD; hex_mode = 0; carry_in = 0; #1;
        check("ADD_A_dec_carry",    res, 64'h0000000000000100, carry_out, 1'b0);

        // ADD A field, dec at limit: A=99999 + B=1 → A=00000, c=1
        a = 64'h0000000000099999; b = 64'h0000000000000001;
        p = 0; field = `FC_A; op = `ALUOP_ADD; hex_mode = 0; carry_in = 0; #1;
        check("ADD_A_dec_wrap",     res, 64'h0000000000000000, carry_out, 1'b1);

        // SUB A field, hex: A=0 - B=1 → A=FFFFF, c=1
        a = 64'h0000000000000000; b = 64'h0000000000000001;
        p = 0; field = `FC_A; op = `ALUOP_SUB; hex_mode = 1; carry_in = 0; #1;
        check("SUB_A_hex_borrow",   res, 64'h00000000000FFFFF, carry_out, 1'b1);

        // NEG2 W: A = -(0x1) hex → W = FFFFFFFFFFFFFFFF, c=1
        a = 64'h0000000000000001; b = 64'h0;
        p = 0; field = `FC_W; op = `ALUOP_NEG2; hex_mode = 1; carry_in = 0; #1;
        check("NEG2_W_-1",          res, 64'hFFFFFFFFFFFFFFFF, carry_out, 1'b1);

        // NEG2 W of 0 → 0, c=0 (no nonzero nibble)
        a = 64'h0; b = 64'h0;
        p = 0; field = `FC_W; op = `ALUOP_NEG2; hex_mode = 1; #1;
        check("NEG2_W_0",           res, 64'h0, carry_out, 1'b0);

        // SHL P-field (nibble P=3): a=0x1_000 → res=0x1_000 with nibble3 shifted left (same pos)
        // For a 1-nibble field, SHL zeros the LSN of the field (same position).
        a = 64'h0000_0000_0000_5000; p = 4'd3; field = `FC_P; op = `ALUOP_SHL; #1;
        check("SHL_P3",             res, 64'h0000_0000_0000_0000, carry_out, 1'b0);

        // SHR W: a=0xABCDEF01234567 → res>>1 nibble, SB=(LSN!=0)
        a = 64'hABCDEF0123456789; field = `FC_W; op = `ALUOP_SHR; #1;
        check("SHR_W_LSN9",         res, 64'h0ABCDEF012345678, carry_out, 1'b1);

        // COPY M-field: res[M] = b[M], outside preserved
        a = 64'hFEDCBA9876543210; b = 64'h1111111111111111;
        field = `FC_M; op = `ALUOP_COPY; #1;
        // M = nibbles 3..14; outside (0..2 and 15) is preserved from a=FEDCBA9876543210
        check("COPY_M",             res, 64'hF111111111111210, carry_out, 1'b0);

        // EQ W: a==b → c=1
        a = 64'hDEADBEEFCAFEBABE; b = 64'hDEADBEEFCAFEBABE;
        field = `FC_W; op = `ALUOP_EQ; #1;
        check("EQ_W_equal",         res, a, carry_out, 1'b1);

        // LT A: a<b (1 < 2) → c=1
        a = 64'h0000000000000001; b = 64'h0000000000000002;
        field = `FC_A; op = `ALUOP_LT; #1;
        check("LT_A_1lt2",          res, a, carry_out, 1'b1);

        if (err == 0) $display("ALU SMOKE OK");
        else          $display("ALU SMOKE FAIL  %0d errors", err);
        $finish;
    end
endmodule
