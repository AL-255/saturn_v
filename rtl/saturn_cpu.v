// saturn_cpu.v — HP Saturn CPU (behavioral, reference-style RTL).
//
// Dispatch mirrors x48ng emulate.c:
//   top nibble (n0) selects group;
//   group 0 dispatches on n1 (most ops are 2 nibbles; 0E dispatches on n2);
//   group 1 dispatches on n1 → sub-groups 10..15 dispatch on n2;
//   group 8 dispatches on n1 → 80/81 dispatch further on n2/n3.
//
`include "saturn_pkg.vh"

module saturn_cpu (
    input  wire        clk,
    input  wire        rst_n,
    output reg  [19:0] mem_addr,
    output reg         mem_we,
    output reg         mem_re,
    output reg  [3:0]  mem_wdata,
    input  wire [3:0]  mem_rdata,
    // observability
    output wire [19:0] dbg_pc,
    output wire [63:0] dbg_A, dbg_B, dbg_C, dbg_D,
    output wire [19:0] dbg_D0, dbg_D1,
    output wire [3:0]  dbg_P,
    output wire [3:0]  dbg_ST,
    output wire [15:0] dbg_PSTAT,
    output wire        dbg_CARRY,
    output wire        dbg_HEXMODE,
    output wire signed [3:0] dbg_RSTKP,
    output wire [159:0] dbg_RSTK,
    input  wire        step,
    output reg         step_done
);
    // ── architectural state ──────────────────────────────
    reg [63:0] regA, regB, regC, regD;
    reg [63:0] regR [0:4];
    reg [19:0] D0, D1;
    reg [3:0]  P;
    reg [19:0] PC;
    reg [3:0]  ST;
    reg [15:0] PSTAT;
    reg        CARRY;
    reg        HEXMODE;
    reg [19:0] RSTK [0:7];
    reg signed [3:0] RSTKP;
    reg [15:0] HST;
    reg [63:0] IN_REG, OUT_REG;   // 16-nibble buffers; only a few nibbles used

    assign dbg_pc      = PC;
    assign dbg_A       = regA;  assign dbg_B = regB;
    assign dbg_C       = regC;  assign dbg_D = regD;
    assign dbg_D0      = D0;    assign dbg_D1 = D1;
    assign dbg_P       = P;     assign dbg_ST = ST;
    assign dbg_PSTAT   = PSTAT;
    assign dbg_CARRY   = CARRY;
    assign dbg_HEXMODE = HEXMODE;
    assign dbg_RSTKP   = RSTKP;
    genvar gi;
    generate
        for (gi = 0; gi < 8; gi = gi + 1)
            assign dbg_RSTK[gi*20 +: 20] = RSTK[gi];
    endgenerate

    // ── ALU ──────────────────────────────────────────────
    reg  [63:0] alu_a, alu_b;
    reg  [4:0]  alu_field;
    reg  [5:0]  alu_op;
    wire [63:0] alu_res, alu_res_b;
    wire        alu_cout;
    saturn_alu u_alu (
        .a(alu_a), .b(alu_b), .p(P), .field(alu_field), .op(alu_op),
        .hex_mode(HEXMODE), .carry_in(CARRY),
        .res(alu_res), .res_b(alu_res_b), .carry_out(alu_cout)
    );
    // Field start/end for alu_apply's dest-merge (needed when dest != ia;
    // saturn_alu preserves out-of-field nibbles from a, but dest's own
    // out-of-field nibbles must be kept in those cases).
    wire [3:0] alu_fs, alu_fe;
    saturn_field u_alu_fd (.code(alu_field), .p(P), .start_nib(alu_fs), .end_nib(alu_fe));

    // ── FSM ──────────────────────────────────────────────
    localparam S_IDLE = 4'd0, S_FETCH = 4'd1, S_EXEC = 4'd2,
               S_MEMR = 4'd3, S_MEMW = 4'd4, S_DONE = 4'd5;
    reg [3:0]  state;
    // Fetch buffer: 32 nibbles, enough for the longest instruction (LA with 16-nib
    // constant = 22 nibbles; LC/LA/long GOTO/GOSUB all fit in 22).
    reg [3:0]  ib [0:31];
    reg [5:0]  fetch_i;

    // pending memory op
    reg        do_memop;
    reg        memop_is_write;
    reg [19:0] memop_addr;
    reg [63:0] memop_data;
    reg [1:0]  memop_dst_reg;
    reg [4:0]  memop_len;
    reg [3:0]  memop_start_nib; // starting nibble index in register
    reg [4:0]  mem_i;

    // ── helpers ──────────────────────────────────────────
    function [63:0] read_reg(input [1:0] r);
        case (r)
            2'd0: read_reg = regA;
            2'd1: read_reg = regB;
            2'd2: read_reg = regC;
            2'd3: read_reg = regD;
        endcase
    endfunction

    task write_reg(input [1:0] r, input [63:0] v);
        begin
            case (r)
                2'd0: regA = v;
                2'd1: regB = v;
                2'd2: regC = v;
                2'd3: regD = v;
            endcase
        end
    endtask

    task alu_apply(input [1:0] rd, input [63:0] ia, input [63:0] ibb,
                   input [4:0] fld, input [5:0] o);
        reg [63:0] dest_old, merged;
        integer    j;
        begin
            dest_old = read_reg(rd);
            alu_a = ia; alu_b = ibb; alu_field = fld; alu_op = o;
            #0;
            // In-field nibbles from alu_res; out-of-field nibbles from the
            // destination's prior value. Correct whether or not rd == ia.
            merged = dest_old;
            for (j = 0; j < 16; j = j + 1)
                if (j[3:0] >= alu_fs && j[3:0] <= alu_fe)
                    merged[j*4 +: 4] = alu_res[j*4 +: 4];
            write_reg(rd, merged);
            CARRY = alu_cout;
        end
    endtask

    task alu_compare(input [63:0] ia, input [63:0] ibb,
                     input [4:0] fld, input [5:0] o);
        begin
            alu_a = ia; alu_b = ibb; alu_field = fld; alu_op = o;
            #0;
            CARRY = alu_cout;
        end
    endtask

    task alu_exch(input [1:0] r1, input [1:0] r2, input [4:0] fld);
        begin
            alu_a = read_reg(r1); alu_b = read_reg(r2);
            alu_field = fld; alu_op = `ALUOP_EXCH;
            #0;
            write_reg(r1, alu_res);
            write_reg(r2, alu_res_b);
        end
    endtask

    task rstk_push(input [19:0] v);
        integer i;
        begin
            // C ref push_return_addr(): if full, shift entries down (drop oldest)
            // and keep rstk_ptr at the top; otherwise grow the stack.
            if (RSTKP >= 4'sd7) begin
                for (i = 1; i < 8; i = i + 1)
                    RSTK[i-1] = RSTK[i];
            end else begin
                RSTKP = RSTKP + 4'sd1;
            end
            RSTK[RSTKP] = v;
        end
    endtask

    function [19:0] rstk_top(input dummy);
        rstk_top = (RSTKP >= 0) ? RSTK[RSTKP] : 20'd0;
    endfunction

    task rstk_pop;
        begin
            if (RSTKP >= 0) RSTKP = RSTKP - 4'sd1;
        end
    endtask

    function [19:0] sext8(input [7:0]  v);  sext8  = {{12{v[7]}},  v}; endfunction
    function [19:0] sext12(input [11:0] v); sext12 = {{8{v[11]}},  v}; endfunction
    function [19:0] sext16(input [15:0] v); sext16 = {{4{v[15]}},  v}; endfunction

    function [4:0] fld_len(input [4:0] fcode);
        case (fcode)
            `FC_P:   fld_len = 5'd1;
            `FC_WP:  fld_len = {1'b0, P} + 5'd1;
            `FC_XS:  fld_len = 5'd1;
            `FC_X:   fld_len = 5'd3;
            `FC_S:   fld_len = 5'd1;
            `FC_M:   fld_len = 5'd12;
            `FC_B:   fld_len = 5'd2;
            `FC_W:   fld_len = 5'd16;
            `FC_A:   fld_len = 5'd5;
            default: fld_len = 5'd1;
        endcase
    endfunction

    // ── Execute one instruction (all nibbles pre-fetched in ib[]) ──
    task execute;
        reg [3:0]  n0,n1,n2,n3,n4,n5,n6,n7;
        reg [4:0]  fld5;
        reg [1:0]  ra, rb;
        reg [19:0] disp, new_pc;
        reg        any_zero;
        reg        ill;
        reg [3:0]  tmp4;
        reg [19:0] tmp20;
        reg [63:0] tmp64;
        integer    k;
        reg [3:0]  pos;
        begin
            n0=ib[0]; n1=ib[1]; n2=ib[2]; n3=ib[3];
            n4=ib[4]; n5=ib[5]; n6=ib[6]; n7=ib[7];
            ill=0; do_memop=0; new_pc = PC;

            case (n0)
            // ═══════════════════════════ GROUP 0 ═══════════════════════════
            4'h0: begin
                case (n1)
                4'h0: begin ST[`XM]=1'b1; new_pc=rstk_top(0); rstk_pop(); end // RTNSXM
                4'h1: begin new_pc=rstk_top(0); rstk_pop(); end               // RTN
                4'h2: begin CARRY=1'b1; new_pc=rstk_top(0); rstk_pop(); end   // RTNSC
                4'h3: begin CARRY=1'b0; new_pc=rstk_top(0); rstk_pop(); end   // RTNCC
                4'h4: begin HEXMODE=1'b1; new_pc=PC+20'd2; end                // SETHEX
                4'h5: begin HEXMODE=1'b0; new_pc=PC+20'd2; end                // SETDEC
                4'h6: begin rstk_push(regC[19:0]); new_pc=PC+20'd2; end       // RSTK=C
                4'h7: begin regC[19:0]=rstk_top(0); rstk_pop(); new_pc=PC+20'd2; end // C=RSTK
                4'h8: begin PSTAT = PSTAT & 16'hF000; new_pc=PC+20'd2; end    // CLRST: clear low 12 bits of PSTAT
                4'h9: begin                                                    // C=ST
                    regC[11:0] = PSTAT[11:0];    // only low 12 pstat bits per C ref
                    new_pc=PC+20'd2;
                end
                4'hA: begin                                                    // ST=C
                    PSTAT[11:0] = regC[11:0];
                    new_pc=PC+20'd2;
                end
                4'hB: begin                                                    // CSTEX
                    tmp64 = {52'd0, PSTAT[11:0]};
                    PSTAT[11:0] = regC[11:0];
                    regC[11:0]  = tmp64[11:0];
                    new_pc=PC+20'd2;
                end
                4'hC: begin                                                    // P=P+1
                    if (P==4'd15) begin P=4'd0;  CARRY=1'b1; end
                    else          begin P=P+4'd1;CARRY=1'b0; end
                    new_pc=PC+20'd2;
                end
                4'hD: begin                                                    // P=P-1
                    if (P==4'd0)  begin P=4'd15; CARRY=1'b1; end
                    else          begin P=P-4'd1;CARRY=1'b0; end
                    new_pc=PC+20'd2;
                end
                4'hE: begin // 0E fs op : AND/OR (4-nibble); does NOT update carry
                    fld5 = {1'b0, n2};
                    case (n3)
                    4'h0: begin alu_a=regA; alu_b=regB; alu_field=fld5; alu_op=`ALUOP_AND; #0; regA=alu_res; end
                    4'h1: begin alu_a=regB; alu_b=regC; alu_field=fld5; alu_op=`ALUOP_AND; #0; regB=alu_res; end
                    4'h2: begin alu_a=regC; alu_b=regA; alu_field=fld5; alu_op=`ALUOP_AND; #0; regC=alu_res; end
                    4'h3: begin alu_a=regD; alu_b=regC; alu_field=fld5; alu_op=`ALUOP_AND; #0; regD=alu_res; end
                    4'h4: begin alu_a=regB; alu_b=regA; alu_field=fld5; alu_op=`ALUOP_AND; #0; regB=alu_res; end
                    4'h5: begin alu_a=regC; alu_b=regB; alu_field=fld5; alu_op=`ALUOP_AND; #0; regC=alu_res; end
                    4'h6: begin alu_a=regA; alu_b=regC; alu_field=fld5; alu_op=`ALUOP_AND; #0; regA=alu_res; end
                    4'h7: begin alu_a=regC; alu_b=regD; alu_field=fld5; alu_op=`ALUOP_AND; #0; regC=alu_res; end
                    4'h8: begin alu_a=regA; alu_b=regB; alu_field=fld5; alu_op=`ALUOP_OR;  #0; regA=alu_res; end
                    4'h9: begin alu_a=regB; alu_b=regC; alu_field=fld5; alu_op=`ALUOP_OR;  #0; regB=alu_res; end
                    4'hA: begin alu_a=regC; alu_b=regA; alu_field=fld5; alu_op=`ALUOP_OR;  #0; regC=alu_res; end
                    4'hB: begin alu_a=regD; alu_b=regC; alu_field=fld5; alu_op=`ALUOP_OR;  #0; regD=alu_res; end
                    4'hC: begin alu_a=regB; alu_b=regA; alu_field=fld5; alu_op=`ALUOP_OR;  #0; regB=alu_res; end
                    4'hD: begin alu_a=regC; alu_b=regB; alu_field=fld5; alu_op=`ALUOP_OR;  #0; regC=alu_res; end
                    4'hE: begin alu_a=regA; alu_b=regC; alu_field=fld5; alu_op=`ALUOP_OR;  #0; regA=alu_res; end
                    4'hF: begin alu_a=regC; alu_b=regD; alu_field=fld5; alu_op=`ALUOP_OR;  #0; regC=alu_res; end
                    endcase
                    new_pc = PC + 20'd4;
                end
                4'hF: begin new_pc=rstk_top(0); rstk_pop(); end                // RTI
                endcase
            end
            // ═══════════════════════════ GROUP 1 ═══════════════════════════
            4'h1: begin
                case (n1)
                // ── 10x : R{0..4} = A or R{0..4} = C (W-field) ──
                // n2 in {0,1,2,3,4}: R{n2}=A ; {5,6,7}: dup of R{1,2,3}=A
                // n2 in {8,9,a,b,c}: R{0..4}=C ; {d,e,f}: dup of R{1,2,3}=C
                4'h0: begin
                    case (n2)
                    4'h0: regR[0] = regA;
                    4'h1, 4'h5: regR[1] = regA;
                    4'h2, 4'h6: regR[2] = regA;
                    4'h3, 4'h7: regR[3] = regA;
                    4'h4: regR[4] = regA;
                    4'h8: regR[0] = regC;
                    4'h9, 4'hD: regR[1] = regC;
                    4'hA, 4'hE: regR[2] = regC;
                    4'hB, 4'hF: regR[3] = regC;
                    4'hC: regR[4] = regC;
                    endcase
                    new_pc = PC + 20'd3;
                end
                // ── 11x : A = R{0..4} or C = R{0..4} (W-field) ──
                4'h1: begin
                    case (n2)
                    4'h0: regA = regR[0];
                    4'h1, 4'h5: regA = regR[1];
                    4'h2, 4'h6: regA = regR[2];
                    4'h3, 4'h7: regA = regR[3];
                    4'h4: regA = regR[4];
                    4'h8: regC = regR[0];
                    4'h9, 4'hD: regC = regR[1];
                    4'hA, 4'hE: regC = regR[2];
                    4'hB, 4'hF: regC = regR[3];
                    4'hC: regC = regR[4];
                    endcase
                    new_pc = PC + 20'd3;
                end
                // ── 12x : AR{0..4}EX or CR{0..4}EX (W-field) ──
                4'h2: begin
                    case (n2)
                    4'h0:        begin tmp64 = regA; regA = regR[0]; regR[0] = tmp64; end
                    4'h1, 4'h5:  begin tmp64 = regA; regA = regR[1]; regR[1] = tmp64; end
                    4'h2, 4'h6:  begin tmp64 = regA; regA = regR[2]; regR[2] = tmp64; end
                    4'h3, 4'h7:  begin tmp64 = regA; regA = regR[3]; regR[3] = tmp64; end
                    4'h4:        begin tmp64 = regA; regA = regR[4]; regR[4] = tmp64; end
                    4'h8:        begin tmp64 = regC; regC = regR[0]; regR[0] = tmp64; end
                    4'h9, 4'hD:  begin tmp64 = regC; regC = regR[1]; regR[1] = tmp64; end
                    4'hA, 4'hE:  begin tmp64 = regC; regC = regR[2]; regR[2] = tmp64; end
                    4'hB, 4'hF:  begin tmp64 = regC; regC = regR[3]; regR[3] = tmp64; end
                    4'hC:        begin tmp64 = regC; regC = regR[4]; regR[4] = tmp64; end
                    endcase
                    new_pc = PC + 20'd3;
                end
                // ── 13x : D0/D1 assignments and exchanges ──
                4'h3: begin
                    case (n2)
                    4'h0: D0 = regA[19:0];                                        // D0 = A
                    4'h1: D1 = regA[19:0];                                        // D1 = A
                    4'h2: begin tmp20=D0; D0=regA[19:0]; regA[19:0]=tmp20; end     // AD0EX (5-nib)
                    4'h3: begin tmp20=D1; D1=regA[19:0]; regA[19:0]=tmp20; end     // AD1EX
                    4'h4: D0 = regC[19:0];                                        // D0 = C
                    4'h5: D1 = regC[19:0];                                        // D1 = C
                    4'h6: begin tmp20=D0; D0=regC[19:0]; regC[19:0]=tmp20; end     // CD0EX
                    4'h7: begin tmp20=D1; D1=regC[19:0]; regC[19:0]=tmp20; end     // CD1EX
                    4'h8: D0[15:0] = regA[15:0];                                  // D0 = AS (4-nib)
                    4'h9: D1[15:0] = regA[15:0];                                  // D1 = AS
                    4'hA: begin tmp20[15:0]=D0[15:0]; D0[15:0]=regA[15:0]; regA[15:0]=tmp20[15:0]; end // AD0XS
                    4'hB: begin tmp20[15:0]=D1[15:0]; D1[15:0]=regA[15:0]; regA[15:0]=tmp20[15:0]; end // AD1XS
                    4'hC: D0[15:0] = regC[15:0];                                  // D0 = CS
                    4'hD: D1[15:0] = regC[15:0];                                  // D1 = CS
                    4'hE: begin tmp20[15:0]=D0[15:0]; D0[15:0]=regC[15:0]; regC[15:0]=tmp20[15:0]; end // CD0XS
                    4'hF: begin tmp20[15:0]=D1[15:0]; D1[15:0]=regC[15:0]; regC[15:0]=tmp20[15:0]; end // CD1XS
                    endcase
                    new_pc = PC + 20'd3;
                end
                4'h4: begin // DAT recall/store 3-nibble: 1 4 op3 (W if op3<8, B if op3>=8)
                    case (n2 & 4'h7)
                    4'h0: begin memop_is_write=1; memop_dst_reg=`RA; memop_addr=D0; end
                    4'h1: begin memop_is_write=1; memop_dst_reg=`RA; memop_addr=D1; end
                    4'h2: begin memop_is_write=0; memop_dst_reg=`RA; memop_addr=D0; end
                    4'h3: begin memop_is_write=0; memop_dst_reg=`RA; memop_addr=D1; end
                    4'h4: begin memop_is_write=1; memop_dst_reg=`RC; memop_addr=D0; end
                    4'h5: begin memop_is_write=1; memop_dst_reg=`RC; memop_addr=D1; end
                    4'h6: begin memop_is_write=0; memop_dst_reg=`RC; memop_addr=D0; end
                    4'h7: begin memop_is_write=0; memop_dst_reg=`RC; memop_addr=D1; end
                    endcase
                    memop_data       = read_reg(memop_dst_reg);
                    // x48ng: opX = op3<8 ? A_FIELD(5 nibbles) : B_FIELD(2 nibbles)
                    memop_len        = (n2 < 4'd8) ? 5'd5 : 5'd2;
                    memop_start_nib  = 4'd0;
                    do_memop         = 1'b1;
                    new_pc=PC+20'd3;
                end
                4'h5: begin // DAT recall/store 4-nibble: 1 5 op3 op4
                    case (n2 & 4'h7)
                    4'h0: begin memop_is_write=1; memop_dst_reg=`RA; memop_addr=D0; end
                    4'h1: begin memop_is_write=1; memop_dst_reg=`RA; memop_addr=D1; end
                    4'h2: begin memop_is_write=0; memop_dst_reg=`RA; memop_addr=D0; end
                    4'h3: begin memop_is_write=0; memop_dst_reg=`RA; memop_addr=D1; end
                    4'h4: begin memop_is_write=1; memop_dst_reg=`RC; memop_addr=D0; end
                    4'h5: begin memop_is_write=1; memop_dst_reg=`RC; memop_addr=D1; end
                    4'h6: begin memop_is_write=0; memop_dst_reg=`RC; memop_addr=D0; end
                    4'h7: begin memop_is_write=0; memop_dst_reg=`RC; memop_addr=D1; end
                    endcase
                    memop_data      = read_reg(memop_dst_reg);
                    if (n2 >= 4'd8) begin
                        // explicit n-nibble count: op4 = n3, n4 is the count-1
                        memop_len       = {1'b0, n3} + 5'd1;
                        memop_start_nib = 4'd0;
                    end else begin
                        // field-code variant: op4 (n3) is field code
                        memop_len       = fld_len({1'b0, n3});
                        // start = 0 for all fields except S(15) and M(3) and P/WP (depends on P)
                        case (n3)
                        `FC_P:   memop_start_nib = P;
                        `FC_WP:  memop_start_nib = 4'd0;
                        `FC_XS:  memop_start_nib = 4'd2;
                        `FC_X:   memop_start_nib = 4'd0;
                        `FC_S:   memop_start_nib = 4'd15;
                        `FC_M:   memop_start_nib = 4'd3;
                        `FC_B:   memop_start_nib = 4'd0;
                        `FC_W:   memop_start_nib = 4'd0;
                        `FC_A:   memop_start_nib = 4'd0;
                        default: memop_start_nib = 4'd0;
                        endcase
                    end
                    do_memop = 1'b1;
                    new_pc   = PC + 20'd4;
                end
                4'h6: begin
                    // D0 = D0 + (n2+1) ; carry = 1 if result overflows 20 bits
                    {CARRY, D0} = {1'b0, D0} + {16'd0, n2} + 21'd1;
                    D0 = D0 & 20'hFFFFF;
                    new_pc = PC + 20'd3;
                end
                4'h7: begin
                    {CARRY, D1} = {1'b0, D1} + {16'd0, n2} + 21'd1;
                    D1 = D1 & 20'hFFFFF;
                    new_pc = PC + 20'd3;
                end
                4'h8: begin
                    // D0 = D0 - (n2+1) ; carry = 1 if underflow (D0 < n2+1)
                    {CARRY, D0} = {1'b0, D0} - ({16'd0, n2} + 21'd1);
                    D0 = D0 & 20'hFFFFF;
                    new_pc = PC + 20'd3;
                end
                4'h9: begin D0[7:0]  = {n3,n2};                      new_pc=PC+20'd4; end
                4'hA: begin D0[15:0] = {n5,n4,n3,n2};                new_pc=PC+20'd6; end
                4'hB: begin D0       = {n6,n5,n4,n3,n2};             new_pc=PC+20'd7; end
                4'hC: begin
                    {CARRY, D1} = {1'b0, D1} - ({16'd0, n2} + 21'd1);
                    D1 = D1 & 20'hFFFFF;
                    new_pc = PC + 20'd3;
                end
                4'hD: begin D1[7:0]  = {n3,n2};                      new_pc=PC+20'd4; end
                4'hE: begin D1[15:0] = {n5,n4,n3,n2};                new_pc=PC+20'd6; end
                4'hF: begin D1       = {n6,n5,n4,n3,n2};             new_pc=PC+20'd7; end
                endcase
            end
            // ═══════════════════════════ GROUP 2 ═══════════════════════════
            4'h2: begin P = n1; new_pc=PC+20'd2; end
            // ═══════════════════════════ GROUP 3 : LC ══════════════════════
            // Load (n1+1) nibbles into C starting at C[P], wrapping. P unchanged.
            4'h3: begin
                pos = P;
                for (k = 0; k <= n1; k = k + 1) begin
                    regC[pos*4 +: 4] = ib[2+k];
                    pos = (pos==4'd15) ? 4'd0 : pos + 4'd1;
                end
                new_pc = PC + 20'd2 + {15'd0, n1} + 20'd1;
            end
            // ═══════════════════════════ GROUP 4 : JC (jump if carry) ══════
            4'h4: begin
                disp = sext8({n2, n1});
                if ({n2, n1} == 8'h02) begin
                    new_pc = PC + 20'd3;
                end else if (CARRY) begin
                    if (disp != 20'd0)
                        new_pc = (PC + disp + 20'd1) & 20'hFFFFF;
                    else begin
                        new_pc = rstk_top(0); rstk_pop();
                    end
                end else
                    new_pc = PC + 20'd3;
            end
            // ═══════════════════════════ GROUP 5 : JNC (jump if no carry) ══
            4'h5: begin
                if (!CARRY) begin
                    disp = sext8({n2, n1});
                    if (disp != 20'd0)
                        new_pc = (PC + disp + 20'd1) & 20'hFFFFF;
                    else begin
                        new_pc = rstk_top(0); rstk_pop();
                    end
                end else
                    new_pc = PC + 20'd3;
            end
            // ═══════════════════════════ GROUP 6 : GOTO (3-nib signed) ═════
            4'h6: begin
                disp = sext12({n3, n2, n1});
                if ({n3, n2, n1} == 12'h003)
                    new_pc = PC + 20'd4;
                else if ({n3, n2, n1} == 12'h004)
                    new_pc = PC + 20'd5;   // trap stub
                else
                    new_pc = (PC + disp + 20'd1) & 20'hFFFFF;
            end
            // ═══════════════════════════ GROUP 7 : GOSUB (3-nib signed) ════
            4'h7: begin
                disp = sext12({n3, n2, n1});
                rstk_push(PC + 20'd4);
                new_pc = (PC + disp + 20'd4) & 20'hFFFFF;
            end
            // ═══════════════════════════ GROUP 8 ═══════════════════════════
            4'h8: begin
                case (n1)
                // ── 80x ──
                4'h0: begin
                    case (n2)
                    4'h0: begin /* OUT=CS */ OUT_REG[3:0]  = regC[3:0];  new_pc=PC+20'd3; end
                    4'h1: begin /* OUT=C  */ OUT_REG[11:0] = regC[11:0]; new_pc=PC+20'd3; end
                    4'h2: begin /* A=IN   */ regA[15:0] = IN_REG[15:0]; new_pc=PC+20'd3; end
                    4'h3: begin /* C=IN   */ regC[15:0] = IN_REG[15:0]; new_pc=PC+20'd3; end
                    4'h4: new_pc = PC + 20'd3; // UNCNFG stub
                    4'h5: new_pc = PC + 20'd3; // CONFIG stub
                    4'h6: begin regC[19:0]=20'd0; new_pc=PC+20'd3; end // C=ID stub
                    4'h7: new_pc = PC + 20'd3; // SHUTDN stub
                    4'h8: begin  // 0808x subdispatch on n3
                        case (n3)
                        4'h0: new_pc = PC + 20'd4;                          // INTON
                        4'h1: new_pc = PC + 20'd5;                          // RSI
                        4'h2: begin                                          // LA n : load A with (n4+1) nibbles from ib[5..]
                            pos = P;
                            for (k = 0; k <= n4; k = k + 1) begin
                                regA[pos*4 +: 4] = ib[5 + k];
                                pos = (pos == 4'd15) ? 4'd0 : pos + 4'd1;
                            end
                            new_pc = PC + 20'd6 + {15'd0, n4};
                        end
                        4'h3: new_pc = PC + 20'd4;                          // BUSCB
                        4'h4: begin regA[n4] = 1'b0; new_pc = PC + 20'd5; end // ABIT=0 n
                        4'h5: begin regA[n4] = 1'b1; new_pc = PC + 20'd5; end // ABIT=1 n
                        4'h6: begin                                          // ?ABIT=0 n + branch
                            CARRY = (regA[n4] == 1'b0);
                            if (CARRY) begin
                                if ({n6, n5} != 8'h00) new_pc = (PC + 20'd5 + sext8({n6, n5})) & 20'hFFFFF;
                                else begin new_pc = rstk_top(0); rstk_pop(); end
                            end else new_pc = PC + 20'd7;
                        end
                        4'h7: begin                                          // ?ABIT=1 n + branch
                            CARRY = (regA[n4] == 1'b1);
                            if (CARRY) begin
                                if ({n6, n5} != 8'h00) new_pc = (PC + 20'd5 + sext8({n6, n5})) & 20'hFFFFF;
                                else begin new_pc = rstk_top(0); rstk_pop(); end
                            end else new_pc = PC + 20'd7;
                        end
                        4'h8: begin regC[n4] = 1'b0; new_pc = PC + 20'd5; end // CBIT=0 n
                        4'h9: begin regC[n4] = 1'b1; new_pc = PC + 20'd5; end // CBIT=1 n
                        4'hA: begin                                          // ?CBIT=0 n + branch
                            CARRY = (regC[n4] == 1'b0);
                            if (CARRY) begin
                                if ({n6, n5} != 8'h00) new_pc = (PC + 20'd5 + sext8({n6, n5})) & 20'hFFFFF;
                                else begin new_pc = rstk_top(0); rstk_pop(); end
                            end else new_pc = PC + 20'd7;
                        end
                        4'hB: begin                                          // ?CBIT=1 n + branch
                            CARRY = (regC[n4] == 1'b1);
                            if (CARRY) begin
                                if ({n6, n5} != 8'h00) new_pc = (PC + 20'd5 + sext8({n6, n5})) & 20'hFFFFF;
                                else begin new_pc = rstk_top(0); rstk_pop(); end
                            end else new_pc = PC + 20'd7;
                        end
                        4'hC: new_pc = PC + 20'd4;                          // PC=(A) (stub — needs mem read)
                        4'hD: new_pc = PC + 20'd4;                          // BUSCD stub
                        4'hE: new_pc = PC + 20'd4;                          // PC=(C) (stub)
                        4'hF: new_pc = PC + 20'd4;                          // INTOFF
                        default: new_pc = PC + 20'd4;
                        endcase
                    end
                    4'h9: begin // C+P+1: always hex (ref add_p_plus_one ignores HEXMODE).
                        alu_a     = regC;
                        alu_b     = 64'd0; alu_b[4:0] = {1'b0, P} + 5'd1;
                        alu_field = `FC_A;
                        alu_op    = `ALUOP_ADDCON;
                        #0;
                        regC  = alu_res;
                        CARRY = alu_cout;
                        new_pc=PC+20'd3;
                    end
                    4'hA: new_pc = PC + 20'd3; // RESET stub
                    4'hB: new_pc = PC + 20'd3; // BUSCC stub
                    4'hC: begin regC[{1'b0,n3}*4 +: 4] = P; new_pc=PC+20'd4; end  // C=P n
                    4'hD: begin P = regC[{1'b0,n3}*4 +: 4]; new_pc=PC+20'd4; end  // P=C n
                    4'hE: begin regC[3:0]=4'd0; new_pc=PC+20'd3; end              // SREQ?
                    4'hF: begin                                                    // CPEX n
                        tmp4 = regC[{1'b0,n3}*4 +: 4];
                        regC[{1'b0,n3}*4 +: 4] = P;
                        P = tmp4;
                        new_pc=PC+20'd4;
                    end
                    default: new_pc = PC + 20'd3;
                    endcase
                end
                // ── 81x : SLC/SRC + 0818/0819/081A/081B + SRB ──
                4'h1: begin
                    case (n2)
                    4'h0: begin alu_a=regA; alu_b=64'd0; alu_field=`FC_W; alu_op=`ALUOP_SHLC; #0; regA=alu_res; new_pc=PC+20'd3; end
                    4'h1: begin alu_a=regB; alu_b=64'd0; alu_field=`FC_W; alu_op=`ALUOP_SHLC; #0; regB=alu_res; new_pc=PC+20'd3; end
                    4'h2: begin alu_a=regC; alu_b=64'd0; alu_field=`FC_W; alu_op=`ALUOP_SHLC; #0; regC=alu_res; new_pc=PC+20'd3; end
                    4'h3: begin alu_a=regD; alu_b=64'd0; alu_field=`FC_W; alu_op=`ALUOP_SHLC; #0; regD=alu_res; new_pc=PC+20'd3; end
                    4'h4: begin alu_a=regA; alu_b=64'd0; alu_field=`FC_W; alu_op=`ALUOP_SHRC; #0; regA=alu_res; if (alu_cout) ST[`SB]=1'b1; new_pc=PC+20'd3; end
                    4'h5: begin alu_a=regB; alu_b=64'd0; alu_field=`FC_W; alu_op=`ALUOP_SHRC; #0; regB=alu_res; if (alu_cout) ST[`SB]=1'b1; new_pc=PC+20'd3; end
                    4'h6: begin alu_a=regC; alu_b=64'd0; alu_field=`FC_W; alu_op=`ALUOP_SHRC; #0; regC=alu_res; if (alu_cout) ST[`SB]=1'b1; new_pc=PC+20'd3; end
                    4'h7: begin alu_a=regD; alu_b=64'd0; alu_field=`FC_W; alu_op=`ALUOP_SHRC; #0; regD=alu_res; if (alu_cout) ST[`SB]=1'b1; new_pc=PC+20'd3; end
                    4'h8: begin // 0818 : R = R +/- CON (6 nibbles)
                        // n3=fs, n4=op4 (bit3: +/-, bits 1:0: reg), n5=const-1
                        // val range is 1..16 — need 5 bits so n5=0xF (→16) fits.
                        fld5 = {1'b0, n3};
                        alu_b = 64'd0; alu_b[4:0] = {1'b0, n5} + 5'd1;
                        if (n4 < 4'd8) begin
                            case (n4[1:0])
                            2'd0: begin alu_a=regA; alu_field=fld5; alu_op=`ALUOP_ADDCON; #0; regA=alu_res; CARRY=alu_cout; end
                            2'd1: begin alu_a=regB; alu_field=fld5; alu_op=`ALUOP_ADDCON; #0; regB=alu_res; CARRY=alu_cout; end
                            2'd2: begin alu_a=regC; alu_field=fld5; alu_op=`ALUOP_ADDCON; #0; regC=alu_res; CARRY=alu_cout; end
                            2'd3: begin alu_a=regD; alu_field=fld5; alu_op=`ALUOP_ADDCON; #0; regD=alu_res; CARRY=alu_cout; end
                            endcase
                        end else begin
                            case (n4[1:0])
                            2'd0: begin alu_a=regA; alu_field=fld5; alu_op=`ALUOP_SUBCON; #0; regA=alu_res; CARRY=alu_cout; end
                            2'd1: begin alu_a=regB; alu_field=fld5; alu_op=`ALUOP_SUBCON; #0; regB=alu_res; CARRY=alu_cout; end
                            2'd2: begin alu_a=regC; alu_field=fld5; alu_op=`ALUOP_SUBCON; #0; regC=alu_res; CARRY=alu_cout; end
                            2'd3: begin alu_a=regD; alu_field=fld5; alu_op=`ALUOP_SUBCON; #0; regD=alu_res; CARRY=alu_cout; end
                            endcase
                        end
                        new_pc = PC + 20'd6;
                    end
                    4'h9: begin // 0819 : R SRB fs (5 nibbles)
                        fld5 = {1'b0, n3};
                        case (n4[1:0])
                        2'd0: begin alu_a=regA; alu_b=64'd0; alu_field=fld5; alu_op=`ALUOP_SHRB; #0; regA=alu_res; if (alu_cout) ST[`SB]=1'b1; end
                        2'd1: begin alu_a=regB; alu_b=64'd0; alu_field=fld5; alu_op=`ALUOP_SHRB; #0; regB=alu_res; if (alu_cout) ST[`SB]=1'b1; end
                        2'd2: begin alu_a=regC; alu_b=64'd0; alu_field=fld5; alu_op=`ALUOP_SHRB; #0; regC=alu_res; if (alu_cout) ST[`SB]=1'b1; end
                        2'd3: begin alu_a=regD; alu_b=64'd0; alu_field=fld5; alu_op=`ALUOP_SHRB; #0; regD=alu_res; if (alu_cout) ST[`SB]=1'b1; end
                        endcase
                        new_pc = PC + 20'd5;
                    end
                    4'hA: begin // 081A subdispatch on n4
                        fld5 = {1'b0, n3};
                        case (n4)
                        4'h0: begin  // 081A0 : R{x}={A,C} field fs
                            case (n5)
                            4'h0:       begin alu_a=regR[0]; alu_b=regA; alu_field=fld5; alu_op=`ALUOP_COPY; #0; regR[0]=alu_res; end
                            4'h1, 4'h5: begin alu_a=regR[1]; alu_b=regA; alu_field=fld5; alu_op=`ALUOP_COPY; #0; regR[1]=alu_res; end
                            4'h2, 4'h6: begin alu_a=regR[2]; alu_b=regA; alu_field=fld5; alu_op=`ALUOP_COPY; #0; regR[2]=alu_res; end
                            4'h3, 4'h7: begin alu_a=regR[3]; alu_b=regA; alu_field=fld5; alu_op=`ALUOP_COPY; #0; regR[3]=alu_res; end
                            4'h4:       begin alu_a=regR[4]; alu_b=regA; alu_field=fld5; alu_op=`ALUOP_COPY; #0; regR[4]=alu_res; end
                            4'h8:       begin alu_a=regR[0]; alu_b=regC; alu_field=fld5; alu_op=`ALUOP_COPY; #0; regR[0]=alu_res; end
                            4'h9, 4'hD: begin alu_a=regR[1]; alu_b=regC; alu_field=fld5; alu_op=`ALUOP_COPY; #0; regR[1]=alu_res; end
                            4'hA, 4'hE: begin alu_a=regR[2]; alu_b=regC; alu_field=fld5; alu_op=`ALUOP_COPY; #0; regR[2]=alu_res; end
                            4'hB, 4'hF: begin alu_a=regR[3]; alu_b=regC; alu_field=fld5; alu_op=`ALUOP_COPY; #0; regR[3]=alu_res; end
                            4'hC:       begin alu_a=regR[4]; alu_b=regC; alu_field=fld5; alu_op=`ALUOP_COPY; #0; regR[4]=alu_res; end
                            endcase
                            new_pc = PC + 20'd6;
                        end
                        4'h1: begin  // 081A1 : {A,C}=R{x} field fs
                            case (n5)
                            4'h0:       begin alu_a=regA; alu_b=regR[0]; alu_field=fld5; alu_op=`ALUOP_COPY; #0; regA=alu_res; end
                            4'h1, 4'h5: begin alu_a=regA; alu_b=regR[1]; alu_field=fld5; alu_op=`ALUOP_COPY; #0; regA=alu_res; end
                            4'h2, 4'h6: begin alu_a=regA; alu_b=regR[2]; alu_field=fld5; alu_op=`ALUOP_COPY; #0; regA=alu_res; end
                            4'h3, 4'h7: begin alu_a=regA; alu_b=regR[3]; alu_field=fld5; alu_op=`ALUOP_COPY; #0; regA=alu_res; end
                            4'h4:       begin alu_a=regA; alu_b=regR[4]; alu_field=fld5; alu_op=`ALUOP_COPY; #0; regA=alu_res; end
                            4'h8:       begin alu_a=regC; alu_b=regR[0]; alu_field=fld5; alu_op=`ALUOP_COPY; #0; regC=alu_res; end
                            4'h9, 4'hD: begin alu_a=regC; alu_b=regR[1]; alu_field=fld5; alu_op=`ALUOP_COPY; #0; regC=alu_res; end
                            4'hA, 4'hE: begin alu_a=regC; alu_b=regR[2]; alu_field=fld5; alu_op=`ALUOP_COPY; #0; regC=alu_res; end
                            4'hB, 4'hF: begin alu_a=regC; alu_b=regR[3]; alu_field=fld5; alu_op=`ALUOP_COPY; #0; regC=alu_res; end
                            4'hC:       begin alu_a=regC; alu_b=regR[4]; alu_field=fld5; alu_op=`ALUOP_COPY; #0; regC=alu_res; end
                            endcase
                            new_pc = PC + 20'd6;
                        end
                        4'h2: begin  // 081A2 : AR{x}EX / CR{x}EX field
                            case (n5)
                            4'h0:       begin alu_a=regA; alu_b=regR[0]; alu_field=fld5; alu_op=`ALUOP_EXCH; #0; regA=alu_res; regR[0]=alu_res_b; end
                            4'h1, 4'h5: begin alu_a=regA; alu_b=regR[1]; alu_field=fld5; alu_op=`ALUOP_EXCH; #0; regA=alu_res; regR[1]=alu_res_b; end
                            4'h2, 4'h6: begin alu_a=regA; alu_b=regR[2]; alu_field=fld5; alu_op=`ALUOP_EXCH; #0; regA=alu_res; regR[2]=alu_res_b; end
                            4'h3, 4'h7: begin alu_a=regA; alu_b=regR[3]; alu_field=fld5; alu_op=`ALUOP_EXCH; #0; regA=alu_res; regR[3]=alu_res_b; end
                            4'h4:       begin alu_a=regA; alu_b=regR[4]; alu_field=fld5; alu_op=`ALUOP_EXCH; #0; regA=alu_res; regR[4]=alu_res_b; end
                            4'h8:       begin alu_a=regC; alu_b=regR[0]; alu_field=fld5; alu_op=`ALUOP_EXCH; #0; regC=alu_res; regR[0]=alu_res_b; end
                            4'h9, 4'hD: begin alu_a=regC; alu_b=regR[1]; alu_field=fld5; alu_op=`ALUOP_EXCH; #0; regC=alu_res; regR[1]=alu_res_b; end
                            4'hA, 4'hE: begin alu_a=regC; alu_b=regR[2]; alu_field=fld5; alu_op=`ALUOP_EXCH; #0; regC=alu_res; regR[2]=alu_res_b; end
                            4'hB, 4'hF: begin alu_a=regC; alu_b=regR[3]; alu_field=fld5; alu_op=`ALUOP_EXCH; #0; regC=alu_res; regR[3]=alu_res_b; end
                            4'hC:       begin alu_a=regC; alu_b=regR[4]; alu_field=fld5; alu_op=`ALUOP_EXCH; #0; regC=alu_res; regR[4]=alu_res_b; end
                            endcase
                            new_pc = PC + 20'd6;
                        end
                        default: new_pc = PC + 20'd5;
                        endcase
                    end
                    4'hB: begin  // 081B : PC=A/PC=C/A=PC/C=PC/APCEX/CPCEX
                        case (n3)
                        4'h2: new_pc = regA[19:0];
                        4'h3: new_pc = regC[19:0];
                        4'h4: begin regA[19:0] = PC + 20'd4; new_pc = PC + 20'd4; end
                        4'h5: begin regC[19:0] = PC + 20'd4; new_pc = PC + 20'd4; end
                        4'h6: begin tmp20 = regA[19:0]; regA[19:0] = PC + 20'd4; new_pc = tmp20; end
                        4'h7: begin tmp20 = regC[19:0]; regC[19:0] = PC + 20'd4; new_pc = tmp20; end
                        default: new_pc = PC + 20'd4;
                        endcase
                    end
                    4'hC: begin alu_a=regA; alu_b=64'd0; alu_field=`FC_W; alu_op=`ALUOP_SHRB; #0; regA=alu_res; if (alu_cout) ST[`SB]=1'b1; new_pc=PC+20'd3; end // ASRB
                    4'hD: begin alu_a=regB; alu_b=64'd0; alu_field=`FC_W; alu_op=`ALUOP_SHRB; #0; regB=alu_res; if (alu_cout) ST[`SB]=1'b1; new_pc=PC+20'd3; end // BSRB
                    4'hE: begin alu_a=regC; alu_b=64'd0; alu_field=`FC_W; alu_op=`ALUOP_SHRB; #0; regC=alu_res; if (alu_cout) ST[`SB]=1'b1; new_pc=PC+20'd3; end // CSRB
                    4'hF: begin alu_a=regD; alu_b=64'd0; alu_field=`FC_W; alu_op=`ALUOP_SHRB; #0; regD=alu_res; if (alu_cout) ST[`SB]=1'b1; new_pc=PC+20'd3; end // DSRB
                    endcase
                end
                4'h2: begin // 82x  CLRST mask
                    if (n2[0]) ST[`XM]=0;
                    if (n2[1]) ST[`SB]=0;
                    if (n2[2]) ST[`SR]=0;
                    if (n2[3]) ST[`MP]=0;
                    new_pc=PC+20'd3;
                end
                4'h3: begin // 83x  ?ST=1 mask (carry=1 iff all selected bits are 0) + branch
                    CARRY = 1'b1;
                    if (n2[0] && ST[`XM]) CARRY = 1'b0;
                    if (n2[1] && ST[`SB]) CARRY = 1'b0;
                    if (n2[2] && ST[`SR]) CARRY = 1'b0;
                    if (n2[3] && ST[`MP]) CARRY = 1'b0;
                    if (CARRY) begin
                        if ({n4, n3} != 8'h00) new_pc = (PC + 20'd3 + sext8({n4, n3})) & 20'hFFFFF;
                        else begin new_pc = rstk_top(0); rstk_pop(); end
                    end else new_pc = PC + 20'd5;
                end
                4'h4: begin PSTAT[n2]=1'b0; new_pc=PC+20'd3; end  // ST=0 n
                4'h5: begin PSTAT[n2]=1'b1; new_pc=PC+20'd3; end  // ST=1 n
                4'h6: begin                                       // ?ST=0 n + branch
                    CARRY = (PSTAT[n2] == 1'b0);
                    if (CARRY) begin
                        if ({n4, n3} != 8'h00) new_pc = (PC + 20'd3 + sext8({n4, n3})) & 20'hFFFFF;
                        else begin new_pc = rstk_top(0); rstk_pop(); end
                    end else new_pc = PC + 20'd5;
                end
                4'h7: begin                                       // ?ST=1 n + branch
                    CARRY = (PSTAT[n2] == 1'b1);
                    if (CARRY) begin
                        if ({n4, n3} != 8'h00) new_pc = (PC + 20'd3 + sext8({n4, n3})) & 20'hFFFFF;
                        else begin new_pc = rstk_top(0); rstk_pop(); end
                    end else new_pc = PC + 20'd5;
                end
                4'h8: begin                                       // ?P#n + branch
                    CARRY = (P != n2);
                    if (CARRY) begin
                        if ({n4, n3} != 8'h00) new_pc = (PC + 20'd3 + sext8({n4, n3})) & 20'hFFFFF;
                        else begin new_pc = rstk_top(0); rstk_pop(); end
                    end else new_pc = PC + 20'd5;
                end
                4'h9: begin                                       // ?P=n + branch
                    CARRY = (P == n2);
                    if (CARRY) begin
                        if ({n4, n3} != 8'h00) new_pc = (PC + 20'd3 + sext8({n4, n3})) & 20'hFFFFF;
                        else begin new_pc = rstk_top(0); rstk_pop(); end
                    end else new_pc = PC + 20'd5;
                end
                4'hA: begin // 8Ax : ?A=B/?A=0 family on A-field + branch
                    case (n2)
                    4'h0: alu_compare(regA, regB, `FC_A, `ALUOP_EQ);
                    4'h1: alu_compare(regB, regC, `FC_A, `ALUOP_EQ);
                    4'h2: alu_compare(regA, regC, `FC_A, `ALUOP_EQ);
                    4'h3: alu_compare(regC, regD, `FC_A, `ALUOP_EQ);
                    4'h4: alu_compare(regA, regB, `FC_A, `ALUOP_NE);
                    4'h5: alu_compare(regB, regC, `FC_A, `ALUOP_NE);
                    4'h6: alu_compare(regA, regC, `FC_A, `ALUOP_NE);
                    4'h7: alu_compare(regC, regD, `FC_A, `ALUOP_NE);
                    4'h8: alu_compare(regA, 64'd0, `FC_A, `ALUOP_ZERQ);
                    4'h9: alu_compare(regB, 64'd0, `FC_A, `ALUOP_ZERQ);
                    4'hA: alu_compare(regC, 64'd0, `FC_A, `ALUOP_ZERQ);
                    4'hB: alu_compare(regD, 64'd0, `FC_A, `ALUOP_ZERQ);
                    4'hC: alu_compare(regA, 64'd0, `FC_A, `ALUOP_NZRQ);
                    4'hD: alu_compare(regB, 64'd0, `FC_A, `ALUOP_NZRQ);
                    4'hE: alu_compare(regC, 64'd0, `FC_A, `ALUOP_NZRQ);
                    4'hF: alu_compare(regD, 64'd0, `FC_A, `ALUOP_NZRQ);
                    endcase
                    if (CARRY) begin
                        if ({n4, n3} != 8'h00) new_pc = (PC + 20'd3 + sext8({n4, n3})) & 20'hFFFFF;
                        else begin new_pc = rstk_top(0); rstk_pop(); end
                    end else new_pc = PC + 20'd5;
                end
                4'hB: begin // 8Bx : ?A>B/?A<B family on A-field + branch
                    case (n2)
                    4'h0: alu_compare(regA, regB, `FC_A, `ALUOP_GT);
                    4'h1: alu_compare(regB, regC, `FC_A, `ALUOP_GT);
                    4'h2: alu_compare(regC, regA, `FC_A, `ALUOP_GT);
                    4'h3: alu_compare(regD, regC, `FC_A, `ALUOP_GT);
                    4'h4: alu_compare(regA, regB, `FC_A, `ALUOP_LT);
                    4'h5: alu_compare(regB, regC, `FC_A, `ALUOP_LT);
                    4'h6: alu_compare(regC, regA, `FC_A, `ALUOP_LT);
                    4'h7: alu_compare(regD, regC, `FC_A, `ALUOP_LT);
                    4'h8: alu_compare(regA, regB, `FC_A, `ALUOP_GE);
                    4'h9: alu_compare(regB, regC, `FC_A, `ALUOP_GE);
                    4'hA: alu_compare(regC, regA, `FC_A, `ALUOP_GE);
                    4'hB: alu_compare(regD, regC, `FC_A, `ALUOP_GE);
                    4'hC: alu_compare(regA, regB, `FC_A, `ALUOP_LE);
                    4'hD: alu_compare(regB, regC, `FC_A, `ALUOP_LE);
                    4'hE: alu_compare(regC, regA, `FC_A, `ALUOP_LE);
                    4'hF: alu_compare(regD, regC, `FC_A, `ALUOP_LE);
                    endcase
                    if (CARRY) begin
                        if ({n4, n3} != 8'h00) new_pc = (PC + 20'd3 + sext8({n4, n3})) & 20'hFFFFF;
                        else begin new_pc = rstk_top(0); rstk_pop(); end
                    end else new_pc = PC + 20'd5;
                end
                // ── 8C : GOTO_LONG 4-nibble rel, PC = PC + disp + 2 ──
                4'hC: begin
                    disp = sext16({n5, n4, n3, n2});
                    new_pc = (PC + disp + 20'd2) & 20'hFFFFF;
                end
                // ── 8D : GOTO_ABS 5-nibble absolute ──
                4'hD: begin
                    new_pc = {n6, n5, n4, n3, n2};
                end
                // ── 8E : GOSUB_LONG 4-nibble rel, push PC+6, PC = PC + disp + 6 ──
                4'hE: begin
                    disp = sext16({n5, n4, n3, n2});
                    rstk_push(PC + 20'd6);
                    new_pc = (PC + disp + 20'd6) & 20'hFFFFF;
                end
                // ── 8F : GOSUB_ABS 5-nibble absolute, push PC+7, PC = abs ──
                4'hF: begin
                    rstk_push(PC + 20'd7);
                    new_pc = {n6, n5, n4, n3, n2};
                end
                default: new_pc = PC + 20'd3;
                endcase
            end
            // ═══════════════════════════ GROUP 9 : test + cond-branch ══════
            // 5-nibble: n0=9, n1=fs, n2=op, n3,n4=signed 2-nibble displacement.
            // After test: if carry → pc+=3, disp = sext8(n4,n3);
            //                        if disp != 0 → pc = pc + disp,
            //                        else         → pc = pop_rstk() (RTN).
            //             else → pc += 5.
            4'h9: begin
                if (n1 < 4'd8) begin
                    // fs < 8 : equality/inequality with field fs
                    fld5 = {1'b0, n1};
                    case (n2)
                    4'h0: alu_compare(regA, regB, fld5, `ALUOP_EQ);
                    4'h1: alu_compare(regB, regC, fld5, `ALUOP_EQ);
                    4'h2: alu_compare(regA, regC, fld5, `ALUOP_EQ);
                    4'h3: alu_compare(regC, regD, fld5, `ALUOP_EQ);
                    4'h4: alu_compare(regA, regB, fld5, `ALUOP_NE);
                    4'h5: alu_compare(regB, regC, fld5, `ALUOP_NE);
                    4'h6: alu_compare(regA, regC, fld5, `ALUOP_NE);
                    4'h7: alu_compare(regC, regD, fld5, `ALUOP_NE);
                    4'h8: alu_compare(regA, 64'd0, fld5, `ALUOP_ZERQ);
                    4'h9: alu_compare(regB, 64'd0, fld5, `ALUOP_ZERQ);
                    4'hA: alu_compare(regC, 64'd0, fld5, `ALUOP_ZERQ);
                    4'hB: alu_compare(regD, 64'd0, fld5, `ALUOP_ZERQ);
                    4'hC: alu_compare(regA, 64'd0, fld5, `ALUOP_NZRQ);
                    4'hD: alu_compare(regB, 64'd0, fld5, `ALUOP_NZRQ);
                    4'hE: alu_compare(regC, 64'd0, fld5, `ALUOP_NZRQ);
                    4'hF: alu_compare(regD, 64'd0, fld5, `ALUOP_NZRQ);
                    endcase
                end else begin
                    // fs >= 8 : ordered compares, actual field = fs & 7
                    fld5 = {1'b0, n1 & 4'h7};
                    case (n2)
                    4'h0: alu_compare(regA, regB, fld5, `ALUOP_GT);
                    4'h1: alu_compare(regB, regC, fld5, `ALUOP_GT);
                    4'h2: alu_compare(regC, regA, fld5, `ALUOP_GT);
                    4'h3: alu_compare(regD, regC, fld5, `ALUOP_GT);
                    4'h4: alu_compare(regA, regB, fld5, `ALUOP_LT);
                    4'h5: alu_compare(regB, regC, fld5, `ALUOP_LT);
                    4'h6: alu_compare(regC, regA, fld5, `ALUOP_LT);
                    4'h7: alu_compare(regD, regC, fld5, `ALUOP_LT);
                    4'h8: alu_compare(regA, regB, fld5, `ALUOP_GE);
                    4'h9: alu_compare(regB, regC, fld5, `ALUOP_GE);
                    4'hA: alu_compare(regC, regA, fld5, `ALUOP_GE);
                    4'hB: alu_compare(regD, regC, fld5, `ALUOP_GE);
                    4'hC: alu_compare(regA, regB, fld5, `ALUOP_LE);
                    4'hD: alu_compare(regB, regC, fld5, `ALUOP_LE);
                    4'hE: alu_compare(regC, regA, fld5, `ALUOP_LE);
                    4'hF: alu_compare(regD, regC, fld5, `ALUOP_LE);
                    endcase
                end
                // conditional branch
                if (CARRY) begin
                    disp = sext8({n4, n3});
                    if (disp == 20'd0) begin
                        new_pc = rstk_top(0);
                        rstk_pop();
                    end else
                        new_pc = (PC + 20'd3 + disp) & 20'hFFFFF;
                end else
                    new_pc = PC + 20'd5;
            end
            // ═══════════════════════════ GROUP A : field arith (ADD/DEC or ZERO/COPY/EXCH) ══
            4'hA: begin
                if (n1 < 4'd8) begin
                    fld5 = {1'b0, n1};
                    case (n2)
                    4'h0: alu_apply(`RA, regA, regB, fld5, `ALUOP_ADD);
                    4'h1: alu_apply(`RB, regB, regC, fld5, `ALUOP_ADD);
                    4'h2: alu_apply(`RC, regC, regA, fld5, `ALUOP_ADD);
                    4'h3: alu_apply(`RD, regD, regC, fld5, `ALUOP_ADD);
                    4'h4: alu_apply(`RA, regA, regA, fld5, `ALUOP_ADD);
                    4'h5: alu_apply(`RB, regB, regB, fld5, `ALUOP_ADD);
                    4'h6: alu_apply(`RC, regC, regC, fld5, `ALUOP_ADD);
                    4'h7: alu_apply(`RD, regD, regD, fld5, `ALUOP_ADD);
                    4'h8: alu_apply(`RB, regB, regA, fld5, `ALUOP_ADD);
                    4'h9: alu_apply(`RC, regC, regB, fld5, `ALUOP_ADD);
                    4'hA: alu_apply(`RA, regA, regC, fld5, `ALUOP_ADD);
                    4'hB: alu_apply(`RC, regC, regD, fld5, `ALUOP_ADD);
                    4'hC: alu_apply(`RA, regA, 64'd0, fld5, `ALUOP_DEC);
                    4'hD: alu_apply(`RB, regB, 64'd0, fld5, `ALUOP_DEC);
                    4'hE: alu_apply(`RC, regC, 64'd0, fld5, `ALUOP_DEC);
                    4'hF: alu_apply(`RD, regD, 64'd0, fld5, `ALUOP_DEC);
                    endcase
                end else begin
                    fld5 = {1'b0, n1 & 4'h7};
                    case (n2)
                    4'h0: alu_apply(`RA, regA, 64'd0, fld5, `ALUOP_ZERO);
                    4'h1: alu_apply(`RB, regB, 64'd0, fld5, `ALUOP_ZERO);
                    4'h2: alu_apply(`RC, regC, 64'd0, fld5, `ALUOP_ZERO);
                    4'h3: alu_apply(`RD, regD, 64'd0, fld5, `ALUOP_ZERO);
                    4'h4: alu_apply(`RA, regA, regB, fld5, `ALUOP_COPY);  // A=B
                    4'h5: alu_apply(`RB, regB, regC, fld5, `ALUOP_COPY);
                    4'h6: alu_apply(`RC, regC, regA, fld5, `ALUOP_COPY);
                    4'h7: alu_apply(`RD, regD, regC, fld5, `ALUOP_COPY);
                    4'h8: alu_apply(`RB, regB, regA, fld5, `ALUOP_COPY);
                    4'h9: alu_apply(`RC, regC, regB, fld5, `ALUOP_COPY);
                    4'hA: alu_apply(`RA, regA, regC, fld5, `ALUOP_COPY);
                    4'hB: alu_apply(`RC, regC, regD, fld5, `ALUOP_COPY);
                    4'hC: alu_exch(`RA, `RB, fld5);
                    4'hD: alu_exch(`RB, `RC, fld5);
                    4'hE: alu_exch(`RA, `RC, fld5);
                    4'hF: alu_exch(`RC, `RD, fld5);
                    endcase
                end
                new_pc = PC + 20'd3;
            end
            // ═══════════════════════════ GROUP B : field arith (SUB/INC or SHIFT/NEG) ══════
            4'hB: begin
                if (n1 < 4'd8) begin
                    fld5 = {1'b0, n1};
                    case (n2)
                    4'h0: alu_apply(`RA, regA, regB, fld5, `ALUOP_SUB);
                    4'h1: alu_apply(`RB, regB, regC, fld5, `ALUOP_SUB);
                    4'h2: alu_apply(`RC, regC, regA, fld5, `ALUOP_SUB);
                    4'h3: alu_apply(`RD, regD, regC, fld5, `ALUOP_SUB);
                    4'h4: alu_apply(`RA, regA, 64'd0, fld5, `ALUOP_INC);
                    4'h5: alu_apply(`RB, regB, 64'd0, fld5, `ALUOP_INC);
                    4'h6: alu_apply(`RC, regC, 64'd0, fld5, `ALUOP_INC);
                    4'h7: alu_apply(`RD, regD, 64'd0, fld5, `ALUOP_INC);
                    4'h8: alu_apply(`RB, regB, regA, fld5, `ALUOP_SUB);
                    4'h9: alu_apply(`RC, regC, regB, fld5, `ALUOP_SUB);
                    4'hA: alu_apply(`RA, regA, regC, fld5, `ALUOP_SUB);
                    4'hB: alu_apply(`RC, regC, regD, fld5, `ALUOP_SUB);
                    4'hC: alu_apply(`RA, regB, regA, fld5, `ALUOP_SUB); // A=B-A
                    4'hD: alu_apply(`RB, regC, regB, fld5, `ALUOP_SUB); // B=C-B
                    4'hE: alu_apply(`RC, regA, regC, fld5, `ALUOP_SUB); // C=A-C
                    4'hF: alu_apply(`RD, regC, regD, fld5, `ALUOP_SUB); // D=C-D
                    endcase
                end else begin
                    fld5 = {1'b0, n1 & 4'h7};
                    case (n2)
                    4'h0: alu_apply(`RA, regA, 64'd0, fld5, `ALUOP_SHL);
                    4'h1: alu_apply(`RB, regB, 64'd0, fld5, `ALUOP_SHL);
                    4'h2: alu_apply(`RC, regC, 64'd0, fld5, `ALUOP_SHL);
                    4'h3: alu_apply(`RD, regD, 64'd0, fld5, `ALUOP_SHL);
                    4'h4: begin alu_a=regA; alu_b=64'd0; alu_field=fld5; alu_op=`ALUOP_SHR; #0; regA=alu_res; if (alu_cout) ST[`SB] = 1'b1; end
                    4'h5: begin alu_a=regB; alu_b=64'd0; alu_field=fld5; alu_op=`ALUOP_SHR; #0; regB=alu_res; if (alu_cout) ST[`SB] = 1'b1; end
                    4'h6: begin alu_a=regC; alu_b=64'd0; alu_field=fld5; alu_op=`ALUOP_SHR; #0; regC=alu_res; if (alu_cout) ST[`SB] = 1'b1; end
                    4'h7: begin alu_a=regD; alu_b=64'd0; alu_field=fld5; alu_op=`ALUOP_SHR; #0; regD=alu_res; if (alu_cout) ST[`SB] = 1'b1; end
                    4'h8: alu_apply(`RA, regA, 64'd0, fld5, `ALUOP_NEG2);
                    4'h9: alu_apply(`RB, regB, 64'd0, fld5, `ALUOP_NEG2);
                    4'hA: alu_apply(`RC, regC, 64'd0, fld5, `ALUOP_NEG2);
                    4'hB: alu_apply(`RD, regD, 64'd0, fld5, `ALUOP_NEG2);
                    4'hC: alu_apply(`RA, regA, 64'd0, fld5, `ALUOP_NEG1);
                    4'hD: alu_apply(`RB, regB, 64'd0, fld5, `ALUOP_NEG1);
                    4'hE: alu_apply(`RC, regC, 64'd0, fld5, `ALUOP_NEG1);
                    4'hF: alu_apply(`RD, regD, 64'd0, fld5, `ALUOP_NEG1);
                    endcase
                end
                new_pc = PC + 20'd3;
            end
            // ═══════════════════════════ GROUP C : A-field ADD/DEC ══════════
            4'hC: begin
                case (n1)
                4'h0: alu_apply(`RA, regA, regB, `FC_A, `ALUOP_ADD);
                4'h1: alu_apply(`RB, regB, regC, `FC_A, `ALUOP_ADD);
                4'h2: alu_apply(`RC, regC, regA, `FC_A, `ALUOP_ADD);
                4'h3: alu_apply(`RD, regD, regC, `FC_A, `ALUOP_ADD);
                4'h4: alu_apply(`RA, regA, regA, `FC_A, `ALUOP_ADD);
                4'h5: alu_apply(`RB, regB, regB, `FC_A, `ALUOP_ADD);
                4'h6: alu_apply(`RC, regC, regC, `FC_A, `ALUOP_ADD);
                4'h7: alu_apply(`RD, regD, regD, `FC_A, `ALUOP_ADD);
                4'h8: alu_apply(`RB, regB, regA, `FC_A, `ALUOP_ADD);
                4'h9: alu_apply(`RC, regC, regB, `FC_A, `ALUOP_ADD);
                4'hA: alu_apply(`RA, regA, regC, `FC_A, `ALUOP_ADD);
                4'hB: alu_apply(`RC, regC, regD, `FC_A, `ALUOP_ADD);
                4'hC: alu_apply(`RA, regA, 64'd0, `FC_A, `ALUOP_DEC);
                4'hD: alu_apply(`RB, regB, 64'd0, `FC_A, `ALUOP_DEC);
                4'hE: alu_apply(`RC, regC, 64'd0, `FC_A, `ALUOP_DEC);
                4'hF: alu_apply(`RD, regD, 64'd0, `FC_A, `ALUOP_DEC);
                endcase
                new_pc = PC + 20'd2;
            end
            // ═══════════════════════════ GROUP D : A-field ZERO/COPY/EXCH ═══
            4'hD: begin
                case (n1)
                4'h0: alu_apply(`RA, regA, 64'd0, `FC_A, `ALUOP_ZERO);
                4'h1: alu_apply(`RB, regB, 64'd0, `FC_A, `ALUOP_ZERO);
                4'h2: alu_apply(`RC, regC, 64'd0, `FC_A, `ALUOP_ZERO);
                4'h3: alu_apply(`RD, regD, 64'd0, `FC_A, `ALUOP_ZERO);
                4'h4: alu_apply(`RA, regA, regB, `FC_A, `ALUOP_COPY); // A=B
                4'h5: alu_apply(`RB, regB, regC, `FC_A, `ALUOP_COPY);
                4'h6: alu_apply(`RC, regC, regA, `FC_A, `ALUOP_COPY);
                4'h7: alu_apply(`RD, regD, regC, `FC_A, `ALUOP_COPY);
                4'h8: alu_apply(`RB, regB, regA, `FC_A, `ALUOP_COPY);
                4'h9: alu_apply(`RC, regC, regB, `FC_A, `ALUOP_COPY);
                4'hA: alu_apply(`RA, regA, regC, `FC_A, `ALUOP_COPY);
                4'hB: alu_apply(`RC, regC, regD, `FC_A, `ALUOP_COPY);
                4'hC: alu_exch(`RA, `RB, `FC_A);
                4'hD: alu_exch(`RB, `RC, `FC_A);
                4'hE: alu_exch(`RA, `RC, `FC_A);
                4'hF: alu_exch(`RC, `RD, `FC_A);
                endcase
                new_pc = PC + 20'd2;
            end
            // ═══════════════════════════ GROUP E : A-field SUB/INC ══════════
            4'hE: begin
                case (n1)
                4'h0: alu_apply(`RA, regA, regB, `FC_A, `ALUOP_SUB);
                4'h1: alu_apply(`RB, regB, regC, `FC_A, `ALUOP_SUB);
                4'h2: alu_apply(`RC, regC, regA, `FC_A, `ALUOP_SUB);
                4'h3: alu_apply(`RD, regD, regC, `FC_A, `ALUOP_SUB);
                4'h4: alu_apply(`RA, regA, 64'd0, `FC_A, `ALUOP_INC);
                4'h5: alu_apply(`RB, regB, 64'd0, `FC_A, `ALUOP_INC);
                4'h6: alu_apply(`RC, regC, 64'd0, `FC_A, `ALUOP_INC);
                4'h7: alu_apply(`RD, regD, 64'd0, `FC_A, `ALUOP_INC);
                4'h8: alu_apply(`RB, regB, regA, `FC_A, `ALUOP_SUB);
                4'h9: alu_apply(`RC, regC, regB, `FC_A, `ALUOP_SUB);
                4'hA: alu_apply(`RA, regA, regC, `FC_A, `ALUOP_SUB);
                4'hB: alu_apply(`RC, regC, regD, `FC_A, `ALUOP_SUB);
                4'hC: alu_apply(`RA, regB, regA, `FC_A, `ALUOP_SUB);
                4'hD: alu_apply(`RB, regC, regB, `FC_A, `ALUOP_SUB);
                4'hE: alu_apply(`RC, regA, regC, `FC_A, `ALUOP_SUB);
                4'hF: alu_apply(`RD, regC, regD, `FC_A, `ALUOP_SUB);
                endcase
                new_pc = PC + 20'd2;
            end
            // ═══════════════════════════ GROUP F : A-field SHIFTS / NEG ═════
            4'hF: begin
                case (n1)
                4'h0: alu_apply(`RA, regA, 64'd0, `FC_A, `ALUOP_SHL);
                4'h1: alu_apply(`RB, regB, 64'd0, `FC_A, `ALUOP_SHL);
                4'h2: alu_apply(`RC, regC, 64'd0, `FC_A, `ALUOP_SHL);
                4'h3: alu_apply(`RD, regD, 64'd0, `FC_A, `ALUOP_SHL);
                4'h4: begin alu_a=regA; alu_b=64'd0; alu_field=`FC_A; alu_op=`ALUOP_SHR; #0; regA=alu_res; if (alu_cout) ST[`SB] = 1'b1; end
                4'h5: begin alu_a=regB; alu_b=64'd0; alu_field=`FC_A; alu_op=`ALUOP_SHR; #0; regB=alu_res; if (alu_cout) ST[`SB] = 1'b1; end
                4'h6: begin alu_a=regC; alu_b=64'd0; alu_field=`FC_A; alu_op=`ALUOP_SHR; #0; regC=alu_res; if (alu_cout) ST[`SB] = 1'b1; end
                4'h7: begin alu_a=regD; alu_b=64'd0; alu_field=`FC_A; alu_op=`ALUOP_SHR; #0; regD=alu_res; if (alu_cout) ST[`SB] = 1'b1; end
                4'h8: alu_apply(`RA, regA, 64'd0, `FC_A, `ALUOP_NEG2);
                4'h9: alu_apply(`RB, regB, 64'd0, `FC_A, `ALUOP_NEG2);
                4'hA: alu_apply(`RC, regC, 64'd0, `FC_A, `ALUOP_NEG2);
                4'hB: alu_apply(`RD, regD, 64'd0, `FC_A, `ALUOP_NEG2);
                4'hC: alu_apply(`RA, regA, 64'd0, `FC_A, `ALUOP_NEG1);
                4'hD: alu_apply(`RB, regB, 64'd0, `FC_A, `ALUOP_NEG1);
                4'hE: alu_apply(`RC, regC, 64'd0, `FC_A, `ALUOP_NEG1);
                4'hF: alu_apply(`RD, regD, 64'd0, `FC_A, `ALUOP_NEG1);
                endcase
                new_pc = PC + 20'd2;
            end
            default: ill = 1'b1;
            endcase

            if (ill) new_pc = PC + 20'd1;
            PC = new_pc & 20'hFFFFF;
        end
    endtask

    // ── FSM driver ───────────────────────────────────────
    always @(posedge clk or negedge rst_n) begin : fsm
        integer i;
        if (!rst_n) begin
            state<=S_IDLE; PC<=20'd0;
            regA<=64'd0; regB<=64'd0; regC<=64'd0; regD<=64'd0;
            regR[0]<=64'd0; regR[1]<=64'd0; regR[2]<=64'd0; regR[3]<=64'd0; regR[4]<=64'd0;
            D0<=20'd0; D1<=20'd0;
            P<=4'd0; ST<=4'd0; PSTAT<=16'd0; HST<=16'd0;
            IN_REG<=64'd0; OUT_REG<=64'd0;
            CARRY<=1'b0; HEXMODE<=1'b1;
            RSTKP <= -4'sd1;
            for (i=0;i<8;i=i+1) RSTK[i]<=20'd0;
            mem_addr<=20'd0; mem_we<=1'b0; mem_re<=1'b0; mem_wdata<=4'd0;
            step_done<=1'b0;
            fetch_i<=6'd0; mem_i<=5'd0;
        end else begin
            step_done<=1'b0;
            case (state)
            S_IDLE: begin
                mem_we<=1'b0; mem_re<=1'b0;
                if (step) begin
                    fetch_i<=6'd0;
                    mem_addr<=PC;
                    mem_re<=1'b1;
                    state<=S_FETCH;
                end
            end
            S_FETCH: begin
                ib[fetch_i] <= mem_rdata;
                if (fetch_i == 6'd31) begin
                    mem_re<=1'b0;
                    state <=S_EXEC;
                end else begin
                    mem_addr <= PC + {14'd0, fetch_i} + 20'd1;
                    fetch_i  <= fetch_i + 6'd1;
                end
            end
            S_EXEC: begin
                execute;
                if (do_memop) begin
                    mem_i <= 5'd0;
                    if (memop_is_write) begin
                        mem_addr  <= memop_addr;
                        mem_wdata <= memop_data[memop_start_nib * 4 +: 4];
                        mem_we    <= 1'b1;
                        state     <= S_MEMW;
                    end else begin
                        mem_addr <= memop_addr;
                        mem_re   <= 1'b1;
                        state    <= S_MEMR;
                    end
                end else begin
                    state     <= S_DONE;
                    step_done <= 1'b1;
                end
            end
            S_MEMW: begin
                if (mem_i == memop_len - 5'd1) begin
                    mem_we    <= 1'b0;
                    state     <= S_DONE;
                    step_done <= 1'b1;
                end else begin
                    mem_i     <= mem_i + 5'd1;
                    mem_addr  <= memop_addr + {15'd0, mem_i} + 20'd1;
                    mem_wdata <= memop_data[(memop_start_nib + mem_i + 4'd1) * 4 +: 4];
                end
            end
            S_MEMR: begin
                case (memop_dst_reg)
                2'd0: regA[(memop_start_nib + mem_i) * 4 +: 4] <= mem_rdata;
                2'd2: regC[(memop_start_nib + mem_i) * 4 +: 4] <= mem_rdata;
                default: ;
                endcase
                if (mem_i == memop_len - 5'd1) begin
                    mem_re    <= 1'b0;
                    state     <= S_DONE;
                    step_done <= 1'b1;
                end else begin
                    mem_i    <= mem_i + 5'd1;
                    mem_addr <= memop_addr + {15'd0, mem_i} + 20'd1;
                end
            end
            S_DONE: state <= S_IDLE;
            endcase
        end
    end
endmodule
