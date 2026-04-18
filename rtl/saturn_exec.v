/**
 * @file saturn_exec.v
 * @brief Pure combinational instruction decoder and executer for the
 *        HP Saturn CPU.
 *
 * One big `always @*` block. Takes all current architectural state plus
 * the pre-fetched instruction nibble buffer (ib0..ib23) as inputs,
 * drives the ALU control signals (alu_a / alu_b / alu_field / alu_op),
 * consumes `alu_res / alu_res_b / alu_cout`, and produces `*_next`
 * values for every state element the FSM can update in a single
 * instruction. The FSM in saturn_cpu.v latches those next values on the
 * cycle it spends in S_EXEC.
 *
 * Dispatch mirrors `x48ng/src/core/emulate.c`:
 *  - top nibble (n0) picks the opcode group 0x0..0xF
 *  - some groups sub-dispatch on n1, then n2, then n3
 *  - LC (group 3) and LA (0x808_2) constant-load instructions unroll a
 *    16-nibble copy loop over ib[]
 *
 * No tasks, no `#0` delays — fully synthesizable. The ALU sits outside
 * this module (in saturn_cpu) so the combinational chain
 * @code
 *   state+ib → alu_a,b,field,op → saturn_alu → alu_res → *_next
 * @endcode
 * has a clean feed-forward dependency, no simulator re-evaluation
 * trickery.
 */
`include "saturn_pkg.vh"

/**
 * @brief Saturn instruction decoder / executer (combinational).
 *
 * All outputs are updated in a single `always @*`. For instructions
 * that don't use the ALU, the ALU outputs are `alu_op=ALUOP_PASS`,
 * `alu_a=regA_in` so the ALU computes a harmless value that nothing
 * downstream consumes.
 */
module saturn_exec (
    // ── current state ────────────────────────────────────────────────
    input  wire [19:0] pc_in,     ///< current PC
    input  wire [63:0] regA_in, regB_in, regC_in, regD_in,  ///< working regs
    input  wire [63:0] regR0_in, regR1_in, regR2_in, regR3_in, regR4_in, ///< scratch regs
    input  wire [19:0] D0_in, D1_in,     ///< data pointers
    input  wire [3:0]  P_in,             ///< P register
    input  wire [3:0]  ST_in,            ///< ST status bits
    input  wire [15:0] PSTAT_in,         ///< 16-bit program-status
    input  wire        CARRY_in,         ///< carry flag
    input  wire        HEXMODE_in,       ///< 1=hex, 0=dec
    input  wire [19:0] RSTK_top_in,      ///< top of return stack (0 if empty)
    input  wire [63:0] IN_REG_in,        ///< latched IN_REG (keyboard scan)
    input  wire [63:0] OUT_REG_in,       ///< OUT_REG shadow

    // ── instruction bytes (nibbles, PC..PC+23) ───────────────────────
    input  wire [3:0]  ib0, ib1, ib2, ib3, ib4, ib5, ib6, ib7,
    ///< @brief extra nibbles for LC / LA constant loads (up to 22-nib ops).
    input  wire [3:0]  ib8,  ib9,  ib10, ib11, ib12, ib13, ib14, ib15,
    input  wire [3:0]  ib16, ib17, ib18, ib19, ib20, ib21, ib22, ib23,

    // ── ALU bidirectional interface ──────────────────────────────────
    output reg  [63:0] alu_a,     ///< ALU operand a (drives external saturn_alu)
    output reg  [63:0] alu_b,     ///< ALU operand b
    output reg  [4:0]  alu_field, ///< ALU field code
    output reg  [5:0]  alu_op,    ///< ALU op code
    input  wire [63:0] alu_res,   ///< ALU primary result (combinational)
    input  wire [63:0] alu_res_b, ///< ALU secondary result (for EXCH)
    input  wire        alu_cout,  ///< ALU carry / compare output
    input  wire [3:0]  alu_fs,    ///< decoded field start nibble
    input  wire [3:0]  alu_fe,    ///< decoded field end nibble

    // ── next-state outputs (latched by saturn_cpu on S_EXEC edge) ────
    output reg  [19:0] pc_next,       ///< next PC
    output reg  [63:0] regA_next, regB_next, regC_next, regD_next,
    output reg  [63:0] regR0_next, regR1_next, regR2_next, regR3_next, regR4_next,
    output reg  [19:0] D0_next, D1_next,
    output reg  [3:0]  P_next,
    output reg  [3:0]  ST_next,
    output reg  [15:0] PSTAT_next,
    output reg         CARRY_next,
    output reg         HEXMODE_next,
    output reg  [63:0] OUT_REG_next,
    output reg         rstk_push_en,   ///< 1 = push rstk_push_val this step
    output reg  [19:0] rstk_push_val,
    output reg         rstk_pop_en,    ///< 1 = pop top of RSTK this step

    // ── memory-op request (handed to S_MEMR / S_MEMW in the FSM) ────
    output reg         do_memop,         ///< 1 = instruction needs a memory transaction
    output reg         memop_is_write,   ///< 1 = store (reg → mem), 0 = recall
    output reg  [19:0] memop_addr,       ///< base address (nibble-aligned)
    output reg  [63:0] memop_data,       ///< full reg snapshot (store only)
    output reg  [1:0]  memop_dst_reg,    ///< destination reg for recall: 0=A, 2=C
    output reg  [4:0]  memop_len,        ///< number of nibbles to transfer
    output reg  [3:0]  memop_start_nib   ///< starting nibble offset within reg
);

    // ── helpers ──────────────────────────────────────────────────────
    function [19:0] sext8;  input [7:0]  v; sext8  = {{12{v[7]}},  v}; endfunction
    function [19:0] sext12; input [11:0] v; sext12 = {{8{v[11]}},  v}; endfunction
    function [19:0] sext16; input [15:0] v; sext16 = {{4{v[15]}},  v}; endfunction

    function [4:0] fld_len;
        input [4:0] fcode;
        begin
            case (fcode)
                `FC_P:   fld_len = 5'd1;
                `FC_WP:  fld_len = {1'b0, P_in} + 5'd1;
                `FC_XS:  fld_len = 5'd1;
                `FC_X:   fld_len = 5'd3;
                `FC_S:   fld_len = 5'd1;
                `FC_M:   fld_len = 5'd12;
                `FC_B:   fld_len = 5'd2;
                `FC_W:   fld_len = 5'd16;
                `FC_A:   fld_len = 5'd5;
                default: fld_len = 5'd1;
            endcase
        end
    endfunction

    // Field merge: return `dest` with nibbles [fs..fe] replaced by `src`.
    // All combinational, one bit-mask per call.
    function [63:0] field_merge;
        input [63:0] dest, src;
        input [3:0]  fs, fe;
        integer i;
        begin
            field_merge = dest;
            for (i = 0; i < 16; i = i + 1)
                if (i[3:0] >= fs && i[3:0] <= fe)
                    field_merge[i*4 +: 4] = src[i*4 +: 4];
        end
    endfunction

    // Replace nibble at index `idx` with `nib`.
    function [63:0] set_nibble;
        input [63:0] r;
        input [3:0]  idx;
        input [3:0]  nib;
        begin
            set_nibble = r;
            set_nibble[idx*4 +: 4] = nib;
        end
    endfunction

    // Set/clear a single bit at index `idx` (0..63) of r.
    function [63:0] set_bit;
        input [63:0] r;
        input [3:0]  idx;
        input        val;
        begin
            set_bit = r;
            set_bit[idx] = val;
        end
    endfunction

    // ── short aliases for the instruction nibbles ────────────────────
    wire [3:0] n0 = ib0, n1 = ib1, n2 = ib2, n3 = ib3;
    wire [3:0] n4 = ib4, n5 = ib5, n6 = ib6;

    // ── local working state for the LC-constant load ─────────────────
    // (Plain combinational logic that overlays constants onto regC[P..].)
    reg [63:0] lc_regC;
    reg [3:0]  lc_pos;
    integer    lc_k;

    // ── local working state for the LA-constant load ─────────────────
    reg [63:0] la_regA;
    reg [3:0]  la_pos;
    integer    la_k;

    // ── branch target helpers ────────────────────────────────────────
    reg [19:0] br_tgt;

    // ── main dispatch ────────────────────────────────────────────────
    always @* begin
        // Defaults: no architectural state change.
        pc_next      = pc_in;
        regA_next    = regA_in;  regB_next  = regB_in;
        regC_next    = regC_in;  regD_next  = regD_in;
        regR0_next   = regR0_in; regR1_next = regR1_in;
        regR2_next   = regR2_in; regR3_next = regR3_in; regR4_next = regR4_in;
        D0_next      = D0_in;    D1_next    = D1_in;
        P_next       = P_in;
        ST_next      = ST_in;    PSTAT_next = PSTAT_in;
        CARRY_next   = CARRY_in; HEXMODE_next = HEXMODE_in;
        OUT_REG_next = OUT_REG_in;
        rstk_push_en = 1'b0; rstk_push_val = 20'd0; rstk_pop_en = 1'b0;

        do_memop        = 1'b0;
        memop_is_write  = 1'b0;
        memop_addr      = 20'd0;
        memop_data      = 64'd0;
        memop_dst_reg   = 2'd0;
        memop_len       = 5'd0;
        memop_start_nib = 4'd0;

        // Default ALU drive: harmless identity op.
        alu_a      = regA_in;
        alu_b      = 64'd0;
        alu_field  = `FC_W;
        alu_op     = `ALUOP_PASS;

        // LC / LA scratch
        lc_regC = regC_in;  lc_pos = P_in;
        la_regA = regA_in;  la_pos = P_in;

        br_tgt = 20'd0;

        case (n0)
        // ═══════════════════════════ GROUP 0 ═══════════════════════════
        4'h0: begin
            case (n1)
            4'h0: begin ST_next[`XM]=1'b1; pc_next=RSTK_top_in; rstk_pop_en=1'b1; end // RTNSXM
            4'h1: begin pc_next=RSTK_top_in; rstk_pop_en=1'b1; end                   // RTN
            4'h2: begin CARRY_next=1'b1; pc_next=RSTK_top_in; rstk_pop_en=1'b1; end  // RTNSC
            4'h3: begin CARRY_next=1'b0; pc_next=RSTK_top_in; rstk_pop_en=1'b1; end  // RTNCC
            4'h4: begin HEXMODE_next=1'b1; pc_next=pc_in+20'd2; end                  // SETHEX
            4'h5: begin HEXMODE_next=1'b0; pc_next=pc_in+20'd2; end                  // SETDEC
            4'h6: begin rstk_push_en=1'b1; rstk_push_val=regC_in[19:0]; pc_next=pc_in+20'd2; end // RSTK=C
            4'h7: begin regC_next[19:0]=RSTK_top_in; rstk_pop_en=1'b1; pc_next=pc_in+20'd2; end  // C=RSTK
            4'h8: begin PSTAT_next = PSTAT_in & 16'hF000; pc_next=pc_in+20'd2; end   // CLRST
            4'h9: begin regC_next[11:0] = PSTAT_in[11:0]; pc_next=pc_in+20'd2; end   // C=ST
            4'hA: begin PSTAT_next[11:0] = regC_in[11:0]; pc_next=pc_in+20'd2; end   // ST=C
            4'hB: begin                                                               // CSTEX
                PSTAT_next[11:0] = regC_in[11:0];
                regC_next[11:0]  = PSTAT_in[11:0];
                pc_next=pc_in+20'd2;
            end
            4'hC: begin                                                               // P=P+1
                if (P_in==4'd15) begin P_next=4'd0;  CARRY_next=1'b1; end
                else             begin P_next=P_in+4'd1; CARRY_next=1'b0; end
                pc_next=pc_in+20'd2;
            end
            4'hD: begin                                                               // P=P-1
                if (P_in==4'd0)  begin P_next=4'd15; CARRY_next=1'b1; end
                else             begin P_next=P_in-4'd1; CARRY_next=1'b0; end
                pc_next=pc_in+20'd2;
            end
            4'hE: begin                                                               // 0E fs op : AND/OR
                alu_field = {1'b0, n2};
                alu_op    = (n3[3]) ? `ALUOP_OR : `ALUOP_AND;
                case (n3)
                4'h0, 4'h8: begin alu_a=regA_in; alu_b=regB_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); end
                4'h1, 4'h9: begin alu_a=regB_in; alu_b=regC_in; regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); end
                4'h2, 4'hA: begin alu_a=regC_in; alu_b=regA_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); end
                4'h3, 4'hB: begin alu_a=regD_in; alu_b=regC_in; regD_next=field_merge(regD_in,alu_res,alu_fs,alu_fe); end
                4'h4, 4'hC: begin alu_a=regB_in; alu_b=regA_in; regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); end
                4'h5, 4'hD: begin alu_a=regC_in; alu_b=regB_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); end
                4'h6, 4'hE: begin alu_a=regA_in; alu_b=regC_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); end
                4'h7, 4'hF: begin alu_a=regC_in; alu_b=regD_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); end
                endcase
                pc_next = pc_in + 20'd4;
            end
            4'hF: begin pc_next=RSTK_top_in; rstk_pop_en=1'b1; end                   // RTI
            endcase
        end

        // ═══════════════════════════ GROUP 1 ═══════════════════════════
        4'h1: begin
            case (n1)
            4'h0: begin // 10x : R{x}=A or R{x}=C (W-field)
                case (n2)
                4'h0:            regR0_next = regA_in;
                4'h1, 4'h5:      regR1_next = regA_in;
                4'h2, 4'h6:      regR2_next = regA_in;
                4'h3, 4'h7:      regR3_next = regA_in;
                4'h4:            regR4_next = regA_in;
                4'h8:            regR0_next = regC_in;
                4'h9, 4'hD:      regR1_next = regC_in;
                4'hA, 4'hE:      regR2_next = regC_in;
                4'hB, 4'hF:      regR3_next = regC_in;
                4'hC:            regR4_next = regC_in;
                endcase
                pc_next = pc_in + 20'd3;
            end
            4'h1: begin // 11x : A=R{x} or C=R{x}
                case (n2)
                4'h0:            regA_next = regR0_in;
                4'h1, 4'h5:      regA_next = regR1_in;
                4'h2, 4'h6:      regA_next = regR2_in;
                4'h3, 4'h7:      regA_next = regR3_in;
                4'h4:            regA_next = regR4_in;
                4'h8:            regC_next = regR0_in;
                4'h9, 4'hD:      regC_next = regR1_in;
                4'hA, 4'hE:      regC_next = regR2_in;
                4'hB, 4'hF:      regC_next = regR3_in;
                4'hC:            regC_next = regR4_in;
                endcase
                pc_next = pc_in + 20'd3;
            end
            4'h2: begin // 12x : AR{x}EX / CR{x}EX (W-field swap)
                case (n2)
                4'h0:            begin regA_next = regR0_in; regR0_next = regA_in; end
                4'h1, 4'h5:      begin regA_next = regR1_in; regR1_next = regA_in; end
                4'h2, 4'h6:      begin regA_next = regR2_in; regR2_next = regA_in; end
                4'h3, 4'h7:      begin regA_next = regR3_in; regR3_next = regA_in; end
                4'h4:            begin regA_next = regR4_in; regR4_next = regA_in; end
                4'h8:            begin regC_next = regR0_in; regR0_next = regC_in; end
                4'h9, 4'hD:      begin regC_next = regR1_in; regR1_next = regC_in; end
                4'hA, 4'hE:      begin regC_next = regR2_in; regR2_next = regC_in; end
                4'hB, 4'hF:      begin regC_next = regR3_in; regR3_next = regC_in; end
                4'hC:            begin regC_next = regR4_in; regR4_next = regC_in; end
                endcase
                pc_next = pc_in + 20'd3;
            end
            4'h3: begin // 13x : D0/D1 / AD0EX etc.
                case (n2)
                4'h0: D0_next = regA_in[19:0];                                        // D0=A
                4'h1: D1_next = regA_in[19:0];                                        // D1=A
                4'h2: begin D0_next = regA_in[19:0]; regA_next[19:0] = D0_in; end     // AD0EX
                4'h3: begin D1_next = regA_in[19:0]; regA_next[19:0] = D1_in; end     // AD1EX
                4'h4: D0_next = regC_in[19:0];                                        // D0=C
                4'h5: D1_next = regC_in[19:0];                                        // D1=C
                4'h6: begin D0_next = regC_in[19:0]; regC_next[19:0] = D0_in; end     // CD0EX
                4'h7: begin D1_next = regC_in[19:0]; regC_next[19:0] = D1_in; end     // CD1EX
                4'h8: D0_next[15:0] = regA_in[15:0];                                  // D0=AS
                4'h9: D1_next[15:0] = regA_in[15:0];                                  // D1=AS
                4'hA: begin D0_next[15:0] = regA_in[15:0]; regA_next[15:0] = D0_in[15:0]; end // AD0XS
                4'hB: begin D1_next[15:0] = regA_in[15:0]; regA_next[15:0] = D1_in[15:0]; end // AD1XS
                4'hC: D0_next[15:0] = regC_in[15:0];                                  // D0=CS
                4'hD: D1_next[15:0] = regC_in[15:0];                                  // D1=CS
                4'hE: begin D0_next[15:0] = regC_in[15:0]; regC_next[15:0] = D0_in[15:0]; end // CD0XS
                4'hF: begin D1_next[15:0] = regC_in[15:0]; regC_next[15:0] = D1_in[15:0]; end // CD1XS
                endcase
                pc_next = pc_in + 20'd3;
            end
            4'h4: begin // DAT recall/store 3-nibble: W (n2<8) or B (n2>=8)
                case (n2 & 4'h7)
                4'h0: begin memop_is_write=1'b1; memop_dst_reg=`RA; memop_addr=D0_in; memop_data=regA_in; end
                4'h1: begin memop_is_write=1'b1; memop_dst_reg=`RA; memop_addr=D1_in; memop_data=regA_in; end
                4'h2: begin memop_is_write=1'b0; memop_dst_reg=`RA; memop_addr=D0_in; end
                4'h3: begin memop_is_write=1'b0; memop_dst_reg=`RA; memop_addr=D1_in; end
                4'h4: begin memop_is_write=1'b1; memop_dst_reg=`RC; memop_addr=D0_in; memop_data=regC_in; end
                4'h5: begin memop_is_write=1'b1; memop_dst_reg=`RC; memop_addr=D1_in; memop_data=regC_in; end
                4'h6: begin memop_is_write=1'b0; memop_dst_reg=`RC; memop_addr=D0_in; end
                4'h7: begin memop_is_write=1'b0; memop_dst_reg=`RC; memop_addr=D1_in; end
                endcase
                memop_len       = (n2 < 4'd8) ? 5'd5 : 5'd2;
                memop_start_nib = 4'd0;
                do_memop        = 1'b1;
                pc_next         = pc_in + 20'd3;
            end
            4'h5: begin // DAT recall/store 4-nibble: 1 5 op3 op4
                case (n2 & 4'h7)
                4'h0: begin memop_is_write=1'b1; memop_dst_reg=`RA; memop_addr=D0_in; memop_data=regA_in; end
                4'h1: begin memop_is_write=1'b1; memop_dst_reg=`RA; memop_addr=D1_in; memop_data=regA_in; end
                4'h2: begin memop_is_write=1'b0; memop_dst_reg=`RA; memop_addr=D0_in; end
                4'h3: begin memop_is_write=1'b0; memop_dst_reg=`RA; memop_addr=D1_in; end
                4'h4: begin memop_is_write=1'b1; memop_dst_reg=`RC; memop_addr=D0_in; memop_data=regC_in; end
                4'h5: begin memop_is_write=1'b1; memop_dst_reg=`RC; memop_addr=D1_in; memop_data=regC_in; end
                4'h6: begin memop_is_write=1'b0; memop_dst_reg=`RC; memop_addr=D0_in; end
                4'h7: begin memop_is_write=1'b0; memop_dst_reg=`RC; memop_addr=D1_in; end
                endcase
                if (n2 >= 4'd8) begin
                    memop_len       = {1'b0, n3} + 5'd1;
                    memop_start_nib = 4'd0;
                end else begin
                    memop_len = fld_len({1'b0, n3});
                    // start nibble per field code (mirror of registers.c
                    // start_fields[], taking the low 4 bits of the FC_*
                    // constants defined in saturn_pkg.vh).
                    case (n3)
                    4'd0:    memop_start_nib = P_in;   // FC_P
                    4'd2:    memop_start_nib = 4'd2;   // FC_XS
                    4'd4:    memop_start_nib = 4'd15;  // FC_S
                    4'd5:    memop_start_nib = 4'd3;   // FC_M
                    default: memop_start_nib = 4'd0;
                    endcase
                end
                do_memop = 1'b1;
                pc_next  = pc_in + 20'd4;
            end
            4'h6: begin {CARRY_next, D0_next} = {1'b0, D0_in} + {16'd0, n2} + 21'd1; pc_next = pc_in + 20'd3; end
            4'h7: begin {CARRY_next, D1_next} = {1'b0, D1_in} + {16'd0, n2} + 21'd1; pc_next = pc_in + 20'd3; end
            4'h8: begin {CARRY_next, D0_next} = {1'b0, D0_in} - ({16'd0, n2} + 21'd1); pc_next = pc_in + 20'd3; end
            4'h9: begin D0_next[7:0]  = {n3,n2};                 pc_next = pc_in + 20'd4; end
            4'hA: begin D0_next[15:0] = {n5,n4,n3,n2};           pc_next = pc_in + 20'd6; end
            4'hB: begin D0_next       = {ib6,n5,n4,n3,n2};       pc_next = pc_in + 20'd7; end
            4'hC: begin {CARRY_next, D1_next} = {1'b0, D1_in} - ({16'd0, n2} + 21'd1); pc_next = pc_in + 20'd3; end
            4'hD: begin D1_next[7:0]  = {n3,n2};                 pc_next = pc_in + 20'd4; end
            4'hE: begin D1_next[15:0] = {n5,n4,n3,n2};           pc_next = pc_in + 20'd6; end
            4'hF: begin D1_next       = {ib6,n5,n4,n3,n2};       pc_next = pc_in + 20'd7; end
            endcase
        end

        // ═══════════════════════════ GROUP 2 ═══════════════════════════
        4'h2: begin P_next = n1; pc_next = pc_in + 20'd2; end

        // ═══════════════════════════ GROUP 3 : LC ══════════════════════
        4'h3: begin
            // Load (n1+1) nibbles into regC starting at C[P], wrapping. P unchanged.
            lc_regC = regC_in;
            lc_pos  = P_in;
            // Unrolled max-16 nibble overlay, guarded by <= n1.
            for (lc_k = 0; lc_k < 16; lc_k = lc_k + 1) begin
                if (lc_k <= {28'd0, n1}) begin
                    case (lc_k)
                    0:  lc_regC[lc_pos*4 +: 4] = ib2;
                    1:  lc_regC[lc_pos*4 +: 4] = ib3;
                    2:  lc_regC[lc_pos*4 +: 4] = ib4;
                    3:  lc_regC[lc_pos*4 +: 4] = ib5;
                    4:  lc_regC[lc_pos*4 +: 4] = ib6;
                    5:  lc_regC[lc_pos*4 +: 4] = ib7;
                    6:  lc_regC[lc_pos*4 +: 4] = ib8;
                    7:  lc_regC[lc_pos*4 +: 4] = ib9;
                    8:  lc_regC[lc_pos*4 +: 4] = ib10;
                    9:  lc_regC[lc_pos*4 +: 4] = ib11;
                    10: lc_regC[lc_pos*4 +: 4] = ib12;
                    11: lc_regC[lc_pos*4 +: 4] = ib13;
                    12: lc_regC[lc_pos*4 +: 4] = ib14;
                    13: lc_regC[lc_pos*4 +: 4] = ib15;
                    14: lc_regC[lc_pos*4 +: 4] = ib16;
                    15: lc_regC[lc_pos*4 +: 4] = ib17;
                    endcase
                    lc_pos = (lc_pos == 4'd15) ? 4'd0 : lc_pos + 4'd1;
                end
            end
            regC_next = lc_regC;
            pc_next   = pc_in + 20'd3 + {15'd0, n1};
        end

        // ═══════════════════════════ GROUP 4 : JC (jump if carry) ══════
        4'h4: begin
            if ({n2, n1} == 8'h02)
                pc_next = pc_in + 20'd3;
            else if (CARRY_in) begin
                if ({n2, n1} != 8'h00)
                    pc_next = (pc_in + sext8({n2, n1}) + 20'd1) & 20'hFFFFF;
                else begin
                    pc_next = RSTK_top_in;
                    rstk_pop_en = 1'b1;
                end
            end else
                pc_next = pc_in + 20'd3;
        end

        // ═══════════════════════════ GROUP 5 : JNC ═════════════════════
        4'h5: begin
            if (!CARRY_in) begin
                if ({n2, n1} != 8'h00)
                    pc_next = (pc_in + sext8({n2, n1}) + 20'd1) & 20'hFFFFF;
                else begin
                    pc_next = RSTK_top_in;
                    rstk_pop_en = 1'b1;
                end
            end else
                pc_next = pc_in + 20'd3;
        end

        // ═══════════════════════════ GROUP 6 : GOTO (3-nib signed) ═════
        4'h6: begin
            if ({n3, n2, n1} == 12'h003)      pc_next = pc_in + 20'd4;
            else if ({n3, n2, n1} == 12'h004) pc_next = pc_in + 20'd5;
            else                              pc_next = (pc_in + sext12({n3, n2, n1}) + 20'd1) & 20'hFFFFF;
        end

        // ═══════════════════════════ GROUP 7 : GOSUB ═══════════════════
        4'h7: begin
            rstk_push_en  = 1'b1;
            rstk_push_val = pc_in + 20'd4;
            pc_next       = (pc_in + sext12({n3, n2, n1}) + 20'd4) & 20'hFFFFF;
        end

        // ═══════════════════════════ GROUP 8 ═══════════════════════════
        4'h8: begin
            case (n1)
            // ── 80x ──────────────────────────────────────────────────
            4'h0: begin
                case (n2)
                4'h0: begin OUT_REG_next[3:0]  = regC_in[3:0];  pc_next=pc_in+20'd3; end // OUT=CS
                4'h1: begin OUT_REG_next[11:0] = regC_in[11:0]; pc_next=pc_in+20'd3; end // OUT=C
                4'h2: begin regA_next[15:0] = IN_REG_in[15:0]; pc_next=pc_in+20'd3; end  // A=IN
                4'h3: begin regC_next[15:0] = IN_REG_in[15:0]; pc_next=pc_in+20'd3; end  // C=IN
                4'h4: pc_next = pc_in + 20'd3;                                           // UNCNFG stub
                4'h5: pc_next = pc_in + 20'd3;                                           // CONFIG stub
                4'h6: begin regC_next[19:0] = 20'd0; pc_next = pc_in + 20'd3; end        // C=ID stub
                4'h7: pc_next = pc_in + 20'd3;                                           // SHUTDN stub
                4'h8: begin                                                               // 0808 subdispatch
                    case (n3)
                    4'h0: pc_next = pc_in + 20'd4;                                       // INTON
                    4'h1: pc_next = pc_in + 20'd5;                                       // RSI
                    4'h2: begin                                                           // LA n : load A
                        la_regA = regA_in;
                        la_pos  = P_in;
                        for (la_k = 0; la_k < 16; la_k = la_k + 1) begin
                            if (la_k <= {28'd0, n4}) begin
                                case (la_k)
                                0:  la_regA[la_pos*4 +: 4] = ib5;
                                1:  la_regA[la_pos*4 +: 4] = ib6;
                                2:  la_regA[la_pos*4 +: 4] = ib7;
                                3:  la_regA[la_pos*4 +: 4] = ib8;
                                4:  la_regA[la_pos*4 +: 4] = ib9;
                                5:  la_regA[la_pos*4 +: 4] = ib10;
                                6:  la_regA[la_pos*4 +: 4] = ib11;
                                7:  la_regA[la_pos*4 +: 4] = ib12;
                                8:  la_regA[la_pos*4 +: 4] = ib13;
                                9:  la_regA[la_pos*4 +: 4] = ib14;
                                10: la_regA[la_pos*4 +: 4] = ib15;
                                11: la_regA[la_pos*4 +: 4] = ib16;
                                12: la_regA[la_pos*4 +: 4] = ib17;
                                13: la_regA[la_pos*4 +: 4] = ib18;
                                14: la_regA[la_pos*4 +: 4] = ib19;
                                15: la_regA[la_pos*4 +: 4] = ib20;
                                endcase
                                la_pos = (la_pos == 4'd15) ? 4'd0 : la_pos + 4'd1;
                            end
                        end
                        regA_next = la_regA;
                        pc_next   = pc_in + 20'd6 + {15'd0, n4};
                    end
                    4'h3: pc_next = pc_in + 20'd4;                                       // BUSCB
                    4'h4: begin regA_next = set_bit(regA_in, n4, 1'b0); pc_next = pc_in + 20'd5; end // ABIT=0 n
                    4'h5: begin regA_next = set_bit(regA_in, n4, 1'b1); pc_next = pc_in + 20'd5; end // ABIT=1 n
                    4'h6: begin                                                           // ?ABIT=0 n + br
                        CARRY_next = (regA_in[n4] == 1'b0);
                        if (CARRY_next) begin
                            if ({n6, n5} != 8'h00) pc_next = (pc_in + 20'd5 + sext8({n6, n5})) & 20'hFFFFF;
                            else begin pc_next = RSTK_top_in; rstk_pop_en = 1'b1; end
                        end else pc_next = pc_in + 20'd7;
                    end
                    4'h7: begin                                                           // ?ABIT=1 n + br
                        CARRY_next = (regA_in[n4] == 1'b1);
                        if (CARRY_next) begin
                            if ({n6, n5} != 8'h00) pc_next = (pc_in + 20'd5 + sext8({n6, n5})) & 20'hFFFFF;
                            else begin pc_next = RSTK_top_in; rstk_pop_en = 1'b1; end
                        end else pc_next = pc_in + 20'd7;
                    end
                    4'h8: begin regC_next = set_bit(regC_in, n4, 1'b0); pc_next = pc_in + 20'd5; end
                    4'h9: begin regC_next = set_bit(regC_in, n4, 1'b1); pc_next = pc_in + 20'd5; end
                    4'hA: begin                                                           // ?CBIT=0 n + br
                        CARRY_next = (regC_in[n4] == 1'b0);
                        if (CARRY_next) begin
                            if ({n6, n5} != 8'h00) pc_next = (pc_in + 20'd5 + sext8({n6, n5})) & 20'hFFFFF;
                            else begin pc_next = RSTK_top_in; rstk_pop_en = 1'b1; end
                        end else pc_next = pc_in + 20'd7;
                    end
                    4'hB: begin                                                           // ?CBIT=1 n + br
                        CARRY_next = (regC_in[n4] == 1'b1);
                        if (CARRY_next) begin
                            if ({n6, n5} != 8'h00) pc_next = (pc_in + 20'd5 + sext8({n6, n5})) & 20'hFFFFF;
                            else begin pc_next = RSTK_top_in; rstk_pop_en = 1'b1; end
                        end else pc_next = pc_in + 20'd7;
                    end
                    4'hC: pc_next = pc_in + 20'd4;                                        // PC=(A) stub
                    4'hD: pc_next = pc_in + 20'd4;                                        // BUSCD
                    4'hE: pc_next = pc_in + 20'd4;                                        // PC=(C) stub
                    4'hF: pc_next = pc_in + 20'd4;                                        // INTOFF
                    default: pc_next = pc_in + 20'd4;
                    endcase
                end
                4'h9: begin                                                               // C+P+1 (always hex)
                    alu_a     = regC_in;
                    alu_b     = {59'd0, P_in} + 64'd1;
                    alu_field = `FC_A;
                    alu_op    = `ALUOP_ADDCON;
                    regC_next  = field_merge(regC_in, alu_res, alu_fs, alu_fe);
                    CARRY_next = alu_cout;
                    pc_next    = pc_in + 20'd3;
                end
                4'hA: pc_next = pc_in + 20'd3; // RESET
                4'hB: pc_next = pc_in + 20'd3; // BUSCC
                4'hC: begin regC_next = set_nibble(regC_in, n3, P_in);         pc_next = pc_in + 20'd4; end // C=P n
                4'hD: begin P_next    = regC_in[{1'b0,n3}*4 +: 4];             pc_next = pc_in + 20'd4; end // P=C n
                4'hE: begin regC_next = set_nibble(regC_in, 4'd0, 4'd0);       pc_next = pc_in + 20'd3; end // SREQ?
                4'hF: begin                                                                                    // CPEX n
                    regC_next = set_nibble(regC_in, n3, P_in);
                    P_next    = regC_in[{1'b0,n3}*4 +: 4];
                    pc_next   = pc_in + 20'd4;
                end
                default: pc_next = pc_in + 20'd3;
                endcase
            end
            // ── 81x ──────────────────────────────────────────────────
            4'h1: begin
                case (n2)
                4'h0,4'h1,4'h2,4'h3: begin // ASLC/BSLC/CSLC/DSLC (W-field, left-circular)
                    alu_field = `FC_W;
                    alu_op    = `ALUOP_SHLC;
                    case (n2)
                    4'h0: begin alu_a=regA_in; regA_next = alu_res; end
                    4'h1: begin alu_a=regB_in; regB_next = alu_res; end
                    4'h2: begin alu_a=regC_in; regC_next = alu_res; end
                    4'h3: begin alu_a=regD_in; regD_next = alu_res; end
                    endcase
                    pc_next = pc_in + 20'd3;
                end
                4'h4,4'h5,4'h6,4'h7: begin // ASRC/BSRC/CSRC/DSRC
                    alu_field = `FC_W;
                    alu_op    = `ALUOP_SHRC;
                    case (n2)
                    4'h4: begin alu_a=regA_in; regA_next = alu_res; end
                    4'h5: begin alu_a=regB_in; regB_next = alu_res; end
                    4'h6: begin alu_a=regC_in; regC_next = alu_res; end
                    4'h7: begin alu_a=regD_in; regD_next = alu_res; end
                    endcase
                    if (alu_cout) ST_next[`SB] = 1'b1;
                    pc_next = pc_in + 20'd3;
                end
                4'h8: begin // 0818 : R = R +/- CON (6 nibbles), HEX
                    alu_field = {1'b0, n3};
                    alu_b     = 64'd0;
                    alu_b[4:0]= {1'b0, n5} + 5'd1;
                    alu_op    = (n4 < 4'd8) ? `ALUOP_ADDCON : `ALUOP_SUBCON;
                    case (n4[1:0])
                    2'd0: begin alu_a=regA_in; regA_next = field_merge(regA_in, alu_res, alu_fs, alu_fe); end
                    2'd1: begin alu_a=regB_in; regB_next = field_merge(regB_in, alu_res, alu_fs, alu_fe); end
                    2'd2: begin alu_a=regC_in; regC_next = field_merge(regC_in, alu_res, alu_fs, alu_fe); end
                    2'd3: begin alu_a=regD_in; regD_next = field_merge(regD_in, alu_res, alu_fs, alu_fe); end
                    endcase
                    CARRY_next = alu_cout;
                    pc_next = pc_in + 20'd6;
                end
                4'h9: begin // 0819 : R SRB fs
                    alu_field = {1'b0, n3};
                    alu_op    = `ALUOP_SHRB;
                    alu_b     = 64'd0;
                    case (n4[1:0])
                    2'd0: begin alu_a=regA_in; regA_next = field_merge(regA_in, alu_res, alu_fs, alu_fe); end
                    2'd1: begin alu_a=regB_in; regB_next = field_merge(regB_in, alu_res, alu_fs, alu_fe); end
                    2'd2: begin alu_a=regC_in; regC_next = field_merge(regC_in, alu_res, alu_fs, alu_fe); end
                    2'd3: begin alu_a=regD_in; regD_next = field_merge(regD_in, alu_res, alu_fs, alu_fe); end
                    endcase
                    if (alu_cout) ST_next[`SB] = 1'b1;
                    pc_next = pc_in + 20'd5;
                end
                4'hA: begin // 081A{n4} : R/A/C field ops
                    alu_field = {1'b0, n3};
                    case (n4)
                    4'h0: begin // R{x}={A,C}
                        alu_op = `ALUOP_COPY;
                        case (n5)
                        4'h0:      begin alu_a=regR0_in; alu_b=regA_in; regR0_next = field_merge(regR0_in, alu_res, alu_fs, alu_fe); end
                        4'h1,4'h5: begin alu_a=regR1_in; alu_b=regA_in; regR1_next = field_merge(regR1_in, alu_res, alu_fs, alu_fe); end
                        4'h2,4'h6: begin alu_a=regR2_in; alu_b=regA_in; regR2_next = field_merge(regR2_in, alu_res, alu_fs, alu_fe); end
                        4'h3,4'h7: begin alu_a=regR3_in; alu_b=regA_in; regR3_next = field_merge(regR3_in, alu_res, alu_fs, alu_fe); end
                        4'h4:      begin alu_a=regR4_in; alu_b=regA_in; regR4_next = field_merge(regR4_in, alu_res, alu_fs, alu_fe); end
                        4'h8:      begin alu_a=regR0_in; alu_b=regC_in; regR0_next = field_merge(regR0_in, alu_res, alu_fs, alu_fe); end
                        4'h9,4'hD: begin alu_a=regR1_in; alu_b=regC_in; regR1_next = field_merge(regR1_in, alu_res, alu_fs, alu_fe); end
                        4'hA,4'hE: begin alu_a=regR2_in; alu_b=regC_in; regR2_next = field_merge(regR2_in, alu_res, alu_fs, alu_fe); end
                        4'hB,4'hF: begin alu_a=regR3_in; alu_b=regC_in; regR3_next = field_merge(regR3_in, alu_res, alu_fs, alu_fe); end
                        4'hC:      begin alu_a=regR4_in; alu_b=regC_in; regR4_next = field_merge(regR4_in, alu_res, alu_fs, alu_fe); end
                        endcase
                        pc_next = pc_in + 20'd6;
                    end
                    4'h1: begin // {A,C}=R{x}
                        alu_op = `ALUOP_COPY;
                        case (n5)
                        4'h0:      begin alu_a=regA_in; alu_b=regR0_in; regA_next = field_merge(regA_in, alu_res, alu_fs, alu_fe); end
                        4'h1,4'h5: begin alu_a=regA_in; alu_b=regR1_in; regA_next = field_merge(regA_in, alu_res, alu_fs, alu_fe); end
                        4'h2,4'h6: begin alu_a=regA_in; alu_b=regR2_in; regA_next = field_merge(regA_in, alu_res, alu_fs, alu_fe); end
                        4'h3,4'h7: begin alu_a=regA_in; alu_b=regR3_in; regA_next = field_merge(regA_in, alu_res, alu_fs, alu_fe); end
                        4'h4:      begin alu_a=regA_in; alu_b=regR4_in; regA_next = field_merge(regA_in, alu_res, alu_fs, alu_fe); end
                        4'h8:      begin alu_a=regC_in; alu_b=regR0_in; regC_next = field_merge(regC_in, alu_res, alu_fs, alu_fe); end
                        4'h9,4'hD: begin alu_a=regC_in; alu_b=regR1_in; regC_next = field_merge(regC_in, alu_res, alu_fs, alu_fe); end
                        4'hA,4'hE: begin alu_a=regC_in; alu_b=regR2_in; regC_next = field_merge(regC_in, alu_res, alu_fs, alu_fe); end
                        4'hB,4'hF: begin alu_a=regC_in; alu_b=regR3_in; regC_next = field_merge(regC_in, alu_res, alu_fs, alu_fe); end
                        4'hC:      begin alu_a=regC_in; alu_b=regR4_in; regC_next = field_merge(regC_in, alu_res, alu_fs, alu_fe); end
                        endcase
                        pc_next = pc_in + 20'd6;
                    end
                    4'h2: begin // AR{x}EX / CR{x}EX field
                        alu_op = `ALUOP_EXCH;
                        case (n5)
                        4'h0:      begin alu_a=regA_in; alu_b=regR0_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe);   regR0_next=field_merge(regR0_in,alu_res_b,alu_fs,alu_fe); end
                        4'h1,4'h5: begin alu_a=regA_in; alu_b=regR1_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe);   regR1_next=field_merge(regR1_in,alu_res_b,alu_fs,alu_fe); end
                        4'h2,4'h6: begin alu_a=regA_in; alu_b=regR2_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe);   regR2_next=field_merge(regR2_in,alu_res_b,alu_fs,alu_fe); end
                        4'h3,4'h7: begin alu_a=regA_in; alu_b=regR3_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe);   regR3_next=field_merge(regR3_in,alu_res_b,alu_fs,alu_fe); end
                        4'h4:      begin alu_a=regA_in; alu_b=regR4_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe);   regR4_next=field_merge(regR4_in,alu_res_b,alu_fs,alu_fe); end
                        4'h8:      begin alu_a=regC_in; alu_b=regR0_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe);   regR0_next=field_merge(regR0_in,alu_res_b,alu_fs,alu_fe); end
                        4'h9,4'hD: begin alu_a=regC_in; alu_b=regR1_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe);   regR1_next=field_merge(regR1_in,alu_res_b,alu_fs,alu_fe); end
                        4'hA,4'hE: begin alu_a=regC_in; alu_b=regR2_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe);   regR2_next=field_merge(regR2_in,alu_res_b,alu_fs,alu_fe); end
                        4'hB,4'hF: begin alu_a=regC_in; alu_b=regR3_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe);   regR3_next=field_merge(regR3_in,alu_res_b,alu_fs,alu_fe); end
                        4'hC:      begin alu_a=regC_in; alu_b=regR4_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe);   regR4_next=field_merge(regR4_in,alu_res_b,alu_fs,alu_fe); end
                        endcase
                        pc_next = pc_in + 20'd6;
                    end
                    default: pc_next = pc_in + 20'd5;
                    endcase
                end
                4'hB: begin // PC=A/PC=C/A=PC/C=PC/APCEX/CPCEX
                    case (n3)
                    4'h2: pc_next = regA_in[19:0];
                    4'h3: pc_next = regC_in[19:0];
                    4'h4: begin regA_next[19:0] = pc_in + 20'd4; pc_next = pc_in + 20'd4; end
                    4'h5: begin regC_next[19:0] = pc_in + 20'd4; pc_next = pc_in + 20'd4; end
                    4'h6: begin regA_next[19:0] = pc_in + 20'd4; pc_next = regA_in[19:0]; end
                    4'h7: begin regC_next[19:0] = pc_in + 20'd4; pc_next = regC_in[19:0]; end
                    default: pc_next = pc_in + 20'd4;
                    endcase
                end
                4'hC,4'hD,4'hE,4'hF: begin // ASRB/BSRB/CSRB/DSRB (W-field, 1-bit right)
                    alu_field = `FC_W;
                    alu_op    = `ALUOP_SHRB;
                    alu_b     = 64'd0;
                    case (n2)
                    4'hC: begin alu_a=regA_in; regA_next = alu_res; end
                    4'hD: begin alu_a=regB_in; regB_next = alu_res; end
                    4'hE: begin alu_a=regC_in; regC_next = alu_res; end
                    4'hF: begin alu_a=regD_in; regD_next = alu_res; end
                    endcase
                    if (alu_cout) ST_next[`SB] = 1'b1;
                    pc_next = pc_in + 20'd3;
                end
                endcase
            end
            // ── 82x : CLRST mask ────────────────────────────────────
            4'h2: begin
                if (n2[0]) ST_next[`XM]=1'b0;
                if (n2[1]) ST_next[`SB]=1'b0;
                if (n2[2]) ST_next[`SR]=1'b0;
                if (n2[3]) ST_next[`MP]=1'b0;
                pc_next = pc_in + 20'd3;
            end
            // ── 83x : ?ST=1 mask (carry set iff ALL selected bits zero) + branch ─
            4'h3: begin
                CARRY_next = ~((n2[0] & ST_in[`XM]) |
                               (n2[1] & ST_in[`SB]) |
                               (n2[2] & ST_in[`SR]) |
                               (n2[3] & ST_in[`MP]));
                if (CARRY_next) begin
                    if ({n4, n3} != 8'h00)
                        pc_next = (pc_in + 20'd3 + sext8({n4, n3})) & 20'hFFFFF;
                    else begin
                        pc_next = RSTK_top_in; rstk_pop_en = 1'b1;
                    end
                end else pc_next = pc_in + 20'd5;
            end
            4'h4: begin PSTAT_next[n2] = 1'b0; pc_next = pc_in + 20'd3; end // ST=0 n
            4'h5: begin PSTAT_next[n2] = 1'b1; pc_next = pc_in + 20'd3; end // ST=1 n
            4'h6: begin                                                      // ?ST=0 n + br
                CARRY_next = (PSTAT_in[n2] == 1'b0);
                if (CARRY_next) begin
                    if ({n4, n3} != 8'h00) pc_next = (pc_in + 20'd3 + sext8({n4, n3})) & 20'hFFFFF;
                    else begin pc_next = RSTK_top_in; rstk_pop_en = 1'b1; end
                end else pc_next = pc_in + 20'd5;
            end
            4'h7: begin                                                      // ?ST=1 n + br
                CARRY_next = (PSTAT_in[n2] == 1'b1);
                if (CARRY_next) begin
                    if ({n4, n3} != 8'h00) pc_next = (pc_in + 20'd3 + sext8({n4, n3})) & 20'hFFFFF;
                    else begin pc_next = RSTK_top_in; rstk_pop_en = 1'b1; end
                end else pc_next = pc_in + 20'd5;
            end
            4'h8: begin                                                      // ?P#n + br
                CARRY_next = (P_in != n2);
                if (CARRY_next) begin
                    if ({n4, n3} != 8'h00) pc_next = (pc_in + 20'd3 + sext8({n4, n3})) & 20'hFFFFF;
                    else begin pc_next = RSTK_top_in; rstk_pop_en = 1'b1; end
                end else pc_next = pc_in + 20'd5;
            end
            4'h9: begin                                                      // ?P=n + br
                CARRY_next = (P_in == n2);
                if (CARRY_next) begin
                    if ({n4, n3} != 8'h00) pc_next = (pc_in + 20'd3 + sext8({n4, n3})) & 20'hFFFFF;
                    else begin pc_next = RSTK_top_in; rstk_pop_en = 1'b1; end
                end else pc_next = pc_in + 20'd5;
            end
            4'hA: begin                                                      // 8Ax : ?A=B/?A=0 + br
                alu_field = `FC_A;
                case (n2)
                4'h0: begin alu_a=regA_in; alu_b=regB_in; alu_op=`ALUOP_EQ; end
                4'h1: begin alu_a=regB_in; alu_b=regC_in; alu_op=`ALUOP_EQ; end
                4'h2: begin alu_a=regA_in; alu_b=regC_in; alu_op=`ALUOP_EQ; end
                4'h3: begin alu_a=regC_in; alu_b=regD_in; alu_op=`ALUOP_EQ; end
                4'h4: begin alu_a=regA_in; alu_b=regB_in; alu_op=`ALUOP_NE; end
                4'h5: begin alu_a=regB_in; alu_b=regC_in; alu_op=`ALUOP_NE; end
                4'h6: begin alu_a=regA_in; alu_b=regC_in; alu_op=`ALUOP_NE; end
                4'h7: begin alu_a=regC_in; alu_b=regD_in; alu_op=`ALUOP_NE; end
                4'h8: begin alu_a=regA_in; alu_b=64'd0;  alu_op=`ALUOP_ZERQ; end
                4'h9: begin alu_a=regB_in; alu_b=64'd0;  alu_op=`ALUOP_ZERQ; end
                4'hA: begin alu_a=regC_in; alu_b=64'd0;  alu_op=`ALUOP_ZERQ; end
                4'hB: begin alu_a=regD_in; alu_b=64'd0;  alu_op=`ALUOP_ZERQ; end
                4'hC: begin alu_a=regA_in; alu_b=64'd0;  alu_op=`ALUOP_NZRQ; end
                4'hD: begin alu_a=regB_in; alu_b=64'd0;  alu_op=`ALUOP_NZRQ; end
                4'hE: begin alu_a=regC_in; alu_b=64'd0;  alu_op=`ALUOP_NZRQ; end
                4'hF: begin alu_a=regD_in; alu_b=64'd0;  alu_op=`ALUOP_NZRQ; end
                endcase
                CARRY_next = alu_cout;
                if (alu_cout) begin
                    if ({n4, n3} != 8'h00) pc_next = (pc_in + 20'd3 + sext8({n4, n3})) & 20'hFFFFF;
                    else begin pc_next = RSTK_top_in; rstk_pop_en = 1'b1; end
                end else pc_next = pc_in + 20'd5;
            end
            4'hB: begin                                                      // 8Bx : ordered compares + br
                alu_field = `FC_A;
                case (n2)
                4'h0: begin alu_a=regA_in; alu_b=regB_in; alu_op=`ALUOP_GT; end
                4'h1: begin alu_a=regB_in; alu_b=regC_in; alu_op=`ALUOP_GT; end
                4'h2: begin alu_a=regC_in; alu_b=regA_in; alu_op=`ALUOP_GT; end
                4'h3: begin alu_a=regD_in; alu_b=regC_in; alu_op=`ALUOP_GT; end
                4'h4: begin alu_a=regA_in; alu_b=regB_in; alu_op=`ALUOP_LT; end
                4'h5: begin alu_a=regB_in; alu_b=regC_in; alu_op=`ALUOP_LT; end
                4'h6: begin alu_a=regC_in; alu_b=regA_in; alu_op=`ALUOP_LT; end
                4'h7: begin alu_a=regD_in; alu_b=regC_in; alu_op=`ALUOP_LT; end
                4'h8: begin alu_a=regA_in; alu_b=regB_in; alu_op=`ALUOP_GE; end
                4'h9: begin alu_a=regB_in; alu_b=regC_in; alu_op=`ALUOP_GE; end
                4'hA: begin alu_a=regC_in; alu_b=regA_in; alu_op=`ALUOP_GE; end
                4'hB: begin alu_a=regD_in; alu_b=regC_in; alu_op=`ALUOP_GE; end
                4'hC: begin alu_a=regA_in; alu_b=regB_in; alu_op=`ALUOP_LE; end
                4'hD: begin alu_a=regB_in; alu_b=regC_in; alu_op=`ALUOP_LE; end
                4'hE: begin alu_a=regC_in; alu_b=regA_in; alu_op=`ALUOP_LE; end
                4'hF: begin alu_a=regD_in; alu_b=regC_in; alu_op=`ALUOP_LE; end
                endcase
                CARRY_next = alu_cout;
                if (alu_cout) begin
                    if ({n4, n3} != 8'h00) pc_next = (pc_in + 20'd3 + sext8({n4, n3})) & 20'hFFFFF;
                    else begin pc_next = RSTK_top_in; rstk_pop_en = 1'b1; end
                end else pc_next = pc_in + 20'd5;
            end
            4'hC: pc_next = (pc_in + sext16({n5, n4, n3, n2}) + 20'd2) & 20'hFFFFF;               // GOTO_LONG
            4'hD: pc_next = {n6, n5, n4, n3, n2};                                                  // GOTO_ABS
            4'hE: begin                                                                            // GOSUB_LONG
                rstk_push_en  = 1'b1;
                rstk_push_val = pc_in + 20'd6;
                pc_next       = (pc_in + sext16({n5, n4, n3, n2}) + 20'd6) & 20'hFFFFF;
            end
            4'hF: begin                                                                            // GOSUB_ABS
                rstk_push_en  = 1'b1;
                rstk_push_val = pc_in + 20'd7;
                pc_next       = {n6, n5, n4, n3, n2};
            end
            default: pc_next = pc_in + 20'd3;
            endcase
        end

        // ═══════════════════════════ GROUP 9 : test + cond-branch ═════
        4'h9: begin
            if (n1 < 4'd8) begin
                alu_field = {1'b0, n1};
                case (n2)
                4'h0: begin alu_a=regA_in; alu_b=regB_in; alu_op=`ALUOP_EQ; end
                4'h1: begin alu_a=regB_in; alu_b=regC_in; alu_op=`ALUOP_EQ; end
                4'h2: begin alu_a=regA_in; alu_b=regC_in; alu_op=`ALUOP_EQ; end
                4'h3: begin alu_a=regC_in; alu_b=regD_in; alu_op=`ALUOP_EQ; end
                4'h4: begin alu_a=regA_in; alu_b=regB_in; alu_op=`ALUOP_NE; end
                4'h5: begin alu_a=regB_in; alu_b=regC_in; alu_op=`ALUOP_NE; end
                4'h6: begin alu_a=regA_in; alu_b=regC_in; alu_op=`ALUOP_NE; end
                4'h7: begin alu_a=regC_in; alu_b=regD_in; alu_op=`ALUOP_NE; end
                4'h8: begin alu_a=regA_in; alu_b=64'd0;  alu_op=`ALUOP_ZERQ; end
                4'h9: begin alu_a=regB_in; alu_b=64'd0;  alu_op=`ALUOP_ZERQ; end
                4'hA: begin alu_a=regC_in; alu_b=64'd0;  alu_op=`ALUOP_ZERQ; end
                4'hB: begin alu_a=regD_in; alu_b=64'd0;  alu_op=`ALUOP_ZERQ; end
                4'hC: begin alu_a=regA_in; alu_b=64'd0;  alu_op=`ALUOP_NZRQ; end
                4'hD: begin alu_a=regB_in; alu_b=64'd0;  alu_op=`ALUOP_NZRQ; end
                4'hE: begin alu_a=regC_in; alu_b=64'd0;  alu_op=`ALUOP_NZRQ; end
                4'hF: begin alu_a=regD_in; alu_b=64'd0;  alu_op=`ALUOP_NZRQ; end
                endcase
            end else begin
                alu_field = {1'b0, n1 & 4'h7};
                case (n2)
                4'h0: begin alu_a=regA_in; alu_b=regB_in; alu_op=`ALUOP_GT; end
                4'h1: begin alu_a=regB_in; alu_b=regC_in; alu_op=`ALUOP_GT; end
                4'h2: begin alu_a=regC_in; alu_b=regA_in; alu_op=`ALUOP_GT; end
                4'h3: begin alu_a=regD_in; alu_b=regC_in; alu_op=`ALUOP_GT; end
                4'h4: begin alu_a=regA_in; alu_b=regB_in; alu_op=`ALUOP_LT; end
                4'h5: begin alu_a=regB_in; alu_b=regC_in; alu_op=`ALUOP_LT; end
                4'h6: begin alu_a=regC_in; alu_b=regA_in; alu_op=`ALUOP_LT; end
                4'h7: begin alu_a=regD_in; alu_b=regC_in; alu_op=`ALUOP_LT; end
                4'h8: begin alu_a=regA_in; alu_b=regB_in; alu_op=`ALUOP_GE; end
                4'h9: begin alu_a=regB_in; alu_b=regC_in; alu_op=`ALUOP_GE; end
                4'hA: begin alu_a=regC_in; alu_b=regA_in; alu_op=`ALUOP_GE; end
                4'hB: begin alu_a=regD_in; alu_b=regC_in; alu_op=`ALUOP_GE; end
                4'hC: begin alu_a=regA_in; alu_b=regB_in; alu_op=`ALUOP_LE; end
                4'hD: begin alu_a=regB_in; alu_b=regC_in; alu_op=`ALUOP_LE; end
                4'hE: begin alu_a=regC_in; alu_b=regA_in; alu_op=`ALUOP_LE; end
                4'hF: begin alu_a=regD_in; alu_b=regC_in; alu_op=`ALUOP_LE; end
                endcase
            end
            CARRY_next = alu_cout;
            if (alu_cout) begin
                if ({n4, n3} == 8'h00) begin
                    pc_next = RSTK_top_in; rstk_pop_en = 1'b1;
                end else
                    pc_next = (pc_in + 20'd3 + sext8({n4, n3})) & 20'hFFFFF;
            end else
                pc_next = pc_in + 20'd5;
        end

        // ═══════════════════════════ GROUP A : field ADD/DEC or ZERO/COPY/EXCH ══
        4'hA: begin
            if (n1 < 4'd8) begin
                alu_field = {1'b0, n1};
                case (n2)
                4'h0: begin alu_op=`ALUOP_ADD; alu_a=regA_in; alu_b=regB_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); end
                4'h1: begin alu_op=`ALUOP_ADD; alu_a=regB_in; alu_b=regC_in; regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); end
                4'h2: begin alu_op=`ALUOP_ADD; alu_a=regC_in; alu_b=regA_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); end
                4'h3: begin alu_op=`ALUOP_ADD; alu_a=regD_in; alu_b=regC_in; regD_next=field_merge(regD_in,alu_res,alu_fs,alu_fe); end
                4'h4: begin alu_op=`ALUOP_ADD; alu_a=regA_in; alu_b=regA_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); end
                4'h5: begin alu_op=`ALUOP_ADD; alu_a=regB_in; alu_b=regB_in; regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); end
                4'h6: begin alu_op=`ALUOP_ADD; alu_a=regC_in; alu_b=regC_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); end
                4'h7: begin alu_op=`ALUOP_ADD; alu_a=regD_in; alu_b=regD_in; regD_next=field_merge(regD_in,alu_res,alu_fs,alu_fe); end
                4'h8: begin alu_op=`ALUOP_ADD; alu_a=regB_in; alu_b=regA_in; regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); end
                4'h9: begin alu_op=`ALUOP_ADD; alu_a=regC_in; alu_b=regB_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); end
                4'hA: begin alu_op=`ALUOP_ADD; alu_a=regA_in; alu_b=regC_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); end
                4'hB: begin alu_op=`ALUOP_ADD; alu_a=regC_in; alu_b=regD_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); end
                4'hC: begin alu_op=`ALUOP_DEC; alu_a=regA_in; alu_b=64'd0;  regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); end
                4'hD: begin alu_op=`ALUOP_DEC; alu_a=regB_in; alu_b=64'd0;  regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); end
                4'hE: begin alu_op=`ALUOP_DEC; alu_a=regC_in; alu_b=64'd0;  regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); end
                4'hF: begin alu_op=`ALUOP_DEC; alu_a=regD_in; alu_b=64'd0;  regD_next=field_merge(regD_in,alu_res,alu_fs,alu_fe); end
                endcase
                CARRY_next = alu_cout;
            end else begin
                alu_field = {1'b0, n1 & 4'h7};
                case (n2)
                4'h0: begin alu_op=`ALUOP_ZERO; alu_a=regA_in; alu_b=64'd0;  regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
                4'h1: begin alu_op=`ALUOP_ZERO; alu_a=regB_in; alu_b=64'd0;  regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
                4'h2: begin alu_op=`ALUOP_ZERO; alu_a=regC_in; alu_b=64'd0;  regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
                4'h3: begin alu_op=`ALUOP_ZERO; alu_a=regD_in; alu_b=64'd0;  regD_next=field_merge(regD_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
                4'h4: begin alu_op=`ALUOP_COPY; alu_a=regA_in; alu_b=regB_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
                4'h5: begin alu_op=`ALUOP_COPY; alu_a=regB_in; alu_b=regC_in; regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
                4'h6: begin alu_op=`ALUOP_COPY; alu_a=regC_in; alu_b=regA_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
                4'h7: begin alu_op=`ALUOP_COPY; alu_a=regD_in; alu_b=regC_in; regD_next=field_merge(regD_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
                4'h8: begin alu_op=`ALUOP_COPY; alu_a=regB_in; alu_b=regA_in; regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
                4'h9: begin alu_op=`ALUOP_COPY; alu_a=regC_in; alu_b=regB_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
                4'hA: begin alu_op=`ALUOP_COPY; alu_a=regA_in; alu_b=regC_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
                4'hB: begin alu_op=`ALUOP_COPY; alu_a=regC_in; alu_b=regD_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
                4'hC: begin alu_op=`ALUOP_EXCH; alu_a=regA_in; alu_b=regB_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); regB_next=field_merge(regB_in,alu_res_b,alu_fs,alu_fe); end
                4'hD: begin alu_op=`ALUOP_EXCH; alu_a=regB_in; alu_b=regC_in; regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); regC_next=field_merge(regC_in,alu_res_b,alu_fs,alu_fe); end
                4'hE: begin alu_op=`ALUOP_EXCH; alu_a=regA_in; alu_b=regC_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); regC_next=field_merge(regC_in,alu_res_b,alu_fs,alu_fe); end
                4'hF: begin alu_op=`ALUOP_EXCH; alu_a=regC_in; alu_b=regD_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); regD_next=field_merge(regD_in,alu_res_b,alu_fs,alu_fe); end
                endcase
            end
            pc_next = pc_in + 20'd3;
        end

        // ═══════════════════════════ GROUP B : field SUB/INC or SHL/SHR/NEG ═════
        4'hB: begin
            if (n1 < 4'd8) begin
                alu_field = {1'b0, n1};
                case (n2)
                4'h0: begin alu_op=`ALUOP_SUB; alu_a=regA_in; alu_b=regB_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); end
                4'h1: begin alu_op=`ALUOP_SUB; alu_a=regB_in; alu_b=regC_in; regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); end
                4'h2: begin alu_op=`ALUOP_SUB; alu_a=regC_in; alu_b=regA_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); end
                4'h3: begin alu_op=`ALUOP_SUB; alu_a=regD_in; alu_b=regC_in; regD_next=field_merge(regD_in,alu_res,alu_fs,alu_fe); end
                4'h4: begin alu_op=`ALUOP_INC; alu_a=regA_in; alu_b=64'd0;  regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); end
                4'h5: begin alu_op=`ALUOP_INC; alu_a=regB_in; alu_b=64'd0;  regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); end
                4'h6: begin alu_op=`ALUOP_INC; alu_a=regC_in; alu_b=64'd0;  regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); end
                4'h7: begin alu_op=`ALUOP_INC; alu_a=regD_in; alu_b=64'd0;  regD_next=field_merge(regD_in,alu_res,alu_fs,alu_fe); end
                4'h8: begin alu_op=`ALUOP_SUB; alu_a=regB_in; alu_b=regA_in; regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); end
                4'h9: begin alu_op=`ALUOP_SUB; alu_a=regC_in; alu_b=regB_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); end
                4'hA: begin alu_op=`ALUOP_SUB; alu_a=regA_in; alu_b=regC_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); end
                4'hB: begin alu_op=`ALUOP_SUB; alu_a=regC_in; alu_b=regD_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); end
                4'hC: begin alu_op=`ALUOP_SUB; alu_a=regB_in; alu_b=regA_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); end // A=B-A
                4'hD: begin alu_op=`ALUOP_SUB; alu_a=regC_in; alu_b=regB_in; regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); end // B=C-B
                4'hE: begin alu_op=`ALUOP_SUB; alu_a=regA_in; alu_b=regC_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); end // C=A-C
                4'hF: begin alu_op=`ALUOP_SUB; alu_a=regC_in; alu_b=regD_in; regD_next=field_merge(regD_in,alu_res,alu_fs,alu_fe); end // D=C-D
                endcase
                CARRY_next = alu_cout;
            end else begin
                alu_field = {1'b0, n1 & 4'h7};
                alu_b     = 64'd0;
                case (n2)
                4'h0: begin alu_op=`ALUOP_SHL; alu_a=regA_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
                4'h1: begin alu_op=`ALUOP_SHL; alu_a=regB_in; regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
                4'h2: begin alu_op=`ALUOP_SHL; alu_a=regC_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
                4'h3: begin alu_op=`ALUOP_SHL; alu_a=regD_in; regD_next=field_merge(regD_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
                4'h4: begin alu_op=`ALUOP_SHR; alu_a=regA_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); if (alu_cout) ST_next[`SB]=1'b1; end
                4'h5: begin alu_op=`ALUOP_SHR; alu_a=regB_in; regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); if (alu_cout) ST_next[`SB]=1'b1; end
                4'h6: begin alu_op=`ALUOP_SHR; alu_a=regC_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); if (alu_cout) ST_next[`SB]=1'b1; end
                4'h7: begin alu_op=`ALUOP_SHR; alu_a=regD_in; regD_next=field_merge(regD_in,alu_res,alu_fs,alu_fe); if (alu_cout) ST_next[`SB]=1'b1; end
                4'h8: begin alu_op=`ALUOP_NEG2;alu_a=regA_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
                4'h9: begin alu_op=`ALUOP_NEG2;alu_a=regB_in; regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
                4'hA: begin alu_op=`ALUOP_NEG2;alu_a=regC_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
                4'hB: begin alu_op=`ALUOP_NEG2;alu_a=regD_in; regD_next=field_merge(regD_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
                4'hC: begin alu_op=`ALUOP_NEG1;alu_a=regA_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
                4'hD: begin alu_op=`ALUOP_NEG1;alu_a=regB_in; regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
                4'hE: begin alu_op=`ALUOP_NEG1;alu_a=regC_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
                4'hF: begin alu_op=`ALUOP_NEG1;alu_a=regD_in; regD_next=field_merge(regD_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
                endcase
            end
            pc_next = pc_in + 20'd3;
        end

        // ═══════════════════════════ GROUP C : A-field ADD/DEC ═══════
        4'hC: begin
            alu_field = `FC_A;
            case (n1)
            4'h0: begin alu_op=`ALUOP_ADD; alu_a=regA_in; alu_b=regB_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); end
            4'h1: begin alu_op=`ALUOP_ADD; alu_a=regB_in; alu_b=regC_in; regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); end
            4'h2: begin alu_op=`ALUOP_ADD; alu_a=regC_in; alu_b=regA_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); end
            4'h3: begin alu_op=`ALUOP_ADD; alu_a=regD_in; alu_b=regC_in; regD_next=field_merge(regD_in,alu_res,alu_fs,alu_fe); end
            4'h4: begin alu_op=`ALUOP_ADD; alu_a=regA_in; alu_b=regA_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); end
            4'h5: begin alu_op=`ALUOP_ADD; alu_a=regB_in; alu_b=regB_in; regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); end
            4'h6: begin alu_op=`ALUOP_ADD; alu_a=regC_in; alu_b=regC_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); end
            4'h7: begin alu_op=`ALUOP_ADD; alu_a=regD_in; alu_b=regD_in; regD_next=field_merge(regD_in,alu_res,alu_fs,alu_fe); end
            4'h8: begin alu_op=`ALUOP_ADD; alu_a=regB_in; alu_b=regA_in; regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); end
            4'h9: begin alu_op=`ALUOP_ADD; alu_a=regC_in; alu_b=regB_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); end
            4'hA: begin alu_op=`ALUOP_ADD; alu_a=regA_in; alu_b=regC_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); end
            4'hB: begin alu_op=`ALUOP_ADD; alu_a=regC_in; alu_b=regD_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); end
            4'hC: begin alu_op=`ALUOP_DEC; alu_a=regA_in; alu_b=64'd0;  regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); end
            4'hD: begin alu_op=`ALUOP_DEC; alu_a=regB_in; alu_b=64'd0;  regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); end
            4'hE: begin alu_op=`ALUOP_DEC; alu_a=regC_in; alu_b=64'd0;  regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); end
            4'hF: begin alu_op=`ALUOP_DEC; alu_a=regD_in; alu_b=64'd0;  regD_next=field_merge(regD_in,alu_res,alu_fs,alu_fe); end
            endcase
            CARRY_next = alu_cout;
            pc_next    = pc_in + 20'd2;
        end

        // ═══════════════════════════ GROUP D : A-field ZERO/COPY/EXCH ══
        4'hD: begin
            alu_field = `FC_A;
            case (n1)
            4'h0: begin alu_op=`ALUOP_ZERO; alu_a=regA_in; alu_b=64'd0;  regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
            4'h1: begin alu_op=`ALUOP_ZERO; alu_a=regB_in; alu_b=64'd0;  regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
            4'h2: begin alu_op=`ALUOP_ZERO; alu_a=regC_in; alu_b=64'd0;  regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
            4'h3: begin alu_op=`ALUOP_ZERO; alu_a=regD_in; alu_b=64'd0;  regD_next=field_merge(regD_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
            4'h4: begin alu_op=`ALUOP_COPY; alu_a=regA_in; alu_b=regB_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
            4'h5: begin alu_op=`ALUOP_COPY; alu_a=regB_in; alu_b=regC_in; regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
            4'h6: begin alu_op=`ALUOP_COPY; alu_a=regC_in; alu_b=regA_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
            4'h7: begin alu_op=`ALUOP_COPY; alu_a=regD_in; alu_b=regC_in; regD_next=field_merge(regD_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
            4'h8: begin alu_op=`ALUOP_COPY; alu_a=regB_in; alu_b=regA_in; regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
            4'h9: begin alu_op=`ALUOP_COPY; alu_a=regC_in; alu_b=regB_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
            4'hA: begin alu_op=`ALUOP_COPY; alu_a=regA_in; alu_b=regC_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
            4'hB: begin alu_op=`ALUOP_COPY; alu_a=regC_in; alu_b=regD_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
            4'hC: begin alu_op=`ALUOP_EXCH; alu_a=regA_in; alu_b=regB_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); regB_next=field_merge(regB_in,alu_res_b,alu_fs,alu_fe); end
            4'hD: begin alu_op=`ALUOP_EXCH; alu_a=regB_in; alu_b=regC_in; regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); regC_next=field_merge(regC_in,alu_res_b,alu_fs,alu_fe); end
            4'hE: begin alu_op=`ALUOP_EXCH; alu_a=regA_in; alu_b=regC_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); regC_next=field_merge(regC_in,alu_res_b,alu_fs,alu_fe); end
            4'hF: begin alu_op=`ALUOP_EXCH; alu_a=regC_in; alu_b=regD_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); regD_next=field_merge(regD_in,alu_res_b,alu_fs,alu_fe); end
            endcase
            pc_next = pc_in + 20'd2;
        end

        // ═══════════════════════════ GROUP E : A-field SUB/INC ═══════
        4'hE: begin
            alu_field = `FC_A;
            case (n1)
            4'h0: begin alu_op=`ALUOP_SUB; alu_a=regA_in; alu_b=regB_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); end
            4'h1: begin alu_op=`ALUOP_SUB; alu_a=regB_in; alu_b=regC_in; regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); end
            4'h2: begin alu_op=`ALUOP_SUB; alu_a=regC_in; alu_b=regA_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); end
            4'h3: begin alu_op=`ALUOP_SUB; alu_a=regD_in; alu_b=regC_in; regD_next=field_merge(regD_in,alu_res,alu_fs,alu_fe); end
            4'h4: begin alu_op=`ALUOP_INC; alu_a=regA_in; alu_b=64'd0;  regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); end
            4'h5: begin alu_op=`ALUOP_INC; alu_a=regB_in; alu_b=64'd0;  regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); end
            4'h6: begin alu_op=`ALUOP_INC; alu_a=regC_in; alu_b=64'd0;  regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); end
            4'h7: begin alu_op=`ALUOP_INC; alu_a=regD_in; alu_b=64'd0;  regD_next=field_merge(regD_in,alu_res,alu_fs,alu_fe); end
            4'h8: begin alu_op=`ALUOP_SUB; alu_a=regB_in; alu_b=regA_in; regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); end
            4'h9: begin alu_op=`ALUOP_SUB; alu_a=regC_in; alu_b=regB_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); end
            4'hA: begin alu_op=`ALUOP_SUB; alu_a=regA_in; alu_b=regC_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); end
            4'hB: begin alu_op=`ALUOP_SUB; alu_a=regC_in; alu_b=regD_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); end
            4'hC: begin alu_op=`ALUOP_SUB; alu_a=regB_in; alu_b=regA_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); end
            4'hD: begin alu_op=`ALUOP_SUB; alu_a=regC_in; alu_b=regB_in; regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); end
            4'hE: begin alu_op=`ALUOP_SUB; alu_a=regA_in; alu_b=regC_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); end
            4'hF: begin alu_op=`ALUOP_SUB; alu_a=regC_in; alu_b=regD_in; regD_next=field_merge(regD_in,alu_res,alu_fs,alu_fe); end
            endcase
            CARRY_next = alu_cout;
            pc_next    = pc_in + 20'd2;
        end

        // ═══════════════════════════ GROUP F : A-field SHIFTS / NEG ══
        4'hF: begin
            alu_field = `FC_A;
            alu_b     = 64'd0;
            case (n1)
            4'h0: begin alu_op=`ALUOP_SHL; alu_a=regA_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
            4'h1: begin alu_op=`ALUOP_SHL; alu_a=regB_in; regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
            4'h2: begin alu_op=`ALUOP_SHL; alu_a=regC_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
            4'h3: begin alu_op=`ALUOP_SHL; alu_a=regD_in; regD_next=field_merge(regD_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
            4'h4: begin alu_op=`ALUOP_SHR; alu_a=regA_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); if (alu_cout) ST_next[`SB]=1'b1; end
            4'h5: begin alu_op=`ALUOP_SHR; alu_a=regB_in; regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); if (alu_cout) ST_next[`SB]=1'b1; end
            4'h6: begin alu_op=`ALUOP_SHR; alu_a=regC_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); if (alu_cout) ST_next[`SB]=1'b1; end
            4'h7: begin alu_op=`ALUOP_SHR; alu_a=regD_in; regD_next=field_merge(regD_in,alu_res,alu_fs,alu_fe); if (alu_cout) ST_next[`SB]=1'b1; end
            4'h8: begin alu_op=`ALUOP_NEG2;alu_a=regA_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
            4'h9: begin alu_op=`ALUOP_NEG2;alu_a=regB_in; regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
            4'hA: begin alu_op=`ALUOP_NEG2;alu_a=regC_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
            4'hB: begin alu_op=`ALUOP_NEG2;alu_a=regD_in; regD_next=field_merge(regD_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
            4'hC: begin alu_op=`ALUOP_NEG1;alu_a=regA_in; regA_next=field_merge(regA_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
            4'hD: begin alu_op=`ALUOP_NEG1;alu_a=regB_in; regB_next=field_merge(regB_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
            4'hE: begin alu_op=`ALUOP_NEG1;alu_a=regC_in; regC_next=field_merge(regC_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
            4'hF: begin alu_op=`ALUOP_NEG1;alu_a=regD_in; regD_next=field_merge(regD_in,alu_res,alu_fs,alu_fe); CARRY_next=alu_cout; end
            endcase
            pc_next = pc_in + 20'd2;
        end

        default: pc_next = pc_in + 20'd1;  // illegal instruction
        endcase
    end

endmodule
