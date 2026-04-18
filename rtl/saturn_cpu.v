/**
 * @file saturn_cpu.v
 * @brief Synthesizable top-level of the HP Saturn CPU.
 *
 * This module owns the architectural state (A/B/C/D, R0..R4, D0/D1, PC,
 * P, ST, PSTAT, CARRY, HEXMODE, return stack, IN/OUT), the fetch buffer,
 * and the memory-bus FSM. All combinational instruction dispatch lives
 * in saturn_exec.v — this file just latches the next-state wires the
 * decoder produces on the cycle the FSM spends in S_EXEC.
 *
 * ## FSM
 *  1. @b S_IDLE  — wait for external `step` pulse.
 *  2. @b S_FETCH — read 32 nibbles from memory[PC..PC+31] into ib[].
 *  3. @b S_EXEC  — latch all `*_next` outputs of saturn_exec; go to
 *                 S_MEMR / S_MEMW if the instruction carried a memop,
 *                 else S_DONE.
 *  4. @b S_MEMW / @b S_MEMR — one nibble per cycle, address bumps by 1.
 *  5. @b S_DONE  — assert `step_done` for one cycle and return to S_IDLE.
 *
 * No `#0` delays, no tasks: every always block is either a pure
 * `posedge clk` sequential or a plain `@*` combinational.
 *
 * External signal names (regA, regB, regC, regD, regR, D0, D1, P, PC,
 * ST, PSTAT, CARRY, HEXMODE, RSTK, RSTKP, IN_REG, OUT_REG, state,
 * mem_*) are preserved so sim/verilog_core.cpp can read/write them via
 * Verilator's `--public-flat-rw` hierarchy without path rewriting.
 */
`include "saturn_pkg.vh"

/**
 * @brief Synthesizable HP Saturn CPU core.
 *
 * Runs one instruction per external `step` pulse. The memory bus is a
 * single-port nibble interface owned by the parent (typically a C++
 * Verilator harness or a simple reg array in a standalone testbench).
 */
module saturn_cpu (
    input  wire        clk,       ///< system clock (rising edge)
    input  wire        rst_n,     ///< active-low async reset
    output reg  [19:0] mem_addr,  ///< memory address (20-bit nibble address)
    output reg         mem_we,    ///< write enable (1-cycle pulses during S_MEMW)
    output reg         mem_re,    ///< read  enable (held during S_FETCH / S_MEMR)
    output reg  [3:0]  mem_wdata, ///< nibble to write when mem_we is asserted
    input  wire [3:0]  mem_rdata, ///< nibble read back on the following cycle
    // ── observability (unchanged so tb_cpu / x48_sim don't need rewiring) ─
    output wire [19:0] dbg_pc,        ///< live PC
    output wire [63:0] dbg_A, dbg_B, dbg_C, dbg_D,  ///< working registers
    output wire [19:0] dbg_D0, dbg_D1,             ///< data pointers
    output wire [3:0]  dbg_P,                      ///< P register (0..15)
    output wire [3:0]  dbg_ST,                     ///< status bits {MP,SR,SB,XM}
    output wire [15:0] dbg_PSTAT,                  ///< program-status 16 bits
    output wire        dbg_CARRY,                  ///< carry flag
    output wire        dbg_HEXMODE,                ///< 1 = hex, 0 = decimal
    output wire signed [3:0] dbg_RSTKP,            ///< return-stack pointer (-1 = empty)
    output wire [159:0] dbg_RSTK,                  ///< packed RSTK[0..7] each 20b
    input  wire        step,                       ///< pulse high for 1 cycle to start an instruction
    output reg         step_done                   ///< asserted high for the cycle that completes an instruction
);
    // ─────────────────────────────────────────────────────────────
    // Architectural state registers
    // ─────────────────────────────────────────────────────────────
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
    reg [63:0] IN_REG, OUT_REG;

    assign dbg_pc      = PC;
    assign dbg_A       = regA;   assign dbg_B = regB;
    assign dbg_C       = regC;   assign dbg_D = regD;
    assign dbg_D0      = D0;     assign dbg_D1 = D1;
    assign dbg_P       = P;      assign dbg_ST = ST;
    assign dbg_PSTAT   = PSTAT;
    assign dbg_CARRY   = CARRY;
    assign dbg_HEXMODE = HEXMODE;
    assign dbg_RSTKP   = RSTKP;
    genvar gi;
    generate
        for (gi = 0; gi < 8; gi = gi + 1)
            assign dbg_RSTK[gi*20 +: 20] = RSTK[gi];
    endgenerate

    // ─────────────────────────────────────────────────────────────
    // Fetch buffer (32 nibbles — longest instruction is LA with 16-nib
    // constant = 22 nibbles).
    // ─────────────────────────────────────────────────────────────
    reg [3:0]  ib [0:31];
    reg [5:0]  fetch_i;

    // ─────────────────────────────────────────────────────────────
    // FSM
    // ─────────────────────────────────────────────────────────────
    localparam S_IDLE = 4'd0,
               S_FETCH = 4'd1,
               S_EXEC = 4'd2,
               S_MEMR = 4'd3,
               S_MEMW = 4'd4,
               S_DONE = 4'd5;
    reg [3:0]  state;

    // ─────────────────────────────────────────────────────────────
    // Pending memory-op descriptor (latched on S_EXEC entry from exec)
    // ─────────────────────────────────────────────────────────────
    reg        memop_is_write;
    reg [19:0] memop_addr;
    reg [63:0] memop_data;
    reg [1:0]  memop_dst_reg;
    reg [4:0]  memop_len;
    reg [3:0]  memop_start_nib;
    reg [4:0]  mem_i;

    // ─────────────────────────────────────────────────────────────
    // ALU and field decoder (combinational, outside saturn_exec so the
    // exec block can consume the result wire on the same cycle it drives
    // the control wires)
    // ─────────────────────────────────────────────────────────────
    wire [63:0] alu_a, alu_b;
    wire [4:0]  alu_field;
    wire [5:0]  alu_op;
    wire [63:0] alu_res, alu_res_b;
    wire        alu_cout;
    wire [3:0]  alu_fs, alu_fe;

    saturn_alu u_alu (
        .a        (alu_a),
        .b        (alu_b),
        .p        (P),
        .field    (alu_field),
        .op       (alu_op),
        .hex_mode (HEXMODE),
        .carry_in (CARRY),
        .res      (alu_res),
        .res_b    (alu_res_b),
        .carry_out(alu_cout)
    );

    saturn_field u_alu_fd (
        .code      (alu_field),
        .p         (P),
        .start_nib (alu_fs),
        .end_nib   (alu_fe)
    );

    // ─────────────────────────────────────────────────────────────
    // saturn_exec : pure combinational decoder → next_* wires
    // ─────────────────────────────────────────────────────────────
    wire [19:0] pc_next_w;
    wire [63:0] regA_next_w, regB_next_w, regC_next_w, regD_next_w;
    wire [63:0] regR0_next_w, regR1_next_w, regR2_next_w, regR3_next_w, regR4_next_w;
    wire [19:0] D0_next_w, D1_next_w;
    wire [3:0]  P_next_w;
    wire [3:0]  ST_next_w;
    wire [15:0] PSTAT_next_w;
    wire        CARRY_next_w;
    wire        HEXMODE_next_w;
    wire [63:0] OUT_REG_next_w;
    wire        rstk_push_en_w;
    wire [19:0] rstk_push_val_w;
    wire        rstk_pop_en_w;
    wire        do_memop_w;
    wire        memop_is_write_w;
    wire [19:0] memop_addr_w;
    wire [63:0] memop_data_w;
    wire [1:0]  memop_dst_reg_w;
    wire [4:0]  memop_len_w;
    wire [3:0]  memop_start_nib_w;
    wire [19:0] rstk_top_w;
    assign rstk_top_w = (RSTKP >= 0) ? RSTK[RSTKP] : 20'd0;

    saturn_exec u_exec (
        .pc_in       (PC),
        .regA_in     (regA),  .regB_in(regB), .regC_in(regC), .regD_in(regD),
        .regR0_in    (regR[0]), .regR1_in(regR[1]), .regR2_in(regR[2]),
        .regR3_in    (regR[3]), .regR4_in(regR[4]),
        .D0_in       (D0),    .D1_in (D1),
        .P_in        (P),
        .ST_in       (ST),
        .PSTAT_in    (PSTAT),
        .CARRY_in    (CARRY),
        .HEXMODE_in  (HEXMODE),
        .RSTK_top_in (rstk_top_w),
        .IN_REG_in   (IN_REG),
        .OUT_REG_in  (OUT_REG),
        // instruction bytes
        .ib0 (ib[ 0]), .ib1 (ib[ 1]), .ib2 (ib[ 2]), .ib3 (ib[ 3]),
        .ib4 (ib[ 4]), .ib5 (ib[ 5]), .ib6 (ib[ 6]), .ib7 (ib[ 7]),
        .ib8 (ib[ 8]), .ib9 (ib[ 9]), .ib10(ib[10]), .ib11(ib[11]),
        .ib12(ib[12]), .ib13(ib[13]), .ib14(ib[14]), .ib15(ib[15]),
        .ib16(ib[16]), .ib17(ib[17]), .ib18(ib[18]), .ib19(ib[19]),
        .ib20(ib[20]), .ib21(ib[21]), .ib22(ib[22]), .ib23(ib[23]),
        // ALU
        .alu_a (alu_a), .alu_b(alu_b),
        .alu_field(alu_field), .alu_op(alu_op),
        .alu_res(alu_res), .alu_res_b(alu_res_b),
        .alu_cout(alu_cout),
        .alu_fs(alu_fs), .alu_fe(alu_fe),
        // next state
        .pc_next       (pc_next_w),
        .regA_next     (regA_next_w),  .regB_next(regB_next_w),
        .regC_next     (regC_next_w),  .regD_next(regD_next_w),
        .regR0_next    (regR0_next_w), .regR1_next(regR1_next_w),
        .regR2_next    (regR2_next_w), .regR3_next(regR3_next_w),
        .regR4_next    (regR4_next_w),
        .D0_next       (D0_next_w),    .D1_next  (D1_next_w),
        .P_next        (P_next_w),
        .ST_next       (ST_next_w),
        .PSTAT_next    (PSTAT_next_w),
        .CARRY_next    (CARRY_next_w),
        .HEXMODE_next  (HEXMODE_next_w),
        .OUT_REG_next  (OUT_REG_next_w),
        .rstk_push_en  (rstk_push_en_w),
        .rstk_push_val (rstk_push_val_w),
        .rstk_pop_en   (rstk_pop_en_w),
        .do_memop        (do_memop_w),
        .memop_is_write  (memop_is_write_w),
        .memop_addr      (memop_addr_w),
        .memop_data      (memop_data_w),
        .memop_dst_reg   (memop_dst_reg_w),
        .memop_len       (memop_len_w),
        .memop_start_nib (memop_start_nib_w)
    );

    // ─────────────────────────────────────────────────────────────
    // Sequential FSM + state update
    // ─────────────────────────────────────────────────────────────
    integer i;
    integer k;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= S_IDLE;
            PC      <= 20'd0;
            regA    <= 64'd0;
            regB    <= 64'd0;
            regC    <= 64'd0;
            regD    <= 64'd0;
            regR[0] <= 64'd0;
            regR[1] <= 64'd0;
            regR[2] <= 64'd0;
            regR[3] <= 64'd0;
            regR[4] <= 64'd0;
            D0      <= 20'd0;
            D1      <= 20'd0;
            P       <= 4'd0;
            ST      <= 4'd0;
            PSTAT   <= 16'd0;
            CARRY   <= 1'b0;
            HEXMODE <= 1'b1;
            IN_REG  <= 64'd0;
            OUT_REG <= 64'd0;
            RSTKP   <= -4'sd1;
            for (i = 0; i < 8; i = i + 1) RSTK[i] <= 20'd0;
            for (i = 0; i < 32; i = i + 1) ib[i]  <= 4'd0;
            mem_addr  <= 20'd0;
            mem_we    <= 1'b0;
            mem_re    <= 1'b0;
            mem_wdata <= 4'd0;
            step_done <= 1'b0;
            fetch_i   <= 6'd0;
            mem_i     <= 5'd0;
            memop_is_write  <= 1'b0;
            memop_addr      <= 20'd0;
            memop_data      <= 64'd0;
            memop_dst_reg   <= 2'd0;
            memop_len       <= 5'd0;
            memop_start_nib <= 4'd0;
        end else begin
            step_done <= 1'b0;

            case (state)
            // ── wait for external step pulse ─────────────────────
            S_IDLE: begin
                mem_we <= 1'b0;
                mem_re <= 1'b0;
                if (step) begin
                    fetch_i  <= 6'd0;
                    mem_addr <= PC;
                    mem_re   <= 1'b1;
                    state    <= S_FETCH;
                end
            end

            // ── pull 32 nibbles into ib[0..31] ───────────────────
            S_FETCH: begin
                ib[fetch_i] <= mem_rdata;
                if (fetch_i == 6'd31) begin
                    mem_re <= 1'b0;
                    state  <= S_EXEC;
                end else begin
                    mem_addr <= PC + {14'd0, fetch_i} + 20'd1;
                    fetch_i  <= fetch_i + 6'd1;
                end
            end

            // ── latch all next_* from saturn_exec in one cycle ───
            S_EXEC: begin
                PC      <= pc_next_w;
                regA    <= regA_next_w;
                regB    <= regB_next_w;
                regC    <= regC_next_w;
                regD    <= regD_next_w;
                regR[0] <= regR0_next_w;
                regR[1] <= regR1_next_w;
                regR[2] <= regR2_next_w;
                regR[3] <= regR3_next_w;
                regR[4] <= regR4_next_w;
                D0      <= D0_next_w;
                D1      <= D1_next_w;
                P       <= P_next_w;
                ST      <= ST_next_w;
                PSTAT   <= PSTAT_next_w;
                CARRY   <= CARRY_next_w;
                HEXMODE <= HEXMODE_next_w;
                OUT_REG <= OUT_REG_next_w;

                // RSTK update (push takes priority over pop; only one fires
                // per instruction per the exec decoder).
                if (rstk_push_en_w) begin
                    if (RSTKP >= 4'sd7) begin
                        // full stack: shift everything down, drop oldest,
                        // RSTKP stays at 7
                        for (k = 1; k < 8; k = k + 1)
                            RSTK[k - 1] <= RSTK[k];
                        RSTK[7] <= rstk_push_val_w;
                    end else begin
                        RSTKP         <= RSTKP + 4'sd1;
                        RSTK[RSTKP + 4'sd1] <= rstk_push_val_w;
                    end
                end else if (rstk_pop_en_w) begin
                    if (RSTKP >= 0) RSTKP <= RSTKP - 4'sd1;
                end

                // Hand off to memop or wrap up this instruction.
                if (do_memop_w) begin
                    mem_i           <= 5'd0;
                    memop_is_write  <= memop_is_write_w;
                    memop_addr      <= memop_addr_w;
                    memop_data      <= memop_data_w;
                    memop_dst_reg   <= memop_dst_reg_w;
                    memop_len       <= memop_len_w;
                    memop_start_nib <= memop_start_nib_w;
                    if (memop_is_write_w) begin
                        mem_addr  <= memop_addr_w;
                        mem_wdata <= memop_data_w[memop_start_nib_w * 4 +: 4];
                        mem_we    <= 1'b1;
                        state     <= S_MEMW;
                    end else begin
                        mem_addr <= memop_addr_w;
                        mem_re   <= 1'b1;
                        state    <= S_MEMR;
                    end
                end else begin
                    state     <= S_DONE;
                    step_done <= 1'b1;
                end
            end

            // ── one nibble per cycle, addr bumps by 1 ────────────
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

            default: state <= S_IDLE;
            endcase
        end
    end

endmodule
