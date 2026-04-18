// tb_cpu.v — replay test_vectors.dat against saturn_cpu.
//
// Each test:
//   - apply PRE state via hierarchical refs
//   - write INST nibbles into flat memory at PC
//   - pulse `step`, wait for `step_done`
//   - compare every field of POST to DUT state

`timescale 1ns/1ps
`include "saturn_pkg.vh"

module tb_cpu;
    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;

    // flat 1 MB nibble memory
    reg [3:0] mem [0:(1<<20)-1];

    wire [19:0] mem_addr;
    wire        mem_we, mem_re;
    wire [3:0]  mem_wdata;
    reg  [3:0]  mem_rdata;

    wire [19:0] dbg_pc;
    wire [63:0] dbg_A, dbg_B, dbg_C, dbg_D;
    wire [19:0] dbg_D0, dbg_D1;
    wire [3:0]  dbg_P, dbg_ST;
    wire [15:0] dbg_PSTAT;
    wire        dbg_CARRY, dbg_HEXMODE;
    wire signed [3:0] dbg_RSTKP;
    wire [159:0] dbg_RSTK;
    reg         step;
    wire        step_done;

    saturn_cpu dut (
        .clk(clk), .rst_n(rst_n),
        .mem_addr(mem_addr), .mem_we(mem_we), .mem_re(mem_re),
        .mem_wdata(mem_wdata), .mem_rdata(mem_rdata),
        .dbg_pc(dbg_pc),
        .dbg_A(dbg_A), .dbg_B(dbg_B), .dbg_C(dbg_C), .dbg_D(dbg_D),
        .dbg_D0(dbg_D0), .dbg_D1(dbg_D1),
        .dbg_P(dbg_P), .dbg_ST(dbg_ST), .dbg_PSTAT(dbg_PSTAT),
        .dbg_CARRY(dbg_CARRY), .dbg_HEXMODE(dbg_HEXMODE),
        .dbg_RSTKP(dbg_RSTKP), .dbg_RSTK(dbg_RSTK),
        .step(step), .step_done(step_done)
    );

    always @(posedge clk) if (mem_we) mem[mem_addr] <= mem_wdata;
    always @(*) mem_rdata = mem[mem_addr];

    integer fh, r;
    integer ntests;
    integer pass_count, fail_count;

    // state parsed from one "P " or "O " line
    reg [19:0] st_pc;
    reg [63:0] st_A, st_B, st_C, st_D;
    reg [19:0] st_D0, st_D1;
    reg [3:0]  st_P, st_ST;
    reg [15:0] st_PSTAT;
    reg        st_CARRY, st_HEX;
    reg [3:0]  st_RSTKP_u;      // 4-bit 2's-comp
    reg signed [3:0] st_RSTKP;
    reg [19:0] st_RSTK [0:7];
    reg [63:0] st_R [0:4];
    reg [15:0] st_IN;
    reg [11:0] st_OUT;

    reg [3:0]  inst [0:7];
    reg [8*64-1:0] name_buf;
    reg [8*512-1:0] linebuf;

    task read_state_line;
        begin
            r = $fscanf(fh,
                "%h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h\n",
                st_pc, st_A, st_B, st_C, st_D, st_D0, st_D1,
                st_P, st_ST, st_PSTAT, st_CARRY, st_HEX, st_RSTKP_u,
                st_RSTK[0], st_RSTK[1], st_RSTK[2], st_RSTK[3],
                st_RSTK[4], st_RSTK[5], st_RSTK[6], st_RSTK[7],
                st_R[0], st_R[1], st_R[2], st_R[3], st_R[4],
                st_IN, st_OUT);
            st_RSTKP = $signed(st_RSTKP_u);
        end
    endtask

    task force_pre;
        integer j;
        begin
            dut.regA = st_A; dut.regB = st_B; dut.regC = st_C; dut.regD = st_D;
            dut.PC = st_pc; dut.D0 = st_D0; dut.D1 = st_D1;
            dut.P  = st_P;  dut.ST = st_ST; dut.PSTAT = st_PSTAT;
            dut.CARRY = st_CARRY; dut.HEXMODE = st_HEX;
            dut.RSTKP = st_RSTKP;
            for (j = 0; j < 8; j = j + 1) dut.RSTK[j] = st_RSTK[j];
            for (j = 0; j < 5; j = j + 1) dut.regR[j] = st_R[j];
            dut.IN_REG[15:0]  = st_IN;
            dut.OUT_REG[11:0] = st_OUT;
        end
    endtask

    task write_inst_to_mem;
        input [19:0] addr;
        integer j;
        begin
            for (j = 0; j < 8; j = j + 1)
                mem[(addr + j) & 20'hFFFFF] = inst[j];
        end
    endtask

    task run_step;
        integer timeout;
        begin
            step = 1;
            @(posedge clk);
            step = 0;
            timeout = 0;
            while (!step_done && timeout < 200) begin
                @(posedge clk);
                timeout = timeout + 1;
            end
            @(posedge clk);
        end
    endtask

    // Compare helper — returns 1 if mismatch and prints diagnostic.
    function integer mism;
        input [8*64-1:0] field;
        input [63:0] got, exp;
        begin
            if (got !== exp) begin
                $display("    MISMATCH %0s: got=%016h exp=%016h", field, got, exp);
                mism = 1;
            end else mism = 0;
        end
    endfunction

    task check_post;
        input [8*64-1:0] tname;
        integer nerr;
        integer j;
        begin
            nerr = 0;
            nerr = nerr + mism("PC",    {44'd0, dut.PC}, {44'd0, st_pc});
            nerr = nerr + mism("A",     dut.regA, st_A);
            nerr = nerr + mism("B",     dut.regB, st_B);
            nerr = nerr + mism("C",     dut.regC, st_C);
            nerr = nerr + mism("D",     dut.regD, st_D);
            nerr = nerr + mism("D0",    {44'd0, dut.D0}, {44'd0, st_D0});
            nerr = nerr + mism("D1",    {44'd0, dut.D1}, {44'd0, st_D1});
            nerr = nerr + mism("P",     {60'd0, dut.P},  {60'd0, st_P});
            nerr = nerr + mism("ST",    {60'd0, dut.ST}, {60'd0, st_ST});
            nerr = nerr + mism("PSTAT", {48'd0, dut.PSTAT}, {48'd0, st_PSTAT});
            nerr = nerr + mism("CARRY", {63'd0, dut.CARRY}, {63'd0, st_CARRY});
            nerr = nerr + mism("HEX",   {63'd0, dut.HEXMODE}, {63'd0, st_HEX});
            nerr = nerr + mism("RSTKP", {60'd0, dut.RSTKP},   {60'd0, st_RSTKP});
            for (j = 0; j < 8; j = j + 1)
                nerr = nerr + mism("RSTK", {44'd0, dut.RSTK[j]}, {44'd0, st_RSTK[j]});
            for (j = 0; j < 5; j = j + 1)
                nerr = nerr + mism("R",    dut.regR[j], st_R[j]);
            if (nerr == 0) begin
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL %0s  (%0d mismatches)", tname, nerr);
                fail_count = fail_count + 1;
            end
        end
    endtask

    integer i;
    integer tok;

    initial begin
        pass_count = 0; fail_count = 0;
        step = 0;
        rst_n = 0;
        for (i = 0; i < (1<<20); i = i + 1) mem[i] = 4'h0;
        #25 rst_n = 1;
        @(posedge clk);

        fh = $fopen("test_vectors.dat", "r");
        if (fh == 0) begin
            $display("ERROR: cannot open test_vectors.dat");
            $finish;
        end

        r = $fscanf(fh, "%d\n", ntests);
        $display("Running %0d test vectors (fscanf r=%0d)", ntests, r);

        for (i = 0; i < ntests; i = i + 1) begin : test_loop
            integer mj, mk, mcnt;
            reg [19:0] mem_addr_v;
            reg [63:0] mem_data_v;
            r = $fscanf(fh, "%s\n", name_buf);
            read_state_line();
            force_pre();
            // memory preloads
            r = $fscanf(fh, "%d\n", mcnt);
            for (mj = 0; mj < mcnt; mj = mj + 1) begin
                r = $fscanf(fh, "%h %h\n", mem_addr_v, mem_data_v);
                for (mk = 0; mk < 16; mk = mk + 1)
                    mem[(mem_addr_v + mk) & 20'hFFFFF] = mem_data_v[mk*4 +: 4];
            end
            r = $fscanf(fh, "%h %h %h %h %h %h %h %h\n",
                        inst[0], inst[1], inst[2], inst[3],
                        inst[4], inst[5], inst[6], inst[7]);
            write_inst_to_mem(st_pc);
            run_step();
            read_state_line();
            check_post(name_buf);
        end

        $fclose(fh);
        $display("======================================================");
        $display("  DONE  %0d/%0d passed  (%0d failed)", pass_count, ntests, fail_count);
        $display("======================================================");
        $finish;
    end

    // overall timeout
    initial begin
        #50000000;
        $display("TIMEOUT overall");
        $finish;
    end
endmodule
