#
# sim/Makefile — build C-core and Verilog-core Saturn tracers.
#
# csim : x48ng emulate.c + registers.c linked with flat memory / stubs.
# vsim : Verilator-built saturn_cpu.v driven by a C++ main that services a
#        nibble memory bus.
#
# Run both, diff their traces, and you've verified the Verilog matches the
# original C behavior instruction-for-instruction.
#

CORE       := ../x48ng/src/core
HARNESS    := ../test_harness
RTL        := ../rtl
VERILATOR  := /tools/verilator/v5.046/bin/verilator
LUA_INC    := $(shell pkg-config --cflags lua5.4 2>/dev/null || echo -I/usr/include/lua5.4)

CFLAGS     := -std=c99 -O0 -g -Wall -Wextra \
              -I$(CORE) -I$(CORE)/.. -I$(HARNESS) -I. $(LUA_INC) \
              -DTEST_HARNESS=1
CXXFLAGS   := -std=c++17 -O2 -I.

VFLAGS     := --cc --exe --build --timing \
              -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNOPTFLAT -Wno-CASEINCOMPLETE \
              -Wno-LATCH -Wno-SYMRSVDWORD -Wno-BLKANDNBLK -Wno-MULTIDRIVEN \
              -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM \
              --top-module saturn_cpu \
              -I$(RTL)

CORE_SRCS  := $(CORE)/emulate.c $(CORE)/registers.c \
              $(HARNESS)/flat_memory.c $(HARNESS)/stubs.c

all: csim vsim

csim: csim.c $(CORE_SRCS)
	$(CC) $(CFLAGS) -o $@ csim.c $(CORE_SRCS)

RTL_SRCS := $(RTL)/saturn_cpu.v $(RTL)/saturn_exec.v $(RTL)/saturn_alu.v $(RTL)/saturn_field.v

vsim: vsim.cpp $(RTL_SRCS)
	$(VERILATOR) $(VFLAGS) --Mdir obj_dir_v -CFLAGS "$(CXXFLAGS)" \
	    $(RTL_SRCS) \
	    vsim.cpp
	cp obj_dir_v/Vsaturn_cpu ./vsim

ROM       ?= ../rom
ROM_STEPS ?= 50000

compare: all
	./csim prog.hex 32 trace_c.log
	./vsim prog.hex 32 trace_v.log
	@grep -v '^#' trace_c.log > /tmp/.tc.$$$$ ; \
	 grep -v '^#' trace_v.log > /tmp/.tv.$$$$ ; \
	 if diff -q /tmp/.tc.$$$$ /tmp/.tv.$$$$ > /dev/null; then \
	    echo ""; \
	    echo "==============================================="; \
	    echo "  MATCH: Verilog and C traces identical over"; \
	    echo "         $$(grep -c '^STEP' trace_c.log) instructions."; \
	    echo "==============================================="; \
	 else \
	    echo ""; \
	    echo "==============================================="; \
	    echo "  DIFF found:"; \
	    diff -u /tmp/.tc.$$$$ /tmp/.tv.$$$$ | head -40; \
	    echo "==============================================="; \
	 fi ; \
	 rm -f /tmp/.tc.$$$$ /tmp/.tv.$$$$

compare-rom: all
	./csim --rom $(ROM) $(ROM_STEPS) trace_c.log
	./vsim --rom $(ROM) $(ROM_STEPS) trace_v.log
	@grep -v '^#' trace_c.log > /tmp/.rc.$$$$ ; \
	 grep -v '^#' trace_v.log > /tmp/.rv.$$$$ ; \
	 if diff -q /tmp/.rc.$$$$ /tmp/.rv.$$$$ > /dev/null; then \
	    echo ""; \
	    echo "==============================================="; \
	    echo "  MATCH: Verilog and C traces identical over"; \
	    echo "         $$(grep -c '^STEP' trace_c.log) ROM instructions."; \
	    echo "==============================================="; \
	 else \
	    echo ""; \
	    echo "==============================================="; \
	    echo "  DIFF found:"; \
	    diff -u /tmp/.rc.$$$$ /tmp/.rv.$$$$ | head -40; \
	    echo "==============================================="; \
	 fi ; \
	 rm -f /tmp/.rc.$$$$ /tmp/.rv.$$$$

clean:
	rm -rf csim vsim obj_dir_v trace_c.log trace_v.log

.PHONY: all compare compare-rom clean
