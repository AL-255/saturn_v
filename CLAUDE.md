# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Purpose

This repo is a behavioral Verilog reimplementation of the HP Saturn CPU, verified bit-for-bit against the C core inside the vendored `x48ng` HP48 emulator. The Verilog is the thing being built; `x48ng` is the reference oracle. "Equivalence" here specifically means: same PC, full 64-bit A/B/C/D, D0/D1, P, ST, 16-bit PSTAT, carry, hex/dec mode, RSTK, and scratch registers at every instruction boundary.

## Component map

- `rtl/` — the Verilog CPU (`saturn_cpu.v` + `saturn_alu.v` + `saturn_field.v`, shared defs in `saturn_pkg.vh`). Testbenches `tb_cpu.v` / `tb_alu.v`. `convert_vectors.py` turns `../test_harness/test_vectors.txt` (human-readable, LSN-first) into `test_vectors.dat` (MSN-first, `$sscanf`-friendly).
- `test_harness/` — drives `x48ng`'s `emulate.c` + `registers.c` in isolation via `flat_memory.c` (1 M-nibble flat RAM, no bank switching) and `stubs.c` (timer/UI/debugger stubs). `generate_tests.c` emits per-instruction PRE/INST/POST vectors that are the source of truth for RTL equivalence tests.
- `sim/` — two drop-in tracers over the same instruction stream:
  - `csim` links `x48ng`'s C core with the harness shims.
  - `vsim` is a Verilator C++ harness around `saturn_cpu.v` servicing a 1 M-nibble memory bus (pulse `step`, wait `step_done`, dump `dbg_*`).
  - Lockstep equivalence = run both, diff traces (both use identical line-per-instruction format).
  - `Makefile_full` builds a different beast: `x48_sim`, the full x48ng SDL/ncurses/GTK GUI with `--core=c|rtl|lockstep`. It uses `ld --wrap=step_instruction` to splice in a dispatcher (`step_dispatch.c`) without modifying `x48ng` sources; the RTL path requires Verilator's `--public-flat-rw` so C++ can sync state in/out of the model.
- `x48ng/` — vendored HP48 emulator (upstream: codeberg.org/gwh/x48ng). **Treat as a read-only reference.** The test harness and sim link its `src/core/*.c` directly; changing those files will silently break the equivalence claim.
- `rom` — HP 48 GX ROM, packed 2-nibbles-per-byte (header `32 96 1b 80 …`), used for long `compare-rom` runs.
- `SDL3-3.4.4/` — vendored SDL3 source used when building the full GUI sim; not needed for `csim`/`vsim`/`tb_cpu` work.

## Commands

All `make` invocations are per-subdir; there is no top-level Makefile.

```sh
# Regenerate the 190-case test vectors from the C reference (writes test_vectors.txt):
make -C test_harness

# RTL testbench driving those vectors through saturn_cpu via Verilator:
cd rtl && python3 convert_vectors.py          # writes test_vectors.dat (MSN-first)
# (then run tb_cpu / tb_alu via your Verilator or iverilog setup;
#  obj_dir/ and obj_dir_pub/ already contain prior Verilator output)

# Lockstep tracer build + equivalence run on prog.hex:
make -C sim                                   # builds csim and vsim
make -C sim compare                           # runs prog.hex on both, diffs traces

# Same, against the real HP 48 GX ROM:
make -C sim compare-rom                       # default 50,000 instructions
make -C sim compare-rom ROM_STEPS=500000      # verified lockstep to 500k
make -C sim compare-rom ROM=/path/to/alt.rom

# Full x48ng GUI with swappable core (requires SDL3, gtk4, ncursesw, lua5.4, readline):
make -C sim -f Makefile_full
./sim/x48_sim --core=rtl ../rom               # Verilog CPU driving x48ng's peripherals
./sim/x48_sim --core=lockstep ../rom          # aborts on any divergence
```

Verilator is pinned to `/tools/verilator/v5.046/bin/verilator` in both sim Makefiles — override `VERILATOR=` (or `VER=`) on the make line if that path isn't valid locally.

## Invariants worth knowing before editing

- **Nibble ordering is inconsistent by design.** The C harness prints registers LSN-first (`nibble[0]` first) because that's the Saturn architectural layout. `convert_vectors.py` reverses to MSN-first for Verilog `$sscanf`. Any new trace emitter must pick a side and be consistent — `sim/csim` and `sim/vsim` both print MSN-first (natural C `%X` of a 64-bit int) specifically so their traces diff cleanly.
- **`harness.h`'s `config_t` must match `x48ng/src/options.h` field-for-field.** `emulate.c` reads `x48ng_config.inhibit_shutdown` by offset; a mismatched struct silently corrupts control flow.
- **`test_harness/flat_memory.c` implements the real `bus_fetch_nibble` / `bus_write_nibble` symbols.** It replaces `x48ng`'s banked MMU with 1 MB of flat nibbles. Don't `#include "memory.c"`; link against the harness version.
- **Instruction dispatch in `saturn_cpu.v` mirrors `emulate.c`'s nested switch** (group by `n0`, sub-dispatch on `n1`, then `n2`/`n3` for 0E/10-15/80-81). When adding opcodes, follow the same tree — divergence here is the most common source of trace mismatches.
- **Trace format is the contract.** One line per completed instruction: `STEP n PC= A= B= C= D= D0= D1= P= ST= PSTAT= CARRY= HEX=`. `sim/Makefile` diffs these after stripping `#` comments; anything that perturbs whitespace or field order breaks `compare`/`compare-rom`.
- **`Makefile_full`'s `--wrap=step_instruction` means both the real and wrapped symbol must stay linkable.** Don't rename `step_instruction` in `x48ng/src/core/emulate.c`; the linker trick is how we avoid forking x48ng.
