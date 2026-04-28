# saturn_sim

Behavioral Verilog reimplementation of the HP Saturn CPU, verified
bit-for-bit against the C core inside the vendored
[`x48ng`](x48ng/) HP48 emulator. The Verilog (`rtl/saturn_cpu.v`) is the
thing being built; `x48ng` is the reference oracle.

"Equivalence" here means: same PC, full 64-bit A/B/C/D, D0/D1, P, ST,
16-bit PSTAT, carry, hex/dec mode, RSTK, and scratch registers at every
instruction boundary.

See [`CLAUDE.md`](CLAUDE.md) for the component map and architectural
invariants, and [`ISA.md`](ISA.md) for the instruction-set reference.

## Prerequisites

- **Verilator** ≥ 5.046. Both `sim/Makefile` and `sim/Makefile_full`
  hard-code `/tools/verilator/v5.046/bin/verilator`. Override on the
  make line if your install lives elsewhere:
  ```sh
  make -C sim VERILATOR=/usr/local/bin/verilator
  make -C sim -f Makefile_full VER=/usr/local/bin/verilator
  ```
- **C toolchain**: `gcc`, `g++` (C++17).
- **lua5.4** dev headers (linked by both the harness and `csim`).
- **GUI build only** (`Makefile_full`): `pkg-config`, `glib-2.0`,
  `gtk4`, `ncursesw`, `readline`, `sdl3` (vendored under
  [`SDL3-3.4.4/`](SDL3-3.4.4) — build/install separately if your distro
  doesn't package it).

## Quickstart — lockstep equivalence on a tiny program

```sh
make -C sim                  # builds csim (C reference) and vsim (Verilog)
make -C sim compare          # runs sim/prog.hex on both, diffs traces
```

Expected output:

```
===============================================
  MATCH: Verilog and C traces identical over
         <N> instructions.
===============================================
```

## Lockstep on the real HP 48 GX ROM

```sh
make -C sim compare-rom                          # default 50,000 instructions
make -C sim compare-rom ROM_STEPS=500000         # verified lockstep to 500k
make -C sim compare-rom ROM=/path/to/alt.rom     # different ROM image
```

The ROM at `./rom` is packed two-nibbles-per-byte (header
`32 96 1b 80 …`); the loader unpacks it into the harness's flat 1 M-nibble
memory.

## Per-instruction test vectors (190 cases)

The C reference generates PRE/INST/POST vectors that are the source of
truth for the RTL testbench:

```sh
make -C test_harness         # builds generate_tests, writes test_vectors.txt
cd rtl && python3 convert_vectors.py
                             # converts LSN-first text → MSN-first test_vectors.dat
                             # for $sscanf inside tb_cpu.v / tb_alu.v
```

Run `tb_cpu.v` / `tb_alu.v` through your Verilator (or iverilog) setup
of choice — `rtl/obj_dir*/` already contain prior Verilator output as a
reference.

## Full GUI sim with swappable core

`sim/Makefile_full` wraps the entire x48ng SDL/GTK/ncurses GUI around a
runtime-selectable CPU core. Uses `ld --wrap=step_instruction` to splice
in a dispatcher (`sim/step_dispatch.c`) without modifying any vendored
`x48ng` source.

```sh
make -C sim -f Makefile_full
./sim/x48_sim --core=c        rom    # x48ng's original C CPU
./sim/x48_sim --core=rtl      rom    # Verilog saturn_cpu.v driving x48ng peripherals
./sim/x48_sim --core=lockstep rom    # both step together; abort on divergence
```

The `rtl` and `lockstep` cores require Verilator's `--public-flat-rw`
(already set in `Makefile_full`'s `VFLAGS`) so the C++ shim can sync
state in and out of the model.

## Trace format

One line per completed instruction, identical between `csim` and `vsim`:

```
STEP <n> PC=<5hex> A=<16hex> B=<16hex> C=<16hex> D=<16hex>
         D0=<5hex> D1=<5hex> P=<1hex> ST=<1hex> PSTAT=<4hex>
         CARRY=<0|1> HEX=<0|1>
```

All hex MSN-first. `make compare` / `compare-rom` strip `#` comment
lines before diffing — anything that perturbs whitespace or field order
will break equivalence.

## Layout

| Path             | Role                                                                |
| ---------------- | ------------------------------------------------------------------- |
| `rtl/`           | Verilog CPU + ALU + field unit, plus `tb_cpu` / `tb_alu`            |
| `test_harness/`  | Vector generator linking `x48ng/src/core/*.c` with flat memory      |
| `sim/`           | `csim` (C reference) + `vsim` (Verilator) tracers; full-GUI variant |
| `x48ng/`         | Vendored HP48 emulator — read-only reference                        |
| `rom`            | HP 48 GX ROM (packed 2-nibbles-per-byte)                            |
| `SDL3-3.4.4/`    | Vendored SDL3 source (only needed for `Makefile_full`)              |
| `ISA.md`         | Saturn instruction-set reference + encoding chart                   |
| `CLAUDE.md`      | Architectural invariants and component map                          |
