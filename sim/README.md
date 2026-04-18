# sim/ — Verilog CPU as drop-in replacement for x48ng's C core

This folder contains two side-by-side simulators of the HP Saturn CPU,
both running against the same `prog.hex` format and emitting identical
line-per-instruction traces:

| binary   | CPU source            | library path                           |
| -------- | --------------------- | -------------------------------------- |
| `csim`   | **C** (`emulate.c`)   | `../x48ng/src/core/emulate.c`          |
| `vsim`   | **Verilog** RTL       | `../rtl/saturn_cpu.v`                  |

Identical state each step = Verilog is behaviorally equivalent to the
original C core of x48ng.

## Build & run

```sh
make                                       # builds both csim and vsim
make compare                               # runs prog.hex on both and diffs
make compare-rom                           # boots ../rom through both
make compare-rom ROM_STEPS=500000          # longer run
make compare-rom ROM=/path/to/other.rom    # different ROM
```

Sample output:

```
===============================================
  MATCH: Verilog and C traces identical over
         50000 ROM instructions.
===============================================
```

Verified lockstep on the HP 48 GX ROM (`../rom`, 0x80000 bytes packed) for
up to **500,000 instructions** — Verilog state is byte-identical to the
x48ng C core at every instruction boundary.

## Files

- **`prog.hex`, `prog2.hex`** — hand-written Saturn programs:
  whitespace-separated hex nibbles, `#` starts a line comment, stream
  loads into `mem[0..]` (nibble 0 = PC fetches first).
- **`csim.c`** — links `x48ng/src/core/{emulate,registers}.c` with
  `test_harness/flat_memory.c` and `stubs.c`. Calls `step_instruction()`
  N times and dumps state after each. Supports `--rom <file>` to load a
  packed HP48 ROM (0x32,0x96,0x1b,0x80-headered) with `mem[2i] = byte&0xf`,
  `mem[2i+1] = byte>>4`.
- **`vsim.cpp`** — Verilator C++ harness. Owns a 1 M-nibble memory array,
  services the CPU's memory bus each cycle, pulses `step`, waits for
  `step_done`, then dumps state from the `dbg_*` ports. Same `--rom`
  loader.
- **`load_prog.h`** — shared loaders: `load_prog()` for hex text,
  `load_rom_bin()` for packed 2-nibble-per-byte ROMs.
- **`Makefile`** — `all`, `compare`, `compare-rom` targets.

## Trace format

```
STEP <n> PC=<5hex> A=<16hex> B=<16hex> C=<16hex> D=<16hex>
         D0=<5hex> D1=<5hex> P=<1hex> ST=<1hex> PSTAT=<4hex>
         CARRY=<0|1> HEX=<0|1>
```

All hex is MSN-first (natural C formatting). One line per completed
instruction.

## What this proves

Running any program through both binaries and diffing the traces is a
bit-for-bit equivalence check of the Verilog against the x48ng reference
on: PC, A/B/C/D full 64-bit width, both address registers, P pointer,
status register, all 16 program-status bits, carry, and hex/dec mode.

`test_harness/` already drives 190 independent test vectors through both
cores (well — the Verilog through Verilator, the C through the real
emulate.c). This `sim/` folder turns that into a program-level replay so
you can run an actual Saturn instruction stream through the Verilog.
