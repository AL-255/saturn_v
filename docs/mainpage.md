# HP Saturn CPU (Verilog)

Synthesizable RTL re-implementation of the HP Saturn CPU, verified
bit-for-bit against the C core inside the vendored
[x48ng](https://codeberg.org/gwh/x48ng) HP48 emulator.

## Modules

| Module | Role | File |
|--------|------|------|
| `saturn_cpu`   | Top level. State regs + fetch/memop FSM + ALU plumbing.     | @ref saturn_cpu.v   |
| `saturn_exec`  | Combinational instruction decoder and next-state producer. | @ref saturn_exec.v  |
| `saturn_alu`   | Field-based 64-bit ALU (arith, logic, shift, compare).     | @ref saturn_alu.v   |
| `saturn_field` | Field-code → (start_nib, end_nib) decoder.                 | @ref saturn_field.v |

Constants shared across modules (register indices, status bits, field
codes, ALU op codes) live in `rtl/saturn_pkg.vh`.

## Build the docs

From the repo root:

    ./docs/build.sh

The script locates `doxygen` (via `$PATH` or `~/.local/doxygen/...`) and
drops HTML output into `docs/html/`. Open `docs/html/index.html`.

## Block diagrams

Graphviz block / hierarchy diagrams of the RTL are produced by a
separate script — see `rtl/draw_diagram.py`. They complement the HTML
docs: the Python script gives you a visual "boxes and wires" view, the
doxygen site gives you per-port descriptions and source cross-links.

## Reference

- **C reference**: `x48ng/src/core/emulate.c` and `registers.c` (the
  behavioral model every RTL decision is compared against).
- **Integration harness**: `sim/x48_sim`, which swaps between the C
  core and this Verilog CPU via a linker-wrap on `step_instruction`.
- **Equivalence testing**: `make -C sim compare` (prog.hex) and
  `make -C sim compare-rom` (HP 48 GX ROM up to 500k instructions).
