#!/usr/bin/env python3
"""Generate a block diagram of the Saturn CPU RTL.

Parses the Verilog files in this directory for module declarations, ports,
and instantiations, then emits a graphviz .dot file that shows the top
module (saturn_cpu by default) as a cluster containing its submodule
instances, with inter-instance wires labeled by the names used in the
port-map. If `dot` is on $PATH the .dot is also rendered to SVG (and PNG).

Usage:
    rtl/draw_diagram.py                         # writes rtl/diagram/*
    rtl/draw_diagram.py --top saturn_alu
    rtl/draw_diagram.py --out /tmp/foo
    rtl/draw_diagram.py --no-render             # stop after writing .dot
"""
from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path


# ── Verilog parsing ─────────────────────────────────────────────────────

STRIP_LINE_COMMENT = re.compile(r"//[^\n]*")
STRIP_BLOCK_COMMENT = re.compile(r"/\*.*?\*/", re.DOTALL)

# module NAME (ansi-style port list)
MODULE_RE = re.compile(
    r"\bmodule\s+(\w+)\s*(?:#\([^)]*\))?\s*\((.*?)\)\s*;(.*?)\bendmodule\b",
    re.DOTALL,
)

# An ANSI port declaration inside the header: input/output/inout [...] NAME[, NAME...]
PORT_DECL_RE = re.compile(
    r"""
    (?P<dir>input|output|inout)\s+
    (?:(?P<nettype>wire|reg|logic)\s+)?
    (?P<signed>signed\s+)?
    (?P<width1>\[[^\]]+\]\s*)?
    (?P<signed2>signed\s+)?
    (?P<width2>\[[^\]]+\]\s*)?
    (?P<names>[\w\s,]+)
    """,
    re.VERBOSE,
)

# Instance pattern: MODNAME INSTNAME ( ... );
# The body can have newlines and nested parens (for wire indexing),
# so we capture greedy-balance-ish: up to the matching ")" followed by ";".
# Good enough for our codebase, which doesn't use obscure constructs.
INST_RE = re.compile(
    r"\b(?P<mod>\w+)\s+(?P<inst>\w+)\s*\((?P<body>[^;]*?)\)\s*;",
    re.DOTALL,
)

# Named-port map entry: .PORT(WIRE_OR_EXPR)
PORT_MAP_RE = re.compile(r"\.(\w+)\s*\(\s*([^),]+?)\s*\)")

# Keywords that can look like `WORD identifier (` but aren't instantiations.
VERILOG_KEYWORDS = {
    "if", "for", "while", "case", "casex", "casez", "begin", "end",
    "module", "endmodule", "always", "initial", "assign", "wire", "reg",
    "input", "output", "inout", "function", "endfunction", "task",
    "endtask", "generate", "endgenerate", "localparam", "parameter",
    "genvar", "integer", "return", "disable", "logic", "typedef",
    "package", "endpackage", "import", "export", "specify", "endspecify",
    "fork", "join", "join_any", "join_none", "repeat", "forever",
    "default", "endcase", "do", "static", "automatic", "const", "signed",
    "unsigned", "posedge", "negedge", "or", "and", "not", "xor", "nor",
    "nand", "xnor", "buf", "supply0", "supply1", "pullup", "pulldown",
}


@dataclass
class Port:
    direction: str   # input / output / inout
    width:     str   # e.g. "[63:0]"; empty = 1-bit
    name:      str


@dataclass
class Instance:
    module:   str                       # submodule type
    name:     str                       # instance name
    port_map: dict[str, str] = field(default_factory=dict)  # port -> wire expr


@dataclass
class Module:
    name:      str
    file:      str
    ports:     list[Port]                 = field(default_factory=list)
    instances: list[Instance]             = field(default_factory=list)


def parse_ports(header: str) -> list[Port]:
    """Pull input/output/inout declarations out of an ANSI port header."""
    ports: list[Port] = []
    # The header may have commas separating declarations, but widths have
    # brackets. Split on commas that aren't inside [].
    depth = 0
    buf   = []
    parts = []
    for ch in header:
        if ch == "[":
            depth += 1
            buf.append(ch)
        elif ch == "]":
            depth -= 1
            buf.append(ch)
        elif ch == "," and depth == 0:
            parts.append("".join(buf))
            buf = []
        else:
            buf.append(ch)
    if buf:
        parts.append("".join(buf))

    current_dir = None
    current_width = ""
    for raw in parts:
        s = raw.strip()
        if not s:
            continue
        m = PORT_DECL_RE.match(s)
        if m:
            current_dir = m.group("dir")
            w1 = (m.group("width1") or "").strip()
            w2 = (m.group("width2") or "").strip()
            current_width = (w1 + " " + w2).strip()
            names = m.group("names")
        elif current_dir:
            # continuation: just a name or [width] name
            # try to match a leading width
            wm = re.match(r"\s*(\[[^\]]+\])?\s*(.*)$", s)
            w = (wm.group(1) or "").strip()
            names = wm.group(2)
            if w:
                current_width = w
        else:
            continue
        for n in names.split(","):
            n = n.strip()
            if not n:
                continue
            ports.append(Port(current_dir, current_width, n))
    return ports


def parse_instances(body: str, known_modules: set[str]) -> list[Instance]:
    """Find `SUBMOD INSTNAME (...)` patterns in a module body."""
    out: list[Instance] = []
    for m in INST_RE.finditer(body):
        submod = m.group("mod")
        inst   = m.group("inst")
        if submod in VERILOG_KEYWORDS or inst in VERILOG_KEYWORDS:
            continue
        # Only accept known module names OR anything that looks like a
        # project-local module (saturn_*). Anything else is probably a
        # function call or Verilog construct we don't care about.
        if submod not in known_modules and not submod.startswith("saturn_"):
            continue
        pm = dict(PORT_MAP_RE.findall(m.group("body")))
        out.append(Instance(submod, inst, pm))
    return out


def parse_files(files: list[Path]) -> dict[str, Module]:
    """Two-pass parse: first collect module names, then parse instances."""
    texts = []
    for f in files:
        raw = f.read_text()
        raw = STRIP_BLOCK_COMMENT.sub("", raw)
        raw = STRIP_LINE_COMMENT.sub("", raw)
        texts.append((f, raw))

    modules: dict[str, Module] = {}
    # pass 1: module names + ports
    for f, text in texts:
        for m in MODULE_RE.finditer(text):
            name   = m.group(1)
            header = m.group(2)
            modules[name] = Module(name=name, file=f.name, ports=parse_ports(header))

    # pass 2: instances
    known = set(modules)
    for f, text in texts:
        for m in MODULE_RE.finditer(text):
            parent = m.group(1)
            body   = m.group(3)
            modules[parent].instances = parse_instances(body, known)

    return modules


# ── DOT emission ───────────────────────────────────────────────────────

def dot_escape(s: str) -> str:
    return s.replace('"', r'\"').replace("<", r"\<").replace(">", r"\>")


def port_record_label(mod: Module) -> str:
    """Compact HTML-like label with port lists in a record shape."""
    ins  = [p for p in mod.ports if p.direction == "input"]
    outs = [p for p in mod.ports if p.direction == "output"]

    def port_row(p: Port) -> str:
        w = f" {p.width}" if p.width else ""
        return f'<TR><TD PORT="{p.name}" ALIGN="LEFT">{p.name}{dot_escape(w)}</TD></TR>'

    tr_title = f'<TR><TD BGCOLOR="lightblue" COLSPAN="2"><B>{mod.name}</B><BR/><I>{mod.file}</I></TD></TR>'
    if not ins and not outs:
        return f'<<TABLE BORDER="0" CELLBORDER="1" CELLSPACING="0">{tr_title}</TABLE>>'

    rows = [tr_title]
    for i in range(max(len(ins), len(outs))):
        lt = port_row(ins[i])  if i < len(ins)  else '<TD ALIGN="LEFT"> </TD>'
        rt = port_row(outs[i]) if i < len(outs) else '<TD ALIGN="LEFT"> </TD>'
        # Strip wrapping <TR>...</TR> from the helper to recombine into one row.
        lt_inner = lt.replace("<TR>", "").replace("</TR>", "") if i < len(ins)  else lt
        rt_inner = rt.replace("<TR>", "").replace("</TR>", "") if i < len(outs) else rt
        rows.append(f"<TR>{lt_inner}{rt_inner}</TR>")
    return "<<TABLE BORDER=\"0\" CELLBORDER=\"1\" CELLSPACING=\"0\">" + "".join(rows) + "</TABLE>>"


def emit_block_diagram(top: str, modules: dict[str, Module]) -> str:
    """Draw `top` with each instance as a record node and connections as
    edges labeled by the shared wire name in the parent's port-map."""
    if top not in modules:
        sys.exit(f"error: module {top!r} not found (known: {sorted(modules)})")

    tm = modules[top]
    dot = [
        f'// saturn CPU block diagram — top = {top}',
        "digraph saturn {",
        '  rankdir=LR;',
        '  graph [fontname="Helvetica", fontsize=11, splines=polyline, nodesep=0.4, ranksep=1.0];',
        '  node  [fontname="Helvetica", fontsize=10];',
        '  edge  [fontname="Helvetica", fontsize=9];',
        f'  labelloc="t"; label=<<B>{top}</B> &mdash; {tm.file}>;',
        "",
    ]

    # External ports: one node per input and output, placed on left/right.
    in_ports  = [p for p in tm.ports if p.direction == "input"]
    out_ports = [p for p in tm.ports if p.direction == "output"]

    dot.append("  // top-module external ports")
    with_rank = lambda rank, names: f'  {{ rank={rank}; {" ".join(names)} }}'
    in_names  = []
    for p in in_ports:
        nid = f"ext_in_{p.name}"
        lbl = f"{p.name}\\n{p.width}" if p.width else p.name
        dot.append(f'  "{nid}" [shape=plaintext, label="{dot_escape(lbl)}", fontcolor="darkgreen"];')
        in_names.append(f'"{nid}"')
    out_names = []
    for p in out_ports:
        nid = f"ext_out_{p.name}"
        lbl = f"{p.name}\\n{p.width}" if p.width else p.name
        dot.append(f'  "{nid}" [shape=plaintext, label="{dot_escape(lbl)}", fontcolor="darkred"];')
        out_names.append(f'"{nid}"')
    dot.append(with_rank("source", in_names))
    dot.append(with_rank("sink",   out_names))
    dot.append("")

    # Instance nodes.
    dot.append("  // sub-module instances")
    for inst in tm.instances:
        sub = modules.get(inst.module)
        if sub is None:
            # unknown module (e.g. 3rd-party) — fallback to simple box
            dot.append(f'  "{inst.name}" [shape=box, style=filled, fillcolor=lightyellow, '
                       f'label="{inst.name}\\n{inst.module}"];')
        else:
            label = port_record_label(sub).replace("\n", "")
            # For HTML-like labels we use shape=plaintext.
            dot.append(f'  "{inst.name}" [shape=plaintext, label={label}];')
    dot.append("")

    # Build wire → (endpoints) map so we can draw connections.
    # Endpoints are (kind, node_id, port).
    #   kind in {"inst_in", "inst_out", "ext_in", "ext_out"}.
    wire_eps: dict[str, list[tuple[str, str, str]]] = {}

    def add_ep(wire: str, kind: str, node: str, port: str):
        wire = wire.strip()
        if not wire:
            return
        # Drop bit-select / concat for the diagram's "wire name".
        key = re.sub(r"\[.*?\]", "", wire).split("{")[0].split("}")[0].strip()
        key = key.split("&")[0].split("|")[0].strip()
        if not key or not re.match(r"^\w+$", key):
            return
        wire_eps.setdefault(key, []).append((kind, node, port))

    # Top-module external ports feed `wire`s with the same name as the port
    # (ANSI-style). So we treat each external port as having its wire name
    # equal to the port name, matched to the submodule port-map wires.
    for p in in_ports:
        add_ep(p.name, "ext_in", f"ext_in_{p.name}", p.name)
    for p in out_ports:
        add_ep(p.name, "ext_out", f"ext_out_{p.name}", p.name)

    for inst in tm.instances:
        sub = modules.get(inst.module)
        for pname, wire in inst.port_map.items():
            direction = "inout"
            if sub is not None:
                for p in sub.ports:
                    if p.name == pname:
                        direction = p.direction
                        break
            kind = "inst_in" if direction == "input" else "inst_out" if direction == "output" else "inst_io"
            add_ep(wire, kind, inst.name, pname)

    # Emit one edge per connection.
    dot.append("  // wires between instances / ports")
    def pick_driver_then_sinks(eps):
        drivers = [e for e in eps if e[0] in ("inst_out", "ext_in")]
        sinks   = [e for e in eps if e[0] in ("inst_in",  "ext_out")]
        ios     = [e for e in eps if e[0] == "inst_io"]
        return drivers + ios, sinks + ios  # io acts as both

    for wire, eps in sorted(wire_eps.items()):
        if len(eps) < 2:
            continue  # single-ended; not a useful connection
        drivers, sinks = pick_driver_then_sinks(eps)
        if not drivers or not sinks:
            # still emit undirected connections (e.g. both ends inputs)
            for i in range(len(eps) - 1):
                a, b = eps[i], eps[i + 1]
                dot.append(f'  "{a[1]}":"{a[2]}" -> "{b[1]}":"{b[2]}" [label="{wire}", arrowhead=none, color="gray"];')
            continue
        for d in drivers:
            for s in sinks:
                if d[1] == s[1] and d[2] == s[2]:
                    continue
                dot.append(f'  "{d[1]}":"{d[2]}" -> "{s[1]}":"{s[2]}" [label="{wire}"];')

    dot.append("}")
    return "\n".join(dot)


def emit_hierarchy(top: str, modules: dict[str, Module]) -> str:
    """Simple module instantiation tree rooted at `top`."""
    out = [
        "digraph saturn_hier {",
        '  rankdir=LR;',
        '  graph [fontname="Helvetica", fontsize=11];',
        '  node  [shape=box, style=filled, fillcolor=lightyellow, fontname="Helvetica"];',
        '  edge  [fontname="Helvetica", fontsize=9];',
        f'  labelloc="t"; label=<<B>saturn_cpu module hierarchy</B>>;',
    ]
    seen = set()

    def walk(name: str):
        if name in seen or name not in modules:
            return
        seen.add(name)
        m = modules[name]
        out.append(f'  "{name}" [label="{name}\\n{m.file}"];')
        for inst in m.instances:
            out.append(f'  "{name}" -> "{inst.module}" [label="{inst.name}"];')
            walk(inst.module)

    walk(top)
    out.append("}")
    return "\n".join(out)


# ── main ───────────────────────────────────────────────────────────────

def render(dot_path: Path, fmt: str) -> Path | None:
    if not shutil.which("dot"):
        return None
    out = dot_path.with_suffix("." + fmt)
    subprocess.run(["dot", f"-T{fmt}", str(dot_path), "-o", str(out)], check=True)
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--top", default="saturn_cpu")
    ap.add_argument("--rtl", default=str(Path(__file__).resolve().parent))
    ap.add_argument("--out", default=None, help="output dir (default rtl/diagram)")
    ap.add_argument("--no-render", action="store_true")
    args = ap.parse_args()

    rtl_dir = Path(args.rtl)
    out_dir = Path(args.out) if args.out else rtl_dir / "diagram"
    out_dir.mkdir(parents=True, exist_ok=True)

    files   = sorted(rtl_dir.glob("*.v"))
    if not files:
        sys.exit(f"no .v files under {rtl_dir}")
    modules = parse_files(files)

    print(f"[parse] {len(modules)} modules from {len(files)} files:")
    for name in sorted(modules):
        m = modules[name]
        print(f"  {name:<16s} {len(m.ports):>3d} ports, {len(m.instances):>2d} instances   ({m.file})")

    # block diagram
    bd_dot = out_dir / f"{args.top}_blocks.dot"
    bd_dot.write_text(emit_block_diagram(args.top, modules))
    print(f"[write] {bd_dot}")

    # hierarchy tree
    hier_dot = out_dir / f"{args.top}_hier.dot"
    hier_dot.write_text(emit_hierarchy(args.top, modules))
    print(f"[write] {hier_dot}")

    if args.no_render:
        return

    for d in (bd_dot, hier_dot):
        for fmt in ("svg", "png"):
            try:
                out = render(d, fmt)
                if out:
                    print(f"[dot ] {out}")
            except subprocess.CalledProcessError as e:
                print(f"[warn] dot failed on {d.name}: {e}", file=sys.stderr)
    if not shutil.which("dot"):
        print("note: install graphviz (apt install graphviz) to render to SVG/PNG")


if __name__ == "__main__":
    main()
