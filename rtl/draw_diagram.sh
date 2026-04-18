#!/bin/bash
# draw_diagram.sh — block diagrams of the Saturn CPU RTL, via yosys.
#
# Produces one graphviz DOT + SVG per module under rtl/diagram/. We run
# yosys with `hierarchy -check -top <mod>; show`, without going through
# `proc`: that keeps always-blocks as opaque process nodes so the output
# is a RTL-level block diagram (ports on the boundary, sub-module
# instances as boxes, named processes in the middle) instead of a fully
# elaborated gate schematic. Much faster and much more readable.
#
# saturn_exec.v is skipped by default: with 80+ ports and a giant
# combinational decoder it produces a multi-megabyte DOT that takes
# graphviz minutes to lay out. Pass --all to render it anyway.
#
# Usage:
#   rtl/draw_diagram.sh                 # default four modules
#   rtl/draw_diagram.sh --all           # including saturn_exec
#   rtl/draw_diagram.sh saturn_alu      # specific module(s)
#   YOSYS=/path/to/yosys  DOT=/path/to/dot  rtl/draw_diagram.sh
set -eu

RTL_DIR="$(cd "$(dirname "$0")" && pwd)"
OUT_DIR="$RTL_DIR/diagram"
YOSYS="${YOSYS:-/tools/yosys/v0.64/bin/yosys}"
DOT="${DOT:-dot}"

[ -x "$YOSYS" ] || { echo "yosys not found at $YOSYS (override with YOSYS=)" >&2; exit 1; }
command -v "$DOT" >/dev/null || { echo "graphviz dot not found (override with DOT=)" >&2; exit 1; }

MODULES_DEFAULT=(saturn_alu saturn_field saturn_cpu)
MODULES=()
RENDER_EXEC=0

for arg in "$@"; do
    case "$arg" in
        --all)  RENDER_EXEC=1 ;;
        --help|-h)
            sed -n '3,24p' "$0"; exit 0 ;;
        *)      MODULES+=("$arg") ;;
    esac
done
if [ "${#MODULES[@]}" -eq 0 ]; then
    MODULES=("${MODULES_DEFAULT[@]}")
    if [ "$RENDER_EXEC" = 1 ]; then
        MODULES+=("saturn_exec")
    fi
fi

mkdir -p "$OUT_DIR"
echo "[draw] yosys = $($YOSYS -V 2>&1 | head -1)"
echo "[draw] out   = $OUT_DIR"

for mod in "${MODULES[@]}"; do
    dot_file="$OUT_DIR/$mod.dot"
    svg_file="$OUT_DIR/$mod.svg"
    echo "[yosys] $mod …"
    # -nomem2reg keeps `reg [W:0] arr [0:N]` arrays as $mem blocks instead
    # of exploding into N individual DFFs, which drastically trims the
    # output for saturn_cpu (ib[32], RSTK[8], regR[5]).
    SRCS="$RTL_DIR/saturn_alu.v $RTL_DIR/saturn_field.v $RTL_DIR/saturn_exec.v $RTL_DIR/saturn_cpu.v"
    "$YOSYS" -q -p "
        read_verilog -nomem2reg -I $RTL_DIR $SRCS
        hierarchy -check -top $mod
        show -format dot -prefix $OUT_DIR/$mod -notitle $mod
    " 2>&1 | grep -vE "^(Warning: Replacing memory|Generating|Dumping)" || true

    if [ ! -s "$dot_file" ]; then
        echo "[warn ] yosys produced no DOT for $mod" >&2
        continue
    fi

    dot_bytes=$(stat -c %s "$dot_file")
    if [ "$dot_bytes" -gt 4000000 ]; then
        echo "[skip ] $mod.dot is ${dot_bytes} bytes — SVG would be huge,"\
             "render manually with: $DOT -Tsvg $dot_file -o $svg_file"
        continue
    fi
    echo "[dot  ] $mod.svg …"
    "$DOT" -Tsvg "$dot_file" -o "$svg_file"
    printf "  %-15s dot=%s  svg=%s\n" "$mod" \
        "$(numfmt --to=iec --suffix=B $dot_bytes)" \
        "$(numfmt --to=iec --suffix=B $(stat -c %s "$svg_file"))"
done

echo
echo "[draw] done — browse $OUT_DIR/"
