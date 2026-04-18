#!/bin/bash
# Build the HP Saturn CPU RTL documentation with doxygen.
#
# Run from the repo root:  ./docs/build.sh
# Output:                  docs/html/index.html
set -eu

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_ROOT"

# Prefer the system doxygen; fall back to the locally-extracted one
# under ~/.local/doxygen (dpkg-deb -x without sudo). If neither is
# present, print install instructions and exit.
if command -v doxygen >/dev/null 2>&1; then
    DOXYGEN=doxygen
    DOX_LD=""
elif [ -x "$HOME/.local/doxygen/usr/bin/doxygen" ]; then
    DOXYGEN="$HOME/.local/doxygen/usr/bin/doxygen"
    DOX_LD="$HOME/.local/doxygen/usr/lib/x86_64-linux-gnu"
else
    cat >&2 <<EOF
docs/build.sh: doxygen not found.

Install options:
    sudo apt install doxygen graphviz        # system-wide
or, without sudo:
    apt-get download doxygen libfmt9
    mkdir -p ~/.local/doxygen
    dpkg-deb -x doxygen_*.deb ~/.local/doxygen
    dpkg-deb -x libfmt9_*.deb  ~/.local/doxygen
    rm doxygen_*.deb libfmt9_*.deb

Then re-run this script.
EOF
    exit 1
fi

export LD_LIBRARY_PATH="$DOX_LD${DOX_LD:+:}${LD_LIBRARY_PATH-}"
VERSION=$("$DOXYGEN" -v 2>&1 | head -1)
echo "[docs] using $DOXYGEN ($VERSION)"
rm -rf docs/html

# The warnings from the C++ parser stumbling on Verilog syntax are
# harmless — the /** */ comment blocks doxygen cares about are extracted
# correctly. Suppress that class of warning so the output is readable.
"$DOXYGEN" docs/Doxyfile 2>&1 |
    grep -vE "(Found ';' while parsing|Searching|Preprocessing|Parsing|Generating|\
Found documentation for module [A-Za-z_]+ but it has no primary interface|\
documented symbol 'input' was not)" \
    || true

echo
echo "[docs] open docs/html/index.html"
