#!/usr/bin/env bash
# gen_docs.sh — Standalone Doxygen build for rv2_control_signal_transport
#
# Usage:
#   ./gen_docs.sh          # generate docs into .docs/
#   ./gen_docs.sh clean    # remove .docs/
#
# Output:
#   .docs/html/            — standalone HTML (open .docs/html/index.html)
#   .docs/xml/             — XML consumed by Breathe/Sphinx (via build_docs.sh)
#
# Note: .docs/ is gitignored. For the full multi-package Sphinx build,
#       use rv2_project/build_docs.sh instead.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="${SCRIPT_DIR}/.docs"

case "${1:-}" in
    clean)
        echo "[doxygen] Removing ${OUT_DIR} ..."
        rm -rf "${OUT_DIR}"
        echo "[doxygen] Clean done."
        ;;
    "")
        echo "[doxygen] Generating docs → ${OUT_DIR}"
        mkdir -p "${OUT_DIR}"
        (
            cat "${SCRIPT_DIR}/Doxyfile"
            echo "OUTPUT_DIRECTORY = ${OUT_DIR}"
            echo "WARN_LOGFILE     = ${OUT_DIR}/doxygen_warnings.log"
        ) | doxygen -
        echo "[doxygen] Done. Open: ${OUT_DIR}/html/index.html"
        ;;
    *)
        echo "Usage: $0 [clean]"
        exit 1
        ;;
esac
