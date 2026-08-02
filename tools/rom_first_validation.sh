#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <serial-port> [track] [sector] [side] [density]"
  echo "Example: $0 /dev/ttyACM0 0 1 0 0"
  exit 1
fi

PORT="$1"
TRACK="${2:-0}"
SECTOR="${3:-1}"
SIDE="${4:-0}"
DENSITY="${5:-0}"

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROBE="$ROOT_DIR/tools/mzp_probe.py"

if [[ ! -f "$PROBE" ]]; then
  echo "Probe tool not found at $PROBE"
  exit 1
fi

echo "=== ROM-first validation: preflight build ==="
cmake --build "$ROOT_DIR/build" -j

echo
echo "=== Bench phase A: FDC status snapshot ==="
python3 "$PROBE" "$PORT" fdcdump

echo
echo "=== Bench phase B: command history snapshot ==="
python3 "$PROBE" "$PORT" fdclog

echo
echo "=== Bench phase C: direct FDC sector read ==="
python3 "$PROBE" "$PORT" fdcread "$TRACK" "$SECTOR" "$SIDE" "$DENSITY"

echo
echo "=== Bench phase D: streaming scan sanity ==="
python3 "$PROBE" "$PORT" streamscan "$TRACK" "$SIDE" "$DENSITY" 220

echo
echo "=== Bench phase E: final FDC status snapshot ==="
python3 "$PROBE" "$PORT" fdcdump

echo
echo "Bench probe sequence completed."
echo "Next: run live MZ-800 ROM test and evaluate with docs/rom-first-validation.md."
