#!/usr/bin/env bash
# Measures what double-precision body positions cost against a float32 store on the particle scene: memory of the position column and
# the time of the position-touching loops next to a whole step. A timing run: run it on an otherwise idle machine.
# Usage: tools/position-cost.sh [bodies=5000] [steps=200] [repeats=7]
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
dotnet run -c Release --project bench/Micro -- --position-cost "${1:-5000}" "${2:-200}" "${3:-7}"
