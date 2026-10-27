#!/usr/bin/env bash
# Times the batching experiment's solver at every SIMD width and several thread counts. This is a timing run: it takes the
# shared benchmark lock itself (waiting if another job holds it), records the machine and the load before, during and after,
# and releases the lock on exit. Run it on the benchmark host with nothing else of yours going.
#
#   experiments/batching/bench.sh [OUTDIR]        (default ~/onephysics/batching-bench)
#
# Environment: BENCH_THREADS (comma list, default 1,2,4,8,16,<logical processors>), BENCH_REPEATS (default 5),
# BENCH_STEPS (default 200), BENCH_WARMUP (default 150), BENCH_LOCK_WAIT_MIN (minutes to wait for the lock, default 30),
# BENCH_TIMING=0 to skip the lock (a functional check only; the numbers are then not publishable).
# After the timing it runs the one-thread and all-cores hash columns of the batching/pile scenarios and compares them.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUTDIR="${1:-$HOME/onephysics/batching-bench}"
LOCK="$HOME/WORKER-BENCH.lock"
mkdir -p "$OUTDIR"
export DOTNET_ROOT="${DOTNET_ROOT:-$HOME/dotnet10}" PATH="${DOTNET_ROOT}:$PATH" DOTNET_CLI_TELEMETRY_OPTOUT=1 DOTNET_NOLOGO=1
export ONEPHYSICS_ARTIFACTS="${ONEPHYSICS_ARTIFACTS:-$HOME/onephysics/artifacts}"

CORES="$( (nproc 2>/dev/null || sysctl -n hw.ncpu) )"
THREADS="${BENCH_THREADS:-1,2,4,8,16,$CORES}"
REPEATS="${BENCH_REPEATS:-5}"
STEPS="${BENCH_STEPS:-200}"
WARMUP="${BENCH_WARMUP:-150}"

# Build first, so compile time is outside the lock.
dotnet build "$ROOT/experiments/batching" -c Release -v quiet -clp:NoSummary >/dev/null
DLL="$ONEPHYSICS_ARTIFACTS/bin/Batching/release/BatchingExperiment.dll"

if [[ "${BENCH_TIMING:-1}" != "0" ]]; then
  waited=0
  until ( set -o noclobber; echo "onephysics batching-bench started $(date -u +%FT%TZ) expected end +30min" > "$LOCK" ) 2>/dev/null; do
    if [[ $waited -ge ${BENCH_LOCK_WAIT_MIN:-30} ]]; then echo "lock held by: $(cat "$LOCK" 2>/dev/null)"; exit 3; fi
    sleep 60; waited=$((waited + 1))
  done
  trap 'rm -f "$LOCK"; [[ -n "${SAMPLER:-}" ]] && kill "$SAMPLER" 2>/dev/null || true' EXIT
fi

{
  echo "date: $(date -u +%FT%TZ)"
  echo "host: $(uname -srm)"
  lscpu | grep -E 'Model name|^CPU\(s\)|Thread|Core|Socket|MHz|Flags' | sed -E 's/(Flags:).*/\1 (avx512 present: '"$(grep -c avx512f /proc/cpuinfo || true)"' logical cpus)/'
  echo "sdk: $(dotnet --version)"
  echo "avx512 flags: $(grep -m1 -o 'avx512[a-z_0-9]*' /proc/cpuinfo | sort -u | tr '\n' ' ')"
  echo "load before: $(cat /proc/loadavg)"
} | tee "$OUTDIR/machine.txt"

( while true; do echo "$(date -u +%T) $(cat /proc/loadavg)" >> "$OUTDIR/load-during.txt"; sleep 10; done ) &
SAMPLER=$!
: > "$OUTDIR/load-during.txt"

# Tiered compilation off: every method is fully optimised from its first call, so warm-up length does not matter.
DOTNET_TieredCompilation=0 dotnet "$DLL" bench --colors 16 --threads "$THREADS" --repeats "$REPEATS" --steps "$STEPS" --warmup "$WARMUP" | tee "$OUTDIR/bench.csv"

# While the lock is held, also run the all-cores hash column (it uses more than 8 cores) next to the one-thread reference.
HASHMATRIX_FILTER=batching/pile "$ROOT/tools/hash-matrix.sh" -o "$OUTDIR/hm" jit threads-all 2>&1 | tee "$OUTDIR/hash-matrix.txt"

kill "$SAMPLER" 2>/dev/null || true
SAMPLER=""
echo "load after: $(cat /proc/loadavg)" | tee -a "$OUTDIR/machine.txt"
echo "load during (every 10 s): $(awk '{print $2}' "$OUTDIR/load-during.txt" | tr '\n' ' ')" | tee -a "$OUTDIR/machine.txt"
