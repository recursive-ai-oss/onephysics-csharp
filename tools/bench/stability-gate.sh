#!/usr/bin/env bash
# The stability gate: every stability check, one table.
#   tools/bench/stability-gate.sh [--out DIR] [--budget SECONDS] [--threads 1,8]
# Runs `bar run --mode stability` on every generated scene (ours only) and every `*-stability` command of bench/Bar, then
# writes DIR/stability.tsv (source, scene, dim, threads, metric, threshold, ours, verdict) and DIR/meta.txt. The item list comes
# from the code (`bar gate-plan`), so a new scene or command appears without editing this file.
# Resumable: finished items (DIR/items/<id>.rc) are skipped. With --budget, no new item starts after that many seconds and the
# script exits 75; run it again with the same --out to continue. Exit 0 = all PASS, 1 = a FAIL, 2 = usage/build error.
# Not a timing run: runs niced, never while the shared benchmark lock exists. Environment: ONEPHYSICS_STATE (default ~/onephysics).
set -uo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "$here/../.." && pwd)"
state="${ONEPHYSICS_STATE:-$HOME/onephysics}"
lock="${BENCH_LOCK:-$HOME/WORKER-BENCH.lock}"
out="$state/stability-gate"; budget=0; threads=1
while [ $# -gt 0 ]; do
  case "$1" in
    --out) out="$2"; shift 2 ;;
    --budget) budget="$2"; shift 2 ;;
    --threads) threads="$2"; shift 2 ;;
    *) echo "usage: stability-gate.sh [--out DIR] [--budget SECONDS] [--threads 1,8]" >&2; exit 2 ;;
  esac
done
[ -x "$state/dotnet/dotnet" ] && export PATH="$state/dotnet:$PATH" DOTNET_ROOT="$state/dotnet"
[ -d "$HOME/dotnet10" ] && export PATH="$HOME/dotnet10:$PATH" DOTNET_ROOT="$HOME/dotnet10"
export DOTNET_CLI_TELEMETRY_OPTOUT=1 DOTNET_NOLOGO=1
export ONEPHYSICS_ARTIFACTS="${ONEPHYSICS_ARTIFACTS:-$state/artifacts}"
mkdir -p "$out/items"
if [ -e "$lock" ]; then echo "bench lock exists; not running" >&2; exit 3; fi

dotnet build "$repo/bench/Bar/Bar.csproj" -c Release -v quiet -clp:NoSummary >"$out/build.log" 2>&1 || { tail -20 "$out/build.log"; exit 2; }
bar="$ONEPHYSICS_ARTIFACTS/bin/Bar/release/Bar.dll"
scenes="$out/scenes"
if [ ! -f "$scenes/SHA256SUMS" ] || ! cmp -s "$scenes/SHA256SUMS" "$repo/scenes/SHA256SUMS"; then
  dotnet "$bar" gen --out "$scenes" >"$out/gen.log" 2>&1
  cmp -s "$scenes/SHA256SUMS" "$repo/scenes/SHA256SUMS" || { echo "generated scenes differ from scenes/SHA256SUMS" >&2; exit 2; }
fi
dotnet "$bar" gate-plan --scene-dir "$scenes" --threads "$threads" >"$out/plan.tsv" || exit 2

load() { cut -d' ' -f1-3 /proc/loadavg 2>/dev/null || uptime | sed 's/.*load averages*: //'; }
if [ ! -f "$out/meta.txt" ]; then
  { echo "machine=$(hostname)"; echo "os=$(uname -sr)"; echo "cpu=$( (grep -m1 'model name' /proc/cpuinfo | cut -d: -f2) 2>/dev/null || sysctl -n machdep.cpu.brand_string 2>/dev/null)"
    echo "cores=$( (nproc) 2>/dev/null || sysctl -n hw.ncpu)"; echo "dotnet=$(dotnet --version)"; echo "job_id=${BAR_JOB_ID:-}"
    echo "commit=${GATE_COMMIT:-}"; echo "load_before=$(load)"; echo "started=$(date -u +%FT%TZ)"; } >"$out/meta.txt"
fi
start=$(date +%s); left=0
while IFS=$'\t' read -r id args; do
  [ -f "$out/items/$id.rc" ] && continue
  if [ "$budget" -gt 0 ] && [ $(( $(date +%s) - start )) -ge "$budget" ]; then left=$((left+1)); continue; fi
  if [ -e "$lock" ]; then echo "bench lock appeared; stopping" >&2; exit 3; fi
  item="$out/items/$id"; a="${args//\{ITEM\}/$item}"
  echo "$(date -u +%T) $id" >>"$out/progress.log"
  # shellcheck disable=SC2086
  nice -n 10 dotnet "$bar" $a >"$out/items/$id.log" 2>&1; echo $? >"$out/items/$id.rc"
  echo "load_during=$(load)" >>"$out/load.log"
done <"$out/plan.tsv"
if [ "$left" -gt 0 ]; then echo "$left items left; run again with the same --out" >&2; exit 75; fi
{ echo "finished=$(date -u +%FT%TZ)"; echo "load_after=$(load)"; } >>"$out/meta.txt"
dotnet "$bar" gate-table --in "$out" --scene-dir "$scenes" --out "$out/stability.tsv"
