#!/usr/bin/env bash
# Runs the hash-matrix scenarios in every column and compares the manifests.
#
#   tools/hash-matrix.sh [-o OUTDIR] [-p PREFIX] [column ...]
#   tools/hash-matrix.sh [-o OUTDIR] compare
#
# Columns (default: all of them except those that need a tool that is missing):
#   jit  threads8  threads-all  scalar  noavx512  noavx2  tc0  aot  wasm  wasm-aot
#     jit          .NET JIT, 1 thread
#     threads8     JIT, 8 threads
#     threads-all  JIT, one thread per logical processor (the count is read here, passed explicitly, and recorded)
#     scalar       DOTNET_EnableHWIntrinsic=0: every vector operation falls back to software
#     noavx512     DOTNET_EnableAVX512F=0
#     noavx2       DOTNET_EnableAVX2=0
#     tc0          DOTNET_TieredCompilation=0 (fully optimised code from the first call)
#     aot          NativeAOT publish, 1 thread
#     wasm         browser-wasm, interpreted, run under Node
#     wasm-aot     browser-wasm, ahead-of-time compiled, run under Node
# Each column writes OUTDIR/PREFIX<column>.txt (the runner's manifest) and .steps (per-step digests). Manifests from other
# machines (for example an arm64 Mac, run with -p arm64-) are dropped into OUTDIR and compared by `compare`.
# The first manifest in alphabetical order of the column list given (or "PREFIXjit" for `compare`) is the reference; the
# script exits non-zero if any manifest differs on a scenario both contain, and for every difference prints the first step
# at which the two disagree. A column may run a subset of the scenarios (HASHMATRIX_FILTER); only shared lines are compared.
# Within each column, scenarios that differ only in their last name component under batching/ (the vector widths) must also
# hold the same hash.
#
# Environment: HASHMATRIX_FILTER (runner --filter prefix, default none: every scenario), HASHMATRIX_SLOW_FILTER (filter for
# the scalar and wasm columns, which run much slower; default: the same as HASHMATRIX_FILTER), HASHMATRIX_THREADS_ALL (override the all-cores count), ONEPHYSICS_ARTIFACTS.
# Needs the .NET 10 SDK on PATH (DOTNET_ROOT set) and, for the wasm columns, the wasm-tools workload and Node.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ART="${ONEPHYSICS_ARTIFACTS:-$HOME/.cache/onephysics-artifacts}"
OUT="${TMPDIR:-/tmp}/hash-matrix"
PREFIX=""
while [[ "${1:-}" == "-o" || "${1:-}" == "-p" ]]; do
  if [[ "$1" == "-o" ]]; then OUT="$2"; else PREFIX="$2"; fi
  shift 2
done
mkdir -p "$OUT"
RID="${HASHMATRIX_RID:-$(dotnet --info | sed -n 's/^ *RID: *//p' | head -1)}"
FILTER="${HASHMATRIX_FILTER:-}"
SLOW_FILTER="${HASHMATRIX_SLOW_FILTER:-$FILTER}"
CORES="${HASHMATRIX_THREADS_ALL:-$( (nproc 2>/dev/null || sysctl -n hw.ncpu) )}"

ALL=(jit threads8 threads-all scalar noavx512 noavx2 tc0 aot wasm wasm-aot)
if [[ $# -eq 0 ]]; then set -- "${ALL[@]}"; fi

# Runs a dotnet build or publish quietly. When it fails, prints everything it said (errors included) to stderr and stops the script with a non-zero status.
quiet_dotnet() {
  local log
  log="$(mktemp "${TMPDIR:-/tmp}/hash-matrix-build.XXXXXX")"
  if ! dotnet "$@" > "$log" 2>&1; then
    echo "hash-matrix: 'dotnet $1' failed:" >&2
    cat "$log" >&2
    rm -f "$log"
    exit 1
  fi
  rm -f "$log"
}

build_jit() {
  [[ -n "${JIT_BUILT:-}" ]] && return 0
  quiet_dotnet build "$ROOT/tools/HashMatrix" -c Release -v quiet -clp:NoSummary
  JIT_BUILT=1
}
jit_dll() { echo "$ART/bin/HashMatrix/release/HashMatrix.dll"; }

# Splits the runner's stdout into the manifest (name + hash lines, then "manifest") and the per-step lines.
split_output() { # $1 = column, stdin = runner output
  local raw="$OUT/$PREFIX$1.raw"
  cat > "$raw"
  grep -E '^([a-z0-9/_.-]+ [0-9a-f]{64}|manifest [0-9a-f]{64})$' "$raw" > "$OUT/$PREFIX$1.txt"
  grep -E '^step ' "$raw" > "$OUT/$PREFIX$1.steps" || true
  rm -f "$raw"
}

publish_aot() {
  quiet_dotnet publish "$ROOT/tools/HashMatrix" -c Release -r "$RID" -p:PublishAot=true -o "$ART/hm-aot" -v quiet -clp:NoSummary
}

publish_wasm() { # $1 = out dir name, rest = extra msbuild args
  local dest="$1"; shift
  # Start from a clean tree so an interpreted build never reuses ahead-of-time output or the other way round.
  rm -rf "$ART/bin/HashMatrix/release_browser-wasm" "$ART/obj/HashMatrix/release_browser-wasm" "$ART/$dest-publish"
  quiet_dotnet publish "$ROOT/tools/HashMatrix" -c Release -r browser-wasm -o "$ART/$dest-publish" "$@" -v quiet -clp:NoSummary
  rm -rf "$ART/$dest"
  cp -r "$ART/bin/HashMatrix/release_browser-wasm/AppBundle" "$ART/$dest"
  cp "$ROOT/tests/WasmSmoke/main.mjs" "$ART/$dest/main.mjs"
}

run_jit() { # $1 = column, $2 = threads, $3 = filter, rest = env assignments
  local col="$1" threads="$2" filter="$3"; shift 3
  build_jit
  echo "   threads=$threads filter='${filter}' env: $*"
  env "$@" dotnet "$(jit_dll)" --threads "$threads" --steps --filter "$filter" | split_output "$col"
}

run_column() {
  local col="$1"
  case "$col" in
    jit)         run_jit "$col" 1 "$FILTER" ;;
    threads8)    run_jit "$col" 8 "$FILTER" ;;
    threads-all) run_jit "$col" "$CORES" "$FILTER" ;;
    scalar)      run_jit "$col" 1 "$SLOW_FILTER" DOTNET_EnableHWIntrinsic=0 ;;
    noavx512)    run_jit "$col" 1 "$FILTER" DOTNET_EnableAVX512F=0 ;;
    noavx2)      run_jit "$col" 1 "$FILTER" DOTNET_EnableAVX2=0 ;;
    tc0)         run_jit "$col" 1 "$FILTER" DOTNET_TieredCompilation=0 DOTNET_ReadyToRun=0 ;;
    aot)         publish_aot; "$ART/hm-aot/HashMatrix" --threads 1 --steps --filter "$FILTER" | split_output "$col" ;;
    wasm)        publish_wasm wasm;                              (cd "$ART/wasm" && node main.mjs --threads 1 --steps --filter "$SLOW_FILTER") | split_output "$col" ;;
    wasm-aot)    publish_wasm wasm-aot -p:RunAOTCompilation=true; (cd "$ART/wasm-aot" && node main.mjs --threads 1 --steps --filter "$SLOW_FILTER") | split_output "$col" ;;
    *) echo "unknown column: $col" >&2; return 2 ;;
  esac
}

# First step at which two per-step files disagree for a scenario.
first_diff() { # $1 = ref steps, $2 = column steps, $3 = scenario
  awk -v name="$3" '
    FNR == NR { if ($2 == name) ref[$3] = $4; next }
    $2 == name && ($3 in ref) && ref[$3] != $4 { print "      first differing step: " $3 " (of scenario " name ")"; exit }
  ' "$1" "$2"
}

compare() {
  local ref="" refname status=0 f name
  for f in "$OUT"/*.txt; do
    [[ -e "$f" ]] || { echo "no manifests in $OUT"; return 1; }
    [[ -z "$ref" ]] && ref="$f"
  done
  [[ -e "$OUT/jit.txt" ]] && ref="$OUT/jit.txt"
  [[ -e "$OUT/jit.txt" ]] || { [[ -e "$OUT/${PREFIX}jit.txt" ]] && ref="$OUT/${PREFIX}jit.txt"; }
  refname="$(basename "$ref" .txt)"
  for f in "$OUT"/*.txt; do
    name="$(basename "$f" .txt)"
    local total shared=0 bad=0 sname shash rhash
    total="$(grep -vc '^manifest ' "$f" || true)"
    while read -r sname shash; do
      rhash="$(awk -v n="$sname" '$1 == n { print $2 }' "$ref")"
      [[ -z "$rhash" ]] && continue
      shared=$((shared + 1))
      if [[ "$rhash" != "$shash" ]]; then
        bad=$((bad + 1)); status=1
        echo "DIFFERS $name vs $refname: $sname"
        if [[ -e "${ref%.txt}.steps" && -e "${f%.txt}.steps" ]]; then first_diff "${ref%.txt}.steps" "${f%.txt}.steps" "$sname"; fi
      fi
    done < <(grep -v '^manifest ' "$f")
    if [[ $bad -eq 0 ]]; then
      echo "equal   $name  ($shared of $total scenarios shared with $refname) manifest $(grep '^manifest ' "$f" | cut -d' ' -f2 | cut -c1-16)"
    fi

    # Vector widths of one batching family must agree within this very manifest.
    local fam
    for fam in $(grep -v '^manifest ' "$f" | awk '{n=$1; sub(/\/[^\/]*$/,"",n); if (n ~ /^batching\//) print n}' | sort -u); do
      if [[ "$(awk -v p="$fam/" 'index($1, p) == 1 { print $2 }' "$f" | sort -u | wc -l)" -ne 1 ]]; then
        echo "DIFFERS $name: the widths of $fam disagree within the column"; status=1
      fi
    done
  done
  return $status
}

if [[ "${1:-}" == "compare" ]]; then compare; exit $?; fi
for col in "$@"; do
  echo "== $PREFIX$col"
  run_column "$col"
done
compare
