#!/usr/bin/env bash
# Coverage-guided fuzzing of the fuzz targets in fuzz/Fuzz (see docs/spec/fuzzing.md).
#
#   tools/fuzz.sh build                      build the Checked configuration and instrument the library copy with SharpFuzz
#   tools/fuzz.sh seeds                      copy the committed seed corpora (testdata/fuzz-seeds) into the corpus directory, keeping what is there
#   tools/fuzz.sh run <targets> <seconds> [jobs]
#                                            fuzz each selected target for <seconds> (jobs at a time, niced), continuing from its saved corpus
#   tools/fuzz.sh chunk <targets> <total-seconds> [jobs]
#                                            like run, but sizes the per-target budget so the whole chunk takes about <total-seconds>
#   tools/fuzz.sh minimize <target> <input>  shrink a crashing input and copy it to testdata/fuzz/<target>/ as a regression test
#   tools/fuzz.sh report                     one row per target (executions, coverage, wall time, chunks, crashes on disk) and a total; exits 1 when any crash exists
#   tools/fuzz.sh list                       the target names
#
# <targets> is `all`, a comma-separated list of target names, or `shard:K/N` (the K-th of N interleaved parts of the sorted list, K from 1),
# so one campaign can be cut into chunks and spread over machines without overlap. Names are checked against `list`.
# Environment: FUZZ_HOME (state directory), FUZZ_SEED (fixed seed for every target; default the clock, so each chunk explores differently).
#
# State lives outside the checkout, under $FUZZ_HOME (default ~/onephysics/fuzz): instr/ (instrumented build), corpus/<target>/,
# crashes/<target>/, logs/ and stats.tsv (one row per target per chunk). Needs the .NET 10 SDK on PATH; the SharpFuzz tool is installed
# into $FUZZ_HOME/tools on first use. Runs on Linux and macOS (bash 3.2 compatible).
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
home="${FUZZ_HOME:-$HOME/onephysics/fuzz}"
instr="$home/instr"
tool="$home/tools/sharpfuzz"
cmd="${1:-}"; shift || true

targets() { dotnet "$instr/Fuzz.dll" 2>&1 | sed -n 's/.*targets: //p' | tr -d ' ' | tr ',' '\n' | grep -v '^$' | sort; }

# Expands a selection into target names, one per line.
select_targets() {
  local sel="$1" all
  all="$(targets)"
  [ -n "$all" ] || { echo "no targets found; run 'tools/fuzz.sh build' first" >&2; exit 2; }
  case "$sel" in
    all) printf '%s\n' "$all" ;;
    shard:*/*)
      local k="${sel#shard:}"; local n="${k#*/}"; k="${k%/*}"
      case "$k$n" in *[!0-9]*|'') echo "bad shard '$sel' (want shard:K/N)" >&2; exit 2 ;; esac
      if [ "$n" -lt 1 ] || [ "$k" -lt 1 ] || [ "$k" -gt "$n" ]; then echo "bad shard '$sel' (want 1 <= K <= N)" >&2; exit 2; fi
      printf '%s\n' "$all" | awk -v k="$k" -v n="$n" '(NR - 1) % n == k - 1'
      ;;
    *)
      local t
      for t in $(printf '%s' "$sel" | tr ',' ' '); do
        printf '%s\n' "$all" | grep -qx -- "$t" || { echo "unknown target '$t' (tools/fuzz.sh list)" >&2; exit 2; }
        printf '%s\n' "$t"
      done
      ;;
  esac
}

# Largest file in a directory, portable (no find -printf: BSD find lacks it).
largest_file() {
  find "$1" -type f -exec wc -c {} + 2>/dev/null | awk '$2 != "total" { if ($1 > m) m = $1 } END { print m + 0 }'
}

run_targets() {
  local sel="$1" secs="$2" jobs="$3" list
  [ -d "$instr" ] || { echo "run 'tools/fuzz.sh build' first"; exit 2; }
  case "$secs$jobs" in *[!0-9]*|'') echo "seconds and jobs must be whole numbers" >&2; exit 2 ;; esac
  [ "$secs" -ge 1 ] && [ "$jobs" -ge 1 ] || { echo "seconds and jobs must be at least 1" >&2; exit 2; }
  list="$(select_targets "$sel")"
  [ -n "$list" ] || { echo "no targets selected by '$sel'" >&2; exit 2; }
  mkdir -p "$home/corpus" "$home/crashes" "$home/logs"
  one() {
    local t="$1" big maxlen code=0 stamp
    mkdir -p "$home/corpus/$t" "$home/crashes/$t"
    # The length cap leaves room to grow the largest seed; the heap limit turns runaway allocation into a finding instead of a stalled host.
    big="$(largest_file "$home/corpus/$t")"
    maxlen=$(( big * 2 > 4096 ? big * 2 : 4096 ))
    DOTNET_GCHeapHardLimit=0x80000000 DOTNET_TieredPGO=0 DOTNET_gcServer=0 \
      nice -n 10 dotnet "$instr/Fuzz.dll" "$t" --guided "$secs" --corpus "$home/corpus/$t" --crashes "$home/crashes/$t" \
        --seed "${FUZZ_SEED:-$(date +%s)}" --maxlen "$maxlen" --stats "$home/stats.tsv" > "$home/logs/$t.log" 2>&1 || code=$?
    # Exit 0 is clean, 1 is a finding (saved by the driver), 77 a hang (saved). Anything else killed the process (stack overflow, out of memory):
    # the input in flight is kept so the crash is counted and reproducible.
    if [ "$code" -ne 0 ] && [ "$code" -ne 1 ] && [ "$code" -ne 77 ]; then
      stamp="$(date +%s)"
      [ -s "$home/crashes/$t/.current" ] && cp "$home/crashes/$t/.current" "$home/crashes/$t/killed-$stamp"
      echo "$t: process died with exit $code, input kept as crashes/$t/killed-$stamp (see logs/$t.log)"
    fi
    grep -aE '^(STATS [a-z]|CRASH|HANG)' "$home/logs/$t.log" | grep -av 'STATS target' | tail -n 4 || true
  }
  export -f one largest_file; export home instr secs
  printf '%s\n' "$list" | xargs -P "$jobs" -I{} bash -c 'one {}'
}

case "$cmd" in
  build)
    mkdir -p "$home"
    if [ ! -x "$tool" ]; then dotnet tool install SharpFuzz.CommandLine --version 2.3.0 --tool-path "$home/tools" >/dev/null; fi
    rm -rf "$instr"
    dotnet publish fuzz/Fuzz/Fuzz.csproj -c Checked -o "$instr" -v quiet -clp:NoSummary
    # Only the library under test is instrumented; the fuzz driver and onemath are not.
    "$tool" "$instr/RecursiveAI.OnePhysics.dll" >/dev/null
    "$tool" "$instr/RecursiveAI.OnePhysics.Tools.dll" >/dev/null
    "$tool" "$instr/RecursiveAI.OnePhysics.Interchange.dll" >/dev/null
    echo "instrumented build in $instr"
    ;;
  seeds)
    # testdata/fuzz-seeds is written by `dotnet run --project fuzz/Fuzz -- --seeds testdata/fuzz-seeds`; an existing corpus file is never overwritten.
    for d in testdata/fuzz-seeds/*/; do
      t="$(basename "$d")"; mkdir -p "$home/corpus/$t"
      for f in "$d"*; do [ -e "$home/corpus/$t/$(basename "$f")" ] || cp "$f" "$home/corpus/$t/"; done
    done
    ;;
  list)
    targets
    ;;
  run)
    run_targets "${1:?targets: all, a comma-separated list or shard:K/N}" "${2:?seconds per target}" "${3:-1}"
    ;;
  chunk)
    sel="${1:?targets: all, a comma-separated list or shard:K/N}"; total="${2:?total seconds}"; jobs="${3:-1}"
    case "$total$jobs" in *[!0-9]*|'') echo "total seconds and jobs must be whole numbers" >&2; exit 2 ;; esac
    count="$(select_targets "$sel" | wc -l | tr -d ' ')"
    [ "$count" -ge 1 ] || { echo "no targets selected by '$sel'" >&2; exit 2; }
    # Targets run `jobs` at a time, so a wave of `jobs` targets gets total / waves seconds each (a few seconds go to process start and the final statistics).
    waves=$(( (count + jobs - 1) / jobs ))
    per=$(( total / waves - 5 )); [ "$per" -ge 5 ] || per=5
    echo "chunk: $count targets, $jobs at a time, $per s each, about $(( waves * (per + 5) )) s"
    run_targets "$sel" "$per" "$jobs"
    ;;
  minimize)
    t="${1:?target}"; in="${2:?crash file}"
    out="testdata/fuzz/$t/$(basename "$in").bin"
    mkdir -p "testdata/fuzz/$t"
    dotnet "$instr/Fuzz.dll" "$t" --minimize "$in" "$out"
    echo "wrote $out; fix the finding, then run the test suite (FuzzRegressionTests replays it)"
    ;;
  report)
    # Crashes are counted from the files on disk (one per distinct finding, however many chunks re-found it), not from the per-chunk rows, which
    # restart at zero in every chunk. Rows of stats.tsv that are not "<known target><TAB>six numbers" are ignored and counted, never summed.
    [ -f "$home/stats.tsv" ] || : > "$home/stats.tsv"
    known="$( { targets 2>/dev/null || true; } | tr '\n' ' ')"
    if [ -z "$known" ]; then known="$(awk -F'\t' 'NF == 7 && $2 ~ /^[0-9]+$/ { print $1 }' "$home/stats.tsv" | sort -u | tr '\n' ' ')"; fi
    crashfile="$(mktemp "${TMPDIR:-/tmp}/fuzz-report.XXXXXX")"
    trap 'rm -f "$crashfile"' EXIT
    for t in $known; do
      n=0
      if [ -d "$home/crashes/$t" ]; then
        n="$(find "$home/crashes/$t" -type f \( -name 'crash-*' -o -name 'hang-*' -o -name 'killed-*' \) ! -name '*.txt' | wc -l | tr -d ' ')"
      fi
      printf '%s\t%s\n' "$t" "$n" >> "$crashfile"
    done
    awk -F'\t' -v known="$known" '
      BEGIN { n = split(known, k, " "); for (i = 1; i <= n; i++) { isknown[k[i]] = 1; order[i] = k[i] } }
      FILENAME == ARGV[1] { crashes[$1] = $2; next }
      NF != 7 || !($1 in isknown) || $2 !~ /^[0-9]+$/ || $3 !~ /^[0-9]+$/ || $4 !~ /^[0-9]+$/ || $7 !~ /^[0-9]+$/ { bad++; next }
      { e[$1] += $2; chunks[$1]++; s[$1] += $7; if ($3 + 0 > ed[$1]) ed[$1] = $3 + 0; if ($4 + 0 > f[$1]) f[$1] = $4 + 0 }
      END {
        printf "target\texecs\tedges\tfeatures\tseconds\tchunks\tcrashes\n"
        for (i = 1; i <= n; i++) {
          t = order[i]; c = crashes[t] + 0
          printf "%s\t%d\t%d\t%d\t%d\t%d\t%d\n", t, e[t], ed[t], f[t], s[t], chunks[t], c
          te += e[t]; ts += s[t]; tc += c; if (chunks[t] > 0) run++; if (c > 0) bad_t++
        }
        printf "TOTAL\t%d\t\t\t%d\t\t%d\n", te, ts, tc
        printf "targets %d, run at least once %d, with crashes %d, crashes in total %d\n", n, run, bad_t, tc
        if (bad > 0) printf "ignored %d malformed row(s) in stats.tsv\n", bad
        if (tc > 0) exit 1
      }' "$crashfile" "$home/stats.tsv"
    ;;
  *)
    sed -n '2,18p' "$0"; exit 2 ;;
esac
