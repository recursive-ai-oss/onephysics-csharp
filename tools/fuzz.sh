#!/usr/bin/env bash
# Coverage-guided fuzzing of the fuzz targets in fuzz/Fuzz (see docs/fuzzing.md).
#
#   tools/fuzz.sh build                      build the Checked configuration and instrument the library copy with SharpFuzz
#   tools/fuzz.sh seeds                      copy the committed seed corpora (testdata/fuzz-seeds) into the corpus directory, keeping what is there
#   tools/fuzz.sh run <target|all> <seconds> [jobs]
#                                            fuzz one target, or every target (jobs at a time, niced), for <seconds> each
#   tools/fuzz.sh minimize <target> <input>  shrink a crashing input and copy it to testdata/fuzz/<target>/ as a regression test
#   tools/fuzz.sh report                     one row per target: executions, edges, features, wall time and crashes over all chunks
#   tools/fuzz.sh list                       the target names
#
# State lives outside the checkout, under $FUZZ_HOME (default ~/onephysics/fuzz): instr/ (instrumented build), corpus/<target>/,
# crashes/<target>/ and stats.tsv. Needs the .NET 10 SDK on PATH; the SharpFuzz tool is installed into $FUZZ_HOME/tools on first use.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
home="${FUZZ_HOME:-$HOME/onephysics/fuzz}"
instr="$home/instr"
tool="$home/tools/sharpfuzz"
cmd="${1:-}"; shift || true

targets() { dotnet "$instr/Fuzz.dll" 2>&1 | sed -n 's/.*targets: //p' | tr -d ' ' | tr ',' '\n'; }

case "$cmd" in
  build)
    mkdir -p "$home"
    if [ ! -x "$tool" ]; then dotnet tool install SharpFuzz.CommandLine --version 2.3.0 --tool-path "$home/tools" >/dev/null; fi
    rm -rf "$instr"
    dotnet publish fuzz/Fuzz/Fuzz.csproj -c Checked -o "$instr" -v quiet -clp:NoSummary
    # Only the library under test is instrumented; the fuzz driver and onemath are not.
    "$tool" "$instr/RecursiveAI.OnePhysics.dll" >/dev/null
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
    which="${1:?target or all}"; secs="${2:?seconds}"; jobs="${3:-1}"
    [ -d "$instr" ] || { echo "run 'tools/fuzz.sh build' first"; exit 2; }
    mkdir -p "$home/corpus" "$home/crashes" "$home/logs"
    one() {
      t="$1"; mkdir -p "$home/corpus/$t" "$home/crashes/$t"
      # The length cap leaves room to grow the largest seed; the heap limit turns runaway allocation into a finding instead of a stalled host.
      big=$(find "$home/corpus/$t" -type f -printf '%s\n' | sort -n | tail -1); big="${big:-0}"
      maxlen=$(( big * 2 > 4096 ? big * 2 : 4096 ))
      DOTNET_GCHeapHardLimit=0x80000000 DOTNET_TieredPGO=0 DOTNET_gcServer=0 \
        nice -n 10 dotnet "$instr/Fuzz.dll" "$t" --guided "$secs" --corpus "$home/corpus/$t" --crashes "$home/crashes/$t" \
          --seed "$(date +%s)" --maxlen "$maxlen" --stats "$home/stats.tsv" > "$home/logs/$t.log" 2>&1 || echo "$t: exit $? (see $home/logs/$t.log)"
      grep -aE '^(STATS [a-z]|CRASH|HANG)' "$home/logs/$t.log" | grep -av 'STATS target' | tail -n 4 || true
    }
    export -f one; export home instr secs
    if [ "$which" = all ]; then targets | xargs -P "$jobs" -I{} bash -c 'one {}'; else one "$which"; fi
    ;;
  minimize)
    t="${1:?target}"; in="${2:?crash file}"
    out="testdata/fuzz/$t/$(basename "$in").bin"
    mkdir -p "testdata/fuzz/$t"
    dotnet "$instr/Fuzz.dll" "$t" --minimize "$in" "$out"
    echo "wrote $out; fix the finding, then run the test suite (FuzzRegressionTests replays it)"
    ;;
  report)
    printf 'target\texecs\tedges\tfeatures\tseconds\tcrashes_found\n'
    sort -k1,1 "$home/stats.tsv" | awk -F'\t' '{e[$1]+=$2; s[$1]+=$7; c[$1]+=$6; if ($3>ed[$1]) ed[$1]=$3; if ($4>f[$1]) f[$1]=$4}
      END {for (t in e) printf "%s\t%d\t%d\t%d\t%d\t%d\n", t, e[t], ed[t], f[t], s[t], c[t]}' | sort
    ;;
  *)
    sed -n '2,12p' "$0"; exit 2 ;;
esac
