#!/usr/bin/env bash
# One command for scene x engine x thread-count runs.
#   tools/bench/run.sh stability [bar run options]   stability metrics (nice, <= 8 cores, no lock needed; refuses while the bench lock exists)
#   tools/bench/run.sh timing    [bar run options]   step-time rows at 1, 8 and all threads (takes the shared bench lock itself)
# Environment: ONEPHYSICS_STATE (default ~/onephysics) holds reference builds, results and the lock-free scratch;
#              BENCH_LOCK (default ~/WORKER-BENCH.lock); BAR_JOB_ID is recorded in the results.
# Options after the mode go to `bar run` (e.g. --scenes pyramid-20,tumbler --engines box2d --threads 1,8 --repeat 3 --out DIR).
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "$here/../.." && pwd)"
mode="${1:?usage: run.sh stability|timing [bar run options]}"; shift
state="${ONEPHYSICS_STATE:-$HOME/onephysics}"
lock="${BENCH_LOCK:-$HOME/WORKER-BENCH.lock}"
[ -x "$state/dotnet/dotnet" ] && export PATH="$state/dotnet:$PATH" DOTNET_ROOT="$state/dotnet"
[ -d "$HOME/dotnet10" ] && export PATH="$HOME/dotnet10:$PATH" DOTNET_ROOT="$HOME/dotnet10"
export DOTNET_CLI_TELEMETRY_OPTOUT=1 DOTNET_NOLOGO=1
[ -d "$HOME/.cargo/bin" ] && export PATH="$HOME/.cargo/bin:$PATH"
export ONEPHYSICS_ARTIFACTS="${ONEPHYSICS_ARTIFACTS:-$state/artifacts}"

mkdir -p "$state"
"$repo/tools/refs/setup.sh" "$state" all >"$state/setup.log" 2>&1 || { tail -30 "$state/setup.log"; exit 1; } # incremental: only changed drivers rebuild
dotnet build "$repo/bench/Bar/Bar.csproj" -c Release -v quiet -clp:NoSummary
bar="$ONEPHYSICS_ARTIFACTS/bin/Bar/release/Bar.dll"

scenes="$state/scenes"
if [ ! -f "$scenes/SHA256SUMS" ] || ! cmp -s "$scenes/SHA256SUMS" "$repo/scenes/SHA256SUMS"; then
  dotnet "$bar" gen --out "$scenes"
  cmp -s "$scenes/SHA256SUMS" "$repo/scenes/SHA256SUMS" || { echo "generated scenes differ from scenes/SHA256SUMS" >&2; exit 1; }
fi

case "$mode" in
  stability)
    if [ -e "$lock" ]; then echo "bench lock exists ($(cat "$lock")); not running" >&2; exit 3; fi
    exec nice -n 10 dotnet "$bar" run --mode stability --scene-dir "$scenes" --state "$state" "$@" ;;
  timing)
    set -o noclobber
    echo "onephysics timing sweep $(date -Is) expected 1h" > "$lock" || { echo "bench lock is held: $(cat "$lock")" >&2; exit 3; }
    trap 'rm -f "$lock"' EXIT
    set +o noclobber
    dotnet "$bar" run --mode timing --lock-held --scene-dir "$scenes" --state "$state" "$@" ;;
  *) echo "unknown mode $mode" >&2; exit 2 ;;
esac
