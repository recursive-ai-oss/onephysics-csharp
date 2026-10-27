#!/usr/bin/env bash
# Build, test and scan the sources. Run from anywhere; needs dotnet on PATH.
# Usage: tools/check.sh [Configuration]   (default Release; use Checked to run with bounds-checked accessors)
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
config="${1:-Release}"

allow="$(mktemp)"
trap 'rm -f "$allow"' EXIT
grep -vE '^[[:space:]]*(#|$)' tools/check-allow.txt > "$allow" || true

fail=0

# git grep inside a repository; plain grep over the same tree where only a working-tree copy exists.
if git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  search() { git grep -nE "$1" -- "${@:2}"; }
  tracked() { git ls-files; }
else
  search() { grep -rnE --include='*.cs' --include='*.csproj' --include='*.props' "$1" "${@:2}" | grep -vE '/(bin|obj)/'; }
  tracked() { find . -type f -not -path './.localfeed/*' | sed 's#^\./##'; }
fi

scan() { # scan <description> <ERE>
  local hits
  hits="$(search "$2" src || true)"
  if [ -s "$allow" ] && [ -n "$hits" ]; then hits="$(printf '%s\n' "$hits" | grep -vEf "$allow" || true)"; fi
  if [ -n "$hits" ]; then echo "scan failed ($1):"; echo "$hits"; fail=1; fi
}

scan 'banned determinism APIs' 'FusedMultiplyAdd|MultiplyAddEstimate|ReciprocalEstimate|ReciprocalSqrtEstimate|Stopwatch|DateTime|TickCount|Reflection\.Emit|DynamicMethod'
scan 'Vector<T>' 'Vector<'
scan 'platform math' '\b(Math|MathF)\.(Sin|Cos|Tan|Asin|Acos|Atan2?|SinCos|Sinh|Cosh|Tanh|Exp|Log2?|Log10|Pow|Cbrt|Hypot)\b'
scan 'unsafe or hardware-dependent code' 'unsafe|Unsafe\.|MemoryMarshal|ProcessorCount|IsHardwareAccelerated|Vector512'

# The package has exactly one dependency: onemath.
refs="$(search '<PackageReference' src | grep '\.csproj:' || true)"
count="$(printf '%s\n' "$refs" | grep -c . || true)"
if [ "$count" -ne 1 ] || ! printf '%s' "$refs" | grep -q 'Include="RecursiveAI.OneMath"'; then
  echo "scan failed: src must have exactly one PackageReference, RecursiveAI.OneMath; found:"; echo "$refs"; fail=1
fi

# Nothing tracked may be build output or carry a private path.
if tracked | grep -E '(^|/)(bin|obj)/|\.nupkg$'; then echo "scan failed: build output is tracked"; fail=1; fi

if [ "$fail" -ne 0 ]; then exit 1; fi

dotnet build OnePhysics.slnx -c "$config" -v quiet -clp:NoSummary
dotnet test --solution OnePhysics.slnx -c "$config" --no-build
dotnet run -c "$config" --no-build --project fuzz/Fuzz -- snapshot --random 1 10000
echo "check passed ($config)"
