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

# Scenes shared with the hash matrix, and the bench scene generators, must not call the platform's transcendentals:
# their results differ between runtimes (browser-wasm against x64) and would make the same scene differ. Use Math64/Math32.
# Exempt by path: bench metrics, reference drivers and tests that compare against Math as a black-box reference.
scene_files="tests/Common tools/HashMatrix bench/Bar/Gen.cs bench/Bar/Gen3.cs bench/Bar/GenMachines.cs bench/Bar/SoftGen.cs bench/Bar/SceneBuilder2D.cs bench/Bar/SceneBuilder3D.cs"
# shellcheck disable=SC2086
hits="$(search '\b(Math|MathF)\.(Sin|Cos|Tan|Asin|Acos|Atan2?|SinCos|Sinh|Cosh|Tanh|Exp|Log2?|Log10|Pow|Cbrt|Hypot)\b' $scene_files || true)"
if [ -n "$hits" ]; then echo "scan failed (platform math in a shared scene; use Math64/Math32):"; echo "$hits"; fail=1; fi

# The README is the package page on nuget.org, where relative links break.
if grep -nE '\]\((\.|/|[A-Za-z0-9_-]+/|[A-Za-z0-9_-]+\.[a-z]+)' README.md | grep -vE '\]\((https?:|mailto:|#)'; then
  echo "scan failed: README.md has a relative link; use absolute https://github.com/recursive-ai-oss/onephysics-csharp/... URLs"; fail=1
fi

# The package has exactly one dependency: onemath.
# (the OneScope module, src/OnePhysics.Scope, is built outside CI against the OneScope SDK and has its own reference.)
refs="$(search '<PackageReference' src | grep '\.csproj:' | grep -v '^src/OnePhysics\.Scope/' || true)"
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
dotnet run -c "$config" --no-build --project fuzz/Fuzz -- tools-polygon --random 1 2000
dotnet run -c "$config" --no-build --project fuzz/Fuzz -- tools-sprite --random 1 2000
dotnet run -c "$config" --no-build --project fuzz/Fuzz -- tools-tetrahedralize --random 1 1500
dotnet run -c "$config" --no-build --project fuzz/Fuzz -- tools-triangulate2d --random 1 2000
dotnet run -c "$config" --no-build --project fuzz/Fuzz -- tools-autoragdoll --random 1 3000
for t in interchange-urdf interchange-mjcf interchange-gltf interchange-tiled interchange-ldtk; do dotnet run -c "$config" --no-build --project fuzz/Fuzz -- "$t" --random 1 3000; done
dotnet run -c "$config" --no-build --project bench/Bar -- selftest
gen="$(mktemp -d)"
dotnet run -c "$config" --no-build --project bench/Bar -- gen --out "$gen"
cmp "$gen/SHA256SUMS" scenes/SHA256SUMS || { echo "generated scenes differ from scenes/SHA256SUMS"; exit 1; }
rm -rf "$gen"
dotnet run -c "$config" --no-build --project experiments/batching -- check --small --steps 60
echo "check passed ($config)"
