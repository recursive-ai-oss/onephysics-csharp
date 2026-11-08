#!/usr/bin/env bash
# Publish a formats-capable 2D-only app and confirm the spatial engine is not retained.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
art="${ONEPHYSICS_ARTIFACTS:-$HOME/.cache/onephysics-artifacts}"
rid="${SMOKE_RID:-$(dotnet --info | sed -n 's/^ *RID: *//p' | head -1)}"
out="$art/planar-trim-smoke"
dotnet publish tests/PlanarTrimSmoke -c Release -r "$rid" --self-contained true -o "$out" -v quiet -clp:NoSummary
dotnet "$out/PlanarTrimSmoke.dll"
if grep -aEq 'Engine3D|PhysicsWorld3D' "$out/RecursiveAI.OnePhysics.dll"; then
  echo '2D-only trimmed app retained spatial engine types' >&2
  exit 1
fi
echo '2D-only trimmed app contains no spatial engine type names'
