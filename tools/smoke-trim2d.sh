#!/usr/bin/env bash
# Publish a 2D-only inspection host, run it, and inspect our trimmed assembly's type metadata.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
art="${ONEPHYSICS_ARTIFACTS:-$HOME/.cache/onephysics-artifacts}"
rid="${1:-$(dotnet --info | sed -n 's/^ *RID: *//p' | head -1)}"
out="$art/trim-2d"
dotnet publish tests/Trim2DSmoke -c Release -r "$rid" --self-contained true -p:PublishTrimmed=true -p:TrimMode=link -o "$out" -v quiet -clp:NoSummary
"$out/Trim2DSmoke"
dotnet run --project tools/TrimCheck -c Release -- "$out/RecursiveAI.OnePhysics.dll"
