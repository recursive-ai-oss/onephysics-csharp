#!/usr/bin/env bash
# Publishes the smoke app with NativeAOT (trim and AOT warnings are errors) and runs it.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
out="$(mktemp -d)"
trap 'rm -rf "$out"' EXIT
dotnet publish tests/AotSmoke/AotSmoke.csproj -c Release -o "$out" -v quiet -clp:NoSummary
"$out/AotSmoke"
