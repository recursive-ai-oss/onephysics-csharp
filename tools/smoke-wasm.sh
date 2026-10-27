#!/usr/bin/env bash
# Builds the smoke app for browser WebAssembly and runs it under Node. Needs the wasm-tools workload and node.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
dotnet build tests/WasmSmoke/WasmSmoke.csproj -c Release -v quiet -clp:NoSummary
out="$(dotnet build tests/WasmSmoke/WasmSmoke.csproj -c Release --no-restore -getProperty:OutDir)"
bundle="$(mktemp -d)"
trap 'rm -rf "$bundle"' EXIT
cp -r "$out/AppBundle/." "$bundle"
cp tests/WasmSmoke/main.mjs "$bundle/main.mjs"
(cd "$bundle" && node main.mjs)
