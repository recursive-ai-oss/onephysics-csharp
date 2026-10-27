#!/usr/bin/env bash
# Copies the onemath package into the ignored .localfeed/ directory so restore can find it until it is on nuget.org.
# Usage: ONEMATH_FEED=<directory holding RecursiveAI.OneMath.*.nupkg> tools/localfeed.sh
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
version="0.1.0-preview.2"
if [ -z "${ONEMATH_FEED:-}" ]; then
  echo "set ONEMATH_FEED to the directory containing RecursiveAI.OneMath.$version.nupkg" >&2
  exit 2
fi
mkdir -p "$root/.localfeed"
cp "$ONEMATH_FEED/RecursiveAI.OneMath.$version.nupkg" "$root/.localfeed/"
echo "copied RecursiveAI.OneMath.$version into .localfeed/"
