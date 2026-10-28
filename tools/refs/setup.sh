#!/usr/bin/env bash
# Fetches, verifies and builds the reference drivers under a state directory (default ~/onephysics).
# Usage: tools/refs/setup.sh [STATE_DIR] [box2d|rapier|rapier3d|jolt|box3d|all|all3d|profile]
# `profile` builds the Rapier drivers with the crate's profiler feature (rapier-driver-profile, rapier3d-driver-profile); they are used only by the per-stage profile mode.
# Needs a C compiler and, for Rapier, cargo. CMake is downloaded (pinned) into STATE_DIR/tools when missing.
# Nothing it fetches or builds is ever read; drivers only link against the installed libraries.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "$here/../.." && pwd)"
state="${1:-$HOME/onephysics}"; what="${2:-all}"
. "$here/pins.env"
mkdir -p "$state/dl" "$state/tools" "$state/refs" "$state/bin"

sha256_of() { if command -v sha256sum >/dev/null; then sha256sum "$1" | cut -d' ' -f1; else shasum -a 256 "$1" | cut -d' ' -f1; fi; }
fetch() { # url sha dest
  if [ -f "$3" ] && [ "$(sha256_of "$3")" = "$2" ]; then return; fi
  curl -fsSL -o "$3.part" "$1"
  [ "$(sha256_of "$3.part")" = "$2" ] || { echo "sha256 mismatch for $1" >&2; rm -f "$3.part"; exit 1; }
  mv "$3.part" "$3"
}

os="$(uname -s)"
if [ "$os" = Darwin ]; then RUSTUP_URL=$RUSTUP_MACOS_URL; RUSTUP_SHA=$RUSTUP_MACOS_SHA; else RUSTUP_URL=$RUSTUP_LINUX_URL; RUSTUP_SHA=$RUSTUP_LINUX_SHA; fi
if [ "$os" = Darwin ]; then cm_url=$CMAKE_MACOS_URL; cm_sha=$CMAKE_MACOS_SHA; cm_dir="cmake-$CMAKE_VERSION-macos-universal/CMake.app/Contents/bin"
else cm_url=$CMAKE_LINUX_X64_URL; cm_sha=$CMAKE_LINUX_X64_SHA; cm_dir="cmake-$CMAKE_VERSION-linux-x86_64/bin"; fi
if ! command -v cmake >/dev/null 2>&1; then
  fetch "$cm_url" "$cm_sha" "$state/dl/cmake-$CMAKE_VERSION.tar.gz"
  [ -d "$state/tools/$cm_dir" ] || tar -xzf "$state/dl/cmake-$CMAKE_VERSION.tar.gz" -C "$state/tools"
  export PATH="$state/tools/$cm_dir:$PATH"
fi

build_box2d() {
  local root="$state/refs/box2d-$BOX2D_VERSION"
  fetch "$BOX2D_URL" "$BOX2D_SHA" "$state/dl/box2d-$BOX2D_VERSION.tar.gz"
  if [ ! -d "$root/src" ]; then mkdir -p "$root/src"; tar -xzf "$state/dl/box2d-$BOX2D_VERSION.tar.gz" -C "$root/src" --strip-components=1; fi
  cmake -S "$root/src" -B "$root/build" -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF \
    -DBOX2D_SAMPLES=OFF -DBOX2D_UNIT_TESTS=OFF -DBOX2D_DOCS=OFF -DCMAKE_INSTALL_PREFIX="$root/install" >"$root/configure.log" 2>&1 || { tail -30 "$root/configure.log"; exit 1; }
  cmake --build "$root/build" --config Release -j 4 >"$root/build.log" 2>&1 || { tail -30 "$root/build.log"; exit 1; }
  cmake --install "$root/build" --config Release >"$root/install.log" 2>&1
  local lib; lib="$(find "$root/install" -name 'libbox2d*.a' | head -1)"
  cc -O2 -std=gnu11 -DDRIVER_VERSION="\"$BOX2D_VERSION\"" -I"$root/install/include" "$repo/bench/drivers/box2d/driver.c" "$lib" -lm -lpthread -o "$state/bin/box2d-driver"
  echo "built $state/bin/box2d-driver (Box2D $BOX2D_VERSION)"
}

build_box3d() {
  local root="$state/refs/box3d-$BOX3D_VERSION"
  fetch "$BOX3D_URL" "$BOX3D_SHA" "$state/dl/box3d-$BOX3D_VERSION.tar.gz"
  if [ ! -d "$root/src" ]; then mkdir -p "$root/src"; tar -xzf "$state/dl/box3d-$BOX3D_VERSION.tar.gz" -C "$root/src" --strip-components=1; fi
  cmake -S "$root/src" -B "$root/build" -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF \
    -DBOX3D_SAMPLES=OFF -DBOX3D_UNIT_TESTS=OFF -DBOX3D_DOCS=OFF -DCMAKE_INSTALL_PREFIX="$root/install" >"$root/configure.log" 2>&1 || { tail -30 "$root/configure.log"; exit 1; }
  cmake --build "$root/build" --config Release -j 4 >"$root/build.log" 2>&1 || { grep -E 'error|Error' "$root/build.log" | head -30; exit 1; }
  cmake --install "$root/build" --config Release >"$root/install.log" 2>&1
  local lib; lib="$(find "$root/install" -name 'libbox3d*.a' | head -1)"
  cc -O2 -std=gnu11 -DDRIVER_VERSION="\"$BOX3D_VERSION\"" -I"$root/install/include" "$repo/bench/drivers/box3d/driver.c" "$lib" -lm -lpthread -o "$state/bin/box3d-driver"
  echo "built $state/bin/box3d-driver (Box3D $BOX3D_VERSION)"
}

build_jolt() {
  local root="$state/refs/jolt-$JOLT_VERSION"
  fetch "$JOLT_URL" "$JOLT_SHA" "$state/dl/jolt-$JOLT_VERSION.tar.gz"
  if [ ! -d "$root/src" ]; then mkdir -p "$root/src"; tar -xzf "$state/dl/jolt-$JOLT_VERSION.tar.gz" -C "$root/src" --strip-components=1; fi
  cmake -S "$repo/bench/drivers/jolt" -B "$root/build" -DCMAKE_BUILD_TYPE=Distribution -DJOLT_ROOT="$root/src" >"$root/configure.log" 2>&1 || { tail -30 "$root/configure.log"; exit 1; }
  cmake --build "$root/build" -j 6 >"$root/build.log" 2>&1 || { grep -E 'error|Error' "$root/build.log" | head -30; exit 1; }
  cp "$root/build/jolt-driver" "$state/bin/jolt-driver"
  echo "built $state/bin/jolt-driver (Jolt $JOLT_VERSION, Distribution)"
}

build_rapier() { # build_rapier 2|3 [profile]
  local dim="${1:-2}"; local variant="${2:-}"; local crate=rapier; [ "$dim" = 3 ] && crate=rapier3d
  local mdir="$repo/bench/drivers/$crate"
  command -v cargo >/dev/null || export PATH="$HOME/.cargo/bin:$PATH"
  if ! command -v cargo >/dev/null; then # user-local Rust toolchain, installed once under the state directory
    export RUSTUP_HOME="$state/tools/rustup" CARGO_HOME="$state/tools/cargo"
    if [ ! -x "$CARGO_HOME/bin/cargo" ]; then
      fetch "$RUSTUP_URL" "$RUSTUP_SHA" "$state/dl/rustup-init"; chmod +x "$state/dl/rustup-init"
      "$state/dl/rustup-init" -y --no-modify-path --profile minimal --default-toolchain stable >"$state/dl/rustup.log" 2>&1
    fi
    export PATH="$CARGO_HOME/bin:$PATH"
  fi
  local exe=rapier-driver; [ "$dim" = 3 ] && exe=rapier3d-driver
  if [ "$variant" = profile ]; then
    CARGO_TARGET_DIR="$state/refs/$crate-target-profile" cargo build --release --features profile $( [ -f "$mdir/Cargo.lock" ] && echo --locked ) --manifest-path "$mdir/Cargo.toml"
    cp "$state/refs/$crate-target-profile/release/$exe" "$state/bin/$exe-profile"
    echo "built $state/bin/$exe-profile"; return
  fi
  CARGO_TARGET_DIR="$state/refs/$crate-target" cargo build --release $( [ -f "$mdir/Cargo.lock" ] && echo --locked ) --manifest-path "$mdir/Cargo.toml"
  cp "$state/refs/$crate-target/release/$exe" "$state/bin/$exe"
  echo "built $state/bin/$exe"
}

case "$what" in
  box2d) build_box2d ;;
  rapier) build_rapier 2 ;;
  rapier3d) build_rapier 3 ;;
  jolt) build_jolt ;;
  box3d) build_box3d ;;
  all) build_box2d; build_rapier 2 ;;
  all3d) build_jolt; build_box3d; build_rapier 3 ;;
  profile) build_rapier 2 profile; build_rapier 3 profile ;;
  *) echo "unknown target $what" >&2; exit 2 ;;
esac
