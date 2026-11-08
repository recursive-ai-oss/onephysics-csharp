# Checkpoint replay envelope, version 1

This is the streaming transport for snapshots and simulated steps. It is not an
API-call recorder: calls made between steps must currently be represented by an
explicit replacement snapshot. A command track, automatic recording, bounded
history and the public text scene format are not implemented by this envelope.
A snapshot cannot represent host delegates or contact/solver callbacks. A host
using such callbacks must install equivalent callbacks before playback.

## Binary layout

All integer and floating-point fields are little-endian. Offsets are relative to
the beginning of a replay, not the underlying stream's current absolute position.
There is no padding, compression, native-sized integer, string, or object type name.

The 16-byte header is:

| Offset | Type | Value |
| --- | --- | --- |
| 0 | uint32 | `0x5250504f`, ASCII `OPPR` |
| 4 | uint16 | version, `1` |
| 6 | uint8 | dimension, `2` or `3` |
| 7 | uint8 | flags: bit 0 = per-step hash track, all other bits zero |
| 8 | int64 | nonnegative initial absolute completed step count |

Each record has a five-byte header: kind (uint8), payload byte length (int32,
nonnegative), followed immediately by that many bytes. Record kinds are:

| Kind | Payload | Meaning |
| --- | --- | --- |
| 0 | zero bytes | mandatory terminator |
| 1 | nonempty snapshot bytes | restore full state at the current completed step |
| 2 | float32 dt, then optional uint64 hash | simulate one step and compare the resulting hash |

The first record must be a snapshot. A snapshot does not increment the completed
step count and must contain the count implied by the header and preceding steps.
Subsequent snapshots can capture changed inputs between steps or serve as
keyframes. They are full world snapshots, byte for byte, with their own existing
snapshot version. The envelope does not change the world snapshot layout.

A step's dt is positive finite seconds, in exactly the float32 representation
passed to `world.Step`. With the hash flag its payload is exactly 12 bytes;
without, exactly 4. Each step increments the absolute completed step count by
one; overflow of signed int64 is invalid. The saved hash is the world's
`ComputeHash()` after the completed step. A step is executed during playback,
not replaced with a saved resulting pose. Hashes are compared at every step;
the first mismatch reports the absolute step count, expected hash and actual
hash. The divergent world remains available for inspection.

A terminator is required even for a replay with no steps. EOF before it is
truncation. A terminator leaves following stream bytes unread, allowing embedded
or concatenated replay objects. A file loader wanting exactly one object must
also check that the enclosing file ends there. Unknown versions, flags, and
kinds are rejected, never silently skipped. A future command track will need an
explicit format extension; this reader does not claim to understand commands.

## Reading and resource bounds

`ReplayReader` is the raw tier. `ReadNext` reads one record, including on
non-seekable streams returning arbitrarily short reads. `Payload` borrows its
reusable buffer until the next read; retain a copy if needed longer. The reader
validates envelope structure and dt, not opaque snapshot geometry. Apply a
snapshot through the destination world's validator before using it.

The default maximum payload length is 64 MiB, configurable at construction for
larger worlds. A declared length is checked against this bound before allocation
or payload reads. Buffer capacity starts at min(4096, bound) and grows to the
largest accepted record, never beyond the bound. There is no allocation based on
the total stream length and no input-controlled recursion. Validating a stream
is linear in the bytes read. Readers and writers leave the caller's stream open.
On a parse or I/O error discard the reader/player; on a write error discard the
writer and incomplete output. Errors name the input and a zero-based byte offset.

`ReplayPlayer.ReadNextStep(world)` is the convenient incremental consumer, with
2D and 3D overloads and no callback interface required. Keep the same destination
world for the whole playback. It restores snapshots and simulates one step per
successful call, returning false at the terminator. World callbacks are
suppressed during simulated steps; polled events are still available. Live
host-side mutations of the playback world are not supported. Invalid snapshot
bytes become `ReplayFormatException`; the world's existing snapshot validator
provides atomic rejection of malformed snapshots. An otherwise valid snapshot
with an inconsistent replay step count is rejected after restore, not rolled
back. Playback is not a transaction across records.

`ReplayWriter` writes the same canonical envelope without seeking. It allocates
only its fixed scratch array on construction, not per record. The caller provides
snapshot bytes and hashes and must synchronize live world options before taking
snapshots. The destination stream may have its own allocation behavior. The
reader allocates nothing per record after its payload buffer reaches the largest
record size. The player additionally relies on the world's warmed restore and
step storage, as ordinary snapshot playback does.

## Reproducible fixtures

`testdata/replay/planar.oppr` and `spatial.oppr` are generated entirely by
`FuzzReplay.Seed(2)` and `Seed(3)` in `fuzz/Fuzz/FuzzReplay.cs`. Regenerate with
`dotnet run --project fuzz/Fuzz -- --seeds <directory>`, then copy
`<directory>/replay/planar` and `spatial` to their `.oppr` paths. Both fixtures
start with a floor and box, simulate 12 steps at float32 `1/60`, and insert a
second snapshot before step 7 after adding a circle/sphere. Their expected
per-step hashes are in the files, not regenerated during matrix playback.

SHA-256:

- planar: `04aa41ffd70e08f24635b8258f4651b1be6956085dba8b05c49798ac3208b946`
- spatial: `f0c1d0c053171cf3e0c3ec87ea9f5da0f5a3f5659184deec9b3df9be934cb94d`

The matrix embeds these files for filesystem-independent AOT and browser-WASM
playback. `replay/checkpoint-2d` and `replay/checkpoint-3d` validate the saved
hashes and emit the actual count and world hash at each step. Existing world
scenarios and benchmark scene bytes are unchanged.
