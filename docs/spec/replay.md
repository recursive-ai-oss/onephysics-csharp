# Checkpoint replay envelope, version 1

The transport supports checkpoint playback (version 1) and automatic mutation
recording (version 2). A recorder attaches opt-in to either public world and logs
outer public mutation calls by name and their complete resulting state. It does
not attempt to serialize executable host code, delegates or callback closures.
Contact/solver callbacks must be installed equivalently by a playback host;
body/event callback mutations are recorded and replayed with callbacks suppressed.

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
kinds are rejected, never silently skipped. Version 2 adds the command records below; version 1 rejects them.

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

## Automatic mutation recording (version 2)

`using var recorder = world.StartRecording(stream)` attaches to a world; dispose
writes End and detaches, leaving the caller's stream open. The initial snapshot
is followed by live options. All successful outer public world/body/shape/joint/
character/vehicle/ragdoll/soft-body/fluid mutators are scoped. Nested implementation
calls are one outer command, not duplicate partially-constructed states. An
attempted mutator which throws after leaving a valid unchanged state may record
that unchanged effect; exceptions themselves are not replayed. Raw writable
storage cannot intercept assignments: `CaptureRawChanges(name)` provides an
explicit raw-tier boundary, and every Step synchronizes outstanding raw writes
before simulating. Mutations and Step counts are not selected by clocks or threads.

Version 2 keeps the same 16-byte header (version field = 2) and kind/length record
framing. In addition to version 1 kinds it defines:

| Kind | Payload |
| --- | --- |
| 3 BeginStep | float32 dt, positive finite seconds |
| 4 Mutation | int64 resulting world StepCount; uint16 name byte count; printable ASCII name bytes (1..512); nonempty complete snapshot bytes |
| 5 EndStep | int64 final world StepCount, then uint64 final hash when enabled |
| 6 LiveOptions | the dimension-specific fixed layout below |

A BeginStep increments the implied world count and must be matched by exactly
one EndStep before another BeginStep or End. Mutations are allowed before a frame
or after its simulation. They may change the world counter through an explicit
Restore; logical recorded frame count remains monotonic. Snapshot checkpoints
(kind 1) are not allowed inside a frame; mutation effects (kind 4) are. Playback
executes BeginStep with callbacks suppressed, applies callback-issued mutations
in recorded call order, then checks the EndStep hash. This ordering also handles
mutations of live options made by callbacks after simulation. The first differing
EndStep raises ReplayHashMismatchException with the resulting world counter.

Mutation effects deliberately use full snapshots rather than a per-type argument
serializer. This covers every current shape and service without reflection,
keeps geometry validation in the existing snapshot readers, and avoids invoking
creation side effects twice. It trades file size and recording time for simplicity;
there is no compression. Operation names identify the public mutation boundary.
Creation/destruction may grow storage; warmed steps and mutations allocate no
managed memory in the recorder. Caller streams have their own allocation behavior.

Live options are stored separately because core gravity/scheduling options only
synchronize at the next Step. Recording must not synchronize them early and change
a world's hash. A LiveOptions record follows the initial snapshot and every
mutation effect. Its first 12 bytes are int32 source world id and float64 actual
creation-time broadphase margin. Remaining fields follow the WorldOptions2D/3D
property declaration order, excluding WorldId, ThreadCount, MaterialMixer and
PairFilter. Vector components are float32 in X/Y/Z order, scalar float32/float64
use their IEEE bits, integers and EventMask use int32, booleans int32 0 or 1.
The exact field list is encoded explicitly in ReplayOptions2D/3D; no reflection
or runtime property enumeration is used. WorldId and thread count remain the
destination's namespace and execution settings. BroadphaseMargin's live value
is included separately from the fixed creation margin. Applying live values does
not update core options before the next Step. Host delegate code is not serialized.

`player.CreateWorld2D(threadCount)` and `CreateWorld3D` read the initial snapshot
and live options to construct a compatible world, including its original identity
and fixed broadphase settings. Continue with ReadNextStep on that world. Direct
playback into an existing world remains available; that world's fixed creation
options must be compatible, as for ordinary Restore. For version 1, automatic
creation uses default creation options because that envelope has no live-options
record. A caller requiring another version 1 configuration supplies the world.

## Bounded history

`world.StartHistory(new HistoryOptions { KeyframeInterval = 30,
MemoryBudgetBytes = 32 * 1024 * 1024 })` owns the recorder. It keeps a circular byte
store and fixed-capacity interval index, not a growing list of arrays. The total
budget includes that store/index, fixed overhead allowance and reserved working
snapshot storage. Source/destination world's simulation and restore storage are
not history storage. Oldest complete intervals are evicted in index order;
FirstStep and LastStep expose the retained logical range. Logical step 0 is the
starting state, independent of the initial world's StepCount.

Full keyframes are made after a completed Step including callback mutations,
at most K steps apart; memory pressure can insert one earlier. Each interval is
an independently headed replay segment. The writer reuses its buffers at segment
boundaries, so warmed recording, including keyframe cadence, allocates nothing.
A frame/interval or enlarged working snapshot that cannot fit the configured
budget raises HistoryCapacityException; the recorder detaches on an I/O/storage
failure rather than growing past its budget. Increase the budget or reduce K.

Seek restores the latest keyframe at or before the requested logical step and
replays only the remaining frames, validating their saved hashes. It does not
apply following input mutations merely because they share the world's counter.
Seeking into the source world suspends recording during playback; resuming its
simulation appends a new logical frame sequence, not a rewrite of old history.
Seeking during the destination's active Step is rejected. Stop leaves retained
history seekable; Dispose stops and invalidates further seeks. Keyframe capture
and restore pass through the internal IHistoryKeyframes seam, currently using
full snapshots, so a different storage representation does not change public APIs.

### Live option field order after the 12-byte prefix

- 2D: `Gravity`, `Substeps`, `ContactSubsteps`, `ContactVelocityIterations`, `ContactRelaxIterations`, `VelocityIterations`, `RelaxIterations`, `PairCapacity`, `ContactCapacity`, `IslandCapacity`, `QueryScratchCapacity`, `SleepEnabled`, `TimeToSleep`, `SleepThreshold`, `Events`, `ContactHertz`, `ContactDampingRatio`, `PushOutVelocity`, `ContactSlop`, `JointHertz`, `JointDampingRatio`, `RestitutionThreshold`, `MaxLinearSpeed`, `MaxRotationPerSubstep`, `ContinuousEnabled`, `BroadphaseMargin`, `HitSpeedThreshold`.
- 3D: `SoftSubsteps`, `SoftIterations`, `Gravity`, `Substeps`, `VelocityIterations`, `RelaxIterations`, `ContactSubsteps`, `ContactVelocityIterations`, `ContactRelaxIterations`, `PairCapacity`, `ContactCapacity`, `IslandCapacity`, `QueryScratchCapacity`, `SleepEnabled`, `TimeToSleep`, `SleepThreshold`, `Events`, `ContactHertz`, `ContactDampingRatio`, `PushOutVelocity`, `ContactSlop`, `RestitutionThreshold`, `MaxLinearSpeed`, `MaxRotationPerSubstep`, `ContinuousEnabled`, `JointHertz`, `JointDampingRatio`, `HitSpeedThreshold`, `SpeculativeDistance`, `BroadphaseMargin`.

Creation from files treats capacity values as preallocation hints, not allocation
commands: the world is initially constructed without reserving those pools, then
restored according to actual snapshot contents. Live hint values are retained for
round-trip fidelity. An explicit later Warmup may honor them, so a host loading
untrusted files should choose its own capacity policy before requesting warmup.
