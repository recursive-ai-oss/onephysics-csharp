# Snapshot byte transforms

These transforms wrap arbitrary byte sequences. In particular, a full snapshot
remains snapshot format v1, with every optional engine section intact. The
transforms neither interpret nor discard sections. A decoder returns bytes; the
world's existing atomic restore validates those bytes separately.

## Delta version 1

All integers are unsigned 32-bit little-endian. Header, in order:

| Offset | Value |
| --- | --- |
| 0 | Magic `0x4441504f` (`OPAD`) |
| 4 | Version `1` |
| 8 | Base byte length |
| 12 | Target byte length |
| 16 | SHA-256 of the entire base (32 bytes) |
| 48 | SHA-256 of the entire target (32 bytes) |

Records follow until end of input: target byte offset, positive byte count, then
that many literal target bytes. Records have ascending, nonoverlapping ranges
inside the target. The target starts as the base truncated to the target length;
all bytes beyond the base must be explicitly covered by records. There is no
terminator or padding. An unchanged target uses only the 80-byte header. A
shortened unchanged prefix also needs only the header.

The canonical writer scans offsets from zero. At the first changed byte it
starts a record, extends through subsequent changed bytes, and absorbs equal
gaps of at most eight bytes between changes. Equal bytes after the last change
are not included. Every byte past the end of the base counts as changed.

Apply checks magic/version, output limit, base length/digest, and every record
range before allocating the target. It then copies and checks the target digest.
Wrong bases, malformed records and checksum failures raise
`SnapshotEncodingException`. Checksums detect accidental damage and base mixups;
they are not authentication. Authenticate untrusted messages at the host boundary.

## Compression version 1

Header: magic `0x4341504f` (`OPAC`), version `1`, decoded byte length, then
SHA-256 of decoded bytes. Header length: 44 bytes. This envelope may contain a
full snapshot, a delta, or any other bytes. No native compression library is used.

Each packet starts with one byte `t`:

- `t < 128`: copy the following `t + 1` literal bytes.
- `t >= 128`: repeat the following single byte `t - 128 + 3` times.

The canonical writer takes the longest equal-byte run at the current offset,
capped at 130. A run of at least three becomes a repeat packet. Otherwise it
starts a literal packet and extends until the next run of at least three, end of
input, or 128 literals. A run of 131 bytes therefore becomes a 130-byte repeat
and a one-byte literal. Empty input has only the header. Encoded bytes depend
only on input bytes, not CPU, thread count, runtime codec version or OS.

Decode validates all packets, their total output length and the output limit
before allocation, then checks the decoded digest. Truncation, trailing packets,
unknown versions and damaged checksums raise `SnapshotEncodingException`.
The default output limit is 256 MiB; callers can pass a nonnegative byte limit
on each decode/apply call. Applications should choose a limit appropriate for
their worlds. There is no recursive decoding. Compressed deltas are explicitly
decompressed, then applied, so both limits can be enforced independently.

## APIs and state

`SnapshotEncoding` is the raw byte API: `CreateDelta`, `ApplyDelta`, `Compress`,
`Decompress`. Both worlds expose identical `SnapshotDelta`, `RestoreDelta`,
`SnapshotCompressed` and `RestoreCompressed` convenience members. A host can keep
a base keyframe and encode later steps against it without retaining a world.
The transforms keep no persistent world state and add no registered snapshot
sections. They allocate output buffers and are intended for host tooling, not the
allocation-free simulation loop. Delegates remain excluded just as in full
snapshots. Decode failure leaves a world untouched.

Compression is designed for sparse binary snapshots, especially their zero
lanes, not for general-purpose archival ratios. For highly fragmented changes
a delta can exceed a full snapshot; hosts can compare lengths and send the
smaller representation. Compression also has bounded literal overhead (one byte
per 128 bytes, plus its header).

## Delta version 2: XOR payloads

`CreateXorDelta` uses the delta v1 header and record layout with version `2`.
The target digest still hashes the original full target. Each record payload
byte is `target[offset] XOR base[offset]`, taking base bytes past its end as
zero. Apply starts with the truncated/zero-extended base and XORs each payload
into it. All v1 ordering, length, explicit extension and base-identity checks
still apply. The v1 writer and decoder remain available unchanged. Both versions
have an 80-byte header. XOR often exposes zero runs in equal high bytes or short
gaps absorbed by a record; it is optional, not a change to snapshot v1.

## Compression version 2: byte planes

`Compress(bytes, bytePlanes: true)` writes compression version `2`, with the
same 44-byte header and SHA-256 of the **original** bytes. Before packet coding,
each complete 64-byte tile is transposed as an 8 by 8 byte matrix:
`output[tile + column*8 + row] = input[tile + row*8 + column]`.
The tail of fewer than 64 bytes is unchanged. Transposition is its own inverse.
The run/literal packet writer is otherwise exactly version 1. Decode validates
packet lengths, decodes, transposes the complete tiles again, then checks the
original digest. There are no architecture-dependent lanes, numerical
conversions, native codecs or schema/alignment assumptions.

Full snapshots, literal deltas, XOR deltas and region archives may use either
compression version. Hosts can compare lengths; byte planes do not guarantee
improvement on arbitrary input. Known-answer tests pin both new versions in
addition to the unchanged compression-v1 answer.

## Structural world forks

`PhysicsWorld2D.Fork`, `PhysicsWorld3D.Fork` and raw `WorldCore.Fork` copy state
**directly**, without serialization. Every live store's array contents,
capacities, handle allocator generations/free heaps, coloring, islands, sleep,
contacts and warm starts, broadphase nodes/pending moves, pair table and all
built-in engine sections are independently owned. Soft-body particles, poses,
constraints, skin targets and definitions, character/controller records, vehicle
wheel/drivetrain state, ragdolls and fluid definitions are copied as well.
Immutable collision geometry, meshes and skin definitions may be shared. Mutable
geometry records/reference counts are separate; disposable derived scratch and
geometry cutting caches may start empty and are rebuilt before use.

This is a linear-time, linear-memory structural clone, **not copy-on-write**.
No COW branches or borrow barriers are added to the solver or writable spans.
Previously borrowed source spans remain source spans and cannot mutate the
fork. COW is a later optimization only if host workloads justify it; it is not
part of the current cost claim.

A fork retains the numeric world id, handle slots and generations. Typed handle
structs still refer to their original world object: obtain the corresponding
handles from the fork before using its happy-path API. Raw handles use the
numeric id and are meaningful in either explicitly chosen world context.

Event subscriptions, destroyed-handler retention, pair-predicate delegates,
struct contact-filter callbacks and suppression scopes are **not copied**.
No source handler is invoked by stepping the fork. Saved delivered/pending
physics events **are** copied; public 3D event aggregation is rebuilt from them.
As with restoring into a fresh world, user callback code is host input, not
serializable simulation state: reinstall equivalent physics-filter policies if
prediction needs them. With the same external inputs/policies, fork and a fresh
restore follow identical per-step hashes. Event bytes are also identical when
the restored world uses the same numeric world id.

Call between steps on the world's owning thread. `Fork` does not mutate the
source, transfer its arrays or invalidate its handles. Mutating/stepping either
copy does not change the other's stored simulation state. A custom registered
section must implement `ISnapshotSection.Fork(destination)` and return an
independent section with the same id. Dependencies earlier in registration
order are available through `destination.GetSnapshotSection(id)`. The default
implementation throws `NotSupportedException`; custom state is never silently
omitted. Built-in sections implement the same hook.

## Optional 2D vehicle and ragdoll sections

Two optional sections of the planar world, written only while used: "VEH2" (`0x32484556`, version 1: vehicles) and "RAG2" (`0x32474152`, version 1: ragdolls). Layouts are in `world2d.md` → Vehicles. They follow the section rules above (an older reader rejects them by id and length; worlds without vehicles or ragdolls keep their bytes).


## Optional aerodynamics section

One optional section in both worlds, "AERO" (`0x4F524541`, version 1): thrusters, boats, hovercraft, shape surfaces and the air density. Written only while any of them exists or the air density is not 1.225 kg/m³; registered after every other section (2D order: engine, fields, rods, machines, mass offsets, tiles, SF2D, VEH2, RAG2, AERO). Entity kinds for the desync finder and regions: `thruster`, `boat`, `hovercraft`, `aero` (by shape slot). Layout in `actuators.md`. Stored axes are kept bit for bit so a restored world re-serializes to the same bytes.

## Optional character-ability section

One optional section in both worlds, "CEXT" (`0x43455854`, version 1), registered after every other section (so after the aerodynamics section): `i32 version`, `i32 count`, then per record in ascending character slot: `i32 slot`, `i32 generation`, `u8 flags` (bit 0 may climb, bit 1 carries by pose, bit 2 is carried), `u8 mode` (0..4), `i32 body, i32 bodyGeneration`, `f32 local[3]` (seat position, held centre, or the carried character's position in the body's frame), `f32 localRotation[4]` (a unit quaternion; identity when unused; about z in 2D), `f32 normal[3]`, `f32 along[3]`, `i32 rod, i32 rodGeneration, i32 particle`, `f32 fraction`; then `i32 climbableCount` and per climbable shape in ascending slot `i32 slot, i32 generation`. Every record has every field whatever its mode, so the layout does not change when a character sits down or lets go (regional restores need a stable field set). A record exists exactly while a flag is set or the mode is not Free; a reader rejects an all-zero record, a slot at or above the restored body count (a character has a body of its own), an unordered slot, a mode above 4, flags above 7, a non-unit rotation, a non-unit normal where one is used, a non-finite or beyond-4194304 anchor, a rope grip outside its ranges, and a character generation that differs from the restored character table's. It is written only while something is in use, so worlds that never use abilities keep their bytes and hashes. Region archives label the records `character` and climbable shapes `shape`.

## Diverged worlds

Restore refuses a non-finite body position or origin, so writing one would produce bytes nothing can read.
Every snapshot write (`Snapshot`, `TrySnapshot`, `GetSnapshotSize`, `GetSnapshotSizeBound`) therefore first checks
the origin lanes and then every body-slot position lane in index order, and raises `NonFiniteStateException`
naming the first non-finite value (`Body slot S position lane L is V`) before anything is written. The world is
unchanged, `ComputeHash` still hashes it, and destroying or resetting the body makes it saveable again. Velocities
and rotations are not checked at write time, because restore accepts them (the next step reports such a body).

## Named fields and desync reports

`SnapshotDesync.Compare(fullA, fullB)` validates both full snapshots with the
existing readers, then describes them with the real writers. It does not use a
second independent snapshot-layout parser. Construction-time broadphase margin
is probed through the core's validated section envelopes before creating the
dimension-specific reader. Unknown section ids are rejected cleanly. For custom
sections, compare existing worlds/cores, or use
`world.FindDifferences(snapshot)` so the current world's registered section
schema supplies the readers. The current world is unchanged.

`SnapshotDesync.Inspect` returns named primitive/byte-block fields with section,
entity kind, numeric slot, generation, name, occurrence, primitive type, byte
offset/length, invariant readable value and a copyable raw little-endian value.
Offsets refer to the inspected full snapshot or region archive. The raw typed
list is useful independently of the readable report.

Field names identify the corresponding written state member, with explicit
vector lanes/array elements. Core body columns use `position x/y/z`, `rotation`
plus quaternion lane, `linear velocity x/y/z`, `angular velocity`, `inverse mass`,
`inverse inertia`, and `sleep time`; scalar/packed columns retain their named
index where applicable. Other sections use their written member names (for
example `Force[0]`, `CharVx[c]`, `Forward`, `MotorSpeed`, `Strength`, `Density`,
`Pos[0]`, `Vel[0]`, `MinX`). These are field identities, not reflection over user
types. Repeated fixed-format rows such as wheel/vertex parameters have an
explicit zero-based **occurrence** within their entity, in write order.

The descriptor uses the compile-time written expression as its default label:
strip a terminal `.AsSpan(...)` slice, strip a leading `s.` or `d.`, remove
leading underscores, apply the core-body aliases above, then append the column
lane/element. Literal labels supplied by custom `StateWriter.DescribeField`
are retained verbatim. No source filenames, stack walking or runtime reflection
are involved. Custom annotations must use valid UTF-8 identifiers of at most
4096 encoded bytes. Any unannotated bytes, including framing and custom payload,
are still compared as an explicit opaque range, never ignored.

Differences are ordered by ordinal section and entity name, numeric slot and
generation, ordinal field name, then numeric occurrence. Fields present only
in B follow those inspected from A. Missing fields/entities have a null side;
floats compare **bits**, so signed zero and different NaN payloads differ. Values
show round-trip invariant numbers and hexadecimal float bits. Long byte blocks
show a prefix and length; the full bytes remain available separately. The
summary count is the count of differing named fields, not bytes. `First` is
null for equality. `ToString` prints the count and at most 16 lines:

```
bodies: body 412 (gen 3) linear velocity y: a=0 [0x00000000] b=2 [0x40000000]
```

No handlers, filters or simulation callbacks are run during comparison.

## Region selection and closure

`CaptureRegion(minimum, maximum)`, `SnapshotRegion` and `ComputeRegionHash` have
identical semantics in both dimensions. Bounds are inclusive doubles in metres
relative to the current origin; every lane must be finite, ordered and within
±2097152. Membership seeds are **live rigid bodies whose stored body origins lie
inside the box**, not shape bounds or centre-of-mass offsets. A body's whole set
of shapes comes with it. Body slots are stable identities, not renumbered.

Closure is conservative and iterated to a fixed point in ascending slot order:

1. Every selected mass-bearing dynamic body's live constraints bring the other
   endpoint and its shapes. Include complete joints, contact/sensor constraints,
   warm-start/manifold payloads, feature ids, colors and associated island state.
   Static/kinematic anchors come along but do not fan out through other objects
   touching the same floor/platform. Constraints whose endpoints are selected
   (or whose other endpoint is the fixed world) are archived.
2. Sensor overlaps kept only in the engine pair table get the same dynamic-to-
   anchor closure. Cached pair records with both shape endpoints selected are
   archived; pair/free-slot allocation metadata is not a regional resource.
3. A selected ragdoll part brings the whole ragdoll and all its parts/joints.
   A selected vehicle chassis brings its controller and last wheel-ground body
   dependencies. A selected character brings its controller and ground/platform
   body. These added dynamic bodies participate in the same constraint closure.
4. A 3D soft body is seeded when **any particle point** lies in the box. Include
   the whole body, never a cut particle subset: mesh, definition, pinning, sleep,
   skin poses and every saved particle column. For soft/rigid coupling, a selected
   soft body's particle AABB, expanded by particle radius, brings all touching
   rigid shape owners. Conversely a selected dynamic rigid body's tight shape
   bounds bring touching soft bodies. Repeat closure. This is AABB-conservative;
   static anchors never pull in unrelated soft bodies resting on them.
5. Fluids are world fields and all saved fluid definitions are included as global
   context. Unknown custom sections are likewise global context, not silently
   cut apart.

Include each selected body's/shape's handle generation, shape geometry and
ownership order, each selected island's saved sleep/removal state and every
selected service record. Delivered/pending events touching any selected body
are included; their filtered local sequence is renumbered from zero separately
for delivered and pending lists. Outside events cannot shift these local ids.
The archive also includes saved tick, origin and numeric simulation configuration,
including active engine option/filter-table blocks. Activating/changing global
configuration can therefore change every region hash. Historical global slot
heaps, counts of unrelated objects, tree node identities, spatial-index internals
and solver scratch are not regional state. Selected handle/contact/cache ids
remain significant: this is a simulation-state hash, not a geometry-only hash
independent of allocation history.

Straddling objects are included whole. A small seed box can intentionally bring
a large connected mechanism or coupled soft/rigid cluster. Unrelated islands
sharing only a static anchor are not included. There is no promise that an
archive can be stepped autonomously: future motion may create new boundary
interactions, and hosts recapture interest regions at their chosen fixed ticks.

## Region archive version 1

All integers little-endian. Header: magic `0x5241504f` (`OPAR`), version `1`,
dimension int32 (2 or 3), `dimension` float64 minimum coordinates, `dimension`
float64 maximum coordinates, then int32 field count. Each record contains:

- section UTF-8 string;
- entity-kind UTF-8 string;
- int32 slot (-1 for global) and int32 generation (0 when not applicable);
- complete field-name UTF-8 string;
- primitive-type UTF-8 string;
- int32 byte count, then the raw little-endian value bytes.

A string is an int32 byte count followed by strict UTF-8, length 1..4096 bytes.
Records use the same canonical key/occurrence order as desync fields. Repeated
names are interpreted in occurrence order, not deduplicated. Accepted types are
`byte`, `bool`, `int32`, `uint32`, `int64`, `float32`, `float64`, and `bytes`;
fixed-width types must have their exact width. The dimension and bounds are also
exposed as named header fields. No trailing bytes or noncanonical record ordering
are permitted. Archive creation is deterministic; `Read` is lossless.

The hash is the existing world's `StateWriter.ForHash` fold over **every archive
byte**, including header/bounds/context and framing. It is not the full world's
hash with bodies masked out. Compression can wrap an archive without changing
its logical hash; hash the decoded canonical archive, not a chosen codec.

`RegionSnapshot.Read` defaults to a 256 MiB input limit and 1,000,000-record limit,
both explicitly overridable for larger host worlds. Counts/ranges, UTF-8, type
widths and bounds are checked before the relevant allocation. Malformed archives
raise `SnapshotFormatException`; negative caller limits raise
`ArgumentOutOfRangeException`. This format is **not** full snapshot v1 and cannot
be passed to ordinary `Restore`.

## Regional partial restore

`RestoreRegion` is an atomic **value resync at the same tick**, not an entity
importer or topology reconciler. It requires:

- identical dimension, tick, origin and saved global numeric options/context;
- selected body handles still live with matching generations;
- the same selected field schemas, entity/shape/constraint/island/cache identities
  and closure on the receiving world; no missing/extra rows or width changes.

A source body may have drifted outside the receiving box: restore selects the
archive's explicit handles, not a fresh position-only seed on the receiver, then
checks its closure. Topology changes, new/destroyed entities, incompatible caches
or different ticks require a full snapshot. Rejection raises
`SnapshotFormatException` with the current world unchanged.

On success, only selected field values are replaced in an existing full snapshot.
The real full-snapshot readers validate that candidate in an independent staging
world. Selected proxies are refreshed there from the restored poses/geometry;
tree-node identities are never imported. A final ordinary atomic restore commits
it. Unselected body/service values and existing subscriptions are preserved;
callbacks/events are not dispatched. Global allocation/free-list history is not
replaced. This intentionally favors correctness and inspection/resync utility
over allocation-free or constant-time regional restore.

## Full-hash comparison after regional repair

A regional archive does not import broadphase tree history. `RestoreRegion` alone
therefore promises regional, **not full-world**, hash equality. For full-hash
resynchronization call `CanonicalizeBroadphase()` on **both** worlds between the
same steps, after repair. This explicit operation leaves existing snapshots and
hashes unchanged for worlds that never call it. It allocates and is a tooling
operation, not a per-step requirement.

Canonicalization validates every current shape bound before mutation, discards
all three trees, clears move/staging/pair buffers and resets the three refit
counters to zero. It then visits live shape slots in ascending index order,
creating each proxy in its body's type tree from its current tight bound, using
the usual static/dynamic margin and zero predicted displacement. Proxy keys in
the engine are rewritten; every live proxy is queued for pair discovery in that
same order. This replaces history-dependent bounds, placement, node flags, free
lists and move membership together. Existing pair/contact caches, islands,
constraints, events, bodies, subscriptions and options are not changed.

If the non-broadphase state already matches (including topology, global handle
and pair-slot histories, tick, caches and options), this procedure gives identical
full snapshot bytes and hashes and identical next steps. It does not hide a
remaining outside-region divergence or repair incompatible allocation histories;
use the desync finder and a full snapshot for those. Repeating it without changing
state is byte-idempotent. Historical proxy keys/tree references obtained through
the raw tier must be reacquired afterward. Call `Warmup` again if subsequent
steps must allocate nothing: tree scratch capacity is rebuilt too. Rebuilding from geometry is simpler
than importing partial tree nodes, whose parent/free-list topology crosses the
region boundary.

## Exact character slope inputs

Character definitions store `SlopeLimitRadians` as float32 radians; the getter
returns the input bits, including signed zero. For compatibility they retain a
legacy degree wire value. If converting that degree value to radians reproduces
the public radian bits, the existing snapshot bytes and solver double-radian
conversion are retained (including the default and Giant presets). Otherwise the
solver uses the exact float32 radians widened to double.

A world with at least one slope that cannot survive the legacy conversion uses
**engine section version 5 in 2D, version 3 in 3D**. Full snapshot envelope version
1 is unchanged. These section versions have the preceding character-capable
version's layout and flags, except that each live character record starts with
one boolean byte before its body index: false means its slope field retains
legacy degrees, true means the same float32 field contains exact radians. At
least one true record is required, and a true record whose radians survive the
legacy conversion is rejected as noncanonical. All other block orders and fields
are unchanged. Old readers reject the new section version cleanly. Current
readers accept legacy sections and reserialize them byte-exactly. Mixed legacy
and exact slopes retain each legacy record's degree bits. Worlds without exact
slope inputs keep their previous section version, snapshot bytes and hashes.

## Indicative measurement method

`bench/SnapshotFoundations` uses the shared generated `large-pile` and
`large-pile-3d` definitions, one simulation thread and fixed dt, after 300 steps.
Each separate process measures either structural fork or full Snapshot+fresh
Restore (same numeric id). Two warm copies precede five samples. Collection is
outside the clock. It reports best/median elapsed time, managed allocated bytes,
a separately collected approximate retained-heap delta, and process peak working
set. Every copy's hash is verified outside the clock. The source hash is unchanged.
Run processes separately under `/usr/bin/time -v` for comparable peak RSS, with
runtime tiering off for both methods. These are indicative, loaded-host numbers,
not managed-engine bar rows. CPU/core count, runtime/OS and load before/during/
after are recorded by the runner. Compression sizes use a keyframe at step 300
and target at 301, with the real soft bodies added for cloth scenes.

Terrain edits (carvable bitmask terrain, voxel edits, signed distance fields) add no section: the 2D tile section
is version 3 when a contour grid exists (see `collision2d.md`), a voxel edit rewrites the occupancy bytes of its
geometry recipe (kind 15, `collision3d.md`), and a signed distance field is geometry recipe kind 16 in the 3D engine
section (kind byte, int32 sizes x y z, float32 cell size, float32 origin x y z, float32 samples). Voxel geometries are
never shared by recipe: every voxel shape has its own geometry entry. Readers validate sizes against the bytes left and
reject non-finite samples with a typed snapshot error.
