# Bake cache format (version 1)

Normative for the bytes `world.BeginBake(...)` writes and `BakeClip` reads, and for the offline quality preset. A reader in another language needs only this page.

A bake cache holds what a world did over a run, as tracks of poses, points, fractures and events, so that an animation can be played back with no world. The file is a deterministic function of the world's history and the bake options: every platform, vector width and thread count writes the same bytes. A reader of version 1 reads every later 1.x file (new data arrives as new chunks, which it skips).

## Conventions

- Little-endian. `u8/u16/u32/u64` unsigned, `i8/i64` signed, `f32/f64` IEEE-754 bit patterns (a NaN is written as the canonical quiet NaN 0x7FC00000 / 0x7FF8000000000000 and rejected by readers where a finite number is required).
- **varint**: unsigned LEB128, 7 bits per byte, low group first, at most 10 bytes (the tenth byte is 0 or 1). **zigzag**: a signed 64-bit integer `v` is stored as the varint of `(v << 1) ^ (v >> 63)`.
- **string**: a varint byte length (at most 1024) and that many bytes of UTF-8.
- A reader refuses a file by a typed error that names the field and the byte offset; nothing in this page requires recursion, and every count is checked against the bytes left before anything is allocated.

## File layout

```
header (fixed part 72 bytes, then two strings)
chunk*      (any order, each tag at most once)
END chunk   (last)
```

### Header

| offset | type | field | rule |
|---|---|---|---|
| 0 | u32 | magic | 0x4B42504F (the bytes `O P B K`) |
| 4 | u16 | format version | 1; a reader refuses another value |
| 6 | u16 | flags | 0; ignored by readers |
| 8 | u8 | dimension | 2 or 3 |
| 9 | i8 | position exponent | -40 to 0: rigid-body positions are multiples of 2^e metres |
| 10 | i8 | vertex exponent | -40 to 0: soft, rod and particle positions are multiples of 2^e metres |
| 11 | u8 | rotation bits | 0 to 30: rotation components are multiples of 2^-bits; 0 means exact float32 bit patterns |
| 12 | u32 | frame count | 1 to 2^24 |
| 16 | u32 | frame stride | 1 to 1024: world steps between frames |
| 20 | u32 | block frames | 1 to 4096: frames per independently coded block |
| 24 | f32 | dt | seconds per world step, finite, above 0 |
| 28 | u32 | reserved | 0, ignored |
| 32 | i64 | start step | the world's step count when frame 0 was taken |
| 40 | 32 bytes | replay digest | SHA-256 of the replay file that reproduces the run (all zero when none was given) |
| 72 | string | scene name | |
| | string | library version | `major.minor.patch` of the writing library, or the text the host chose |

The time of frame `k` is `k * stride * dt` seconds (`stride * dt` computed in double from the float `dt`). Frame 0 is the world as it was when the bake began; frame `k` is the world after `k * stride` steps. Re-simulating from the replay named by the digest and comparing the world hash at frame `k` with entry `k` of the HASH chunk checks any frame.

### Chunks

Each chunk is `u32 tag, u32 length, length bytes`. The length must fit in the rest of the file. A tag a reader does not know is skipped; a known tag may appear once only.

| tag (ASCII) | contents |
|---|---|
| `HASH` | `frame count` × u64: the world hash after each frame was taken |
| `RIGD` | rigid-body tracks |
| `SOFT` | soft-body and cloth tracks |
| `RODS` | rod, rope, cable and hair tracks |
| `PART` | fluid and granular particle tracks |
| `FRAC` | fracture records and fragment geometry |
| `EVNT` | events |
| `END ` | exactly 8 bytes: the u64 FNV-1a (offset basis 14695981039346656037, prime 1099511628211) of every file byte before this chunk's tag |

A track chunk is present when the track was recorded (an empty one has count 0). `END ` is the last chunk and nothing follows it. A reader may skip the checksum.

## Entity tracks (`RIGD`, `SOFT`, `RODS`, `PART`)

```
varint entity count
entity*:
  varint key, varint generation, varint first frame, varint frame count (>= 1, first + count <= frame count)
  kind-specific fields (below)
  varint block count   = ceil(frame count / block frames)
  varint block length  * block count
  block bytes          * block count   (concatenated, in order)
```

An entity is a body (`key` = slot, `generation`), a soft body (slot, generation), a rod (slot, generation) or a fluid particle (`key` = particle id, generation 0). It exists for the consecutive frames `first` .. `first + count - 1`. A body that appears later (a fracture fragment, a body created mid-run) has its own `first`; a thing that is destroyed ends. A slot that is reused has a new generation and so a new entity. Entities are listed in the order they first appear; within one frame, in slot order (particles by id).

Kind-specific fields:

- `SOFT`: `u8 flags` (bit 0: cloth, that is a body with fabric behaviour; other bits ignored), `varint point count` (1 to 2^24), `varint triangle count`, then `3 × triangle count` varint indices (each below the point count): the body's surface triangles, once.
- `RODS`: `varint point count` (1 to 2^24).
- `PART`: `varint fluid` (the fluid slot the particle belongs to). One point per entity.
- `RIGD`: nothing.

### The numbers of a frame

Each frame of an entity is a vector of signed 64-bit integers, in this order:

- `RIGD`, 3D: `px py pz` then `rx ry rz rw` (7 numbers). 2D: `px py` then `rc rs` (4 numbers; the rotation's cosine and sine).
- `SOFT`, `RODS`, `PART`: for each point in order, its `x y z` (3D) or `x y` (2D).

Positions: `q = roundToEven(x * 2^-exponent)` where `exponent` is the position exponent (rigid) or the vertex exponent (the rest); the product is exact (a power of two), so the rounding is the only error: at most half a quantum. A value whose `|x * 2^-exponent|` reaches 4e18 or that is not finite cannot be stored (the writer raises `BakeRangeException`). Position is the body origin (`Body.Position`), not the centre of mass.

Rotations: when rotation bits `b > 0`, `q = roundToEven(c * 2^b)` for each float32 component `c`; when `b = 0`, `q` is the component's bit pattern as a signed 32-bit integer (exact). The 3D quaternion is `(x, y, z, w)`. The writer keeps `q` and `-q` (the same rotation) on one side: a frame whose 4-component dot product with the previous stored frame's floats (computed in double) is negative is negated before quantising; the first frame is stored as it is. A reader needs no such rule.

### Blocks

The frames of an entity are cut into blocks of `block frames` frames (the last one shorter). A block is coded alone, so a reader can start at any block. Inside a block, frame `j` is predicted from the two before it by integer arithmetic modulo 2^64:

```
pred(0) = 0
pred(1) = v[0]
pred(j) = 2 * v[j-1] - v[j-2]        (j >= 2)
```

and only `residual = v[j] - pred(j)` is stored. A block is a sequence of records until its frames are used up:

| first byte | meaning |
|---|---|
| `0` | a zero run: a varint `n` (1 to the frames left in the block) follows; the next `n` frames have all residuals 0 (they equal their predictions) |
| `1` | one frame follows: its residuals as zigzag varints, for the first group of the vector |
| `2` | (bodies only) one frame: residuals of the second group only |
| `3` | (bodies only) one frame: the first group's residuals, then the second's |

For a body the first group is the position (3 or 2 numbers), the second the rotation (4 or 2). Residuals of a group a tag leaves out are 0. For point tracks the vector is one group, and only tags 0 and 1 occur. A frame whose residuals are all 0 must be written as part of a zero run (a writer never writes tag 1/2/3 with all zero residuals, but a reader accepts it). The block must end exactly at its last frame: leftover bytes are an error.

A resting body costs two bytes per zero run however long; a body moving steadily costs few bytes because constant velocity predicts exactly.

### Sampling (reader behaviour)

The time `t` (seconds from frame 0) is converted to `f = t / (stride * dt)`; an `f` within 1e-9 of an integer is snapped to it. An entity exists at `f` when `first <= f <= last`; between two frames both must exist. At an integer frame the stored value is returned as decoded: positions `q * 2^exponent` (exact), rotation components `q * 2^-b` rounded to float32 (or the stored float). Between frames `i` and `i+1` with `a = f - i`:

- position: `p0 + (p1 - p0) * a` in double; vertices and particles likewise per point;
- rotation 3D: `m = r0 * (1 - a) + s * r1 * a` where `s` is -1 when `dot(r0, r1) < 0` and +1 otherwise, componentwise in double, normalised by dividing by its Euclidean length (`sqrt`); a zero `m` returns `r0`. This is normalised linear interpolation (nlerp) along the shorter arc; its error against slerp (constant angular velocity) is at most about `Theta^3 / 248` radians of rotation for a turn `Theta` between two frames (7e-8 rad for 90 degrees a second at 60 Hz; 0.016 rad for a quarter turn in one frame). 2D: the same on `(cos, sin)`.

## Fracture chunk (`FRAC`)

```
varint record count
record*:
  varint step         steps since the bake began (the first step is 1)
  u8 kind             0 impact, 1 manual, 2 bond break, 3 slice
  varint source slot, varint source generation     (the body that broke)
  f32 impulse, f64 x, f64 y, f64 z                 (z is 0 in 2D)
  varint fragment count, then (varint slot, varint generation) per fragment
varint geometry count
geometry*:
  varint slot, varint generation                   (a fragment body)
  varint vertex count, then vertex count * dimension f32   (body frame)
  varint triangle count, then 3 * triangle count varints   (3D; 0 in 2D)
```

The fragments' tracks are in `RIGD` with the ids the record names; a fragment's first frame is the first frame at or after its step, `ceil(step / stride)`. The source body's track ends before it. Geometry is stored once per fragment, in the fragment body's own frame: for a hull or box, the corners and the triangles of its surface; in 2D the first polygon's vertices in order. Only fractures that happen inside `Step` are recorded.

## Event chunk (`EVNT`)

```
varint event count
event*: varint step, u8 kind, varint slotA+1, varint genA, varint slotB+1, varint genB, u32 value
```

`kind` is the world's `EventKind` (0 to 8); a body slot is stored plus one, 0 meaning none (a sleep event has no second body). `value` is the event's float32 value (non-finite values are written as 0). Events are those the world collected in each step, in its own order, filtered by the bake's event mask (default: everything but contact persistence); they are recorded for every step whatever the frame stride.

## Writer rules

- Frames are taken after a `Step` when `(steps since the start) % stride = 0`; the writer reads the world and changes nothing in it.
- The dt of every step must equal the options' `Dt`, and the world's step count must advance by one per step; otherwise the bake is faulted and `Finish` throws (a restore or fork during a bake does this).
- Entity order, block cut points and every integer are fixed by this page; nothing depends on a hash or a dictionary order. A soft body or rod appears in slot order; fluid particles are sorted by id before they are coded.
- A bake larger than 2 GiB cannot be written. A body with more than 2^24 / (dimension × block frames) points needs a smaller block size (the writer faults and says so).

## Limits and errors

| field | limit |
|---|---|
| frames | 1 to 2^24 |
| points per soft body or rod | 1 to 2^24, and `points × dimension × block frames` at most 2^24 |
| block frames | 1 to 4096 |
| strings | 1024 bytes |
| file | below 2 GiB |

A reader raises `BakeFormatException` (field, byte offset, problem) for a short file, a bad magic, version or checksum, a count above the limit or the bytes left, an index out of range, a duplicate chunk, a missing `END `, bytes after it, a bad tag in a block, a zero run past its block, a non-finite float where one is required, or leftover bytes in a block. Structure is checked when the file is opened; the numbers of a block when it is first sampled.

## Defaults and the sizes they give

| option | default | error |
|---|---|---|
| `PositionExponent` | -16 (15.3 µm) | at most 7.6 µm |
| `VertexExponent` | -14 (61 µm) | at most 31 µm |
| `RotationBits` | 16 | at most 7.6e-6 per component (below 0.001 degrees) |
| `BlockFrames` | 64 | |
| `FrameStride` | 1 | |

Indicative sizes (x64, default options, 60 Hz): a 1000-body pile for 600 frames, rigid track alone, 5.08 MB (8.5 bytes per body per frame; the raw float32 pose is 28, the double position and float quaternion 40); a 45 × 45 cotton drape (2025 particles) for 600 frames, soft track alone, 3.78 MB (3.1 bytes per particle per frame; raw three doubles are 24). The writer takes about 3 ms per frame for the 1000 bodies and 0.5 ms for the 2025 particles; the step itself is far larger. A stride of 2 halves the size and `PositionExponent = -12` or `VertexExponent = -10` saves about a byte per moving number.

## The offline quality preset

`world.ApplyOfflineQuality(options)` (both worlds) raises settings for a world that is baked, not played. Every number is a floor; a larger value the world already has stays. The changes are ordinary option and body changes (recorded by a replay, saved by a snapshot).

| setting | target (default) | what it does |
|---|---|---|
| `Substeps` | 8 (world 4) | rigid substeps per step: shorter integration interval, less drift on stacks, joints and fast bodies |
| `VelocityIterations`, `RelaxIterations` | 2, 2 (world 1, 1) | solver passes per substep, also for the contact-only schedule |
| `ContactSubsteps` | 4 (3D world 2; 2D stays off) | substeps of islands without joints |
| `SoftSubsteps`, `SoftIterations` | 8, 6 (world 4, 3) | soft body, cloth and strip solve |
| `FluidSubsteps`, `FluidIterations` | 4, 8 (world 2, 4) | particle fluid steps and density iterations |
| `DisableSleep` | true | nothing freezes mid-motion; no late wake |
| `Continuous` | true | continuous collision on |
| `SoftSurfaceCollision` | true | turns surface collision on for every existing soft body (3D default off) |

Measured, 3D, x64, Release, indicative (a shared host): a 10-box stack of 1 m boxes with a 10 × heavier top box, 600 steps: lateral sway of the top 0.29 m with the defaults against 0.018 m with the preset (max penetration 1.02e-2 against 0.99e-2: the contact slop, unchanged; step 5.6 against 10.6 ms). A 45 × 45 cotton cloth thrown over a sphere, 300 steps: the largest edge stretch 6.55 % against 5.26 %; 6.2 s against 9.0 s. The preset does not iterate "to a tolerance": the world's iteration counts come from its options by the determinism rule, so the budget is the count; `Settle` is the way to run to a residual.
