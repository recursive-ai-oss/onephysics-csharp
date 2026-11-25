# Presentation (follower) worlds

A follower world holds cosmetics beside an authoritative world: debris, character cloth and hair, ropes, particles, secondary ragdoll motion. It is an ordinary world (`PhysicsWorld2D` or `PhysicsWorld3D`, the same kernels) plus a driver, `PresentationWorld2D` / `PresentationWorld3D`. The authoritative world is only read. The follower is not saved in the authoritative snapshot, is not in its hash and has no network form. This page is normative for the driver, its recording and its budgets; the world underneath is specified by the other pages.

## What the driver keeps

The follower's state is its world. The driver adds three things that are not world state: the list of mirrors (which authoritative body each follower body copies), the creation order of cosmetic bodies (for culling), and the settings below. None of them is in any snapshot. A recording (below) does not need them: it records what the driver did to the world.

## Mirrors

`Follow(body)` creates a body in the follower at the authoritative body's origin and rotation: **static** when the authoritative body is static, **kinematic** otherwise. Every non-sensor shape is copied with its geometry (the recipe bytes in 3D, the baked `Shape2D` in 2D), local pose, density, friction, restitution, rolling resistance, category, mask, group and layer; surface properties and aerodynamic surfaces are not copied. A shape or body edited in place after the copy (voxel edits, carving) is not tracked. Following a followed body returns its mirror. Mirrors are kept in the order they were followed.

Per `Observe(authority)`:

1. The pose (origin, rotation) of every followed non-static body of that authority is read. A position beyond ±2^21 is refused (`ArgumentOutOfRangeException`, nothing kept).
2. The authority's fracture events are handled in event order (below), then mirrors whose authoritative body no longer exists, or whose follower body was destroyed, are removed (the follower body is destroyed with them).
3. The poses are stored as the next frame's targets, then the mapped events are delivered.

## One step

`Step(dt)` does, in this order:

1. **Maintenance:** mirrors whose follower body is gone are dropped; the quality settings are applied; bodies are culled to the budget (below).
2. When recording: if the world hash or the live-options block differs from what the last step left, a snapshot record is written (everything the host or the maintenance changed since).
3. **Frame:** a frame is built from the stored targets, in mirror order. For each mirror the move is a **jump** when the origin would travel farther than `min(TeleportSpeed, MaxLinearSpeed/2) * dt` (3D and 2D) or the rotation would turn by more than `TeleportSpin * dt` (3D: `2*|vector part of target * conj(current)|`; 2D: the angle difference wrapped to ±π); otherwise it is a drive.
4. **Apply** (the one routine both a live step and playback run, on the frame's bytes): a drive is `MoveKinematic(pose)`, which gives the mirror the velocity that reaches the pose this step; a jump is `SetTransform(pose)` with zero linear and angular velocity. Then the world steps with callbacks suppressed. Then **the speed bound**: every dynamic body whose speed exceeds `MaxSpeed` is scaled to it along its direction (`s = MaxSpeed / sqrt(|v|²)` in float32, once more if rounding left it above), and a non-finite speed becomes zero.
5. The frame and the world hash after the bound are written when recording.

Defaults: `MaxSpeed` 30 m/s, `TeleportSpeed` 30 m/s (at most `MaxLinearSpeed`), `TeleportSpin` 30 rad/s. The defaults of `TeleportSpeed` and `MaxSpeed` are equal so that a driven mirror never pushes anything faster than the bound.

## Rollback

The follower has no notion of rollback: it is given poses. When the authority is restored and re-simulated, the host keeps calling `Observe` and `Step`. A pose that moves a mirror by more than the jump limit teleports it with zero velocity (nothing resting on it is thrown); a smaller pose change is driven, so cosmetics touching a mirror whose body moved up to `TeleportSpeed * dt` in one step feel that speed; whatever a jumped mirror now overlaps is pushed out at the world's depenetration speed; and after every step no dynamic body is faster than `MaxSpeed`. The bound holds for any jump; the test rolls the authoritative scene back by 120 steps three times with cosmetic bodies at rest around the mirrors.

## Events

- **Fracture events** of the authority, in order, unless `FractureMapper` is set (then the mapper alone decides). Default: the mean velocity of the live fragments is taken; if the source was followed its mirror is removed and, with `FollowFragments`, the live non-static fragments are followed (in fragment order); then `SpawnShards(point, meanVelocity, eventIndex)`.
- **`SpawnShards`:** `Count` boxes (3D) / squares (2D) of edge `Size` and density `Density`. The generator is SplitMix64 seeded with `Seed XOR (stepCount * 0x9E3779B97F4A7C15) XOR (key * 0xD1B54A32D192ED03)` (64-bit wrapping). Per shard: up to 16 draws of a point in the cube/square [-1, 1]^D from three/two `NextDouble()` values (`(bits >> 11) * 2^-53`), the first inside the unit ball/disc is kept (the origin if none); the launch direction is that point normalized (up if its length is below 1e-6); the speed is `Speed * (0.5 + NextDouble())`; the position is `at + point * Radius`; the velocity is `baseVelocity + direction * speed` in float32. Shards are created in order and numbered for culling at creation.
- **Other events:** every event of `authority.Events` whose kind bit (`1 << kind`) is in `MappedEvents` is passed to `EventMapper` in the authority's order.

## Budgets (`PresentationQuality`)

Applied in the maintenance phase of every step. The values are those of the quality at that time.

- **`MaxBodies`:** the dynamic bodies of the follower world, except pinned ones, are counted. When more than the budget, the excess is destroyed in this order: **lowest mass first, then oldest, then lowest slot**. A body is numbered when it is created by the driver, or when the driver first sees it (bodies the host added, in slot order); a reused slot is a new body. The order is total, so every platform removes the same bodies. Joints and attached rods of a destroyed body go with it, as for `Body.Destroy`.
- **`MaxParticles`:** when the follower holds particle fluids, `FluidSettings.MaxParticles` is set to the budget (emitters stop there) and, if there are more particles, the **oldest** (lowest id; ids grow with creation) are removed, the others keeping their order.
- **`SoftSubsteps`, `SoftIterations`:** when not 0, set on the world's options (soft bodies and cloth).
- **`RodSubsteps`, `RodIterations`:** when not 0, no rod runs more than that; a rod keeps what it was created with when that is smaller, and gets it back when the cap is raised or removed.
- Raising a budget changes nothing that exists; lowering never destroys more than the excess.

## Recording

A recording replays a follower without the authority. Little-endian throughout, no padding.

| Offset | Type | Value |
|---|---|---|
| 0 | uint32 | `0x4650504F`, ASCII `OPPF` |
| 4 | uint16 | version, 1 |
| 6 | uint8 | dimension, 2 or 3 |
| 7 | uint8 | flags, 0 |
| 8 | int64 | the follower world's completed step count when recording started, nonnegative |

Records follow: kind (uint8), payload length (int32, nonnegative), payload.

| Kind | Payload |
|---|---|
| 0 End | empty; required; stream bytes after it are not read |
| 1 Snapshot | int32 options length (the live-options block size of the dimension: the replay v2 `LiveOptions` layout), the options block, the world snapshot (format v1) to the end of the payload |
| 2 Frame | float32 dt, float32 speed bound, int32 pose count, the poses, uint64 world hash after the step |

The first record is a snapshot. Snapshots may appear before any frame; they restore the world and apply the live options, exactly as replay v2's mutation records do, and are written when the world hash or the live options differ from what the last step left. A pose is: int32 body slot, int32 generation, uint8 flags (bit 0: jump; others 0), then the position (3D: three float64; 2D: two) and the rotation (3D: four float32, x y z w; 2D: one float32 angle). The body must be alive and kinematic. A driven pose must be reachable within half the world's `MaxLinearSpeed` in the frame's dt; a jump has no such limit. Positions are finite within ±2^21, rotations finite and not zero. dt and the speed bound are positive and finite; the pose count must match the payload length exactly.

`PresentationPlayer` reads it: the header, then `CreateFollower2D/3D` (the first record, which builds a world from the recorded creation and live options), then `ReadNextStep` (snapshots applied, one frame applied and its hash compared). Errors are `ReplayFormatException` with the byte offset, and `ReplayHashMismatchException` with the step. A declared payload length above the reader's bound (default 64 MiB) is refused before any allocation; nothing recurses on input; reading is linear in the bytes. The recorded hash is the world's hash (`docs/spec/snapshots.md`), computed after the speed bound.

Callbacks are always suppressed while a follower steps, so that a recording cannot depend on host code; a host reads `World.Events` after the step and changes the world between steps (which the next recorded snapshot captures).

## Determinism

The follower is as deterministic as any world: the same inputs (poses, dt, spawns, budgets) give the same bits on every platform, vector width and thread count. The hash-matrix rows `presentation/follow-2d`, `follow-3d`, `replay-2d` and `replay-3d` hold the follower's hash per step, the recording's SHA-256 and the replayed hashes.

## Limits

Mirrors copy shapes at `Follow` time. The follower world's own limits apply (a rod, a fluid, a body count). A follower does not follow joints, characters or vehicles as such, only their bodies. 2D soft strips have no attachment to rigid bodies, so a 2D cape is pinned in the world or hung from a rod.
