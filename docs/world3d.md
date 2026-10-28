# The 3D world

Normative, language-neutral description of `src/OnePhysics/Physics3D/`: what a 3D world does in a step and which choices a port must reproduce to get the same bits. It sits on the shared core (`architecture.md`), the 3D collision code (`collision3d.md`) and the broadphase (`broadphase.md`). It mirrors `world2d.md`; this page lists what is the same in one line and spells out what differs. All arithmetic is IEEE `+ − × ÷ sqrt` on float32 (positions and broadphase boxes are double); no fused operations. The solver kernel is scalar, one constraint at a time, so lane width cannot matter.

## Bodies, shapes, mass

* Same body model as 2D: the core position is the centre of mass, the origin is `com − R·lc`, rotation is a unit quaternion `(x, y, z, w)`, `StartAsleep` puts a body to sleep when created. `BodyDef.Rotation` is normalized on creation (zero, NaN and infinite quaternions are rejected with `ArgumentException` naming the field).
* Mass data per shape and unit density comes from `collision3d.md` (`InertiaF.Combine` for several shapes): `m = Σ density·volume`, `lc` the density-weighted centroid, inertia tensor about the centre of mass stored as six floats `xx xy xz yy yz zz`. No mass at all gives mass 1, identity inertia. `FixedRotation` gives zero inverse inertia. Static and kinematic bodies have zero inverse mass and inertia. Meshes and height fields are static only (`StaticOnlyShapeException`).
* The inverse inertia in world axes `R·I⁻¹·Rᵀ` is recomputed (float, rows in the order of the product) whenever a body's rotation changes, at the start of a step and after every substep rotation.
* Per shape the box of the shape's bounds in the body frame (`ShapeBounds3D` of its pose; the local bounding box for compounds, meshes and height fields) gives: `maxExtent` = the largest over shapes of `sqrt(dx² + dy² + dz²)` with `d` the largest per-axis distance from the centre of mass to that box; `minExtent` = the smallest over shapes of half the smallest box side. A body without shapes has `maxExtent` 0 and `minExtent` `float.MaxValue`. The gyroscopic flag is set when the inertia tensor is not isotropic (off-diagonals above `1e-6·Ixx` or diagonal differences above that) and rotation is not fixed.

## Shape geometry and recipes

A shape is created from a **recipe**: a byte string that fully determines its geometry. Recipes are what snapshots store; geometry is rebuilt from them on restore. Identical recipes share one geometry object (looked up by the byte string; slots are reused lowest first). Little-endian; floats are IEEE bit patterns.

| kind byte | layout after the kind |
|---|---|
| 1 sphere | radius |
| 2 box | half extents x y z, convex radius |
| 3 capsule | half height, radius |
| 4 tapered capsule | half height, top radius, bottom radius |
| 5 cylinder | half height, radius, convex radius |
| 6 hull | convex radius, int32 point count, points (x y z each) |
| 7 compound | int32 part count; per part: rotation x y z w, position x y z, then the part's convex recipe |
| 8 mesh | double flat-edge tolerance, int32 vertex count, vertices, int32 index count, indices |
| 9 height field | double flat-edge tolerance, int32 nx, int32 nz, scale x y z, bool has-holes, nx·nz heights, then one bool per cell if has-holes |

A shape also has a placement in the body (local rotation and translation, each shape slot), a material (friction, restitution, rolling resistance), a density, filter bits (category, mask, group) and the owner body.

## Pair table

One entry per pair of shapes `(a, b)`, `a < b`, whose broadphase boxes overlap, exactly as in 2D (same proxy margins, same acceptance rules, same update rule, parallel narrowphase over fixed chunks of 64 updated pairs, serial apply in ascending pair index). Differences:

* The tight box of a shape is the shape's float bounds relative to the centre of mass, added to the double position.
* **One pair may own several constraints**, one per sub-shape pair that has points: compound children, mesh triangles and height-field triangles are sub-shapes. A pair's constraints form a chain sorted ascending by `(subShapeA, subShapeB)`; the pair's results are applied in that order, so constraint creation order, and therefore the color of each constraint, is fixed. The core reports contact events per constraint; the world collapses them to one event per shape pair (see Events).
* The pose of B in A's frame is built from the shape frames relative to each centre of mass, with the double centre-of-mass difference converted to float once. The speculative distance is `SpeculativeDistance` (default 0.02).
* **Triangle soups** (meshes, height fields): the candidate triangles for the other shape's bounds (grown by the speculative distance) come from `collision3d.md` (`GetCandidates`), each triangle is a degenerate convex shape, and each gives its own manifold. Soup against soup never collides.
* **Internal-edge rule.** For a triangle manifold whose normal differs from the triangle's face normal (`dot < 0.99999` and `≥ −0.1`): if any manifold point's closest triangle feature is an inactive edge, or a vertex with both of its edges inactive, the manifold is dropped. This keeps a body from bumping over the seams of a flat mesh. Replacing the normal by the face normal was tried and gave double contacts, so the manifold is dropped instead.
* **Manifold payload** per constraint: normal (3), mixed friction, restitution, rolling resistance, the rolling impulse (3, kept from the previous manifold), per point (up to 4) the arms, separation, normal impulse, two tangent impulses and the max normal impulse; warm starting copies impulses from the old point with the same feature id (0 if none). Layout in `architecture.md`.
* **Materials.** Friction `sqrt(fA·fB)`, restitution `max`, rolling resistance `max`; `MaterialMixer` replaces it.

## Step

The core pipeline (`architecture.md`) with these stage bodies; `h` is the substep time. Defaults: 4 substeps, 1 solve iteration and 1 relax iteration per substep, contact hertz 30, damping ratio 10, push-out velocity 3 m/s, contact slop 0, restitution threshold 1 m/s, maximum linear speed 400 m/s, maximum rotation per substep π/4, speculative distance 0.02, broadphase margin 0.1.

* **Prepare** (per active constraint): `hertz = min(ContactHertz, 0.25/h)`; soft coefficients as in 2D with `hertz`, and with `2·hertz` when either body has zero inverse mass. Tangent basis from the normal `n`: if `|nx| ≥ 0.57735` then `t = (ny, −nx, 0)` else `t = (0, nz, −ny)`; `t1 = t/|t|`, `t2 = n × t1`. Per point: effective masses `1/(mA + mB + (rA×d)·IA(rA×d) + (rB×d)·IB(rB×d))` for `d` in `n, t1, t2`; adjusted separation `s − (rB − rA)·n`; relative normal velocity; max normal impulse reset. Rolling mass is the inverse of `IA + IB` (3×3, zero when singular).
* **IntegrateVelocities** (dynamic bodies): `v += h·(gravityScale·g + invMass·force)`; `w += h·I⁻¹·torque`; **gyroscopic term** for bodies flagged as non-isotropic: one Newton step on `I(ω − ω₀) + h·ω × Iω` in the body frame with the exact Jacobian `I + h(ω̃I − (Iω)~)` (skipped when the determinant is not finite or tiny); then damping `1/(1 + h·damping)` on `v` and `w`; `|v|` clamped to the maximum speed, `|w|` to `MaxRotationPerSubstep/h`.
* **IntegratePositions**: `Δp += h·v`, `position += (double)(h·v)`; if `w ≠ 0`, `Δq` and the body rotation advance by `q += h/2·(w, 0)·q` and are renormalized with `sqrt` and a divide (if the norm is zero or not finite the old quaternion is kept); the world inverse inertia is refreshed.
* **Kernel** (per constraint; the arithmetic below is exactly one constraint's, evaluated lane by lane, see Performance). Warm start: `p = n·λn + t1·λt1 + t2·λt2` per point at the arms, then the rolling impulse as an angular impulse. Solve (with bias) and Relax (without), in this order: rolling resistance (clamped to `rolling · Σλn`, projected perpendicular to the normal), friction (both tangent impulses accumulated and clamped to the disc of radius `friction·λn` by scaling), then the normal row of each point:
  `s = ((Δp_B − Δp_A) + (Δq_B·rB − Δq_A·rA))·n + adjustedSeparation`; if `s > 0`: `bias = s/h`, mass scale 1, impulse scale 0; else when biased `bias = max(biasRate·min(s + slop, 0), −pushOut)` with the soft scales; relax uses bias 0 and scales 1, 0. `λ = −normalMass·massScale·(vn + bias) − impulseScale·λold`, accumulated and clamped at zero. Restitution (after all substeps, constraints whose restitution is 0 are skipped): for points whose pre-solve relative normal velocity is `≤ −threshold` and whose max normal impulse is positive, `λ = −normalMass·(vn + e·vrel)` accumulated and clamped.
* **Sweep order.** Within a color the order cannot matter. The sequence of colors alternates: in even substeps Solve visits overflow, then colors 0 to 23 ascending, and Relax visits colors 23 to 0 descending and the overflow last; odd substeps swap the two. Warm start and restitution are order-independent in effect and use the forward order. With one iteration per phase a fixed forward sweep leaves a systematic bias along the constraint order that makes a tall stack of boxes creep sideways (a 18-layer pyramid's top moved 0.1 m in 2 s); the alternating sweep removes it. 2D keeps the forward order.
* **FinalizeBodies**: forces and torques cleared; continuous collision.
* **BodyMotion**: `float.MaxValue` if a touching partner is a moving kinematic body, else `|v| + |w|·maxExtent`. Sleeping is the core's, with one 3D setting: an island that has lost constraints is split as soon as any of its bodies has rested for the sleep time (instead of only when all of them have), the longest-resting such island first, one split per step; the resting part then sleeps while the moving part stays awake.

## Continuous collision against static shapes

For each dynamic body after the substeps: `m` = centre-of-mass displacement over the step, `spin = 2·|xyz of q_start⁻¹·q_end|·maxExtent`. If `|m| > 0` and `|m| + spin > 0.5·minExtent` (and all finite), every non-soup shape (compound children one by one) is linearly cast along `m` at its start orientation against the static shapes whose boxes overlap the swept box (the union of the start and end bounds). A hit counts only if: the cast does not start overlapping; the body would end the step at least `0.25·minExtent` deep along the hit normal (`(1 − t)·approach ≥ 0.25·minExtent`, `approach` = `−m·n_out`); and the hit is farther than twice the speculative distance along the motion (`t·|m| ≥ 2·SpeculativeDistance`): nearer surfaces already have speculative contacts, and clamping against them pins a body that rests on or rolls over a surface. The smallest accepted fraction `t` moves the body to `t' = t − 0.005/|m|` (not below 0) along `m`, with the rotation interpolated by a normalized lerp along the shorter arc; velocity is unchanged and `Δp` is reset to the clamped displacement. Triangle soups are not cast (they are thin walls only for convex movers that the discrete contacts already handle); a fast sphere against a thin box or a mesh is stopped.

## Snapshot section

Section id `0x33445747` ("GW3D"), version 1 (version 2 adds the flagged blocks below), written by the engine after the core's sections in this order (little-endian, all through the core's state writer):

1. `uint32` version; floats `ContactHertz`, `ContactDampingRatio`, `PushOutVelocity`, `RestitutionThreshold`, `MaxLinearSpeed`, `MaxRotationPerSubstep`; bool `ContinuousEnabled`; floats `ContactSlop`, `SpeculativeDistance`.
2. `int32` body slot count `nb`, then arrays over `nb`: local centre (3), mass, inertia (6), maxExtent, minExtent, linear damping, angular damping, gravity scale, force (3), torque (3), flags (bytes), first shape, last shape, `Δp` (3).
3. `int32` geometry count; per slot a bool (present) and, if present, `int32` length and the recipe bytes.
4. `int32` shape slot count; per slot a bool (live) and, if live: geometry index, local rotation (4), local translation (3), proxy id, friction, restitution, rolling resistance, density, category, mask, group, next shape of the body.
5. `int32` pair count; per pair a bool (live) and, if live: shape a, shape b, `int32` constraint count and the constraint ids in chain order. Then `int32` free pair count and the free list.

The reader checks every count against the bytes that remain before allocating, rejects non-finite values, non-unit rotations (squared length outside `[0.999, 1.001]`), invalid materials and references to missing geometry, rebuilds geometry from the recipes (a recipe that fails validation rejects the snapshot), cross-checks the section against the core (`IsConsistentWithCore`: proxies, shape chains, unit body rotations, constraint chains sorted by sub-shape pair), and restores atomically: any failure leaves the world as it was. The world hash covers the core state in `architecture.md`'s order plus nothing from this section that the core does not already hold (impulses live in the payloads); the origin is part of the hash.

## Origin

`ShiftOrigin(offset)` subtracts the offset from every body position and every broadphase box and adds it to the accumulated origin; components must be multiples of 2^-31 (`ArgumentException` otherwise). Positions are checked against ±2^21 (inclusive of the shifted origin) at creation and at the end of each step; a body outside, or with a non-finite position, raises `BodyOutOfBoundsException` naming it, and the step has no effect on the world's state (the hash is unchanged); destroying the body lets stepping continue.

## Joints

Twelve joint types (`JointType3D`): fixed, point, distance, hinge, slider, cone, swing-twist, six-degrees-of-freedom, path, gear, rack-and-pinion, pulley. A joint is a constraint of the same storage, colouring and batching as a contact: it takes colours like a contact between its two bodies, is solved in the same colour sweeps, and a static or kinematic body takes none. Both ends are bodies of this world; at least one must be dynamic, and a joint between a body and itself is refused (`InvalidJointException`, as for any value a type does not allow: non-finite, out of range, a zero-length axis, a non-unit frame rotation).

**Frames.** Each end has a frame (a point and a unit quaternion in the body's origin frame). The frame's x axis is the main axis: hinge axis, slider axis, cone and twist axis, path tangent. The happy-path helpers take a world anchor and axis and build frames from the axis and a perpendicular chosen by a fixed rule (`PathSpline3D.Perpendicular`: the component pair of the unit axis swapped and negated, depending on whether |x| is at least 0.57735), so the same call gives the same frames on every platform.

**Rows.** Every type is a set of scalar rows `Cdot = LA·vA + AA·wA + LB·vB + AB·wB` (`JointRow`). Equality rows of up to three are solved as one 3×3 (or 2×2, 1×1) block with the exact inverse of the small system, so coupled rows (a point constraint, a hinge's two perpendicular rotations) do not fight. Limits are one-sided rows: speculative while the limit is not reached (the allowed velocity closes the gap in one substep), soft when violated; accumulated impulses stay at least zero. Motors drive `Cdot` to the target with the accumulated impulse clamped to ±force·h. User springs use the soft coefficients of frequency and damping ratio on their row. Geometry (anchors, frames, axes) is rebuilt at every solve call from the start-of-step pose plus the displacement accumulated since, as the contact solver does, so substeps and relax passes see current directions.

**Soft step.** Rigid rows use the soft-step coefficients (Catto, "Solver2D" and the soft-step talk): `WorldOptions3D.JointHertz` (default 60) and `JointDampingRatio` (default 2), with the frequency capped at a quarter of the substep rate. The relax pass uses no bias.

**Rows per type** (payload slots in the constraint's float payload, 64 floats in 3D, so no type is limited by row count; the largest, six-dof, uses 30):

| Type | Rigid rows | Optional rows |
|---|---|---|
| Fixed | point block (slots 0–2), angular block (3–5) | |
| Point | point block (0–2) | |
| Distance | one row along the anchor-to-anchor direction (0) | spring (10), lower and upper length limits (12, 13) |
| Hinge | point block (0–2), the two rotations perpendicular to the axis (3–4) | spring (10), motor (11), angle limits (12, 13) |
| Slider | two translations perpendicular to the axis (0–1), rotation lock (3–5) | spring (10), motor (11), limits (12, 13) |
| Cone | point block (0–2) | cone limit (12): the angle between the two x axes at most the half angle, about their cross product |
| Swing-twist | point block (0–2) | twist limits (12, 13), swing limit (14) |
| Six-dof | locked translations as a compact block (0–2), locked rotations as a compact block (3–5) | per axis i: motor (6+4i), spring (7+4i), upper (8+4i), lower (9+4i) |
| Path | two translations perpendicular to the tangent (0–1), orientation per `PathRotation` (3–5) | motor along the tangent (11), end clamps (12, 13) |
| Gear | one velocity row `wA·axisA + ratio·wB·axisB = 0` (0) | |
| Rack and pinion | one velocity row `wA·axisA = ratio·vB·axisB` (0) | |
| Pulley | one row on `lengthA + ratio·lengthB` (0) | range limits instead of the equality (12, 13) |

**Swing-twist limits.** With the relative rotation `q = FA⁻¹·FB` (sign chosen so `w ≥ 0`), twist is `2·atan2(q.x, q.w)` (onemath's atan2) and `swing = q · conj(twist)`. The twist row is the exact gradient of the twist angle with respect to the relative angular velocity, `(1, (wz + xy)/(x² + w²), (xz − wy)/(x² + w²))` in frame A (from `q' = ½ ω q`); a row about frame A's x axis alone stops enforcing the limit once the swing is large, because the twist axis is then the swung one. The swing limit is the ellipse `(y/sin(hy/2))² + (z/sin(hz/2))² ≤ 1` on the swing quaternion's components (hy, hz the half angles); the row is `−L·∇ρ`, the gradient of the ellipse function ρ scaled by the swing angle `L` at which the current direction meets the ellipse, again from `q' = ½ ω q`, and the slack is `(1 − ρ)·L`. Near `x² + w² < 1e-4` (swing close to a half turn) the twist row falls back to frame A's x axis and the swing row is skipped. Limit angles are at most π.

**Cone** uses the true angle between the axes (onemath's atan2 of the cross and dot products), and about the cross product (frame A's y axis when the axes are parallel or opposed).

**Six-dof.** Each axis is `Locked`, `Limited` or `Free`; the three translations are along frame A's axes, the rotations are the twist about x and two swing components (twice the swing quaternion's y and z) which are limited independently; this is exact for small swings and an approximation for large ones. A free axis with a spring or motor gets those rows only. `Joint.SetAxisMode` changes the mode and clears the joint's accumulated impulses.

**Gear and rack-and-pinion** constrain velocities only: there is no drift correction on the angle or position, so the relation holds in rate and may wander by integration error over a long run. They name no other joint.

**Path** follows a uniform Catmull-Rom spline through at least 2 points (refused otherwise, and for non-finite points), given in world coordinates at creation and stored relative to the path body; `Loop` closes it. The follower's attachment is projected onto the nearest point of the curve with a coarse scan of 8 samples per segment (the lowest parameter wins a tie) and a bisection inside the best interval; the frame there is built from the tangent and `Up`.

**Pulley.** The fixed points are doubles in world coordinates and move with `ShiftOrigin`. With `MinLength`/`MaxLength` the equality becomes a range.

**Breaking.** `BreakForce` and `BreakTorque` (default infinity) compare against the reaction of the last substep (its accumulated impulse divided by the substep length); a joint that exceeds either is flagged during the step, destroyed after it, and reports one `JointBreak` event with the joint's handle, both bodies and the value, in the event order of the shared core (after contact and sensor events, before sleep and wake).

**Collision between the connected bodies** is off unless `CollideConnected`.

**Islands.** A joint links its two bodies in the island graph like a contact, so a sleeping island holds its joints and a joint wakes both bodies. Destroying a body destroys its joints (their `Joint` handles go stale; a stale handle throws `StaleHandleException` and a default or foreign one `InvalidHandleException`, each naming it).

**Snapshot.** The 3D section becomes version 2 when any joint or any non-default service state exists: after the version-1 body data comes a block flags word (bit 0 = joints, bit 1 = services; never 0), then the flagged blocks in that order. The joint block is `JointHertz` and `JointDampingRatio`, the joint slot count, and per slot a bool (live) and, if live, the joint record (type, flags, body-local frames, limits, springs, motors, break thresholds, the six-dof axis arrays, path points as a count and a list, pulley fixed points as doubles, the constraint payload with accumulated impulses). The reader accepts versions 1 and 2, checks every count against the remaining bytes, rejects non-finite values and values `JointData3D.Validate` refuses, and rebinds joints to constraints after the contacts section is read; a version-1 restore removes all joints. A world with neither joints nor service state writes exactly the version-1 bytes. The services block is described under Snapshot version 2: services. The world hash is the core's: the constraint payloads hold the accumulated joint impulses; joint definitions are not hashed separately.

**Known limits.** There is no swing-twist motor (powered ragdolls drive limbs through six-dof axis motors). Reaction force and torque are those of the last step in which the joint was solved and are not state: they are not in the snapshot.

## Layers and filters

The decision order is the 2D one (`world2d.md`, Layers and filters): sensor-sensor never collides; the 32 x 32 layer matrix (`SetLayerCollision`, `IgnoreLayerCollision`, default all ones, setting a pair sets both rows); the group index; category and mask; then the optional `WorldOptions3D.PairFilter` delegate and the optional struct `IContactFilter` (`SetContactFilter`), each called serially with the shapes in slot order. Re-filtering uses the same filter-dirty flag, with the same effects: pairs no longer allowed lose every constraint of their chain (contact end events, islands woken), and every proxy is touched so newly allowed pairs enter in sorted order. `Body.Layer` sets the layer of all the body's shapes. Delegates and filters are not saved.

## Sensors

A shape created with `ShapeDef.IsSensor` contributes no mass or inertia, produces no contact response and is skipped by continuous collision. A pair with a sensor owns **one** sensor constraint (a chain of length 1, never solved, colored or linked into islands) while any manifold point of any of its sub-shape pairs has separation at most 0; it is added when the overlap begins (sensor enter) and removed when it ends, when the boxes stop overlapping, or when a shape or body is destroyed (sensor exit, delivered at the end of the next step). A sensor pair is evaluated when either body is awake and moving, and once in the step in which it is first found, so a sleeping body already inside a sensor gets its enter event. Sleeping and waking never end an overlap.

## Events

Kinds, their order and `ContactHit` are as in 2D. Differences in 3D:

* **One event per shape pair.** The core's contact events are per sub-shape constraint; the world builds the public list from them at the end of every step and restore. A pair gets a begin event if it had no contact constraint at the start of the step and has one now, an end event for the reverse, at most one persist, at most one hit (the largest approach speed over its constraints). A body over 40 mesh triangles produces one begin, not 40. The sub-shape ids of the reported event are those of the first sub-shape constraint of the chain. Everything is computed from chain lengths, so the list does not depend on the thread count.
* `Collision` carries 0 to 4 points (`PointCount`, `GetPoint`): the points of the pair's first sub-shape contact, world position of the anchor on A's centre of mass plus the arm, with `Separation`, the normal from A to B and `NormalImpulse` / `TangentImpulse` equal to the last substep's accumulated impulse times the number of substeps. `ApproachSpeed` is the hit speed. A contact that has ended has no points.
* Handlers are `OnCollisionEnter/Stay/Exit/Hit`, `OnTriggerEnter/Exit`, `OnSleep`, `OnWake`, fired after the step first on body A then on body B. Polled: `Events`, `GetEvents(kind)`, `GetCollision(event)`, `GetTrigger(event)`.

## Queries

The 2D rules (`world2d.md`, Queries) apply with 3D shapes: `Raycast`, `ShapeCast`, `SphereCast`, `BoxCast`, `CapsuleCast`, `OverlapShape`, `OverlapPoint`, `OverlapSphere`, `OverlapBox`, `OverlapCapsule` and `ClosestPoint`, each as a single (closest), `Any`, and `All` (caller span or new array) form, and `PhysicsQuery3D` for batches. Positions are double within +-2^22; directions, lengths and sizes are float; a ray or cast length of infinity means `MaxQueryDistance` (2^23). Rays and casts sort by distance (ties by shape slot), overlaps by shape slot, `OverlapShape` (single) returns the shape whose surface is nearest the query shape's origin, closest point sorts by distance. `Fraction` is the fraction of the cast length; `Distance` the distance travelled (overlap and closest point: from the query centre to the surface, 0 inside). `SubShapeId` is the compound child or mesh or height-field triangle (the id contact events use), 0 for a convex shape, -1 for overlaps. A ray starting inside a convex shape hits it at distance 0 with a zero normal; a cast starting overlapping reports a zero normal. Casts take a convex query shape (compound query shapes are not supported). A segment on a compound is clipped to half the compound's coordinate range before the compound sees it (the compound refuses longer reach), and fractions are mapped back. The candidate set comes from one broadphase tree per body type, padded by the largest gap between a shape's tight and stored box; the narrow phase works in each shape's own frame after one double subtraction. Per-thread scratch makes queries with caller buffers allocation-free in steady state; the triangle-soup cast reuses one thread-static scratch triangle. Errors are as in 2D (`ArgumentOutOfRangeException` / `ArgumentException` naming the argument, nothing changed). Batches run in fixed chunks of 64 queries on the world's executor and do not depend on the thread count.

## Debug geometry

`GetDebugGeometry(content, buffer)` as in 2D, with these 3D entries (flags: Shapes 1, Bounds 2, BroadphaseBounds 4, Contacts 8, Bodies 16, Joints 32, Characters 64, MeshTriangles 128; `All` is 255). Shapes by slot, a compound as one entry per child. Sphere: kind Circle, one point (centre) and the radius. Box: 8 corner points (bit 0 = +x, bit 1 = +y, bit 2 = +z of the core, rounding radius separate) and 12 triangles in `Indices` (`FirstIndex`, `IndexCount`). Capsule: the two axis ends and the radius. Cylinder and tapered capsule: bottom and top axis centres, `Radius` (top for the tapered capsule) and `BottomRadius`. Hull: its vertices and the faces fanned from each face's first vertex into triangles. Mesh and height field: the box corners, and with `MeshTriangles` the vertices and triangles. A joint entry adds the joint type, the reference axis in world coordinates, the limits and the cone half angle where the type has them. Points are `origin + (double)(R * v)`.

## Snapshot version 2: services

Block flag bit 1. Layout: the 32 layer-matrix rows (`uint32` each), `float32` `HitSpeedThreshold` (finite, at least 0), a bool filter-dirty flag, then for every shape slot in slot order a `uint8` shape flags (bit 0 = sensor) and a `uint8` layer (below 32); both are 0 on a dead slot, and the reader rejects any other value. The reader checks the flags and layers before committing, and the restore stays atomic.

## Determinism check

`tools/HashMatrix/Scenarios/World3D/World3DScenarios.cs` registers `world3d/pyramid-10` (300 steps), `world3d/pile` (100), `world3d/terrain` (100, bodies landing on a triangle mesh) `world3d/restore-and-shift` (snapshot at step 30, restore at 45, origin shift at 60) and `world3d/joints` (one joint of every kind swinging over a floor, 120 steps, snapshot at 40 and restore at 60) and `world3d/services` (layers changed at runtime, a sensor volume, spheres rolling over a mesh, boxes falling through the sensor, 120 steps, a snapshot restored into a fresh world at step 60; the hash covers every event and every query result of every step; the scene is `tests/Common/Services3DScene.cs`, shared with both smoke apps), each writing `ComputeHash()` after every step; the runner compares them across threads, vector widths, JIT settings, NativeAOT and WebAssembly.


## Performance

**Contact kernel.** Contacts are solved by one kernel written over lanes (`ILane<T>`, shared with 2D): the scalar twin (1 lane) and 128-bit (4) and 256-bit (8) vectors. A color batch (at most eight constraints, no dynamic body in common) is one group; `PrepareBatch` writes the group once per step in field-major rows (field `f` of lane `l` at `f·8 + l`; layout `W3D` in `ContactKernel3D.cs`): normal and tangents, material, inverse masses, soft coefficients, rolling mass, four point slots of arms, adjusted separation, normal, tangent-1 and tangent-2 masses, relative velocity, and the impulses that change while solving. The impulses live in the group during the step and `StoreResults` writes them back to the payloads. Each phase gathers the two bodies of every lane (velocity, angular velocity, accumulated translation and rotation, world inverse inertia), runs the arithmetic of the Step section for each lane with the same operations in the same order at every width, and scatters velocities of dynamic bodies. A lane with fewer points than the group's largest masks the unused slots with selects (a masked lane keeps its bits); slots past the group's largest point count are skipped. Joints in a batch solve one by one beside the kernel's lanes (they share no dynamic body with the batch's contacts); contacts take the first lanes of the group. The overflow set (which may share bodies, joints included) runs in list order, contacts one lane at a time. Square root is the IEEE operation, so every width gives the scalar answer; the world hash is the same at widths 1, 4 and 8 and equals the hash of the scalar solver that preceded the kernel. `Engine3D.KernelWidth` (default 8) only picks the width.

**Bounding spheres.** A polyhedron pair whose shape-origin spheres (core extent plus radius) are farther apart than the speculative distance returns an empty manifold before any axis test.

**Islands.** A freed island keeps its body list for the next one and the split uses reusable scratch, so merges and splits allocate nothing once warm.

**Measured** (local, one machine shared with other work, load 12 to 20 during the runs; median of three interleaved runs of the step time, ms per step; before = the world before these changes, after = now):

| scene | threads | before | after | speedup |
|---|---|---|---|---|
| large-pile-3d | 1 | 65.7 | 42.0 | 1.56 |
| large-pile-3d | 8 | 14.9 | 11.7 | 1.27 |
| mesh-terrain-3d | 1 | 22.7 | 9.5 | 2.40 |
| mesh-terrain-3d | 8 | 6.2 | 3.2 | 1.91 |
| heightfield-terrain-3d | 1 | 24.6 | 8.7 | 2.84 |
| heightfield-terrain-3d | 8 | 6.4 | 3.3 | 1.95 |
| sleep-wake-3d | 1 | 13.6 | 9.1 | 1.50 |
| sleep-wake-3d | 8 | 5.3 | 4.0 | 1.32 |
| queries-3d (settle) | 1 | 59.7 | 39.7 | 1.50 |
| queries-3d (settle) | 8 | 13.7 | 11.6 | 1.19 |

Sampled profile of large-pile-3d at one thread, before: solver 48 %, contacts 44 % (narrowphase 35 %, of which generic polyhedron pair 21 %). After: kernel about 31 %, narrowphase about 36 % (edge-pair axis test 11 %, polyhedron loading 5 %, GJK for separated cores 5 %, face axis test 4 %). Terrain after: kernel about 27 %, polyhedron loading 5 %, GJK 4 %, edge axes 3 %.

**Allocation.** After warm-up a step allocates nothing on large-pile-3d and, apart from rare growth of reusable buffers, nothing on mesh-terrain-3d and heightfield-terrain-3d (median 0 bytes per step; it was 9.8 KB and 7 KB). The garbage came from handle checks that built their message text before testing (`Resolve`), a fresh options object per option check, `Span.Sort`/`Array.Sort` with a comparer (a delegate per call; pairs and events now use `Core/Sorting.cs`, whose orders are total so the result is unchanged), and the list a continuous-collision cast against a mesh allocated per call (now one per thread). Island body lists that outgrow their island go to a small spare pool that the next growth draws from. What is left is growth of narrowphase result buffers (one per 64-pair chunk, doubling) and island lists when a bigger contact set than before appears.

**Edge pairs.** The per-edge data of the second hull (negated face normals, their cross product, first vertex, direction and length) is computed once per query instead of once per edge pair; the arithmetic is the same operations, so the result is bit for bit the same. It is a small gain (EdgeQuery 11.6 % to 10.6 % of large-pile-3d at one thread); a dedicated box-box test was not written because the Gauss-map pruning already keeps only the pairs that can form a face of the Minkowski difference, and the edge pair chosen by it, not only the axis, decides the contact.

**Sleeping on terrain.** The 3D world splits an island as soon as any of its bodies has rested for the sleep time (Step section). Awake bodies on mesh-terrain-3d at steps 120, 240, 360, 480, 600: before 1295, 1149, 1144, 1169, 1176 (all but a few of them motionless but joined to a rolling body); after 1294, 926, 753, 577, 507. Reference engines on the same scene (black-box dumps, awake flag): Jolt 1251, 615, 476, 235, 149; Box3D 1288, 859, 685, 571, 484; Rapier3D 1297, 888, 686, 571, 479. Rolling spheres and capsules have no rolling resistance by default here, as in the references' documented defaults (Jolt documents linear and angular damping of 0.05 per second and no rolling resistance; Rapier documents no damping by default), so rolling bodies stay awake on slopes until they reach a basin.

**Remaining gap** at one thread against the fastest reference (ms): large-pile-3d 42.0 vs 5.2 (Rapier3D), mesh-terrain-3d 9.5 vs 0.91 (Jolt), heightfield-terrain-3d 8.7 vs 0.89 (Jolt), sleep-wake-3d 9.1 vs 1.51 (Rapier3D), queries-3d settle 39.7 vs 5.7 (Rapier3D). Causes: the gather and scatter of body state per lane is scalar (nineteen floats per body per phase from five arrays; a packed per-step body row would turn it into one row load); the polyhedron axis test runs all edge pairs for every touching pair (a cached separating feature per pair does not help touching pairs, only an incremental manifold would); the group's lanes are masked for the unused point slots rather than sorted by point count; steady-state allocation is not zero on soup scenes (about 6.7 KB per step in pair churn and 2.8 KB in continuous collision's mesh casts on mesh-terrain-3d; 0.7 KB on large-pile-3d). Dedicated box-box code and the per-pair caching of the separating feature were not done. None of this is determinism-bound: no row is blocked by determinism rules.
