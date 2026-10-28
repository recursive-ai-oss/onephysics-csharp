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
* **One pair may own several constraints**, one per sub-shape pair that has points: compound children, mesh triangles and height-field triangles are sub-shapes. A pair's constraints form a chain sorted ascending by `(subShapeA, subShapeB)`; the pair's results are applied in that order, so constraint creation order, and therefore the color of each constraint, is fixed. Contact begin and end events are per constraint (a body over N triangles gives N events).
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

Section id `0x33445747` ("GW3D"), version 1, written by the engine after the core's sections in this order (little-endian, all through the core's state writer):

1. `uint32` version; floats `ContactHertz`, `ContactDampingRatio`, `PushOutVelocity`, `RestitutionThreshold`, `MaxLinearSpeed`, `MaxRotationPerSubstep`; bool `ContinuousEnabled`; floats `ContactSlop`, `SpeculativeDistance`.
2. `int32` body slot count `nb`, then arrays over `nb`: local centre (3), mass, inertia (6), maxExtent, minExtent, linear damping, angular damping, gravity scale, force (3), torque (3), flags (bytes), first shape, last shape, `Δp` (3).
3. `int32` geometry count; per slot a bool (present) and, if present, `int32` length and the recipe bytes.
4. `int32` shape slot count; per slot a bool (live) and, if live: geometry index, local rotation (4), local translation (3), proxy id, friction, restitution, rolling resistance, density, category, mask, group, next shape of the body.
5. `int32` pair count; per pair a bool (live) and, if live: shape a, shape b, `int32` constraint count and the constraint ids in chain order. Then `int32` free pair count and the free list.

The reader checks every count against the bytes that remain before allocating, rejects non-finite values, non-unit rotations (squared length outside `[0.999, 1.001]`), invalid materials and references to missing geometry, rebuilds geometry from the recipes (a recipe that fails validation rejects the snapshot), cross-checks the section against the core (`IsConsistentWithCore`: proxies, shape chains, unit body rotations, constraint chains sorted by sub-shape pair), and restores atomically: any failure leaves the world as it was. The world hash covers the core state in `architecture.md`'s order plus nothing from this section that the core does not already hold (impulses live in the payloads); the origin is part of the hash.

## Origin

`ShiftOrigin(offset)` subtracts the offset from every body position and every broadphase box and adds it to the accumulated origin; components must be multiples of 2^-31 (`ArgumentException` otherwise). Positions are checked against ±2^21 (inclusive of the shifted origin) at creation and at the end of each step; a body outside, or with a non-finite position, raises `BodyOutOfBoundsException` naming it, and the step has no effect on the world's state (the hash is unchanged); destroying the body lets stepping continue.

## Determinism check

`tools/HashMatrix/Scenarios/World3D/World3DScenarios.cs` registers `world3d/pyramid-10` (300 steps), `world3d/pile` (100), `world3d/terrain` (100, bodies landing on a triangle mesh) and `world3d/restore-and-shift` (snapshot at step 30, restore at 45, origin shift at 60), each writing `ComputeHash()` after every step; the runner compares them across threads, vector widths, JIT settings, NativeAOT and WebAssembly.

## Performance

**Contact kernel.** Contacts are solved by one kernel written over lanes (`ILane<T>`, shared with 2D): the scalar twin (1 lane) and 128-bit (4) and 256-bit (8) vectors. A color batch (at most eight constraints, no dynamic body in common) is one group; `PrepareBatch` writes the group once per step in field-major rows (field `f` of lane `l` at `f·8 + l`; layout `W3D` in `ContactKernel3D.cs`): normal and tangents, material, inverse masses, soft coefficients, rolling mass, four point slots of arms, adjusted separation, normal, tangent-1 and tangent-2 masses, relative velocity, and the impulses that change while solving. The impulses live in the group during the step and `StoreResults` writes them back to the payloads. Each phase gathers the two bodies of every lane (velocity, angular velocity, accumulated translation and rotation, world inverse inertia), runs the arithmetic of the Step section for each lane with the same operations in the same order at every width, and scatters velocities of dynamic bodies. A lane with fewer points than the group's largest masks the unused slots with selects (a masked lane keeps its bits); slots past the group's largest point count are skipped. The overflow set (which may share bodies) runs one lane at a time in list order. Square root is the IEEE operation, so every width gives the scalar answer; the world hash is the same at widths 1, 4 and 8 and equals the hash of the scalar solver that preceded the kernel. `Engine3D.KernelWidth` (default 8) only picks the width.

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

**Sleeping on terrain.** The 3D world splits an island as soon as any of its bodies has rested for the sleep time (Step section). Awake bodies on mesh-terrain-3d at steps 120, 240, 360, 480, 600: before 1295, 1149, 1144, 1169, 1176 (all but a few of them motionless but joined to a rolling body); after 1294, 926, 753, 577, 507. Reference engines on the same scene (black-box dumps, awake flag): Jolt 1251, 615, 476, 235, 149; Box3D 1288, 859, 685, 571, 484; Rapier3D 1297, 888, 686, 571, 479. Rolling spheres and capsules have no rolling resistance by default here, as in the references' documented defaults (Jolt documents linear and angular damping of 0.05 per second and no rolling resistance; Rapier documents no damping by default), so rolling bodies stay awake on slopes until they reach a basin.

**Remaining gap** at one thread against the fastest reference (ms): large-pile-3d 42.0 vs 5.2 (Rapier3D), mesh-terrain-3d 9.5 vs 0.91 (Jolt), heightfield-terrain-3d 8.7 vs 0.89 (Jolt), sleep-wake-3d 9.1 vs 1.51 (Rapier3D), queries-3d settle 39.7 vs 5.7 (Rapier3D). Causes: the gather and scatter of body state per lane is scalar (nineteen floats per body per phase from five arrays; a packed per-step body row would turn it into one row load); the polyhedron axis test runs all edge pairs for every touching pair (a cached separating feature per pair does not help touching pairs, only an incremental manifold would); the group's lanes are masked for the unused point slots rather than sorted by point count; steady-state allocation is not zero on soup scenes (about 6.7 KB per step in pair churn and 2.8 KB in continuous collision's mesh casts on mesh-terrain-3d; 0.7 KB on large-pile-3d). Dedicated box-box code and the per-pair caching of the separating feature were not done. None of this is determinism-bound: no row is blocked by determinism rules.
