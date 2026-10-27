# The 2D world

Normative, language-neutral description of `src/OnePhysics/Physics2D/`: what a 2D world does in a step and which choices a port must reproduce to get the same bits. It sits on the shared core (`architecture.md`), the 2D collision code (`collision2d.md`) and the broadphase (`broadphase.md`). All arithmetic is IEEE `+ − × ÷ sqrt` on float32 (positions and broadphase boxes are double); no fused operations.

## Bodies, shapes, mass

* A body's **core position is its centre of mass**; the body origin is `com − R·lc` where `lc` is the local centre (float, stored per body). Shapes keep their authored coordinates in the origin frame. `Body.Position` is the origin.
* Mass data per shape and unit density comes from `collision2d.md`. For a dynamic body: `m = Σ density·area`, `lc = Σ density·area·centroid / m` (double, rounded to float), inertia about the centre of mass `I = Σ density·InertiaAbout(lc)`. Shapes with density 0 add no mass. No mass at all gives mass 1, inertia 1, `lc = 0`. `FixedRotation` gives inverse inertia 0. Adding or removing a shape recomputes this and moves the centre-of-mass position by `R·(lc_new − lc_old)` so the origin stays put.
* `maxExtent` = largest distance from `lc` to any core vertex plus the radius; `minExtent` = smallest of: radius (circle, capsule, segment), smallest `n·(v − centroid)` over polygon edges plus radius.
* Rotation is stored as (cos, sin). Angles in definitions are converted with onemath's correctly rounded `SinCos` (double, rounded to float); `Body.Angle` uses its `Atan2`.
* Bodies created with `StartAsleep` are put to sleep when created.

## Pair table

The world keeps one entry per pair of shapes `(a, b)`, `a < b` by shape slot, whose broadphase boxes overlap; it holds the contact constraint while the shapes touch. Entries get the lowest free index first (free list is a stack, snapshotted).

1. **Proxies.** Every proxy is the shape's tight box in world space (double: `position + float box relative to the centre of mass`), created with the shape. At the start of a step each awake body's proxies are moved with displacement = the body's displacement over the previous step. Margins: 0.1 for moving proxies, 0 for static (broadphase defaults, an option).
2. **New pairs.** `UpdatePairs` returns candidates in its sorted order. A candidate is accepted when the shapes are on different bodies, `CanCollide` holds (group rule, then category/mask), the optional pair predicate agrees, and it is not in the table.
3. **Which pairs update.** A pair is updated when either body is an awake dynamic body, or a kinematic body with non-zero velocity.
4. **Narrowphase**, in parallel over fixed chunks of 64 updated pairs, in ascending pair index: if the two stored boxes no longer overlap the pair is dropped. Otherwise the pose of B in A's frame is built from the origins `com − R·lc` subtracted in double and then converted to float, and `Collide2D.Collide` runs with the speculative distance `4 × 0.005`. Contact arms: `rA = qA·(p − lcA)`, `rB = rA + float(comA − comB)`; normal `qA·n`.
5. **Apply**, serially in ascending pair index: no points → constraint removed (if any) and the pair stays; boxes apart → constraint and pair removed; points → constraint added if needed (kind Contact, flag ReportPersist, bodies and shapes in pair order) or updated. A sleeping body in an existing touching contact with a moving kinematic body is woken. Because constraints exist only while shapes are within the speculative distance, contact begin/end events mean "touching (within 0.02)".
6. **Manifold payload** (layout in `architecture.md`): normal x, y; mixed friction, restitution, rolling resistance; the rolling impulse is kept from the previous manifold; per point the arms, separation, and normal and tangent impulses taken from the old point with the same feature id (0 if none); max normal impulse 0.

**Materials.** Default combine rule: friction `sqrt(fA·fB)`, restitution `max`, rolling resistance `max`. `WorldOptions2D.MaterialMixer` replaces it (it sees the first shape's material, the lower slot, then the second). Mixing happens in step 5.

## Step

The core pipeline (`architecture.md`) with these stage bodies; `h` is the substep time, `invH = 1/h` computed once.

* **UpdateContacts**: step 1 above for awake bodies; then every live body gets `start pose = pose`, `Δp = 0`, `Δq = (1, 0)`; then steps 2–5.
* **Prepare** (per active constraint): `hertz = min(ContactHertz, 0.25·invH)`; soft coefficients (below) with `hertz` and with `2·hertz` (used when either body has inverse mass 0), damping ratio `ContactDampingRatio`, over `h`. Per point: effective masses `1/(mA + mB + iA·(rA×n)² + iB·(rB×n)²)` for the normal and likewise for the tangent `t = (ny, −nx)`; adjusted separation `s − (rB − rA)·n`; relative normal velocity from the current velocities; the stored max normal impulse is reset. Rolling mass `1/(iA + iB)`.
* **Soft coefficients** (Catto, soft step): `ω = 2π·hertz` (float constant 6.2831855), `a1 = 2ζ + hω`, `a2 = hω·a1`, `a3 = 1/(1 + a2)`; bias rate `ω/a1`, mass scale `a2·a3`, impulse scale `a3`. Zero hertz gives bias 0, scales 1 and 0.
* **IntegrateVelocities** (dynamic bodies): `v += h·(gravityScale·g + invMass·force)`, `w += h·invI·torque`, each multiplied by `1/(1 + h·damping)`; linear speed clamped to `MaxLinearSpeed` by scaling; `|w|` clamped to `MaxRotationPerSubstep/h`.
* **SolveBatch**: the kernel below. A batch of colored constraints is run as one 8-lane group; the overflow set is run one constraint at a time in list order. The same arithmetic runs at 1, 4 or 8 lanes.
* **IntegratePositions** (awake dynamic and kinematic): `Δp += h·v`, `position += (double)(h·v)`; if `w ≠ 0`, `Δq` and the body rotation each advance by angle `h·w` (`c' = c − a·s`, `s' = s + a·c`, divide both by `sqrt(c'² + s'²)`).
* **FinalizeBodies**: forces and torque of dynamic bodies are cleared; continuous collision (below).
* **BodyMotion**: float.MaxValue if a touching constraint partner is a moving kinematic body, else `|v| + |w|·maxExtent`. Sleeping is the core's.
* **StoreResults**: nothing; impulses live in the payloads.

### Kernel (per point, per lane)

Gathered per constraint: velocities and Δp, Δq of both bodies; normal, friction, restitution, rolling, inverse masses and inertias, soft coefficients; per point arms, adjusted separation, masses, relative velocity, impulses.

* **Warm start**: apply `n·λn + t·λt` at the arms for both points; then rolling impulse as angular impulse on the bodies.
* **Solve** (bias) and **Relax** (no bias), in this order: rolling resistance (limit `rolling · (λn0 + λn1)`); friction of point 0 then 1 (limit `friction·λn`, tangent impulse accumulated); normal of point 0 then 1:
  `s = ((Δp_B − Δp_A + Δq_B·rB − Δq_A·rA)·n) + adjustedSeparation`;
  if `s > 0`: `bias = s·invH`, scales 1 and 0 (speculative: close the gap in one substep);
  else when biased: `bias = max(biasRate·min(s + slop, 0), −PushOutVelocity)` with the constraint's mass and impulse scales; else (relax) `bias = 0`, scales 1 and 0.
  `Δλ = −normalMass·massScale·(vn + bias) − impulseScale·λn`, accumulated impulse clamped at 0, maximum remembered.
* **Restitution**, once after the last substep: for points with relative velocity `≤ −RestitutionThreshold`, max normal impulse > 0 and restitution > 0: `Δλ = −normalMass·(vn + e·relVel)`, accumulated clamp at 0.
* Maximum and minimum are compare-and-select (`max(a, b) = a > b ? a : b`), negation is `0 − x`; the kernel has no divisions and no cross-lane operations. Only dynamic bodies are written.

### Continuous collision

For each awake dynamic body at the end of the step: movement `m = |com_end − com_start| + |sin(Δangle)|·maxExtent`; if `m ≤ 0.5·minExtent` nothing happens. Otherwise each shape's swept box (union of its start and end boxes) is looked up in the static tree; for each static shape that passes the filter, `TimeOfImpact` (target gap 0.005) between the static shape and the mover. For segments and chain segments a hit counts only if the mover's centre approaches the segment line by at least `0.25·minExtent` over the step, and (one-sided chains) started on the solid side. The smallest fraction `t` with `0 < t < 1` wins; the body's centre of mass goes back to `start + t·movement` and its rotation to the normalized lerp. Velocity is unchanged: the next step finds a speculative contact. Bullets against moving bodies are not implemented yet.

## Defaults

Substeps 4, solve iterations 1, relax iterations 1, contact hertz 36 (0.15 of the substep rate; the sleep-wake final penetration is chaotic, 0.10 to 0.22 across settings from 30 to 42, and 35 to 38 all measure under 0.15 while the pyramids sink less than at 30), damping ratio 10, push-out 3 m/s, restitution threshold 1 m/s, contact slop 0, max speed 400 m/s, max rotation per substep π/4, sleep: 0.5 s below 0.05 m/s, broadphase margin 0.1. Default gravity (0, −9.81).

## Events and callbacks

Events are the core's; contact begin/end carry the shape handles, in the core's sort order. After a step, `OnCollisionEnter/Stay/Exit` handlers run in that order, first for body A then body B of each event. Persist events are recorded only when the options' event mask includes `ContactPersist` (it also sets the stay callbacks' availability). A handler can destroy bodies: events already delivered are not changed.

## Snapshot section

Section id `0x32445747`, after the broadphase section. Version (u32 = 1); tunables (ContactHertz, ContactDampingRatio, PushOutVelocity, RestitutionThreshold, MaxLinearSpeed, MaxRotationPerSubstep as f32, ContinuousEnabled as u8, ContactSlop f32); body count n, then arrays for all n slots: local centre (2n f32), mass, inertia, max extent, min extent, linear damping, angular damping, gravity scale (n f32 each), force (2n f32), torque (n f32), flags (n u8), shape head, shape tail (n i32), last-step displacement (2n f32); shape count, per slot a live byte and for live slots the shape (kind u8, count i32, radius f32, flags u8, centroid, ghost 1, ghost 2, then 8 vertices and 8 normals as f32 pairs), proxy key i32, friction, restitution, rolling, density (f32), category, mask (u32), group, next shape (i32); pair slot count, per slot a live byte and for live slots shape A, shape B, constraint (i32); free pair count and the free stack (i32). The world options a snapshot restores are the core's plus these tunables; `BroadphaseMargin`, thread count and world id are construction-time. Delegates (material mixer, pair predicate, collision handlers) are never saved.

## Bodies leaving the supported range

Before any change, `Step` checks every non-static body's shape boxes, stretched the way the broadphase would store them (margin plus twice the last step's displacement). A box that is not finite or reaches beyond 2^22 in either axis raises `BodyOutOfBoundsException` (a `PhysicsException`) naming the body handle and its position, lowest body slot first. Nothing has been changed when it is raised, so the world hash is the same before and after, and every later `Step` raises it again until the host moves the body back, destroys it (`PhysicsWorld2D.GetBody(handle).Destroy()`) or shifts the origin. `Restore` applies the same test to every body (static included) and to the restored tunables, and rejects the snapshot with `SnapshotFormatException`, leaving the world unchanged. A snapshot may declare at most 2^22 bodies and 2^22 shapes, and the body count must fit in the bytes that remain.

## Origin shifting

After the core shifts positions, every shape's proxy is destroyed and re-created in ascending shape slot order; the pair table is unchanged. The next step reports all overlapping pairs again, of which the existing ones are ignored.

## Core additions this relies on

`IWorldStages.FinalizeBodies` (default no-op, called over chunks of awake bodies after the restitution pass and before `StoreResults`) and `WorldCore.Sleep(BodyHandle)`. Neither changes snapshot format version 1.
