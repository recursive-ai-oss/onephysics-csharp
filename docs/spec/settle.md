# Solving to rest: settle, stability margin, rest shapes, insertion check

Normative for both worlds (`PhysicsWorld2D` and `PhysicsWorld3D`; the members have the same names, with 2D or 3D math types). A world that was built by a script or a level editor is rarely at equilibrium: boxes are placed a little above their supports, ropes are laid out straight, cloth hangs flat. These four features bring a world to rest and read back what a host needs without watching it run.

All of it is deterministic: the same inputs give the same bits on every platform, vector width and thread count. None of it changes a world that does not call it.

## Settle

`world.Settle(options)` runs ordinary steps until the world is at equilibrium or a budget runs out, then returns a `SettleReport`.

### The run

1. Options are validated (`SettleOptions2D` / `SettleOptions3D`, copied; an invalid field raises `ArgumentOutOfRangeException` naming it). Calling Settle from inside a step, a callback or another settle raises `InvalidOperationException`.
2. The world's sleep option is switched off, callbacks are suppressed, and a recorder is paused. After the run (also when a step throws) the sleep option is restored and the live options are copied into the core again, so a snapshot taken afterwards holds the caller's options, not the run's.
3. Each iteration is one public `Step(dt)` at `Dt`, then the extra damping, then the measurement. The step count of the world advances by the steps used.
4. The run ends when the measurement has held for `StableSteps` consecutive iterations (converged), or after `MaxSteps` iterations (not converged).

Sleeping is off so that a body that is still moving cannot hide by falling asleep, and so that the run does not depend on sleep timers. Bodies that were asleep when the run started stay asleep until something touches them. After the run bodies fall asleep in later steps as usual.

### Extra damping

When `ExtraLinearDamping` c is above zero, after every step the velocity of every awake dynamic body and of every particle of an awake soft body and of every rod is divided by `1 + c·dt` (implicit damping: stable for every c ≥ 0). `ExtraAngularDamping` does the same to the angular velocity of rigid bodies. Damping is part of the run; it is not left on afterwards. It shortens the run and removes bounce; it also changes where a bouncing system comes to rest, so a host that wants the undamped rest state leaves it at 0.

### The measurement

For every awake dynamic body whose centre of mass is inside the bounds (all bodies when there are none), after the step:

- **speed** = |v| in double, rounded to float32 (m/s);
- **angular speed** = |ω| (rad/s; the single component in 2D);
- **acceleration** = |v − v_previous| / dt, where v_previous is the velocity after the previous step (the velocity before the run for the first step);
- **force** = mass × acceleration.

Soft-body and rod particles contribute their speed only.

A step is *at rest* when the worst speed ≤ `SpeedTolerance`, worst angular speed ≤ `AngularSpeedTolerance`, worst acceleration ≤ `AccelerationTolerance` and worst particle speed ≤ `SpeedTolerance`. Speed alone would accept a ball at the top of a bounce; acceleration alone would accept a block sliding at constant speed on a slope at exactly its friction angle; a body held by contacts or joints shows an acceleration of about zero because the solver cancels gravity inside the step.

Defaults (metres, seconds): `Dt` 1/60, `MaxSteps` 1200, `SpeedTolerance` 0.01, `AngularSpeedTolerance` 0.02, `AccelerationTolerance` 0.25, `StableSteps` 20, `SupportTolerance` 0.005, both dampings 0, no bounds, no per-body arrays.

### Support and bounds

After the run, every dynamic body is classified:

- **Supported**: connected to a static or kinematic body through joints and *touching* contacts. A contact touches when one of its points has a separation of at most `SupportTolerance` or a stored normal impulse above zero. Connection is transitive (a box on a box on the ground is supported); computed by union-find over the constraints with the lowest slot as root. A body resting on nothing static (a stack floating in zero gravity, or a body still falling) is *unsupported*. Soft bodies, particle fluids and characters' ground checks are not part of this graph.
- **Out of bounds**: its centre of mass lies outside `Bounds` (inclusive box). Such a body is counted and listed; it is not judged for convergence, so one body that fell out of the level does not keep the rest from converging. `IsClean` is `Converged && UnsupportedCount == 0 && OutOfBoundsCount == 0`.

### The report

| Field | Meaning |
|---|---|
| `Converged`, `StepsUsed` | outcome and iterations taken |
| `BodiesJudged` | awake dynamic bodies inside the bounds in the last step |
| `WorstSpeed`, `WorstSpeedBody` | fastest linear speed after the last step and its body |
| `WorstAngularSpeed` | fastest spin |
| `WorstForce`, `WorstForceBody` | largest mass × acceleration in the last step and its body |
| `WorstDeformableSpeed` | fastest particle of an awake soft body or rod |
| `UnsupportedCount`, `OutOfBoundsCount` | as above |
| `Unsupported`, `OutOfBounds`, `Bodies` | **only with `ReportBodies`**: handles in slot order, and per awake dynamic body its speed, angular speed, force, support and bounds flags; otherwise null |

Ties for the worst body go to the lowest slot. Allocation: Settle allocates scratch proportional to the body count for the call (previous velocities, the union-find arrays) and the report object; the per-body arrays exist only when asked for. It is an offline operation and does not belong in a frame loop.

### Recording

A recorder attached to the world sees one `Settle` record (replay record kind 7, below), not the steps. Playback runs the same settle with the recorded options on the playback world and requires the same encoded report and the same world hash; a mismatch raises `ReplayFormatException` (report) or `ReplayHashMismatchException` (hash). Settle inside a history recorder is a non-frame record: seeking to a step applies every settle recorded before it.

Report encoding (72 bytes, little-endian): byte 0 converged; int32 StepsUsed at 4, BodiesJudged at 8; float32 WorstSpeed 12, WorstAngularSpeed 16, WorstForce 20, WorstDeformableSpeed 24; int32 UnsupportedCount 28, OutOfBoundsCount 32, WorstSpeedBody index 36 and generation 40, WorstForceBody index 44 and generation 48; the rest zero.

Options encoding: 3D 88 bytes, 2D 72 bytes: float32 Dt 0; int32 MaxSteps 4; float32 SpeedTolerance 8, AngularSpeedTolerance 12, AccelerationTolerance 16; int32 StableSteps 20; float32 ExtraLinearDamping 24, ExtraAngularDamping 28; int32 flags 32 (bit 0 ReportBodies, bit 1 has bounds); float32 SupportTolerance 36; float64 bounds from 40 (min then max: 6 values in 3D, 4 in 2D).

## Stability margin

`world.GetStabilityMargin(assembly, options)` and `body.GetStabilityMargin(options)` measure how hard it is to tip a body, or a set of dynamic bodies treated as one rigid assembly, that rests on other bodies. Read-only; call it after a settle.

- **Up** is the unit vector opposite to the world gravity (`InvalidOperationException` when gravity is zero).
- **Assembly centre of mass** is the mass-weighted mean of the bodies' centres of mass (double). Mass is the sum.
- **Support points** are the contact points between a member and any body outside the assembly (static, kinematic or dynamic), in constraint-slot then point order, whose separation is at most `ContactTolerance` (default 0.005 m). Joints do not support.
- **Support polygon** (3D): the points projected onto the plane perpendicular to up; their convex hull (monotone chain; ties broken on the second coordinate, then the height, then the contact order), counter-clockwise seen from above, strictly convex. `SupportPolygon` returns the corresponding 3D contact points. **Support interval** (2D): the two extreme points along the horizontal tangent `t = (up.y, −up.x)`; `SupportPolygon` returns them (one point when equal).
- A hull with fewer than three corners or no area gives margin 0, is not stable, and reports zero tip angles and pushes.
- **Margin**: the smallest distance from the centre of mass's projection to a hull edge (inside positive, outside negative).
- **Tipping**: for an edge e with outward unit normal n_e, let w_e be the horizontal distance from the centre of mass to the edge and H_e its height above the edge (the height of the point of the edge nearest the projection, interpolated between the corner heights). A horizontal push F in direction d at the centre of mass tips the assembly about e when F (d·n_e) H_e ≥ m g w_e. So the tipping push for direction d is the smallest `m g w_e / (H_e (d·n_e))` over edges with d·n_e > 0, and the **tip angle** — how far the ground can be tilted toward d before the assembly tips — is `atan(w_e / (H_e (d·n_e)))` for the same edge (the same ratio). 2D: two directions, ±t, and one edge each.
- **Directions** (3D): `DirectionCount` (4..360, default 16) unit vectors around the up axis at angles 2πk/N from a base axis found by crossing up with the world axis it is least aligned with (for up = +Y the base is +Z and the direction a quarter turn later is +X, so a box aligned with the world axes is hit exactly when N is a multiple of 4). Angles come from onemath's `Cos` and `Sin`.
- `MinTipAngle`, `MinTipDirection` (first direction for equal angles), `MinTippingForce`.
- **Sliding**: `SlidingForce` = μ m g, with μ the mean of the support contacts' mixed friction weighted by their normal impulse (unweighted when no impulse is stored). `SlidesBeforeTipping` is `MinTippingForce > SlidingForce`. The tipping estimate itself ignores friction.

The estimate is a statics result, not a simulation: it assumes the contacts lie on one support plane (heights are interpolated otherwise), and a push that is applied below or above the centre of mass changes the numbers (the torque arm is the push's own height).

## Rest shapes

`world.GetRestShape(softBody)` and `world.GetRestShape(rod)` return a `RestShape2D` / `RestShape3D`, a copy: particle positions (double, world coordinates), triangles (soft bodies and cloth; 3D: the surface triangles, 2D: the mesh triangles; empty for rods and bodies without a surface), edges as index pairs (soft bodies: constraint edges; rods: segments), the segment frames of a rod (3D quaternions; 2D angles in radians), bounds, `Up`, `Sag`, `ArcLength` (rods) and `MaxSpeed`.

`Sag` = (reference height − lowest particle height), floored at 0, where heights are measured along up and the reference is the lowest pinned particle, or the highest particle when none is pinned. A string pinned at both ends gives its sag below the lower end; a cloth pinned at its corners gives its droop. `MaxSpeed` is the fastest particle at the moment of reading: zero-ish after a converged settle, which tells a host the shape can be baked.

A stale or foreign handle raises `StaleHandleException` / `InvalidHandleException`.

## Insertion check

`world.CheckInsertion(part, axis, distance, options)` and `body.CheckInsertion(axis, distance, options)` push a part (one body, or several that move together; none static) along `axis` for up to `distance` metres into whatever it is going to be mated with, without moving or waking anything. The other bodies of the part are ignored by every sweep (so is a body joined to a part body when `ExcludeJointed` is set).

- **First contact** is the whole-body translation sweep (the queries spec) of every part body along the axis; the nearest wins, ties by body slot order. `FirstContactDistance` is +∞ when nothing is hit within `distance`; the point, the normal (facing the part) and `Blocker` come from the hit. Cast accuracy is about 2×10⁻⁴ of the cast length.
- **Travel** is `min(first contact, distance)`.
- **Clearance profile**: `Stations + 1` (default 16 + 1) stations spaced evenly from 0 to the travel (to `travel − SeatTolerance` when something is hit, because a part resting on its mate along the axis also touches it sideways). At a station every part body is cast sideways over `ProbeDistance` (default 0.05 m) in `ProbeDirections` directions (3D: default 8 evenly spaced around the axis from a base direction found like the stability directions; 2D: the two perpendiculars). The station's clearance is the smallest hit distance, or `ProbeDistance` when nothing is hit; a clearance of `ProbeDistance` means "at least that". The sweeps start from the part's pose translated along the axis, so the part is never moved. `MinClearance`, its distance along the path and direction are reported.
- **Seats** = not blocked (first contact is not before `distance − SeatTolerance`, default 1 mm) and `MinClearance ≥ RequiredClearance` (default 0). `IsBlocked` is the first condition alone. To test seating give the depth at which the part is fully home as `distance`.
- Sampling: the true smallest lateral clearance can be smaller than the smallest sampled one when no probe direction points at the nearest wall: by at most a factor 1/cos(π/ProbeDirections) (8% at 8 directions). Axis-aligned parts in an axis-aligned fixture are exact when `ProbeDirections` is a multiple of 4.

Cost: `(Stations + 1) × ProbeDirections × part bodies` casts plus one per body for the first contact. The report allocates the profile array.

## Determinism notes

Settle's loop, the support graph and the margin are single-threaded code over index-ordered data; the steps inside are the world's own and are thread-count and width independent. The hash matrix rows `settle/pile-3d`, `settle/pile-2d`, `settle/string-3d`, `settle/string-2d` fold the world hash after slices of a settle, every number of the reports, the margin, the insertion profile, the rest shapes and a recorded-and-replayed settle.
