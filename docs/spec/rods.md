# Circular Cosserat rods

The experimental rod solver is shared by both dimensions. It provides stretch,
shear, bend and spatial twist, pins and driven particle/material-frame targets,
generation-checked rigid attachments, force breaking, frictional capsule contacts
with the world, rod–rod contacts and self contacts. The rod APIs remain experimental.
The strain solver and contact geometry use physical SI parameters; they are not a
joint-chain wrapper. Production performance has not yet been established.

## Representation and units

Positions are double world coordinates, velocities and orientations float32.
A centerline has 2–4097 particles, with 0.0001–100 metre rest edges. A segment's
unit quaternion rotates local X onto its tangent. The planar engine uses the
same storage with Z positions and velocities zero, and rotations about Z only.
Each segment has a circular section of radius r, area A=pi*r², bending second
moment I=pi*r⁴/4 and polar moment J=2I. Density is kg/m³; E and G are pascals.
Each edge's rho*A*length mass is divided equally between its end particles.
Segment inertias are mass*(length²/12+r²/4) transverse and mass*r²/2 axial.
Pins and hard frame clamps have zero effective inverse mass/inertia.

Presets (E, G, radius, density): rope (1e7, 4e6, .01, 1000), cable
(2e11, 8e10, .005, 7800), hair (4e9, 1.5e9, .00005, 1300). These are starting
values, not a claim to model every manufactured rope or biological hair.
There are at most 65536 rod slots and 1048576 particles per world. Slot reuse is
lowest-first, with positive generations; a destroyed handle fails clearly.

## Constraints and solve order

The model uses the explicit centerline and segment frames of Kugelstadt and
Schömer, *Position and Orientation Based Cosserat Rods* (2016), sections 5–6,
with XPBD accumulated multipliers. It is not a distance-only particle chain:
frames are independent degrees of freedom and retain material twist. The
simultaneous Newton solve follows the direct-solve rationale of Deul et al.,
*Direct Position-Based Solver for Stiff Rods* (2018); here the unbranched chain's
linearized constraint matrix has fixed bandwidth, so tree traversal is not
needed. This formulation keeps the same solver and storage in both dimensions
without changing the existing soft-body solver.

For edge vector e=p[i+1]-p[i], frame q and length L, its three stretch/shear
constraints are C[k]=dot(R(q)*axis[k],e)-L*(k==0). The compliance is L/(E*A)
for X and L/(G*A) for Y,Z. Position gradients are minus/plus R(q)*axis[k];
the world angular gradient is cross(R(q)*axis[k],e).

Adjacent-frame bend/twist constraints are 2*sign*Im(conjugate(q[i])*q[i+1])
minus their rest value; sign is -1 if the relative quaternion W is negative,
+1 otherwise (including zero). With dual length D=(L[i]+L[i+1])/2, compliance
is D/(G*J) for material X (twist), D/(E*I) for Y,Z (bend). The angular gradient
on frame B for component k has world component j equal to
sign*Im(conjugate(qA)*(axis[j],0)*qB)[k]; frame A has its negative. The last
segment's bend rows are inactive. Planar Z shear and X/Y bend rows are inactive.

`ClampStart` fixes the orientation at the first **particle**, not the first
segment midpoint: three leading rows constrain frame zero to its creation
frame with half-length compliance L[0]/(2*G*J) or L[0]/(2*E*I). This avoids
moving the beam boundary by half an element on refinement. `ClampedSegments`
instead holds the specified material frame exactly, for deliberate hard clamps.
Inactive rows have zero gradient/violation and unit scaled compliance.

A substep h resets multipliers, saves old positions/frames, predicts with
v=(v+gravity*GravityScale*h)/(1+Damping*h) and angular velocity divided by the
same damping factor, then projects constraints. Positions and each position
increment use the world 2^-32 grid. Quaternion increments are normalized
q+0.5*(deltaTheta,0)*q. Particle velocities are displacement/h; world angular
velocities are 2*sign*Im(qNew*conjugate(qOld))/h with sign chosen by W.
Targets of pinned particles interpolate from their step-start position to the
new target over substeps; the last substep uses the target exactly.

Implicit drag also scales the constraint response: use W=M^-1/(1+Damping*h)
for both positional and angular response. Damping only the prediction but not W
would incorrectly reduce static deflection by 1/(1+Damping*h). Each nonlinear
iteration assembles J*W*J^T+compliance/h² and right-hand side
-C-(compliance/h²)*lambda. Groups have six rows: boundary first, then each
segment's XYZ stretch and XYZ bend/twist. Only adjacent groups share degrees
of freedom, giving lower bandwidth 11. Lower-band Cholesky, forward and back
substitution use ascending row/column indices (back substitution descending).
A pivot is bounded below by 1e-30 against roundoff. Reductions and multiplier
updates use double. Corrections accumulate in row order per particle/frame
and apply once. The default is 8 requested substeps and 4 nonlinear iterations.

All live rods share the maximum requested substep count. A rod's own iteration
count is retained, within the maximum iteration count's passes. Independent rod
predictions/projections/finalizations run in parallel by slot; every pass then
solves attachments serially and searches world contacts and rod pairs (parallel
sweeps, serial solves; see Contact search). This replaces
whole-rod-at-a-time stepping: interacting strands must share the same time slice.
The phase runs after existing rigid/soft phases. No scratch allocation occurs
after storage and the executor have been warmed.

## State and tooling

The optional snapshot section `0x524F4453` (RODS), version 2 (also reading version 1), is absent until
rod slots exist. No existing section layout, hash or phase order changes when
rods are unused. Little-endian fields:

- version; version 2 diagnostics: float32 maximum speed, float64 maximum relative
  stretch, float32 maximum penetration, int32 contact count, int32 conservative
  advancement limit count; slot count;
- per slot: generation, live flag;
- per live rod: E, G, radius, density; substeps, iterations, damping, gravity
  multiplier; ClampStart flag and root-frame XYZW; version 2 adds friction,
  restitution, collision/self-collision/collide-with-attached-bodies bools,
  category/mask uint32 and layer/group int32; particle count;
- per particle: XYZ double position, XYZ double pin target, XYZ float velocity,
  pinned bool, inverse mass; version 2 then has attachment body index (-1 for
  none), generation, local anchor XYZ, break force, last average force XYZ and
  broken bool. World identity is rebound to the destination world, not saved;
- per segment: XYZW frame, XYZ angular velocity, rest length, XYZ rest Darboux
  vector, clamped bool, transverse and axial inverse inertia;
- six double multipliers per group, including the leading boundary group.

Floats, bools, doubles and counts use the existing primitive encodings. The
reader checks counts against remaining bytes and the world limits, positive
generations, finite state, bounds, unit frames and planar components before
replacing state. Attachment body indices are checked against the body store being
restored (the snapshot's own bodies), so a snapshot restores into a freshly created world
with the same world id; a generation that no longer matches a live body at step time
breaks the attachment rather than rebinding a reused slot. A rejected whole-world restore is atomic. Previous positions,
old frames, contact candidates, attachment substep multipliers and solver factors
are scratch, rebuilt before use. A fork copies all
state and owns independent scratch. Region membership selects a whole rod if
any particle lies in the inclusive region; attachment closure includes selected
rods' bodies and selected dynamic bodies' attached rods. Restore requires compatible selected
slots/generations and field topology. Desync labels identify the rod slot;
repeated particle/segment fields use deterministic occurrence indices.

Exact-state scenes and recorded mutations use the same section. Origin shifts
validate positions and targets before translating them. Debug data emits one
segment with its endpoints, radius and material quaternion, in slot/segment
order. Raw span writes bypass recording and validation; invalid raw state is
rejected at the start of Step with RodStateException before rigid stepping.

## Swept contacts and rigid coupling

A rod segment is a capsule, not just two colliding particles. Existing 2D
Distance2D/Collide2D and 3D GJK/manifold routines provide signed separation,
normal and witnesses against ordinary shapes, compound children and triangle
soup leaves. End-cap sweeps supplement the interior witness: a face can constrain
both ends even when the closest-feature tie chooses only one endpoint. Shape
and indexed triangle materials use the world's ordinary/custom material mixing.
Sensors, layers, category/mask and signed groups retain their ordinary meaning.
Contacts with attached bodies are excluded by default; the definition can enable
them (as in the hair fixture).

Conservative advancement bounds the closing speed by the larger endpoint travel
relative to the rigid shape's translation over the substep, plus a quaternion-interpolation
angular bound times body extent. It advances by 0.8*(separation-skin)/speed, with
skin=max(1e-6,0.002*radius), at most 64 advances. For a shape that is one convex piece
the witness plane (the closest pair's separating plane) lies below the whole segment, so
each endpoint's own plane distance and its closing speed along the plane normal
(plus the angular bound) replace the isotropic speed, and the earlier of the two endpoints'
crossings decides: a rod sliding along a surface, or with one end far from it, no longer
creeps toward the skin in many small steps. Exhaustion retains a conservative contact, never a miss.
Rigid collision poses interpolate the rigid step's poses and the rod substep's
coupling corrections; they use the shortest normalized quaternion interpolation.
A witness supplies a unilateral plane constraint at a barycentric segment point.
Positions and terminal velocities are solved; restitution targets the incoming
speed, not the clipped displacement. Coulomb friction has anisotropic tangent
limits and surface velocities; spin/rolling torques act on segment frames and
rigid bodies with opposite impulses.

World features are applied in rod/segment/shape order, followed by endpoint
caps. Rod pairs are applied in (rod, segment, rod, segment) order, the smaller
(rod, segment) first. Self contacts exclude material neighbours:
the rest arc gap between their non-shared ends is at most twice the radius.
Lookup dictionaries are never enumerated; encounter order alone assigns contact
slots. Segment–segment witnesses use finite-segment closest points and the same
conservative advancement bound, with both strands' endpoint speeds.

The individual ContactModifier also sees rod contacts. Shape A is the rigid
shape, B is null for a world contact; RodIndex/RodSegment identify the rod. For
rod–rod contacts both bodies/shapes are null and OtherRodIndex identifies the
other strand. The normal is A-to-B as usual. A callback runs once per feature
per rod substep, after built-in one-way and material rules; it may disable
response or edit coefficients, separation, tangent speed and normal impulse
limit. Batch callbacks still address rigid constraint batches, not the separate
rod store. Callback subscriptions are not state; recorder step effects preserve
the modified simulation result without requiring callback code on playback.

A body attachment joins any particle to a body-authoring-frame anchor (the
centre-of-mass offset is accounted for). A symmetric 3x3 point effective mass
includes rod inverse mass, rigid inverse mass and the rigid lever-arm inertia.
Opposite position impulses and terminal velocity impulses are accumulated in
fixed order. Rigid velocity corrections advance the virtual rigid pose during
later rod substeps; omitting this feed-forward creates unstable repeated impulses.
The final corrected rigid poses refresh their broadphase proxies. The net
attachment multiplier divided by h² is the break force; the correction that
would exceed the threshold is not applied. Destroying the referenced generation
also breaks the attachment, without binding a reused slot. Readback is the
average force over the last whole step.

## Contacts at pinned ends and damaged state

A contact at parameter u on a segment moves the segment's end particles by penetration/u when the other end
cannot move (pinned) or is much heavier, which is unbounded as u approaches 0: a rope pinned on a surface used
to throw its free end kilometres away. Per pass no particle moves more than 4 penetration depths; when
the contact point needs more than 16 penetration depths of motion (the contact is all but at the pin, and no
motion of the rod can undo the penetration) the contact is left alone for that pass.

A snapshot can hold finite but enormous velocities. A step must complete or raise a typed error before changing
anything: a rod with a non-finite or out-of-range particle, or one whose segment is longer than 500000 radii or
farther than 1e6 m from the origin, is left out of collision for the rest of the step (the shape queries cannot
take such input), a rigid body whose accumulated rod push is non-finite or would leave ±4194304 m is not
written, and the next step's bounds check raises `RodStateException` for the damaged rod.

## Contact search

Candidates are gathered once per substep, not rediscovered every pass, and the cache never
decides a contact: each pass re-runs the exact tests on the cached candidates and a pass
that finds the cache possibly short rebuilds it. Margin G = 0.004 m. A sphere's distance to a
segment, while they are apart, is computed in closed form (the segment point nearest the centre);
every other shape goes through the general distance routines.

- **Shapes.** Per rod, the broadphase is asked for the rod's swept bounds (every segment's
  box over its previous and current particles) grown by radius+0.001+G in the static tree, and
  by that plus the largest, over moving bodies, of the larger absolute component of the body's
  pose shift plus its motion bound in the kinematic and dynamic trees. The union is sorted by
  shape slot, filtered by layers, category/mask, groups and sensors, then per segment kept when
  the segment's bounds meet the shape's near box grown by G. The near box is the shape's tight
  box shifted to the pose at the end of the substep and grown by the motion bound, the rod radius
  and 0.001, which is the exact per-pass test with G added. An endpoint cap uses the list of its
  segment (the last segment for the last particle).
- **Pairs.** Each segment's bounds grown by radius, skin and G go into a uniform grid
  (cell: twice the mean largest box edge, grown until there are at most 4n+64 cells; a box over
  64 cells is tested against every box instead). A pair is reported by the lowest cell the two
  boxes share; pairs between rods pass the layer matrix, groups and category/mask, pairs within one
  rod need self collision and non-neighbouring segments. Pairs are sorted by the key
  rod<<42|segment<<29|rod<<13|segment with the smaller (rod, segment) first, which is unique,
  so the grid's order is irrelevant.
- **Validity.** Before each pass every segment's current bounds must lie within G/2 of the
  bounds at the gather, and every rigid body a solve has pushed since the gather must have kept
  its pose shift plus motion bound within G/2 of the gathered value; otherwise the gather is
  redone. A body that was not pushed has not moved (its near box is a function of the
  substep alone).
- **Proofs of no contact.** A sweep that finds no contact records its clearance (a fifth of the
  smallest gap above the skin it measured, which bounds the gap over the whole sweep), the gap
  bound at the end of the sweep, the endpoints and the body's push count. The next pass skips the
  sweep while the endpoints have moved by at most half the clearance and the body was not pushed
  since. The next substep skips it when the end gap, less the drift between the recorded
  endpoints and the new start, exceeds the new substep's speed bound over 0.8, for a rigid body
  that has not been pushed this step; the proof then moves on with the reduced gap. Proofs are
  dropped at the start of every step and for shapes that leave the candidate list.
- **Sweeps and solves.** Sweeps read the rod and rigid poses as they stand at the start of the
  pass and run as one task per rod (world) or per block of 32 pairs, writing to per-rod and
  per-pair slots; the contact bookkeeping, contact modifiers and solves then run serially in the
  order above. A sweep therefore no longer sees corrections made by earlier contacts of the same
  pass (it did when search and solve alternated): results do not depend on the thread count.
  The count of conservative stops is summed in slot order.

## Analytic checks and numerical dissipation

The catenary check uses a 32-segment, 0.2 mm-radius rope of span 2 m and arc
length 2.4 m, settled for 1200 steps (sag tolerance 1%, relative stretch below
1%). A clamped 1 m beam under its own weight uses q*L^4/(8*E*I), 8/16 segments,
2/4/8 requested substeps and 1/4 nonlinear iterations (tip tolerance 3%). The
model includes shear, unlike Euler–Bernoulli theory. A held straight rod retains
pi/2 material twist within 1e-4 radians. Prescribed over-twist with endpoint
compression produces measured spatial writhe; self contacts prevent a free
crossing of the folded strands.

The zero-explicit-damping pendulum check includes kinetic, gravitational and
elastic strain energy. Over 10000 steps its maximum may not exceed its initial
energy by 0.001 J. This is a no-energy-growth test, **not energy conservation**.
Prediction, position projection and velocity reconstruction form an implicit,
dissipative integrator. Finite substeps and incomplete nonlinear convergence
produce numerical damping even when Damping=0. Near a linear mode of frequency
omega, implicit-Euler energy scales approximately as 1/(1+omega²*h²) per substep;
smaller h reduces that loss. A separate two-second small-angle test at 1/4/16
substeps measures decreasing excess-energy loss (about 1.891/0.527/0.123 mJ from
7.698 mJ initially). The long pendulum's roughly -1.54 J final energy is close
to the hanging rope's minimum gravitational potential; the removed energy was
numerically dissipated, not a hidden velocity-damping parameter.

## Shared fixtures

`tests/Common/RodScenes.cs` defines rope-bridge-3d (two ropes, twelve attached
planks and six dropped boxes), rope-whip-3d (30 m/s against a 1 cm wall),
hair-3d (1000 six-segment strands attached to a moving spherical head) and
rope-2d (driven swinging rope, attached weight and an obstacle field). The bar's
`rod-run` command runs 600 steps at 1/60 s and reports speed, relative stretch,
remaining capsule penetration, tunnelled count and a final hash.

No managed engine runs rods, so the thresholds are our own: 1.5 times the worst of the
x64 and arm64 values, rounded up to a 1/1.5/2/3/5/7 step, with a 1 mm penetration floor.

| scene | maxSpeed | max relative stretch | rod penetration | tunnelled | measured (speed / stretch / penetration) |
|---|---|---|---|---|---|
| rope-whip-3d | <= 45 | <= 0.005 | <= 0.001 | 0 | 30.0 / 0.0016 / 0 |
| rope-2d | <= 5 | <= 0.05 | <= 0.001 | 0 | 2.62 / 0.0294 / 0.00025 |
| rope-bridge-3d | <= 10 | <= 0.05 | <= 0.001 | 0 | 6.64 / 0.0215 / 0 |
| hair-3d | <= 5 | <= 0.15 | <= 0.001 | 0 | 2.91 / 0.0931 / 0.00043 |

Speed is the larger of any rod particle's and any rigid body's speed. Penetration is the
deepest remaining overlap between a rod capsule and a shape, after the last pass of a
substep; it does not include the rigid plank and box contacts of the bridge. Tunnelled is
defined per scene, not as a general topology test: whip counts particles whose centre
is past the wall's centre plane; hair counts particles whose centre is inside the head
sphere (radius 0.34 against a 0.35 head); bridge and swing count dynamic bodies and rod
particles whose centre is below the floor slab's centre plane.

x64 (1, 3 and 8 threads) end hashes after 600 steps: whip 32bfdfb01bd5cec5,
rope-2d d10ad261d31ad6de, bridge 193ca0ba3e8809ce, hair 35e60c339911ff26.

Joint-chain comparison uses matching particle locations/masses and distance joints, but
has no Cosserat bend/twist, collides as spheres (at least 1 cm radius) instead of swept
capsules, and reports no penetration. Indicative timings (Apple M2, 8 cores, load about 2,
120 steps, first 30 dropped, median step; not a managed-engine verdict). The rod rows are
from before the contact search was reworked (Contact search above); the next table is after it.

| scene | model | t1 ms | t8 ms | peak RSS |
|---|---|---|---|---|
| hair-3d (1000 strands) | rods | 344 | 303 | 77 MB |
| hair-3d | joint chain | 71 | 25.5 | 1.53 GB |
| rope-bridge-3d | rods | 5.05 | 4.39 | 57 MB |
| rope-bridge-3d | joint chain | 0.15 | 0.17 | 59 MB |

The bridge's joint chain stretches 0.158 (limit 0.05), so it is unstable at the same
scene definition.

After the contact search rework (same machine and method, 150 steps, 3-6 loaded cores; the
branch's table row against the same-day run of the previous code):

| scene | before t1 / t8 ms | after t1 / t8 ms |
|---|---|---|
| hair-3d (1000 strands) | 309.8 / 268.3 | 102.7 / 32.1 |
| rope-bridge-3d | 5.06 / 4.40 | 2.16 / 1.25 |

What remains in hair-3d at one thread is mostly the rod solve (about 75 of 165 ms on x64, which the
contact search does not touch), the capsule distance queries of the sweeps near the head, and the candidate
gather (about 10 ms a step at one thread). The bridge touches no shape, so its cost was
all search.

## Pulley systems: ropes over wheels, winches, belts and chains

A pulley system is a rope (or a closed loop of belt or chain) that runs over circular wheels and ends on anchors or winch
drums. It lives beside the rods: the rod solver is untouched, no rod row or hash moves when a world has no pulley system,
and the system's state is its own optional snapshot section. It is shared by both dimensions (a planar world uses the Z
axis and Z-zero points) and has the three API tiers: `AddRopeOverPulleys`, `AddWinch`, `AddBelt` and `AddChainDrive`
(tier 1), `PulleySystemDef`/`PulleyNodeDef` with `AddPulleySystem` (tier 2), and the `PulleySystem` handle's readbacks.

**Why the free spans are not rods.** A wrap needs exact tangent geometry, an exact capstan slip law and a rope length that
is conserved to rounding across the wheels; a Cosserat rod is an elastic chain whose length changes with its tension and
whose contacts with a wheel give a friction that depends on discretisation. The system is therefore a taut, inextensible,
massless rope: its spans are straight between tangent points, a slack span exerts nothing, and rope mass, bending stiffness,
collision of the rope with other shapes and a sagging free span are not modelled (`Sag(span)` reports a parabola estimate
for drawing). A rope that must swing, drape or collide is a rod, attached to the same bodies.

### Nodes and units

Nodes run in path order: an open rope is anchor or drum, wheels, anchor or drum; a loop (`Closed`) holds only wheels. An
anchor is a point on a body (authoring frame) or in the world. A wheel has a centre, unit axis, radius (0.001–1000 m), the
direction the rope turns about the axis, a Coulomb friction coefficient (0–100), optional `NoSlip`, `ExtraTurns` (0–64), and
either a rotor (own inertia in kg·m², infinity locks it; bearing damping per second; motor and brake torques) or the carrier
body's own spin (`WheelSpin.Body`, for sprockets and wheels on hinges; no motor or brake). A drum is a rotor wheel at an
end of an open rope where the rope is wound on and never slips. 2 to 256 nodes, at most 65536 systems per world, slots
reused lowest-first, generations positive. Coordinates are finite within ±2^21 m. `FixedAxle` says the carrier is mounted
(a static body, or a body held by joints): the solver then treats it as immovable and still passes the rope's force to it.

### Geometry

All geometry is double and is computed relative to the first node's carrier (differences of grid positions are exact, so a
shifted world sees the same numbers). A span runs from the tangent point of its start node to that of its end node.

- Point to wheel: in the plane perpendicular to the axis, the tangent from a point at distance d > r has radial direction
  `cos(phi) w + sigma sin(phi) (n x w)` with `cos(phi) = r/d` and sign `sigma` chosen by the turning direction and by
  whether the rope arrives or leaves; a point inside the rim (d ≤ r) has no tangent and is an error on creation.
- Wheel to wheel, parallel axes (the common case, exact): with `a = s1 r1 - s2 r2` (signs relative to the first axis, an
  anti-parallel axis flips the second) and `d` the planar centre distance, the rope direction is
  `t = sqrt(1-(a/d)²) w + (a/d)(n x w)`, an external tangent for equal turning directions and an internal one otherwise;
  `|a| ≥ d` has no tangent. The axial offset of the centres is carried along, so a rope may leave the wheel's plane.
- Wheel to wheel, skew axes: eight alternating point-to-wheel tangents. The result touches both rims to better than 1e-10
  (checked on 200 random pairs).
- Wrap angle: the angle between the tangent points in the turning direction, unwrapped by taking the branch nearest the last
  step's (`Math64.Atan2`), plus `ExtraTurns` full turns at creation. A wrap just below 2π at creation is read as zero.
- Out of plane (a fleet angle), the arc is still counted as the planar rim arc. That is exact for the straight length and
  the velocity law below only while the rope stays near the plane: at a fleet angle φ the length error is of order
  `(1 - cos φ)` times the slip speed over the rim. The crane scene reaches about 15° (see Fixtures).

### Material length and stretch

Each span k has a material length `m[k]`: the rope between the reference points of its two wheels (a reference point is a
point of the rim, fixed in the wheel's frame, chosen at the creation tangent). The surface length is
`h[k] = |Q - P| + side r (alphaOut - ref) of the start wheel + side r (ref - alphaIn) of the end wheel`, with angles
measured in the wheel's own frame (carrier frame minus the rotor angle). `Stretch(k) = h[k] - m[k]` is positive when the
geometry needs more rope than the span has. The sum of the `m` is the rope's total length; it changes only by
`AdjustLength` and, in a closed loop or run of wheels, rope slipping from one span to its neighbour moves material
without changing the sum (checked: after 10,000 steps of a driven block and tackle the sum differs by 7e-15 m).
`PulleySystemDef.Length` (or `Slack`) sets the rope: 0 means exactly taut, a shorter rope than the path is an error.

### The step

The stage runs once per world step after body controls, fields, vehicles and fluids and before the rigid step, serially
and in slot order, so it does not depend on the thread count. Per system:

1. Read every body's velocity, predict one step ahead (`v + dt (g scale + F/m)`, and the same for angular velocity with the
   torque already queued by the host and the fields), keeping the rigid step's own forces out of the rope's way.
2. Build one row per span, `J.V = d . (v_Q - v_P)` where `v_P` and `v_Q` are the velocities of the rim *material* points at the
   tangent points (carrier velocity, carrier spin, rotor spin). Sliding of a tangent point along the rim cancels in the
   derivative of `h`, which is why this is exact for planar wheels. A rotor's spin is its own entity (inertia, bearing
   damping and carrier reaction included); a `FixedAxle` carrier contributes nothing to the rows but receives the loads.
3. Bias: the speculative form for slack (`e/dt` for a negative stretch `e`, so a slack rope can at most become taut) and
   `0.25 e/dt` for a positive one, after a deadband of 10 µm (1 mm for a chain) that stops float32 body-angle rounding
   from turning into a velocity error.
4. Sequential impulses, `Iterations` sweeps (default 32; warm-started from the previous step's tension impulses), tension
   impulses ≥ 0 and at least `Preload × dt` (a belt tensioner: a belt transmits torque only through its slack side).
5. Capstan slip (Euler–Eytelwein): each wheel with friction μ and wrap θ is *stick* (the rope moves with the rim, tension
   ratio anywhere in `[e^-μθ, e^μθ]`), *slip forward* or *slip back* (ratio exactly `e^±μθ`). Spans joined through slipping
   wheels are solved as one unknown with the ratios fixed; after each round the state changes where the ratio bound is
   violated (stick to slip) or the slip speed has the wrong sign (slip to stick), at most six rounds. A frictionless wheel
   always slips with ratio 1; a closed loop keeps at least one sticking wheel. Rope slides through slipping wheels by
   moving material between the spans of a run (each span takes what keeps it taut, the last gives back the sum). A run
   whose tension changes nothing it can feel (a belt slipping over a wheel that cannot move) has no determined level and
   takes its preload.
6. Motor and brake rows on rotors (relative spin target and torque limit; reaction on the carrier).
7. Output: the total tension impulse of each span divided by dt becomes a constant force and torque on each dynamic carrier
   through the host's force arrays, applied in every substep (so a hanging load is exactly at rest rather than ringing at
   the substep rate). `KeepJointSchedule` puts the carriers' islands on the world schedule. A body that sleeps is woken
   only when its predicted velocity change exceeds the sleep threshold; a system whose bodies all sleep and whose rotors
   are still does nothing. A chain additionally hands the loaded span's sub-millimetre stretch to the next span (the
   integrator rounding of two sprockets' angles), so the speed ratio is the tooth ratio instead of bending to hide it.
8. Break: a span tension above `BreakForce`, or a destroyed carrier, breaks the system for good; a broken system exerts
   nothing and `IsBroken` reports it.

Behaviour that follows: the machine assumes a mounted carrier is immovable and that the other bodies are free, so a hinge
that carries a wheel must be stiff relative to the rope tension, and a very light wheel body on a hinge under a tension of
hundreds of newtons will jitter exactly as any light body under a large external force would. Contact and joint impulses
of the same step are resolved after the machine, so they correct, not undo, its loads.

### Winches, belts and chains

A winch is a drum at the end of an open rope; its spin winds rope on and off (the drum's reference angle moves, so
the free length `FreeLength(0)` changes by `r ω dt`) and its motor and brake are the rotor's rows: lifting a 20 kg load at
a rated 1.000 m/s took a drum torque of exactly `m g r` = 39.240 N·m. A brake holds up to its torque and the drum slips
beyond it. `AdjustLength` pays rope out without a drum. Belts are closed loops with friction and preload; with equal
wheels the capacity before the driver slips is `R (e^(μπ) - 1) T0`, T0 the preload. Chains are closed loops with
`NoSlip` wheels whose radius is `teeth × pitch / 2π`; the length is a whole number of pitches (default: the shortest even
number that reaches round the sprockets), the slack side hangs slack (zero tension) and the loaded side carries all the
torque.

### State, tooling and API limits

Optional section `0x4341424C` (`pulley systems`), version 1, absent until a system exists: version, slot count, then per
slot generation and live flag; per live system closed, chain, iterations, break force, preload, pitch, broken, last dt,
advance dt, node count; per node kind, body index and generation, local point (3 doubles), axis and two reference vectors
(floats), radius, side, friction, no-slip, spin mode, inertia, bearing, fixed axle, motor speed, motor torque, brake
torque, spin, rotor angle, entry angle, exit angle, reference angle (doubles), slip state, motor and brake impulse;
then the doubles of material length, tension impulse, stretch, surface length, free length per span and wrap angle per
node. Restore validates every range (non-finite values, slip states, axis lengths, body generations) and rejects with a
`SnapshotFormatException`, leaving the world unchanged. Fork copies the systems, region snapshots select a system with its
carrier bodies (`cable` entities), the origin shift moves world-fixed points, replay records every creation, option and
motor call, and the desync finder names fields as `pulley systems: cable i ... m.Material[k]`. `DebugContent.Pulleys` adds
`PulleySpans` (tangent points, tension, slack flag) and `PulleyWheels` (centre, axis, entry and exit points, wrap, spin).
A broken, destroyed or default handle throws `InvalidOperationException`; bad definitions throw
`ArgumentException`/`ArgumentOutOfRangeException` naming the node and field.

### Checks

Analytic (tests `PulleySystemTests`, `PulleySystem3DTests`): an Atwood machine gives exactly `(m1-m2)g/(m1+m2)` and
tension `2 m1 m2 g/(m1+m2)`; the capstan tension ratio over a locked wheel equals `e^(μθ)` to better than 1e-5 relative at
wraps of π/2, π, 2π and 4π (μ = 0.25); block and tackle 2:1 and 4:1 hold their load at tension `M g/n` to 1e-3 relative and
accelerate at the predicted `(F + m g - M g/n)/(m n + M/n)` to 1e-4; a winch lifts at its rated speed and holds with its
brake; the tooth ratio holds exactly (0.0 relative error, 600 steps); a belt locked at one wheel carries exactly
`R (e^(μπ) - 1) T0` (3.1327 N·m) and spins up at the predicted rate; 1 and 8 threads, restore and fork agree per step;
a shifted world steps bit-identically.

### Fixtures

`tests/Common/PulleyScenes.cs`, bar command `pulley-run`: **tackle-2d** (a four-fall block and tackle with a hauler pulled
by a ±6 % force, plus a friction pulley whose weights stick and slide in turn), **belt-2d** (a three-wheel belt with an
idler under a driver torque and a speed-dependent load, slipping at the peaks), **chain-drive-3d** (a bicycle: frame,
two wheels and a crank on hinges, 44-tooth chainring to a 16-tooth sprocket on the rear wheel, driven by a pedal torque
of 6 ± 3 N·m for 10 s, 13 m travelled) and **crane-3d** (a slewing kinematic boom carrying a winch drum and a sheave,
a two-fall block with a 108 kg crate that is lifted, swung by the slewing boom and lowered). 600 steps at 1/60 s. No managed
engine runs pulley systems, so the thresholds are ours: 1.5 times the worst of x64 and arm64 values (which agree), rounded up
to a 1/1.5/2/3/5/7 step, floors 1 mm of stretch and penetration. Overstretch is the largest total path length minus rope of
a loaded system; length drift is the largest change of the total material length; tunnelled counts bodies below the floor and
broken systems.

| scene | maxSpeed | overstretch | penetration | length drift | tunnelled | measured (speed / overstretch / penetration / drift) |
|---|---|---|---|---|---|---|
| tackle-2d | <= 1.5 | <= 0.001 | <= 0.001 | <= 1e-9 | 0 | 0.881 / 1.8e-4 / 0 / 7e-15 |
| belt-2d | <= 0.5 | <= 0.001 | <= 0.001 | <= 1e-9 | 0 | 0.327 / 0 / 6e-5 / 4e-15 |
| chain-drive-3d | <= 5 | <= 0.001 | <= 0.001 | <= 1e-9 | 0 | 2.555 / 0 / 0 / 0 |
| crane-3d | <= 2 | <= 0.007 | <= 0.001 | <= 1e-9 | 0 | 1.093 / 4.7e-3 / 0 / 7e-14 |

Limits: rope mass, rope collision and bending are not modelled; fleet angles beyond about 25° give percent-level length
errors (the crane scene at a 0.25 rad slew amplitude, about 35° of swing, overstretches by 3.5 cm against 4.7 mm at 0.15); chain links are not individually simulated (no polygon effect, and a chain longer than the taut path sags only as a
reported estimate); sprocket or wheel bodies must hang on hinges; a break emits no event, only the flag.
