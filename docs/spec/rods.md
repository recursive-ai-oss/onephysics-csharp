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

## Contact search

Candidates are gathered once per substep, not rediscovered every pass, and the cache never
decides a contact: each pass re-runs the exact tests on the cached candidates and a pass
that finds the cache possibly short rebuilds it. Margin G = 0.004 m.

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
| hair-3d | <= 5 | <= 0.15 | <= 0.001 | 0 | 2.91 / 0.0931 / 0.00044 |

Speed is the larger of any rod particle's and any rigid body's speed. Penetration is the
deepest remaining overlap between a rod capsule and a shape, after the last pass of a
substep; it does not include the rigid plank and box contacts of the bridge. Tunnelled is
defined per scene, not as a general topology test: whip counts particles whose centre
is past the wall's centre plane; hair counts particles whose centre is inside the head
sphere (radius 0.34 against a 0.35 head); bridge and swing count dynamic bodies and rod
particles whose centre is below the floor slab's centre plane.

x64 (1, 3 and 8 threads) end hashes after 600 steps: whip 32bfdfb01bd5cec5,
rope-2d d10ad261d31ad6de, bridge 193ca0ba3e8809ce, hair 4a7c4a1682836549.

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
| hair-3d (1000 strands) | 309.8 / 268.3 | 113.0 / 37.6 |
| rope-bridge-3d | 5.06 / 4.40 | 2.16 / 1.25 |

What remains in hair-3d at one thread is mostly the rod solve (about 75 of 165 ms on x64, which the
contact search does not touch), the 100k or so capsule distance queries of the sweeps near the head,
and the candidate gather (about 10 ms a step, serial). The bridge touches no shape, so its cost was
all search.
