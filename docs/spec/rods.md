# Circular Cosserat rods

The experimental rod solver is shared by both dimensions. It currently provides
stretch, shear, bend and spatial twist, world pins, driven particle targets and
material frames. **Collision, rod/self contacts, rigid-body attachments and force
breaking are not implemented yet.** Do not use this version where ropes must
remain outside obstacles. All rod APIs carry the raw-tier experimental marker.

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

Each nonlinear iteration assembles J*M^-1*J^T+compliance/h² and right-hand side
-C-(compliance/h²)*lambda. Groups have six rows: boundary first, then each
segment's XYZ stretch and XYZ bend/twist. Only adjacent groups share degrees
of freedom, giving lower bandwidth 11. Lower-band Cholesky, forward and back
substitution use ascending row/column indices (back substitution descending).
A pivot is bounded below by 1e-30 against roundoff. Reductions and multiplier
updates use double. Corrections accumulate in row order per particle/frame
and apply once. The default is 8 substeps and 4 nonlinear iterations. Rods solve
in ascending slot order after existing rigid/soft phases, serially and
independently of thread count. No scratch allocation occurs during a step.

## State and tooling

The optional snapshot section `0x524F4453` (RODS), version 1, is absent until
rod slots exist. No existing section layout, hash or phase order changes when
rods are unused. Little-endian fields:

- version, slot count;
- per slot: generation, live flag;
- per live rod: E, G, radius, density; substeps, iterations, damping, gravity
  multiplier; ClampStart flag and root-frame XYZW; particle count;
- per particle: XYZ double position, XYZ double pin target, XYZ float velocity,
  pinned bool, inverse mass;
- per segment: XYZW frame, XYZ angular velocity, rest length, XYZ rest Darboux
  vector, clamped bool, transverse and axial inverse inertia;
- six double multipliers per group, including the leading boundary group.

Floats, bools, doubles and counts use the existing primitive encodings. The
reader checks counts against remaining bytes and the world limits, positive
generations, finite state, bounds, unit frames and planar components before
replacing state. A rejected whole-world restore is atomic. Previous positions,
old frames and solver factors are scratch, rebuilt before use. A fork copies all
state and owns independent scratch. Region membership selects a whole rod if
any particle lies in the inclusive region; restore requires compatible selected
slots/generations and field topology. Desync labels identify the rod slot;
repeated particle/segment fields use deterministic occurrence indices.

Exact-state scenes and recorded mutations use the same section. Origin shifts
validate positions and targets before translating them. Debug data emits one
segment with its endpoints, radius and material quaternion, in slot/segment
order. Raw span writes bypass recording and validation; invalid raw state is
rejected at the start of Step with RodStateException before rigid stepping.

## Analytic checks

The targeted tests compare a 32-segment rope against the catenary with span 2 m
and arc length 2.4 m (sag tolerance 5%, relative stretch below 1%). A clamped
1 m beam under its own weight uses the small-deflection result q*L^4/(8*E*I),
8 and 16 segments and 2/4/8 substeps (tip tolerance 8%). The finite-element
model includes shear, unlike Euler–Bernoulli theory. A free planar pendulum
runs 10000 steps without explicit damping; maximum total mechanical energy
may not exceed its initial value by 0.001 J. These checks do not establish
collision stability, rigid coupling, plectonemes or production performance.
