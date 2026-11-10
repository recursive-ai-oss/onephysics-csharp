# Machine joints and break policy

Units are metres, seconds, radians, newtons and newton-metres. Creation validates
all parameters before allocating a joint. Connected-body collision defaults to
off. A filter joint has no solver rows; it still connects the simulation island.
Removing the last collision-suppressing joint makes the bodies eligible for
contact again. This matches the publicly documented filter-joint behaviour:
https://box2d.org/documentation/group__filter__joint.html .

## Typed views and the shared definition interface (2D)

The planar world returns `GearJoint`, `RackAndPinionJoint`, `PulleyJoint` and `PathJoint` from `CreateJoint(def)` and the `Add*` shortcuts, with the same members as the spatial views (`Ratio`; the path motor's `EnableMotor`, `MotorSpeed`, `MaxMotorForce`), implicit conversion to `Joint` and an explicit cast back that throws `InvalidJointException` for another kind. Every planar definition struct (`RevoluteJointDef` ... `ConfigurableJointDef`) implements `IJointDef2D` (`BodyA`, `BodyB`), as the spatial ones implement `IJointDef3D`.

## Spatial distance motor and fixed-joint springs

* **Distance motor.** `DistanceJointDef.EnableMotor`, `MotorSpeed` (rate of change of the length, m/s, positive lengthens) and `MaxMotorForce` (N, at least 0), also on the `Joint` and `DistanceJoint`. As in 2D the motor acts only while the spring is on and the length range (`MinLength` < `MaxLength`) is open: after the spring row of each substep it is a velocity row along the joint axis with an accumulated impulse clamped to `MaxMotorForce` times the substep, warm started like the hinge and slider motors. The joint keeps its payload slot `JP.Motor`; the stored joint record already held the speed, limit and flag, so snapshot bytes of existing worlds do not change.
* **Distance spring at 0 Hz.** With the spring on and the length range open, `SpringHertz` 0 applies no spring force (the length floats between the limits; the limits and the motor still act), as in the planar distance joint and the 3D hinge, slider and wheel. Earlier the zero-frequency spring coefficients turned the row into a velocity lock. The warm start still applies any stored spring impulse, so lowering the frequency to 0 mid-simulation releases it within a step. No stored layout changes.
* **Fixed-joint springs.** `FixedJointDef.LinearHertz`/`LinearDampingRatio` and `AngularHertz`/`AngularDampingRatio` (0 = rigid, the default), `Joint.SetFixedSprings` and `FixedJoint.SetFixedSprings`. A spring is Catto's soft constraint with frequency and damping ratio (as the other joint springs, `MakeSpring`): the three position rows and the three rotation rows are each solved as one block with the spring coefficients in the biased pass of a substep, and only the stored impulses are applied in the warm start; the relax pass skips them, as in 2D's weld. The linear spring is kept in the joint record's `Hertz` and `Damping`, the angular one in the `Swing1`/`Swing2` fields, which a fixed joint does not otherwise use (the snapshot record is unchanged). A fixed joint with a linear spring leaves the wide point-joint kernel (which only knows the rigid block) for the scalar path, like a joint whose body has axis locks.

## Planar machines

The existing revolute joint is both a point joint and the planar cone/limb
analogue. `AddCone` holds the anchors and limits relative rotation symmetrically
about the initial pose. A weld already has independently soft linear and angular
springs (`LinearHertz`, `AngularHertz`); no extra soft-weld type is needed. Twist
and two independent swing coordinates have no separate meaning in a plane.

New type values: filter 8, gear 9, rack-and-pinion 10, pulley 11, path 12,
configurable 13. Existing type values and records are unchanged.

* Gear: `omegaA + ratio * omegaB = 0`. Positive ratios counter-rotate. This is a
  velocity coupling, as in the spatial gear; supporting hinges are independent.
* Rack: `ratio * omegaA = dot(vB, axisB)`, axis attached to B. Ratio is metres per
  radian. Supporting hinge and slider are independent. Ratios must have magnitude
  in [1e-6, 1e6]; pulley ratios must be positive.
* Pulley: `lengthA + ratio * lengthB` is held between `MinLength` and `MaxLength`.
  Each length runs from its fixed world point to its body anchor. Negative default
  limits derive from `Length`, or the initial pose when Length is zero. Equal
  limits make a bilateral row; otherwise two nonnegative accumulated limit rows
  act speculatively. Zero-length segments use a zero gradient below 1e-4 metres.
* Path: uniform Catmull-Rom segments, parameter [0, segment count], repeated open
  endpoints or modular closed points. The nearest parameter scans eight samples
  per segment, keeping the lowest parameter on a tie, then bisects the derivative
  sign sixteen times in the best sample's neighbourhood. Compare the refined
  candidate with the coarse sample. A zero tangent uses local +x. The normal row
  holds B's anchor on the curve; optional tangent speed has a force cap; open
  endpoints have one-sided travel rows. Optional alignment holds B's local +x on
  the tangent. Points (2..256) are copied. Coincident and duplicate points are
  accepted unless no point differs from the first by more than 1e-6 metres.
* Configurable: x and y are attached to A, rotated by `LocalFrameAngleA`;
  angle is B minus A minus ReferenceAngle, in [-pi, pi]. Each axis is locked,
  limited, or free. Each can have a zero-target spring and speed motor. Axis order
  is x, y, angle. Within each: spring, motor, lower, upper, locked row last.
  Axis edits use LinearX=1, LinearY=2 and AngularZ=32, selecting exactly one bit.
  SetAxisMode, SetAxisLimits, SetAxisMotor and SetAxisSpring mirror the spatial
  vocabulary; SetAxis also accepts compact indices 0=x, 1=y, 2=angle.
  Each axis uses five persistent impulse slots: locked, spring, motor, lower,
  upper. Changing settings clears those five slots and wakes both bodies.

Rows use the existing soft-step mass/bias/impulse coefficients and accumulate
warm-start impulses. Gear and rack use rigid velocity rows (no positional bias).
The general row is `LA.vA + AA.omegaA + LB.vB + AB.omegaB`; its effective inverse
mass is the sum of the four squared, inverse-mass/inertia weighted gradients.
This follows Catto, *Understanding Constraints*, GDC 2014, slides 10–18 and 33–38
(https://box2d.org/files/ErinCatto_UnderstandingConstraints_GDC2014.pdf).

## Spatial machines

New type values: filter 13, wheel 14, motor 15, mouse 16. All use the existing
joint record without adding fields or changing old records.

* Wheel: frame A x is suspension and frame y is spin. The frame origins may slide
  along x. Two perpendicular translation rows and two spin-alignment angular
  rows hold the remaining freedoms. The suspension spring pulls towards zero
  translation, limits bound travel, and a capped relative angular motor drives
  spin about y. The spring acts in biased passes; limits are speculative.
  `AddWheel` takes a world suspension direction and chooses its perpendicular
  spin direction using the existing deterministic frame convention. Explicit
  frames let callers choose both directions.
* Motor: B's origin and orientation are driven towards LinearOffset and
  AngularOffset relative to A. LinearOffset is stored as frame A's position,
  AngularOffset as frame A's rotation, MaxForce as MaxMotor, MaxTorque as Ratio,
  CorrectionFactor as Length. Defaults: zero offsets, identity orientation,
  zero caps, correction 0.3. Angular rows precede linear rows. Each triple solves
  the coupled effective mass, then clamps the accumulated vector magnitude to
  its force/torque cap times substep duration. Pose bias acts in each solve pass,
  matching the planar relative-pose motor. Singular blocks produce no correction.
* Mouse: only B's anchor is pulled towards a world target. A is a graph endpoint
  and receives no impulse. Defaults: 5 Hz, damping ratio 0.7, zero cap. Target is
  stored in FxA/FyA/FzA, force in MaxMotor, frequency/damping in Hertz/Damping.
  The coupled point solve uses the existing spring coefficients, clamps the
  accumulated vector magnitude, and acts in the biased pass. Target shifts with
  the world origin. A shift that would put a new machine world target outside
  +-2^21 is rejected before changing state, so the resulting snapshot stays valid. `AddTarget` drags the point initially at the target;
  `AddMouse` drags the body origin.

Wheel, motor and mouse always require the full island substep schedule, as do
spatial gear, rack, pulley and path. Planar joints use the existing world joint
schedule. Coloring, batch ordering and island construction are unchanged.

## Whole-step breaking and lifetime

BreakForce and BreakTorque default to positive infinity. The break test uses
whole-step accumulated, actually applied impulses (warm start, solve and relax),
converted to force/COM torque by dividing by the full step duration. Take the
larger Euclidean magnitude of A and B for force, and independently for torque.
Torque includes the anchor-force moment. A strict `>` comparison triggers a break;
force takes precedence over torque. The float32 event value is the exceeded
magnitude, saturated at float32 MaxValue when the double-precision mean exceeds
that range, so finite impulses never create a non-finite break event.
Inspection uses the same accumulation. Finite thresholds enable collection even
when DetailedInspection is false; public inspection remains opt-in. Worlds with
no finite thresholds and no planar machine requiring readback keep collection
off. Threshold edits invalidate a per-world collection cache, not simulation state.
The older ReactionForce/ReactionTorque convenience properties retain their
last-substep meaning for existing joint types; inspection is the vector API.

LifetimeSteps defaults to zero (unlimited). Remaining lifetime decrements once per
completed world step, including sleeping joints, before break removal and event
sorting. A remaining value of one breaks that step, with event value zero unless
force/torque already requested breaking. No clock or variable timestep is involved.
The general handle and raw JointLifetime expose the remaining counter; editing it
is recorded for replay. Default/stale handles fail as for other joint properties.

Snapshot v1 layout is unchanged. Joint payload slots 32 and 33, previously unused
by joint solvers, store low/high unsigned 16-bit words of the nonnegative Int32
remaining lifetime as exact float32 integers. Snapshot reading validates both
words (high at most 32767) and their agreement with constraint flag bit 2
(HasLifetime). The flag avoids reading every immortal joint's strided payload. Zero remains two positive-zero floats, so all old
snapshots retain their bytes. No bit-cast floats, NaNs or inexact large integers
encode the counter. Built-in joint solver rows use slots below 32. An unflagged raw constraint's
payload remains opaque: lifetime decoding and validation do not reinterpret
legacy custom rows. Set lifetime through JointLifetime to opt into its encoding.

The planar optional section `0x4A324458` follows the engine section and is emitted
only with a live pulley, path or configurable joint. Its version is UInt32 1,
then Int32 record count, records in increasing constraint-slot order:

1. Int32 slot, byte joint type;
2. double fixedAx, fixedAy, fixedBx, fixedBy;
3. bool loop, bool align;
4. x, y and angle settings: byte mode, float lower, upper, bool motor, float
   speed, maxMotor, bool spring, float hertz, damping;
5. Int32 point count, then float x,y per point.

All primitives use the existing little-endian state writer. Records must match
the engine's joint types; required records cannot be missing. Definitions and
point arrays are independent in a structural fork. The section participates in
world hashing and full-state scenes/replay, and an old reader rejects its id.

## Authoring scene records

Existing scene records are unchanged. Joints additionally accept `data N ...`
(up to 1024 finite values), `lifetime STEPS`, and `break FORCE TORQUE` (nonnegative,
positive infinity means unlimited). Full-state scene v2 continues to preserve
all joint settings and warm starts without translation.

The optional scene parameter `param world-id ID` selects an explicit positive
world id when building an authoring scene; omitted, the id remains automatic.
Machine fixtures specify ids so adding their matrix rows cannot perturb another
scenario's automatically assigned event handles.

Type-specific `data` sequences (anchors still use the normal local anchor fields):

| dimension/type | ordered values |
|---|---|
| either/filter | none |
| either/gear | ratio |
| 2/rack | ratio, local axis B x,y |
| 2/pulley | ratio, length (zero derives), fixedAx,Ay,Bx,By |
| 2/wheel | local axis A x,y, hertz, damping |
| 2/path | loop (0/1), align (0/1), point0 x,y, ... |
| 2/configurable | frame angle A, reference angle, x settings, y settings, angle settings |
| 3/pulley | ratio, fixedAx,Ay,Az,Bx,By,Bz |
| 3/wheel | hertz, damping; frame rotations are identity |
| 3/motor | desired quaternion x,y,z,w, maxForce, maxTorque, correction; anchor A is LinearOffset |
| 3/mouse | target x,y,z, hertz, damping, maxForce |

A configurable axis setting is nine numbers: mode (0 locked, 1 limited, 2 free),
lower, upper, motor enabled, speed, cap, spring enabled, hertz, damping. Wheel/path
motors and wheel limits also accept the existing motor/limit options. More general
frames and pulley ranges are represented exactly by full-state scene v2.

## Machine stability fixtures

`joint-machines-2d` and `joint-machines-3d` use seed 0x4d414348494e45, eight
separated stations, zero gravity, and 600 steps at 1/60 s, dumping every step.
A force-zero filter expires after 150 steps, including across restore. They combine the new
families with supporting joints. No reference engine is driven for this combined
feature scene. Thresholds: no NaN/infinite states, no bodies escaping the declared
container, maximum body speed <= 4 m/s. The analytic tests separately check gear
and rack speed residuals <= 0.001 after initial convergence for 1000 steps,
pulley length within 0.02 m, suspension static deflection within 10%, motor pose
within 0.001 m, and mouse force within 0.0001 N of its vector cap.

Indicative regression checks use three alternating baseline/changed processes,
600 fixed steps with the first 60 discarded, median step duration per process
and the best of three medians. Both sides run at one thread on the same pinned
logical core with tiered compilation disabled. Loaded-host indicative results
are not publishable bar rows.

## Ropes over wheels, belts and chains

The pulley, gear and rack joints above are two-body constraints. A rope that runs over several wheels, a winch, a belt or
a chain connects any number of bodies and has capstan friction and a conserved length, so it is not a joint: it is a pulley
system, specified in `rods.md` (Pulley systems), with its own optional snapshot section and its own stage before the rigid
step. Break thresholds, force and torque readback and the host hooks follow the same rules as joints.
