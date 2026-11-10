# Fields, effectors and planar fluids

Both worlds expose `AddField(FieldDefinition)` and generation-checked `Field` handles.
Happy-path helpers create area, point, platform, surface and buoyancy effectors,
radial gravity, gravity zones, custom gravity, magnetism and vortices. Definitions
are copied on read/write. `GetField(slot)`, raw body handles and the standalone
`Buoyancy2D.ComputeSubmergedArea` routine form the raw boundary. Planar definitions
use XY positions and vectors (a vortex uses a +Z axis); spatial definitions use XYZ.
Positions cross the boundary as double metres, within inclusive ±2^21. Stored
coordinates may reach ±2^22 through origin shifting; shifts prevalidate all fields.
Definition replacement may retain shifted coordinates unchanged while editing
other properties; newly authored coordinates still obey the creation bound.
All other numeric properties are finite, bounded by 10^12 in magnitude; radius,
softening and blend weight are positive, coefficients nonnegative. Axis squared
length differs from 1 by at most 10^-5. Restitution is within 0..1.

## Selection, order and time

A region includes its minimum and maximum. Selection is by body centre of mass,
not its shape bounds; water clips shapes against its surface after that selection.
At least one nonsensor shape must be on an affected `Layers` bit. An optional
`Sensor` slot further requires a sensor overlap in the last completed contact
update. This means newly entered force-field sensors act on the next step. Sensor
generations are captured when the field is created or updated; a deleted/reused
sensor never silently retargets a field. Collision filtering still controls which
sensor overlaps exist.

All force effects run serially before the rigid step, in ascending field slot,
then body slot, then shape creation order. User forces already accumulated are
added to, not replaced. Forces remain constant through the world's substeps and
are cleared by the rigid engine. Worlds that never use fields run no force pass and add no
snapshot bytes. The contact adapter runs just after contact generation, in field
then constraint-slot order. No order depends on a worker count or vector width.

A sleeping body wakes if the single effect's predicted linear speed change
`|a| dt` exceeds its linear sleep threshold, or its predicted angular speed change
exceeds its angular threshold. Explicit body thresholds override world thresholds;
otherwise both use the world sleep threshold. Below-threshold effects on sleeping
bodies are discarded, not accumulated for a later wake. Awake bodies receive all
forces. An explosion explicitly wakes its impulse recipients like `ApplyImpulse`.

## Force laws

* **Area:** `Direction` is acceleration (default) or force when `Acceleration` is
  false. Drag adds `(Flow - velocity) drag / (1 + dt drag)` acceleration. Angular
  drag uses the same implicit damping rate, converted through world inertia.
* **Point:** positive strength repels; negative attracts. With `r` the source to
  body vector and `s=max(|r|,Softening)`, magnitude is `Strength`, `Strength/s` or
  `Strength/(s*s)` for constant, inverse-linear or inverse-square falloff. Direction
  is `r/|r|`; coincident points receive zero. Radius is an inclusive hard cutoff.
  Force/acceleration selection is the same as an area.
* **Radial gravity:** the point law, inward for positive strength, always an
  acceleration. `Line=true` first projects the displacement perpendicular to the
  line through `Center` parallel to `Axis`. It is additive to world gravity: set
  world gravity to zero for an isolated central-force orbit.
* **Regional gravity:** matching zones of the largest priority replace world
  gravity. Equal-priority vectors blend with normalized positive weights; weighted
  sums accumulate in double, in field-slot order. Body gravity scale applies.
  The replacement minus base gravity is accumulated once at the first zone slot.
* **Custom gravity:** a pure delegate returns replacement gravity, m/s²; body
  gravity scale applies. `CallbackId` is saved, but delegates and mutable callback
  state are host context, as collision predicates are. Bind identical callbacks
  before stepping a loaded scene, fork or replay world. Missing bindings throw a
  clear `InvalidOperationException`; nonfinite results throw an argument error.
  Middle-tier struct callbacks adapt once at binding, never once per body/step.
* **Vortex:** project source displacement perpendicular to the axis. At unit radial
  direction `n`, acceleration is `Strength cross(Axis,n) - Inward n`. Positive
  strength is right-handed about the axis (+Z gives counterclockwise XY). The axis
  itself receives zero. Radius limits perpendicular distance, not axial distance.
* **Magnetism:** bodies explicitly carry world-oriented dipole vectors; zero is
  untagged. With `n=(pA-pB)/distance`, `a=dot(mA,n)`, `b=dot(mB,n)`, the force on A is
  `3 Strength / s^4 * (a mB + b mA + (dot(mA,mB)-5ab)n)`, where `s` is the softened
  distance. This is the gradient of dipole energy with the physical permeability
  constant absorbed into Strength. Coincident pairs receive zero. No dipole torque
  or automatic dipole reorientation is applied. A cutoff-sized grid, sorted lexicographically by integer cell X/Y/Z then
  body slot, bounds each cell to 64 dipoles and each 27-cell neighbourhood to 1728
  candidates. Larger occupancies throw before this field applies any force; use a
  smaller cutoff or fewer tags. Magnetic cutoff is 0.0001..2097152 m. Cell indices
  are floor(position/cutoff). Neighbour cell lookup uses binary search, and gathered
  candidates sort by body slot before summation. Bounded occupancy makes the
  evaluation O(n log n), without a dense quadratic fallback. Untagged bodies do
  not participate. Generation checks prevent tags transferring to recycled bodies.

## Results beyond the limit

Definitions are validated when they are created (every scalar finite within ±1e12), but what a field
produces at a body depends on distance, velocity, mass and host callbacks. An application whose
acceleration, force or torque is not finite, or has a component beyond ±1e12, is dropped before anything
changes: no force, no wake. The step carries on with the other bodies and fields, and
`StepCounters.FieldApplicationsSkipped` counts the drops of the last step. This covers a gravity callback that
returns NaN, infinity or a huge vector, and a valid definition whose terms multiply past the limit (for
example strength 1e12 over a softening of 1e-30 at the centre). A custom gravity field whose callback id has no delegate bound (a fork, or a snapshot restored into a fresh
world) is dropped the same way until the host binds one. A field step therefore never throws from
inside the step.

## Contact effectors

A platform/surface region selects the static or kinematic **surface body** origin
and layers. Contacts between two dynamic bodies are not modified.

A platform is solid toward `Axis`. The contact normal pointing from surface to
other body has dot product below -0.5 with Axis on the underside: its points,
impulses and payload are cleared for this update. Dot product in [-0.5,0.5)
is a side: `SideFriction` and `SideBounce` replace the mixed values. Top contacts
retain the ordinary material. Side coefficients are restored after the rigid
step, so removing an effector never leaves a cached material override behind. This minimal geometric one-way policy does not latch
bodies through large teleports or suppress collision events.

A surface effector is a contact-only conveyor drive. Direction is projected into
the contact tangent plane. It adds mass-proportional force toward target tangent
velocity `Strength * projectedDirection` with rate `Drag` (helper default 20/s).
The normal contact solver is unchanged; friction can oppose the drive. Each
manifold contributes one drive. It is a finite-force conveyor, not an infinitely
strong velocity prescription. It does not require changing a material.

## Planar buoyancy

`AddWater`/`AddFluid` mirror the spatial vocabulary. A buoyancy field is also a
fluid; `FluidCount` counts both. Density is kg/m² for unit-depth planar bodies.
The occupied half-space is `dot(Axis,p) <= SurfaceHeight`.

Polygon cores clip against the plane edge by edge. Cross products give twice the
area and the first moments give the centroid (the shoelace integral). Circles use
analytic circular-segment area and centroid. Capsules and rounded polygons use
128 fixed support directions, sine/cosine from OneMath, stable vertex-index ties;
the directions are cached on first use as pure derived data, not saved state.
This approximates their curved boundaries (a full circle would have about 0.04%
area error, although circles themselves use the analytic path). Segments have
zero area. Duplicate outline support points are harmless in the moment integral.

Each shape contributes `-density * submergedArea * gravity * BuoyancyFactor` through
its centre of buoyancy. The factor defaults to 1; zero disables lift but not drag.
Viscous drag is `density * area * Drag` times relative velocity. Optional quadratic
drag is `0.5 * density * QuadraticDrag * sqrt(area) * speed` times relative velocity.
`sqrt(area)` is the documented projected-width approximation. Drag acts at the
centre of mass, with a shared per-body cap of mass/dt across shapes, so one fluid
cannot reverse the current relative velocity in one step. Angular drag uses the
implicit rate once per body. Buoyant torque is the moment arm cross lift.
Spatial buoyancy fields reuse the existing clipped-volume buoyancy implementation.

**Evaluation pose.** The force is applied once per step and held across the substeps, which would be explicit
in position and feed energy into an undamped floating body (growth about 4% per period at 4 substeps).
Both dimensions therefore cut the hull at the pose the substeps will have reached by the middle of the step:
position advanced by `v * lead` and orientation by `omega * lead` (first order, renormalised), with
`lead = dt * (n - 1) / (2n)` and `n = Substeps` (`n = 1` gives no lead). Islands on a shorter schedule are
over-compensated, which only adds a little damping. An undamped box released at rest stays within its release
amplitude plus 1% over 40 s. Saved state may hold a finite rotation that is not unit length; the plane normal is
renormalised and a plane that cannot be formed cuts nothing. Torque and force have no magnitude limit (a large
heavy spinner is valid); a non-finite result applies nothing.

## Explosions

`Explode(center, impulse, radius, occluded=true)` visits dynamic bodies by slot.
Impulse units are N s, not acceleration. Falloff is the point law with softening
0.1 m. The centre-coincident direction is +Y. With occlusion disabled every selected
body receives the full impulse. Otherwise exactly five rays sample the centre of
mass and ±one-quarter body extent on world X and Y. Static, nonsensor geometry is
opaque; kinematic/dynamic geometry is ignored. Each visible sample contributes 1/5
of the impulse. Ray ties follow the query's stable shape-slot ordering. This is a
fixed sampling approximation to visibility, not volumetric blast propagation.

## Saved state and tools

Optional snapshot section `0x464C4453`, version 1, leaves snapshot container version
1 unchanged. Little endian, in this order:

* version i32, field-slot count i32;
* each slot: generation i32, live bool; for a live slot: kind i32;
  minimum, maximum, centre (three f64 each); direction, axis, flow (three f32 each);
  strength/radius/softening f32; falloff i32; acceleration/line bool;
  drag/quadratic-drag/angular-drag/density f32; surface-height f64; inward f32;
  priority i32; weight f32; layers u32; sensor slot/generation/callback id i32;
  side-friction/side-bounce f32;
* body-tag count i32; each slot: body generation i32, dipole (three f32), planar
  buoyancy factor f32 (1 by default). Spatial buoyancy factors remain in the
  existing spatial fluid state.

Removed field slots retain their generations. Lowest dead slot is reused first;
generation exhaustion throws before destruction. Definitions, tags and every
force/impulse consequence are covered by snapshot, hash, exact-state scene v2,
mutation recording and replay. Fork copies state but clears custom callback
bindings. Restoring into an existing world retains its host callback bindings.
The desync report labels field and magnetic-body slots. Field state is global
context for regional snapshots. `DebugContent.Fields` emits bounds, axes, source,
radius, strength and surface offset, in slot order, as `DebugField` plain data.

## Reproducible checks

`FieldsTests` checks analytic acceleration/falloff, weighted gravity zones, a
1,000-step orbit, dipole signs, vortices, static-wall blast occlusion, analytic
clipped moments, float draft/righting, contact effectors, host hooks and allocation.
The matrix adds `world2d/fields` and `world3d/fields` with explosions, fork/restore
and origin shifting. The smoke apps use these same scene constructors.

`bar fields-stability` runs two scenes for 600 steps at 1/60 s:

* **buoyancy-2d:** twelve separate half-density circles and rotated boxes, a deep
  bed, still water and two weak magnetic tags. maxSpeed ≤10 m/s, final centre-height
  error ≤0.02 m (half-area draft), summed final kinetic energy ≤0.05 J.
* **fields-3d:** a circular orbit, an isolated vortex/zone and two dipoles.
  maxSpeed ≤20 m/s, orbit radius error ≤0.03 m.

Every reported value must be finite. No managed comparison engine is driven for
these feature-specific scenes; the thresholds are analytic/own-behaviour checks,
not comparative performance claims.
