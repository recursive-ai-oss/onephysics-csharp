# Contacts and pre-solve modification

## Contact modification

Both dimensions expose `world.ContactModifier`, an optional delegate taking a
`ref ContactModification`. `SetContactModifier<T>` adapts a struct implementing
`IContactModifier` once at installation. The raw tier exposes the same delegate
on `world.Core.ContactModifier`; both properties name the same pipeline.
The stack-only view directly edits the constraint payload. A callback must not
mutate any other world storage, change callback registration, recursively step,
or depend on thread count or external nondeterministic inputs.

The engine completes narrowphase and all manifold writes first. It then visits
non-sensor manifolds serially on the stepping thread, before constraint
preparation, warm-starting or solving. Sort order is ascending shape A index,
shape B index, subshape A index, subshape B index, constraint index. Shape A has
the smaller shape index. Point order is the narrowphase's feature order.
Sleeping contacts are included. Geometry is recomputed while a modifier is
installed; manifold recycling cannot carry last step's modified separations or
coefficients into this callback. Removing the modifier forces one final fresh
contact update so the original material mix and geometry take effect again.

The callback can change Coulomb friction (0..1e15), restitution (0..1), each
point's finite signed separation in metres, and whether the manifold applies
impulses. Separations are negative inside. The normal points from A to B.
The normal's Z component is exactly zero in 2D. Invalid coefficients, indices or
non-finite separations throw `ArgumentOutOfRangeException` at the setter.

Fresh geometry starts enabled. Built-in one-way rules run before the user delegate, which may re-enable it. Disabling sets the solver point
count to zero and clears normal, tangent and rolling warm starts. The
geometric pair and its begin/persist/end lifetime are retained: disabling
response does not turn a contact into a sensor or an overlap query. A subsequent
step computes a fresh manifold and can enable response again. No disabled
point may contribute an impulse, including in a mixed enabled/disabled batch.

Edits to friction, restitution, separation and point count are ordinary contact
payload state, covered by existing snapshot v1 and world hashing. An optional
section with id `0x434D4F44` marks payloads that need fresh geometry on the next
step. Its entire payload is one little-endian int32 version, currently 1. The
section is present after a modifier has visited a manifold or while a surface/material change requires fresh contact mixing;
a step without a modifier refreshes those manifolds and removes the section.
Restoring or forking edited contacts without the original callback therefore
restores ordinary material and geometry rules on the next step. Snapshot v1's
core layout is unchanged. Worlds that never install a modifier keep identical
bytes and hashes. Callback code,
delegates and captured host state are not serialized, just as material mixers
are not serialized; hosts must reinstall them for continued simulation after
restoring into a fresh world. Structural forks have no callback subscriptions.

Recording a step with an installed modifier emits a named
`ContactModification` whole-state effect after simulation and before event
handlers, so playback does not need the original callback code. Worlds without
a modifier emit no extra effect and execute no contact sorting pass.

### Raw manifold batches

`world.Core.ContactBatchModifier` is an optional `ContactBatchModifier` delegate.
It runs once per step, on the calling thread, after built-in surface rules and
individual callbacks and before impulse clearing/clamping. `ContactManifoldBatch`
provides the sorted constraint slots, the raw core and `GetManifold(index)` mutable
views. Enabled flags are live, so this hook may re-enable a response disabled by
an earlier rule. Views retain the original geometry point count even when
response is disabled. Topology mutation and retained views are prohibited.
The recorder captures the resulting simulation effect just as for individual
modifiers. Neither callback subscription is serialized or copied by a fork.

### Tangent speed and normal impulse limits

The callback may set `TangentSpeed`, a finite relative surface velocity in
metres/second, and `NormalImpulseLimit`, a finite nonnegative maximum accumulated
normal impulse per point per substep (newton seconds). Defaults are zero and
`float.MaxValue` respectively. A zero limit prevents all normal response without
changing the point count. Limits clamp the warm start as well as solve, relax
and restitution. Tangent speed subtracts from relative velocity in the first
friction equation; positive speed makes B move in the first tangent direction
relative to A.

In 2D the first tangent is `(normal.y, -normal.x)`. In 3D form
`t = (normal.y, -normal.x, 0)` when `abs(normal.x) >= 0.57735`, otherwise
`t = (0, normal.z, -normal.y)`, normalize it to get tangent 1 and use
`cross(normal, tangent1)` for tangent 2. Thus a floor whose A-to-B normal is +Y
has tangent +X in 2D and -Z in 3D.

These two callback outputs are solver scratch regenerated on each invocation,
not persistent configuration. Their resulting impulses, positions and velocities
are already snapshot state. Extended solver arithmetic is bypassed when no
callback used either output, preserving the original default arithmetic,
including signed zeros. Scalar and wide kernels use identical lane operations.

## Body-frame surfaces

`ShapeDef.Surface`, `Shape.Surface` and `Material2D/3D.Surface` contain optional
`SurfaceProperties`; materials copy them through `WithMaterial`. `Velocity` is
a conveyor velocity in the owning body's frame, projected into the contact
plane. Relative surface target is A's rotated surface velocity minus B's,
projected onto the two solver tangents. User modification runs after these
built-in rules and may override them.

A nonzero `OneWayDirection` selects an allowed outward body-frame normal. The
collision normal towards the other shape must have dot product at least 0.5
with the normalized allowed direction; this rejects sides and undersides.
Relative point velocity away from the platform above 0.01 m/s disables response.
Otherwise response requires previous separation (current separation minus
normal relative velocity times the full step dt) at least -0.02 m, or an existing
positive normal warm-start impulse. Continuous collision likewise rejects sweeps that do not approach the allowed
side or whose hit normal fails the same 0.5 cosine test. This prevents a jump whose apex remains
inside the platform from acquiring response on its downward path, while a
falling body can land even if a discrete frame overshoots the plane.

In 3D, nonzero `FrictionAxis` selects a body-frame longitudinal axis. Shape A's
axis wins if both shapes provide one. Project it into the contact plane and
normalize (fall back to ordinary tangents if squared length <= 1e-12). Each
surface contributes coefficient `sqrt(longitudinal²*cos² + transverse²*(1-cos²))`
along each tangent; an isotropic surface contributes its ordinary coefficient.
The two surfaces combine by geometric mean on each axis. Project the tangent
impulse into the corresponding ellipse; a zero coefficient forbids impulse on
that axis. Tangential warm starts are cleared rather than reinterpreted in a
changed material frame. In 2D there is only one tangential linear degree of
freedom; ordinary scalar friction is its complete analogue, and anisotropic
axes are rejected.

`SpinningFriction` in 3D bounds torque about the normal by coefficient times
normal load. A separate accumulated scalar impulse drives relative normal-axis
spin to zero, clamped to ±coefficient times the sum of normal impulses. It is
reinitialized at substep preparation (no torsional warm start). There is no
normal-axis spin degree of freedom in 2D, so a nonzero torsional coefficient is
rejected there. Ordinary rolling resistance remains perpendicular to the normal.

Surface records occupy optional section `0x53555246`: int32 version 1, int32
record count, then strictly increasing shape-index records: shape index and
generation (int32 each), velocity XYZ, one-way direction XYZ, friction axis XYZ,
longitudinal coefficient, transverse coefficient, spinning coefficient (float32
in that order). Records are 56 bytes, little endian. Default records are omitted;
destroying a shape removes its record. Invalid values or stale shape references
are rejected on restore. The section is absent in ordinary worlds.

## Contact inspection and audio

`Body.Contacts`, `world.GetContacts(body)` and `world.Core.GetContacts(handle)`
iterate read-only `ContactView` manifolds in ascending constraint slot order,
without allocations. They include speculative contacts; sensors and disabled
zero-point responses are excluded. Views expire at world mutation and advancing
an enumeration validates the owning body. This is the contact-only companion
to broader body/constraint inspection.

`ContactMotionEnabled` is opt-in and false by default. It collects a maximum
pre-solve approach speed at manifold birth (`ImpactSpeed`), maximum residual
tangential point speed at step end (`SlidingSpeed`, relative to conveyor surface
motion), and relative rolling angular speed times the smaller dynamic contact arm
length (or the only dynamic body's arm) (`RollingSpeed`). Values are nonnegative metres/second. Contact events,
polled event arrays and `Collision` expose the same data. End events retain the
last data. The first contact may be speculative, so impact speed is at manifold
birth, not an extrapolation to exact geometric impact time.

Motion state is optional section `0x43444154`, present only while collection is
enabled. Payload: int32 version 1, int32 record count; each 37-byte record has
shape A index/generation, shape B index/generation, subshape A/B (six int32),
impact/sliding/rolling speed (three float32), then a bool live flag. Keys increase
lexicographically in that field order. Removed manifolds retain data for one
step to supply end events. This section restores the additive public event data;
the existing event serialization bytes and snapshot v1 core remain unchanged.

## Indexed materials and coplanar patches

`Shape.SetMaterials` and raw `Core.SetContactMaterials` copy a scalar material
table plus one index per kept triangle (mesh), triangle id (height field,
including holes), or one index for a segment. Empty spans remove the override.
The planar `CreateChain` material overload assigns one table index to each
segment, retaining its ghost vertices. Each triangle manifold resolves its
friction, restitution and rolling resistance before the normal combine rules;
points on different material regions never share a solver manifold. Indexed
scalar materials do not replace shape-wide `Surface` settings.

Indexed configuration occupies optional section `0x4D415453`: int32 version 1,
int32 shape-record count; each shape record has int32 shape index/generation,
int32 table length, table triples friction/restitution/rolling resistance as
float32, then int32 index count and int32 indices. Shape records strictly
increase by shape index. Table/index lengths and every coefficient/reference
are validated. Cached patch partitions are derived, not saved.

Active-edge flags retain the mesh/height-field convexity test described in
`collision3d.md`. An internal seam also needs coherent *manifold identity*:
otherwise crossing a triangle boundary discards warm-start support and applies
an artificial torque even with exactly vertical normals. At geometry build,
triangles connected across inactive edges with exactly equal oriented planes
`(normal.xyz, dot(normal, vertex0))` receive the lowest connected triangle id as
their coplanar patch id. A nonrecursive union-find with lower-root joins and
path compression derives these ids in linear storage. Query triangle ids remain
unchanged. Indexed materials further partition a plane by material index, each
partition using its lowest triangle id. Plane/material keys are reconstructed
in fixed numeric triangle order.

A front contact within cosine 0.99999 of a face with inactive edges uses the
face normal exactly. Coplanar contacts with the same child and material patch
merge into at most four points (Gregorius 2015, contact manifold coherence and
point reduction, slides 96 and 103–113). Selection starts at the lexicographically
largest support point in the non-soup body's frame, then farthest point,
largest triangle area, and largest added outside area. Selection ties keep
triangle/point order. Final ordering is lexicographic in that body's local
coordinates rounded to a 1e-5 grid **for ordering only**, not for positions or
separations. Reduced feature ids are `0xFC000000 + slot`. A five-comparator sort
network orders the four selections. This keeps warm-start slots stable across
flat diagonals instead of tying them to changing triangle clips.

The flat-floor regression uses a box sliding at 4 m/s across a 32×32-cell
triangulated floor after 60 settling steps, over 330 measured steps. Original
maximum absolute vertical speeds were 0.000647471 m/s (mesh) and 0.0004230275 m/s
(height field); the persistent coplanar patches reduce both to 2.2351742e-8 m/s,
below the 1e-4 m/s limit. An ordinary box floor measures 2.7939677e-8 m/s. These
are deterministic physical error metrics, not timing rows. Mesh/height-field
contact keys and resulting hashes intentionally change; core snapshot v1 does
not change, and geometry patch ids are derived on construction/restore.

### Inspection integration

Body contact iteration is a filtered projection of the same ascending-slot constraint enumeration exposed by body inspection: joints and zero-point response-disabled manifolds are omitted. `ConstraintInspection.Contact` returns the same `ContactView` used by `Body.Contacts`, `world.GetContacts(body)` and raw `Core.GetContacts(handle)`; there is no second contact representation for inspection. Typed world overloads reject foreign body objects even when the worlds were assigned the same numeric id. Raw handles retain their numeric world-id semantics.

The continuous-collision one-way normal test normalizes each vector separately before its cosine comparison, so large permitted direction magnitudes cannot overflow a product of squared lengths.
