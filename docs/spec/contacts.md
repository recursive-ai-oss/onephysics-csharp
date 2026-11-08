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

`Enabled` starts true for every invocation. Disabling sets the solver point
count to zero and clears normal, tangent and rolling warm starts. The
geometric pair and its begin/persist/end lifetime are retained: disabling
response does not turn a contact into a sensor or an overlap query. A subsequent
step computes a fresh manifold and can enable response again. No disabled
point may contribute an impulse, including in a mixed enabled/disabled batch.

Edits to friction, restitution, separation and point count are ordinary contact
payload state, covered by existing snapshot v1 and world hashing. An optional
section with id `0x434D4F44` marks payloads that need fresh geometry on the next
step. Its entire payload is one little-endian int32 version, currently 1. The
section is present only after a modifier has visited at least one manifold;
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
