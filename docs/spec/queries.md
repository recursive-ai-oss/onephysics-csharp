# Extended queries

Queries read simulation state; no query buffers, cached query objects or scratch
are serialized or hashed. Querying concurrently with stepping, restore or edits
is not supported. Shapes and their slots follow the existing world query rules.
All distances are metres. Existing query limits and validation still apply.

## Mixed batches

`QueryInput2D` and `QueryInput3D` are immutable requests created by validated
factories: ray, convex shape cast, convex overlap, closest point and penetration.
The default request is invalid. Shape-specific queries can be expressed with the
corresponding convex shape. The middle-tier query object applies one filter to
the entire batch; world convenience methods use the default filter.

`QueryBatch` writes one nearest hit per input, or the default hit for a miss.
`QueryAnyBatch` writes one existence boolean per input. `QueryAllBatch` reserves
`hitsPerQuery` consecutive output slots for each input and writes a stored count
(not an untruncated count). Unused slots are cleared. Zero slots is valid. All
forms check buffer sizes and all requests before starting work. User output is
copied only after successful completion, including successful filter callbacks.

All forms preserve input order. Within an All result, existing scalar ordering
applies: overlaps by shape slot; rays and casts by fraction then slot; closest
points by distance then slot; penetration by decreasing depth then slot. A short
segment keeps the first hits in that order. Tasks contain 64 consecutive inputs;
each input has disjoint output slots. Thread count changes only task execution,
never arithmetic or reduction order. The total hit count reduces in input order.

Query objects retain arrays, growing only when a larger batch/output arrives.
Inputs are copied to those arrays before workers start; no pinning or escaped
spans are needed. Concurrent or callback-reentrant mixed batches on the same
object are rejected. Independent query objects own independent batch buffers.
Filter predicates must be pure and thread-safe, as with existing array batches.
After capacity and worker scratch warmup, buffers allocate nothing per call.

## Restricted queries and boxes

Body and shape handles expose ray, convex shape cast, convex overlap and closest
point queries. They validate the handle before querying, use normal query
filters and only visit the selected shape or the selected body's shape list.
A destroyed handle raises `StaleHandleException`; a default handle raises
`InvalidHandleException`. The result convention is the world query convention.

`QueryAabb` returns shapes whose stored broadphase proxy boxes inclusively
intersect the input box. It performs no geometry test and applies normal query
filters, including sensor exclusion by default. Boxes can be fat or stale until
the next step: this method is explicitly proxy selection, not an exact spatial
overlap test. Output is in ascending shape slot order; a short buffer keeps the
lowest slots. Minimum and maximum must be finite, in the existing world query
coordinate range, and ordered on every axis. Point and flat boxes are valid.

## Penetration

World penetration queries report strict overlap, not touching. `Distance` is
positive penetration depth, `Normal` points from the target toward the query
shape, `Point` is one contact point and `Fraction` is zero. Nearest means deepest
for this query. The planar solver uses its contact manifold's deepest point;
the spatial solver uses EPA. `ComputePenetration` tests two convex shape values
at given poses without a world: its direction moves B out of A. False clears
all output values. Existing low-level collision routines remain available.

For compound and triangle-soup targets, world queries report the deepest convex
leaf contact, with ties by lowest child or triangle id. That is **not** a promise
that one displacement separates the entire nonconvex union or every shape in
the world. `Depenetrate` supplies a bounded iterative alternative, described below.
Empty shapes miss penetration queries. Frusta, triangles, mass-centre decorators
and static half-spaces dispatch through the same convex routines. Two infinite
half-spaces have no finite contact and raise `ArgumentException`.

Penetration and angular sweeps are geometric queries: chain and terrain segments,
triangles and meshes are two-sided surfaces. Planar ghost-edge suppression and
one-way contact rules are solver behavior, not geometric separation constraints.
A plane is different: it represents a volume (a half-space), not just its surface.

### Bounded iterative depenetration

An exact union/world minimum translation cannot be obtained by choosing the
shallowest or deepest EPA result: corrections against one leaf can enter another,
and a nonconvex configuration-space obstacle can have disconnected escape routes.
Instead of claiming a global optimum, `Depenetrate` returns an explicitly
non-minimal, tested correction and a residual certificate. This is useful for
kinematic placement and compound/concave obstacles without hiding failure.

At the current trial pose, find the deepest penetrating leaf across the entire
selection (ties: target shape slot, then child/triangle id, or manifold point
order in 2D). If its depth is at most tolerance, return `Cleared`. Otherwise move
along its normalized outward normal by depth + tolerance/2, limited by the
remaining cumulative path budget. Requery all selected geometry at the new pose.
Bodies and shapes also expose this operation restricted to themselves; query
objects apply their filter. The world is never modified.

Defaults: 32 corrections, tolerance 0.0001 m, cumulative path budget 1000 m.
Accepted limits: 1..1024 corrections; finite tolerance in (0,1] m and finite
path budget in (0,MaxQueryDistance] m. A default result is `NotRun`, not success.
`Converged` means `Cleared` only. `RemainingDepth` is the maximum accepted leaf
penetration at the returned pose, not the depth of the last applied correction.
Iteration and distance exhaustion return their corresponding status and a tested
partial translation; a proposed correction outside the world query coordinate
range returns `CoordinateLimit` at the last valid pose. Unusable directions return
`DegenerateContact`. Impossible enclosures can oscillate to the iteration limit;
that is reported, never mistaken for a globally minimum or collision-free result.

## Whole-body translation sweeps

`SweepBody` and `Body.Sweep` sweep every source shape and, in 3D, every compound
child or soup triangle. Local placements and centre-of-mass offsets are applied
before dispatch to the existing shape-cast routines. The source body is always
excluded. With `excludeJointed`, direct live joint neighbours are also excluded;
exclusion does not propagate transitively. Target filters still apply. Source
shapes are geometry, regardless of their sensor/filter flags.

The first fraction wins, then lowest target shape slot; remaining ties keep
source shape-list and source leaf traversal order. Result normal, point and
starting-overlap conventions match shape casts. The source is not moved, woken
or otherwise modified. Static bodies may be swept as queries, including soups.
The overload accepting translation and angular displacement also supports pure
rotation and stationary overlap tests. Displacement is a signed angle in 2D or a
world-axis rotation vector in 3D, radians, with magnitude at most 64. Translation
length is at most MaxQueryDistance; zero is valid. The centre of mass translates
linearly while orientation follows constant angular velocity. Every source leaf
keeps its placement relative to that centre, including decorator and compound
offsets. Empty leaves are skipped. Infinite source planes support translation and rotation against finite targets;
plane/plane contacts are rejected with `ArgumentException`.
An infinite query mover bypasses finite proxy-box and reference-point reach
rejection. Compounds and soups enumerate their finite leaves for half-space
queries: an arbitrarily distant plane reference point can still enclose them.

### Continuous angular advancement

Mirtich 1996 §2.3.2 and Catto's 2013 continuous-collision talk give the separating
plane motion bound: closing translation along the current separating normal plus
|angular displacement| times a bound on the farthest source point from its mass
centre. We use core GJK witnesses to choose a normal, then compute the actual
support-plane gap. This projection is a lower separation bound, unlike the
approximate witness distance alone. Planar polygon face normals also provide
well-conditioned axes near contact; using the difference of nearly cancelled
radii-adjusted witnesses as a direction is avoided.

For each source/target convex-leaf pair, fraction starts at zero. At fraction t,
evaluate the exact sine/cosine rotation (onemath) and centre translation. A hit
occurs when GJK separation is within tolerance plus float roundoff margin.
Otherwise advance by 0.9 * (support gap - margin - tolerance/2) / closing bound,
clamped to the remaining fraction. Closing translation is projected in double to avoid rounding a nearly tangential
large motion to zero. Closing zero certifies a miss. Endpoint
checks alone and fixed motion sampling are never used; rotating into and out of
an obstacle between two clear endpoints is detected.

For a rotating half-space mover there is no finite point-speed radius. Instead
bound its signed support-distance derivative against the finite target: the
linear term is |translation| and the angular radius bound is
(target local-box corner norm + |rotation centre| + |translation|) * 1.0005.
The source plane's rotated offset arm cancels from this derivative. This detects
rotating planes without inventing finite plane corners or rejecting valid motion.

The finite-mover radius bound is (distance of leaf origin from mass centre + norm of the local
box's largest absolute corner) * 1.0005, covering near-unit pose rounding. The
roundoff margin is 8e-6 times max(1, local coordinate magnitudes of either shape,
current mover origin, initial offset arm and rotation centre in the target frame); artificial plane
proxy-box extents are excluded, using its plane offset instead. This is a
relative float-geometry accuracy allowance, not an absolute promise at extreme
scales. Tolerance is finite in (0,1] m, default 0.0001 m. Raw relative pose and
rotation-centre components are within +-2^25, and rotations must be near unit
(squared norm within 1e-4).

Each pair has at most 4096 advancement iterations. Nonpositive certified
progress, float-pose stagnation or iteration exhaustion throws
`QueryConvergenceException` with the last certified safe fraction; it never
reports a silent miss or an invented impact. This bound can be pessimistic for
near-tangent spinning shapes. Initial overlap reports time zero. The result
fraction is normalized time and Distance is the distance travelled by the mass
centre, hence zero for pure rotation. Earliest pair wins; ties retain target
shape slot, then source shape/leaf order. Leaf scans are iterative and use
separate reusable source and target triangle buffers, including reentrant-query
protection.

## Indicative measurement

`bar queries-indicative --threads N` builds the generated `queries` and
`queries-3d` scenes, warms the world, and settles each for its scene step count
at 1/60 second. It compares a scalar loop of nearest-hit calls against the mixed
span batch, separately for rays, circle/sphere overlaps and circle/sphere casts.
The overlap count is the number of queries with a hit, not the number of shapes
hit. Input requests are prepared before timing. Each operation warms four times
and reports the best of three measured repetitions. Hit counts must match or the
command fails. These are indicative comparisons, not publishable bar rows;
they do not measure reference engines or imply isolation from other workloads.

## Deformable queries

Ray, shape cast, overlap and closest point against soft bodies, cloth, strips and rods, in single, all and any forms, both dimensions (`RaycastDeformables`, `ShapeCastDeformables`, `SphereCastDeformables`/`CircleCastDeformables`, `OverlapDeformables`, `ClosestDeformable` and their `…All`/`…Any` forms on the world, the same through `PhysicsQuery*`, batches `RaycastDeformablesBatch`, `SphereCastDeformablesBatch`/`CircleCastDeformablesBatch`, `Overlap…DeformablesCountBatch`). They read the particles as they are between steps and change no state.

**Elements.** Soft bodies and cloth (3D) are queried on their surface triangles (the mesh's, or the boundary faces of a tetrahedral mesh) as bare two-sided triangles; a body with no surface is queried on its particles as spheres of the particle radius. 2D soft bodies are queried on the edges of their outline (strips and meshes without an outline: their edge list) as capsules of the particle radius, or on particles. Rods: each segment as a capsule of the rod's radius (a segment shorter than a millionth of its radius, or of zero length, is a sphere). **Cloth** means fabric or strip, or a body with no tetrahedra and no enclosed volume.

**Order.** Soft bodies and cloth by slot, then rods by slot; inside a body by element index. Hits sort by distance, then kind (soft body, cloth, rod), slot, element kind, index; equal distances are therefore stable. A buffer takes the nearest `results.Length`. `Any` forms stop at the first element found in scan order. Batches run in tasks of 64 inputs into disjoint output slots and equal the loop of single calls bit for bit at any thread count.

**Hit data.** `DeformableHit*`: kind, slot and generation of the deformable, element kind and index, the particles of the element (−1 for unused), barycentric weights of the hit point on them (for a point off a triangle's plane: its projection; outside the triangle a weight is negative), the point, the unit normal pointing at the query (zero if it started inside, and for overlaps), `Fraction` (of the cast length) and `Distance` (travelled; for closest-point queries the gap, 0 inside). A ray starting inside an element hits at 0.

**Maths.** The element shape is placed relative to the query origin in float32 (the double positions subtracted first), then the convex routines of the rigid queries run: `Gjk3D.RayCast`, `ShapeCast` and `Distance` in 3D, the 2D ray cast, `Distance2D.ShapeCast` and `Distance2D.Compute` in 2D. Bounding boxes cull in double. The scan is serial per query; callers' buffers and per-thread scratch make warm calls allocation-free.

**Filters.** `DeformableFilter*`: kinds, layer mask, category and mask bits, with the rigid-shape bit rules applied to the soft body's `Layer`/`CategoryBits`/`MaskBits` or the rod's. A shape that is a plane (3D) or empty (2D) is rejected with `ArgumentException`; inputs are validated as for the rigid queries (`ArgumentOutOfRangeException` naming the value).

**Fluids.** The particle-fluid queries (`OverlapFluidSphere`, `OverlapFluidBox`, `RaycastFluid`, `FluidDensityAt`) and shallow-water queries (`QueryWater`, `RaycastWater`) are separate calls with their own hit types, because a fluid has no elements to name; they exist in both worlds with identical names.

## Deformable contact events

`TrackDeformableContacts` (world property, default off, a host setting that is not state and not saved or forked). When on, before and after every step the world finds, for each soft body, cloth and rod (soft bodies by slot, rods by slot), the rigid bodies it touches: the broadphase boxes around the deformable's particles (grown by the radius and 0.05 m) give the candidate shapes in shape-slot order, filtered by the layer matrix, group, category and mask of the deformable; a shape is touched when a sphere of the deformable's particle radius (for rods also one at the middle of each segment) is within 5 mm of it. The sets before and after are compared in order (deformable kind class, slot, body slot): `Began` for bodies only in the later set, `Ended` for bodies only in the earlier; events fire after the step (unless callbacks are suppressed) through `DeformableContactChanged` and are polled with `DeformableContactEvents`. The earlier set is found from geometry at the start of the step, so a restored or forked world produces the same events.
