# 3D convex collision core

Normative description of the 3D collision code (namespace `RecursiveAI.OnePhysics.LowLevel`, the experimental raw tier; the shape and hull types a world takes in queries, and the exceptions, are in `RecursiveAI.OnePhysics`): support-mapped convex shapes, GJK distance and casts, EPA penetration, Quickhull hulls and mass properties. A port that follows this text and the arithmetic rules below produces the same bits.

## Arithmetic

float32 for shapes, queries and results; `+ - * /` and the correctly rounded square root only (no fused operations, no estimates, no transcendental functions). The sums in a dot product run `x*x + y*y + z*z` left to right; cross products are `(ay*bz - az*by, az*bx - ax*bz, ax*by - ay*bx)`.

Two places compute in double, because float32 loses too much there: Quickhull (all geometry), and the closest-point-on-simplex solver of GJK (`Simplex`) together with its tetrahedron side tests. Inputs are float32 and exactly representable; results are rounded back to float32 once.

Directions passed to support mappings may have any length. Where a support mapping needs a unit direction (cylinder, tapered capsule, convex radius) it first divides the direction by its largest absolute component, so tiny and huge directions neither underflow nor overflow, then takes the length of that scaled vector.

Frames: every shape lives in its own local frame, origin at its centre. Pair queries take `bInA`, a rigid transform (unit quaternion plus translation) from B's frame to A's, and answer in A's frame. The rotation matrix is `Rotation.ToMatrix()`; B's support in A's frame is `R * s_B(R^T * d) + t`.

## Shapes

A shape is a *core* plus a *convex radius* `r >= 0`: the set of points within `r` of the core. Queries run on the cores and add the radii back.

| Kind | Core | Radius | Support of the core in direction d |
|---|---|---|---|
| Sphere(radius) | the origin | radius | origin |
| Box(h, cr) | box of half extents `h - cr` | cr | `(d.x < 0 ? -hx : hx, ...)` per axis (`d = 0` picks the positive side) |
| Capsule(hh, r) | segment `(0, ±hh, 0)` | r | `(0, d.y < 0 ? -hh : hh, 0)` |
| TaperedCapsule(hh, rt, rb) | the hull of the two spheres (radius 0) | 0 | the farther of `c + r * u/|u|` for the top sphere (`c = (0, hh, 0)`) and the bottom sphere; compared by `±hh*u.y + r*|u|`; the top wins ties |
| Cylinder(hh, r, cr) | cylinder of radius `r - cr`, half height `hh - cr` | cr | `((r-cr) * (d.x, d.z)/|(d.x, d.z)|, d.y < 0 ? -(hh-cr) : hh-cr)`; on the axis the xz part is zero |
| Hull(hull, cr) | the hull | cr (outward inflation) | the vertex with the largest dot product; the lowest index wins ties |
| Point | the origin | 0 | origin (a default `ConvexShape3D`) |

The full support of a shape is the core support plus `r * u/|u|`. A box or cylinder given a convex radius keeps its outer size (the core shrinks); a hull's radius inflates it.

Limits: every dimension is finite and in `(0, 2^21]`; a convex radius in `[0, smallest outer half size]`; a tapered capsule's radii differ by less than twice the half height (otherwise one sphere contains the other). Violations throw `ArgumentOutOfRangeException` naming the parameter, value and allowed range.

### Mass properties

Unit density, in the shape's frame, as `MassPropertiesF` (mass = volume, centre of mass, inertia about the centre). Sphere, box and cylinder use `InertiaF.SolidSphere/SolidBox/SolidCylinder`. Capsule and tapered capsule are solids of revolution about Y: the profile is cut into the lower cap, the tangent cone frustum and the upper cap, and volume, first moment and the inertia integrands (polynomials of degree at most 4 in `y`) are integrated in double with 3-point Gauss–Legendre quadrature per piece, which is exact. Boxes and cylinders with a convex radius report the unrounded outer shape; a hull's inflation radius is ignored. A hull sums tetrahedra (vertex mean, polygon fan triangle) using the simplex second-moment formula `V/20 * (sum p p^T + (sum p)(sum p)^T)`.

## GJK (`Gjk3D`)

Support of the difference `D = A - B` in direction `u` is `s_A(u) - s_B(-u)` on the cores. `Simplex` holds up to four such points with the witness points on A and B.

Distance loop (`Run`): start from the support in the direction of B's translation (or +X when B is at the origin); each iteration asks for the support in `-v`, stops when `v.v - v.w <= max(2e-6 * v.v, 4e-7 * |v| * sqrt(maxW2))` (`maxW2` is the largest `|w|^2` seen), when `w` is already a simplex vertex, when adding `w` does not reduce `|v|`, or after 48 iterations. The cores count as overlapping when the simplex encloses the origin or `v.v <= (2e-5)^2 * maxW2`. A tetrahedron whose closest-point test says "inside" while `v.w > 0.5 * v.v` is a numerical sliver and is discarded.

Closest point on a simplex: segment by clamped projection; triangle by Voronoi regions of the origin (Ericson, *Real-Time Collision Detection*, 5.1.5), falling back to the nearest edge when the triangle is degenerate; tetrahedron by testing each face's side with the fourth vertex (a face with squared normal below `1e-10 * scale^4`, or a fourth vertex within `1e-6 * scale` of its plane, always competes) and taking the nearest face triangle in index order `(0,1,2) (0,2,3) (0,3,1) (1,3,2)` (ties to the first).

`Distance` returns the distance between surfaces `|v| - rA - rB`, the closest points (moved out by the radii) and a unit normal from A to B. Cores that overlap, or surfaces within the radii, report `Overlapping` with distance 0.

Accuracy: distance error below 1e-6 of the shapes' size on random hull pairs (measured 3e-7 over 10^5 pairs).

### Casts

Shape cast and ray cast follow van den Bergen (2004): the origin moves along `r = -motion` against `D` inflated by the radii. Per iteration: `p = s_D(v)`, `sep = n.(x - p) - R` with `n = v/|v|`; if `sep > tol` the point advances `lambda -= sep / (n.r)` (no hit if `n.r >= 0` or `lambda > 1`); `p` joins the simplex unless it is already there; the closest point of `x - P` becomes the new `v`. It stops when `|v| <= R + 4 * 4e-7 * scale`, when `p` repeats without `x` moving, or when `x` moves nowhere and `|v|` does not drop; 128 iterations at most. Fraction error on random hulls is below 2e-4 (motion length 8).

Results: `Fraction` in `[0, 1]` of the motion, `Normal` (A to B for shape casts; the outward normal of the hit shape for ray casts), contact points, and `StartsOverlapping` when the hit is at fraction 0 with the shapes touching or overlapping. `RayCast` is a shape cast of a `Point` shape; `maxFraction` scales the direction.

## EPA (`Epa3D`)

Penetration depth and direction when the cores overlap (van den Bergen 2001; Ericson, ch. 9). Start: the GJK simplex, grown to a tetrahedron by probing supports along the axes, the perpendiculars to the segment, or the triangle normal (a Minkowski difference without volume, such as two crossing segments, has depth 0 and a normal along its thin direction chosen towards B). Faces carry the vertex triple, neighbour faces, unit outward normal and plane distance (clamped at 0). Each iteration takes the live face with the smallest distance (lowest slot on ties), asks for the support along its normal and stops when `n.w - dist <= 4e-6 * scale`. Otherwise the faces connected to it (by shared edges) that see `w` by more than `1e-6 * scale` are removed and the horizon, which must be one simple loop with non-degenerate triangles, is joined to `w`; if it is not, expansion stops. Capacity is 128 vertices.

If the closest-face distance falls by more than `1e-4 * scale` the polytope has been broken by round-off, and the best distance seen is kept. If the vertex budget ran out before convergence (curved shapes), the depth is refined by descending the support distance `h(n) = n.s(n)` over directions (up to 16 backtracking steps), which only ever returns an upper bound that is consistent with the polytope's lower bound.

`Epa3D.Collide` returns the signed separation (distance when apart; `-(depth + rA + rB)` when overlapping), a unit normal from A to B, and the deepest points of A and B (barycentric points of the closest face; support points after refinement).

Accuracy: polyhedra match a separating-axis search in double to 5e-6 in depth and 1e-6 in direction (10^5 random overlapping hull pairs). Curved shapes are limited by the vertex budget: usually below 1e-4, worst measured 0.5% of the depth. Contact generation for spheres, capsules and cylinders should use dedicated routines.

## Quickhull (`ConvexHull3D.Build`)

Input: 4 to 2^22 points, finite, each coordinate in `[-2^21, 2^21]`. Errors are `HullBuildException` with `HullBuildError` and a message naming the input index or option; it derives from `ArgumentException`.

Tolerance `eps`: the option, or `3 * 2^-23 * (extent x + extent y + extent z)`, but at least `1e-14` times the sum of the largest absolute coordinates. A point within `eps` of a face plane counts as inside.

1. **Initial tetrahedron.** Candidates are the lowest-index minimum and maximum along each axis. The pair with the largest distance (first pair in `(0,1) (0,2) ... (4,5)` order wins ties) gives the first edge; the third point is the one farthest from its line, the fourth the one farthest from the plane (lowest index on ties). A distance not above `eps` is `Degenerate` (coincident, collinear, coplanar).
2. **Expansion.** Each point is assigned to the face it is farthest above (by more than `eps`). The face stack is processed last-created first; the apex is the face's farthest point. Faces visible from the apex (by more than `eps * 1e-3`) are found by walking neighbours; if the horizon is not one simple loop, faces with two visible neighbours are added and the horizon is recomputed once, otherwise the apex is skipped. The horizon is walked in order, new faces `(a, b, apex)` replace the visible ones, and orphaned points go to the new face they are farthest above. `MaxVertices` / `MaxFaces` stop the process before an apex would exceed them, counting the vertices the step would remove.
3. **Merging.** Adjacent triangles join when each lies within `eps` of the other's plane (a triangle whose smallest altitude is within `eps` joins only the neighbour across its longest edge, judged by that neighbour's plane). Joined groups are unioned with the lowest triangle index as root. A group becomes a polygon by chaining its boundary edges; if that is not a single loop the group stays as separate triangles. A vertex on exactly two polygons is not a corner and is removed from both; a polygon left with fewer than three vertices is dropped.
4. **Canonical form.** Polygons start at their lowest vertex index and are ordered by their first three vertex indices. Vertices are the used input points in input order. Face planes use Newell's method about the polygon mean; offset is the mean of `n.v`.
5. **Half-edges.** Directed edges are sorted by `((min*V + max) * 2 + (from > to ? 1 : 0))`; each undirected edge is two adjacent half-edges, so the twin of `e` is `e ^ 1`. The result must have `V - E + F = 2`.

`ConvexHull3D` exposes `Vertices`, `Faces` (`Normal`, `Offset`, `FirstEdge`, `EdgeCount`), `Edges` (`Origin`, `Next`, `Face`), `VertexEdges`, `MassProperties` and the support functions. The hull is immutable.

## Determinism check

`tests/OnePhysics.Tests/Collision3D/Collision3DHash.cs` builds 24 hulls and runs 600 random shape pairs (all kinds, convex radii, transforms) through `Distance`, `Collide`, `ShapeCast`, `RayCast` and `GetMassProperties`, folding every output float's bits into an FNV-1a (64-bit) hash, seeded with SplitMix64 streams. The unit tests, the NativeAOT smoke app and the WASM smoke app check it against one constant.

## Contact manifolds (`ManifoldGenerator3D`)

`Collide(a, b, bInA, speculativeDistance = 0.02)` returns a `Manifold3D`: a unit normal from A to B and at most four points, all in A's frame. A point has a position midway between the two surfaces, a separation (surface distance along the normal; negative when overlapping) and a 32-bit feature id. Points with separation above `speculativeDistance` are dropped; an empty manifold means "too far apart". The four-point limit and the normal, separation and position fields fit the shared core's 3D contact layout (`DimensionLayout.Spatial`) unchanged. Input checks: the rotation's squared length within 1% of 1, each translation component at most 2^22 in magnitude (else `ArgumentOutOfRangeException`).

Arithmetic as above (float32, `+ - * /` and sqrt, fixed summation order). Dot and cross products are the plain left-to-right forms.

### Routing

| Pair | Method |
|---|---|
| sphere or capsule against sphere or capsule | analytic segment–segment: closest points of the core segments (a sphere is a zero-length segment), radii added back; parallel segments (cross product squared at most 1e-8 of the product of squared lengths) that overlap give the two ends of the overlap |
| sphere against box or hull | analytic: for a centre outside the core polyhedron, the nearest point over all faces (face interior if the projection is inside the face's sides, else its edges); inside, the face plane with the largest signed distance |
| box or hull against box or hull | separating axis test (below), then clipping |
| everything else (cylinder, tapered capsule, capsule against box or hull, sphere against cylinder or tapered capsule) | `Epa3D.Collide` (GJK, then EPA when the cores overlap) gives the normal; the supporting features of both shapes are clipped |

A box is a hull with the topology of a cube (the hull built from the corners (±1, ±1, ±1)) and vertices scaled by the core half extents; its feature indices are that hull's.

### Separating axis test

Both cores are placed in A's frame. Face queries (Gregorius, GDC 2013): for each face of P, the gap `min over Q's vertices of n·v − offset`, maximum over faces (lowest face wins ties), for P = A and P = B. Edge query: for each pair of undirected edges, the pair is tested only if the arcs of their face normals intersect on the Gauss map (B's normals negated): C and D on opposite sides of the plane of A and B, A and B on opposite sides of the plane of C and D, and the two crossing points (each a positive mix of one arc's ends) pointing the same way. A surviving pair with `|eA × eB|² > 1e-10 |eA|² |eB|²` gives the axis `eA × eB` normalised and oriented away from A's vertex mean, with gap `axis · (originB − originA)`; the largest gap wins (lowest A edge, then B edge, on ties).

Choice, with `tol = 5e-4 (extentA + extentB)` where extent is the largest distance of a vertex from the vertex mean: B's face replaces A's when `gapB > gapA + 0.005 |gapA| + 0.2 tol` (frame-to-frame coherence: A is the reference unless B is clearly better); an edge pair replaces the face when `gapE > gapFace + 0.02 |gapFace| + tol`. If the best gap exceeds `radiusA + radiusB + speculativeDistance` the manifold is empty.

When the best gap is positive (separated cores) the axis may under-estimate the true distance near vertex–vertex and vertex–edge closest pairs. Then GJK is consulted; the generic route below is used instead if its distance exceeds the axis gap (minus the radii) by more than `2e-4 (extentA + extentB + centre distance)`, or its normal differs from the axis by more than `cos = 0.9995`.

### Features and clipping

A contact feature is a point, a segment or a convex polygon of *surface* points (core points moved out along the normal by the convex radius; a tapered capsule uses each sphere's own radius). Winding does not matter: side planes are oriented towards the polygon's centroid.

- SAT face contact: the reference face of P, and the incident face of Q with the smallest normal·reference normal (lowest index on ties). Edge contact: the two edges.
- Generic route, for the outward direction u of a shape: sphere, a point; capsule-like shapes, the segment through the two end points if their offsets along u differ by at most `thick`, else the nearer end; hull or box, the best-aligned face if all its vertices are within `thick` of the support plane, else the flattest edge at the support vertex if its other end is within `thick`, else the support vertex; cylinder, the twelve-sided cap polygon (samples every 30° from +X towards +Z) if the rim's spread `2 r sin(angle)` is within `thick`, else the side line along the axis if its ends are within `thick`, else the rim point. `thick = speculativeDistance + 1e-4 (extentA + extentB)`.
- Polygon against anything: Sutherland–Hodgman clipping of the incident feature against the sides of the reference polygon (a segment is clipped by parameter, a lone point kept only if inside); the larger polygon is the reference when both are polygons (A when within 5%). Separation is measured along the contact normal to the reference polygon's plane (Newell normal; if that plane is more than 60° from the contact normal, the plane perpendicular to the contact normal is used). Points beyond `speculativeDistance` are dropped. Segments and points without a polygon are matched in the view along the normal; segments within 1% sin² of parallel there contribute the two ends of their overlap, otherwise their crossing point.
- Whenever the GJK/EPA contact is known (generic route, separated SAT) and the clipped points are all farther than it by more than `2e-4 (extentA + extentB)`, the contact's own point joins the set (id `0xC001C001`).

### Reduction

More than four points are reduced as in Gregorius, GDC 2015: the deepest point (lowest index on ties); the point farthest from it; the point giving the largest triangle area about the normal; the point lying farthest outside that triangle's edges (none if all are inside). The survivors are sorted by id (insertion sort, ties keep selection order), so a point keeps its slot while its feature pair persists.

### Feature ids

`id = featureA << 16 | featureB`. A feature code is 16 bits: kind in the top two (0 face, 1 edge, 2 vertex, 3 derived) and an index in the low 14 (larger indices wrap). Indices: hull or box face, vertex, and edge (half-edge index / 2); sphere: vertex 0; capsule and tapered capsule: vertices 0 (+Y end) and 1 (−Y end), edge 0; cylinder: cap polygon corners `cap * 16 + k` (cap 0 is +Y), the side line vertices 32 (+Y) and 33, edge 32, the rim point `34 + cap`. A point of a clipped polygon pairs the reference face with the incident corner it came from, or the reference edge that cut with the incident edge that was cut (an edge created by an earlier cut carries a derived code of the reference edge). Segment contacts pair vertex or edge codes by where the closest point or overlap end lies (end → vertex, interior → edge).

### Measured accuracy

Over 10^5 random separated pairs (all shape kinds, rounded and sharp, speculative distance 0.05, distance 1e-3 to 0.05) the smallest manifold separation differs from `Gjk3D.Distance` by at most 4.8e-4 (shapes of size about 1); the test limit is 2e-3. Against a brute-force separating axis test in double on overlapping hull pairs, the manifold normal agrees (cosine above 0.999) wherever the best axis leads by more than the bias above, and the manifold is never deeper than the brute-force depth by more than that bias.

## Decorators and compounds

Raw-tier types in `RecursiveAI.OnePhysics.LowLevel`: `ScaledShape3D`, `PlacedShape3D`, `ShapeBounds3D`, `Compound3D`, `MutableCompound3D` (both derive from `CompoundShape3D`), `CompoundPairs3D`, the compound methods of `ManifoldGenerator3D`, and `ShapeCompositionException` (with `ShapeCompositionError`). Arithmetic is float32 as above; the tree's boxes are double (widened exactly from float32).

### Scaled shapes

`ScaledShape3D.Create(shape, scale)` returns an ordinary `ConvexShape3D`, so every query, the mass properties and the manifold generator (analytic routes, feature ids) treat it as the equivalent unscaled shape. Scale components must be finite and above 0 (`InvalidScale` otherwise; a mirror is expressed in a child's rotation instead). What each kind accepts (`GetScaleSupport`):

| Shape | Scale accepted | Result |
|---|---|---|
| sphere, capsule, tapered capsule | equal on all axes | every dimension times the scale |
| box, hull, convex radius 0 | any positive scale per axis | box: outer half extents times scale; hull: vertices times scale, rebuilt with `ConvexHull3D.Build` |
| cylinder, convex radius 0 | X equal to Z, Y free | radius times X, half height times Y |
| box, cylinder, hull with convex radius above 0 | equal on all axes | outer size and radius times the scale (a box or cylinder keeps its outer size, as when built) |

Unequal scale where the table says equal is `UnsupportedScale` (a sphere would become an ellipsoid; a convex radius is a sphere offset). A result a factory rejects (a dimension underflowing to 0 or above 2^21, a scaled hull that fails to build) is `ShapeBuildFailed`, with the factory's error as the inner exception. A scale of exactly (1, 1, 1) returns the shape unchanged. A box's rounded radius is `radius * scale` clamped to its smallest outer half extent.

### Placed shapes and bounds

`PlacedShape3D` is a shape with a local transform (rotation and translation) in its holder's frame, validated on creation: rotation squared length within 1% of 1 (stored normalised), translation components finite and at most `MaxLocalOffset = 2^20`. A default value is invalid and its methods throw `InvalidOperationException`. Its pair queries (`Distance`, `Collide`, `ShapeCast`, `RayCast`) and `ManifoldGenerator3D.Collide(PlacedShape3D, PlacedShape3D, ...)` take the pose of the other *holder* in this holder (`otherInThis`), run the plain routine on `bInA = localA^-1 * otherInThis * localB` (onemath's rigid transform product and inverse, in that order) and map normals by `localA.Rotation` and points by `localA` into the holder's frame. Separations and feature ids are those of the shapes, so a placed shape has the ids it would have unplaced. Mass properties: volume unchanged, centre `local.TransformPoint(centre)`, tensor `InertiaF.Rotate(tensor, local.Rotation)`.

`ShapeBounds3D.Compute(shape, pose)` takes six support points, along `±` row `i` of the pose's rotation matrix for axis `i` (world coordinate `i` of `R s + t` is `row_i · s + t_i`), then grows each side by `2^-16 (|lo| + |hi| + (hi - lo))` per axis, which covers float32 rounding in the transform and the few-1e-6 tolerance of the GJK casts, so a shape never leaves its box.

### Compounds

A compound has up to `MaxChildren = 2^16` child slots. Each child is a `CompoundChild3D`: a placed shape (not the point shape) and an `int` material id (any value). The child **id** is its slot: the position for `Compound3D`; for `MutableCompound3D` the lowest free slot (a min-heap of freed slots), with a per-slot generation starting at 1 and increasing on removal. A `CompoundChildHandle3D` is (slot, generation). Child bounds are `ShapeBounds3D.Compute(child.Shape, child.Local)` and must lie within ±2^22 (`BoundsTooLarge`). All bounds and results use child ids.

**Tree.** Children are inserted in ascending slot into a `DynamicTree<Aabb3D>` (see `broadphase.md`), tight boxes (no margin), user id = slot. `Compound3D.Create` then calls `Rebuild` (binned surface-area heuristic, deterministic from the leaves' boxes and node indices). `MutableCompound3D` edits with `CreateProxy`, `DestroyProxy` and `MoveProxy` (insertion with rotations), `Optimize()` rebuilds. The tree therefore depends on the input (static) or on the whole edit sequence (mutable), nothing else. `WriteBvh` writes the tree (`DynamicTree.Write`), then the slot count and for every slot: alive (u8), generation (i32); for live slots also material (i32), local rotation x y z w and translation x y z (f32), padded bounds min x y z, max x y z (f32). Little-endian; `StateWriter` conventions.

**Query results do not depend on the tree.** Every query tests the children's own stored bounds through the tree (a leaf's box is the child's box), visits every child whose bounds it reaches, and applies a rule that does not depend on visiting order, so a mutable compound answers exactly as a static one with the same children (the single exception is `RayCastAny`, documented to return whichever hit the traversal meets first).

- **Overlap** (`OverlapBounds`): child ids whose bounds overlap or touch a box (comparisons on float32 values widened to double), ascending.
- **Ray cast** (`RayCast`): each reached child is cast with `Gjk3D.RayCast(child.Shape, child.Local, origin, direction, maxFraction)`, always with the caller's `maxFraction`; the closest fraction wins, equal fractions go to the lowest id. The result carries the child id, its material, the point, the outward normal (compound frame), and `StartsInside`. A fraction-0 hit clips the traversal to the smallest positive double rather than 0, so other children at fraction 0 are still compared. The ray's start and end (`origin + maxFraction * direction`) must be within ±2^22: GJK's tolerances scale with the largest coordinate it sees, so a far-away ray would make everything look hit (`InvalidQuery`).
- **Shape cast** (`ShapeCast`): the swept shape's padded bounds at the start are swept along the motion (`BoxCast3D`, a ray from the box centre against tree boxes grown by the box's half extents, as in `broadphase.md`); each reached child gets `Gjk3D.ShapeCast(shape, child.Shape, shapeInCompound^-1 * child.Local, shapeInCompound.Rotation^-1 * motion)`; closest fraction, lowest id on ties. The normal points from the cast shape to the child; points are mapped by `shapeInCompound`. Start and end positions within ±2^22.
- **Mass properties** (`GetMassProperties`): each live child's properties moved into the compound frame, combined in ascending id by `InertiaF.Combine` (sum of masses, mass-weighted centre, parallel-axis shift of each tensor). Overlapping children count twice. An empty compound throws `EmptyCompound`.

**Errors** (`ShapeCompositionException`, derives from `ArgumentException`, message names the input, e.g. `children[3]`): `InvalidScale`, `UnsupportedScale`, `InvalidTransform` (NaN, infinite, non-unit rotation, translation beyond limit), `InvalidShape`, `EmptyCompound`, `TooManyChildren`, `BoundsTooLarge`, `UnknownChild` (stale, foreign or default handle, dead id), `ShapeBuildFailed`, `InvalidQuery` (NaN or infinite query values, inverted box, `maxFraction` not positive, range). A failed edit changes nothing.

### Child pairs and manifolds

`CompoundPairs3D.FindChildren(compound, shape, shapeInCompound, margin, results)`: ids of children whose bounds meet the shape's padded bounds under the pose, grown by `margin` on every side in double; ascending. `FindPairs(a, b, bInA, margin, results)`: for every child `j` of B (ascending) the box of `bInA * child_j.Local` grown by `margin` is queried in A's tree; pairs `(i, j)` sorted by `i`, then `j`. `GetQueryBox` exposes the box. The test is on bounds only; a pair may not touch.

`ManifoldGenerator3D.CollideCompoundConvex`, `CollideConvexCompound` and `CollideCompoundCompound` enumerate with `margin = speculativeDistance` and build one manifold per pair with the placed-shape routine (`bInA` composed as above), dropping empty ones, in enumeration order. Manifolds are in A's frame (the compound's frame when A is a compound).

**Id layout.** A manifold point keeps its 32-bit feature id (`featureA << 16 | featureB`, as in "Feature ids" above), which are the child shapes' own features. Each child pair is its own contact, keyed like any contact by shape A, shape B and the pair `(SubShapeA, SubShapeB)`: the sub-shape ids are the child ids, and 0 for a shape that is not a compound. The shared core already stores both sub-shape ids per constraint (and carries them in events and the event order) and one `uint` feature id per point, so nothing changes in the 3D contact payload: warm starting matches points by feature id within the constraint for the child pair. For a host that wants one key per point across child pairs, `ChildContact3D.GetPointKey(i) = (SubShapeA << 48) | (SubShapeB << 32) | featureId`, which is lossless because child ids are below 2^16.

### Checks

`tests/OnePhysics.Tests/Collision3D/CompoundTests.cs` (known answers, hostile input) and `CompoundBruteTests.cs` (10^5 each against iterating every child: ray, shape cast, bounds overlap, child pairs; 2·10^4 compound pairs with all-pairs manifolds; 2000 random edit sequences compared with a static compound rebuilt from the surviving children); fuzz target `compound3d`; hash-matrix scenario `collision3d/compound` (tree bytes after every edit, queries, pairs and manifolds).
## Triangle meshes and height fields (`TriangleMesh3D`, `HeightField3D`, `TriangleSoup3D`)

Static, non-convex shapes made of triangles. Both derive from `TriangleSoup3D`, which carries the shared queries. They live in their own float32 frame, are immutable and are **static-only**: `GetMassProperties` throws `StaticOnlyShapeException` (a `NotSupportedException`), and `IsStaticOnly` is true.

### Arithmetic

Vertices are float32. Every test on a single triangle (ray, box, edge activity) converts the vertices to double exactly and computes in double with `+ - * /` and square root only; results go back to float32 once. Double arithmetic is IEEE-exact, so a port reproduces the bits.

### Triangle mesh

- **Input:** `vertices` (at least one) with finite coordinates of magnitude at most 2^21, and `indices` (a multiple of 3, at least one triangle) in `[0, vertices.Length)`; at most 2^26 vertices and 2^26 triangles. Validation runs in this order: options, empty vertices, empty indices, index count, sizes, each vertex (first bad one, naming `vertices[i].X|Y|Z`), each index (naming `indices[i]`, its corner and triangle). Errors are `MeshBuildException` (an `ArgumentException`) with a `MeshBuildError`.
- **Zero-area triangles** are dropped: with `n = (b - a) x (c - a)` in double, a triangle is dropped when `|n| <= 1e-10 * L^2` (`L^2` the squared longest edge) or `|n| = 0`; this covers repeated indices, coincident and collinear points. Dropped triangles are listed (input triangle numbers, ascending) in `DroppedTriangles`; the kept triangles keep their input order and get ids `0..k-1`; `SourceTriangle(id)` maps back. A mesh with no triangle left is `NoTriangles`.
- **Winding:** the front side of a triangle `(v0, v1, v2)` is the side its normal `(v1 - v0) x (v2 - v0)` points to. Consistent (outward) winding is expected; active-edge flags assume it.
- **Active edges.** Bit `e` of a triangle's flags is edge `e`: 0 is `v0 -> v1`, 1 is `v1 -> v2`, 2 is `v2 -> v0`. Vertices at equal positions (compared as float values, so `-0 = +0`) are treated as one vertex when edges are matched. The directed edges are sorted by their undirected key (smaller canonical vertex id in the high 32 bits); an edge is *shared* when exactly two triangles carry it and they traverse it in opposite directions. Every other edge (boundary, three or more triangles, equal direction) is **active**. For a shared edge with face normals `nA = (a1 - a0) x (a2 - a0)`, `nB` likewise and edge vector `eA` of A:
  `sin = ((nA x nB) . eA) / (|eA| |nA| |nB|)` (positive when the surface folds away from the viewer: convex), `cos = (nA . nB) / (|nA| |nB|)`. The edge is **inactive** when `sin < -tol` (concave), or when `cos > 0` and `sin <= tol` (flat); otherwise active (convex, including folds sharper than a right angle). `tol` is `MeshBuildOptions.FlatEdgeTolerance`, default 1e-3 (about 0.06 degrees). Both triangles get the same answer.
- **Bounds** are the float min and max over the vertices of kept triangles.

### Mesh BVH

Static, built over the kept triangles' float32 bounds; its canonical bytes (`ToBvhBytes`) are the same on every platform.

1. Nodes are laid out depth first; the left child of node `i` is `i + 1`, an inner node stores its right child's index, a leaf stores `(first, count)` into the triangle order array. At most 4 triangles per leaf.
2. A node covers a range of the order array (initially `0..n-1`). It first takes the exact float bounds of its triangles and the bounds of their centroid sums `min + max` (in double; twice the centroids).
3. One triangle: leaf. Otherwise, while depth < 32, a **binned SAH** split is searched: for each axis with a positive centroid extent (x, y, z in this order), 16 bins by `bin = clamp(int((c - cmin) * (16 / extent)), 0, 15)`; the cost of splitting after bin `s` is `area(L) * |L| + area(R) * |R|` (areas of float bounds as `2(xy + yz + zx)` in double, both sides non-empty); the lowest cost wins, ties to the lowest axis and then the lowest bin (strict `<`). Splitting costs `1 + cost / area(parent)`, a leaf `count`; a node of at most 4 triangles becomes a leaf when its leaf cost is not above the split cost; larger nodes always split.
4. The partition is in place: scan from the front, a triangle with `bin <= s` stays and otherwise swaps with the one at the back (this fixed rule is the order).
5. When no axis has a positive extent (all centroids equal) or depth is at least 32 and the node has more than 4 triangles, the range is cut at its middle `start + count / 2`; when an axis has positive extent, the range is first sorted by `(centroid sum on the widest axis, triangle index)` (the widest is the first of x, y, z with the largest extent).
6. Bytes: node count (int32), each node as six float32 bit patterns (min xyz, max xyz), offset, count (int32 each), then the triangle order length and its int32 entries; little-endian.

### Height field

- **Layout.** `samplesX` by `samplesZ` samples (each in `[2, 2^14]`, at most 2^26 in all), row by row (`heights[j * samplesX + i]`). Sample `(i, j)` is at `(i * sx, h * sy, j * sz)`, each coordinate one float32 product (`i` converted to float exactly). Scale components are finite and in `(0, 2^21]`; the grid's extent and every `|h * sy|` are at most 2^21.
- **Cells and triangles.** Cell `c = j * (samplesX - 1) + i` has corners `v00 = (i, j)`, `v10 = (i+1, j)`, `v01 = (i, j+1)`, `v11 = (i+1, j+1)` and two triangles: `2c` is `(v00, v01, v11)` and `2c + 1` is `(v00, v11, v10)`. Both face up (+y). The diagonal is always `v00`-`v11`. An optional hole mask has one entry per cell; a hole has no triangles (`TryGetTriangle` returns false); a field of only holes is `NoTriangles`.
- **Active edges** use the same rule as for meshes; the neighbour across an edge is the other triangle of the cell or the adjacent cell's triangle (the grid layout fixes the pairing); a missing or hole neighbour makes the edge active. The flags are stored one byte per cell (3 bits per triangle).
- **Storage** [chosen]: float32 heights (4 bytes per sample, exact), hole mask 1 bit per cell, flags 1 byte per cell, and a pyramid of per-block float ranges of `h * sy` (blocks of 4 x 4 cells at level 0, doubling per level up to one block; about 0.5 byte per cell).
- **Mid-phase:** the pyramid's blocks are boxes (x and z from the same products the vertices use); traversal is depth first, the four children of a block visited nearest first by ray entry fraction (ties in `(dz, dx)` order); a level-0 block tests its up to 32 triangles directly.

### Queries

All take and return the shape's frame. Inputs must be finite (`ArgumentOutOfRangeException` otherwise); `maxFraction` is finite and at least 0.

- **Ray vs triangle** (`TriangleTests3D.RayTriangle`): `n = (b - a) x (c - a)`, `nd = n . d`; `nd = 0` misses; `backFace = nd > 0`. For each edge `(p, q)` of the triangle in order `a b`, `b c`, `c a`, a side bit is computed with the endpoints ordered lexicographically by `(x, y, z)` (the *canonical* order): `s = ((p - o) x (q - o)) . d`; the canonical bit is `s > 0` when `s != 0`, else the sign of the first non-zero of `w.x, w.y, w.z` for `w = (q - p) x d` (a ray exactly on the edge counts as nudged by an infinitesimal step along +x, then +y, then +z), and `true` when all are zero. The bit for the edge as traversed is the canonical bit, inverted when the edge runs against the canonical order. The ray hits when the three bits agree; `t = (n . (a - o)) / nd` must be at least 0. Two triangles sharing an edge see exactly opposite bits for it, so a ray through a shared edge or vertex is accepted by exactly one triangle of a consistently wound fan.
- **Ray cast (closest)**: smallest `t`, then lowest triangle id among equal `t` (in double); `t <= maxFraction`. **Any**: the first hit the traversal meets (deterministic; the traversal order is fixed above). **All**: every hit sorted by `(t, id)`. Back faces are skipped when `cullBackFaces`. A zero direction never hits. Reported `Fraction` is `(float)t`, `Point = o + t d` computed in double then rounded, `Normal` the unit front normal.
- **Ray vs box in the mid-phase:** slab test in double against the node's float bounds padded by `1.5e-8 * (|o| + |lo| + |hi| + (hi - lo))` per axis, so rounding in the slab arithmetic never loses a triangle the triangle test accepts; an axis with `d = 0` or a non-finite `1 / d` is a parallel slab. Nodes with an entry fraction above the current limit are skipped (the limit is the best `t` for closest-hit queries, `maxFraction` otherwise; equal is not skipped, so the id tie rule holds).
- **Overlap** (`Overlap(box, ids)`): ids ascending of the triangles for which `TriangleTests3D.BoxTriangle` holds: the triangle's float bounds touch the box (exact, closed), and the 9 edge-cross-axis tests and the triangle plane test of the separating-axis theorem pass in double (touching counts). The mid-phase culls nodes with exact float box tests, which agree with the first condition.
- **Shape cast** (`ShapeCast`): the shape's bounds at the start (six supports) and end of the motion are united and grown by `1e-5 * (max |coordinate| + 1e-3)`; candidate triangles come from `Overlap`; each, in ascending id, is brought into the shape's frame (`R^T (v - p)`) and cast with `Gjk3D.ShapeCast` as a flat convex shape; the smallest fraction wins, ties to the lowest id (strict `<`). `CastTriangles(shape, placement, motion, ids)` is the same over any id list, and over all ids it is the brute-force reference. The tolerance is the convex cast's (fraction error below 2e-4 for a motion of length 8). `Normal` points from the triangle to the shape.
- **Candidates** (`GetCandidates`): the shape's bounds in the mesh frame, grown by the margin, through `Overlap`; each `SoupTriangle3D` has its id, vertices, active-edge flags and `Normal`, and converts to a convex shape with `ToShape()` (allocates) or `TriangleShapeBuffer3D.Set` (reuses one instance).
- **Triangle as a convex shape:** `TriangleShapeBuffer3D` builds a degenerate two-faced hull whose support is the farthest of the three vertices (lowest index on ties). GJK distance and casts work on it; EPA on a flat shape returns a depth along the triangle normal and is not meant for penetration (a later narrowphase for triangles handles that).

### Determinism check

`tools/HashMatrix/Scenarios/Mesh3D/MeshScenarios.cs` registers `mesh3d/queries`: a jittered terrain mesh, a soup with degenerate triangles and a height field with holes from SplitMix64 streams; step 0 hashes the BVH bytes, the counts, bounds and every triangle's flags; for each shape one step hashes 1500 ray casts (closest, any, some all-hits) and a second step 800 overlaps and 200 shape casts with candidate lists (seven steps in all).

## Triangles in the 3D world

The 3D world (`world3d.md`) collides convex shapes with the triangles of meshes and height fields through the same manifold generator as every other convex pair:

- **Triangle kind.** A candidate triangle (`SoupTriangle3D`, from `GetCandidates`) becomes a convex shape of kind triangle: the three vertices as a degenerate hull with two faces (normal and its negation) and zero convex radius. Its support is the farthest vertex (lowest index on ties), as for `TriangleShapeBuffer3D`.
- **One manifold per triangle.** The generator runs once per (convex sub-shape, triangle) pair in ascending triangle id order for each convex sub-shape in ascending sub-shape id; every result with points becomes its own constraint. Results are not merged per body pair: merging would need a common normal across triangles and the internal-edge rule already removes the seam contacts that would double up, so the per-triangle constraints stay (a body over N triangles has up to N constraints, each with at most 4 points).
- **Feature ids and warm starting.** Each constraint records its sub-shape ids (the triangle id for a soup); the constraint chain of a pair is matched to the new manifolds by `(subA, subB)`, and inside a constraint points keep their impulses by the generator's feature id (vertex, edge and face codes). Crossing to another triangle therefore starts that triangle's constraint from zero.
- **Internal edges.** The rule in `world3d.md` (drop a manifold whose normal is not the face normal when its closest feature is an inactive edge or a vertex with both edges inactive) is applied after the manifold is generated and before points are stored.
