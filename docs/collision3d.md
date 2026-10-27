# 3D convex collision core

Normative description of `RecursiveAI.OnePhysics.Collision3D`: support-mapped convex shapes, GJK distance and casts, EPA penetration, Quickhull hulls and mass properties. A port that follows this text and the arithmetic rules below produces the same bits.

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
