# 2D collision: shapes, distance, casts, manifolds

Normative for `src/OnePhysics/Collision2D/`: what a port must reproduce to get the same bits. All arithmetic is IEEE binary32 with `+ − × ÷ sqrt` only (double where stated); no fused multiply-add, no estimates, no library transcendentals. Nothing here is vectorised, so scalar and SIMD runs are the same code.

## Conventions

* Vectors are `(x, y)`; a rotation is the pair `(cos, sin)`, a transform is rotate-then-translate. `RightPerp(v) = (v.y, −v.x)`, `LeftPerp(v) = (−v.y, v.x)`.
* Shapes live in their body's frame. Collision takes the pose of B in A's frame as one transform (`InvMul(a, b)` = `a⁻¹ ∘ b`), so a caller subtracts body positions in double first and converts the *difference* to float32 (the coordinate rule). All results are in A's frame; the solver stage rotates them to the world.
* Constants (`Constants2D`): `LinearSlop` 0.005, `SpeculativeDistance` 0.02 (4 slop), `MaxPolygonVertices` 8, shape coordinates within ±2^20, minimum extent or segment length 1e-5.
* A shape is a **convex core** of 1–8 points plus a **radius**: circle = 1 point, capsule = 2 points + radius, segment and chain segment = 2 points, radius 0, polygon = 3–8 points counter-clockwise + any radius. Edge *i* runs from vertex *i* to vertex *(i+1) mod n*; its outward normal is `RightPerp(edge)` normalised. A two-point core has edge 0 (p1→p2, normal `n`) and edge 1 (p2→p1, normal `−n`).

## Polygon construction

`CreatePolygon(points, radius)` rejects with `InvalidShapeException` (reason in `ShapeError`): fewer than 3 points, more than 65536, any coordinate NaN/infinite (`NotFinite`) or beyond ±2^20 (`OutOfRange`), a negative or non-finite radius, an extent (larger side of the bounding box) below 1e-5, a hull with fewer than 3 vertices after tolerance, or more than 8 hull vertices (`TooManyVertices`; the hull is never simplified silently).

Hull: sort by (x, y); drop points within `tol = extent · 2^-18` of the previous kept point in both coordinates; Andrew's monotone chain in double where a vertex is removed when its distance to the line through its neighbours is ≤ `tol` (`cross ≤ tol · |ac|`). Result is counter-clockwise starting from the lowest-x (then lowest-y) vertex, strictly convex.

Boxes (`CreateBox`) have vertices `(−hx,−hy), (hx,−hy), (hx,hy), (−hx,hy)` rotated by the box rotation and shifted by its centre. Chains: `Chain2D` needs ≥ 2 points (≥ 3 for a loop), consecutive points ≥ 1e-5 apart; `GetSegment(i)` returns p[i]→p[i+1 mod n] with ghosts p[i−1] and p[i+2] (none at the ends of an open chain). The segment index is the contact's sub-shape id.

## Mass

Per unit density, in double: area, centroid, rotational inertia about the centroid (`MassData2D`); `InertiaAbout(p) = Ic + area·|p − c|²` is the parallel-axis shift, also in double. Polygons: triangle fan from vertex 0 with all coordinates relative to vertex 0 (so shapes far from the body origin keep their precision). Rounded shapes add exact pieces: for each edge a rectangle `r × length` on the outer side, for each vertex a circular sector of radius `r` whose angle is the turn between the neighbouring edge normals (area `r²θ/2`, first moment `(r³/3)(n₁.y − n₀.y, n₀.x − n₁.x)` about the vertex, polar inertia `r⁴θ/4`). The angle comes from the chord and 30 half-angle steps `c ← c / sqrt(2 + sqrt(4 − c²))`, `θ = c · 2^30` (arithmetic only). Circle: `πr²`, `πr⁴/2`. Segments have zero area.

## Distance (GJK)

GJK on the Minkowski difference with a simplex cache of up to three (index A, index B) pairs (Catto, GDC 2010). Support point: the vertex with the largest dot product, **first maximum wins**. Up to 20 iterations; stops on a repeated vertex pair or when the search direction's squared length is below `ε² = (1.1920929e-7)²`. A cache whose count or indices are out of range is ignored. With radii: if `d > rA + rB` the points move onto the rounded surfaces and the distance shrinks by the radii, otherwise distance 0 and both points are the midpoint.

## Ray cast

Against the rounded core: candidates are each core edge pushed out by the radius (entering faces only, hit point within the pushed-out edge), and a circle of the radius at each vertex (entry root). The smallest fraction in `[0, maxFraction]` wins, ties to the first candidate (edges in index order, then vertices). A ray starting inside (distance to the core ≤ radius) hits at fraction 0 with a zero normal. Segments are hit from either side; a one-sided chain segment only from its right side. Non-finite input gives no hit.

## Shape cast and time of impact (conservative advancement)

Catto GDC 2013; Ericson ch. 9.5. At fraction `t`, find the core distance by GJK, `gap = d − rA − rB`. Stop with *Overlapped* if `gap ≤ 0` at the start, *Hit* if `gap ≤ target + tolerance`. Otherwise advance by `(gap − target) / bound`, where `bound = −n·Δv + rA·KA + rB·KB`: `n` is the world-frame direction between the closest points, `Δv` the difference of the centre-of-mass displacements over the sweep, `rX` the largest distance of a core vertex from the body's centre of mass and `KX = |q2 − q1| / (|q1 + q2| / 2)` the bound on the speed of the normalised-lerp rotation `q(t) = normalise((1−t) q1 + t q2)`. If `bound ≤ 0`, or `t ≥ maxFraction`: *Separated*. 50 iterations then *Failed* (the time is still safe). A rotation with `|q1 + q2| / 2 ≤ 1e-3` (about 170° or more) is *Failed* at 0. Because every step is bounded by the true closing speed and the target gap is ≥ 0, **the reported time is never after the first touch**. Defaults: `target = LinearSlop`, `tolerance = LinearSlop / 4`. Shape cast is the same loop with a pure translation, target 0, tolerance `1e-4`; it reports the normal and the point on the target after one more GJK at the hit time.

## Manifolds

`Collide2D.Collide(a, b, xfBInA, speculativeDistance)`: normal from A to B, up to two points, each `(point, separation, id)` in A's frame. The point is midway between the two surfaces; `separation` is negative when overlapping; a point with separation above the speculative distance is dropped (an empty manifold means "too far").

1. Both cores single points: normal `(pB − pA)/d` (or `(0, 1)` if `d ≤ 1e-12`), one point, id 0.
2. SAT over the face normals: `sepA` = max over A's faces of the minimum over B's vertices of `dot(n, v − faceVertex)` (ties: lowest face index), `sepB` likewise. If `max(sepA, sepB) − rA − rB > margin`: empty.
3. Reference face: A's, unless A has no faces or `sepB > sepA + 0.1·LinearSlop` (the tolerance keeps the choice from flickering). The incident edge of the other core is the one whose normal has the smallest dot product with the reference normal (ties: lowest index), or the single vertex.
4. If the cores are apart along the reference normal (`coreSep > 0.1·LinearSlop`), or the incident feature lies wholly beside the reference edge without overlapping (`beside && coreSep ≥ 0`): closest points of the reference edge and incident edge (Ericson 5.1.9). If both are at vertices (parameters exactly 0 or 1) the contact is **vertex–vertex**: normal = direction between them, one point, ids = the two vertex indices.
5. Otherwise **clip** the incident edge to the reference edge's side planes (planes at the reference vertices, along the edge direction). A point beyond a plane is replaced by the intersection and takes the reference vertex as its feature. Keep points with separation ≤ margin, in incident-edge order. If the incident edge lies wholly beside the reference edge (only when the cores overlap), the nearer vertex alone is kept.
6. Point = `p + n · (rRef − s − rInc)/2` where `s = dot(n, p − refVertex)`, separation `s − rRef − rInc`. If the reference was B, the normal is negated.

**Feature ids** are `(featureA << 8) | featureB`, each 8 bits: vertex *i* = `i`, edge *i* = `0x80 | i`. An unclipped incident vertex *j* against the reference edge *i* is `(edge i, vertex j)`; a point clipped at a reference vertex *v* is `(vertex v, incident edge j)`; vertex–vertex is `(vertex, vertex)`. The reference side supplies one half and the incident side the other, placed by shape (A high, B low). The ids stay equal while the same features stay in contact; the test suite checks a resting box over 100 jittered frames, with position jitter ±3 mm and rotation jitter ±0.002 rad against a large ground and ±0.0004 rad for two equal boxes side by side (the reference-face hysteresis is 0.0005).

Invalid input (default shape, non-finite transform, NaN margin) gives an empty manifold.

### Chain segments

A chain segment `(ghost1, p1, p2, ghost2)` collides with every shape except another chain segment. The solid side is the **right** of `p1 → p2` (normal `(dy, −dx)`).

* One-sided: if the other shape's centroid is behind the segment's line, no contact. Two-sided: the segment is mirrored (points, ghosts and normal) so the other shape is in front.
* The manifold is computed with the segment's front face as the only possible reference face of A. Then contacts are filtered so no internal edge is felt: a manifold whose normal equals the segment normal (|cross| ≤ 2^-12, dot > 0) is kept whole. Otherwise each point, by whether it is nearer p1 or p2: at p1 it is dropped when a ghost exists (the previous segment owns that vertex) and kept at a free end; at p2 with a ghost it is kept only when the vertex is convex (`cross(p2−p1, g2−p2) > 0`) and the normal lies in the wedge between this segment's normal and the next segment's, with tolerance 2^-12; at a free end it is kept.
* If the filter removes everything because the other shape's own face was the SAT axis (a tilted body whose corner sank through the segment line, the joint poking into its face), the manifold is recomputed with the segment's face forced as reference and filtered again.

## Determinism check

`Reference.CollisionHash2D.Compute()` hashes (FNV-1a, 64 bit, over the 32-bit words of every float and integer) rejected and accepted shapes, mass data, bounds, 4000 random pair manifolds, GJK results, ray and shape casts and times of impact, from a fixed xorshift generator. Expected value: `0x31E47F1BCE1B321F` is recorded in `DeterminismTests`; it must be equal on x64, scalar (`DOTNET_EnableHWIntrinsic=0`), NativeAOT, browser WebAssembly and arm64.
