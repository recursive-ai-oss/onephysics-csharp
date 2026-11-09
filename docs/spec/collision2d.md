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

## Sampled terrain

`HeightField2D` copies 2..2^20 height samples and an optional interval-hole
mask. Sample i is at (i*spacing, height[i]); coordinates must fit the existing
shape coordinate limit and every surviving segment must pass the chain's
minimum-length check. A true hole removes an interval. Non-hole intervals
are one-sided chain segments directed from sample i+1 to sample i, so their
right-hand normal faces upward. The predecessor ghost is sample i+2 and
the successor is sample i-1; neither crosses a hole or an endpoint. Stable
column order is increasing i. Ghosts suppress internal-edge contacts with
the existing chain contact rule. Segments have zero area and add no mass.

A world attaches one ordinary chain-segment shape per surviving interval to
a static body. This reuses chain queries, debug geometry and snapshot v1
payloads without a second collision representation. Snapshot, fork, exact
scene and mutation replay preserve the segment vertices, flags and ghosts;
the original sampled authoring representation is not stored. An all-hole
field attaches no shapes and returns a default first-segment handle.

## Empty placeholders

Shape kind 6 is `Empty`. It has zero vertices, radius, flags, centroid,
ghosts and stored vertex/normal payload. The v1 shape record has unchanged
length; readers accept a zero vertex count only for this new kind and
validate its zero payload. Empty shapes have no area or inertia, no
contacts or sensor volume, no ray or cast hits and infinite distance to
other geometry. They are not zero-radius circles (points). World queries
exclude them. Debug kind 12 carries no points. `AddEmpty` supplies explicit
body mass and inertia overrides, unit values by default, so a placeholder
still integrates and can participate in joints without collision geometry.

### Per-shape mass-centre offsets

`Shape2D.WithCenterOfMassOffset(o)` leaves vertices, normals, ghost vertices,
geometric centroid, bounds, contact ids and all query answers unchanged.
Offsets are finite local metres within ±2^20; nested offsets add and validate the
sum. Given unit-density area A, centroid c and polar inertia J at c, the decorated
mass properties are `(A, c+o, J + A*dot(o,o))`. Multi-shape bodies combine these
per-shape mass properties before applying any body-wide override. Zero-area shapes
(segments, chains, empty shapes) are unchanged. `PhysicsWorld2D.CreateShape`
attaches validated geometry with or without an offset; it rejects default
geometry (use `Shape2D.Empty` for a placeholder).

The fixed-size v1 geometry record is unchanged. An optional registered section
`0x324D4F43` carries the offsets: int32 version 1, int32 nonzero-entry count,
then ordered entries `(int32 live shape slot, float32 x, float32 y)`. Only live
positive-area shapes with nonzero offsets occur. All fields are little-endian;
slots strictly increase, components are finite and bounded. An absent section
clears offsets. The world hash includes this section only when nonempty; worlds
without offsets retain their existing snapshot bytes and hashes. Restore, fork,
exact-state scene v2 and replay preserve it.

`AddWithCenterOfMassOffset(geometry, offset, ...)` is the single-body convenience
form: it returns a body with one decorated shape. Defaults match other single-
shape helpers (dynamic, unit density, default material, zero pose). The 3D world
exposes the same helper over `ShapeSpec3D`; `CreateShape` attaches decorated
geometry to an existing body on either dimension's definition-level surface.

## Tile grids

`TileGrid2D` copies a rectangular, x-fastest array of `TileKind2D`. Dimensions
are 1..1024, at most 65536 tiles. Tile size is finite, at least 4e-5 metres;
width and height in metres must not exceed 1048576. The body origin is the
bottom-left corner. Empty, full-square, bottom-half, rising-right and
rising-left triangles are supported. All nonempty cells are solid on their
interior, including underneath the terrain. Only static attachment is allowed.

Boundary extraction uses integer coordinates on a half-tile lattice. Each
counter-clockwise tile polygon contributes unit edges. Opposite directed edges
cancel, so no internal faces remain. Edits copy the occupancy and ordered edge
set, undo the old cells' edges, and add the changed cells' edges; unchanged
cells are not re-extracted. The exposed graph is traced from the lowest
(start Y, start X, end Y, end X) edge. At a shared vertex choose the least
nonnegative counter-clockwise turn (eight directions), keeping corner-touching
solids separate. Delete collinear vertices. Each resulting loop supplies both
ghost vertices to each of its one-sided chain segments. Finally sort segments
lexicographically by (first Y/X, second Y/X, ghost-before Y/X, ghost-after Y/X).
Boundary tracing is rebuilt globally from the updated edge set; it is not a
simulation-step operation.

`PhysicsWorld2D.AddTileGrid` creates a static body; `CreateTileGrid` attaches
one grid to an existing static body. `GetTileGrid` returns immutable data.
`SetTile` and `SetTiles` fill or clear rectangles. Invalid inputs leave terrain
unchanged. Collision updates merge the old and new sorted segment lists:
identical segments retain their shape handles and warm-start caches; changed
segments are created in boundary order, then removed segments are destroyed in
old boundary order. A boundary segment cannot be independently destroyed via
its shape handle; edit the grid or destroy its body. Queries and debug drawing
use the resulting chain segments, with their ordinary shape ordering.

World snapshots retain their version-1 core layout. Grid metadata is optional
section `0x54494C45`, omitted when no live grid body exists. In little endian:
int32 section version (1), int32 live grid count; then entries by body slot:
body slot and generation (int32 each), width, height (int32), tile size
(float32), width*height tile-kind bytes, shape definition (density, friction,
restitution, rolling resistance as float32; category and mask as uint32; group
int32; sensor byte; layer int32), int32 segment count, and one shape slot /
generation int32 pair per sorted boundary segment. The engine's existing
section stores the actual chain geometry, broadphase and caches. The reader
reconstructs the boundary and checks segment count, geometry, flags, handle
uniqueness and ownership. Fork copies the metadata and handle arrays; immutable
grid data may be shared. Tile edits use the ordinary named-mutation recorder.

Authoring scene kind `tilegrid` has parameters width, height, tile size, then
width*height integer tile kinds (0 empty, 1 solid, 2 bottom half, 3 rising right,
4 rising left). Exact-state scenes retain all grid metadata through snapshots.

### Indexed terrain materials

Height-field creation accepts a copied scalar table and an index per original
interval, holes included. Validate every index before creating any segment;
each emitted segment uses its selected entry through the indexed material
store. Tile grids similarly accept one index per tile, empty tiles included.
`TileGrid2D.MaterialIndices` is immutable; `WithMaterialIndices` returns a new
boundary. `GetSegmentMaterialIndex` maps its segments back to the authored
table. Collinear edges merge only while their material indices agree; material
transitions retain collinear ghost vertices, not disconnected endpoints.

The owner of a boundary edge is sampled just inside its left side, in integer
eighth-tile coordinates. Only the first half-cell of a merged edge is sampled,
so it never crosses a material transition. `SetTileMaterials` replaces the
copied table and indices, regenerates the material partitions, and updates
unchanged shapes' indexed coefficients. Each planar segment stores only its
selected scalar triple in the common material section; the complete tile table
is stored once in grid metadata, preventing table-size × segment-count storage.
Indexed entries must have default Surface properties. Shape-wide surfaces come
from the creation definition and persist when tiles are edited.

The tile metadata section writes version 1 when every grid uses default surface
properties and has no indexed table. Otherwise it writes version 2: after each
entry's layer, append Surface properties (velocity, one-way direction and
friction axis as three float32 components each, then longitudinal, transverse
and spinning friction as float32), int32 material count, coefficient triples
as float32, int32 tile-index count and int32 indices. The index count is zero
without a table, otherwise width*height. The segment count/handles follow.
Readers accept both versions and reject version 2 with no extra state. Core
snapshot format v1 is unchanged. Material-bearing grids round-trip, fork and
record exactly as ordinary grids.

Character sweeps against one-sided chain segments validate each candidate
hit using the ordinary ghost-aware chain contact routine at the swept capsule
pose, with a 0.002 m speculative margin. A hit belonging to a neighbouring
segment's Voronoi region is discarded. Convex sweeps alone see rounded segment
endpoints even when ghost ownership excludes them; those phantom endpoints
otherwise pin a character just before a ramp crest. Ordinary two-sided segment
casts and the underlying convex distance algorithm are unchanged.

### Carvable bitmask terrain (contour grids)

A tile grid built with `contour` (and `PhysicsWorld2D.AddBitmaskTerrain`) is a bitmask: every
nonempty cell is one sample at its centre, lattice point `(2x+1, 2y+1)`, and only Empty and
Solid cells and no material indices are allowed. The boundary is the marching-squares
contour (Lorensen and Cline 1987) of the samples, taken over the dual cells between four
neighbouring samples (samples outside the grid are empty): a dual cell's polygon has the
solid sample corners and the midpoints of edges whose ends differ, which lie on the
lattice, so the same unit-edge cancellation and boundary tracing apply. Two solid samples
across a diagonal (cases 5 and 10) stay separate. Isolated cells are diamonds, flat edges
lie on cell edges, corners are cut at half a cell.

Pairing at a vertex where solids touch. At a vertex with several arriving edges each arriving
edge takes the least counter-clockwise turn among the leaving ones (as before); if two
arriving edges would take the same leaving edge, every arriving edge instead follows its own
solid wedge, the first leaving edge clockwise from the arriving edge's ray, so the answer
depends on the edges at that vertex only and not on the order loops are traced in. Worlds
whose touching corners resolved the old way and where the order mattered now resolve this
way; all other boundaries are unchanged.

Local rebuild. `TileGrid2D.WithCells(x, y, width, height, mask, kind)` (and
`WithRectangle`) re-extract only near the changed cells. A changed cell can change lattice edges only inside the box V
of lattice points its polygon (tile mode: `[2x, 2x+2]`; contour mode: its four dual cells,
`[2x-1, 2x+3]`) can touch. A vertex outside V keeps its edges and its turn choices, so an old
segment whose window (the ghost before, both ends, the ghost after) never visits a lattice
vertex of V is still a segment of the new boundary. The new boundary is traced from every
exposed edge touching V, forward and backward along the pairing rule, until four corners
outside V have passed since the last vertex in V; unchanged straight runs outside V are crossed
in one jump using the old segment list. Segments whose window visits V are regenerated from the
traced fragments; the result is merged with the kept ones in the comparer's order. The result
equals a fresh construction (tests over random edits in tile, material and contour modes);
if the trace ever cannot be completed locally it falls back to a full rebuild.

`CarveCircle`, `FillCircle` (cells whose centres lie within a world-space circle), `CarvePolygon`,
`FillPolygon` (even-odd rule on the same centres) and `StampMask` set cells and return the number changed; shapes of
unchanged segments keep their handles and caches, and only new shapes ask for a contact refresh
(a refresh would wake every body resting on the grid). Dynamic bodies whose broadphase box meets the
changed cells' box (in world space, plus half a cell plus 0.05 m) wake. Recorded mutations are
named `PhysicsWorld2D.CarveCircle`, `FillCircle`, `CarvePolygon`, `FillPolygon`, `StampMask`,
`DetachIslands`.

`DetachIslands(terrain, anchors, minCells, maxBodies)` takes the 4-connected pieces of solid
cells in order of lowest cell index; a piece touching an anchored edge (default the bottom row) stays.
A detached piece is removed from the grid (one edit) and becomes a dynamic body at its centroid with
the grid's pose, covered by boxes (greedy rectangles grown along x, then y, in cell order, so the
area is cells times cell size squared; the contour's cut corners are not reproduced), with the terrain's
shape definition (density 1 when it is 0). The convex cover is this library's own; the polygon decomposition of the tools package is not used.

Snapshot: the tile section is version 3 when any grid is a contour grid: version 2's layout with a
mode byte (0 tiles, 1 contour) after the tile size of every entry. Readers accept versions 1 to 3; a
version 3 section without a contour grid is rejected. Authoring scene kind `bitmask` has parameters
width, height, cell size, then width*height flags (0 empty, 1 solid).

## Signed distance field terrain (2D)

`SdfGrid2D`: 2..2048 samples per axis, at most 2^20, bilinear, outside the box the distance to the
box plus the clamped value floored at zero; `Gradient`, `Lipschitz` (corner one-sided difference over the
cell size times sqrt(2)) and `Trace` (sphere tracing, step value / Lipschitz) as in 3D. `FromFunction`,
`FromPolygon` (exact distance to the edges, inside where the winding number is not zero) and
the primitives and `Union`, `Intersection`, `Difference`. `ExtractContours(tolerance)` runs marching squares
over the cells including a ring of empty cells outside the grid: crossings are on cell edges at the linear zero; each cell lists its
directed link from its mask alone (a lone inside corner c_i is walked e_i to e_(i-1), a lone
outside corner e_(o-1) to e_o, two adjacent inside corners e_(i+1) to e_(i-1); saddles by the sign of the
corner sum), which keeps the inside on the left even where samples are exactly zero; loops start at
the lowest edge id, repeated points are dropped, and Douglas-Peucker simplification (split at the farthest point
from the first) removes points within the tolerance of the chord. `PhysicsWorld2D.AddSdf` adds a static body
with the contour as closed one-sided chains with ghost vertices (default tolerance a quarter of a cell). Only the contour
collides. A dynamic or kinematic body made by `AddSdf(type:)` is instead covered with convex polygons (`ConvexCover2D`): the
loops are split into outer loops (positive signed area) and holes (negative), each hole goes to the smallest outer loop containing its first
vertex, holes are bridged in order of decreasing rightmost x from that vertex to the visible loop vertex (Eberly, ear clipping: nearest
crossing of a +x ray, then the reflex vertex in the triangle with the smallest angle), the weakly simple polygon is ear-clipped
(lowest valid ear index first, collinear vertices dropped), and triangles are merged in ascending piece and edge order across a shared directed
edge while the union stays convex with at most `MaxPolygonVertices` vertices (Hertel and Mehlhorn). The pieces tile the contour area exactly, so the body's
mass is the contour's inside area times density; collisions use the ordinary polygon routines (not field samples), so a 2D dynamic field is
as accurate as its contour (within half a cell of the field) and costs one shape per piece. More than 8192 contour vertices is an
`ArgumentOutOfRangeException`; a piece below the minimum shape extent is dropped.
