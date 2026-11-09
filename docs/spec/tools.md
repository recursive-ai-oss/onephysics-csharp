# Authoring tools

`RecursiveAI.OnePhysics.Tools` holds geometry processing used when content is made or loaded. Every tool is a pure function of its inputs and options: the same inputs give the same output bits on every platform, vector width and thread count. All arithmetic is IEEE `+ - * /` and `sqrt` (double inside, float32 at the boundary); the few trigonometric functions the test generators use come from onemath. No tool reads a clock, a hash order or a machine property. Errors are `GeometryToolException` (kinds: Empty, NonFinite, OutOfRange, BadIndex, Degenerate, OpenMesh, NonManifold, InvertedWinding, SelfIntersecting, InvalidOptions); the message names the input and the position of the offending value.

## Input limits

- Meshes: at least one triangle (cleaning accepts none), at most 2^24 vertices and 2^24 triangles, coordinates finite with magnitude at most 2^21 (the engine hull limit), indices in range, index count a multiple of 3.
- 2D polygons: 3 to 16384 points in total over outline and holes, finite, magnitude at most 2^20, nonzero coordinates not below 2^-80 (so the exact orientation predicates stay in range). Consecutive duplicate points and a repeated closing point are dropped.
- Sprites: width and height 1 to 16384 and at most 2^26 pixels; alpha has exactly width x height bytes.

## Mesh weld and clean (`MeshCleaner`)

1. Weld tolerance `t` = option, or 10^-6 times the longest axis extent of the vertices. Vertices are sorted by (x, index); every pair at distance at most `t` is linked (a sweep along x; single linkage, so chains of close vertices merge). The lowest index of a group is kept, at its own position.
2. Triangles are remapped. Removed as degenerate: a repeated vertex, zero area, or smallest height (2 x area / longest edge) at most `t`. Of triangles with the same vertex set (either winding) only the first stays.
3. Surviving vertices keep their relative order; unreferenced ones are dropped. Triangles keep their order.
4. Edges are grouped by their sorted vertex pair. Components are found by breadth-first search from the lowest unvisited triangle across edges shared by exactly two triangles; a neighbour is flipped (indices 1 and 2 swapped) when it uses the shared edge in the same direction. A component whose triangles all have three such neighbours (closed) is flipped as a whole if its signed volume is negative. Open components keep the orientation of their lowest triangle.
5. Report: component count, edges with 3 or more triangles (sorted by vertex pair), boundary edge count, and holes = connected groups of boundary edges.

Cleaning is idempotent bit for bit. The input arrays are not modified.

## Mass properties (`MeshMass`)

Closed means every directed edge occurs once and its reverse once. Volume, centre of mass and inertia come from the signed tetrahedra (reference point, triangle) about the vertex mean, with the simplex second moments of Tonon (2004): the integral of x x^T over a tetrahedron is V/20 (sum p_k p_k^T + (sum p_k)(sum p_k)^T); inertia = trace(C) I - C about the centre of mass (Mirtich 1996; Eberly, "Polyhedral Mass Properties (Revisited)"). Unit density; `Mass` is the volume. Negative volume throws InvertedWinding, zero volume Degenerate. For a convex mesh the result equals `ConvexHull3D.MassProperties` to single precision rounding.

## Hull simplification (`HullSimplifier`)

Greedy vertex removal: rank vertices by distance from the centroid of their neighbours (ties by index), re-hull the rest for the first 12 candidates and keep the removal with the smallest measured error (first wins ties). Error = the largest distance from any original vertex to the current hull (exact point to polyhedron distance over every face triangle). The simplified hull is the hull of a subset of the original vertices, so it lies inside the original and this equals the Hausdorff distance. Vertex and face targets are always pursued; beyond them removal continues while the error stays at most `MaxError`. `TargetsMet` is false if four vertices or a degenerate result stops it.

## 3D convex decomposition (`ConvexDecomposer`)

After Mamou and Ghorbel (ICIP 2009), with Lien and Amato's distance-based concavity (2004) and the worst-part-first refinement of Wei et al. (SIGGRAPH 2022).

1. Clean the mesh with default options. Fail with Degenerate when no triangle survives or the smallest bounding-box extent is below 10^-5 of the largest.
2. Voxelize: cell `h` = longest extent / `Resolution`; the grid starts 1.5 cells before the mesh minimum and has `ceil(extent / h) + 4` cells per axis. A cell is a surface cell when a triangle meets it (separating-axis box test on the triangle's plane distance, then clipping to the box by Sutherland-Hodgman, keeping non-empty clips). Every clipped polygon is stored with its triangle's outward unit normal. Empty cells not reachable from the grid corner by 6-neighbour flood fill (queue in index order) through non-surface cells are interior.
3. A part is an axis-aligned box over the mesh. Its surface points are the stored polygons clipped to the box, except polygons that only touch the box along an edge or a point or lie exactly on a box face with their normal pointing out of the box, plus the corners of the cross-sections where the box face planes cut interior cells. The hull of these points (at most 128 vertices) is the part's hull; its **concavity** is the largest, over surface points, of the distance a ray from the point along its outward normal travels inside the hull before leaving it (faces whose normal opposes the ray are ignored), divided by the mesh's bounding-box diagonal.
4. Starting from the whole mesh, repeat: choose the splittable part of largest concavity above `MaxConcavity` (lowest index on ties) while fewer than `MaxHulls` parts exist. Candidate cuts per axis: 6 evenly spaced planes plus up to 6 planes at mesh vertex coordinates, at least a quarter cell from the part's faces and from each other. Each candidate cost is `max(c1, c2) + 0.5 min(c1, c2) + 0.02 |n1 - n2| / (n1 + n2)` where `n` counts solid cells whose centre is in the child. The lowest cost wins (candidate order on ties: axis, then plane position); candidates are evaluated into fixed slots, in parallel when `MaxThreads` > 1, and compared in index order, so the result does not depend on the thread count. A part with no valid cut stops being splittable.
5. Output hulls in creation order; a hull with more than `MaxVerticesPerHull` (default 64) vertices is reduced by `HullSimplifier`. `ThresholdMet` is false if a part above the threshold remains.

Defaults: 16 hulls, concavity 0.01, 64 vertices per hull, resolution 32. Hulls are in the mesh's own coordinates; `ToCompoundParts` places each at the origin.

Stated accuracy (own test meshes): a box is one hull; the L-block 2 hulls and the U-block 3 hulls at the exact volume; a torus at concavity 0.01 and a mug at 0.05 meet their thresholds with at most 40 and 32 hulls, the union covering over 90% of the torus. The voxel grid limits the smallest feature found to about one cell.

## 2D polygon decomposition (`PolygonDecomposer`)

1. Outline and holes are normalised (outline counter-clockwise, holes clockwise). Every edge pair is tested with onemath's exact segment predicates: crossing, touching or overlapping edges (other than neighbours of one loop meeting at their shared vertex) are SelfIntersecting. Each hole must lie inside the outline (even-odd test) and in no other hole.
2. Holes are joined by bridges in order of decreasing maximum x (ties by index): from the hole's rightmost vertex (lowest index on ties) to the nearest ring vertex (distance, then ring position) that is visible, whose interior wedge contains the bridge and that no edge of the ring or of an unmerged hole blocks. The ring then contains both bridge ends twice.
3. Ear clipping (Eberly): a vertex is an ear when convex and no non-convex vertex lies inside or on its triangle (vertices equal to a triangle corner ignored); zero-area vertices are removed without a triangle. If no ear exists the test relaxes to strict interior containment once before failing with SelfIntersecting.
4. Hertel-Mehlhorn (1983): in triangle then edge order, a diagonal is removed when the merged piece stays convex at the two junction vertices and has at most `MaxVerticesPerPiece` (default 8, 3 to 64) corners.
5. Output pieces counter-clockwise, collinear vertices dropped; the pieces share vertices exactly, sum to the polygon area, and do not overlap. The count is at most four times the minimum (Hertel-Mehlhorn).

## Sprite outlines (`SpriteOutliner`)

1. The mask is padded with a zero border. A sample is solid when alpha is at least the threshold (default 128). Marching squares over cells between sample centres: each edge whose samples straddle the threshold carries one crossing at the linearly interpolated position; the two ambiguous cells keep diagonal solids apart. Segments are directed so the solid is on the left in image coordinates, and loops are followed from the lowest unvisited crossing id (cell row-major, horizontal edge before vertical).
2. Loops below `MinArea` square pixels are dropped. Each loop is simplified by Douglas-Peucker split at its first point and the point furthest from it, with the pixel tolerance (default 1). If simplification makes any loops cross or touch, the tolerance is halved (up to 10 times, then 0 = the raw contour).
3. Outer loops and holes are told apart by winding; each hole belongs to the smallest-area outer loop containing it. Islands inside holes are outer loops of their own. Output is ordered by the creation order of the loops. Points map to `(x * PixelSize + Origin.X, (height - y) * PixelSize + Origin.Y)` (y up; `KeepImageRows` leaves y growing down); outers are counter-clockwise and holes clockwise as the numbers read.
4. `ToConvexPieces` decomposes each island with `PolygonDecomposer`.

## Scene and hash matrix

`decomposed-pile-3d` (`tests/Common/DecomposedPileScene.cs`): 24 compound bodies (L, U, ring, mug; each hull set made by `ConvexDecomposer` at resolution 24, concavity 0.03, at most 24 hulls) dropped on a floor from a staggered grid with seeded orientations, 600 steps at 60 Hz. Own thresholds (no reference engine runs it): no NaN, no body escaping 16 m or sinking below its floor-contact height, maxSpeed at most 12, 99th-percentile final speed at most 0.1. Hash-matrix scenarios: `tools/mesh`, `tools/decompose3d`, `tools/polygon`, `tools/sprite`, `tools/decomposed-pile-3d`.
