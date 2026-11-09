# Destruction: fracture, slicing, breakable compounds and debris

Normative, language-neutral. Both dimensions share one implementation; a 2D world embeds its plane at z = 0 and uses polygons where 3D uses polyhedra. Everything here is deterministic: no platform generator, no
hash or dictionary order decides a result, and only IEEE-exact `+ - * /` and `sqrt` (plus one correctly rounded `pow` from onemath for the clustered pattern) are used.

## Geometry

A *piece* is a convex polytope in a body's frame, held in double precision as outward counter-clockwise faces (3D) or one counter-clockwise polygon (2D). It is cut by the half-space `n . x <= d` (unit `n`):

1. Every vertex is classified against the plane with tolerance `eps = 1e-11 * extent + 1e-13 * (sum of coordinate magnitudes)`: inside, on, outside. Nothing outside returns the piece itself; nothing inside returns empty.
2. Each face is clipped by Sutherland-Hodgman; an edge that crosses the plane strictly gets its intersection computed from its lexicographically smaller endpoint, so the two faces sharing the edge get equal bits.
3. The new face (3D) is the convex polygon of every vertex of the result that lies on the plane: de-duplicated within `eps`, ordered counter-clockwise about `n` by cross-product comparisons around the centroid (no angles). A cap of
   area below `eps^2` or fewer than 4 faces (3 vertices in 2D) gives empty.
4. Volume and centroid are by tetrahedra (triangles) fanned from the first vertex.

A piece that becomes shape geometry is rebuilt from its float vertices through the hull builder (3D) or a monotone-chain hull (2D), so a world restored from a snapshot continues from the same pieces as the world that
wrote it. A 2D shape holds at most 8 vertices: a bonded part is simplified to fit (the vertex with the smallest triangle area against its neighbours is dropped, lowest index on ties, until 8 remain); any other piece is
split into convex fans from its first vertex.

## Voronoi fracture

Seeds: `Uniform` rejection-samples the union of the body's pieces inside their bounding box; `Clustered` places a seed at `radius * u^power` from the impact point along a direction found by rejection in the unit ball (3D) or
disc (2D), `u` uniform in [0, 1), `radius` the definition's impact radius or the farthest vertex; `Points` uses the caller's seeds (body frame). The generator is SplitMix64 (state += 0x9E3779B97F4A7C15, the usual two
xor-shift-multiply rounds with 0xBF58476D1CE4E5B9 and 0x94D049BB133111EB), a double is the top 53 bits times 2^-53. At most `200 * cells + 200` attempts; a thin body that still lacks seeds gets the centroid plus offsets
`1e-6 * (k + 1)`.

Cells: for seed `i` the piece is cut by the bisector plane of every other seed `j`, nearest first (ties by index), stopping when half the distance to the next seed exceeds the cell's farthest vertex from seed `i`. A seed within 1e-9 of
an earlier seed owns nothing. Cells tile the piece exactly (the sum of their volumes equals the piece's to rounding). Each body shape is cut separately and the cells of one seed form one fragment (a body with several shapes
yields fragments with several shapes).

Small cells: a fragment under `MinFragmentVolume` times the body's volume (area in 2D) joins the nearest seed's fragment that is above the limit (its shapes are added to that body, mass is kept), or with `MergeSmall = false` is
deleted (mass and momentum are lost, counted as dropped). Fewer than two fragments above the limit means nothing is cut.

Fragments: new dynamic bodies with the source's origin, rotation (copied bit for bit), damping, gravity scale, bullet and sleep flags, and each shape's density, friction, restitution, rolling resistance, filter and layer.
Centre of mass velocity `v + w x r` (`r` from the source's centre of mass to the fragment's), angular velocity `w`: linear and angular momentum are conserved to rounding, because the cells tile the source and the parallel-axis
theorem holds. Bodies resting on the source are woken before it is destroyed. A body with a joint, vehicle, character or ragdoll, or with a shape that is not a sharp convex hull or box (3D) or polygon (2D), cannot be fractured:
manual calls throw `FractureException` naming the reason, automatic fracture forgets the body.

## Automatic fracture

`SetFracturable(def)` with `ImpulseThreshold > 0`. At the end of each step, after events are delivered, in ascending body slot order: a dynamic body whose *impact impulse* reaches the threshold is replaced by its cells. The impact
impulse is, for the contact manifold with the largest value, the sum over its points of the largest normal impulse the solver applied at that point in one substep (kg m/s); the strongest point is the impact point of a clustered
pattern. Children inherit the definition with `Depth - 1`, `ChildCellCount` (0: half the parent's, at least 2) cells, `ImpulseThreshold * ChildThresholdScale`, a seed `seed * 0x9E3779B97F4A7C15 + k + 1` and the uniform pattern.
At most 512 bodies are created per step and none while 2048 fragments are tracked: the rest wait for later steps (a cascade of children that break at once is spread over steps).

## Breakable compounds

A pre-fractured asset (`FractureAsset`) is a list of convex cells in a body frame and the *bonds* between cells that share a face: coplanar within `1e-5 * extent`, opposite normals within 1e-9 of antiparallel, overlap
area (length in 2D) above `1e-8 * extent^2` (`1e-5 * extent`). A bond stores its measure, centroid and normal (from part A to part B). `AddBreakable` makes one body with one shape per cell; parts are the shapes.

Stress, each step for a static body or an awake dynamic body, after the solve:

- Loads on a part: for every contact point, `F = impulse * substeps / dt` along the manifold normal (the accumulated normal impulse of the last substep scaled to the step), applied at the contact point, on the part that owns
  the shape. Friction is not included.
- Demand: the force and moment the bonds must supply to part `i`: `f_i = m_i a_i - m_i g - F_ext,i`, `M_i = -sum (p_k - c_i) x F_k` (moments about the part centroid; the part's own rotational inertia is ignored).
  A static body has `a_i = 0`. A dynamic body moves rigidly: `a_i = a + alpha x r_i + w x (w x r_i)` with `a = (F_total + M g) / M`, `alpha = I^-1 (T - w x I w)` from the contact loads.
- Flow: parts get a depth by breadth-first search over live bonds from the anchored parts (static body) or, per connected component without one, from its most loaded part (largest `|F|^2 + m^2`, lowest index on ties). Parts
  are processed deepest first (ties by descending index). A part's total demand `G` (its own plus what its children pass up) and moment `H` is divided over its bonds towards depth - 1 in proportion to their measure `w_k`:
  bond force on the part `P_k = w_k G`, moment about the bond centre `M_k = w_k H - (p_k - c_i) x P_k`; the parent receives `G += P_k`, `H += (p_k - c_j) x P_k + M_k`. On a chain this is the exact cut force and moment;
  where several bonds lead towards the root it is a proportional split.
- Stress: `max(0, P.n)/A + shear * |P_t|/A + |M_t|/W`, `n` the unit bond normal towards the parent, `P_t` and `M_t` the components perpendicular to it, `A` the measure, `W = A^1.5 / 6` (3D) or `A^2 / 6` (2D, unit thickness).
  A bond whose stress reaches its strength (`BreakableDef.Strength`, or the bond's own) for `OverloadSteps` consecutive steps breaks.

Analytic check: a static cantilever of equal cells loaded at the tip has bond stress `V/A + M/W` with `M = m g L n^2/2 + W_b g (n - 1/2) L` for `n` cells beyond the bond; the test matches it within 10% and the stress peaks at the
bond nearest the support.

When a bond has broken, the body is replaced by one body per connected component of the live bonds (flood fill in ascending part order, components ordered by their lowest part): on a static body a component holding an anchored
part stays static (so a structure loses only what is cut free), every other component is a dynamic fragment inheriting the velocity field as above. Each new body keeps the bond graph of its component. A break is retried
every step while a dead bond has not yet split the body (the per-step limit above).

## Slicing

`Slice(body, point, normal)`: the plane (a line in 2D) in the body frame, every shape cut into its negative and positive part (parts under `1e-9` of the body's volume are dropped), two bodies with the original's velocity field. The
plane must cut at least one shape. A bonded compound is cut part by part: bonds between pieces that stay on one side survive with their faces recomputed, bonds across the plane are gone. Halves that touch along the cut push apart
through contact bias; a spinning body's halves fly apart as free bodies do.

## Debris

World options (`DebrisOptions`, saved in the snapshot): `MaxFragments` (after each step, remove the smallest fragments by volume, then the oldest by creation sequence, until the count holds), `SleepToStaticSteps` (a fragment asleep
for that many consecutive steps is replaced by a static body with the same shapes; its handle changes), `LifetimeSteps` (removed that many steps after creation). Order within a step: lifetime, then freezing, then the budget.
Fragments come from fractures, bond breaks of dynamic compounds and slices made with `trackAsDebris`. Counters (`DestructionCounters`, cumulative and saved; `StepCounters.Fractures/BondBreaks/FragmentsCulled/Fragments` per step).

## Events

`FractureEvents` (and `OnFracture`): kind (`Impact`, `Manual`, `BondBreak`, `Slice`), the destroyed body's handle, the range of its fragments in `FractureFragments`, the impact impulse and a point. Cleared when the next step
begins; not saved. Fractures run after the step's events were delivered, so those events refer to bodies that may no longer exist.

## State

Optional snapshot section `0x52545344` ("DSTR"), written only while it holds state, registered last. Version 1: debris options (`i32 maxFragments, i32 freezeAfterSteps, i64 lifetimeSteps`), counters (nine `i64`:
fractures, slices, bond breaks, fragments created, culled, frozen, expired, slivers dropped, slivers merged) and the next fragment sequence (`i64`); records in ascending body slot (`i32 body, i32 generation, bool fracturable`, the
fracture settings if set, `bool bonded`, and if bonded strength, shear factor, overload steps, the parts (`i32 shape, i32 generation, bool anchored, f32 measure, f32 mass, f32 x3 centroid`) and the bonds (`i32 a, i32 b, f32
measure, f32 x3 centroid, f32 x3 normal, f32 strength, bool alive, i32 overload, f32 stress`)); fragments in sequence order (`i32 body, i32 generation, i64 sequence, i64 birth step, f32 measure, i32 asleep steps, bool frozen`). Only live
bodies are written, and a reader refuses a record of a body the snapshot does not contain. Nothing in the section is in world coordinates, so an origin shift changes nothing. A region snapshot carries the records and
fragments of its bodies (entities `breakable` and `debris`); a fork copies the section.

## Stability scenes (own thresholds)

`bar destruction-stability`: 600 steps each. Thresholds are 1.5 times the worst of the measured x64 values, rounded up. Penetration is the deepest contact overlap of the step; energy gain is the largest rise of kinetic plus
potential energy across a single step (bodies added or cut between steps excluded), relative to the energy before it.

| scene | maxSpeed | penetration | bodies | escapes | energy gain |
|---|---|---|---|---|---|
| `fracture-wall-3d` (48-cell bonded wall, projectiles at 35 m/s) | <= 55 | <= 0.55 | <= 400 | 0 | <= 0.02 |
| `fracture-runtime-3d` (24 crates, depth 1, budget 300, freeze 120) | <= 25 | <= 0.05 | <= 500 | 0 | <= 0.02 |
| `breakable-bridge-3d` (10-cell static bridge, anchored both ends, growing load until it collapses) | <= 5 | <= 0.7 | <= 100 | 0 | n/a |
| `fracture-2d` (20 crates, depth 1) | <= 15 | <= 0.2 | <= 400 | 0 | <= 0.02 |
| `slice-2d` (boxes cut in flight) | <= 15 | <= 0.2 | <= 200 | 0 | <= 0.06 |

`fracture-runtime-3d` once showed 0.63 m of penetration (0.95 was its threshold). It was the narrowphase, not the fracture: a fragment that sank into the floor had a manifold of one grazing point, because the clipped face contact missed its deepest corner, so nothing held it. With the deepest point of a deep overlap in the manifold (`collision3d.md`, deep overlaps) and contacts found ahead of fast impacts (`world3d.md`, fast pairs) the scene's deepest contact is 0.023 m (x64).
