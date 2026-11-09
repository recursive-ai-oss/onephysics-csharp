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
the world. A multi-obstacle minimum-translation solver is not supplied here.

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
Only translations are supported; angular sweeps are not yet provided.

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
