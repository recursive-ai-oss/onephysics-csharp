# World scale: streamed tiles, simulation level of detail, local time scale

Normative, language-neutral. Both dimensions share one implementation of the registry, the planner and the step groups; 2D and 3D differ only in the geometry a tile may hold.
Everything here is deterministic: it depends on the world's state, options and the calls made, never on thread count, platform or the order tile calls were made within one step.

## Snapshot section

Optional section **WSCL** (`0x4C435357`, version 1), registered after every other section of each world (so after the shallow-water section), written and hashed only while any of its state is in use; a world that never uses it keeps its bytes and hashes.
Layout (little-endian, in this order):

| Field | Type |
|---|---|
| version | i32 = 1 |
| LOD substep factor | i32, 0 (automatic) to 8 |
| far period | i32, 1, 2, 4 or 8 |
| focus count, then 3 doubles each | i32, f64[] (z is 0 in 2D); at most 64 |
| band count, then per band: max distance f32, period i32 | at most 16; distances positive, finite, ascending |
| LOD region count, then per region: 6 f64 (min xyz, max xyz), period i32 | at most 256 |
| time-scale region count, then per region: 6 f64, scale f32 | at most 256; scale in [1/1024, 4] |
| body scale count, then per body: slot i32, scale f32 | ascending slots of live dynamic bodies; scale in [1/1024, 4], never exactly 1 |
| tile count, then per tile: x, y, z i32, unloading bool, body count i32, slot i32 each | ascending by (x, y, z); every slot a live static body, no body in two tiles |

All coordinates in the section are **host coordinates** (stored position plus the world origin), so `ShiftOrigin` never changes the section. Every other part of the state is the ordinary body, shape and broadphase state.
In descriptions (desync reports, region archives) the configuration and the tile registry are entity `worldscale`; the per-body scales belong to their bodies. A region archive therefore carries the scales of the bodies it holds and
leaves the registry, regions and focus points of the receiving world alone; the section's byte length is part of the snapshot framing, so a regional restore needs the same registry size and the same number of regions and bands.

## Streaming static tiles

A tile is a key `(x, y, z)` (z is 0 in 2D) and a set of static bodies. `LoadTile(key, def)` and `UnloadTile(key, policy)` queue requests; the world applies them at the first of: the start of a step, hashing, snapshotting or sizing a snapshot,
forking, creating or destroying a body or shape, `FlushTiles()`. A flush applies, in this order:

1. **Unloads**, ascending by key. A tile marked unloading stays while any live contact or joint constraint joins one of its bodies to a dynamic body (awake or asleep) and the policy is `WaitForContacts`; the mark is part of the snapshot and the world retries at every flush. `Immediate` removes the tile at once, ending its contacts with the usual end events.
2. **Loads**, ascending by key (two requests for one key: the later wins; an unload cancels a queued load). Each part of the tile is built as a static body in the order height fields, meshes (tile grids in 2D), boxes, voxel grids (3D), each in list order. Body, shape and broadphase slots come from the usual lowest-free-index rule, so the same sequence of other calls plus the same set of tiles gives the same slots and hash whatever order the tile calls came in.

A load of a key that is loaded, queued or unloading is an error; so is a def with too many parts, an array of the wrong length or a non-finite origin. A def whose geometry is invalid when built (non-finite heights, an out-of-range position) is dropped at flush time, counted in `FailedTileLoads` with the message in `LastTileError`; the flush never throws. Queued requests are not state: a restore discards them.
A tile's origin is stored in host coordinates when queued and turned into a position relative to the world origin when built, so an origin shift between the call and the build changes nothing.

**Seams.** A height field may carry the heights of the ring of samples just outside its grid (`HeightFieldEdges3D`: west and east columns, south and north rows; corners are not needed; in 2D a height before the first sample and one after the last). They are only used to judge the active edges along the border (the same test as for an edge inside the field), so a border edge between flat or concave neighbours is inactive and a body crossing the seam meets no ghost contact. They add no collision. Recipe kind 17 (3D height field with edges) = the height-field recipe followed by a mask byte (bit 0 west, 1 east, 2 south, 3 north) and the sides that are set, in that order, `float32` each. 2D chain segments carry their ghost vertices already.
Meshes and 2D tile grids have no neighbour information: their border edges stay active.

## Simulation level of detail

`SetSimulationLod(focus points, bands, far period, regions, substep factor)`. A body's period is the smallest period of the regions that contain its position; outside every region it is the period of the first band whose radius reaches the distance to the nearest focus point, else the far period; with no bands or no focus points it is 1. Periods are 1, 2, 4 or 8.

Each step, before anything else: every awake island gets the smallest period of its bodies, and the smallest time scale (below). Islands holding a body that is pinned (see below) get period 1 and scale 1. An island is **due** when `(StepCount + 1) mod period == 0`. An island that is not due is suspended for the whole step: it is made to look asleep (no contacts are updated, no pairs are searched, nothing is integrated, no events, sleep timers keep their value), and is restored, awake, at the end of the step. A due island advances `dt * period * scale` (see groups).
A body outside the world's rhythm joins it at the next due step; its clock therefore gains or loses up to `period - 1` steps once per crossing, and its velocity and momentum are untouched.
A contact created in the step between an awake body and a suspended island wakes it silently (no wake event) and the merged island takes the finer period for that step.
Sleeping composes: only awake islands are planned, an island at period k accrues `k * dt` of rest time per due step, and sleeping and waking events are the usual ones.

**Pinned**: an island touching a kinematic body (a platform moves every step), and an island holding a body the engine asked to keep on the world's joint schedule (vehicles, bicycles, rail vehicles, rope and pulley machines). Characters, soft bodies, rods and fluids are stepped by their own stages at the full rate whatever the regions say; a body that something else drives each step should be kept in period-1 regions.

**Substeps.** A due step of period k runs the world's substeps times the factor: 0 (default) means 1, except 2 at period 8 (a pile of boxes does not hold at one substep of 8 steps), else the given 1 to 8.

## Local time scale

A body's scale is its own scale (1 by default) times the smallest scale of the time-scale regions that contain it; an island runs at the smallest scale of its bodies, so a fast body that touches a slow one runs with it for as long as they are one island, and momentum is conserved (equal and opposite impulses within one island; a scale never changes a velocity). Scale is in [1/1024, 4]. An island advances `dt * scale` (and `dt * period * scale` with level of detail); a scale above 1 multiplies the substeps by it (rounded up), so a substep stays at most as long as the world's.
Static and kinematic bodies have no scale; a kinematic body runs at the world's rate. Joints break and expire in world time. Sleep time accrues in world time, as steps.

## Step groups

When any region, band or scale is configured the step plans twice: before the narrowphase (to suspend what is not due and to size contact margins for the longest advance, `dt * max(1, period * scale)`) and after it (new contacts may have merged islands). The islands due are then grouped by `(period, scale)`, ordered by `period * scale`, then period, then scale; there are at most 8 groups (with more distinct pairs the group list is thinned evenly and every island joins the group of the largest multiplier not above its own, never faster than asked). Kinematic bodies are integrated once, in the group `(1, 1)` (a pass of their own if thinning removed it). Each group runs the ordinary solve stages (coloring, prepare, the substep programs, finalize, store) with only its islands awake and `dt * multiplier`; then sleeping, breaks and events run once over the due islands. A world with nothing configured takes the ordinary path, bit for bit.

## What the engines' own stages do

Stages outside the rigid solve (vehicles, characters, soft bodies, rods, fluids, fields, water) keep their world rate. The contact force reported by hit events is impulse over the world's `dt`, so for an island at another rate it is off by the multiplier.

## Debug geometry

`DebugContent.WorldScale` (4194304) fills `Tiles` (key, unloading, body slots), `ScaleRegions` (level-of-detail and time-scale boxes, relative to the world origin) and `LodFocusPoints`.
