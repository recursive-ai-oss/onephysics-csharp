# Scene files and stability metrics

Normative, language-neutral. A scene is a plain-text file every harness reads, so one definition drives every engine. Scenes are produced by the generator in `bench/Bar/Gen.cs` (seeded, IEEE-exact arithmetic only, byte-identical on every platform); `scenes/SHA256SUMS` pins the bytes of each generated file. Generated files are not committed: `tools/bench/run.sh` regenerates and verifies them.

## Format, version 1

One record per line, tokens separated by spaces or tabs, `#` starts a comment. Numbers are decimal doubles in invariant culture (round-trip form). Records may appear in any order except that `shape` attaches to the most recent `body`. Positions, velocities, anchors and dimensions are double at the file boundary; engines convert to their own precision.

| record | fields | meaning |
|---|---|---|
| `scene 1` | | format version (first line) |
| `name` | text | scene name = file name |
| `dim` | 2 or 3 | dimension; selects the arities below |
| `dt` | seconds | fixed step; default 1/60 |
| `steps` | n | steps to run |
| `gravity` | gx gy (gz in 3D) | m/s² |
| `substeps` | n | omitted = each engine's default substep/iteration configuration |
| `dump_every` | n | stability runs dump the state every n steps, plus step 0 and the last step |
| `warmup` | n | leading steps excluded from timing rows |
| `settle`, `query_reps` | n | query scenes: steps to run before queries; repetitions (the first is untimed when more than one) |
| `metrics` | names | metrics this scene reports (below) |
| `param` | key values | metric parameters (below) |
| `material` | id friction restitution | the combine rule is each engine's default; scenes keep restitution 0 so the rules agree |
| `body` | type x y angle vx vy w flags | `static`, `kinematic` (moves at its velocity) or `dynamic`; angle in radians; velocities are the body's, w is angular; flags: `-`, or comma-separated `bullet` (continuous collision against everything), `asleep` (created sleeping) |
| `shape` | kind material density radius params… | see below; attaches to the last body; density in kg/m² |
| `joint` | revolute a b ax ay bx by [limit lo hi] [motor speed maxTorque] | body indices; anchors are body-local points; the joint angle is `angleB − angleA` measured from the initial pose, so limits and motors assume equal initial angles; bodies joined by a joint do not collide; a joint between two non-dynamic bodies is dropped |
| `query` | ray ox oy dx dy len / overlap cx cy r / cast cx cy r tx ty | closest ray hit with unit direction d and length len; circle overlap (counts shapes); circle sweep by translation t (closest hit) |
| `event` | step wake_all | before that step every dynamic body is woken |

Shape kinds (2D, local to the body; `radius` is the skin radius):

- `circle cx cy` with radius r.
- `capsule x1 y1 x2 y2` with radius r.
- `box hx hy cx cy angle` (half extents, centre, rotation), optionally rounded by `radius`.
- `polygon x1 y1 … xn yn`, convex, counter-clockwise, n ≤ 8, with skin `radius`.
- `segment x1 y1 x2 y2`, two-sided.
- `chain x1 y1 … xn yn`: an open two-sided polyline; engines create one segment per link.

### 3D (written down, not run yet)

Same records. Arities change: positions and anchors take x y z; `body` takes `x y z qx qy qz qw vx vy vz wx wy wz flags`; `gravity` takes three numbers. Shapes: `sphere cx cy cz`, `box hx hy hz` + local pose `cx cy cz qx qy qz qw`, `capsule`, `cylinder`, `hull` (point list; the engine builds the hull), `mesh` and `heightfield` (vertex/index lists or a grid, produced by the generator), `compound` (several `shape` lines on one body). Joints: `fixed`, `point`, `hinge`, `slider`, `cone`, `distance` with local frames (`ax ay az` + axis `ux uy uz`). Queries: ray, sphere overlap, sphere cast. Additional records `character` (capsule size, speed, input script) belong to the characters scene.

## Scene set

| scene | dim | bodies | steps | what it stresses | metrics |
|---|---|---|---|---|---|
| `pyramid-20`, `pyramid-100` | 2 | 211 / 5051 | 600 | box stacking, sleeping | maxPenetration, finalPenetration, collapse, maxSpeed, nan, restKE, jitter, escapes |
| `large-pile` | 2 | 4001 | 600 | ~4000 mixed circles, capsules, boxes, pentagons and rounded hexagons dropped into a container | maxPenetration, finalPenetration, maxSpeed, nan, escapes, restKE, jitter |
| `tumbler` | 2 | 2001 | 600 | a kinematic rotating box (0.5 rad/s) with 2000 boxes and circles | maxPenetration, finalPenetration, maxSpeed, nan, escapes |
| `joint-grid` | 2 | 10000 | 600 | 100×100 revolute grid hung from a static top row, 19701 joints | jointError, maxSpeed, nan, escapes |
| `chain` | 2 | 102 | 600 | 100-link revolute chain with a 125:1 mass ratio at the free end | jointError, maxSpeed, nan |
| `sleep-wake` | 2 | 3002 | 360 | 3000 boxes created asleep in 60 stacks, all woken at step 120, a heavy ball arrives near step 150 | maxPenetration, finalPenetration, maxSpeed, nan, escapes, asleepBeforeWake, awakeFinal |
| `mixed-terrain` | 2 | 1501 | 600 | 1500 mixed shapes on a 300-segment hilly polyline | maxPenetration, finalPenetration, maxSpeed, nan, escapes, tunnelled |
| `bullets` | 2 | 202 | 180 | 200 small bullets at 150 m/s against a 0.2 m wall (2.5 m per step) | tunnelled, maxSpeed, nan |
| `queries` | 2 | 4001 | 300 + queries | the large pile settled for 300 steps, then 4000 rays, 4000 circle overlaps, 4000 circle sweeps, 10 repetitions | maxSpeed, nan, escapes; throughput |

3D set (definitions for the follow-on harness): `pyramid` (3D box pyramid, base 10 and 40), `large-pile` (≈4000 boxes, spheres, capsules and hulls into a container), `joint-chains` (long hinge/point chains and a hinge grid), `ragdoll-pile` (hundreds of 15-body ragdolls dropped together), `mesh-terrain` and `heightfield-terrain` (bodies on generated terrain, seeded noise), `sleep-wake`, `characters` (hundreds of capsule characters following a scripted input path over steps and slopes), `queries` (ray, sphere overlap and sphere cast over the settled pile).

## Step-state dump

Every driver writes the same binary file in stability mode. Little-endian. Header: the ASCII bytes `OPDUMP01`, `u32` body count, `u32` 0. Then frames: `u32` step number (0 = initial state), `u32` 0, and `7 × n` doubles per frame in scene body order: x, y, angle (body origin, radians), vx, vy (centre-of-mass velocity as the engine reports it), w, awake (1 or 0). 3D adds the quaternion and extra velocity components with `dim 3` in the header word.

## Metrics

Computed by `bar metrics SCENE DUMP` from the dump and the scene alone, so every engine is judged by the same code. Shape penetration is measured geometrically from poses: for two convex cores (point, segment or polygon) the signed distance between them is taken from the Minkowski difference, and the skin radii are subtracted; skin radii are ignored in mass computation. Pairs on the same body, pairs of non-dynamic bodies and jointed pairs are skipped.

| metric | definition (maximum over dumped frames unless noted) |
|---|---|
| `maxPenetration` | deepest overlap of any shape pair, metres |
| `finalPenetration` | the same measure in the final frame only (the resting state) |
| `maxSpeed` | largest centre-of-mass speed of any dynamic body, m/s |
| `nan` | count of non-finite values in the dump (0 required) |
| `restKE` | kinetic energy per unit mass in the final frame, J/kg (linear + rotational, shape-area mass) |
| `jitter` | largest mean speed of any dynamic body over the last dump interval, m/s |
| `escapes` | most dynamic bodies outside `param container x0 y0 x1 y1` (grown by 0.5 m) in any frame |
| `tunnelled` | most dynamic bodies past `param wall x` (+ 0.1 m) or below `param floor y` in any frame |
| `collapse` | `topDrop` and `topLateral`: the drop in height and the sideways drift of `param top <body>` at the final frame |
| `jointError` | largest distance between the two world positions of any joint's anchors, metres |
| `asleepBeforeWake` | dynamic bodies asleep in the frame at the first `event` step (scenes create them asleep) |
| `awakeFinal` | dynamic bodies awake in the final frame |
