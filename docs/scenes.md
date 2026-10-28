# Scene files and stability metrics

Normative, language-neutral. A scene is a plain-text file every harness reads, so one definition drives every engine. Scenes are produced by the generators in `bench/Bar/Gen.cs` and `bench/Bar/Gen3.cs` (seeded, IEEE-exact arithmetic only, byte-identical on every platform); `scenes/SHA256SUMS` pins the bytes of each generated file. Generated files are not committed: `tools/bench/run.sh` regenerates and verifies them.

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
| `character` | speed period jump stop push | marks the last `body` (a single capsule) as a scripted character: walks at `speed` m/s along x, reverses every `period` steps, jumps every `jump` steps (0 = never), stops walking and jumping at step `stop`, `push` is the push strength (0 = never pushes). Engines without a character mover (the references) see a plain dynamic capsule; only the `onephysics` engine runs the script (a scene may set `param engines onephysics`, and the runner skips the others) |
| `vehicle` | preset throttle steerAmp period steerFrom brakeFrom | 3D: marks the last `body` (a box) as the chassis of a scripted car of preset `car` (table below). Throttle is held until step `brakeFrom`, then full brake; between `steerFrom` and `brakeFrom` the steering is a triangle wave of amplitude `steerAmp` (fraction of full lock) and period `period` steps that starts at its peak (cosine-like: a slalom whose sideways offset stays on one side). Drivers without vehicles see a plain box |
| `event` | step wake_all | before that step every dynamic body is woken |

Shape kinds (2D, local to the body; `radius` is the skin radius):

- `circle cx cy` with radius r.
- `capsule x1 y1 x2 y2` with radius r.
- `box hx hy cx cy angle` (half extents, centre, rotation), optionally rounded by `radius`.
- `polygon x1 y1 … xn yn`, convex, counter-clockwise, n ≤ 8, with skin `radius`.
- `segment x1 y1 x2 y2`, two-sided.
- `chain x1 y1 … xn yn`: an open two-sided polyline; engines create one segment per link.

### 3D

Same records and the same rules; `dim 3` changes the arities:

| record | 3D fields |
|---|---|
| `gravity` | gx gy gz |
| `body` | type x y z qx qy qz qw vx vy vz wx wy wz flags (unit quaternion; angular velocity in world axes) |
| `joint` | `point` a b ax ay az bx by bz: a ball-and-socket joint; the anchors are body-local points (measured from the body origin) that coincide in the initial pose; bodies joined by a joint do not collide. `hinge` a b ax ay az bx by bz ux uy uz [limit lo hi] is reserved for the joint stage and not used by any scene yet. `swingtwist` a b ax ay az bx by bz ux uy uz [limit lo hi] [swing sy sz] [drive k c max ax ay az amp period]: a ball joint with a twist axis; the joint frame is the shortest-arc rotation taking the x axis to `u`; `limit` bounds the twist angle, `swing` the two swing half-angles; `drive` (axis `a` in the joint frame) pulls B toward a target rotation about that axis with stiffness `k` (N m/rad), damping `c` (N m s/rad) and torque cap `max` (N m); the target angle is `amp` times a triangle wave 0 to 1 to 0 over `period` steps, evaluated at the index of the step about to run |
| `query` | `ray ox oy oz dx dy dz len` (unit direction, closest hit); `overlap cx cy cz r` (sphere, counts shapes); `cast cx cy cz r tx ty tz` (sphere sweep by t, closest hit) |

Shape kinds (3D, local to the body; `radius` is the skin radius where one applies):

- `sphere cx cy cz` with radius r.
- `box hx hy hz cx cy cz qx qy qz qw`: half extents, then the local pose of the box.
- `capsule x1 y1 z1 x2 y2 z2` with radius r: the scenes only use capsules parallel to the y axis.
- `hull x y z …`: the convex hull of the points, built by each engine (at most 64 points).
- `mesh nv ni v… i…`: `nv` vertices (x y z each) then `ni` indices (three per triangle, counter-clockwise seen from above for terrain); static bodies only.
- `heightfield nx nz cell ox oy oz h…`: `nx × nz` samples, row-major with `nx` samples along x per row, cell size `cell` in x and z, lower corner at body-local `(ox, oy, oz)`, heights `h` added to `oy`; static bodies only, split into two triangles per cell with the diagonal from `(i, j+1)` to `(i+1, j)`.

The `character` record applies in 3D too: the last `body` must be a single y-aligned capsule (the character is centred on the body origin); the 3D scripts walk along +x. Vehicles and soft bodies get their own records when their stages arrive.

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
| `characters` | 2 | 347 | 600 | 300 capsule characters on mixed terrain (240 on the ground, 60 on 6 kinematic platforms) walking, jumping and pushing 40 crates; characters share a layer that ignores itself | charJitter, escapes, tunnelled, maxPenetration, maxSpeed, nan |
| `queries` | 2 | 4001 | 300 + queries | the large pile settled for 300 steps, then 4000 rays, 4000 circle overlaps, 4000 circle sweeps, 10 repetitions | maxSpeed, nan, escapes; throughput |

3D scenes (dimension 3; generator `bench/Bar/Gen3.cs`, same seeded rules). Gravity is (0, −10, 0); every box, sphere, capsule and hull has density 1 unless noted; friction 0.6, restitution 0.

| scene | bodies | steps | what it stresses | metrics |
|---|---|---|---|---|
| `pyramid-3d-10`, `pyramid-3d-30` | 386 / 9456 | 600 | box pyramids with base 10 and 30 on a ground slab, stacked exactly in contact | maxPenetration, finalPenetration, collapse, maxSpeed, nan, restKE, jitter, escapes |
| `large-pile-3d` | 3001 | 600 | 3000 randomly tilted spheres, y-aligned capsules, boxes and two convex hulls (an octahedron and a wedge) dropped into a walled container | maxPenetration, finalPenetration, maxSpeed, nan, escapes, restKE, jitter |
| `joint-chains-3d` | 672 | 600 | 16 horizontal chains of 40 point-jointed boxes pinned to a static anchor, ending in a 21 kg sphere (about 500:1 against a link); they swing down | jointError, maxSpeed, nan |
| `joint-grid-3d` | 1600 | 600 | 40×40 point-jointed grid hung from a static top row, 3081 joints | jointError, maxSpeed, nan, escapes |
| `ragdoll-pile-3d` | 1501 | 600 | 150 limp ragdolls of ten parts (a box torso, a sphere head, eight capsule limbs) joined by 1350 point joints, dropped into a container | jointError, maxPenetration, finalPenetration, maxSpeed, nan, escapes, restKE |
| `mesh-terrain-3d`, `heightfield-terrain-3d` | 1297 | 600 | the same 63-cell seeded terrain (64×64 samples, 2 m cells) as a triangle mesh and as a height field, with 1296 mixed bodies dropped on it | maxPenetration, finalPenetration, maxSpeed, nan, escapes, tunnelled (below y = −15) |
| `sleep-wake-3d` | 1442 | 360 | 1440 boxes created asleep in 36 stacks of 40, all woken at step 120, a 2 m radius, density-10 ball dropped from 81 m lands on the stacks | maxPenetration, finalPenetration, maxSpeed, nan, escapes, asleepBeforeWake, awakeFinal |
| `queries-3d` | 3001 | 300 + queries | the pile settled for 300 steps, then 3000 rays, 3000 sphere overlaps and 3000 sphere casts, 10 repetitions | maxSpeed, nan, escapes; throughput |
| `characters-3d` | 393 | 600 | 300 scripted capsule characters (radius 0.3, height 1.8; slope limit 45 degrees, step 0.3) on: a triangle-mesh hill field and a height-field hill field (50 each), 40 crates with 40 pushers (half pushing), three staircases of twelve 0.5 m treads with 0.20, 0.25 and 0.29 m risers (12 climbers each), a field of 1 m blocks in 0.25 m levels built as one mesh (40 walkers, a quarter of them jumping), four ramps of 30, 40, 50 and 60 degrees (10 walkers each), and six kinematic platforms (three sliding at 1.5 m/s, three turning at 0.4 rad/s) carrying 44 characters. Characters share a layer that ignores itself. Params `engines onephysics jolt` (only these drivers run characters), `stairsN first count minY`, `steepN first count maxY`, `rotatingRiders first count…` | charJitter, stairSuccess, slopeViolations, escapes, tunnelled, maxPenetration, maxSpeed, nan |
| `vehicles-3d` | 102 | 600 | 100 scripted cars (`car` preset, chassis box 1.8 x 0.5 x 4.2 m, 1500 kg, origin at the wheel tops) in two fields of 50: ten lanes 5.8 m apart, five rows 25 m apart; the left field on a 61 x 241 height field (x -60..0), the right on a triangle mesh of the same seeded rolling terrain (x 0..60) with bumps and a ramp. Cars of a lane share a throttle (0.35 + 0.04 lane), steering starts at step 90 (amplitude 0.12, period 150), braking at step 450 | maxPenetration, maxSpeed, nan, escapes, tunnelled (below y = -15), wheelThroughGround, rollovers, suspensionJitter |

**`car` preset** (also `VehicleDef.Car`): four wheels, 0.8 m either side of the centre and 1.3 m ahead and behind, at the chassis origin's height; radius 0.34 m, width 0.25 m; suspension 0.05 to 0.5 m at 1.5 Hz and damping ratio 0.6; front wheels steer to 0.5 rad; handbrake torque 4000 N m on the rear wheels, brake torque 3000 N m on all; wheel inertia 1 kg m^2, angular damping 0.2; engine 400 N m between 1000 and 6500 rpm, inertia 0.5; six-speed automatic with ratios 3.5, 2.2, 1.5, 1.1, 0.85, 0.7 and reverse 3.2; one rear differential of ratio 3.4; anti-roll bars of 20 kN/m on both axles. Everything else keeps each engine's defaults (tyre curves, torque curve, clutch), so the two engines differ where their defaults do. The Jolt driver builds this with `VehicleConstraint` and `WheeledVehicleController` and cuts the 61 x 241 height field into four square tiles (Jolt's height field is square) in a static compound.

| `ragdoll-powered-3d` | 661 | 600 | 50 powered humanoids of thirteen parts (swing-twist joints with limits, pose drive on the hips and shoulders tracking a looping pose) standing on a ground slab, substeps 8; every fifth is hit by a 1.56 kg box at 6 m/s | poseErrorP95, fallen, jointError, maxSpeed, nan, escapes |

Soft bodies are defined when their stage arrives.

## Step-state dump

Every driver writes the same binary file in stability mode. Little-endian. Header: the ASCII bytes `OPDUMP01`, `u32` body count, `u32` 0. Then frames: `u32` step number (0 = initial state), `u32` 0, and `7 × n` doubles per frame in scene body order: x, y, angle (body origin, radians), vx, vy (centre-of-mass velocity as the engine reports it), w, awake (1 or 0). In 3D the header word is 3 and a frame is `14 × n` doubles: x, y, z, qx, qy, qz, qw (body origin, unit quaternion), vx, vy, vz, wx, wy, wz (centre-of-mass velocity as the engine reports it), awake. A frame's second `u32` is the count of extra doubles after the body rows: for scenes with vehicles, x, y, z of every wheel centre of every car (four per car, in body order).

The `characters` scene has no reference rows. Proposed thresholds for this library's engine: `nan` = `escapes` = `tunnelled` = 0, `maxSpeed` ≤ 20, `maxPenetration` ≤ 0.25 (overlap between two scripted characters is not counted), `charJitter` ≤ 0.01. Measured at 1 thread: 0, 0, 0, 14.3, 0.15, 0.

The `characters-3d` scene has Jolt's `CharacterVirtual` as its one reference (driver written from its documented API; rows are the bar run's). Proposed thresholds for this library's engine: `nan` = `escapes` = `tunnelled` = 0, `stairSuccess` = 1, `slopeViolations` = 0, `maxPenetration` ≤ 0.05 (character-character overlap not counted), `charJitter` ≤ 0.01, `maxSpeed` ≤ 25 (a step up is one step's displacement over dt: 0.29 m in 1/60 s is 17 m/s). Measured at 1 thread: 0, 0, 0, 1, 0, 0.015, 0, 20.2.

The `vehicles-3d` scene has Jolt's `VehicleConstraint` as its one reference (driver written from its documented API, default tyres and clutch). Proposed thresholds for this library's engine: `nan` = `escapes` = `tunnelled` = `rollovers` = `wheelThroughGround` = 0, `maxSpeed` ≤ 25, `maxPenetration` ≤ 0.05, `suspensionJitter` ≤ 0.01. Measured at 1 thread (Release, worker, 2026-10-28): ours 0, 0, 0, 0, 0, 17.9, 0, 0.002. Jolt 5.6.0 with the preset above is **not stable** on this scene (escapes 2, tunnelled 2, rollovers 67 of 100, wheelThroughGround 128, maxSpeed 59, maxPenetration 0.12, suspensionJitter 0.11): most of its cars tip over on the bumps and ramp under the slalom. The driver may be missing tuning the documentation does not call for (for example a lowered centre of mass), so no leader is counted stable on this scene until someone tunes the Jolt car and re-runs it. Before the height-field tiling, a plain `HeightFieldShapeSettings` with 61 samples per side left everything beyond z = 60 without ground.

## Metrics

Computed by `bar metrics SCENE DUMP` from the dump and the scene alone, so every engine is judged by the same code. Shape penetration is measured geometrically from poses: for two convex cores (point, segment, triangle, polygon or point cloud) the signed distance between them is taken from the Minkowski difference (in 3D by GJK, with EPA for overlaps, in the bench tool itself; cores that only touch count as zero depth), and the skin radii are subtracted; skin radii are ignored in mass computation. Pairs on the same body, pairs of non-dynamic bodies and jointed pairs are skipped.

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
| `charJitter` | characters scenes: largest change of a character's velocity between consecutive dumps over the last four dump intervals (characters stop at their `stop` step, so a character standing still gives 0; riders of turning platforms, listed in `rotatingRiders`, are skipped because their velocity turns by design), m/s |
| `stairSuccess` | share of the characters listed in `stairsN` (first body, count, height) that reach that height in any dump |
| `slopeViolations` | count of the characters listed in `steepN` (first body, count, height) that get above that height in any dump: they walk at ramps steeper than the slope limit |
| `wheelThroughGround` | vehicles: wheel samples (summed over all dumps) whose centre is lower than half a wheel radius above the terrain (`param terrainHf x0 z0`, `terrainMesh`: bilinear on the generator's grid) beneath it |
| `rollovers` | vehicles: cars whose body up axis has a y component under 0.2 in any dump (`param cars first count`) |
| `suspensionJitter` | vehicles: for cars slower than 0.2 m/s in the last dump, the largest change of a wheel centre's height above the body centre between the last two dumps, metres |
| `poseErrorP95` | powered ragdolls: 95th percentile, over all driven joints and all dumped frames from step 60, of the angle between the joint's current and target rotation, radians |
| `fallen` | powered ragdolls: humanoids (`param heads`) whose head centre is below half its initial height in the final frame |
| `jointError` | largest distance between the two world positions of any joint's anchors, metres |
| `asleepBeforeWake` | dynamic bodies asleep in the frame at the first `event` step (scenes create them asleep) |
| `awakeFinal` | dynamic bodies awake in the final frame |

3D differences: `restKE` is the mean over dynamic bodies of ½(v² + ω²R²), R the body's bounding radius (not mass-weighted); `escapes` and `tunnelled` test x, y and z against the container (z uses the x extent); `topLateral` is the horizontal distance in x and z.
