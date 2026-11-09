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

A `soft` record follows the bodies (3D only): `soft preset kind x y z qx qy qz qw mass p…`. `preset` is `cloth`, `rubber` or `jelly` (each engine's own starting values for that kind of material); the pose places the mesh's origin; `mass` is the total mass in kg. `kind` and its parameters:

| kind | parameters | mesh (built by `bench/Bar/SoftGen.cs`, the same for every engine) |
|---|---|---|
| `cloth` | width depth n pinMode | an (n+1) x (n+1) grid in the xz plane, centred, two triangles per cell; `pinMode` 0 pins nothing, 1 the four corners, 2 the two corners of the edge at the lowest z |
| `box` | sx sy sz nx ny nz | a lattice of full extents sx x sy x sz with nx x ny x nz cells, six tetrahedra per cell (Kuhn subdivision, positive orientation), and the outward boundary triangles |
| `ball` | radius n | the lattice of an n x n x n cube with every point pulled onto the sphere by max-norm over length |
| `sleeve` | radius length around rings | an open tube along y from 0 to `length`, `rings` rings of `around` points (a multiple of 4, at least 8) starting at (radius, 0) and going round in the xz plane from +x towards +z, each quadrant by the rational parametrisation (1 - t^2, 2t) / (1 + t^2) with t = k / (around / 4); two triangles per quad. Skinned: see `soft-skinned-3d` (`bench/Bar/SleeveRig.cs`) |

A driver builds one soft body per record, in record order, and dumps six doubles per particle (x y z vx vy vz) after the bodies' data, particles in vertex order, bodies in record order (the extra channel that vehicles use for wheels).

The `character` record applies in 3D too: the last `body` must be a single y-aligned capsule (the character is centred on the body origin); the 3D scripts walk along +x. Vehicles get their own records.

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
| `buoyancy-3d` | 505 | 600 | 500 bodies dropped from 1 to 1.6 m above a 54 x 44 m pool of water 4 m deep (surface y = 0, `param fluid 0 1000 0.2 0 0.1`: a current of (0.2, 0, 0.1) m/s) on a 25 x 20 grid 2 m apart, five kinds in turn: spheres of density 500 (radius 0.45), flat boxes of 600 (1 x 0.5 x 1) and hexagonal prisms of 600 (radius 0.6, height 0.4), which float, and capsules and octahedra of 2500, which sink. Floaters cannot tip over (boxes twice as wide as tall, prisms wider than tall), so their resting heights are known from Archimedes (`param floatY`: sphere centre at the surface plus the cap solution; box and prism centre at the surface minus 0.6 of the height plus half the height) | equilibriumDepthError, sinkersAfloat, maxSpeed, nan, escapes, restKE |
| `tracked-3d` | 25 | 600 | 24 scripted tanks (`tank` preset, hull box 3.2 x 1 x 6 m, 20 t) on one seeded 61 x 241 height field (x 0..60, the vehicles terrain with bumps and a ramp, seed 41): six lanes 9.5 m apart, four rows 55 m apart. Tanks of a lane share a throttle (0.25 + 0.04 lane), steering starts at step 90 (amplitude 0.15, period 240), braking at step 450 | maxPenetration, maxSpeed, nan, escapes, tunnelled (below y = -15), wheelThroughGround, rollovers, suspensionJitter |
| `motorcycle-3d` | 21 | 600 | 20 scripted motorcycles (`bike` preset, chassis box 0.3 x 0.4 x 1.6 m, 250 kg) on a seeded height field (seed 42): five lanes 12 m apart, four rows 55 m apart. Bikes of a lane share a throttle (0.12 + 0.015 lane), a slalom starts at step 60 (amplitude 0.25, period 300), braking at step 450 | maxPenetration, maxSpeed, nan, escapes, tunnelled, wheelThroughGround, fallen, leanError, suspensionJitter |

**`car` preset** (also `VehicleDef.Car`): four wheels, 0.8 m either side of the centre and 1.3 m ahead and behind, at the chassis origin's height; radius 0.34 m, width 0.25 m; suspension 0.05 to 0.5 m at 1.5 Hz and damping ratio 0.6; front wheels steer to 0.5 rad; handbrake torque 4000 N m on the rear wheels, brake torque 3000 N m on all; wheel inertia 1 kg m^2, angular damping 0.2; engine 400 N m between 1000 and 6500 rpm, inertia 0.5; six-speed automatic with ratios 3.5, 2.2, 1.5, 1.1, 0.85, 0.7 and reverse 3.2; one rear differential of ratio 3.4; anti-roll bars of 20 kN/m on both axles. Everything else keeps each engine's defaults (tyre curves, torque curve, clutch), so the two engines differ where their defaults do. The Jolt driver builds this with `VehicleConstraint` and `WheeledVehicleController` and cuts the 61 x 241 height field into four square tiles (Jolt's height field is square) in a static compound.

| `ragdoll-powered-3d` | 661 | 600 | 50 powered humanoids of thirteen parts (swing-twist joints with limits, pose drive on the hips and shoulders tracking a looping pose) standing on a ground slab, substeps 8; every fifth is hit by a 1.56 kg box at 6 m/s | poseErrorP95, fallen, jointError, maxSpeed, nan, escapes |

### Soft bodies

| scene | bodies | steps | content |
|---|---|---|---|
| `cloth-drape-3d` | 3 (+1 soft) | 600 | a 4 x 4 m cloth of 24 x 24 cells (2 kg, `cloth` preset), pinned at the two corners of one edge, 2 m above a floor with a static sphere (radius 0.8) and a static box (1.2 m cube) under it: it falls over both and hangs from its pins |
| `soft-pile-3d` | 1 (+24 soft) | 600 | twelve `jelly` cubes (0.6 m, 3 x 3 x 3 cells, 3 kg) and twelve `rubber` balls (radius 0.35, 3 kg) dropped in a walled 5 x 5 m yard in two layers |
| `soft-rigid-3d` | 7 (+6 soft) | 600 | six `jelly` cushions (1.6 x 0.4 x 1.6 m, 6 x 2 x 6 cells, 8 kg) with a 0.5 m rigid box of 5 kg (density 40) released 1.25 to 1.75 m above each (0.8 to 1.3 m above the cushion): the cushions stop the boxes and push them back |
| `soft-mesh-3d` | 1 (+20 soft) | 600 | the triangle-mesh terrain of `mesh-terrain-3d` with ten `jelly` cubes (0.6 m, 2.5 kg) and ten `rubber` cubes (0.5 m, 2 kg) dropped 1 to 1.6 m above its hills near the origin |
| `soft-skinned-3d` | 3 (+1 soft) | 600 | a skinned `cloth` sleeve (`sleeve` kind: radius 0.1, length 1, 12 around x 11 rings, 0.5 kg) on a two-joint arm that swings (joint 0 about z by 0.5 x tri(step, 90), joint 1 about x by a further 0.9 x tri(step + 20, 70), tri a triangle wave from -1 to 1) above a floor, with a static sphere (radius 0.2) at (0.6, 0.9, 0) and a 0.15 m box (density 30) dropped on the top. The base ring is kinematic; the other rings may stray 0.06 m from their skinned position and keep out of a back-stop sphere of radius 0.04 centred 0.04 m behind it along the skinned normal. Only `onephysics` runs it |

The first four list `onephysics` and `jitter2` as their engines; `soft-skinned-3d` lists `onephysics` only (no other engine skins soft bodies). Its extra metrics: `skinMaxDistance` (the largest amount by which a free particle is farther from its skinned position than 0.06 m, over the dumped frames after the first, against an independent double-precision skinning of the pose set before the frame's last step), `skinBackstop` (the largest amount by which a free particle is inside the back-stop sphere), `skinCuffError` (the largest distance of a kinematic particle from its skinned position). Thresholds: `soft-skinned-3d` softMaxSpeed 5, softPenetration 0.02, softFinalPenetration 0.005, edgeStrain 0.15 (1.5 x ours, 2.6 / 0.0118 / 0 / 0.077 on the development machine, rounded up), skinMaxDistance 1e-4, skinBackstop 1e-4, skinCuffError 1e-5 (the limits are hard; ours measure 3e-8, 7e-8 and 6e-9, float32 rounding). Jolt's rendered documentation describes skinned soft bodies but its driver is not written, so there is no reference row.

 Metrics (soft bodies; computed from the particle dump by `Metrics3Soft.cs`; the particle sphere radius is a quarter of the body's mean rest edge length for every engine):

| metric | definition |
|---|---|
| `softMaxSpeed` | largest particle speed over all dumped frames, m/s |
| `softPenetration` | deepest particle sphere into any rigid shape over all dumped frames, m |
| `softFinalPenetration` | the median over the last 3 frames of the 95th-percentile particle depth |
| `softRestKE` | the median over the last 3 frames of the 90th-percentile per-particle kinetic energy per unit mass |
| `edgeStrain` | 95th percentile of abs(length / rest length - 1) over all edges in the last frame |
| `volumeError` | largest abs(V / V0 - 1) over the soft volumes in the last frame (sum of signed tetrahedron volumes) |
| `softEscapes` | particles outside the scene's container grown by the escape margin, worst frame (0 required) |
| `softTunnelled` | particles whose centre is inside a solid rigid shape in the last frame (triangle soups are not solid; 0 required) |
| `softNan` | non-finite particle values (0 required) |

Thresholds (maximum): `cloth-drape-3d` softMaxSpeed 10, softPenetration 0.07, softFinalPenetration 0.015, softRestKE 0.0007, edgeStrain 0.15 (1.5 x Jitter2's values 6.1, 0.040, 0.010, 0.00046, 0.093 on the benchmark machine, rounded up; Jitter2's rest energy varies between machines, 0.00018 to 0.00046, so the larger value sets the limit. Jitter2 is stable here, so this scene binds). `soft-pile-3d`: softMaxSpeed 10, softPenetration 0.02, softFinalPenetration 0.01, softRestKE 0.02, volumeError 0.05. `soft-rigid-3d`: softMaxSpeed 10, softPenetration 0.07, softFinalPenetration 0.01, softRestKE 0.001, volumeError 0.03, maxSpeed 10, restKE 0.1. `soft-mesh-3d`: softMaxSpeed 10, softPenetration 0.02, softFinalPenetration 0.01, softRestKE 0.01, volumeError 0.03. In the last three Jitter2 is not stable (its tetrahedra collapse and particles end inside the floor, walls and boxes: `softTunnelled` 340 to 380 in the pile and the cushions, volumeError 0.45 to 0.97), so these thresholds are this library's values with a margin and no reference binds them. Jitter2 is driven from its published API reference as vertices that are rigid bodies (mass spread evenly), one spring constraint per edge (20 Hz, damping ratio 0.5; 8 Hz and 0.3 for `jelly`; the documentation gives no stiffness values), triangle shapes (thickness half the mean edge length) or tetrahedron shapes, and its soft-body collision filters. Jolt's soft bodies are not driven yet.

### 2D soft bodies

Built in code by `tests/Common/SoftBodies2DScene.cs` from a seeded generator (not scene files; `scenes/SHA256SUMS` is unchanged), shared by the tests, the hash-matrix runner, the stability command `bar soft2d-stability` and the smoke apps. 600 steps at 1/60. Soft bodies have an areal density of 20 kg/m^2 (10 for the balloons of `balloon-2d`); the library default of 1000 is a slab of water.

| scene | contents |
|---|---|
| `soft-pile-2d` | 120 bodies of five kinds (jelly box, rubber disc, balloon, shape-matching blob, rubber slab) dropped in a 15 x 8 loose grid into a walled 12 m yard |
| `soft-rigid-2d` | a rigid pyramid of 15 boxes with a jelly box, a rubber disc and a balloon dropped on it; three jelly slabs under a rigid slab and box; four rigid circles dropped into the yard |
| `balloon-2d` | a height-field valley (0.15 x^2 capped at 4 m, 41 samples 0.5 m apart) between two walls, nine balloons dropped in and a rigid slab (density 20) dropped on them |

Metrics: `maxSpeed` (any particle, any step), `maxPenetration` (largest amount by which a particle disc is inside a rigid shape, from the closest-point query every 10th step), `areaError` (largest relative error of the enclosed area of a pressure body against its target over the last 3 steps), `maxSoftOverlap` (deepest overlap of two particle discs of different bodies at steps 100, 200, ...), `finalSpeed99` (99th percentile particle speed over the last 3 steps), `escapes` (particles beyond 40 m sideways, below -1 m or above 100 m) and `nan`. `escapes` and `nan` must be 0.

| scene | maxSpeed | maxPenetration | areaError | maxSoftOverlap | finalSpeed99 | measured (x64, 1 thread) |
|---|---|---|---|---|---|---|
| `soft-pile-2d` | 25 | 0.08 | 0.05 | 0.15 | 1.5 | 12.6 / 0.049 / 0.0065 / 0.093 / 0.19 |
| `soft-rigid-2d` | 15 | 0.05 | 0.02 | 0.10 | 1.5 | 9.8 / 0.031 / 0.0002 / 0.031 / 0.035 |
| `balloon-2d` | 20 | 0.06 | 0.02 | 0.05 | 1.0 | 12.2 / 0.038 / 0.0015 / 0.0003 / 0.10 |

Thresholds are about 1.5 times the measured values (taken before the sweep-direction rule in `softbody.md`, which lowered `finalSpeed99`), rounded up. A symmetric drop of a balloon stays symmetric (the sum of angular momentum about its centre is under 0.05 after 100 steps, test `SymmetricBalloonDropStaysSymmetric`). In the 120-body pile 2 bodies are asleep at 600 steps, 10 at 1200 and 24 at 3600 (the mean of the bodies' fastest particle is about 0.11 m/s against a sleep threshold of 0.05): the particles of touching bodies are discs, so bodies are porous and keep working against each other. That is open; a full sleep is not a threshold.

### Side-view vehicles and ragdolls (own scenes, no reference engine)

No other engine has vehicles or ragdolls to compare, so these four scenes are built in code by `tests/Common/Vehicles2DScene.cs` (seeded splitmix64 generator, 1 m height-field terrain: smoothstep hills between random knots, bumps, ramps) and measured by `bar vehicles2d-stability [--scene name] [--threads n] [--timing]`; the numbers are from the same code at 1 and 8 threads (identical). 600 steps at 1/60, default options (4 substeps; ragdolls with `VelocityIterations` 4).

| scene | what | thresholds | measured (1 and 8 threads) |
|---|---|---|---|
| `vehicles-2d` | six cars and two trucks, one per 160 m stretch of rough road (hills 0.35 m, bumps, a 0.4 m ramp), crates on the road, throttle 0.35 + 0.04 k, alternating pitch input from step 120, half brake from step 450 | `nan` = `escapes` = `rollovers` = `wheelThroughGround` = 0, `maxSpeed` <= 30, `maxPenetration` <= 0.2, `suspensionJitter` <= 0.01, `minProgress` > 30 | 0, 0, 0, 0, 18.5, 0.118, 0.0028, 62.1 |
| `tracked-2d` | four tanks on 120 m stretches of hills 0.5 m with a bump and a 0.6 m ramp, throttle 0.4 + 0.1 k, then brake | same, `maxSpeed` <= 15, `maxPenetration` <= 0.01, `minProgress` > 20 | 0, 0, 0, 0, 8.56, 0, 7e-6, 33.1 |
| `motorcycle-2d` | four bikes on 200 m stretches, a 1 m ramp with a step down (a jump), an air-control rider holding the bike level, then brake | `fallen` (past 1.2 rad) = 0 instead of `rollovers`, `maxSpeed` <= 30, `maxPenetration` <= 0.01 | 0, 0, 0, 0, 17.0, 0, 2e-6, 42.0 |
| `ragdoll-pile-2d` | 120 humanoid ragdolls (every fifth powered to the bind pose at half strength) dropped in 10 layers into a 14 m pit | `nan` = `escapes` = 0, `maxSpeed` <= 50, `maxPenetration` <= 0.5, `jointError` <= 0.15, `limitViolation` <= 1.5, `restKE` <= 1e-4 | 0, 0, 25.7, 0.31, 0.093, 0.76, 1.2e-5 |

Metrics: `rollovers` cars whose chassis passes 1.8 rad from upright, `wheelThroughGround` steps with a wheel centre under half a radius above the terrain, `suspensionJitter` the largest change of a suspension length between the last two steps for vehicles slower than 0.2 m/s, `minProgress` the shortest distance any vehicle drove, `maxPenetration` the deepest contact separation of any body, `jointError` the largest gap between the two anchors of a ragdoll joint (sampled every 10 steps), `limitViolation` the largest overshoot of a joint limit (rad, over the run), `restKE` the 90th percentile of per-body kinetic energy over the last 3 steps. Thresholds are 1.5 times the measured value rounded up to a 1/1.5/2/3/5/7 step, with floors. The ragdoll limit overshoot is a transient of the dense pile (a limit under the weight of many bodies), not a resting error; `JointLimitsHoldUnderADrop` in the tests holds a single ragdoll to 0.02 rad at rest.

### Boats, hovercraft, gliders and thrusters (own scenes, no reference engine)

No other engine has any of these, so the five scenes are built in code by `tests/Common/Actuators3DScene.cs` and `Actuators2DScene.cs` (scripts use integer arithmetic on the step number and the craft's own state, and onemath's cosine, so they are byte-identical on every platform) and measured by `bar actuators-stability [--scene name] [--threads n] [--timing]`. 600 steps at 1/60, default options. Thresholds are 1.5 times the measured values (x64 and arm64 agree to every printed digit, at 1 and 8 threads), rounded up; they are ours.

| scene | what | thresholds | measured |
|---|---|---|---|
| `boats-3d` | a 60 m harbour (floor, four quay walls, a breakwater), three dinghies (150 kg), two speedboats (800 kg), a barge (20 t) sailing with their presets and a helm that swings every 3 to 5 s, 12 loose crates | `nan` = `escapes` = `capsized` (up axis more than 78° from vertical) = `sunk` (origin 1.5 m under water) = 0, `maxSpeed` <= 12, `maxPenetration` <= 0.1 (a boat against the fixed scenery) | 0, 0, 0, 0, 7.78, 0.061 |
| `hovercraft-3d` | three craft (250, 300 and 400 kg) on an 80 m terrain of two hills and a pond (water only over the bowl, a limited fluid) behind a wall, fans spooled up over the first second, then a throttle that changes every 2 s and a helm that swings | same, `flipped` (up axis more than 60° from vertical) = 0, `maxSpeed` <= 10, `maxPenetration` <= 0.07, `minGap` >= 0.07 (smallest gap under any fan after the spool-up; the design gap is 0.146) | 0, 0, 0, 6.83, 0.043, 0.117 |
| `gliders-3d` | six gliders (wing and tail on a fuselage, wing loadings 0.8 to 2.4 kg/m²) launched from 400 m near trim speed in a 2 m/s wind and a 6 m/s gust column | `nan` = `escapes` = `tumbled` (pitch or roll more than 70°) = 0, `maxSpeed` <= 27, `maxSpin` <= 3.5 | 0, 0, 0, 17.6, 2.23 |
| `boats-2d` | a 80 m side-view channel with a pier, two dinghies, a speedboat and a barge sailing back and forth, 8 crates | as `boats-3d` with `maxSpeed` <= 6, `maxPenetration` <= 0.05 | 0, 0, 0, 0, 3.90, 0 |
| `thrusters-2d` | a 400 kg lander on a main engine and two side thrusters with a deterministic controller: from 30 m moving (1.5, −3) m/s to a 12 m hold, then a descent whose speed shrinks with the height | `nan` = `escapes` = 0, `maxSpeed` <= 20, `heightError` <= 0.5 (RMS distance from 12 m over steps 150 to 300), `landingSpeed` <= 1.0 (speed at step 600), tilt at the end <= 0.3 rad, `maxPenetration` <= 0.2 | 0, 0, 12.3, 0.312, 0.534, 0 tilt, 0 |

Metrics: `maxSpeed` the fastest body, `maxPenetration` the deepest contact separation of a craft with a static body, `minGap` the smallest fan gap, `maxSpin` the largest glider angular speed. The analytic checks are in `Actuator3DTests` and `Actuator2DTests`: the plate and wing curves against the documented model from -90° to 90° in 5° steps (relative error under 2e-4), a bluff body's drag on its silhouette, a glider that trims to a steady glide whose ratio (11.66 in 3D, 11.56 in 2D) matches the lift to drag of its surfaces within 5 %, a thrust-to-weight 1.0 rig that hovers with zero drift over 600 steps, a hovercraft at the predicted gap 0.14645 m (0.1464466 predicted, 0.14644659 measured), a dinghy at 4.84 m/s against a predicted 4.79 (3D) and 4.26 against 4.20 (2D), and a rudder that turns the dinghy right at 1.44 rad/s (3D, at 4.9 m/s, rudder alone).

## Step-state dump

Every driver writes the same binary file in stability mode. Little-endian. Header: the ASCII bytes `OPDUMP01`, `u32` body count, `u32` 0. Then frames: `u32` step number (0 = initial state), `u32` 0, and `7 × n` doubles per frame in scene body order: x, y, angle (body origin, radians), vx, vy (centre-of-mass velocity as the engine reports it), w, awake (1 or 0). In 3D the header word is 3 and a frame is `14 × n` doubles: x, y, z, qx, qy, qz, qw (body origin, unit quaternion), vx, vy, vz, wx, wy, wz (centre-of-mass velocity as the engine reports it), awake. A frame's second `u32` is the count of extra doubles after the body rows: for scenes with vehicles, x, y, z of every wheel centre of every car (four per car, in body order).

The `characters` scene has no reference rows. Proposed thresholds for this library's engine: `nan` = `escapes` = `tunnelled` = 0, `maxSpeed` ≤ 20, `maxPenetration` ≤ 0.25 (overlap between two scripted characters is not counted), `charJitter` ≤ 0.01. Measured at 1 thread: 0, 0, 0, 14.3, 0.15, 0.

The `characters-3d` scene has Jolt's `CharacterVirtual` as its one reference (driver written from its documented API; rows are the bar run's). Proposed thresholds for this library's engine: `nan` = `escapes` = `tunnelled` = 0, `stairSuccess` = 1, `slopeViolations` = 0, `maxPenetration` ≤ 0.05 (character-character overlap not counted), `charJitter` ≤ 0.01, `maxSpeed` ≤ 25 (a step up is one step's displacement over dt: 0.29 m in 1/60 s is 17 m/s). Measured at 1 thread: 0, 0, 0, 1, 0, 0.015, 0, 20.2.

The `vehicles-3d` scene has Jolt's `VehicleConstraint` as its one reference (driver written from its documented API, default tyres and clutch). Proposed thresholds for this library's engine: `nan` = `escapes` = `tunnelled` = `rollovers` = `wheelThroughGround` = 0, `maxSpeed` ≤ 25, `maxPenetration` ≤ 0.05, `suspensionJitter` ≤ 0.01. Measured at 1 thread (Release, worker, 2026-10-28): ours 0, 0, 0, 0, 0, 17.9, 0, 0.002. Jolt 5.6.0 with the preset above is **not stable** on this scene (escapes 2, tunnelled 2, rollovers 67 of 100, wheelThroughGround 128, maxSpeed 59, maxPenetration 0.12, suspensionJitter 0.11): most of its cars tip over on the bumps and ramp under the slalom. The driver may be missing tuning the documentation does not call for (for example a lowered centre of mass), so no leader is counted stable on this scene until someone tunes the Jolt car and re-runs it. Before the height-field tiling, a plain `HeightFieldShapeSettings` with 61 samples per side left everything beyond z = 60 without ground.

The `tracked-3d` and `motorcycle-3d` scenes have no reference rows: Jolt's tracked and motorcycle controllers have no driver yet. Proposed thresholds for this library's engine, from the runs at 1 and 3 threads (identical dumps) of 2026-10-28 on a local x64 machine: `nan` = `escapes` = `tunnelled` = `wheelThroughGround` = 0 on both, `maxPenetration` ≤ 0.05 and `suspensionJitter` ≤ 0.01 on both; tracked-3d `rollovers` = 0, `maxSpeed` ≤ 8 (ours 5.6); motorcycle-3d `fallen` = 0, `leanError` ≤ 0.05 rad (ours 0.006), `maxSpeed` ≤ 12 (ours 8.5). Measured: tracked-3d 0, 0, 0, 0, 0, 5.6, suspensionJitter 0.0002; motorcycle-3d fallen 0, leanError 0.006, maxPenetration 0, suspensionJitter 0.00003.


## Metrics

Computed by `bar metrics SCENE DUMP` from the dump and the scene alone, so every engine is judged by the same code. Shape penetration is measured geometrically from poses: for two convex cores (point, segment, triangle, polygon or point cloud) the signed distance between them is taken from the Minkowski difference (in 3D by GJK, with EPA for overlaps, in the bench tool itself; cores that only touch count as zero depth), and the skin radii are subtracted; skin radii are ignored in mass computation. Pairs on the same body, pairs of non-dynamic bodies and jointed pairs are skipped.

| metric | definition (maximum over dumped frames unless noted) |
|---|---|
| `maxPenetration` | deepest overlap of any shape pair, metres |
| `finalPenetration` | robust: the median over the last 3 dumped frames of the 95th-percentile depth of the overlapping pairs of that frame (the resting state; "Robust metrics" below). `finalPenetrationRaw`: the deepest pair in the final frame only |
| `maxSpeed` | largest centre-of-mass speed of any dynamic body, m/s |
| `nan` | count of non-finite values in the dump (0 required) |
| `restKE` | robust: the median over the last 3 dumped frames of the 90th-percentile body kinetic energy per unit mass, J/kg (2D: linear + rotational, shape-area mass; 3D: below). `restKERaw`: the mean over bodies in the final frame (2D mass-weighted) |
| `jitter` | robust: over the last 3 dump intervals, the 99th-percentile mean speed (distance moved over the interval) of the dynamic bodies, m/s; the median of the three. `jitterRaw`: the fastest single body over the last interval |
| `escapes` | most dynamic bodies outside `param container x0 y0 x1 y1` (grown by `param escapeMargin`, default 0.5 m) in any frame. `escapesRaw`: the same with the default margin |
| `tunnelled` | most dynamic bodies past `param wall x` (+ 0.1 m) or below `param floor y` in any frame |
| `collapse` | `topDrop` and `topLateral`: the drop in height and the sideways drift of `param top <body>` at the final frame |
| `charJitter` | characters scenes: largest change of a character's velocity between consecutive dumps over the last four dump intervals (characters stop at their `stop` step, so a character standing still gives 0; riders of turning platforms, listed in `rotatingRiders`, are skipped because their velocity turns by design), m/s |
| `stairSuccess` | share of the characters listed in `stairsN` (first body, count, height) that reach that height in any dump |
| `slopeViolations` | count of the characters listed in `steepN` (first body, count, height) that get above that height in any dump: they walk at ramps steeper than the slope limit |
| `wheelThroughGround` | vehicles: wheel samples (summed over all dumps) whose centre is lower than half a wheel radius above the terrain (`param terrainHf x0 z0`, `terrainMesh`: bilinear on the generator's grid) beneath it |
| `rollovers` | vehicles: cars whose body up axis has a y component under 0.2 in any dump (`param cars first count`; `param wheelsPerCar`, default 4, is the number of wheel centres each car dumps) |
| `suspensionJitter` | vehicles: for cars slower than 0.2 m/s in the last dump, the largest change of a wheel centre's height above the body centre between the last two dumps, metres |
| `fallen` (bikes) | motorcycles (`param bikes first count`): bikes whose up axis is more than 1 rad from vertical (y component under 0.54) in any dump |
| `leanError` | motorcycles: the median, over bikes and dumps from step 120 with a horizontal speed over 3 m/s, of the absolute difference between the lean (chassis up axis from vertical about its heading, positive towards its +x) and atan(v w / g), the lean a steady turn at speed v and yaw rate w needs, radians |
| `poseErrorP95` | powered ragdolls: 95th percentile, over all driven joints and all dumped frames from step 60, of the angle between the joint's current and target rotation, radians |
| `fallen` | powered ragdolls: humanoids (`param heads`) whose head centre is below half its initial height in the final frame |
| `jointError` | largest distance between the two world positions of any joint's anchors, metres |
| `asleepBeforeWake` | dynamic bodies asleep in the frame at the first `event` step (scenes create them asleep) |
| `awakeFinal` | dynamic bodies awake in the final frame |

3D differences: the per-body `restKE` is ½(v² + ω²R²), R the body's bounding radius (not mass-weighted; `restKERaw` is its mean over dynamic bodies); `escapes` and `tunnelled` test x, y and z against the container (z uses the x extent); `topLateral` is the horizontal distance in x and z.

## Robust metrics

A pile has chaotic contact order. A change as small as 1% of the contact frequency, or a different rounding order, changes which body is deepest, fastest or flung furthest in one frame, with no change in how stable the pile is. A pass or fail on a single body in a single frame is partly luck. The metrics that carry a threshold and showed this (`jitter`, `restKE`, `finalPenetration`, and `escapes` where a heavy impact throws bodies) therefore take **a high percentile across bodies or pairs and a median across a window of the last 3 frames or intervals** (`bench/Bar/Robust.cs`; nearest-rank percentiles, the median of an even count is the lower middle value). The window and the percentiles are constants in that file. Each robust metric is reported beside its old definition under the name with the suffix `Raw`, so nothing is hidden. Hard failures keep their definitions: `nan`, `tunnelled`, `maxSpeed` (explosions), `maxPenetration` (the deepest pair in any frame), `jointError`, `topDrop`/`topLateral`, `collapse`.

- **Window.** The last 3 dumped frames (3 intervals for `jitter`; 4 frames). Dumps are 60 steps apart in the pile and pyramid scenes and 30 apart in the others, so the window covers 1 to 2 simulated seconds.
- **`jitter`: 99th percentile of body speeds.** One body sliding on a rolled-over neighbour is not jitter; a pile in which more than 1% of the bodies keep moving is. Median over the window so one noisy interval does not decide.
- **`restKE`: 90th percentile of body energies.** Bodies that roll without rolling resistance (the heads of a ragdoll pile roll between the walls for the whole run, at 3 to 25 J/kg each) are not instability; a pile in which more than a tenth of the bodies still carries energy is.
- **`finalPenetration`: 95th percentile of pair depths.** Under a heavy load (the ball landing on the stacks in the sleep-wake scenes) the deepest pair varies from run to run by a factor of two; the depth that 95% of the overlapping pairs stay under does not. Pairs that do not overlap are not in the set, so a frame with a single overlapping pair reports that pair.
- **`escapes` margin.** In `sleep-wake-3d` a 335 kg ball lands at about 40 m/s on forty-high stacks and throws boxes off the 80 m slab. In nine perturbed runs 1 to 19 boxes were out of the container (grown by 0.5 m) in the last frame, up to 28 m past it and still flying (the scene ends 0.7 s after the impact). The scene sets `param escapeMargin 60`: a body counts as escaped only when it is more than 60 m outside the container, about twice the farthest throw seen. An explosion sends bodies much further and also trips `maxSpeed`. Scenes that do not set the param keep the 0.5 m margin: there the container is a closed set of walls and a body outside it has gone through a wall. Adding the param changed `scenes/SHA256SUMS` for `sleep-wake-3d` only.

### Perturbation sweep (this library's engine, 1 thread)

Nine runs per scene: contact frequency scaled by 0.97, 0.98, 0.99, 0.995, 1.00, 1.005, 1.01, 1.02 and 1.03 (the sweep hooks are environment variables read by the `onephysics` driver of `bench/Bar`: `BAR_CONTACT_HERTZ_SCALE`, `BAR_JOINT_HERTZ_SCALE`, `BAR_PUSH_OUT_SCALE`, `BAR_CONTACT_DAMPING`, `BAR_JOINT_DAMPING`, `BAR_SUBSTEPS`; the library is not touched). The old thresholds are those of the previous definitions.

| scene | metric | old threshold | raw min – max | robust min – median – max | robust spread / old threshold |
|---|---|---|---|---|---|
| large-pile | jitter | 0.25 | 0.00429 – 1.26 | 0.0322 – 0.0465 – 0.0533 | 0.084 |
| large-pile | restKE | 0.005 | 0 – 0.000695 | 0.000272 – 0.000817 – 0.00141 | 0.228 |
| large-pile | finalPenetration | 0.25 | 0.0923 – 0.117 | 0.0334 – 0.0352 – 0.0394 | 0.024 |
| large-pile-3d | jitter | 0.25 | 0.0124 – 0.181 | 0.00116 – 0.00288 – 0.00415 | 0.012 |
| large-pile-3d | restKE | 0.005 | 2.57e-06 – 5.1e-05 | 4.48e-07 – 4.01e-06 – 1.17e-05 | 0.002 |
| large-pile-3d | finalPenetration | 0.25 | 0.0257 – 0.038 | 0.00672 – 0.00723 – 0.00795 | 0.005 |
| pyramid-100 | finalPenetration | 0.04 | 0.0117 – 0.0131 | 0.00963 – 0.0102 – 0.0108 | 0.029 |
| pyramid-3d-30 | finalPenetration | 0.04 | 0.0016 – 0.00207 | 0.00113 – 0.00119 – 0.00126 | 0.003 |
| sleep-wake | finalPenetration | 0.15 | 0.124 – 0.284 | 0.0287 – 0.0316 – 0.0362 | 0.050 |
| sleep-wake-3d | finalPenetration | 1.0 | 0.257 – 0.51 | 0.0625 – 0.0994 – 0.124 | 0.062 |
| sleep-wake-3d | escapes | 10 | 1 – 19 | 0 – 0 – 0 | 0.000 |
| ragdoll-pile-3d | restKE | 0.5 | 0.102 – 0.706 | 2.54e-07 – 3.45e-06 – 1.46e-05 | 0.00003 |
| ragdoll-pile-3d | finalPenetration | 0.2 | 0.0130 – 0.173 | 0.00217 – 0.00313 – 0.00333 | 0.017 |

Every robust spread is under 0.3 of the old threshold, and the raw columns show the single-frame statistics that were not (large-pile `jitter` reached 1.26 in one run, ragdoll `restKE` 1.42, sleep-wake `finalPenetration` 0.28, sleep-wake-3d `escapes` 19).

### Forced-unstable controls

The robust metrics must still fail an unstable configuration. Each row is the same engine on the same scene with the named perturbation; values over the old threshold are in bold.

| scene | control | metric | value | old threshold | fails |
|---|---|---|---|---|---|
| large-pile | 1 substep (default 4) | jitter | **2.28** | 0.25 | yes |
| large-pile | push-out x500, contact damping 0 | jitter | **0.259** | 0.25 | yes |
| large-pile | 1 substep (default 4) | restKE | **1.28** | 0.005 | yes |
| large-pile | damping 0, contact frequency x1.6, push-out x10 | restKE | **0.111** | 0.005 | yes |
| large-pile | push-out x500, contact damping 0 | restKE | **0.718** | 0.005 | yes |
| large-pile | contact frequency x0.1 | finalPenetration | **0.516** | 0.25 | yes |
| pyramid-100 | 1 substep (default 4) | jitter | **18.2** | 0.05 | yes |
| pyramid-100 | damping 0, contact frequency x1.6, push-out x10 | jitter | **0.0562** | 0.05 | yes |
| pyramid-100 | 1 substep (default 4) | restKE | **3.98** | 0.01 | yes |
| pyramid-100 | damping 0, contact frequency x1.6, push-out x10 | restKE | **0.0164** | 0.01 | yes |
| pyramid-100 | 1 substep (default 4) | finalPenetration | **0.0841** | 0.04 | yes |
| pyramid-3d-30 | 1 substep (default 4) | jitter | **0.107** | 0.05 | yes |
| pyramid-3d-30 | damping 0, contact frequency x1.6, push-out x10 | jitter | **0.0792** | 0.05 | yes |
| pyramid-3d-30 | 1 substep (default 4) | restKE | **0.107** | 0.01 | yes |
| sleep-wake | contact frequency x0.1 | finalPenetration | **0.705** | 0.15 | yes |

Four controls (1 substep; "springy": damping 0, contact frequency x1.6, push-out x10; "mushy": contact frequency x0.1; "explosive": push-out x500, contact damping 0) were run on large-pile, pyramid-100, pyramid-3d-30, large-pile-3d, sleep-wake, sleep-wake-3d and ragdoll-pile-3d; only the rows that exceed the old threshold are listed. Every 2D pile and pyramid scene fails at least one robust metric under at least one control, and so does pyramid-3d-30 (and the sinking of a soft contact fails `finalPenetration` in sleep-wake). **Not shown to fail by any control:** `large-pile-3d` (the 3D pile stays quiet even at one substep), `ragdoll-pile-3d` (`restKE` 0.035 to 0.134 against 0.5) and the `escapes` of `sleep-wake-3d` (a body thrown beyond 60 m takes a speed above its `maxSpeed` cap first, and no solver setting reachable from the options sends one there). The thresholds of these three come from the references, not from a control.

### Values under the robust metrics

This library's engine, 1 thread, default options (robust / raw):

| scene | jitter | restKE | finalPenetration | escapes |
|---|---|---|---|---|
| pyramid-20 | 0 / 0 | 0 / 0 | 0.00206 / 0.00235 | 0 / 0 |
| pyramid-100 | 0 / 0 | 0 / 0 | 0.0102 / 0.0124 | 0 / 0 |
| pyramid-3d-10 | 0 / 0 | 0 / 0 | 0.000396 / 0.000477 | 0 / 0 |
| pyramid-3d-30 | 0 / 0 | 0 / 0 | 0.00120 / 0.00164 | 0 / 0 |
| large-pile | 0.0410 / 0.122 | 0.00110 / 3.4e-6 | 0.0353 / 0.110 | 0 / 0 |
| large-pile-3d | 0.00274 / 0.0124 | 1.6e-6 / 6.0e-6 | 0.00722 / 0.0335 | 0 / 0 |
| ragdoll-pile-3d | | 4.6e-6 / 0.416 | 0.00333 / 0.0167 | 0 / 0 |
| sleep-wake | | | 0.0362 / 0.129 | 0 / 0 |
| sleep-wake-3d | | | 0.0978 / 0.503 | 0 / 1 |
| tumbler, mixed-terrain | | | 0.0253 / 0.0661, 0.00356 / 0.0107 | 0 / 0 |
| mesh-terrain-3d, heightfield-terrain-3d | | | 0.000362 / 0.0196, 0.00893 / 0.0344 | 0 / 0 |

### `buoyancy-3d`

`param fluid <surface y> <density> <flow x y z>` tells a driver to fill everything below `y` with a fluid. Metrics: `equilibriumDepthError` (params `floatY`: pairs of body index and expected centre height; the 95th percentile over those bodies of the distance of the final height from it) and `sinkersAfloat` (params `sinkers`: body indices; how many are more than 1 m above `param poolFloor` at the end). `restKE` is the usual robust metric; with the 0.2 m/s current it cannot go below the kinetic energy of drifting with the water (0.025 per unit mass). Jolt 5.6.0 is driven through `BodyInterface::ApplyBuoyancyImpulse` (documented in its class reference): its buoyancy takes a factor where 1 is a neutral body, not a density, so the driver passes fluid density over the body's own density, with the documented typical drags (0.5 linear, 0.01 angular), the current as the fluid velocity, once per step before the update, for awake dynamic bodies. The other references have no buoyancy API, so the scene lists `onephysics` and `jolt` as its engines.

Proposed thresholds (this library, 1 thread, Release, a local run of 600 steps: equilibriumDepthError 0.0033, sinkersAfloat 0, maxSpeed 5.63, nan 0, escapes 0, restKE 0.0255): `nan` = `escapes` = `sinkersAfloat` = 0, `equilibriumDepthError` <= 0.02, `maxSpeed` <= 8 (the sinkers' terminal speed is near 5 m/s), `restKE` <= 0.04.

### Reference values and proposed thresholds

Measured on the x64 worker (Ryzen 9 7950X3D, 32 logical cores; load 26 before, 17 to 23 during, 18 after: other projects' work shared the machine, which does not change simulated results), one thread, .NET 10.0.13, the references at the versions and configurations pinned above, every engine through `bar run --mode stability` on the scenes regenerated from this commit. Values are the robust metric; `*` marks an engine that is not stable on that scene (the engines in the previous section's list, plus any whose `maxSpeed`, `maxPenetration` or `escapes` exceed the old caps), whose value does not set the threshold. The raw values of every engine are in `docs/spec/bench.md`.

**Rule.** The proposed threshold is 1.5 times the worst value among the stable engines other than ours, rounded up to 1, 1.5, 2, 3, 5 or 7 times a power of ten, and not below a floor of 0.01 for `jitter`, 0.0001 for `restKE` and 0.005 for `finalPenetration` (a pile that is at rest in every engine has zeros, and a threshold of zero would fail on rounding noise). `escapes` stays 0 everywhere. "ours ok" says whether this library's engine passes the proposed threshold.

| scene | metric | box2d | box2dnet | rapier2d | jolt | box3d | rapier3d | bepu | jitter2 | ours | proposed | ours ok |
| pyramid-20 | jitter | 0 | 0 | 0 |  |  |  |  |  | 0 | 0.01 | yes |
| pyramid-20 | restKE | 0 | 0 | 0 |  |  |  |  |  | 0 | 0.0001 | yes |
| pyramid-20 | finalPenetration | 0.00304 | 0.00301 | 0.00304 |  |  |  |  |  | 0.00206 | 0.005 | yes |
| pyramid-20 | escapes | 0 | 0 | 0 |  |  |  |  |  | 0 | 0 | yes |

<!-- pyramid-20: stable ['box2d', 'rapier2d', 'box2dnet']; unstable [] -->
| pyramid-100 | jitter | 0.115 | 0 | 0.00965 |  |  |  |  |  | 0 | 0.2 | yes |
| pyramid-100 | restKE | 0.00224 | 0 | 0 |  |  |  |  |  | 0 | 0.005 | yes |
| pyramid-100 | finalPenetration | 0.0152 | 0.0156 | 0.0151 |  |  |  |  |  | 0.0102 | 0.03 | yes |
| pyramid-100 | escapes | 0 | 0 | 0 |  |  |  |  |  | 0 | 0 | yes |

<!-- pyramid-100: stable ['box2d', 'rapier2d', 'box2dnet']; unstable [] -->
| large-pile | jitter | 0.0449 | 0.0137 | 0.078 |  |  |  |  |  | 0.041 | 0.15 | yes |
| large-pile | restKE | 0.00432 | 0.000543 | 0.00834 |  |  |  |  |  | 0.0011 | 0.015 | yes |
| large-pile | finalPenetration | 0.0489 | 0.0502 | 0.0499 |  |  |  |  |  | 0.0353 | 0.1 | yes |
| large-pile | escapes | 0 | 0 | 0 |  |  |  |  |  | 0 | 0 | yes |

<!-- large-pile: stable ['box2d', 'rapier2d', 'box2dnet']; unstable [] -->
| tumbler | finalPenetration | 0.0316 | 0.0342 | 0.0353 |  |  |  |  |  | 0.0253 | 0.07 | yes |
| tumbler | escapes | 0 | 0 | 0 |  |  |  |  |  | 0 | 0 | yes |

<!-- tumbler: stable ['box2d', 'rapier2d', 'box2dnet']; unstable [] -->
| sleep-wake | finalPenetration | 0.0355 | 0.0481 | 0.0351 |  |  |  |  |  | 0.0362 | 0.1 | yes |
| sleep-wake | escapes | 0 | 0 | 0 |  |  |  |  |  | 0 | 0 | yes |

<!-- sleep-wake: stable ['box2d', 'rapier2d', 'box2dnet']; unstable [] -->
| mixed-terrain | finalPenetration | 0.00503 | 0.00481 | 0.00504 |  |  |  |  |  | 0.00356 | 0.01 | yes |
| mixed-terrain | escapes | 0 | 0 | 0 |  |  |  |  |  | 0 | 0 | yes |

<!-- mixed-terrain: stable ['box2d', 'rapier2d', 'box2dnet']; unstable [] -->
| pyramid-3d-10 | jitter |  |  |  | 0 | 0 | 0 | 0 | 0 | 0 | 0.01 | yes |
| pyramid-3d-10 | restKE |  |  |  | 0 | 0 | 0 | 2.41e-07 | 2.12e-12 | 0 | 0.0001 | yes |
| pyramid-3d-10 | finalPenetration |  |  |  | 0.000546 | 0.000422 | 0.000405 | 0.00081 | 0.000935 | 0.000393 | 0.005 | yes |
| pyramid-3d-10 | escapes |  |  |  | 0 | 0 | 0 | 0 | 0 | 0 | 0 | yes |

<!-- pyramid-3d-10: stable ['jolt', 'box3d', 'rapier3d', 'bepu', 'jitter2']; unstable [] -->
| pyramid-3d-30 | jitter |  |  |  | 0 | 0 | 0 | 0.0457* | 0* | 0 | 0.01 | yes |
| pyramid-3d-30 | restKE |  |  |  | 0 | 0 | 0 | 0.0294* | 9.14e-06* | 0 | 0.0001 | yes |
| pyramid-3d-30 | finalPenetration |  |  |  | 0.00267 | 0.00132 | 0.00127 | 0.00215* | 0.0034* | 0.00125 | 0.005 | yes |
| pyramid-3d-30 | escapes |  |  |  | 0 | 0 | 0 | 0* | 0* | 0 | 0 | yes |

<!-- pyramid-3d-30: stable ['jolt', 'box3d', 'rapier3d']; unstable ['bepu', 'jitter2'] -->
| large-pile-3d | jitter |  |  |  | 0.00313 | 8.77e-06 | 0.000332 | 0.00664 | 0.000343 | 0.000121 | 0.01 | yes |
| large-pile-3d | restKE |  |  |  | 9.32e-08 | 0 | 5.95e-07 | 5.87e-06 | 2.97e-08 | 1.71e-08 | 0.0001 | yes |
| large-pile-3d | finalPenetration |  |  |  | 0.0305 | 0.00919 | 0.00518 | 0.00735 | 0.00998 | 0.00729 | 0.05 | yes |
| large-pile-3d | escapes |  |  |  | 0 | 0 | 0 | 0 | 0 | 0 | 0 | yes |

<!-- large-pile-3d: stable ['jolt', 'box3d', 'rapier3d', 'bepu', 'jitter2']; unstable [] -->
| ragdoll-pile-3d | restKE |  |  |  | 0.0315 | 0.00897 | 0.0131 | 0.0169 | 0.00486 | 4.6e-06 | 0.05 | yes |
| ragdoll-pile-3d | finalPenetration |  |  |  | 0.0202 | 0.00394 | 0.00172 | 0.00581 | 0.01 | 0.00333 | 0.05 | yes |
| ragdoll-pile-3d | escapes |  |  |  | 0 | 0 | 0 | 0 | 0 | 0 | 0 | yes |

<!-- ragdoll-pile-3d: stable ['jolt', 'box3d', 'rapier3d', 'bepu', 'jitter2']; unstable [] -->
| mesh-terrain-3d | finalPenetration |  |  |  | 0.02 | 0.00346 | 0.00115 | 0.000876 | 0.01* | 0.000367 | 0.03 | yes |
| mesh-terrain-3d | escapes |  |  |  | 0 | 0 | 0 | 0 | 3* | 0 | 0 | yes |

<!-- mesh-terrain-3d: stable ['jolt', 'box3d', 'rapier3d', 'bepu']; unstable ['jitter2'] -->
| heightfield-terrain-3d | finalPenetration |  |  |  | 0.0209 | 0.195* | 0.00115 | 0.000876 | 0.01* | 0.00893 | 0.05 | yes |
| heightfield-terrain-3d | escapes |  |  |  | 0 | 0* | 0 | 0 | 3* | 0 | 0 | yes |

<!-- heightfield-terrain-3d: stable ['jolt', 'rapier3d', 'bepu']; unstable ['box3d', 'jitter2'] -->
| sleep-wake-3d | finalPenetration |  |  |  | 0.119 | 0.033 | 0.0413 | 0.067 | 0.193 | 0.103 | 0.3 | yes |
| sleep-wake-3d | escapes |  |  |  | 0 | 0 | 0 | 0 | 0 | 0 | 0 | yes |

<!-- sleep-wake-3d: stable ['jolt', 'box3d', 'rapier3d', 'bepu', 'jitter2']; unstable [] -->

The comments list, per scene, which engines were counted stable.

**Findings.**
- `ragdoll-pile-3d` `restKE`: before the fix ours was **0.126** (raw 0.821) against 0.0049 to 0.032 for the others. The cause was spin: the heads and limbs of a pile of point-jointed ragdolls (spheres and capsules touching the floor or each other at one point or along a line) kept turning at 1 to 4 rad/s to the end of the run, because Coulomb friction alone has no torque to stop a spin and the engine had no rolling resistance or body damping by default. Every shape now defaults to a rolling resistance of 0.003 (`Material3D.DefaultRollingResistance`); ours is **4.6e-6** (raw 0.416) and 2.5e-7 to 1.5e-5 over the nine-run contact-frequency sweep. A body angular damping of 0.05 (about Jolt's default) left it at 0.14; 0.5 fixed it but would bleed every free spin, so it was not chosen.
- `heightfield-terrain-3d`: Box3D (0.195 deep, an alpha height-field finding) is not counted. `sleep-wake-3d`: all engines pass with `escapeMargin` 60; with the old 0.5 m margin Bepu, Box3D and Rapier3D threw 7, 4 and 5 boxes out of the container (`escapesRaw`).
- `pyramid-100` `jitter`: Box2D's robust value (0.115) is higher than its raw one (0.01): the window includes an earlier interval in which the pyramid was still settling.
- Scenes without a row (`joint-grid`, `chain`, `bullets`, `queries`, `characters`, vehicles) carry none of the four metrics except `escapes`, which is 0 for every stable engine.

## Public loading and exact-state scenes (version 2)

`RecursiveAI.OnePhysics.Scene.Scene` is the shared public version 1 model and
reader; the benchmark uses this reader, not a private copy. `Read(TextReader,
inputName)` validates record arities, finite numbers, material and joint
references, geometry and bounded generator sizes. `CreateWorld2D/3D` constructs
the world and validates engine-specific definitions. `SceneFormatException`
names the input, one-based line and expected value. Lines are bounded to 1 MiB,
total text to 128 MiB, bodies/joints to 100000 each, and saved snapshot payloads
to 64 MiB. Full-state snapshot contents also pass the world's existing validator.

`world.SaveScene()` or `SaveScene(TextWriter)` writes version 2. This is the
full-state form, not a lossy geometry export: it includes every currently
supported shape, joint, service, material, filter, option, soft body, solver
cache, slot generation, sleeping state, origin and pending event that snapshots
preserve. Contact/solver delegates and host code are not serialized. Pending live
simulation options are stored separately without synchronizing the core early. This form allows exact
save → load → save and hash equality even for a world already in motion.

The canonical records, in order, are:

```
scene 2
dim 2
world_id 24
state 1234
data <base64 chunk>
end
```

`dim` is 2 or 3. `world_id` is positive and preserves handle identity in event
payloads when creating the destination world. An optional `options <base64>` record before `state` uses the dimension-specific
LiveOptions layout in replay.md. World-produced scenes include it, preserving
pending option edits and fixed creation margins without mutating the source.
`state` is the exact nonzero
snapshot byte count. There are as many `data` records as needed; each encodes
3072 bytes (4096 base64 characters), except the last, which may be shorter.
Base64 is RFC 4648's standard alphabet and padding. The concatenated decoded
bytes are the complete little-endian world snapshot, unchanged, with its own
independent version. `end` is mandatory and no records may follow it. All record
terminators are LF and the encoding is UTF-8 without a BOM. The reader also
accepts CRLF and comments. A world id is file input, not a process-global id
allocation. Caller-selected thread count is not saved and does not change results.

`PhysicsWorld2D.LoadScene(reader)` and its identical 3D counterpart accept both
versions. The authoring vocabulary in version 1 remains unchanged for every
reference driver and the generated scene checksums do not change. Version 2 is
an exact-state interchange extension understood by this library and viewers
using it; reference benchmark drivers continue to consume version 1 authoring
records. A viewer can load either form and inspect the resulting world without
reflecting over scene or world types.

### Additional authoring shape records

The public scene v1 reader also accepts these additive shape kinds. The existing
`shape kind material density radius parameters...` prefix is unchanged:

- Both dimensions: `empty`, no parameters.
- 3D `plane`: normal xyz, offset; static bodies only. The normal is normalized
  with its offset, as in the collision specification.
- 3D `triangle`: three xyz vertices; the radius field is spherical inflation.
  Unit area density sets mass; inflation does not affect it.
- 3D `frustum`: half height, top radius, bottom radius; radius field unused.
- 2D `heightfield`: sample count n, spacing, n heights, n-1 hole flags (0 or 1).
  Samples start at body-local x=0; translate the body to place the field.
- `offset-<kind>`: the underlying kind's ordinary parameters followed by two
  (2D) or three (3D) local mass-centre offset components. One prefix is accepted
  in authoring text; arbitrary nested API decorators are preserved by exact-state
  scene v2 instead. Zero-area 2D kinds ignore the validated offset.

These records build through the same validated geometry constructors as the world
APIs. A scene using them is a public-library authoring scene; reference-engine
benchmark drivers need not accept these extension kinds. Existing benchmark
records, scene generation and their checksums are unchanged.

### Grid terrain study

`bar grid-study` builds `tile-world-2d` (64×2 half-metre tiles, a floor and
45-degree ramp, 20 boxes dropping at most 0.2 m and a moving character) and `voxel-world-3d`
(32×5×16 quarter-metre voxels, a floor and quarter-metre stairs, 8 boxes,
8 capsules and a moving character). Each runs 600 steps at 1/60 second, one
thread. The same run is repeated using the tile grid's already merged boundary
as chains, and the voxel grid's exposed faces as a triangle mesh. These are
representation comparisons within the library, not external engine rows.

Bounds: max body speed ≤20 m/s; no nonfinite poses or body-frame escapes
(2D X [-1,33], Y [0,10]; 3D X [-1,9], Y [0,10], Z [-1,5]); penetration of the
base horizontal floor by each rigid body's rotated support extent ≤0.03 m.
The floor metric is not a full terrain-pair penetration metric. Character X
progress must be at least 12 m in 2D and 4 m in 3D. The 3D character traverses
25 cm risers; the 2D one traverses the sloped tile surface with an explicit
50-degree slope limit (not the rounding boundary at 45 degrees). Its move
input is horizontal while grounded, and includes -2 m/s vertically only
while airborne; downward velocity must not cancel the requested uphill motion.

`--timing` first traverses both complete scenes without measurement to JIT
ramp-crest and stair-step paths, then constructs fresh worlds and adds 120
untimed settling steps (including stationary character
moves) before the 600 measured steps. It reports mean microseconds per step,
including character movement, without construction, I/O or metric reduction.
These are indicative comparisons for choosing representations, not speed-bar
rows; record machine, runtime, thread count and load with the output.

### Surface and indexed-material authoring records

After a `shape` record, optional `surface` supplies twelve floats:
`vx vy vz oneWayX oneWayY oneWayZ axisX axisY axisZ muLong muAcross spinning`.
Vectors are in the owning body's frame; SI units, defaults, limits, one-way
rules and dimension-specific exclusions are specified in `contacts.md`.
All twelve values are required; omission means an ordinary stationary,
two-sided isotropic surface. A record without a preceding shape is invalid.

`material_indices i0 i1 ...` after a shape assigns indices into the scene's
material table, one per mesh/height-field triangle or chain segment (one for
other shapes). Indices are nonnegative and within the material table. Empty
arrays are omitted by the writer. These two records are written immediately
after their shape, leaving older scenes byte-identical. Full-state version 2
scenes carry the optional runtime surface/material/motion sections unchanged.
