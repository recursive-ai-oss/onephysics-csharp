# Shallow water

A body of shallow water is a regular grid of square cells in which the water's depth and depth-averaged momentum evolve by the Saint-Venant (shallow-water) equations over a bed, with floating bodies that are pushed by it and push it back. One store serves both worlds. A 3D world has grids over the x–z plane with height along y; a 2D world has one row of cells along x with height along y (a side-view channel: the water has unit depth, volumes are areas). This page is normative: a port reproduces the same bits from it. Reference documentation for the types is in the XML comments of `ShallowWaterDef`, `ShallowWater`, `WaterBoundary`, `WaterSource` and `WaterEvent`.

## State

Per grid (slot, generation; lowest free slot first, as every handle table):

- Layout: `planar` (2D), `nx`, `nz` (`1` when planar), cell size `dx` (float32, metres, the cell is `dx` square; a planar cell has width `dx` and unit depth), the grid's minimum corner `originX`, `originZ` (double, rounded to the 2^-32 grid at creation; `originZ = 0` when planar) and `baseY` (double, the floor height, the zero of every stored height).
- Coefficients (float32): `density`, `linearDrag`, `angularDrag`, `viscousDamping` (the fluid constants bodies see, as in a `FluidDef`), `bottomFriction`, `damping`, `displacement`, `cfl`, and `maxSubsteps` (int32).
- Four sides (west = −x, east = +x, south = −z, north = +z): kind (`Wall` 0, `Open` 1, `Inflow` 2, `Level` 3), level above `baseY` and speed (float32).
- Sources: position and radius in metres from the minimum corner, rate (m³/s; m²/s planar).
- Cells, row-major with x fastest: bed height `B` (float32, above `baseY`, constant over its cell), depth `H` (float32, ≥ 0) and momentum `Hu` (along x) and `Hv` (along z) (float32). A planar grid keeps `Hv = 0`.
- `lostVolume` (double): water that a removal or a step could not take without a negative depth.
- Body records, ascending body slot: slot, generation, the volume added to the water for that body, the volume it displaces, and its footprint `x0, x1, z0, z1` (float32, metres from the minimum corner).

Heights are relative to `baseY`, so moving the world origin moves `originX`, `originZ` and `baseY` and nothing else: a shifted world steps bit for bit like the unshifted one, translated.

## The scheme

The surface is `w = H + B`. A cell is dry when `H ≤ 0.001 m`; a dry cell has zero momentum and holds no water at its faces. All arithmetic is float32 with IEEE-exact operations in the fixed order of the code below; no fused operations, no transcendental functions (`sqrt` only).

Per substep (Heun's method, the two-stage strong-stability-preserving Runge–Kutta): copy `U0 = (H, Hu, Hv)`; reconstruct and compute face fluxes from `U`; choose the substep length `ds` (below); `U ← U + ds·L(U)`; reconstruct and compute fluxes again; `U ← U + ds·L(U)`; `U ← ½ (U0 + U)`; then the damping and bottom friction factors are applied to the momenta (implicit, so they only slow the water), momenta of dry cells set to zero, and a planar grid's `Hv` set to zero. A depth that an update would make negative is set to zero and the shortfall added to `lostVolume`. After the average a depth that is NaN or negative becomes 0, one above 4,194,304 m is cut to it, momentum is cut to ±1000 m/s times the depth (NaN becomes 0), so the state stays finite and saveable even when `maxSubsteps` stretches a step past stability; a volume added or removed is limited the same way.

**Reconstruction.** For every wet cell whose neighbours along the axis are wet (and which is not at the grid's edge), the slope of `w`, of the normal momentum and of the momentum along the face is the generalised minmod `minmod(θ(m − l), (r − l)/2, θ(r − m))` with `θ = 1.3` (`minmod` returns the argument of least magnitude when all three share a sign and 0 otherwise); anywhere else it is 0 (first order next to a dry cell and at the edge). Slopes are differences across a cell; the value at a cell's face is the cell value ± half the slope.

**Faces.** For the face between a cell on the low side `L` and one on the high side `R`:

1. Each cell presents its own depth `hOwn = max(0, w ± s/2 − B)` at the face, its velocities `u = vel(hOwn, q)` and `v = vel(hOwn, r)` with `q`, `r` the momenta at the face and `vel(h, q) = √2 h q / √(h⁴ + max(h⁴, ε))`, `ε = 0.001⁴` (Kurganov and Petrova 2007, desingularisation). A dry cell presents depth 0, velocity 0 and surface `B`.
2. Hydrostatic reconstruction (Audusse et al. 2004): `Bf = max(B_L, B_R)`, `h*_L = max(0, w_L − Bf)`, `h*_R = max(0, w_R − Bf)`, where `w_L`, `w_R` are the surfaces at the face.
3. The central-upwind flux (Kurganov and Petrova 2007) of the states `(h*, u, v)`: `c = √(g h*)`, `a⁺ = max(u_L + c_L, u_R + c_R, 0)`, `a⁻ = min(u_L − c_L, u_R − c_R, 0)`; with `d = a⁺ − a⁻`, `k = a⁺a⁻/d`, the fluxes of mass, normal momentum and along-face momentum are `(a⁺F(U_L) − a⁻F(U_R))/d + k (U_R − U_L)` with `F = (h u, h u² + g h²/2, h u v)` and `U = (h, h u, h v)`. When `d ≤ 10⁻⁷` the flux is zero; when the two states are bitwise equal it is `F(U)` itself.
4. The cell on the low side takes the normal-momentum flux plus `P(hOwn_L) − P(h*_L)` and the cell on the high side the flux plus `P(hOwn_R) − P(h*_R)`, with `P(h) = ½ g h²` computed as `0.5f · g · h · h` and the flux term as `(F − P(h*)) + P(hOwn)`, so a lake at rest cancels to the last bit.

**Edges.** Wall: mass and along-face flux 0, normal momentum flux `P(hOwn)` of the cell inside. Open: the flux of the inside state against itself. Inflow: the flux of the state `(h = max(0, level − B), u = ± speed into the grid)`, normal momentum `h u² + P(h)`. Level: depth `h = max(0, level − B)` at the face and the normal velocity from the Riemann invariant the inside sends out, `u = u_in − 2c_in + 2c` at a low edge and `u_in + 2c_in − 2c` at a high edge; its flux is the state's own. (The wave speed an edge sees is `|u| + c`.)

**Update.** With `rdx = 1/dx`: `dH = −(Fh_east − Fh_west)·rdx − (Gh_north − Gh_south)·rdx`, and likewise for the momenta from the face fluxes of their direction (a planar grid has no `G` terms). Cell order is row-major; a task is `4` rows, a fixed split independent of thread count, and each task writes only its own cells and faces, so every thread count gives the same bits.

**Substeps.** The largest signal speeds `a_x`, `a_z` over all faces of the first reconstruction of a substep give `ds = cfl / (a_x/dx + a_z/dx)` (planar: `cfl·dx/a_x`; `dt` when both are 0). A substep is `min(ds, remaining)`; the substep that would be number `maxSubsteps` takes all the remaining time. The count depends only on the state. Maxima are exact, so their order is irrelevant.

**Damping and friction** (after the average): `f = 1/(1 + ds·damping)` multiplies both momenta; with `bottomFriction = cf > 0`, `f = 1/(1 + ds·cf·|v|/h)` where `|v| = √(u² + v²)` of the averaged state.

**Sources** act once per step before the substeps: a source covers the cells whose index range `[floor(x/dx − r/dx), floor(x/dx + r/dx)]` (clamped, same along z) contains it, and adds `rate·dt/(count·cellArea)` to each; a negative result is clamped at zero depth with the shortfall in `lostVolume`.

## Beds

The bed is one height per cell. At creation it is the supplied array, or the top of the static shapes under each cell's centre (a ray cast straight down from `max(level, floor) + searchHeight` to the floor; the nearest static hit counts; no hit or a hit below the floor gives the floor), or the floor. A bed height is `max(0, height − floor)`. `ResampleBed` does the same with the water kept.

## Bodies

The store runs in two serial stages of the step, grids in slot order.

**Before the rigid step** (after fluids, before the actuators, so a boat sees the surface of the start of the step): for each grid, bodies in ascending slot order that are dynamic. A record whose body is gone or no longer dynamic takes its volume out of the water first. For a body with shapes (sensors excluded) and the union of the shapes' tight boxes `[minX, maxX] × [minZ, maxZ]` overlapping the grid:

- A sleeping body is woken (island and sleep timer) when the water around it moves faster than 0.05 m/s or its surface is steeper than 0.01, and otherwise left alone, its record unchanged.
- The **reference surface** is read just outside the box: samples at `minX − dx/2`, `maxX + dx/2` (and `minZ − dx/2`, `maxZ + dx/2` in 3D) along the box's middle, each a bilinear sample (clamped to the grid) of the surface height `w`, the horizontal velocity and the depth; samples where the depth is 0.001 m or less are dropped, and with none left the body is clear of the water. The reference surface at the box's centre is the mean of the surface samples, the slope along an axis the difference of its two samples over `(box size + dx)`, clamped to ±1, and the water velocity the mean of the velocities. The water under the body is left out on purpose: it holds the volume the body displaced, and reading it makes the pair an undamped oscillator.
- The body's buoyancy and drag are those of a flat fluid whose surface is that plane (normal `(−s_x, 1, −s_z)/‖·‖`, offset along it through the reference point), whose flow is the water velocity and whose density, linear drag, angular drag and viscous damping are the grid's: the same routine as an `AddFluid` fluid (3D) or buoyancy field (2D), including the mid-step lead and the boat hooks. It returns the submerged volume `V` (area in 2D). A body whose box lies above the tilted surface is not cut.
- Records: a body with `V > 0` has a record holding `V` and `V × displacement`, and the box; the volume that was recorded before is removed from its old footprint and the new one added at the new footprint, both queued. A body with `V = 0` whose record exists loses it. An enter event is raised when a record appears and a leave event when it goes, with the submerged volume.

**After the rigid step**: the queued volumes are added in queue order (removal then addition for each body; slot order between bodies), then every grid advances by `dt`. Adding a volume spreads it over the cells the footprint overlaps in proportion to the overlap area; removing never takes a cell below zero (the shortfall goes to `lostVolume`). A body destroyed while recorded takes its volume out of its footprint at once. The total of the cells' water minus the recorded volumes is conserved to float rounding with closed sides and no sources.

Why the displacement is a factor (`displacement`, default 0.5): with all of the volume added, a long hull in a narrow channel rings (the body pushes water up, the raised water around it pushes the body back a few steps later); half of it is stable in the scenes below, and the water volume bookkeeping stays exact because the same fraction is removed again.

Static shapes are the bed. A kinematic body does not float (its motion is its own), but it displaces water exactly like a dynamic one: the same submerged volume, times `displacement`, put under its footprint each step and taken out where it was, so a paddle or a plunger makes waves. It takes part from the first step it overlaps a grid; no force is added to it. Soft bodies, cloth, rods and characters are below.

## Soft bodies, cloth and rods

Every particle of a soft body, cloth or rod (awake soft bodies; rods always) is a small blob of the volume it stands for, treated before the soft and rod steps, grids in slot order, soft bodies by slot then particle, rods by slot then particle:

- **Volume.** A soft body with a volume (tetrahedra, or a closed surface held by pressure; in 2D triangles or a closed outline) gives a particle the share `V·m_i/M` of its rest volume `V` (rest area in 2D); a body with none (cloth, strips, open surfaces) gives it a ball (disc) of the particle radius. A rod particle gets the circular cross-section times half the lengths of its segments (a strip of width 2r in 2D).
- **Level.** The surface plane at the body: for a soft body the water read around the box of its particles as for rigid bodies (a plane at the box's centre with slope along x and z, and the water's velocity there); for a rod particle the level around the particle, a segment-length wide.
- **Submerged fraction.** `t = (level − (y − s)) / (2s)` clamped to [0, 1] and `f = t²(3 − 2t)`, the volume fraction of a sphere cut by the level, with `s` the larger of the particle radius and half the particle's cell size (`∛V` in 3D, `√V` in 2D; for a rod half a segment). The submerged volume is `f·V`.
- **Buoyancy** adds `ρ_w·g·f·V·(1/m)·dt` upward to the particle's velocity (a particle with zero inverse mass, pinned or kinematic, feels none).
- **Drag** takes the particle's velocity relative to the water and removes `(½ρ_w·C_d·A·f·|v|² + ν·ρ_w·f·V·|v|)·(1/m)·dt` of it along its direction, never more than all of it; `C_d` is the grid's `LinearDrag`, `ν` its `ViscousDamping`, `A` the particle's frontal area (`π s²`; `2s` in 2D; for rods the segment length times the diameter).
- **Displacement.** `f·V·displacement` is deposited in the cell under the particle. The grid keeps, per cell, what was deposited the step before (state, ascending cell index); each step it adds the difference between the new and the old amounts, so the water the bodies hold is exact and leaves when they leave. A cell cannot be pushed below zero depth (the shortfall is counted in `LostVolume`).
- A sleeping soft body is left alone unless the water at it moves faster than 0.05 m/s or its surface slopes more than 0.01, which wakes it.

Buoyancy volumes (`AddFluid`) and fluid fields do not yet act on soft bodies or rods.

## Characters in water

See `world3d.md` → Swimming. A character is a body like any other for the water: a virtual character's kinematic capsule displaces water (so it makes a wake), a rigid character floats like any dynamic body. The swim state is read from the same grids.

## The water the actuators see

Boats, thrusters, hovercraft fans and aerodynamic surfaces ask for the water at a point. Where a shallow water covers the point's x and z with a wet cell, the answer is the tangent plane of the surface there (the bilinear surface, slope from differences of half the point's reach, at least one cell, at most four, clamped to ±1) moving at the local velocity, with the grid's density, and it comes before any flat water; elsewhere the old answers are unchanged.

## Queries and events

`QueryWater` returns surface, depth, velocity and the unit normal at a point's x and z from the first grid in slot order whose footprint contains it and whose water there is deeper than 0.001 m. `RaycastWater` clips the ray to each grid's box (footprint, floor to highest surface), marches half a cell along it, and where the sign of `y − surface` changes between wet samples refines by 24 bisections; the nearest hit over the grids wins; dry cells have no surface. Events (`WaterEvents`, `BodyWaterChanged`) are in grid slot order then body slot order, delivered after the step unless callbacks are suppressed, and cleared at the start of the next step.

## Snapshot section

Optional section `"SWTR"` (`0x52545753`), version 1 (version 2 while any grid holds water displaced by soft bodies, cloth or rods; see below), registered after every other section in both worlds, written only while a grid exists, so worlds that never use water keep their bytes and hashes. Layout (little-endian, entity kinds `shallow water` by slot, records as `body` by body slot): `int32 version`; the handle table of grids; for each live grid in slot order: `bool planar`, `int32 nx`, `int32 nz`, `float32 dx`, `float64 originX`, `float64 originZ`, `float64 baseY`, `float32 density, linearDrag, angularDrag, viscousDamping, bottomFriction, damping, displacement, cfl`, `int32 maxSubsteps`, four sides of (`byte kind`, `float32 level`, `float32 speed`), `int32 sourceCount` and per source `float32 x, z, radius, rate`, `float64 lostVolume`, `nx·nz` bed values, `nx·nz` depths, `nx·nz` momenta along x, `nx·nz` momenta along z (float32 each), `int32 recordCount` and per record `int32 slot`, `int32 generation`, `float32 volume`, `float32 submerged`, `float32 x0, x1, z0, z1`. The reader checks sizes against the bytes left before allocating, every float finite, depths in `[0, 2^22]`, coefficients in range, kinds known, records ascending and naming live dynamic bodies of the saved generation. In version 2 each grid ends with `int32 holdCount` and per entry `int32 cell` (ascending) and `float32 volume` (above 0, at most 1e12): the displaced volume the grid holds for soft bodies and rods (the reader checks the count against the cell count). Version 1 files read as an empty hold. Records name dynamic or kinematic bodies. Scratch (slopes, fluxes, the stage copy) and the pending volumes are not saved. A fork copies the grids; a region snapshot keeps every grid whole and the records of the selected bodies.

## Debug geometry

`DebugContent.ShallowWater` (524288): for each grid in slot order a `DebugWaterSurface` (slot, cells, cell size, minimum corner with `y = baseY`, offset into the arrays) and, per cell, `WaterHeights` (surface above `baseY`; the bed where dry) and `WaterDepths` (0 where dry).

## Limits

At most 4,194,304 cells and 1,048,576 cells along an axis per grid; cell size 0.001 to 1000 m; 64 sources; coordinates within ±2^21 m; `maxSubsteps` 1 to 256 and `cfl` up to 0.5; a step that needs more than `maxSubsteps` is stretched and less accurate. Gravity must point along −y; the grid uses `−gravity.y` and does not flow when it is zero or negative.
