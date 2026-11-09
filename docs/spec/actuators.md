# Aerodynamics, thrusters, boats and hovercraft

Normative description of `src/OnePhysics/Aero/` and the parts of `Physics2D/` and `Physics3D/` that carry it: lift and drag of shapes, thrusters, boats and hovercraft in both dimensions. One implementation serves both worlds; a planar world presents its bodies to it as 3D vectors with z = 0 and takes back only the x and y force components and the z torque. Everything is float32 with IEEE `+ − × ÷ sqrt`; every transcendental comes from onemath (`SinCos`, `Atan2`, `Cbrt`); no fused operations.

## What runs when

One serial stage per step, in both worlds, before the core step and after the fields, the vehicles (and in 2D the ragdoll drive) and the fluid buoyancy: `Actuators.Step(dt)`. In order: **shape surfaces** by shape slot; **thrusters** by slot; **boats** by slot; **hovercraft** by slot. Everything adds to the bodies' force and torque accumulators (world axes, through the centre of mass, torque about it), so the solver integrates them with its own substeps. Nothing is parallel, so no combine order is needed. At the start of the step (before the fields) `BeginStep` forgets the submerged measures of the last step.

Sleeping bodies: a sleeping body feels no surface forces and runs no actuator unless it is commanded (a thruster with a non-zero throttle or output, a boat with a throttle or steer, a hovercraft with lift or throttle), in which case its island is woken and its sleep timer reset every step. A hovercraft wakes a sleeping dynamic body it blows on. The throttle lag snaps to exactly zero below 1e-4 once the throttle is closed, so a stopped craft can sleep.

Bodies, shapes and components are referred to by slot and generation; destroying a body drops its thrusters, boat and hovercraft and every surface of its shapes; destroying a shape drops its surface.

## Air, wind and water

* `AirDensity` (default 1.225 kg/m³, 0 to 10^4) belongs to the world and is saved.
* **Wind** at a point is the sum, in field slot order, of the `Flow` of every field of kind `Area` whose box holds the point (box inclusive, `Minimum`/`Maximum`). A field that is not an area field adds nothing, and the field's own force or acceleration is separate (a wind field with a zero direction pushes no body by itself).
* **Water** is looked up by walking the fluids of a 3D world in slot order and then the buoyancy fields in slot order (both worlds). The first whose surface the point is below (or within a given reach above) and whose region covers it wins. A water sample is the unit surface normal, the offset along it (double), the density, the flow and the optional box. Height above the water is `float(n·p − offset)` with the dot product in double.
* **Immersion** of a thing of size `s` whose reference point is at height `h` above the surface: `clamp(0.5 − h / s, 0, 1)`: 0.5 at the surface, 1 once `s/2` below, 0 once `s/2` above. The size is `sqrt(area)` for a surface, `ImmersionDepth` for a propeller.

## The lift and drag model

A lifting surface has a chord direction `c` (from the leading edge to the trailing edge, so the air at zero angle of attack moves along `+c`) and a lifting side `n`, unit and perpendicular (`n` is made perpendicular to `c` when attached). The public definition gives `Forward` (the direction it flies, so `c = −Forward`) and `Up` (`n`). With `u` the velocity of the air relative to the surface point (wind − velocity of the point, the point's velocity being `v + ω × r`), `u_c = u·c`, `u_n = u·n`, speed `V = sqrt(u_c² + u_n²)` (flow along the third axis makes no force), `α = atan2(u_n, u_c)`:

```
(CL, CD) = AeroCoefficients.Evaluate(α)
F_c = q S (CD cos α − CL sin α)       F_n = q S (CD sin α + CL cos α)       q = ½ ρ V²
F   = F_c c + F_n n
```

`AeroCoefficients.Evaluate`: `x = α − α0`; attached `CL = a·x`, `CD = CD0 + k CL²`; separated (flat plate, Hoerner) `CL = C90 sin α cos α`, `CD = CD0 + C90 sin² α`; `t = clamp((|x| − stall) / blend, 0, 1)`, `w = t²(3 − 2t)`, result `attached + w (separated − attached)`. Presets: `Wing(AR)`: `a = 2π / (1 + 2/AR)` (finite wing), `k = 1/(π · 0.85 · AR)`, stall 0.28, blend 0.3, `CD0` 0.02; `Plate(AR)`: the same slope, `k = 1/(π · 0.9 · AR)`, stall 0.17, blend 0.35, `CD0` 0.01; both with `C90 = 1.98` for `AR ≥ 20` and `1.1 + 0.018 AR` below it (Hoerner's flat-plate broadside drag). `Bluff(Cd)` has `CD0 = C90 = Cd`.

A **bluff** surface is drag only: `F = ½ ρ Cd A |u| u`, `A` the fixed area or, when the area is 0, the silhouette of the shape seen along `u` (body frame) every step.

**Area** `S` of a lifting surface: the definition's, or when 0 the silhouette of the shape seen along its lifting side `n`, taken once (cached, not saved, recomputed identically after a restore). In 3D the silhouette is the projected area of the buoyancy cutter with the whole shape below its plane (spheres a disc, hulls and boxes their polyhedron, compounds the sum of their children, meshes and height fields none); in 2D it is the extent of the shape's points across the direction plus twice its radius, per metre of depth (circles, capsules and polygons; segments and chains none). Planar areas and forces are per metre of depth.

**Where it acts.** At `shape offset + shape-to-body rotation · CenterOfPressure` from the centre of mass: in 3D the shape's frame, in 2D the shape's centroid with the offset in body axes (planar shapes have no frame of their own).

**Limit.** The force is scaled down so that applied for one step at its point it could at most take the point's speed relative to the air to zero: `|F| ≤ speed / (dt · w)` with `w = 1/m + (r × F̂)ᵀ I⁻¹ (r × F̂)` the effective inverse mass at the point along the force (the planar `w = 1/m + (r × F̂)² / I`). Without it a light body with a long surface and a large rotational damping from `ω × r` overshoots and the explicit step diverges (a 1.2 kg, 0.23 kg·m² glider did).

A surface in water (`Medium = Water`) takes the density of the water times its immersion and its flow; out of the water it makes nothing.

## Thrusters

A thruster (`ThrusterDef`) has a position and direction in the body frame (direction made unit), `MaxThrust`, `ResponseTime`, `MaxGimbal`, an optional gimbal axis, `ReverseFraction`, `SpinTorque`, `PitchSpeed`, a medium and an `ImmersionDepth`. Per step:

1. `target = clamp(throttle, −ReverseFraction, 1)`; `output += (target − output) · dt / (ResponseTime + dt)` (implicit first order: stable at any step); output snaps to 0 as above.
2. The axis is turned by the gimbal commands, each a fraction clamped to ±1 of `MaxGimbal`: the first about the gimbal axis (when none is given: about z in a planar world, about the unit of `axis × hint` in 3D with `hint = Y` unless `|axis.y| ≥ 0.9`, then `X`), the second (3D, when the first axis is not zero) about `axis × first axis`; Rodrigues rotations with onemath `SinCos`.
3. Water thrusters: no water at the point or immersion 0 → no force; otherwise thrust × immersion. `PitchSpeed > 0`: thrust × `clamp(1 − v_axial · sign(output) / PitchSpeed, 0, 1)` with `v_axial` the point's velocity relative to the medium along the axis.
4. `F = axis · MaxThrust · output · …` at the position; torque `r × F` plus, with `SpinTorque ≠ 0` (3D only), `axis · SpinTorque · |thrust|` signed with the thrust.

`Thruster.Gimbal` in 3D is a `Vector2F` of the two commands, in 2D a float. Positive 2D gimbal is counter-clockwise.

## Boats

`BoatDef`: forward and up axes (body frame), hull drag coefficients (longitudinal, lateral, vertical), `HeaveDamping`, `MaxRudderAngle`, `RudderRate`, foils (position, area, aspect ratio, `Horizontal`, `SteerGain`), propellers (thrusters whose medium is forced to water). In a planar world the lateral axis is z, the lateral coefficient is ignored, foils are always horizontal (lift along up) and steering turns about +z.

Per step, for a boat on an awake body:

* **Rudder.** `target = clamp(steer, ±1) · MaxRudderAngle`; the angle moves toward it by at most `RudderRate · dt`.
* **Hull drag**, when the body has water under it this step (the fluid code leaves the submerged volume `V`, in 2D the area, in `submerged[body]`, summed over the fluids and buoyancy fields) and a water sample is found at the centre of mass within the body's `maxExtent` reach: `u = water flow − v`, axes `fwd`, `up`, `right = fwd × up` (planar: z); `ref = V^(2/3)` (planar `sqrt(A)`); `F_i = ½ ρ C_i ref |u_i| u_i` for each axis, longitudinal and lateral capped at `m |u_i| / dt`; the vertical one adds `HeaveDamping · ρ · V · u_up`, the sum capped at `m |u_up| / dt`. Applied at the centre of mass. While a body is a boat the fluid's own linear drag and viscous damping are not applied to it (its buoyancy and its angular damping are).
* **Foils**: lifting surfaces in water, coefficients `Wing(AR)`, chord `−fwd`, normal `right` (or `up`), at the foil position. A foil with a non-zero `SteerGain` is turned about the up axis (z in 2D) by `rudder · gain` (a positive angle swings the trailing edge to the right, so a positive steer turns the boat right: clockwise seen from above in 3D; counter-clockwise in 2D).
* **Propellers**: thrusters as above with throttle = the boat's throttle and first gimbal command = steer (a positive gimbal about up swings the thrust toward the left, so the stern pushes left and the bow turns right).

Presets `BoatDef.Dinghy`, `Speedboat`, `Barge` (3D: +Z forward, +Y up; 2D: +X, +Y) place a keel at 0.1 L below, a steered foil at the stern 0.05 L below, and a propeller at the stern 0.06 L below, for a hull whose body origin is at the waterline (the scenes shift the hull shape so it is). Dinghy: L 3 m, 400 N, keel 0.25 m², rudder 0.08 m², `C_long` 0.1, `C_lat` 1, `C_vert` 2. Speedboat: L 6, 6 kN, 0.4 and 0.15 m², 0.06. Barge: L 20, 8 kN, 1.5 and 1 m², 0.2, 2, 4.

## Hovercraft

`HovercraftDef`: forward and up axes, fans (position, direction toward the surface, `MaxLift`, `Range`, `Damping`), thrusters, `SkirtDrag`, `AngularDamping`. Per step, for a hovercraft on an awake body, `lift` the fan power clamped to 0..1, fans in definition order:

* The fan's ray is cast from the fan point along its axis for `Range` against every shape except the craft's own (`Raycast`, sensors excluded, nearest hit) and against every water (fluids then buoyancy fields) the point is above and whose surface the ray meets within `Range`; the nearest wins.
* `cush = max(0, 1 − gap / Range)`; `closing = (v_point − v_surface) · axis` (the surface's velocity: the water's flow, the hit body's point velocity, 0 for a static shape); `F = MaxLift · lift · cush² + Damping · cush · closing`, clamped to ≥ 0; the force on the craft is `−axis · F` at the fan point. A dynamic body that was hit takes `+axis · F` at the hit point and is woken if asleep.
* At rest on a level surface a fan carrying weight `W` sits at `gap = Range · (1 − sqrt(W / (MaxLift · lift)))`.
* **Skirt** `k = mean(cush²)` over the fans: `−SkirtDrag · k · v_along` (the velocity without its component along up, capped at one step's worth) and a torque `−AngularDamping · k · ω` capped so it cannot reverse the rotation within a step.
* Thrusters: as above, throttle and `gimbal = steer` (a hovercraft's thrusters are air thrusters unless defined otherwise).

`HovercraftDef.ForMass(m)` (and `Small` = `ForMass(200)`): 4 fans (3D, 2 in 2D) of `MaxLift = 2 · m g / n`, `Range` 0.5, `Damping` 0.4 · `MaxLift`, two rear thrusters (one in 2D) of 1.5 N per kg each with a 0.5 rad gimbal, skirt drag 0.4 m, angular damping 0.3 m. Holds the craft 0.146 m (fan to surface).

## Ordering and tie-breaks

Surfaces by shape slot; thrusters, boats and hovercraft by slot, each component's parts in definition order; water and wind in fluid slot then field slot order; the nearest ray hit takes the lower distance and, on a tie, the shape cast first then the lower water index (strict `<`). Forces are added serially in this order, so float sums are the same on every platform and thread count.

## Snapshot section

Optional section id `0x4F524541` ("AERO"), registered after every other section in both worlds, written only while a surface, thruster, boat or hovercraft exists or the air density is not 1.225, so worlds that never use it keep their bytes and hashes. Layout (little-endian; `Named*` labels for the desync finder; entity kinds `thruster`, `boat`, `hovercraft`, `aero`):

* version i32 = 1; air density f32.
* **Thrusters**: handle table (`"thruster"`), then per live slot: body i32, generation i32, throttle, gimbal 1, gimbal 2, output (f32 each), the definition: position, direction, gimbal axis (3 × f32 each), max thrust, response time, max gimbal, reverse fraction, spin torque, pitch speed, immersion depth (f32 each), medium u8.
* **Boats** (`"boat"`): body, generation, throttle, steer, rudder angle, the definition (forward, up, four drag/damping coefficients, max rudder angle and rate, foil count, per foil a surface record and the steer gain, then the propeller thrusters as a count and records), then one f32 output per propeller. A surface record is kind u8, medium u8, area, the seven coefficients, chord, normal, centre (f32 each).
* **Hovercraft** (`"hovercraft"`): body, generation, lift, throttle, steer, the definition (forward, up, skirt drag, angular damping, fan count, per fan position, direction, max lift, range, damping, then thrusters as a count and records), one f32 output per thruster.
* **Surfaces**: i32 count, then per surface in ascending shape slot: slot i32, shape generation i32, the surface record.

Restore validates every field (finite, ranges, unit axes, counts ≤ 64 per craft, outputs within ±1, inputs within their limits) and, after the sections are read, that every component names a live dynamic body of the saved generation (one boat and one hovercraft per body at most) and every surface a live shape; stored axes are kept bit for bit so a restore re-serializes to the same bytes. The section is in the world hash and the snapshot size bound (it is written through the same writer). Not saved: last forces, fan gaps and forces, the derived planform area, the submerged measures.

## Host hooks

* **Replay and history**: every mutating call records through the usual mutation scope (`AddThruster`, `AddBoat`, `AddHovercraft`, `Thruster.*`, `Boat.*`, `Hovercraft.*`, `Shape.Aero`, `AirDensity`); a replay plays back with every per-step hash equal.
* **Scene v2** keeps everything through the snapshot chunk; scene v1 has no record for them.
* **Fork**: the store is copied structurally (specs cloned, arrays copied).
* **Regions**: a region takes the thrusters, boats and hovercraft of the bodies it holds, and the surfaces of the shapes it holds. Nothing here pulls another body in (the reaction on a body under a hovercraft is applied to its accumulators during the stage and is not state).
* **Desync finder**: entities `thruster`, `boat`, `hovercraft`, `aero`.
* **Origin shift**: nothing is stored in world coordinates; water and wind shift with the fluids and fields. A shifted world matches an unshifted one to float rounding (the bit-identity test of the origin shift does not cover water, as for fluids).
* **Debug geometry**: `DebugContent.Aero` (value 8192; `All` is now 16383) adds `AeroSurfaces` (surface or foil: centre, chord, normal, area, lifting flag) and `AeroForces` (kind, index, point, force) in the order shape surfaces, thrusters, boats (hull, foils, propellers), hovercraft (fans, thrusters); zero forces are not emitted; points are at the current pose.

## Failure behaviour

Definitions are checked when a component is created: NaN, infinities, zero axes, a lifting normal parallel to the chord, areas outside 0..10^6 (a foil's must be above 0), more than 64 fans, foils or thrusters, a z component in a 2D world, `SpinTorque` in 2D, a non-dynamic body, a body that already carries a boat or a hovercraft, null elements: `ArgumentOutOfRangeException`, `ArgumentNullException` or `ArgumentException` naming the member and the value; a stale or default handle: `StaleHandleException` / `InvalidHandleException`; inputs outside their range: `ArgumentOutOfRangeException`. A rejected call leaves the world unchanged.
