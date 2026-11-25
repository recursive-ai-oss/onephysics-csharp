# Length units

Normative, language-neutral. A world may be authored in any length unit: millimetres for a small model, centimetres, or pixels for a 2D game. `LengthUnit` says how many metres one world unit is. Everything the engine tunes in metres follows it, so a scene scaled by a factor `k` in a world whose unit is `1/k` metre behaves as the metre scene does. At the default unit (one metre) nothing changes: every default, snapshot byte and world hash is what it was.

## The value

`LengthUnit` holds `MetresPerUnit`, a double. The default (and the zero value of the struct) is 1. Accepted values are finite and within 1e-6 to 1e6; NaN, infinities, zero, negatives, subnormals and anything outside the range are refused (`ArgumentOutOfRangeException` from the constructor, a typed snapshot, scene or replay error from a file). Presets: `Metres`, `Centimetres` (0.01), `Millimetres` (0.001), `Pixels(pixelsPerMetre)` (1 / pixelsPerMetre).

The engine multiplies its metre-tuned constants by the float `Scale = (float)(1.0 / MetresPerUnit)` (1.0 exactly for the default unit; 1000, 100, 32 and 100 exactly for the presets above). A product of a float constant and the scale is one IEEE float multiplication, so every platform computes the same bits, and at scale 1 it is the constant itself.

The unit is a property of the world's options (`WorldOptions2D.LengthUnit`, `WorldOptions3D.LengthUnit`; the world's `Options.LengthUnit` is live). It does not change positions: the grid stays 2^-32 world units and the range ±2^21 world units, whatever the unit. It does not scale masses, densities, forces, times or angles; those keep the meaning they have (kg, kg per world-unit volume, kg world units per second squared, seconds, radians), except the defaults named below, which are physical values.

## What follows the unit

Only defaults derive. A value the host sets explicitly is in world units and keeps that meaning.

Options (set to the metre default times the scale while still at their default; setting `LengthUnit` moves the ones still at their default and leaves the ones you set):

| option | metre default |
|---|---|
| `Gravity` | (0, -9.81[, 0]) m/s² |
| `SleepThreshold` | 0.05 m/s |
| `SpeculativeDistance` | 0.02 m |
| `PushOutVelocity` (`MaxDepenetrationSpeed`) | 3 m/s |
| `RestitutionThreshold` | 1 m/s |
| `MaxLinearSpeed` | 400 m/s |
| `HitSpeedThreshold` | 1 m/s |
| `BroadphaseMargin` | 0.1 m (2D), 0.03 m (3D); fixed at creation |

The ranges that validate them are in metres too, never narrower than at one metre per unit (the 2D speculative distance is 0 to 1 metre and the broadphase margin 0 to 100 metres, or 1 and 100 world units in a unit larger than a metre).

Engine constants, in both engines unless said otherwise (the constant, in metres, is multiplied by the scale):

- **Contact search and reuse.** The tight-box margin added to the speculative distance (0.005); the "proved apart" slack (0.001); manifold reuse tolerance (0.01 in 2D, 0.0035 in 3D) and depth (0.05 in 2D, 0.02 in 3D); the smallest round contact radius in 2D (0.02); the farthest a pair is searched for fast bodies (0.25, 3D); the reference-face hysteresis of the 2D narrowphase (0.1 of the linear slop, 0.0005); the seam distance of internal-edge filtering on meshes and height fields (0.005, 3D); the cap on the speed change one contact point's impulse may cause (20000 m/s, 3D).
- **Continuous collision.** The target gap and tolerance of the 2D time of impact (the linear slop, 0.005, and a quarter of it); the 2D bullet search padding (1e-4); the 3D sweep padding (1e-4) and stop-short distance (0.005).
- **Queries.** The 2D query padding (1e-4).
- **Characters.** The mover's tolerances (1e-6, 1e-5, 1e-4, 1e-3 and 2e-3) in both dimensions, and the defaults of `CharacterDef`: radius 0.3, height 1.8, step offset 0.3, skin 0.01. A definition derives its unset sizes when the character is created; `CharacterDef.WithDefaultsFor(unit)` shows the result. The ranges that validate the definition (radius to 1000, height to 10000, skin from 0.0001) are in metres too.
- **Shapes.** The default density, 1 kg per cubic metre (per square metre in 2D), so a box of the same real size has the same mass in any unit: the default is `1 / Scale^3` (`1 / Scale^2`) kg per cubic (square) world unit. The default 3D rolling resistance, 0.003 m, which is a length (a torque limit per unit of normal impulse). A density or rolling resistance equal to the default value is the default: it is stored as unset, so a density of 1 means "the default" whatever the unit.
- **Particle fluids.** The defaults of `ParticleFluidDef`: spacing 0.05 m, rest density 1000 kg per cubic (square) metre, speed limit 50 m/s, and the ranges that validate them.

### Everything else that follows the unit

The same rule everywhere: a member left at its metre default follows the unit (it is stored converted, so a value that equals the *metre* default is the default whatever the unit); a member the host sets is in world units and is not touched. Definitions have `WithDefaultsFor(unit)` (the world applies it when the definition is added) and, for those written in metres, `InUnit(unit)`, which converts a whole definition written in metres, newtons and kilograms (a preset, or a skeleton laid out in metres). The conversion is by the dimension of the member, as the power of length in its SI unit with mass and time left alone: a length or a speed is multiplied by `Scale` (dimension 1), an area, a torque, a moment of inertia, an angular stiffness (N·m/rad) or damping by `Scale^2`, a density per volume divided by `Scale^3` (per area by `Scale^2`), a pressure or modulus divided by `Scale`, a compliance per square length divided by `Scale^2`. Masses (kg), times, frequencies, ratios, angles and dimensionless coefficients do not change. A definition that carries metre members records the unit it was written in, and its ranges are checked on the metre equivalent, so an `InUnit` conversion never changes which values are valid.

| area | member (metre default, dimension) |
|---|---|
| Soft bodies (3D, 2D) | `BendCompliance` 1e-3 m^-2 (-2, per square length; 2D per length); `VolumeCompliance` (4, per fourth power) and `TearForce` (1) only through `InUnit`; the automatic particle mass (1000 kg/m^3, 0.3 kg/m^2 for a sheet; 2D 1 kg/m^2) and radius (a quarter of the mean edge, capped at 100 m); the absolute pads of the contact search (+0.02 and +0.05 m); the tier-1 presets (`AddSoftBox`, `AddSoftBall`, `AddCloth`) |
| Cloth (3D, 2D strips) | `ClothMaterial` area density 0.15 kg/m^2 (-2), bend 2e-3 (-2), thickness and tether lengths (1); the presets Cotton, Denim, Silk, Leather, RubberSheet via `InUnit` |
| Rods, ropes, cables, hair | `RodMaterial` Young's and shear modulus (-1), radius (1), density (-3) (`InUnit`; the presets Rope, Cable, Hair; `AddRope`, `AddCable`, `AddHair` convert); the contact search margin 4 mm, skin 1e-6 m, one-way 0.01 m/s, segment length range; the rolling-resistance limit (a length coefficient, divided by the unit once more because the limit also multiplies by the radius) |
| Pulleys, belts, chains, winches | wheel radius (1), inertia (2), deadbands and slip tolerances; cable defaults |
| Vehicles (3D wheeled, tracked, motorcycle; 2D wheeled, tracked, motorcycle) | `WheelDef` radius 0.35 m, width 0.25 m, suspension 0.1..0.5 m, inertia 1 kg m^2 (2), brake torque 3000 (2); engine torque 400 (2), inertia 0.5 (2); tyre low speed 1 m/s; tracked sprocket radius 0.3, belt speed 12 m/s, drive torque 20000 (2), brake torque 120000 (2), chassis size; motorcycle lean torque (2), steer fade speeds, chassis size; air control. A tracked or motorcycle definition's wheeled form converts its engine as well |
| Ragdolls and active ragdolls | part half size 0.1 x 0.05 x 0.05 m, drive stiffness 500 and damping 40 N m (2), max torque 100 N m (2); `Humanoid` and the 2D preset via `InUnit`; the active layer's hit radius 0.45 m, step trigger -0.03 m, step height 0.07 and 0.0867 m, step width 0.2 m, top speed 2.5 m/s, and every tuned length and speed of the biped gait (the gait takes the unit as a factor); the ankle torque source (600 and 6 N m, 2); the inertia floor 1e-4 kg m^2 (2) |
| Aero, thrusters, boats, hovercraft | air density (-3, a world follows the unit while the value is unset), `AeroSurface` area and reference lengths, thrust, boat hull volume terms, hover range and lift; the actuator limits |
| Fields | `FieldDefinition` radius and softening (1), density |
| Water | buoyancy volumes' water density 1000 kg/m^3 (-3 in 3D, -2 in 2D); `ShallowWaterDef` cell size 0.5 m (1), dry depth, density; swim and wade depths and the swim law's density (900 kg/m^3) |
| Characters | the ability constants (ledge reach, climb, mantle, wall jump, swim immersion lengths) |
| Settle, stability margin, insertion check | `SettleOptions` speed 0.01 m/s, acceleration 0.25 m/s^2 and support tolerance 0.005 m; `StabilityOptions` contact tolerance 0.005 m; `InsertionOptions` probe distance 0.05 m and seat tolerance 0.001 m; the sweep constants of the body sweeps |
| Diagnose | the metre constants of the diagnoser (size, speed, gravity and gap thresholds) are judged against the world's unit; the size and speed findings name the unit and the metres it implies; values in findings are in the world's units unless the field says metres |
| `.Tools` static solve | `StaticSolveOptions.LengthUnit` (the solver reads metre loads and lengths in world units; `MetresPerUnit` still works) |

### What is not converted, and why

These areas are in metres only. Their constants are authored in the host's numbers and are tied to a real-world scale (rail gauge and curvature, a bicycle's geometry, an articulated robot's links, the size of fracture cells). A world in another unit still runs them, but their tolerances, which are absolute lengths, were not derived: use a world in metres (or keep their numbers well above the engine's tolerances) until a later release converts them. This is a decision of this release, not a limit of the design.

- **Rail vehicles (3D, 2D)** and **bicycles**: `TrackDef`/`RailDef` geometry, the wheel-rail contact constants (the Kalker creep coefficients and the flange angle are dimensionless; the contact patch and clearance tolerances are lengths), `BicycleDef`. A rail scene is authored around one real gauge; the work is converting about forty tolerances and presets.
- **Articulations (3D, 2D):** the joint-space solver works in double with absolute tolerances (contact slop, limit and drive epsilons, the automatic substep rule's reference frequency); `ArticulationDef` links are the host's own numbers.
- **Destruction:** fracture cell merging and the debris budget use absolute minimum fragment sizes; the `FractureDef` sizes are the host's numbers.
- **Interchange importers and `.Tools` meshing** (`Tetrahedralizer`, `ConvexDecomposer`, `AutoRagdoller`): their output is in the unit of the input file or mesh; they have no world to read a unit from, and each takes its own size parameters.
- **Aerodynamic coefficient tables** (`AeroCoefficients`): dimensionless; nothing to convert.

Quantities with their own physical length scale (a rod's 5e-5 m hair radius, a soft body's tearing threshold) are in the caller's numbers unless the definition is converted with `InUnit`.

## Snapshots

One optional section, "UNIT" (`0x54494E55`), version 1: int32 version (1), then float64 metres per unit. Written only while the unit is not one metre, so a world at the default unit keeps its snapshot bytes and hash; registered after every other section (after the speculative distance section "SPEC" in 2D, after "WSCL" in 3D). Covered by the world hash like any other section. A region snapshot leaves it alone. Reader: version must be 1, the value must satisfy the range above, otherwise a typed `SnapshotFormatException` and the world is unchanged. A restore of a snapshot without the section puts one metre back.

The unit is read before every other section (the section stays last in the bytes), so each section reads, validates and falls back to its defaults in the snapshot's own unit: the 2D speculative distance section and the services block, which hold a value only when it differs from the default of the unit, read against that unit.

The option values themselves (gravity, thresholds, speculative distance, speed limits) are saved as they are, in world units, as before. The broadphase margin is a creation option and is not saved: a world that restores a snapshot is created with the same options, `LengthUnit` included, as for any restore.

## Scenes, replays

- **Authoring scene (version 1):** an optional record `unit <metres per unit>` after `dim`, written only when the unit is not one metre. Every length in the file is in that unit: positions, sizes, anchors, gravity, speeds; densities are per unit of volume (area in 2D) of the unit, masses in kg. Scenes without the record read and write as before.
- **Exact-state scene (version 2):** carries the snapshot, so the unit with it; its live-options payload carries the unit too (below).
- **Replay and live options:** the live-options payload has one more variant for a world that is not in metres. 2D: 144 bytes = the 136-byte payload (with the soft-body options) followed by float64 metres per unit at offset 136. 3D: 152 bytes = the 144-byte payload followed by float64 metres per unit at offset 144. Worlds in metres write the existing sizes, so existing recordings keep their bytes. The unit is read first: the values after it are in it.

## Checks

A world in a unit that is a power of two (1/1024 m) steps a scene scaled by 1024 exactly as the metre world steps the original (for a scene that is not chaotic; a falling ragdoll is judged against what a 1e-7 m nudge of the metre world does), up to the 2^-32 grid: a metre-tuned constant the derivation missed shows as a difference far above the grid (`LengthUnitTests`). The standard stability scenes, rescaled to millimetres and to pixels, pass their thresholds in metres (`bar run` with `BAR_UNITS_PER_METRE`, see `bench.md`).
