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

What does not follow the unit, because it is authored per object in the host's own numbers: the sizes, masses, densities, stiffnesses and compliances of soft bodies, cloth, rods, ropes, vehicles, ragdolls, fields, water volumes and articulations; the sizes of fluid emitters and drains; the presets written in SI (`CharacterDef.Child` and `Giant`, the fluid presets `Oil`, `Honey`, `Sand` and the others). For the character and fluid presets `InUnit(unit)` returns the definition converted; the others are to be written in the world's unit. Quantities with their own physical length scale (a rod's 5e-5 m hair radius, a soft body's tearing threshold) are in the caller's numbers.

## Snapshots

One optional section, "UNIT" (`0x54494E55`), version 1: int32 version (1), then float64 metres per unit. Written only while the unit is not one metre, so a world at the default unit keeps its snapshot bytes and hash; registered after every other section (after the speculative distance section "SPEC" in 2D, after "WSCL" in 3D). Covered by the world hash like any other section. A region snapshot leaves it alone. Reader: version must be 1, the value must satisfy the range above, otherwise a typed `SnapshotFormatException` and the world is unchanged. A restore of a snapshot without the section puts one metre back.

The unit is read before every other section (the section stays last in the bytes), so each section reads, validates and falls back to its defaults in the snapshot's own unit: the 2D speculative distance section and the services block, which hold a value only when it differs from the default of the unit, read against that unit.

The option values themselves (gravity, thresholds, speculative distance, speed limits) are saved as they are, in world units, as before. The broadphase margin is a creation option and is not saved: a world that restores a snapshot is created with the same options, `LengthUnit` included, as for any restore.

## Scenes, replays

- **Authoring scene (version 1):** an optional record `unit <metres per unit>` after `dim`, written only when the unit is not one metre. Every length in the file is in that unit: positions, sizes, anchors, gravity, speeds; densities are per unit of volume (area in 2D) of the unit, masses in kg. Scenes without the record read and write as before.
- **Exact-state scene (version 2):** carries the snapshot, so the unit with it; its live-options payload carries the unit too (below).
- **Replay and live options:** the live-options payload has one more variant for a world that is not in metres. 2D: 144 bytes = the 136-byte payload (with the soft-body options) followed by float64 metres per unit at offset 136. 3D: 152 bytes = the 144-byte payload followed by float64 metres per unit at offset 144. Worlds in metres write the existing sizes, so existing recordings keep their bytes. The unit is read first: the values after it are in it.

## Checks

A world in a unit that is a power of two (1/1024 m) steps a scene scaled by 1024 exactly as the metre world steps the original, up to the 2^-32 grid: a metre-tuned constant the derivation missed shows as a difference far above the grid (`LengthUnitTests`). The standard stability scenes, rescaled to millimetres and to pixels, pass their thresholds in metres (`bar run` with `BAR_UNITS_PER_METRE`, see `bench.md`).
