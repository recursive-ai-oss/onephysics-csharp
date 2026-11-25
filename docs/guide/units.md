# Units and sizes

- **Meters, kilograms, seconds.** Positions cross the API as `double`; everything else is `float`. Gravity defaults to (0, -9.81); density is kg/m^2 in 2D and kg/m^3 in 3D. A world can use another length unit (below).
- **Full sizes.** `AddBox(size)`, `CreateBox(body, size, ...)` and the box queries (`OverlapBox`, `BoxCast`) take the full width and height (and depth). A parameter that is a half extent says so in its name: `halfLength` (capsules), `halfHeight` (cylinders), `ShapeHalfSize` (ragdoll parts), `halfAngle` (cones). A radius is a radius.
- **Radians.** Every angle, limit, motor speed and angular velocity is in radians (per second); Unity's degrees need converting. Parameters that are angles in a method with other numbers say so: `angleRadians` on creators, queries and `SetTransform`. `CharacterDef`'s slope limit is in radians too (`SlopeLimitRadians`).
- **Counter-clockwise.** In 2D a positive angle rotates counter-clockwise, with +y up (gravity is negative y).
- **Definitions have zero-value defaults.** `default(BodyDef)` is a dynamic body; `default(ShapeDef)` has density 1 and friction 0.6. You never need to remember to fill a field in to get the documented default.
- **Mass.** Mass comes from shapes: `density x area` (2D) or `x volume` (3D). A shape that would take a body's mass or inertia outside 1e-18..1e30 is refused with a typed error naming the value; see [input limits](limits.md).

The full list of conventions, including handles and joint names, is in [names and conventions](api-names.md).

## Other length units

A model authored in millimetres, or a 2D game in pixels, sets the length unit and keeps its own numbers:

<!-- example: Guide/Units.cs -->
```csharp
var world = new PhysicsWorld3D(new WorldOptions3D { LengthUnit = LengthUnit.Millimetres });
var pixels = new PhysicsWorld2D(new WorldOptions2D { LengthUnit = LengthUnit.Pixels(32) });   // 32 pixels to the metre
```

`LengthUnit` says how many metres one world unit is (`Metres`, `Centimetres`, `Millimetres`, `Pixels(n)`, or `new LengthUnit(metresPerUnit)`; 1e-6 to 1e6). Every default that is tuned in metres follows it: gravity, the sleep threshold, the speculative distance, the push-out and speed limits, the broadphase margin, the contact and continuous-collision tolerances, a character's default size, the default density (1 kg per cubic metre, so a box of the same real size weighs the same), the default 3D rolling resistance, and a particle fluid's default spacing, density and speed limit. A world in millimetres is 1000 times larger in numbers and behaves the same: the speculative distance is 20 units, a character is 300 units in radius, gravity is 9810.

Only defaults follow the unit. A value you set is in world units and stays so: `SpeculativeDistance = 5` is 5 world units in any unit. Masses stay in kilograms, times in seconds, angles in radians; densities and forces you give are per world unit, so a density of 1000 kg/m^3 is `1000 / 1e9` in a millimetre world (leave it unset and the default does that). Positions keep the 2^-32 grid and the ±2^21 range in world units, so a millimetre world reaches 2 kilometres, a pixel world at 32 to the metre 65 kilometres.

Definitions have a matching pair. `WithDefaultsFor(unit)` is what the world applies when you add a definition: a member you never set follows the unit. `InUnit(unit)` converts a definition you wrote in metres and SI, such as a preset (`VehicleDef.Car.InUnit(LengthUnit.Millimetres)`, `RagdollDef.Humanoid.InUnit(unit)`, `RodMaterial.Rope.InUnit(unit)`, `ClothMaterial.Cotton.InUnit(unit)`, `SoftBodyDef.Rubber(mesh, at).InUnit(unit)`, `CharacterDef.Child.InUnit(unit)`, the fluid presets other than `Water`), and the tier-1 creators (`AddCar`, `AddTank`, `AddMotorcycle`, `AddRagdoll` with no definition, `AddSoftBox`, `AddSoftBall`, `AddCloth`, `AddRope`, `AddCable`, `AddHair`) do it for you. Settle, the stability margin, the insertion check, `Diagnose` and the static solve in `.Tools` take their tolerances from the unit as well. Rail vehicles, bicycles, articulations and destruction are still authored in metres (see the spec for the list).

## Fixed-point coordinates and origin offsets

Every Q31.32 value (raw signed integer divided by 2^32) with |value| ≤ 2^21
round-trips losslessly through double, including ±2097152. At those endpoints the
raw integer is ±2^53; the adjacent values toward zero also fit exactly. Outside
that interval not every Q31.32 value is representable. OneMath 1.0's `FixedBridge`
uses a deliberately **exclusive** magnitude check; its supported interval is
|value| < 2^21. Do not use it to convert the world's accepted endpoints.
Creation and teleport accept the closed interval and reject non-finite or larger
components with `ArgumentOutOfRangeException`. Integration may advance farther:
the ±2^22 body/shape bound supplies headroom, and `Step` reports violations with
`BodyOutOfBoundsException` before changing the world.

The vector `ShiftOrigin` overload allocates nothing when mutation recording is
disabled. Offsets must lie on the **2^-31** grid (even Q31.32 raw integers), not
merely the finer 2^-32 position grid. Odd raw integers raise `ArgumentException`;
non-finite components or magnitude above 2^22 raise `ArgumentOutOfRangeException`.
Validation precedes mutation. Apply the same offset between the same steps on all
peers; `Origin + body.Position` remains the host-space position.
