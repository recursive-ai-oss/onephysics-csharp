# Units and sizes

- **Meters, kilograms, seconds.** Positions cross the API as `double`; everything else is `float`. Gravity defaults to (0, -9.81); density is kg/m^2 in 2D and kg/m^3 in 3D.
- **Full sizes.** `AddBox(size)`, `CreateBox(body, size, ...)` and the box queries (`OverlapBox`, `BoxCast`) take the full width and height (and depth). A parameter that is a half extent says so in its name: `halfLength` (capsules), `halfHeight` (cylinders), `ShapeHalfSize` (ragdoll parts), `halfAngle` (cones). A radius is a radius.
- **Radians.** Every angle, limit, motor speed and angular velocity is in radians (per second); Unity's degrees need converting. Parameters that are angles in a method with other numbers say so: `angleRadians` on creators, queries and `SetTransform`. `CharacterDef`'s slope limit is in radians too (`SlopeLimitRadians`).
- **Counter-clockwise.** In 2D a positive angle rotates counter-clockwise, with +y up (gravity is negative y).
- **Definitions have zero-value defaults.** `default(BodyDef)` is a dynamic body; `default(ShapeDef)` has density 1 and friction 0.6. You never need to remember to fill a field in to get the documented default.
- **Mass.** Mass comes from shapes: `density x area` (2D) or `x volume` (3D). A shape that would take a body's mass or inertia outside 1e-18..1e30 is refused with a typed error naming the value; see [input limits](limits.md).

The full list of conventions, including handles and joint names, is in [names and conventions](api-names.md).

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
