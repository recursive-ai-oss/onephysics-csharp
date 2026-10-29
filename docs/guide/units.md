# Units and sizes

- **Meters, kilograms, seconds.** Positions cross the API as `double`; everything else is `float`. Gravity defaults to (0, -9.81); density is kg/m^2 in 2D and kg/m^3 in 3D.
- **Full sizes.** `AddBox(size)`, `CreateBox(body, size, ...)` and the box queries (`OverlapBox`, `BoxCast`) take the full width and height (and depth). A parameter that is a half extent says so in its name: `halfLength` (capsules), `halfHeight` (cylinders), `ShapeHalfSize` (ragdoll parts), `halfAngle` (cones). A radius is a radius.
- **Radians.** Every angle, limit, motor speed and angular velocity is in radians (per second); Unity's degrees need converting. Parameters that are angles in a method with other numbers say so: `angleRadians` on creators, queries and `SetTransform`. `CharacterDef`'s slope limit is in radians too (`SlopeLimitRadians`).
- **Counter-clockwise.** In 2D a positive angle rotates counter-clockwise, with +y up (gravity is negative y).
- **Definitions have zero-value defaults.** `default(BodyDef)` is a dynamic body; `default(ShapeDef)` has density 1 and friction 0.6. You never need to remember to fill a field in to get the documented default.
- **Mass.** Mass comes from shapes: `density x area` (2D) or `x volume` (3D). A shape that would take a body's mass or inertia outside 1e-18..1e30 is refused with a typed error naming the value; see [input limits](limits.md).

The full list of conventions, including handles and joint names, is in [names and conventions](api-names.md).
