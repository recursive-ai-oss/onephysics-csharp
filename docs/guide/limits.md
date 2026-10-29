# Input limits

Inputs outside these limits are refused with a typed error that names the value and the limit; nothing is changed when an operation refuses.

- **Mass properties.** A dynamic body's total mass, and in 2D its inertia (when not zero) or in 3D each diagonal entry of its inertia tensor, must lie within 1e-18..1e30, and the 3D tensor must be invertible. A shape that would take a body outside that range is refused with `ArgumentOutOfRangeException` and the body keeps its previous mass properties. Removing a shape never throws; a body left with unusable properties falls back to unit mass and inertia.
- **Convex shape thickness.** A box, cylinder, capsule or tapered capsule must be at least 2^-20 as thick as it is long (`ConvexShape3D.MaxAspectRatio`): the cast and distance routines work in float32 with tolerances relative to the largest coordinate they see.
- **Material coefficients.** Friction and rolling resistance are within 0..1e15 (`MaterialLimits.MaxCoefficient`), so the geometric mean of two materials cannot overflow.
- **Fluids.** `SurfaceOffset` divided by the length of `SurfaceNormal` must stay within the coordinate range.
- **Diverged bodies.** `Step` refuses (`BodyOutOfBoundsException`, world unchanged) when a moving dynamic body has a non-finite position, velocity or rotation, as well as when its bounds leave the supported range.
- **Origin shifts** check every shape's shifted bounds first and refuse with `BodyOutOfBoundsException` before changing anything.
- **Rays.** A ray whose whole length is below 1e-6 is tested against the shape's padded bounds first, and a hit whose ray point lies outside those bounds is a miss, so a ray cast and a compound's bounds-pruned cast agree.
