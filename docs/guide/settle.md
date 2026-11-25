# Bringing a world to rest

A world built by a script, a level file or a model is rarely at equilibrium: boxes hang a little above their supports, ropes are laid out straight, cloth is flat in the air. `Settle` runs it to rest and says how it went; three read-only helpers then say what a host usually wants to know next. The same members exist on `PhysicsWorld2D` (with `Vector2D`, `SettleOptions2D` and so on). The bit-level rules are in [the spec](../spec/settle.md).

<!-- example: Guide/Settle.cs -->
```csharp
var world = new PhysicsWorld3D();
world.AddBox(new Vector3F(40, 1, 40), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
var cabinet = world.AddBox(new Vector3F(1, 2, 1), at: new Vector3D(0, 1.2, 0));        // placed 0.2 m too high: it will drop
var rope = world.AddRope(new Vector3D(-4, 4, 0), new Vector3D(-2, 4, 0), 16, pinStart: true, pinEnd: true);

SettleReport report = world.Settle(new SettleOptions3D
{
    ExtraLinearDamping = 1f,                                 // removes the bounce, shortens the run
    Bounds = (new Vector3D(-50, -5, -50), new Vector3D(50, 50, 50)),
    ReportBodies = true,                                     // per-body residuals and the lists below
});
if (!report.IsClean) throw new InvalidOperationException(report.ToString());   // not at rest, loose bodies, or a body that left the level

StabilityMargin3D margin = cabinet.GetStabilityMargin();
double tipDegrees = margin.MinTipAngle * 180 / Math.PI;     // how far the floor can tilt before the cabinet falls over
float push = margin.MinTippingForce;                         // a horizontal push at its centre of mass, newtons

RestShape3D hanging = world.GetRestShape(rope);              // positions, edges, frames and the sag, ready to bake into a model

var peg = world.AddBox(new Vector3F(0.2f, 0.4f, 0.2f), at: new Vector3D(6, 1.5, 0));
InsertionReport3D insertion = peg.CheckInsertion(new Vector3F(0, -1, 0), 1.3f);   // does it go 1.3 m down, and how much room does it have?
```

## What Settle does

It steps the world at `Dt` (1/60) with sleeping off and callbacks suppressed until every awake dynamic body, soft-body particle and rod particle has been slow enough (`SpeedTolerance`, `AngularSpeedTolerance`), and changing velocity slowly enough (`AccelerationTolerance`), for `StableSteps` steps in a row, or until `MaxSteps` is used up. The step counter of the world advances by `report.StepsUsed`; a recorder sees one record, and playing the recording back repeats the settle and checks the report and the hash.

The report says:

- `Converged`, `StepsUsed`;
- the worst speed, angular speed and unbalanced force (mass × the change of velocity per second) of the last step, with the body that has it;
- **unsupported** bodies: not connected to any static or kinematic body through touching contacts and joints (still falling, or a stack floating in zero gravity);
- **out of bounds** bodies: centre of mass outside `Bounds`. They are listed and ignored for convergence, so one body that fell off the level does not hold the rest hostage.

`IsClean` is all three at once. The lists and per-body residuals are filled only with `ReportBodies = true`; without it the report is a small object and nothing proportional to the body count survives the call. Settle itself is an offline operation: it allocates scratch for the call.

Extra damping (1/s) is applied after each step as `v / (1 + c dt)`; use it to remove bounce, and leave it at 0 when you want the undamped rest state.

Tolerances are in metres and seconds. A sliding block never converges (its speed keeps rising); a block on a slope below its friction angle does.

## Stability margin

`body.GetStabilityMargin()`, or `world.GetStabilityMargin(bodies)` for an assembly treated as one rigid piece, reads the contacts as the last step left them. It gives the support polygon (3D) or interval (2D), the distance from the centre of mass to its edge, and for each of 16 horizontal directions (2 in 2D) the angle the floor can tilt toward it before the assembly tips and the horizontal push at the centre of mass that tips it. A box 1 m wide with its centre of mass 1 m up tips at atan(0.5) = 26.6° under a push of half its weight. `SlidingForce` is what friction can hold; when it is smaller than the tipping push, `SlidesBeforeTipping` is true.

## Rest shapes

`world.GetRestShape(softBodyOrRod)` copies the particle positions, the triangles (soft bodies and cloth) or segment frames (rods), the bounds and the sag out as plain arrays. A string pinned at both ends of equal height reads back its catenary; `MaxSpeed` tells you whether it had really stopped.

## Insertion check

`CheckInsertion(axis, distance)` pushes a body (or `world.CheckInsertion(bodies, ...)` a set that moves together) along an axis without moving anything. It reports where the part first touches (`FirstContactDistance`, point, normal, `Blocker`), whether it `Seats` (nothing blocks it within a millimetre of the full distance and the room at the sides never drops below `RequiredClearance`), and a `ClearanceProfile`: the room at the sides at evenly spaced points along the path. Give the depth at which the part is fully home as the distance. The profile probes up to `ProbeDistance` (5 cm) sideways; a value of 5 cm means "at least that".

See also [soft bodies](soft-bodies.md) and [rollback](rollback.md).
