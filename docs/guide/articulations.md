# Articulations

An articulation is a tree of rigid links (a robot arm, a walker, a long chain) simulated in joint coordinates, so its joints cannot come apart, however heavy the end or however long the chain. Use it where a ragdoll's joints would stretch: manipulators, legged machines, cranes, whips. Everything is in both dimensions (`PhysicsWorld2D.AddArticulation` takes the 2D definition: planar links, revolute and prismatic joints).

<!-- example: Guide/Articulations.cs -->
```csharp
var world = new PhysicsWorld3D();
world.AddBox(new Vector3F(20f, 1f, 20f), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);

// A seven-joint arm on the floor, steered by a task-space target on its tool point.
var arm = world.AddArticulation(ArticulationDef.RobotArm(0.9f, 7, 8f), new Vector3D(0, 0, 0));
var tool = new Vector3F(0, 0.1286f, 0);                                   // a point on the last link
var goal = new Vector3D(0.3, 0.5, 0.3);
arm.SetTaskTarget(arm.LinkCount - 1, tool, goal);

// A quadruped on a floating base stands under its leg drives.
var dog = world.AddArticulation(ArticulationDef.Quadruped(), new Vector3D(3, 0.62, 0));

for (int i = 0; i < 300; i++) { world.Step(1f / 60f); }

var tip = arm.GetPointPosition(arm.LinkCount - 1, tool);
```

A definition of your own is a list of links, parents first. Each link has its joint to the parent (kind, axis, where it sits in the parent's frame), its mass properties, limits, a drive and its collision shapes:

<!-- example: Guide/Articulations.cs -->
```csharp
var def = new ArticulationDef { FixedBase = true, Position = new Vector3D(-3, 4, 0) };
def.AddLink(new ArticulationLinkDef { Mass = 1f, InertiaTensor = Matrix3x3F.Identity });
def.AddLink(new ArticulationLinkDef { Parent = 0, Joint = ArticulationJointKind.Revolute, Axis = new Vector3F(0, 0, 1), Mass = 2f, CenterOfMass = new Vector3F(0, -0.5f, 0), InertiaTensor = Matrix3x3F.Identity * 0.1f, InitialPosition = 1f });
var swing = world.AddArticulation(def);
```

The members follow URDF and MJCF one to one (link inertial, joint axis and origin, limits, effort and velocity limits, damping and friction, armature), which is how the importers fill them.

Joint coordinates and forces, for control code:

<!-- example: Guide/Articulations.cs -->
```csharp
var holding = new double[arm.VelocityCount];
arm.ComputeGravityCompensation(holding);                  // joint torques that hold the present pose
var mass = new double[arm.VelocityCount * arm.VelocityCount];
arm.ComputeMassMatrix(mass);                              // M(q), row-major
```

## What to know

- **Links are kinematic bodies.** Each link is an ordinary body in the world (`arm.GetLink(i)`): its shapes collide, raise events, show up in queries and in the debug view. Do not move a link body yourself; set joint positions, targets or forces instead.
- **Drives are PD controllers**: `Drive.Stiffness`, `Damping`, `MaxForce` (the effort limit), `TargetPosition`. They are integrated implicitly, so any stiffness is stable. `SetJointTarget(link, position)` moves the target; `GravityCompensation = true` adds the gravity-holding torque.
- **Task targets** (`SetTaskTarget`) steer a point of a link to a place by moving the drive targets of the joints with a position drive. They respect contacts, so an arm that meets a box pushes it. The tool must come from a direction it can reach: it is a local solver, so route it above an obstacle first and then down.
- **Substeps are automatic.** The articulation steps itself in 4 or more substeps, more for stiff drives or long chains with heavy ends (up to 64), so it costs more than the same number of ordinary joints. Set `Substeps` to pin it.
- **Contacts push links out and bodies away.** A box pushed by an arm is accelerated by the arm's joint-space effective mass, so a weak drive cannot push a heavy box. A body squeezed between a link and static ground is held by the ground.
- **Not modelled:** closed loops, mimic joints, sleeping (articulations stay awake).
- Articulations are part of the snapshot, hash, `Fork`, regions (a region that holds one link holds the whole articulation), replays, `ShiftOrigin` and `DebugContent.Articulations`. The normative description is `docs/spec/articulations.md`.
