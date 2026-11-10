# Particle fluids

Water and other liquids as particles, in the 2D and the 3D world (a preview, like the rest of `Physics3D`: `#pragma warning disable OPHYS0002`; the 2D members are the same with `Vector2D` and `Vector2F`). The fluid is position-based (Macklin and Müller 2013): every particle is held to its rest density by a constraint, so a pool stays a pool, a stream falls and splashes, and a crate floats at the waterline of its density ratio. The particles collide with every shape, push dynamic bodies and are pushed by them, and meet soft bodies, cloth and rods the same way. This is not the analytic buoyancy volume of `AddFluid`: that floats bodies on a water plane without simulating the water; the two do not interact.

## Happy path

<!-- example: Guide/Fluids.cs -->
```csharp
ParticleFluid water = world.AddParticleWater(spacing: 0.06f);
world.FillFluidBox(water, new Vector3D(-0.5, 0, -0.5), new Vector3D(0.5, 0.4, 0.5));
var crate = world.AddBox(new Vector3F(0.25f, 0.25f, 0.25f), at: new Vector3D(0, 0.7, 0), density: 400f);
float fraction = world.GetFluidState(crate, water).SubmergedFraction;
foreach (var e in world.FluidEvents) Console.WriteLine($"{e.Kind} body {e.Body.Index} fluid {e.Fluid}");
```

- `spacing` is the distance between particles at rest, in metres, and sets the resolution: the cost is the particle count, so halve it only when you can pay eight times (3D) or four times (2D) as much. One particle weighs `density × spacing³` (`spacing²` per metre of depth in 2D).
- `FillFluidBox`, `FillFluidSphere` and `AddFluidParticle` add particles; a particle that would pass `FluidSettings.MaxParticles` (262144 by default) throws `FluidBudgetException` and nothing is added.
- `GetFluidState(body, fluid)` gives the fraction of the body under the fluid (an estimate at the resolution of a sample lattice), the surface the particles touch and the force they put on it. `FluidEvents` lists the bodies that entered or left a fluid in the last step, ordered by body, then fluid.
- Particles are not bodies: read them with `GetFluidParticles`, `OverlapFluidSphere`, `OverlapFluidBox` and `RaycastFluid` (spans you own; no allocation). Their order changes every step; their `Id` does not. Surface meshing is yours (or the viewer's): the particles are the output.

## Middle tier

<!-- example: Guide/Fluids.cs -->
```csharp
var honey = ParticleFluidDef.Honey(spacing: 0.06f);
honey.Friction = 0.8f;
world.AddParticleFluid(honey);   // a second fluid with the same spacing
water.AddEmitter(new FluidEmitterDef { Position = new Vector3D(0, 1.8, 0), Direction = new Vector3F(0, -1, 0), Size = new Vector3F(0.1f, 0.1f, 0.1f), Speed = 2f });
water.AddDrain(new FluidDrainDef { Position = new Vector3D(0, 0.05, 0.55), Size = new Vector3F(0.5f, 0.05f, 0.05f) });
world.FluidSettings = new FluidSettings { Substeps = 2, Iterations = 4, MaxParticles = 50000, SleepSpeed = 0.02f, TimeToSleep = 1f, WakeBodySpeed = 0.05f };
```

- `ParticleFluidDef` holds the material: rest density, viscosity (XSPH), vorticity confinement, the artificial pressure that keeps the surface from clumping (it also gives a little surface tension), the softness of the density constraint, friction, speed limit, gravity scale and the layer, category, mask and group that decide which shapes it meets. Presets `Water`, `Oil` and `Honey`. Every fluid of a world shares one spacing (the first sets it).
- An **emitter** is a nozzle: a disc or box facing a direction with a speed. It emits a lattice layer each time the flow has moved one spacing, so the stream has the fluid's density. A **drain** removes every particle inside its box or sphere at the end of each step.
- `FluidSettings`: `Substeps` and `Iterations` trade cost for incompressibility (2 and 4 by default; the dam break matches Martin and Moyce's measurements within 8% in 2D at spacing 0.02). A fluid whose fastest particle stays below `SleepSpeed` for `TimeToSleep` sleeps and costs nothing until a moving body, a soft body, a rod or an emitter reaches it.

## Queries

<!-- example: Guide/Fluids.cs -->
```csharp
var near = new FluidParticleInfo[64];
int count = world.OverlapFluidSphere(new Vector3D(0.3, 0.2, 0), 0.1f, near, out int total);
bool hit = world.RaycastFluid(new Vector3D(0.3, 2, 0), new Vector3F(0, -1, 0), out FluidRayHit first);
```

Particles are spheres of half the spacing for rays; `FluidDensityAt` reads the SPH density at a point (about 1000 kg/m³ inside water, half that on a flat surface).

## What it does with the rest of the world

- **Rigid bodies** are coupled in both directions with momentum conserved: a dense body sinks, a light one floats at the waterline of its density ratio (within about one spacing), a thrown one makes a splash and loses speed. A sleeping body is pushed awake only by a push that changes its velocity by more than `WakeBodySpeed`.
- **Soft bodies, cloth and rods** are surfaces for the fluid. For a cloth to hold water, the cloth's collision radius plus half the fluid spacing must exceed half the diagonal of a cloth cell (a 0.05 m cloth with radius 0.02 m holds 0.05 m water).
- It is deterministic like everything else: the same bits on every platform and thread count, in the snapshot, the world hash, forks, regions, the replay and `ShiftOrigin`. Worlds that never add a fluid keep their bytes.
- Not here yet: shallow water and swimming characters.

Details, the kernels and the layout of the saved state are in `docs/spec/fluids.md`.

## Sand, snow, clay, mud and honey

<!-- example: Guide/Fluids.cs -->
```csharp
var sand = world.AddParticleFluid(ParticleFluidDef.Sand(spacing: 0.06f));   // ElasticModulus above 0 makes a solid
sand.AddEmitter(new FluidEmitterDef { Position = new Vector3D(0, 1, 0), Direction = new Vector3F(0, -1, 0), Kind = FluidVolumeKind.Sphere, Size = new Vector3F(0.08f, 0.08f, 0.08f), Speed = 0.5f });
var clay = ParticleFluidDef.Clay(0.06f);   // yields at 3 kPa and keeps its dents; Snow, Mud, Dough, Gravel likewise
```

Solids (`ElasticModulus` above 0) use the material point method: grains pile at their friction angle, snow breaks, clay keeps a dent, and they push rigid bodies, soft bodies and rods as fluids do. A stiff solid makes the world take more substeps (up to 64), so a softer `ElasticModulus` is cheaper. `Honey` (`DynamicViscosity` in Pa·s) solves viscosity implicitly. See `docs/spec/fluids.md` for the model and what is not met yet.
