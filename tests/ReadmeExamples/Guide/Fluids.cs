using System;
using RecursiveAI.OneMath;               // Vector3F, Vector3D
using RecursiveAI.OnePhysics;            // BodyType
using RecursiveAI.OnePhysics.Fluids;     // ParticleFluid, ParticleFluidDef, FluidEmitterDef, FluidEventKind
using RecursiveAI.OnePhysics.Physics3D;  // PhysicsWorld3D

namespace GuideExamples;

/// <summary>The code blocks of docs/guide/fluids.md.</summary>
public static class Fluids
{
    public static (int Particles, float Fraction, ulong Hash) Run()
    {
        var world = new PhysicsWorld3D();
        world.AddBox(new Vector3F(3, 0.2f, 3), at: new Vector3D(0, -0.1, 0), type: BodyType.Static);
        world.AddBox(new Vector3F(0.2f, 3, 3), at: new Vector3D(-0.6, 1.4, 0), type: BodyType.Static);
        world.AddBox(new Vector3F(0.2f, 3, 3), at: new Vector3D(0.6, 1.4, 0), type: BodyType.Static);
        world.AddBox(new Vector3F(3, 3, 0.2f), at: new Vector3D(0, 1.4, -0.6), type: BodyType.Static);
        world.AddBox(new Vector3F(3, 3, 0.2f), at: new Vector3D(0, 1.4, 0.6), type: BodyType.Static);

        // Happy path: water, a box of water in the tank, a floating crate.
        ParticleFluid water = world.AddParticleWater(spacing: 0.06f);
        world.FillFluidBox(water, new Vector3D(-0.5, 0, -0.5), new Vector3D(0.5, 0.4, 0.5));
        var crate = world.AddBox(new Vector3F(0.25f, 0.25f, 0.25f), at: new Vector3D(0, 0.7, 0), density: 400f);

        for (int i = 0; i < 120; i++) world.Step(1f / 60);

        float fraction = world.GetFluidState(crate, water).SubmergedFraction;
        foreach (var e in world.FluidEvents) Console.WriteLine($"{e.Kind} body {e.Body.Index} fluid {e.Fluid}");

        // Middle tier: a thicker liquid, a nozzle, a drain and the step settings.
        var honey = ParticleFluidDef.Honey(spacing: 0.06f);
        honey.Friction = 0.8f;
        world.AddParticleFluid(honey);   // a second fluid with the same spacing
        water.AddEmitter(new FluidEmitterDef { Position = new Vector3D(0, 1.8, 0), Direction = new Vector3F(0, -1, 0), Size = new Vector3F(0.1f, 0.1f, 0.1f), Speed = 2f });
        water.AddDrain(new FluidDrainDef { Position = new Vector3D(0, 0.05, 0.55), Size = new Vector3F(0.5f, 0.05f, 0.05f) });
        world.FluidSettings = new FluidSettings { Substeps = 2, Iterations = 4, MaxParticles = 50000, SleepSpeed = 0.02f, TimeToSleep = 1f, WakeBodySpeed = 0.05f };
        for (int i = 0; i < 30; i++) world.Step(1f / 60);

        // Queries: the particles near a point, and the first one a ray meets.
        var near = new FluidParticleInfo[64];
        int count = world.OverlapFluidSphere(new Vector3D(0.3, 0.2, 0), 0.1f, near, out int total);
        bool hit = world.RaycastFluid(new Vector3D(0.3, 2, 0), new Vector3F(0, -1, 0), out FluidRayHit first);
        Console.WriteLine($"{count} of {total} particles near, ray {(hit ? first.Distance : -1f)}");
        var dry = new PhysicsWorld3D();   // sand lives in a world of its own here: the fluids of a world share one spacing
        dry.AddBox(new Vector3F(3, 0.2f, 3), at: new Vector3D(0, -0.1, 0), type: BodyType.Static);
        Materials(dry);
        for (int i = 0; i < 20; i++) dry.Step(1f / 60);
        return (world.FluidParticleCount, fraction, world.ComputeHash());
    }

    private static void Materials(PhysicsWorld3D world)
    {
        var sand = world.AddParticleFluid(ParticleFluidDef.Sand(spacing: 0.06f));   // ElasticModulus above 0 makes a solid
        sand.AddEmitter(new FluidEmitterDef { Position = new Vector3D(0, 1, 0), Direction = new Vector3F(0, -1, 0), Kind = FluidVolumeKind.Sphere, Size = new Vector3F(0.08f, 0.08f, 0.08f), Speed = 0.5f });
        var clay = ParticleFluidDef.Clay(0.06f);   // yields at 3 kPa and keeps its dents; Snow, Mud, Dough, Gravel likewise
        _ = clay;
    }
}
