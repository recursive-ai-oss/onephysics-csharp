using System.Runtime.CompilerServices;
using Bar;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.HashMatrix;

/// <summary>
/// The 3D world on the shared scene definitions, hashed after every step: the 10-layer box pyramid for 300 steps (it falls asleep early), the mixed pile for its first 100 steps,
/// the mesh terrain for 100 steps (landing on triangles), a pile that is snapshotted, restored and shifted halfway, and a scene with one joint of every kind that is snapshotted and restored halfway.
/// The number of threads comes from the runner and must not change a bit.
/// </summary>
internal static class World3DScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("world3d/pyramid-10", ctx => RunScene(ctx, "pyramid-3d-10", 300));
        Scenarios.Register("world3d/pile", ctx => RunScene(ctx, "large-pile-3d", 100));
        Scenarios.Register("world3d/terrain", ctx => RunScene(ctx, "mesh-terrain-3d", 100));
        Scenarios.Register("world3d/restore-and-shift", RunRestoreAndShift);
        Scenarios.Register("world3d/joints", RunJoints);
        Scenarios.Register("world3d/characters", ctx => Characters3DScene.Run(ctx.Threads, true, h => { ctx.Write((long)h); ctx.EndStep(); }));
        Scenarios.Register("world3d/vehicles", ctx => Vehicles3DScene.Run(ctx.Threads, true, h => { ctx.Write((long)h); ctx.EndStep(); }));
        Scenarios.Register("world3d/vehicle-kinds", ctx => VehicleKinds3DScene.Run(ctx.Threads, true, h => { ctx.Write((long)h); ctx.EndStep(); }));
        Scenarios.Register("world3d/ragdolls", ctx => Ragdolls3DScene.Run(ctx.Threads, true, h => { ctx.Write((long)h); ctx.EndStep(); }));
        Scenarios.Register("world3d/buoyancy", RunBuoyancy);
        Scenarios.Register("world3d/softbody", RunSoftBodies);
        Scenarios.Register("world3d/softbig", RunSoftBig);
        Scenarios.Register("world3d/softbodies", ctx => SoftBodies3DScene.Run(ctx.Threads, true, h => { ctx.Write((long)h); ctx.EndStep(); }));
        Scenarios.Register("world3d/softskinned", ctx => SoftSkinned3DScene.Run(ctx.Threads, true, h => { ctx.Write((long)h); ctx.EndStep(); }));
        Scenarios.Register("world3d/services", ctx => Services3DScene.Run(ctx.Threads, true, h => { ctx.Write((long)h); ctx.EndStep(); }));
    }

    private static void RunScene(ScenarioContext ctx, string name, int steps)
    {
        var scene = Gen.All().First(s => s.Name == name);
        var (world, _, _) = SceneBuilder3D.Build(scene, ctx.Threads);
        float dt = (float)scene.Dt;
        for (int i = 0; i < steps; i++)
        {
            world.Step(dt);
            ctx.Write((long)world.ComputeHash());
            ctx.EndStep();
        }
    }

    private static void RunRestoreAndShift(ScenarioContext ctx)
    {
        var scene = Gen.All().First(s => s.Name == "large-pile-3d");
        var (world, _, _) = SceneBuilder3D.Build(scene, ctx.Threads);
        float dt = (float)scene.Dt;
        byte[]? snap = null;
        for (int i = 0; i < 90; i++)
        {
            if (i == 30) { snap = world.Snapshot(); ctx.Write(snap); }
            if (i == 45) { world.Restore(snap!); }
            if (i == 60) { world.ShiftOrigin(new Vector3D(32.0, -8.0, 16.0)); }
            world.Step(dt);
            ctx.Write((long)world.ComputeHash());
            ctx.EndStep();
        }
    }

    /// <summary>The buoyancy pool (500 floating and sinking bodies in a current): snapshotted at step 40, restored at 70, the origin shifted at 100, hashed every step for 150 steps.</summary>
    private static void RunBuoyancy(ScenarioContext ctx)
    {
        var scene = Gen.All().First(s => s.Name == "buoyancy-3d");
        var (world, _, _) = SceneBuilder3D.Build(scene, ctx.Threads);
        float dt = (float)scene.Dt;
        byte[]? snap = null;
        for (int i = 0; i < 150; i++)
        {
            if (i == 40) { snap = world.Snapshot(); ctx.Write(snap); }
            if (i == 70) { world.Restore(snap!); }
            if (i == 100) { world.ShiftOrigin(new Vector3D(32.0, -8.0, 16.0)); }
            world.Step(dt);
            ctx.Write((long)world.ComputeHash());
            ctx.EndStep();
        }
    }

    /// <summary>
    /// The soft-body scenes: cushions under falling boxes (snapshotted at step 40, restored at 70, the origin shifted at 100, 150 steps), then the draped cloth for 120 steps and the soft bodies on mesh terrain for 80 steps,
    /// the world hash after every step. Soft bodies are solved colour by colour in tasks cut from the constraint graph, so neither the thread count nor the vector width may change a bit.
    /// </summary>
    private static void RunSoftBodies(ScenarioContext ctx)
    {
        var scene = Gen.All().First(s => s.Name == "soft-rigid-3d");
        var (world, _, _) = SceneBuilder3D.Build(scene, ctx.Threads);
        SceneBuilder3D.AddSofts(scene, world);
        float dt = (float)scene.Dt;
        byte[]? snap = null;
        for (int i = 0; i < 150; i++)
        {
            if (i == 40) { snap = world.Snapshot(); ctx.Write(snap); }
            if (i == 70) { world.Restore(snap!); }
            if (i == 100) { world.ShiftOrigin(new Vector3D(32.0, -8.0, 16.0)); }
            world.Step(dt);
            ctx.Write((long)world.ComputeHash());
            ctx.EndStep();
        }

        foreach (var (name, steps) in new[] { ("cloth-drape-3d", 120), ("soft-mesh-3d", 80) })
        {
            var sc = Gen.All().First(s => s.Name == name);
            var (w, _, _) = SceneBuilder3D.Build(sc, ctx.Threads);
            SceneBuilder3D.AddSofts(sc, w);
            for (int i = 0; i < steps; i++)
            {
                w.Step((float)sc.Dt);
                ctx.Write((long)w.ComputeHash());
                ctx.EndStep();
            }
        }
    }

    /// <summary>
    /// Soft bodies large enough to be solved in tasks by colour: a 36 x 36 cloth (1369 particles, pinned at two corners) over a sphere and a box, and a jelly cube of 10 x 10 x 10 cells (1331 particles, tetrahedra) falling onto it,
    /// a pressurised ball beside them. Snapshotted at step 30 and restored at step 50; the world hash after every step, 90 steps.
    /// </summary>
    private static void RunSoftBig(ScenarioContext ctx)
    {
        var w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = ctx.Threads, WorldId = 23, SoftSubsteps = 4 });
        w.AddBox(new Vector3F(40f, 1f, 40f), at: new Vector3D(0, -1, 0), type: BodyType.Static);
        w.AddSphere(0.8f, at: new Vector3D(-0.6, 0.8, 0.2), type: BodyType.Static);
        w.AddBox(new Vector3F(0.6f, 0.6f, 0.6f), at: new Vector3D(1.4, 0.6, 0.4), type: BodyType.Static);
        w.AddCloth(4f, 4f, 36, new Vector3D(0, 1.8, 0), pinCorners: true);
        w.AddSoftBox(new Vector3F(0.8f, 0.8f, 0.8f), new Vector3D(0.3, 3.0, 0.1), cells: 10);
        w.AddSoftBall(0.4f, new Vector3D(-2.6, 1.0, 0.0), segments: 12);
        byte[]? snap = null;
        for (int i = 0; i < 90; i++)
        {
            if (i == 30) { snap = w.Snapshot(); ctx.Write(snap); }
            if (i == 50) { w.Restore(snap!); }
            w.Step(1f / 60f);
            ctx.Write((long)w.ComputeHash());
            ctx.EndStep();
        }
    }

    /// <summary>One joint of every kind swinging over a floor, plus a joint that breaks; snapshotted and restored at step 40, hashed every step for 120 steps.</summary>
    private static void RunJoints(ScenarioContext ctx)
    {
        var w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = ctx.Threads, WorldId = 21 });
        w.AddBox(new Vector3F(100f, 1f, 100f), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var top = w.AddBox(new Vector3F(0.2f, 0.2f, 0.2f), new Vector3D(0, 6, 0), type: BodyType.Static);
        Body Box(double x, double y, double z) => w.AddBox(new Vector3F(0.4f, 0.4f, 0.4f), new Vector3D(x, y, z));
        var b1 = Box(1, 6, 0); var b2 = Box(2, 6, 0); var b3 = Box(3, 6, 0); var b4 = Box(4, 6, 0); var b5 = Box(5, 6, 0); var b6 = Box(6, 6, 0); var b7 = Box(7, 6, 0); var b8 = Box(8, 6, 0);
        w.AddHinge(top, b1, new Vector3D(0.5, 6, 0), new Vector3F(0, 0, 1)).SetLimits(-1f, 1f);
        w.AddCharacterJoint(b1, b2, new Vector3D(1.5, 6, 0), new Vector3F(1, 0, 0));
        w.AddCone(b2, b3, new Vector3D(2.5, 6, 0), new Vector3F(1, 0, 0), 0.6f);
        var six = w.AddConfigurable(b3, b4, new Vector3D(3.5, 6, 0));
        six.SetAxisMode(JointAxes.AngularZ, JointAxisMode.Free); six.SetAxisMode(JointAxes.LinearX, JointAxisMode.Limited); six.SetAxisLimits(JointAxes.LinearX, -0.1f, 0.1f);
        w.AddSlider(b4, b5, new Vector3D(4.5, 6, 0), new Vector3F(1, 0, 0)).EnableSpring = true;
        w.AddSpring(b5, b6, new Vector3D(5, 6, 0), new Vector3D(6, 6, 0), 3f, 0.3f);
        w.AddDistance(b6, b7, new Vector3D(6.2, 6, 0), new Vector3D(6.8, 6, 0));
        w.AddFixed(b7, b8, new Vector3D(7.5, 6, 0));
        var p = w.AddSphere(0.3f, new Vector3D(-3, 4, 0));
        var q = w.AddSphere(0.3f, new Vector3D(-4, 4, 0));
        w.AddGear(p, q, new Vector3F(0, 0, 1), new Vector3F(0, 0, 1), 1.5f).Ratio = 1.5f;
        p.AngularVelocity = new Vector3F(0, 0, 2);
        var track = w.AddBox(new Vector3F(0.2f, 0.2f, 0.2f), new Vector3D(-8, 3, 0), type: BodyType.Static);
        var cart = w.AddBox(new Vector3F(0.3f, 0.3f, 0.3f), new Vector3D(-8, 3, 0));
        var path = w.AddPath(track, cart, [new Vector3D(-8, 3, 0), new Vector3D(-7, 3, 1), new Vector3D(-6, 3.5, 2), new Vector3D(-5, 3, 3)]);
        path.MotorSpeed = 1f; path.MaxMotorForce = 200f; path.EnableMotor = true;
        var l = w.AddBox(new Vector3F(0.4f, 0.4f, 0.4f), new Vector3D(-1, 5, 3));
        var r = w.AddBox(new Vector3F(0.4f, 0.4f, 0.4f), new Vector3D(1, 4, 3), density: 3f);
        w.AddPulley(l, r, l.Position, r.Position, new Vector3D(-1, 8, 3), new Vector3D(1, 8, 3));

        byte[]? snap = null;
        for (int i = 0; i < 120; i++)
        {
            if (i == 40) { snap = w.Snapshot(); ctx.Write(snap); }
            if (i == 60) { w.Restore(snap!); }
            w.Step(1f / 60f);
            ctx.Write((long)w.ComputeHash());
            ctx.EndStep();
        }
    }
}
