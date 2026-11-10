using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Fluids;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

// The particle-fluid scenes, built in code (no files) and shared by the tests, the hash-matrix runner, the smoke apps and the stability command of the bar runner: a dam break in both dimensions (dam-break-2d, dam-break-3d),
// a light, a medium and a heavy box dropped into a pool (fluid-boxes-3d), a cloth hammock filling from a nozzle (fluid-cloth-3d) and a rope across a channel fed by a nozzle and emptied by a drain (fluid-rope-2d); and the solid and viscous
// materials: sand poured into a pile (sand-pile-3d), sand running out of a silo (hopper-2d), a honey thread falling on a plate (honey-coil-3d), a snowball thrown at a wall (snow-impact-3d) and a clay block pressed by a box (clay-press-2d).
// The spacing argument is the particle spacing in metres: the stability runner uses the defaults, the hash scenarios and tests use coarser ones.
internal static class FluidScenes
{
    internal const float Dt = 1f / 60f;

    internal static readonly string[] Names = { "dam-break-2d", "dam-break-3d", "fluid-boxes-3d", "fluid-cloth-3d", "fluid-rope-2d", "sand-pile-3d", "hopper-2d", "honey-coil-3d", "snow-impact-3d", "clay-press-2d" };

    internal static bool Is3D(string name) => name.EndsWith("3d", StringComparison.Ordinal);

    /// <summary>The scene's default spacing for stability runs.</summary>
    internal static float DefaultSpacing(string name) => name switch
    {
        "dam-break-2d" => 0.02f,
        "dam-break-3d" => 0.04f,
        "fluid-boxes-3d" => 0.05f,
        "fluid-cloth-3d" => 0.05f,
        "fluid-rope-2d" => 0.03f,
        "sand-pile-3d" => 0.04f,
        "hopper-2d" => 0.02f,
        "honey-coil-3d" => 0.006f,
        "snow-impact-3d" => 0.04f,
        "clay-press-2d" => 0.02f,
        _ => throw new ArgumentException("unknown scene " + name, nameof(name)),
    };

    /// <summary>The spacing the hash matrix and the smoke apps use for the scene: coarser than the stability runs, so that a step is quick.</summary>
    internal static float CoarseSpacing(string name) => name switch
    {
        "dam-break-2d" => 0.06f,
        "fluid-rope-2d" => 0.08f,
        "sand-pile-3d" => 0.08f,
        "hopper-2d" => 0.05f,
        "honey-coil-3d" => 0.015f,
        "snow-impact-3d" => 0.08f,
        "clay-press-2d" => 0.05f,
        _ => 0.1f,
    };

    /// <summary>The box particles must stay inside (a margin beyond the container): min x y z, max x y z.</summary>
    internal static (double[] Min, double[] Max) Bounds(string name) => name switch
    {
        "dam-break-2d" => (new[] { -0.1, -0.1, 0.0 }, new[] { 2.6, 4.0, 0.0 }),
        "dam-break-3d" => (new[] { -0.1, -0.1, -0.1 }, new[] { 2.1, 3.0, 0.6 }),
        "fluid-boxes-3d" => (new[] { -0.6, -0.1, -0.6 }, new[] { 0.6, 2.0, 0.6 }),
        "fluid-cloth-3d" => (new[] { -0.52, -0.1, -0.52 }, new[] { 0.52, 3.0, 0.52 }),
        "fluid-rope-2d" => (new[] { -1.05, -0.1, 0.0 }, new[] { 2.05, 4.0, 0.0 }),
        "sand-pile-3d" => (new[] { -3.0, -0.1, -3.0 }, new[] { 3.0, 1.6, 3.0 }),
        "hopper-2d" => (new[] { -0.45, -1.0, 0.0 }, new[] { 0.45, 3.0, 0.0 }),
        "honey-coil-3d" => (new[] { -1.0, -0.05, -1.0 }, new[] { 1.0, 0.4, 1.0 }),
        "snow-impact-3d" => (new[] { -1.0, -0.1, -2.0 }, new[] { 1.95, 6.0, 2.0 }),
        "clay-press-2d" => (new[] { -3.0, -0.1, 0.0 }, new[] { 3.0, 1.0, 0.0 }),
        _ => throw new ArgumentException("unknown scene " + name, nameof(name)),
    };

    internal sealed class Extras
    {
        public ParticleFluid Fluid;
        public P3.Body[] Boxes = Array.Empty<P3.Body>();
        public P3.SoftBody? Cloth;
        public P2.Rod? Rope;
        public P2.Body Press;
    }

    private static readonly System.Runtime.CompilerServices.ConditionalWeakTable<object, Extras> Parts = new();

    internal static Extras ExtrasOf(object world) => Parts.GetOrCreateValue(world);

    // ------------------------------------------------------------------ 2D

    internal static P2.PhysicsWorld2D Build2D(string name, int threads, float spacing = 0f, int worldId = 91)
    {
        if (spacing <= 0f) { spacing = DefaultSpacing(name); }
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { ThreadCount = threads, WorldId = worldId });
        var ex = Parts.GetOrCreateValue(w);
        switch (name)
        {
            case "dam-break-2d": DamBreak2D(w, ex, spacing); break;
            case "fluid-rope-2d": RopeChannel(w, ex, spacing); break;
            case "hopper-2d": Hopper(w, ex, spacing); break;
            case "clay-press-2d": ClayPress(w, ex, spacing); break;
            default: throw new ArgumentException("not a 2D fluid scene: " + name, nameof(name));
        }

        return w;
    }

    // A 0.6 x 0.6 m column against the left wall of a 2.5 m channel (Martin and Moyce 1952, n^2 = 1).
    private static void DamBreak2D(P2.PhysicsWorld2D w, Extras ex, float s)
    {
        w.AddBox(new Vector2F(8f, 0.2f), at: new Vector2D(3, -0.1), type: BodyType.Static);
        w.AddBox(new Vector2F(0.2f, 4f), at: new Vector2D(-0.1, 2), type: BodyType.Static);
        w.AddBox(new Vector2F(0.2f, 4f), at: new Vector2D(2.6, 2), type: BodyType.Static);
        ex.Fluid = w.AddParticleFluid(ParticleFluidDef.Water(s));
        w.FillFluidBox(ex.Fluid, new Vector2D(0, 0), new Vector2D(0.6, 0.6));
    }

    // A 3 m wide channel, a rope pinned a hand's breadth from the left wall that swings down into the pool from 0.9 m, a pool below, a nozzle pouring from above, a drain in the pool's floor corner.
    private static void RopeChannel(P2.PhysicsWorld2D w, Extras ex, float s)
    {
        w.AddBox(new Vector2F(4f, 0.2f), at: new Vector2D(0.5, -0.1), type: BodyType.Static);
        w.AddBox(new Vector2F(0.2f, 5f), at: new Vector2D(-1.1, 2.4), type: BodyType.Static);
        w.AddBox(new Vector2F(0.2f, 5f), at: new Vector2D(2.1, 2.4), type: BodyType.Static);
        ex.Fluid = w.AddParticleFluid(ParticleFluidDef.Water(s));
        w.FillFluidBox(ex.Fluid, new Vector2D(-1.0, 0), new Vector2D(2.0, 0.45));
        ex.Rope = w.AddRope(new Vector2D(-0.9, 0.9), new Vector2D(0.9, 0.9), 18, true, false);
        ex.Fluid.AddEmitter(new FluidEmitterDef { Position = new Vector3D(0.5, 1.3, 0), Direction = new Vector3F(0, -1, 0), Kind = FluidVolumeKind.Box, Size = new Vector3F(0.2f, 0.1f, 0.1f), Speed = 1f, MaxEmitted = 1000 });
        ex.Fluid.AddDrain(new FluidDrainDef { Position = new Vector3D(1.9, 0.05, 0), Kind = FluidVolumeKind.Box, Size = new Vector3F(0.1f, 0.05f, 0.1f) });
    }

    // A 0.8 m wide silo on a floor with a 0.2 m slot, filled with sand a metre deep, and a drain below the slot that counts what ran out.
    private static void Hopper(P2.PhysicsWorld2D w, Extras ex, float s)
    {
        const float width = 0.8f, slot = 0.2f;
        float plate = 0.5f * (width - slot);
        w.AddBox(new Vector2F(plate, 0.1f), at: new Vector2D(-(slot / 2 + plate / 2), -0.05), type: BodyType.Static);
        w.AddBox(new Vector2F(plate, 0.1f), at: new Vector2D(slot / 2 + plate / 2, -0.05), type: BodyType.Static);
        w.AddBox(new Vector2F(0.1f, 3f), at: new Vector2D(-width / 2 - 0.05, 1.45), type: BodyType.Static);
        w.AddBox(new Vector2F(0.1f, 3f), at: new Vector2D(width / 2 + 0.05, 1.45), type: BodyType.Static);
        ex.Fluid = w.AddParticleFluid(ParticleFluidDef.Sand(s));
        ex.Fluid.AddDrain(new FluidDrainDef { Position = new Vector3D(0, -0.6, 0), Kind = FluidVolumeKind.Box, Size = new Vector3F(2f, 0.2f, 1f) });
        w.FillFluidBox(ex.Fluid, new Vector2D(-width / 2, 0), new Vector2D(width / 2, 1.0));
    }

    // A 0.8 m by 0.3 m block of clay on a floor with a 0.3 m wide kinematic press above it; BeforeStep moves the press down 0.05 m into the clay, holds it, and lifts it.
    private static void ClayPress(P2.PhysicsWorld2D w, Extras ex, float s)
    {
        w.AddBox(new Vector2F(6f, 0.2f), at: new Vector2D(0, -0.1), type: BodyType.Static);
        ex.Fluid = w.AddParticleFluid(ParticleFluidDef.Clay(s));
        w.FillFluidBox(ex.Fluid, new Vector2D(-0.4, 0), new Vector2D(0.4, 0.3));
        ex.Press = w.AddBox(new Vector2F(0.3f, 0.1f), at: new Vector2D(0, PressY(0)), type: BodyType.Kinematic);
    }

    /// <summary>The press's height at the step: rest at 0.36 m for 60 steps while the clay settles, down at 0.3 m/s for 10 steps, held for 30, up at 0.3 m/s for 12 steps, then still.</summary>
    internal static double PressY(int step)
    {
        const double Rest = 0.36, Speed = 0.3 / 60.0;
        int t = step - 60;
        if (t <= 0) { return Rest; }
        if (t <= 10) { return Rest - Speed * t; }
        if (t <= 40) { return Rest - Speed * 10; }
        if (t <= 52) { return Rest - Speed * 10 + Speed * (t - 40); }
        return Rest;
    }

    /// <summary>Scene scripts of the planar scenes, run before each step (the kinematic press of clay-press-2d).</summary>
    internal static void BeforeStep2D(string name, P2.PhysicsWorld2D w, int step, Vector2D shifted = default)
    {
        // A world whose origin was shifted by an offset holds everything at its old position minus the offset.
        if (name == "clay-press-2d")
        {
            // The press is the scene's only kinematic body (a forked world has no Extras, so it is found rather than held).
            foreach (var b in w.Bodies) { if (b.IsKinematic) { b.MoveKinematic(new Vector2D(-shifted.X, PressY(step) - shifted.Y), 0f); break; } }
        }
    }

    // ------------------------------------------------------------------ 3D

    internal static P3.PhysicsWorld3D Build3D(string name, int threads, float spacing = 0f, int worldId = 92)
    {
        if (spacing <= 0f) { spacing = DefaultSpacing(name); }
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { ThreadCount = threads, WorldId = worldId });
        var ex = Parts.GetOrCreateValue(w);
        switch (name)
        {
            case "dam-break-3d": DamBreak3D(w, ex, spacing); break;
            case "fluid-boxes-3d": Boxes(w, ex, spacing); break;
            case "fluid-cloth-3d": ClothBag(w, ex, spacing); break;
            case "sand-pile-3d": SandPile(w, ex, spacing); break;
            case "honey-coil-3d": HoneyThread(w, ex, spacing); break;
            case "snow-impact-3d": SnowImpact(w, ex, spacing); break;
            default: throw new ArgumentException("not a 3D fluid scene: " + name, nameof(name));
        }

        return w;
    }

    // A 0.5 m cube of water against one end of a 0.5 m wide, 2 m long channel.
    private static void DamBreak3D(P3.PhysicsWorld3D w, Extras ex, float s)
    {
        w.AddBox(new Vector3F(6f, 0.2f, 1.0f), at: new Vector3D(2, -0.1, 0.25), type: BodyType.Static);
        w.AddBox(new Vector3F(0.2f, 3f, 1.0f), at: new Vector3D(-0.1, 1.5, 0.25), type: BodyType.Static);
        w.AddBox(new Vector3F(0.2f, 3f, 1.0f), at: new Vector3D(2.1, 1.5, 0.25), type: BodyType.Static);
        w.AddBox(new Vector3F(6f, 3f, 0.2f), at: new Vector3D(2, 1.5, -0.1), type: BodyType.Static);
        w.AddBox(new Vector3F(6f, 3f, 0.2f), at: new Vector3D(2, 1.5, 0.6), type: BodyType.Static);
        ex.Fluid = w.AddParticleFluid(ParticleFluidDef.Water(s));
        w.FillFluidBox(ex.Fluid, new Vector3D(0, 0, 0), new Vector3D(0.5, 0.5, 0.5));
    }

    // A 1 m pool of water 0.5 m deep; a light box (density 300), a medium one (800) and a heavy one (2500) are dropped into it, side by side.
    private static void Boxes(P3.PhysicsWorld3D w, Extras ex, float s)
    {
        w.AddBox(new Vector3F(2f, 0.2f, 2f), at: new Vector3D(0, -0.1, 0), type: BodyType.Static);
        w.AddBox(new Vector3F(0.2f, 3f, 2f), at: new Vector3D(-0.6, 1.4, 0), type: BodyType.Static);
        w.AddBox(new Vector3F(0.2f, 3f, 2f), at: new Vector3D(0.6, 1.4, 0), type: BodyType.Static);
        w.AddBox(new Vector3F(2f, 3f, 0.2f), at: new Vector3D(0, 1.4, -0.6), type: BodyType.Static);
        w.AddBox(new Vector3F(2f, 3f, 0.2f), at: new Vector3D(0, 1.4, 0.6), type: BodyType.Static);
        ex.Fluid = w.AddParticleFluid(ParticleFluidDef.Water(s));
        w.FillFluidBox(ex.Fluid, new Vector3D(-0.5, 0, -0.5), new Vector3D(0.5, 0.5, 0.5));
        ex.Boxes = new[]
        {
            w.AddBox(new Vector3F(0.25f, 0.25f, 0.25f), at: new Vector3D(-0.3, 0.7, 0), density: 300f),
            w.AddBox(new Vector3F(0.25f, 0.25f, 0.25f), at: new Vector3D(0.0, 0.7, 0), density: 800f),
            w.AddBox(new Vector3F(0.25f, 0.25f, 0.25f), at: new Vector3D(0.3, 0.7, 0), density: 2500f),
        };
    }

    // A 1 m square of denim pinned along its edge to the walls of a 1 m tank with no floor of its own, a hammock 0.5 m above the ground, filling from a nozzle above.
    private static void ClothBag(P3.PhysicsWorld3D w, Extras ex, float s)
    {
        w.AddBox(new Vector3F(0.2f, 3f, 1.4f), at: new Vector3D(-0.6, 1.0, 0), type: BodyType.Static);
        w.AddBox(new Vector3F(0.2f, 3f, 1.4f), at: new Vector3D(0.6, 1.0, 0), type: BodyType.Static);
        w.AddBox(new Vector3F(1.4f, 3f, 0.2f), at: new Vector3D(0, 1.0, -0.6), type: BodyType.Static);
        w.AddBox(new Vector3F(1.4f, 3f, 0.2f), at: new Vector3D(0, 1.0, 0.6), type: BodyType.Static);
        w.AddBox(new Vector3F(3f, 0.2f, 3f), at: new Vector3D(0, -1.0, 0), type: BodyType.Static);
        const int seg = 20;
        var pins = new List<int>();
        for (int j = 0; j <= seg; j++) { for (int i = 0; i <= seg; i++) { if (i == 0 || j == 0 || i == seg || j == seg) { pins.Add(j * (seg + 1) + i); } } }
        // Collision radius larger than half the particle gap leaves no way between four cloth particles for a fluid particle (spacing 0.05, radius 0.02, fluid contact radius 0.025).
        ex.Cloth = w.AddCloth(new P3.ClothDef { Width = 1.0f, Depth = 1.0f, SegmentsX = seg, SegmentsZ = seg, Position = new Vector3D(0, 0.5, 0), PinnedParticles = pins, ParticleRadius = 0.02f, Material = P3.ClothMaterial.Denim });
        ex.Fluid = w.AddParticleFluid(ParticleFluidDef.Water(s));
        w.FillFluidBox(ex.Fluid, new Vector3D(-0.5, 0.55, -0.5), new Vector3D(0.5, 0.7, 0.5));
        ex.Fluid.AddEmitter(new FluidEmitterDef { Position = new Vector3D(0, 1.6, 0), Direction = new Vector3F(0, -1, 0), Kind = FluidVolumeKind.Sphere, Size = new Vector3F(0.15f, 0.15f, 0.15f), Speed = 2f, MaxEmitted = 1500 });
    }

    // Sand poured from a 0.08 m nozzle a metre above a floor builds a cone.
    private static void SandPile(P3.PhysicsWorld3D w, Extras ex, float s)
    {
        w.AddBox(new Vector3F(8f, 0.2f, 8f), at: new Vector3D(0, -0.1, 0), type: BodyType.Static);
        ex.Fluid = w.AddParticleFluid(ParticleFluidDef.Sand(s));
        ex.Fluid.AddEmitter(new FluidEmitterDef { Position = new Vector3D(0, 1.0, 0), Direction = new Vector3F(0, -1, 0), Kind = FluidVolumeKind.Sphere, Size = new Vector3F(0.08f, 0.08f, 0.08f), Speed = 0.5f, MaxEmitted = 1500 });
    }

    // A thread of honey 2.4 cm across falls 15 cm on a plate; it is fed for the length of the scene.
    private static void HoneyThread(P3.PhysicsWorld3D w, Extras ex, float s)
    {
        w.AddBox(new Vector3F(2f, 0.2f, 2f), at: new Vector3D(0, -0.1, 0), type: BodyType.Static);
        ex.Fluid = w.AddParticleFluid(ParticleFluidDef.Honey(s));
        ex.Fluid.AddEmitter(new FluidEmitterDef { Position = new Vector3D(0.0003, 0.15, 0), Direction = new Vector3F(0, -1, 0), Kind = FluidVolumeKind.Sphere, Size = new Vector3F(0.012f, 0.012f, 0.012f), Speed = 0.1f, MaxEmitted = 4000 });
    }

    // A snowball 0.3 m in radius thrown at 20 m/s at a wall 1.9 m away, over a floor.
    private static void SnowImpact(P3.PhysicsWorld3D w, Extras ex, float s)
    {
        w.AddBox(new Vector3F(8f, 0.2f, 8f), at: new Vector3D(0, -0.1, 0), type: BodyType.Static);
        w.AddBox(new Vector3F(0.2f, 4f, 4f), at: new Vector3D(2.0, 2.0, 0), type: BodyType.Static);
        // Side walls keep the splash on the floor (a ball that breaks at 20 m/s throws pieces sideways by metres, and which way depends on the grid).
        w.AddBox(new Vector3F(4.2f, 4f, 0.2f), at: new Vector3D(0.9, 2.0, -1.95), type: BodyType.Static);
        w.AddBox(new Vector3F(4.2f, 4f, 0.2f), at: new Vector3D(0.9, 2.0, 1.95), type: BodyType.Static);
        ex.Fluid = w.AddParticleFluid(ParticleFluidDef.Snow(s));
        w.FillFluidSphere(ex.Fluid, new Vector3D(0.0, 0.5, 0), 0.3, new Vector3F(20f, 0f, 0f));
    }

    // ------------------------------------------------------------------ stepping and hashes

    /// <summary>Steps the 2D scene and returns the final world hash.</summary>
    internal static ulong Run2D(string name, int threads, int steps, float spacing = 0f, bool restoreMidway = false, Action<int, P2.PhysicsWorld2D>? each = null)
    {
        var w = Build2D(name, threads, spacing);
        for (int i = 0; i < steps; i++)
        {
            BeforeStep2D(name, w, i);
            w.Step(Dt);
            if (restoreMidway && i == steps / 2) { var snap = w.Snapshot(); w = w.Fork(); w.Restore(snap); }
            each?.Invoke(i, w);
        }

        return w.ComputeHash();
    }

    /// <summary>Steps the 3D scene and returns the final world hash.</summary>
    internal static ulong Run3D(string name, int threads, int steps, float spacing = 0f, bool restoreMidway = false, Action<int, P3.PhysicsWorld3D>? each = null)
    {
        var w = Build3D(name, threads, spacing);
        for (int i = 0; i < steps; i++)
        {
            w.Step(Dt);
            if (restoreMidway && i == steps / 2) { var snap = w.Snapshot(); w = w.Fork(); w.Restore(snap); }
            each?.Invoke(i, w);
        }

        return w.ComputeHash();
    }

    // ------------------------------------------------------------------ metrics

    /// <summary>What the stability runner and the tests read off a fluid at one moment.</summary>
    internal struct Sample
    {
        public int Particles;
        public float MaxSpeed;
        public double DensityOver95;   // 95th percentile of the compression, max(0, density / rest density - 1)
        public int Escapes, NonFinite, Penetrating;
        public double MaxY, MeanY;
    }

    internal static Sample Measure2D(P2.PhysicsWorld2D w, string name)
    {
        var (min, max) = Bounds(name);
        int n = w.FluidParticleCount;
        var buf = new FluidParticleInfo[n];
        w.GetFluidParticles(buf);
        return Measure(buf, min, max, w.ParticleFluidCount > 0 ? w.GetParticleFluid(0) : default, (x, y, z, f) => w.FluidDensityAt(new Vector2D(x, y), f),
            (x, y, z, r) => w.OverlapCircleAny(new Vector2D(x, y), r));
    }

    internal static Sample Measure3D(P3.PhysicsWorld3D w, string name)
    {
        var (min, max) = Bounds(name);
        int n = w.FluidParticleCount;
        var buf = new FluidParticleInfo[n];
        w.GetFluidParticles(buf);
        return Measure(buf, min, max, w.ParticleFluidCount > 0 ? w.GetParticleFluid(0) : default, (x, y, z, f) => w.FluidDensityAt(new Vector3D(x, y, z), f),
            (x, y, z, r) => w.OverlapSphereAny(new Vector3D(x, y, z), r));
    }

    private static Sample Measure(FluidParticleInfo[] buf, double[] min, double[] max, ParticleFluid fluid, Func<double, double, double, int, float> density, Func<double, double, double, float, bool> overlaps)
    {
        var s = new Sample { Particles = buf.Length };
        if (!fluid.IsValid || buf.Length == 0) { return s; }
        double rest = fluid.RestSphDensity;
        var def = fluid.Definition;
        var over = new List<double>(buf.Length);
        double sumY = 0, maxY = double.MinValue;
        float rc = 0.5f * def.Spacing;
        foreach (var p in buf)
        {
            var q = p.Position; var v = p.Velocity;
            if (!(double.IsFinite(q.X) && double.IsFinite(q.Y) && double.IsFinite(q.Z) && float.IsFinite(v.X) && float.IsFinite(v.Y) && float.IsFinite(v.Z))) { s.NonFinite++; continue; }
            float speed = MathF.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z);
            if (speed > s.MaxSpeed) { s.MaxSpeed = speed; }
            if (q.X < min[0] || q.X > max[0] || q.Y < min[1] || q.Y > max[1] || q.Z < min[2] || q.Z > max[2]) { s.Escapes++; }
            sumY += q.Y; if (q.Y > maxY) { maxY = q.Y; }
            // A particle is a disc or ball of the contact radius; one that a shape overlaps by more than half of it has gone through the surface.
            if (overlaps(q.X, q.Y, q.Z, 0.5f * rc)) { s.Penetrating++; }
            float d = density(q.X, q.Y, q.Z, p.Fluid);
            over.Add(Math.Max(0.0, d / rest - 1.0));
        }

        over.Sort();
        if (over.Count > 0) { s.DensityOver95 = over[(int)(0.95 * (over.Count - 1))]; }
        s.MaxY = maxY; s.MeanY = sumY / Math.Max(1, buf.Length);
        return s;
    }
}
