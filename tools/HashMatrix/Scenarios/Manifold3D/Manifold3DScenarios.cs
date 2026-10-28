using System;
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.HashMatrix;

/// <summary>
/// Contact manifolds of fixed random shape pairs: every shape kind (hulls built from seeded points, rounded boxes and cylinders included), random poses from deep
/// overlap to separated by more than the speculative distance, and a family of resting boxes and cylinders. Each step hashes 100 manifolds: point count,
/// normal, and every point's position, separation and feature id.
/// </summary>
internal static class Manifold3DScenarios
{
    private sealed class Rng
    {
        private ulong _s;

        public Rng(ulong seed) => _s = seed;

        public ulong Next()
        {
            ulong z = _s += 0x9E3779B97F4A7C15UL;
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
            z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
            return z ^ (z >> 31);
        }

        public float F() => (Next() >> 40) * (1f / 16777216f);

        public float Range(float lo, float hi) => lo + ((hi - lo) * F());

        public int Int(int n) => (int)(Next() % (ulong)n);

        public Vector3F Ball(float r)
        {
            while (true)
            {
                var v = new Vector3F(Range(-1, 1), Range(-1, 1), Range(-1, 1));
                if (v.LengthSquared() <= 1f)
                {
                    return v * r;
                }
            }
        }

        public QuaternionF Rotation()
        {
            while (true)
            {
                var q = new QuaternionF(Range(-1, 1), Range(-1, 1), Range(-1, 1), Range(-1, 1));
                float l = q.LengthSquared();
                if (l > 0.01f && l <= 1f)
                {
                    return q.Normalized();
                }
            }
        }
    }

    [ModuleInitializer]
    internal static void Init() => Scenarios.Register("collision3d/manifold", Run);

    private static ConvexShape3D Shape(Rng rng, ConvexHull3D[] hulls)
    {
        switch (rng.Int(8))
        {
            case 0: return ConvexShape3D.Sphere(rng.Range(0.2f, 1.2f));
            case 1: return ConvexShape3D.Box(new Vector3F(rng.Range(0.3f, 1.2f), rng.Range(0.3f, 1.2f), rng.Range(0.3f, 1.2f)));
            case 2: return ConvexShape3D.Box(new Vector3F(rng.Range(0.3f, 1.2f), rng.Range(0.3f, 1.2f), rng.Range(0.3f, 1.2f)), rng.Range(0f, 0.25f));
            case 3: return ConvexShape3D.Capsule(rng.Range(0.1f, 1f), rng.Range(0.1f, 0.6f));
            case 4: return ConvexShape3D.TaperedCapsule(rng.Range(0.3f, 1f), rng.Range(0.1f, 0.6f), rng.Range(0.1f, 0.6f));
            case 5: return ConvexShape3D.Cylinder(rng.Range(0.2f, 1f), rng.Range(0.2f, 1f), rng.Range(0f, 0.15f));
            default: return ConvexShape3D.FromHull(hulls[rng.Int(hulls.Length)], rng.Int(4) == 0 ? rng.Range(0f, 0.2f) : 0f);
        }
    }

    private static void Write(ScenarioContext ctx, Manifold3D m)
    {
        ctx.Write(m.PointCount);
        ctx.Write(m.Normal.X);
        ctx.Write(m.Normal.Y);
        ctx.Write(m.Normal.Z);
        for (int i = 0; i < m.PointCount; i++)
        {
            ManifoldPoint3D p = m[i];
            ctx.Write(p.Point.X);
            ctx.Write(p.Point.Y);
            ctx.Write(p.Point.Z);
            ctx.Write(p.Separation);
            ctx.Write((int)p.Id);
        }
    }

    private static void Run(ScenarioContext ctx)
    {
        var rng = new Rng(20260427);
        var hulls = new ConvexHull3D[12];
        var pts = new Vector3F[48];
        for (int i = 0; i < hulls.Length; i++)
        {
            int n = 6 + rng.Int(30);
            for (int k = 0; k < n; k++)
            {
                pts[k] = rng.Ball(1f);
            }

            hulls[i] = ConvexHull3D.Build(pts.AsSpan(0, n));
        }

        var gen = new ManifoldGenerator3D();
        for (int step = 0; step < 60; step++)
        {
            for (int k = 0; k < 100; k++)
            {
                ConvexShape3D a = Shape(rng, hulls), b = Shape(rng, hulls);
                var xf = new RigidTransformF(rng.Rotation(), rng.Ball(rng.Range(0.1f, 2.6f)));
                Write(ctx, gen.Collide(a, b, xf, rng.Range(0f, 0.1f)));
            }

            // Resting contacts: boxes and cylinders on a slab, slightly sunk, tilted a little.
            ConvexShape3D slab = ConvexShape3D.Box(new Vector3F(4f, 0.5f, 4f));
            ConvexShape3D top = (step & 1) == 0 ? ConvexShape3D.Box(new Vector3F(0.5f, 0.4f, 0.6f)) : ConvexShape3D.Cylinder(0.4f, 0.6f);
            var tilt = new QuaternionF(rng.Range(-0.02f, 0.02f), rng.Range(-1f, 1f), rng.Range(-0.02f, 0.02f), 1f).Normalized();
            Write(ctx, gen.Collide(slab, top, new RigidTransformF(tilt, new Vector3F(rng.Range(-2f, 2f), 0.88f + rng.Range(-0.02f, 0.02f), rng.Range(-2f, 2f)))));
            ctx.EndStep();
        }
    }
}
