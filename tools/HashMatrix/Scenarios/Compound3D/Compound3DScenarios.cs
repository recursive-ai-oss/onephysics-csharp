using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Collision3D;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.HashMatrix;

/// <summary>
/// Compounds: a static compound's tree bytes and mass properties, then a mutable compound edited step by step (add, remove, move; the tree bytes are hashed after every edit)
/// with ray, shape-cast, bounds-overlap and child-pair queries and child-pair manifolds (compound against convex and against compound) on each step. Children are random shapes,
/// including scaled ones, at random local transforms.
/// </summary>
internal static class Compound3DScenarios
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
    internal static void Init() => Scenarios.Register("collision3d/compound", Run);

    private static ConvexShape3D Shape(Rng rng, ConvexHull3D[] hulls)
    {
        ConvexShape3D s;
        switch (rng.Int(7))
        {
            case 0: s = ConvexShape3D.Sphere(rng.Range(0.2f, 1.0f)); break;
            case 1: s = ConvexShape3D.Box(new Vector3F(rng.Range(0.3f, 1.2f), rng.Range(0.3f, 1.2f), rng.Range(0.3f, 1.2f))); break;
            case 2: s = ConvexShape3D.Box(new Vector3F(rng.Range(0.3f, 1.2f), rng.Range(0.3f, 1.2f), rng.Range(0.3f, 1.2f)), rng.Range(0f, 0.25f)); break;
            case 3: s = ConvexShape3D.Capsule(rng.Range(0.1f, 1f), rng.Range(0.1f, 0.6f)); break;
            case 4: s = ConvexShape3D.Cylinder(rng.Range(0.2f, 1f), rng.Range(0.2f, 1f), rng.Range(0f, 0.15f)); break;
            case 5: s = ConvexShape3D.TaperedCapsule(rng.Range(0.3f, 1f), rng.Range(0.1f, 0.6f), rng.Range(0.1f, 0.6f)); break;
            default: s = ConvexShape3D.FromHull(hulls[rng.Int(hulls.Length)], rng.Int(4) == 0 ? rng.Range(0f, 0.2f) : 0f); break;
        }

        if (rng.Int(3) != 0)
        {
            return s;
        }

        float a = rng.Range(0.6f, 1.8f), b = rng.Range(0.6f, 1.8f), c = rng.Range(0.6f, 1.8f);
        Vector3F scale = ScaledShape3D.GetScaleSupport(s) switch
        {
            ScaleSupport3D.PerAxis => new Vector3F(a, b, c),
            ScaleSupport3D.AxisSymmetricY => new Vector3F(a, b, a),
            _ => new Vector3F(a, a, a),
        };
        return ScaledShape3D.Create(s, scale);
    }

    private static CompoundChild3D Child(Rng rng, ConvexHull3D[] hulls) =>
        new(Shape(rng, hulls), new RigidTransformF(rng.Rotation(), rng.Ball(4f)), rng.Int(6));

    private static void Write(ScenarioContext ctx, Vector3F v)
    {
        ctx.Write(v.X);
        ctx.Write(v.Y);
        ctx.Write(v.Z);
    }

    private static void Write(ScenarioContext ctx, Manifold3D m)
    {
        ctx.Write(m.PointCount);
        Write(ctx, m.Normal);
        for (int i = 0; i < m.PointCount; i++)
        {
            ManifoldPoint3D p = m[i];
            Write(ctx, p.Point);
            ctx.Write(p.Separation);
            ctx.Write((int)p.Id);
        }
    }

    private static void WriteBvh(ScenarioContext ctx, CompoundShape3D c)
    {
        StateWriter w = StateWriter.ToBuffer();
        c.WriteBvh(w);
        ctx.Write(w.ToArray());
    }

    private static void Run(ScenarioContext ctx)
    {
        var rng = new Rng(20261027);
        var hulls = new ConvexHull3D[8];
        var pts = new Vector3F[40];
        for (int i = 0; i < hulls.Length; i++)
        {
            int n = 6 + rng.Int(30);
            for (int k = 0; k < n; k++)
            {
                pts[k] = rng.Ball(1f);
            }

            hulls[i] = ConvexHull3D.Build(pts.AsSpan(0, n));
        }

        // A static compound: tree bytes, mass properties, and a first round of queries.
        var kids = new CompoundChild3D[24];
        for (int i = 0; i < kids.Length; i++)
        {
            kids[i] = Child(rng, hulls);
        }

        Compound3D stat = Compound3D.Create(kids);
        WriteBvh(ctx, stat);
        var mass = stat.GetMassProperties();
        ctx.Write(mass.Mass);
        Write(ctx, mass.Center);
        ctx.Write(mass.Inertia.M11);
        ctx.Write(mass.Inertia.M22);
        ctx.Write(mass.Inertia.M33);
        ctx.Write(mass.Inertia.M12);
        ctx.Write(mass.Inertia.M13);
        ctx.Write(mass.Inertia.M23);
        ctx.EndStep();

        var m = new MutableCompound3D(kids);
        var other = Compound3D.Create(new[] { Child(rng, hulls), Child(rng, hulls), Child(rng, hulls) });
        var gen = new ManifoldGenerator3D();
        var contacts = new List<ChildContact3D>();
        var ids = new List<int>();
        var pairs = new List<SubShapePair3D>();
        for (int step = 0; step < 40; step++)
        {
            // One edit.
            int op = rng.Int(10);
            var live = new List<int>();
            for (int i = 0; i < m.SlotCount; i++)
            {
                if (m.IsLive(i))
                {
                    live.Add(i);
                }
            }

            if (op < 4 || live.Count < 4)
            {
                ctx.Write(m.Add(Child(rng, hulls)).Index);
            }
            else if (op < 7)
            {
                int id = live[rng.Int(live.Count)];
                m.Remove(m.GetHandle(id));
                ctx.Write(-id - 1);
            }
            else
            {
                int id = live[rng.Int(live.Count)];
                m.Move(m.GetHandle(id), new RigidTransformF(rng.Rotation(), rng.Ball(4f)));
                ctx.Write(id);
            }

            if (step % 13 == 12)
            {
                m.Optimize();
            }

            WriteBvh(ctx, m);

            for (int k = 0; k < 8; k++)
            {
                Vector3F o = rng.Ball(8f);
                Vector3F d = rng.Ball(6f) - o;
                bool hit = m.RayCast(o, d, 1.5f, out CompoundRayHit3D r);
                ctx.Write(hit ? 1 : 0);
                if (hit)
                {
                    ctx.Write(r.Fraction);
                    Write(ctx, r.Point);
                    Write(ctx, r.Normal);
                    ctx.Write(r.Child);
                    ctx.Write(r.MaterialId);
                }

                ctx.Write(m.RayCastAny(o, d, 1.5f, out CompoundRayHit3D any) ? any.Child : -1);
            }

            for (int k = 0; k < 4; k++)
            {
                ConvexShape3D probe = Shape(rng, hulls);
                var pose = new RigidTransformF(rng.Rotation(), rng.Ball(6f));
                bool hit = m.ShapeCast(probe, pose, rng.Ball(8f), out CompoundCastHit3D c);
                ctx.Write(hit ? 1 : 0);
                if (hit)
                {
                    ctx.Write(c.Fraction);
                    Write(ctx, c.Normal);
                    Write(ctx, c.PointOnCast);
                    Write(ctx, c.PointOnChild);
                    ctx.Write(c.Child);
                    ctx.Write(c.StartsOverlapping ? 1 : 0);
                }

                CompoundPairs3D.FindChildren(m, probe, pose, 0.02f, ids);
                ctx.Write(ids.Count);
                foreach (int id in ids)
                {
                    ctx.Write(id);
                }

                Vector3F center = rng.Ball(5f);
                Vector3F half = new(rng.Range(0f, 2f), rng.Range(0f, 2f), rng.Range(0f, 2f));
                m.OverlapBounds(center - half, center + half, ids);
                ctx.Write(ids.Count);
                foreach (int id in ids)
                {
                    ctx.Write(id);
                }
            }

            // Child-pair enumeration and manifolds against a convex shape and against another compound, near enough to touch.
            var at = new RigidTransformF(rng.Rotation(), rng.Ball(4f));
            CompoundPairs3D.FindPairs(m, other, at, 0.02f, pairs);
            ctx.Write(pairs.Count);
            foreach (SubShapePair3D p in pairs)
            {
                ctx.Write(p.A);
                ctx.Write(p.B);
            }

            int count = gen.CollideCompoundCompound(m, other, at, contacts);
            ctx.Write(count);
            foreach (ChildContact3D c in contacts)
            {
                ctx.Write(c.SubShapeA);
                ctx.Write(c.SubShapeB);
                Write(ctx, c.Manifold);
            }

            ConvexShape3D convex = Shape(rng, hulls);
            var cp = new RigidTransformF(rng.Rotation(), rng.Ball(3f));
            ctx.Write(gen.CollideCompoundConvex(m, convex, cp, contacts));
            foreach (ChildContact3D c in contacts)
            {
                ctx.Write(c.SubShapeA);
                Write(ctx, c.Manifold);
            }

            ctx.Write(gen.CollideConvexCompound(convex, m, cp, contacts));
            foreach (ChildContact3D c in contacts)
            {
                ctx.Write(c.SubShapeB);
                Write(ctx, c.Manifold);
            }

            ctx.EndStep();
        }
    }
}
