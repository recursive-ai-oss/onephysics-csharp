using System;

namespace RecursiveAI.OnePhysics.Collision2D.Reference;

/// <summary>
/// A fixed scenario over the whole 2D collision API, reduced to one 64-bit hash: random shapes (some of them deliberately invalid), manifolds, distances, ray and shape casts,
/// times of impact and mass data, all from a seeded generator written with exact arithmetic only. The hash is the same on every platform and runtime;
/// it is used by the tests and the AOT and WebAssembly smoke apps to check exactly that.
/// </summary>
public static class CollisionHash2D
{
    private struct Rng
    {
        private ulong _s;

        public Rng(ulong seed) { _s = seed == 0 ? 0x9E3779B97F4A7C15UL : seed; }

        public uint Next()
        {
            _s ^= _s >> 12;
            _s ^= _s << 25;
            _s ^= _s >> 27;
            return (uint)((_s * 0x2545F4914F6CDD1DUL) >> 32);
        }

        // Uniform in [lo, hi) with 24 random bits: exactly representable steps, so the same value everywhere.
        public float Range(float lo, float hi) => lo + (hi - lo) * ((Next() >> 8) * (1f / 16777216f));

        public int Int(int n) => (int)(Next() % (uint)n);
    }

    private struct Hasher
    {
        public ulong H;

        public void Add(uint v)
        {
            H = (H ^ v) * 0x100000001B3UL;
        }

        public void Add(float f) => Add(BitConverter.SingleToUInt32Bits(f));

        public void Add(double d)
        {
            ulong b = BitConverter.DoubleToUInt64Bits(d);
            Add((uint)b);
            Add((uint)(b >> 32));
        }

        public void Add(Vec2 v)
        {
            Add(v.X);
            Add(v.Y);
        }
    }

    private static Shape2D? RandomShape(ref Rng r)
    {
        try
        {
            switch (r.Int(7))
            {
                case 0: return Shape2D.CreateCircle(new Vec2(r.Range(-1f, 1f), r.Range(-1f, 1f)), r.Range(0.05f, 1.2f));
                case 1: return Shape2D.CreateCapsule(new Vec2(r.Range(-1f, 1f), r.Range(-1f, 1f)), new Vec2(r.Range(-1f, 1f), r.Range(-1f, 1f)), r.Range(0.05f, 0.8f));
                case 2: return Shape2D.CreateSegment(new Vec2(r.Range(-1.5f, 1.5f), r.Range(-1.5f, 1.5f)), new Vec2(r.Range(-1.5f, 1.5f), r.Range(-1.5f, 1.5f)));
                case 3: return Shape2D.CreateBox(r.Range(0.1f, 1.2f), r.Range(0.1f, 1.2f), new Vec2(r.Range(-0.5f, 0.5f), r.Range(-0.5f, 0.5f)), Rot2.FromVector(r.Range(-1f, 1f), r.Range(-1f, 1f)), r.Int(3) == 0 ? r.Range(0f, 0.3f) : 0f);
                case 4:
                {
                    var pts = new Vec2[3 + r.Int(10)];
                    for (int i = 0; i < pts.Length; i++) { pts[i] = new Vec2(r.Range(-1.2f, 1.2f), r.Range(-1.2f, 1.2f)); }
                    return Shape2D.CreatePolygon(pts, r.Int(3) == 0 ? r.Range(0f, 0.3f) : 0f);
                }

                case 5:
                {
                    // Points on a circle: a hull of up to 8 points.
                    int n = 3 + r.Int(6);
                    var pts = new Vec2[n];
                    for (int i = 0; i < n; i++)
                    {
                        float a = r.Range(0f, 1f);
                        // A parametrisation of the unit circle by a rational function: no trigonometry needed.
                        float t = a * 2f - 1f;
                        float den = 1f + t * t;
                        float x = (1f - t * t) / den, y = 2f * t / den;
                        if (r.Int(2) == 0) { x = -x; }
                        pts[i] = new Vec2(x * 1.1f, y * 0.8f);
                    }

                    return Shape2D.CreatePolygon(pts);
                }

                default:
                    return Shape2D.CreateChainSegment(r.Int(2) == 0 ? null : new Vec2(r.Range(-2f, -1.6f), r.Range(-0.3f, 0.3f)), new Vec2(-1.5f, r.Range(-0.3f, 0.3f)), new Vec2(1.5f, r.Range(-0.3f, 0.3f)), r.Int(2) == 0 ? null : new Vec2(r.Range(1.6f, 2f), r.Range(-0.3f, 0.3f)), r.Int(2) == 0);
            }
        }
        catch (InvalidShapeException)
        {
            return null;
        }
    }

    /// <summary>Runs the scenario and returns its hash. Takes a few hundred milliseconds.</summary>
    public static ulong Compute()
    {
        var r = new Rng(0x0DDC0FFEE1234567UL);
        var h = new Hasher { H = 0xCBF29CE484222325UL };
        var shapes = new Shape2D[160];
        int made = 0;
        while (made < shapes.Length)
        {
            Shape2D? s = RandomShape(ref r);
            if (s is Shape2D v) { shapes[made++] = v; }
            else { h.Add(0xBAD0u); }
        }

        // Hostile polygon inputs are rejected with the same reason everywhere.
        var hostile = new Vec2[][]
        {
            new[] { new Vec2(0, 0), new Vec2(1, 1) },
            new[] { new Vec2(0, 0), new Vec2(1, 1), new Vec2(2, 2), new Vec2(3, 3) },
            new[] { new Vec2(0, 0), new Vec2(float.NaN, 1), new Vec2(2, 2) },
            new[] { new Vec2(0, 0), new Vec2(float.PositiveInfinity, 1), new Vec2(2, 2) },
            new[] { new Vec2(0, 0), new Vec2(1e9f, 1), new Vec2(2, 2) },
            new[] { new Vec2(0, 0), new Vec2(1e-9f, 0), new Vec2(0, 1e-9f) },
            new[] { new Vec2(1, 1), new Vec2(1, 1), new Vec2(1, 1) },
        };
        foreach (Vec2[] pts in hostile)
        {
            try { Shape2D.CreatePolygon(pts); h.Add(0u); }
            catch (InvalidShapeException e) { h.Add((uint)e.Reason + 1u); }
        }

        for (int i = 0; i < shapes.Length; i++)
        {
            MassData2D m = shapes[i].ComputeMass();
            h.Add(m.Area);
            h.Add(m.CentroidX);
            h.Add(m.CentroidY);
            h.Add(m.InertiaAtCentroid);
            Aabb2 bb = shapes[i].ComputeAabb();
            h.Add(bb.Lower);
            h.Add(bb.Upper);
        }

        for (int k = 0; k < 4000; k++)
        {
            ref readonly Shape2D a = ref shapes[r.Int(shapes.Length)];
            ref readonly Shape2D b = ref shapes[r.Int(shapes.Length)];
            Rot2 q = Rot2.FromVector(r.Range(-1f, 1f), r.Range(-1f, 1f));
            var xf = new Transform2(new Vec2(r.Range(-3f, 3f), r.Range(-3f, 3f)), q);

            Manifold2D m = Collide2D.Collide(in a, in b, xf);
            h.Add(m.Normal);
            h.Add((uint)m.PointCount);
            for (int p = 0; p < m.PointCount; p++)
            {
                h.Add(m[p].Point);
                h.Add(m[p].Separation);
                h.Add(m[p].Id);
            }

            var cache = new SimplexCache2D();
            DistanceOutput2D d = Distance2D.Compute(in a, in b, xf, ref cache);
            h.Add(d.PointA);
            h.Add(d.PointB);
            h.Add(d.Distance);
            h.Add((uint)d.Iterations);

            Vec2 o = new Vec2(r.Range(-3f, 3f), r.Range(-3f, 3f));
            Vec2 dir = new Vec2(r.Range(-4f, 4f), r.Range(-4f, 4f));
            RayCastOutput2D rc = a.RayCast(o, dir);
            h.Add(rc.Hit ? 1u : 0u);
            h.Add(rc.Fraction);
            h.Add(rc.Normal);
            h.Add(rc.Point);

            if (k % 4 == 0)
            {
                RayCastOutput2D sc = Distance2D.ShapeCast(in a, in b, xf, dir);
                h.Add(sc.Hit ? 1u : 0u);
                h.Add(sc.Fraction);
                h.Add(sc.Normal);
                h.Add(sc.Point);

                var sa = new Sweep2D(default, default, new Vec2(r.Range(-1f, 1f), r.Range(-1f, 1f)), Rot2.Identity, Rot2.FromVector(1f, r.Range(-0.5f, 0.5f)));
                var sb = new Sweep2D(default, xf.P, xf.P + dir, xf.Q, Rot2.Mul(xf.Q, Rot2.FromVector(1f, r.Range(-0.5f, 0.5f))));
                ToiOutput2D toi = Distance2D.TimeOfImpact(in a, in sa, in b, in sb);
                h.Add((uint)toi.State);
                h.Add(toi.Fraction);
            }
        }

        return h.H;
    }
}
