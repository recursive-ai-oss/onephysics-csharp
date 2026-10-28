using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

/// <summary>
/// A fixed set of convex-collision inputs run through every query, with all output bits folded into one FNV-1a hash.
/// The hash must be the same on every platform, vector width and runtime; the unit tests, the NativeAOT smoke app and the WASM smoke app all check it.
/// </summary>
internal static class Collision3DHash
{
    /// <summary>The hash every platform must produce.</summary>
    public const ulong Expected = 12086356853580389106UL;

    [ThreadStatic]
    private static ulong _h;

    private static void Mix(float f) => MixBits((uint)BitConverter.SingleToInt32Bits(f));

    private static void MixBits(uint u)
    {
        // -0 and +0 are different bits; keep them different.
        for (int i = 0; i < 4; i++)
        {
            _h = (_h ^ ((u >> (8 * i)) & 0xFF)) * 1099511628211UL;
        }
    }

    private static void Mix(int v) => MixBits((uint)v);

    private static void Mix(Vector3F v)
    {
        Mix(v.X);
        Mix(v.Y);
        Mix(v.Z);
    }

    public static ConvexShape3D RandomShape(TestRng rng, ConvexHull3D[] hulls)
    {
        switch (rng.Next(7))
        {
            case 0: return ConvexShape3D.Sphere(rng.Range(0.2f, 1.2f));
            case 1: return ConvexShape3D.Box(new Vector3F(rng.Range(0.3f, 1.2f), rng.Range(0.3f, 1.2f), rng.Range(0.3f, 1.2f)));
            case 2:
                {
                    var he = new Vector3F(rng.Range(0.3f, 1.2f), rng.Range(0.3f, 1.2f), rng.Range(0.3f, 1.2f));
                    return ConvexShape3D.Box(he, rng.Range(0f, 0.3f));
                }

            case 3: return ConvexShape3D.Capsule(rng.Range(0.1f, 1f), rng.Range(0.1f, 0.6f));
            case 4:
                {
                    float h = rng.Range(0.3f, 1f);
                    return ConvexShape3D.TaperedCapsule(h, rng.Range(0.1f, 0.6f), rng.Range(0.1f, 0.6f));
                }

            case 5: return ConvexShape3D.Cylinder(rng.Range(0.2f, 1f), rng.Range(0.2f, 1f), rng.Range(0f, 0.15f));
            default: return ConvexShape3D.FromHull(hulls[rng.Next(hulls.Length)], rng.Next(3) == 0 ? rng.Range(0f, 0.2f) : 0f);
        }
    }

    public static ConvexHull3D[] RandomHulls(TestRng rng, int count)
    {
        var hulls = new ConvexHull3D[count];
        var pts = new Vector3F[64];
        for (int i = 0; i < count; i++)
        {
            int n = 6 + rng.Next(40);
            for (int k = 0; k < n; k++)
            {
                pts[k] = i % 2 == 0 ? rng.InBall(1f) : rng.OnSphere();
            }

            hulls[i] = ConvexHull3D.Build(pts.AsSpan(0, n));
        }

        return hulls;
    }

    public static ulong Compute()
    {
        _h = 14695981039346656037UL;
        var rng = new TestRng(12345);
        ConvexHull3D[] hulls = RandomHulls(rng, 24);
        foreach (ConvexHull3D hull in hulls)
        {
            Mix(hull.Vertices.Length);
            Mix(hull.Faces.Length);
            foreach (Vector3F v in hull.Vertices)
            {
                Mix(v);
            }

            foreach (HullFace3D f in hull.Faces)
            {
                Mix(f.Normal);
                Mix(f.Offset);
                Mix(f.EdgeCount);
            }

            Mix(hull.MassProperties.Mass);
            Mix(hull.MassProperties.Center);
            Mix(hull.MassProperties.Inertia.M11);
            Mix(hull.MassProperties.Inertia.M12);
            Mix(hull.MassProperties.Inertia.M33);
        }

        var epa = new Epa3D();
        for (int i = 0; i < 600; i++)
        {
            ConvexShape3D a = RandomShape(rng, hulls), b = RandomShape(rng, hulls);
            var xf = new RigidTransformF(rng.Rotation(), rng.InBall(rng.Range(0.2f, 3f)));
            ConvexDistance3D d = Gjk3D.Distance(a, b, xf);
            Mix(d.Distance);
            Mix(d.Overlapping ? 1 : 0);
            Mix(d.Normal);
            Mix(d.PointA);
            Mix(d.PointB);
            Mix(d.Iterations);
            ConvexContact3D c = epa.Collide(a, b, xf);
            Mix(c.Separation);
            Mix(c.Normal);
            Mix(c.PointA);
            Mix(c.PointB);
            Vector3F motion = rng.InBall(4f);
            if (Gjk3D.ShapeCast(a, b, xf, motion, out ConvexCastHit3D h))
            {
                Mix(1);
                Mix(h.Fraction);
                Mix(h.Normal);
                Mix(h.PointA);
                Mix(h.PointB);
            }
            else
            {
                Mix(0);
            }

            Vector3F origin = rng.InBall(3f);
            if (Gjk3D.RayCast(b, xf, origin, rng.OnSphere(), 6f, out ConvexCastHit3D r))
            {
                Mix(1);
                Mix(r.Fraction);
                Mix(r.Normal);
                Mix(r.PointB);
            }
            else
            {
                Mix(0);
            }

            var mp = a.GetMassProperties();
            Mix(mp.Mass);
            Mix(mp.Center);
            Mix(mp.Inertia.M11);
            Mix(mp.Inertia.M22);
        }

        return _h;
    }
}
