using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

// Fuzz target for the 3D contact manifolds. The bytes describe two shapes, a pose and a speculative distance. Most values are ordinary, some are NaN, infinite, huge or tiny,
// so hostile input reaches the shape constructors and the pose check. Allowed outcomes: a manifold with finite numbers, or a typed argument error from a constructor or
// from the pose check. Anything else (another exception, a NaN, more than four points, a normal that is not a unit vector) is a finding.
internal static class Fuzz3D
{
    private static int s_pairs, s_manifolds, s_rejected;

    static Fuzz3D()
    {
        AppDomain.CurrentDomain.ProcessExit += (_, _) => Console.WriteLine("manifold3d: shape pairs built " + s_pairs + ", manifolds with points " + s_manifolds + ", typed rejections " + s_rejected);
    }

    private sealed class Reader
    {
        private readonly byte[] _d;
        private int _i;

        public Reader(byte[] d) { _d = d; }

        public byte Byte() => _i < _d.Length ? _d[_i++] : (byte)0;

        // A size: mostly 2^-20 .. 2^20, sometimes NaN, infinity, zero, negative or beyond the limit.
        public float Dim()
        {
            byte k = Byte();
            switch (k)
            {
                case 250: return float.NaN;
                case 251: return float.PositiveInfinity;
                case 252: return 0f;
                case 253: return -1f;
                case 254: return 3e6f;
                case 255: return BitConverter.Int32BitsToSingle(Byte() | (Byte() << 8) | (Byte() << 16) | (Byte() << 24));
            }

            float frac = 1f + (Byte() / 64f);
            return frac * MathF.ScaleB(1f, (k % 41) - 20);
        }

        // A signed coordinate: a small number scaled by a power of two.
        public float Coord(float scale) => (sbyte)Byte() / 16f * scale;
    }

    public static void ManifoldTarget(byte[] data)
    {
        var r = new Reader(data);
        ConvexShape3D a, b;
        try
        {
            a = Shape(r);
            b = Shape(r);
        }
        catch (ArgumentOutOfRangeException)
        {
            s_rejected++;
            return;
        }
        catch (HullBuildException)
        {
            s_rejected++;
            return;
        }

        s_pairs++;
        float scale = MathF.ScaleB(1f, (r.Byte() % 24) - 8);
        var q = new QuaternionF(r.Coord(1f), r.Coord(1f), r.Coord(1f), r.Coord(1f));
        float ql = q.LengthSquared();
        q = ql > 1e-6f ? q.Normalized() : QuaternionF.Identity;
        byte mode = r.Byte();
        if ((mode & 0x7) == 0)
        {
            q = new QuaternionF(r.Coord(1f), r.Coord(1f), r.Coord(1f), r.Coord(1f)); // not normalised: must be rejected or survived
        }

        var t = new Vector3F(r.Coord(scale), r.Coord(scale), r.Coord(scale));
        if ((mode & 0x38) == 0x38)
        {
            t = new Vector3F(r.Dim(), r.Dim(), r.Dim());
        }

        float margin = (r.Byte() / 255f) * 0.5f;
        if ((mode & 0xC0) == 0xC0)
        {
            margin = r.Dim();
        }

        var gen = new ManifoldGenerator3D();
        Manifold3D m;
        try
        {
            m = gen.Collide(a, b, new RigidTransformF(q, t), margin);
        }
        catch (ArgumentOutOfRangeException)
        {
            s_rejected++;
            return;
        }

        Check(m, a, b);
        if (m.PointCount > 0)
        {
            s_manifolds++;
        }
    }

    private static void Check(Manifold3D m, ConvexShape3D a, ConvexShape3D b)
    {
        string what = a.Kind + "/" + b.Kind;
        if (m.PointCount < 0 || m.PointCount > 4)
        {
            throw new InvalidOperationException("bad point count " + m.PointCount + " for " + what);
        }

        if (m.PointCount == 0)
        {
            return;
        }

        float nl = m.Normal.LengthSquared();
        // A pose rotation may be off by 1% in squared length (documented), and the normal is rotated by it, so its squared length can be off by about 2% plus rounding.
        if (!(nl > 0.97f && nl < 1.03f))
        {
            throw new InvalidOperationException("normal " + m.Normal + " is not a unit vector for " + what);
        }

        for (int i = 0; i < m.PointCount; i++)
        {
            ManifoldPoint3D p = m[i];
            if (!float.IsFinite(p.Point.X + p.Point.Y + p.Point.Z + p.Separation))
            {
                throw new InvalidOperationException("non-finite point " + p.Point + " separation " + p.Separation + " for " + what);
            }
        }
    }

    private static ConvexShape3D Shape(Reader r)
    {
        switch (r.Byte() % 7)
        {
            case 0: return ConvexShape3D.Sphere(r.Dim());
            case 1:
                {
                    var he = new Vector3F(r.Dim(), r.Dim(), r.Dim());
                    return ConvexShape3D.Box(he, r.Byte() < 128 ? 0f : Math.Min(he.X, Math.Min(he.Y, he.Z)) * (r.Byte() / 255f));
                }

            case 2: return ConvexShape3D.Capsule(r.Dim(), r.Dim());
            case 3: return ConvexShape3D.TaperedCapsule(r.Dim(), r.Dim(), r.Dim());
            case 4:
                {
                    float hh = r.Dim(), rad = r.Dim();
                    return ConvexShape3D.Cylinder(hh, rad, r.Byte() < 128 ? 0f : Math.Min(hh, rad) * (r.Byte() / 255f));
                }

            case 5:
                {
                    // A hull of a few lattice points; degenerate sets are rejected by the builder.
                    int n = 4 + (r.Byte() % 12);
                    float s = MathF.ScaleB(1f, (r.Byte() % 24) - 12);
                    var pts = new List<Vector3F>(n);
                    for (int i = 0; i < n; i++)
                    {
                        pts.Add(new Vector3F(r.Coord(s), r.Coord(s), r.Coord(s)));
                    }

                    return ConvexShape3D.FromHull(ConvexHull3D.Build(pts.ToArray()), r.Byte() < 200 ? 0f : s * (r.Byte() / 255f));
                }

            default:
                {
                    // A box-like hull from eight corners with a little noise, to get the common near-axis-aligned topology.
                    float x = r.Dim(), y = r.Dim(), z = r.Dim();
                    var pts = new Vector3F[8];
                    for (int i = 0; i < 8; i++)
                    {
                        pts[i] = new Vector3F((i & 1) == 0 ? -x : x, (i & 2) == 0 ? -y : y, (i & 4) == 0 ? -z : z);
                    }

                    return ConvexShape3D.FromHull(ConvexHull3D.Build(pts));
                }
        }
    }
}
