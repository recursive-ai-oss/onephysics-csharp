using System;
using System.Collections.Generic;
using System.IO;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Collision3D;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Core.Reference;

// Usage: Fuzz <target> <file>...   runs the target once per file (a crash or an unexpected exception is a finding)
//        Fuzz <target> --random <seed> <count> [bytes]   runs it over a seeded pseudo-random byte stream (default 64 bytes per input)
internal static class Program
{
    private static readonly Dictionary<string, Action<byte[]>> Targets = new(StringComparer.Ordinal)
    {
        ["mesh"] = FuzzMesh.MeshTarget,
        ["heightfield"] = FuzzMesh.HeightFieldTarget,
        ["meshquery"] = FuzzMesh.QueryTarget,
        ["snapshot"] = SnapshotTarget,
        ["snapshot-raw"] = SnapshotRawTarget,
        ["hull"] = HullTarget,
        ["convex"] = ConvexTarget,
        ["polygon2d"] = Fuzz2D.PolygonTarget,
        ["manifold2d"] = Fuzz2D.ManifoldTarget,
        ["broadphase"] = BroadphaseTarget.Run,
        ["manifold3d"] = Fuzz3D.ManifoldTarget,
        ["world2d-snapshot"] = Fuzz2DWorld.SnapshotTarget,
        ["world2d-defs"] = Fuzz2DWorld.DefsTarget,
        ["compound3d"] = FuzzCompound3D.Run,
    };

    private static int Main(string[] args)
    {
        if (args.Length < 2 || !Targets.TryGetValue(args[0], out var target))
        {
            Console.Error.WriteLine("usage: Fuzz <target> <file>... | Fuzz <target> --random <seed> <count>; targets: " + string.Join(", ", Targets.Keys));
            return 2;
        }

        if (args[1] == "--random" && (args.Length == 4 || args.Length == 5))
        {
            var rng = new Random(int.Parse(args[2], System.Globalization.CultureInfo.InvariantCulture));
            int count = int.Parse(args[3], System.Globalization.CultureInfo.InvariantCulture);
            var buffer = new byte[args.Length == 5 ? int.Parse(args[4], System.Globalization.CultureInfo.InvariantCulture) : 64];
            for (int i = 0; i < count; i++)
            {
                rng.NextBytes(buffer);
                target(buffer);
            }

            Console.WriteLine("ran " + count + " (restored and stepped: " + s_accepted + ")");
            return 0;
        }

        for (int i = 1; i < args.Length; i++)
        {
            target(File.ReadAllBytes(args[i]));
        }

        Console.WriteLine("ran " + (args.Length - 1));
        return 0;
    }

    // ---- 3D convex collision targets -------------------------------------------------------------------------------

    // Turns bytes into points in one of several styles: raw float bits (mostly hostile), a coarse lattice (many coplanar, collinear and duplicate points), lattices at tiny and huge scales, and noisy lattices.
    private static Vector3F[] DecodePoints(byte[] data, out HullBuildOptions options)
    {
        options = default;
        if (data.Length < 4)
        {
            return Array.Empty<Vector3F>();
        }

        int mode = data[0] & 3;
        int flags = data[0] >> 2;
        if ((flags & 1) != 0)
        {
            options = new HullBuildOptions { MaxVertices = 4 + (data[1] & 31), MaxFaces = (flags & 2) != 0 ? 4 + (data[2] & 63) : 0 };
        }

        double scale = mode == 2 ? Math.Pow(2.0, (data[3] % 120) - 60) : 1.0;
        int stride = mode == 0 ? 12 : 3;
        int count = (data.Length - 4) / stride;
        var pts = new Vector3F[count];
        for (int i = 0; i < count; i++)
        {
            int o = 4 + (i * stride);
            switch (mode)
            {
                case 0:
                    pts[i] = new Vector3F(BitConverter.ToSingle(data, o), BitConverter.ToSingle(data, o + 4), BitConverter.ToSingle(data, o + 8));
                    break;
                case 1:
                case 2:
                    pts[i] = new Vector3F((float)(((sbyte)data[o] / 16.0) * scale), (float)(((sbyte)data[o + 1] / 16.0) * scale), (float)(((sbyte)data[o + 2] / 16.0) * scale));
                    break;
                default:
                    pts[i] = new Vector3F(((data[o] - 128) / 64f) + ((data[o + 1] & 3) * 1e-6f), ((data[o + 1] >> 2) - 32) / 32f, ((data[o + 2] - 128) / 64f) + ((data[o + 2] & 1) * 1e-7f));
                    break;
            }
        }

        return pts;
    }

    // Any bytes either build a well-formed hull or raise HullBuildException; nothing else may escape, and a built hull must be closed and contain its input within tolerance.
    private static void HullTarget(byte[] data)
    {
        Vector3F[] pts = DecodePoints(data, out HullBuildOptions options);
        ConvexHull3D hull;
        try
        {
            hull = ConvexHull3D.Build(pts, options);
        }
        catch (HullBuildException)
        {
            return;
        }

        int v = hull.Vertices.Length, f = hull.Faces.Length, e = hull.Edges.Length;
        if (v < 4 || f < 4 || (e & 1) != 0 || v - (e / 2) + f != 2)
        {
            throw new InvalidOperationException($"hull is not a closed polyhedron: V={v} E={e / 2} F={f}");
        }

        double extent = 0;
        foreach (Vector3F p in hull.Vertices)
        {
            if (!float.IsFinite(p.X) || !float.IsFinite(p.Y) || !float.IsFinite(p.Z))
            {
                throw new InvalidOperationException("non-finite hull vertex");
            }

            extent = Math.Max(extent, Math.Max(Math.Abs(p.X), Math.Max(Math.Abs(p.Y), Math.Abs(p.Z))));
        }

        for (int i = 0; i < f; i++)
        {
            HullFace3D face = hull.Faces[i];
            if (!float.IsFinite(face.Normal.X + face.Normal.Y + face.Normal.Z + face.Offset) || face.EdgeCount < 3)
            {
                throw new InvalidOperationException("bad face " + i);
            }

            if (options.MaxVertices == 0 && options.MaxFaces == 0)
            {
                foreach (Vector3F p in pts)
                {
                    double d = ((double)face.Normal.X * p.X) + ((double)face.Normal.Y * p.Y) + ((double)face.Normal.Z * p.Z) - face.Offset;
                    if (d > 1e-4 * Math.Max(extent, 1e-30))
                    {
                        throw new InvalidOperationException($"input point {p} is {d} outside face {i}");
                    }
                }
            }
        }

        for (int i = 0; i < e; i++)
        {
            HullEdge3D edge = hull.Edges[i];
            if ((uint)edge.Origin >= (uint)v || (uint)edge.Next >= (uint)e || (uint)edge.Face >= (uint)f)
            {
                throw new InvalidOperationException("edge index out of range");
            }
        }
    }

    private static float Param(byte b, float lo, float hi) => lo + ((hi - lo) * (b / 255f));

    private static ConvexShape3D DecodeShape(byte[] data, ref int pos)
    {
        byte kind = pos < data.Length ? data[pos++] : (byte)0;
        byte p0 = pos < data.Length ? data[pos++] : (byte)128, p1 = pos < data.Length ? data[pos++] : (byte)128, p2 = pos < data.Length ? data[pos++] : (byte)128, p3 = pos < data.Length ? data[pos++] : (byte)0;
        switch (kind % 6)
        {
            case 0: return ConvexShape3D.Sphere(Param(p0, 0.01f, 3f));
            case 1: return ConvexShape3D.Box(new Vector3F(Param(p0, 0.01f, 3f), Param(p1, 0.01f, 3f), Param(p2, 0.01f, 3f)), Param(p3, 0f, 1f) * Math.Min(Param(p0, 0.01f, 3f), Math.Min(Param(p1, 0.01f, 3f), Param(p2, 0.01f, 3f))));
            case 2: return ConvexShape3D.Capsule(Param(p0, 0.01f, 3f), Param(p1, 0.01f, 3f));
            case 3:
                {
                    float h = Param(p0, 0.05f, 3f);
                    float rt = Param(p1, 0.01f, 3f), rb = Param(p2, 0.01f, 3f);
                    if (Math.Abs(rt - rb) >= 2f * h)
                    {
                        rb = rt;
                    }

                    return ConvexShape3D.TaperedCapsule(h, rt, rb);
                }

            case 4: return ConvexShape3D.Cylinder(Param(p0, 0.01f, 3f), Param(p1, 0.01f, 3f), Param(p2, 0f, 1f) * Math.Min(Param(p0, 0.01f, 3f), Param(p1, 0.01f, 3f)));
            default:
                {
                    int n = 4 + (p0 & 15);
                    var pts = new Vector3F[n];
                    for (int i = 0; i < n; i++)
                    {
                        byte a = pos < data.Length ? data[pos++] : (byte)(i * 37), b = pos < data.Length ? data[pos++] : (byte)(i * 91), c = pos < data.Length ? data[pos++] : (byte)(i * 53);
                        pts[i] = new Vector3F((a - 128) / 64f, (b - 128) / 64f, (c - 128) / 64f);
                    }

                    try
                    {
                        return ConvexShape3D.FromHull(ConvexHull3D.Build(pts), Param(p3, 0f, 0.5f));
                    }
                    catch (HullBuildException)
                    {
                        return ConvexShape3D.Sphere(0.5f);
                    }
                }
        }
    }

    private static bool Sane(Vector3F v) => float.IsFinite(v.X) && float.IsFinite(v.Y) && float.IsFinite(v.Z);

    // Any pair of valid shapes in any placement must give finite, consistent answers from every query, in bounded time.
    private static void ConvexTarget(byte[] data)
    {
        int pos = 0;
        ConvexShape3D a = DecodeShape(data, ref pos), b = DecodeShape(data, ref pos);
        byte Next() => pos < data.Length ? data[pos++] : (byte)(pos++ * 29);
        var q = new QuaternionF(Param(Next(), -1f, 1f), Param(Next(), -1f, 1f), Param(Next(), -1f, 1f), Param(Next(), -1f, 1f));
        q = q.LengthSquared() > 1e-6f ? q.Normalized() : QuaternionF.Identity;
        var t = new Vector3F(Param(Next(), -4f, 4f), Param(Next(), -4f, 4f), Param(Next(), -4f, 4f));
        var xf = new RigidTransformF(q, t);
        var motion = new Vector3F(Param(Next(), -6f, 6f), Param(Next(), -6f, 6f), Param(Next(), -6f, 6f));

        ConvexDistance3D d = Gjk3D.Distance(a, b, xf);
        if (!(d.Distance >= 0f) || !float.IsFinite(d.Distance) || !Sane(d.Normal) || !Sane(d.PointA) || !Sane(d.PointB))
        {
            throw new InvalidOperationException($"bad distance result {d.Distance} for {a.Kind}/{b.Kind}");
        }

        var epa = new Epa3D();
        ConvexContact3D c = epa.Collide(a, b, xf);
        if (!float.IsFinite(c.Separation) || !Sane(c.Normal) || !Sane(c.PointA) || !Sane(c.PointB))
        {
            throw new InvalidOperationException($"bad contact result {c.Separation} for {a.Kind}/{b.Kind}");
        }

        float nl = c.Normal.Length();
        if (nl > 1.001f || (nl < 0.999f && nl != 0f))
        {
            throw new InvalidOperationException($"contact normal has length {nl}");
        }

        if (Gjk3D.ShapeCast(a, b, xf, motion, out ConvexCastHit3D h) && (!(h.Fraction >= 0f && h.Fraction <= 1f) || !Sane(h.Normal) || !Sane(h.PointA) || !Sane(h.PointB)))
        {
            throw new InvalidOperationException("bad shape cast hit " + h.Fraction);
        }

        Vector3F dir = motion.LengthSquared() > 0f ? motion.Normalized() : Vector3F.UnitX;
        if (Gjk3D.RayCast(b, xf, -dir * 5f, dir, 10f, out ConvexCastHit3D r) && (!(r.Fraction >= 0f && r.Fraction <= 10f) || !Sane(r.Normal) || !Sane(r.PointB)))
        {
            throw new InvalidOperationException("bad ray hit " + r.Fraction);
        }

        MassPropertiesCheck(a);
    }

    private static void MassPropertiesCheck(ConvexShape3D shape)
    {
        var mp = shape.GetMassProperties();
        if (!(mp.Mass > 0f) || !float.IsFinite(mp.Mass))
        {
            throw new InvalidOperationException("bad mass " + mp.Mass + " for " + shape.Kind);
        }
    }

    private static byte[]? s_base;
    private static int s_accepted;

    private static byte[] Base()
    {
        if (s_base == null)
        {
            var scene = ParticleScene.CreatePile(24, 5, worldId: 1);
            scene.ReportPersist = true;
            for (int i = 0; i < 20; i++) { scene.Step(1f / 60f); }
            scene.World.DestroyBody(scene.World.HandleOfBody(2));
            s_base = scene.World.Snapshot();
        }

        return s_base;
    }

    // Bytes of any shape go straight to Restore: the only allowed outcome besides success is a SnapshotFormatException.
    private static void SnapshotRawTarget(byte[] data)
    {
        var w = new WorldCore(new WorldOptions { WorldId = 1 });
        try { w.Restore(data); }
        catch (SnapshotFormatException) { }
    }

    // The input drives mutations of a valid snapshot (pairs of position and xor value, plus truncation), so the fuzzer gets past the header.
    // A snapshot that restores must be a usable world that steps without error and snapshots to the same bytes it was restored from.
    private static void SnapshotTarget(byte[] data)
    {
        var bytes = (byte[])Base().Clone();
        int mutations = data.Length > 0 ? 1 + data[0] % 3 : 0;
        for (int i = 1, m = 0; i + 2 < data.Length && m < mutations; i += 3, m++)
        {
            int at = ((data[i] << 8) | data[i + 1]) % bytes.Length;
            bytes[at] ^= data[i + 2];
        }

        if (data.Length > 0 && (data[0] & 0x0F) == 0x0F) { Array.Resize(ref bytes, (data[^1] * bytes.Length) / 256); }

        var scene = new ParticleScene(new WorldOptions { WorldId = 1 });
        try { scene.World.Restore(bytes); }
        catch (SnapshotFormatException) { return; }
        s_accepted++;
        byte[] again = scene.World.Snapshot();
        if (!again.AsSpan().SequenceEqual(bytes)) { throw new InvalidOperationException("accepted snapshot does not re-serialize to the same bytes"); }
        scene.Step(1f / 60f);
        scene.Step(1f / 60f);
    }
}
