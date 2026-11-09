using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics2D;

// Fuzz targets for 2D soft bodies.
// "world2d-softbodies" turns bytes into a program: hostile meshes (NaN and infinite coordinates, repeated, negative and out-of-range indices, clockwise, degenerate and self-touching triangles and outlines, huge counts),
// hostile definitions (compliances, damping, pressure, radius, mass, pins, cluster sizes), stale handles, impulses, teleports, targets and driven poses, stepping, origin shifts, overlap sensors and snapshot round trips.
// Only argument errors and typed physics errors are allowed.
// "world2d-softbody-snapshot" flips bytes in the snapshot of a world with soft bodies: rejected with the world unchanged, or accepted and re-serialized to the same bytes.
internal static class FuzzSoftBodies2D
{
    private static int s_meshes, s_refused, s_steps, s_accepted, s_rejected;
    private static byte[]? s_base;

    static FuzzSoftBodies2D()
    {
        AppDomain.CurrentDomain.ProcessExit += (_, _) => Console.WriteLine("2d soft-body targets: meshes " + s_meshes + ", refused " + s_refused + ", steps " + s_steps + "; snapshots accepted " + s_accepted + ", rejected " + s_rejected);
    }

    private sealed class Reader
    {
        private readonly byte[] _d;
        private int _i;

        public Reader(byte[] d) { _d = d; }

        public bool More => _i < _d.Length;

        public byte Byte() => _i < _d.Length ? _d[_i++] : (byte)0;

        public float Float()
        {
            byte b = Byte();
            switch (b)
            {
                case 250: return float.NaN;
                case 251: return float.PositiveInfinity;
                case 252: return float.NegativeInfinity;
                case 253: return 3e6f;
                case 254: return 1e-9f;
                case 255: return -0f;
            }

            int v = (short)(b | (Byte() << 8));
            return v / 256f;
        }

        public double Double()
        {
            byte b = Byte();
            switch (b)
            {
                case 250: return double.NaN;
                case 251: return double.PositiveInfinity;
                case 252: return double.NegativeInfinity;
                case 253: return 1e30;
                case 254: return 4194305.0;
            }

            return (short)(b | (Byte() << 8)) / 64.0;
        }

        public Vector2F V2F() => new(Float(), Float());

        public bool Bool() => (Byte() & 1) != 0;

        // Mostly valid indices, sometimes hostile ones.
        public int Index(int count)
        {
            byte b = Byte();
            if (b >= 248) { return b == 248 ? -1 : b == 249 ? count : b == 250 ? int.MaxValue : int.MinValue; }
            return count == 0 ? 0 : b % count;
        }
    }

    private static bool Allowed(Exception e) => e is ArgumentException || e is PhysicsException;

    private static void StepTolerant(PhysicsWorld2D w)
    {
        try { w.StepChecked(1f / 60f); s_steps++; }
        catch (BodyOutOfBoundsException e) { e.Body.Destroy(); }
        catch (SoftBodyOutOfBoundsException e) { e.SoftBody.Destroy(); }
    }

    private static PhysicsWorld2D NewWorld() => new(new WorldOptions2D { WorldId = 12 });

    private static SoftBodyMesh Mesh(Reader r)
    {
        s_meshes++;
        switch (r.Byte() % 7)
        {
            case 0: return SoftBodyMesh.Box(new Vector2F(Math.Abs(r.Float()) % 3f, Math.Abs(r.Float()) % 3f), 1 + r.Byte() % 8, 1 + r.Byte() % 8);
            case 1: return SoftBodyMesh.Disc(Math.Abs(r.Float()) % 3f, 1 + r.Byte() % 6, 3 + r.Byte() % 24);
            case 2: return SoftBodyMesh.Ring(Math.Abs(r.Float()) % 3f, 3 + r.Byte() % 30);
            case 3:
                {
                    int nv = 3 + r.Byte() % 10, nt = 1 + r.Byte() % 12;
                    var v = new Vector2F[nv];
                    for (int i = 0; i < nv; i++) { v[i] = r.V2F(); }
                    var idx = new int[3 * nt];
                    for (int i = 0; i < idx.Length; i++) { idx[i] = r.Index(nv); }
                    return SoftBodyMesh.FromTriangles(v, idx);
                }
            case 4:
                {
                    int nv = 3 + r.Byte() % 12;
                    var v = new Vector2F[nv];
                    for (int i = 0; i < nv; i++) { v[i] = r.V2F(); }
                    return SoftBodyMesh.FromOutline(v);
                }
            case 5:
                {
                    int nv = 3 + r.Byte() % 12;
                    var v = new Vector2F[nv];
                    for (int i = 0; i < nv; i++) { v[i] = r.V2F(); }
                    return SoftBodyMesh.FromPolygon(v);
                }
            default:
                return SoftBodyMesh.FromTriangles(new[] { new Vector2F(0, 0), new Vector2F(1, 0), new Vector2F(0, 1) }, new[] { 0, 1, 2 });
        }
    }

    private static SoftBodyDef Def(Reader r)
    {
        var d = new SoftBodyDef
        {
            Mesh = r.Byte() >= 250 ? null : Mesh(r),
            Position = new Vector2D(r.Double() % 30.0, r.Double() % 30.0 + 2.0),
            Angle = r.Byte() >= 240 ? r.Float() : 0f,
            Scale = r.Byte() >= 240 ? r.Float() : 0.5f + (r.Byte() / 128f),
            Mass = r.Byte() >= 240 ? r.Float() : 0.5f + (r.Byte() / 16f),
            ParticleRadius = r.Byte() >= 240 ? r.Float() : 0f,
            Model = (SoftBodyModel)(r.Byte() >= 250 ? r.Byte() : r.Byte() % 3),
            EdgeCompliance = r.Byte() >= 240 ? r.Float() : 0f,
            BendCompliance = r.Byte() >= 240 ? r.Float() : 1e-3f,
            AreaCompliance = r.Byte() >= 240 ? r.Float() : 0f,
            InternalPressure = r.Byte() >= 240 ? r.Float() : (r.Bool() ? 1f : 0f),
            MatchCompliance = r.Byte() >= 240 ? r.Float() : 1e-3f,
            MatchQuadratic = r.Bool(),
            MatchBlend = r.Byte() >= 240 ? r.Float() : 0.5f,
            ClusterSize = r.Byte() >= 250 ? r.Byte() * 100000 - 5 : r.Byte() % 40,
            Friction = r.Byte() >= 240 ? r.Float() : 0.4f,
            LinearDamping = r.Byte() >= 240 ? r.Float() : 0.1f,
            SelfCollision = r.Bool(),
            LinearVelocity = r.Byte() >= 240 ? r.V2F() : default,
            AngularVelocity = r.Byte() >= 240 ? r.Float() : 0f,
        };
        int pins = r.Byte() % 4;
        if (pins > 0)
        {
            var list = new List<int>();
            for (int i = 0; i < pins; i++) { list.Add(r.Index(d.Mesh?.VertexCount ?? 8)); }
            d.PinnedParticles = list;
        }

        return d;
    }

    private static SoftBody Pick(List<SoftBody> list, Reader r)
    {
        byte b = r.Byte();
        if (list.Count == 0 || b >= 240) { return default; }
        return list[b % list.Count];
    }

    public static void Run(byte[] data)
    {
        var r = new Reader(data);
        var w = NewWorld();
        w.AddBox(new Vector2F(60f, 1f), new Vector2D(0, -0.5), type: BodyType.Static);
        w.AddCircle(1f, new Vector2D(0, 1), type: BodyType.Static);
        var softs = new List<SoftBody>();
        int ops = 4 + r.Byte() % 40;
        for (int n = 0; n < ops && r.More; n++)
        {
            try { Op(w, softs, r); }
            catch (Exception e) when (Allowed(e)) { s_refused++; }
        }

        for (int i = 0; i < 3; i++) { StepTolerant(w); }
        byte[] snap = w.Snapshot();
        var w2 = NewWorld();
        try { w2.Restore(snap); }
        catch (SnapshotFormatException e) when (e.Message.Contains("out of range", StringComparison.Ordinal) || e.Message.Contains("non-finite", StringComparison.Ordinal)) { return; }
        if (w2.ComputeHash() != w.ComputeHash()) { throw new InvalidOperationException("restored world hash differs"); }
        if (!w2.Snapshot().AsSpan().SequenceEqual(snap)) { throw new InvalidOperationException("restored world re-serializes differently"); }
        if (w2.SoftBodyCount != w.SoftBodyCount) { throw new InvalidOperationException("restored world has a different soft body count"); }
        StepTolerant(w); StepTolerant(w2);
        if (w2.ComputeHash() != w.ComputeHash()) { throw new InvalidOperationException("restored world diverged after a step"); }
    }

    private static void Op(PhysicsWorld2D w, List<SoftBody> softs, Reader r)
    {
        switch (r.Byte() % 15)
        {
            case 0:
            case 1: { softs.Add(w.AddSoftBody(Def(r))); break; }
            case 2: { softs.Add(w.AddSoftBox(new Vector2F(Math.Abs(r.Float()) % 3f, Math.Abs(r.Float()) % 3f), new Vector2D(r.Float() % 10f, 2 + r.Float() % 5f), (SoftMaterial)(r.Byte() % 4), r.Byte() % 8)); break; }
            case 3: { softs.Add(w.AddBalloon(Math.Abs(r.Float()) % 2f, new Vector2D(r.Float() % 10f, 2 + r.Float() % 5f), r.Byte() % 40)); break; }
            case 4: { int n = 1 + r.Byte() % 6; for (int i = 0; i < n; i++) { StepTolerant(w); } break; }
            case 5: { var s = Pick(softs, r); s.SetPinned(r.Index(s.IsValid ? s.ParticleCount : 4), r.Bool()); break; }
            case 6: { var s = Pick(softs, r); s.ApplyImpulse(r.Index(s.IsValid ? s.ParticleCount : 4), r.V2F()); break; }
            case 7: { var s = Pick(softs, r); s.SetPosition(r.Index(s.IsValid ? s.ParticleCount : 4), new Vector2D(r.Double(), r.Double())); break; }
            case 8: { var s = Pick(softs, r); s.Destroy(); break; }
            case 9:
                {
                    var s = Pick(softs, r);
                    switch (r.Byte() % 6)
                    {
                        case 0: s.EdgeCompliance = r.Float(); break;
                        case 1: s.BendCompliance = r.Float(); break;
                        case 2: s.AreaCompliance = r.Float(); break;
                        case 3: s.InternalPressure = r.Float(); break;
                        case 4: s.MatchCompliance = r.Float(); break;
                        default: s.GravityScale = r.Float(); break;
                    }

                    break;
                }
            case 10:
                {
                    // Offsets on the 2^-31 grid, small enough to keep bodies in range; huge or off-grid ones must be refused with an argument error.
                    double Off() => r.Byte() >= 250 ? r.Double() : (sbyte)r.Byte() * 0.5;
                    w.ShiftOrigin(new Vector2D(Off(), Off()));
                    break;
                }
            case 11:
                {
                    var s = Pick(softs, r);
                    s.SetTarget(r.Index(s.IsValid ? s.ParticleCount : 4), new Vector2D(r.Double() % 8.0, r.Double() % 8.0 + 1.0), r.Float());
                    break;
                }
            case 12:
                {
                    var s = Pick(softs, r);
                    s.SetTargetPose(new Vector2D(r.Double() % 8.0, r.Double() % 8.0 + 1.0), r.Float(), r.Byte() >= 200 ? r.Float() : 0.1f, r.Byte() >= 200);
                    if (r.Bool()) { s.ClearTarget(r.Index(s.IsValid ? s.ParticleCount : 4)); }
                    break;
                }
            case 13:
                {
                    var hits = new SoftBody[3];
                    var box = Shape2D.CreateBox(Math.Abs(r.Float()) % 4f + 0.01f, Math.Abs(r.Float()) % 4f + 0.01f, default, Rot2.Identity, 0f);
                    _ = w.OverlapSoftBodies(in box, new Vector2D(r.Double() % 8.0, r.Double() % 8.0), r.Float(), hits);
                    break;
                }
            default:
                {
                    var s = Pick(softs, r);
                    s.AddVelocity(r.V2F());
                    _ = s.Area; _ = s.CenterOfMass; _ = s.Bounds; _ = s.Momentum;
                    break;
                }
        }
    }

    private static byte[] Base()
    {
        if (s_base != null) { return s_base; }
        var w = NewWorld();
        w.AddBox(new Vector2F(60f, 1f), new Vector2D(0, -0.5), type: BodyType.Static);
        w.AddSoftBox(new Vector2F(0.6f, 0.6f), new Vector2D(0, 1.0), SoftMaterial.Jelly, 2);
        w.AddBalloon(0.4f, new Vector2D(1.6, 1.0), 12);
        w.AddBlob(0.4f, new Vector2D(-1.6, 1.0), 2);
        var driven = w.AddSoftCircle(0.3f, new Vector2D(3, 1.0), SoftMaterial.Rubber, 2);
        driven.SetTarget(0, new Vector2D(3.5, 2), 0.1f);
        var def = SoftBodyDef.Jelly(SoftBodyMesh.Box(new Vector2F(0.5f, 0.5f), 2, 2), new Vector2D(5, 2));
        def.PinnedParticles = new[] { 6, 7 };
        w.AddSoftBody(def);
        w.AddBox(new Vector2F(0.4f, 0.4f), new Vector2D(1.6, 2.4));
        for (int s = 0; s < 30; s++) { w.StepChecked(1f / 60f); }
        return s_base = w.Snapshot();
    }

    public static void SnapshotTarget(byte[] data)
    {
        var bytes = (byte[])Base().Clone();
        int mutations = data.Length > 0 ? 1 + data[0] % 5 : 0;
        // The soft-body section is near the end of the snapshot: aim most flips at the last 4000 bytes.
        for (int i = 1, m = 0; i + 2 < data.Length && m < mutations; i += 3, m++)
        {
            int span = (data[0] & 0x10) != 0 ? bytes.Length : Math.Min(4000, bytes.Length);
            int at = bytes.Length - 1 - (((data[i] << 8) | data[i + 1]) % span);
            bytes[at] ^= data[i + 2];
        }

        if (data.Length > 0 && (data[0] & 0x0F) == 0x0F) { Array.Resize(ref bytes, (data[^1] * bytes.Length) / 256); }

        var w = NewWorld();
        byte[] empty = w.Snapshot();
        try { w.Restore(bytes); }
        catch (SnapshotFormatException)
        {
            s_rejected++;
            if (!w.Snapshot().AsSpan().SequenceEqual(empty)) { throw new InvalidOperationException("a rejected snapshot changed the world"); }
            return;
        }

        s_accepted++;
        if (!w.Snapshot().AsSpan().SequenceEqual(bytes)) { throw new InvalidOperationException("accepted snapshot does not re-serialize to the same bytes"); }
        StepTolerant(w);
        StepTolerant(w);
    }
}
