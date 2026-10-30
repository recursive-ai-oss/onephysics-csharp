using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics3D;

// Fuzz targets for 3D soft bodies.
// "world3d-softbodies" turns bytes into a program: hostile meshes (NaN and infinite coordinates, repeated, negative and out-of-range indices, degenerate and non-manifold triangles and tetrahedra, huge and tiny scales),
// hostile definitions (compliances, damping, pressure, radius, mass, pins), stale handles, impulses and teleports, stepping, origin shifts and snapshot round trips. Only argument errors and typed physics errors may come out.
// Skinning is in both: hostile skeletons, weights, limits and normals in the definition, hostile poses and teleports, and skinned bodies in the snapshot that is mutated.
// "world3d-softbody-snapshot" flips bytes in the snapshot of a world with soft bodies: rejected with the world unchanged, or accepted and re-serialized to the same bytes.
internal static class FuzzSoftBodies3D
{
    private static int s_meshes, s_refused, s_steps, s_accepted, s_rejected;
    private static byte[]? s_base;

    static FuzzSoftBodies3D()
    {
        AppDomain.CurrentDomain.ProcessExit += (_, _) => Console.WriteLine("soft-body targets: meshes " + s_meshes + ", refused " + s_refused + ", steps " + s_steps + "; snapshots accepted " + s_accepted + ", rejected " + s_rejected);
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

        public Vector3F V3F() => new(Float(), Float(), Float());

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

    private static void StepTolerant(PhysicsWorld3D w)
    {
        try { w.StepChecked(1f / 60f); s_steps++; }
        catch (BodyOutOfBoundsException e) { e.Body.Destroy(); }
        catch (SoftBodyOutOfBoundsException e) { e.SoftBody.Destroy(); }
    }

    private static PhysicsWorld3D NewWorld() => new(new WorldOptions3D { WorldId = 12 });

    private static SoftBodyMesh Mesh(Reader r)
    {
        s_meshes++;
        switch (r.Byte() % 6)
        {
            case 0: return SoftBodyMesh.Cloth(Math.Abs(r.Float()) % 4f, Math.Abs(r.Float()) % 4f, 1 + r.Byte() % 8, 1 + r.Byte() % 8);
            case 1: return SoftBodyMesh.Box(new Vector3F(Math.Abs(r.Float()) % 3f, Math.Abs(r.Float()) % 3f, Math.Abs(r.Float()) % 3f), 1 + r.Byte() % 4, 1 + r.Byte() % 4, 1 + r.Byte() % 4);
            case 2: return SoftBodyMesh.Sphere(Math.Abs(r.Float()) % 3f, 1 + r.Byte() % 6);
            case 3:
                {
                    int nv = 3 + r.Byte() % 10, nt = 1 + r.Byte() % 12;
                    var v = new Vector3F[nv];
                    for (int i = 0; i < nv; i++) { v[i] = r.V3F(); }
                    var idx = new int[3 * nt];
                    for (int i = 0; i < idx.Length; i++) { idx[i] = r.Index(nv); }
                    return SoftBodyMesh.FromTriangles(v, idx);
                }
            case 4:
                {
                    int nv = 4 + r.Byte() % 10, nt = 1 + r.Byte() % 10;
                    var v = new Vector3F[nv];
                    for (int i = 0; i < nv; i++) { v[i] = r.V3F(); }
                    var idx = new int[4 * nt];
                    for (int i = 0; i < idx.Length; i++) { idx[i] = r.Index(nv); }
                    return SoftBodyMesh.FromTetrahedra(v, idx);
                }
            default:
                return SoftBodyMesh.FromTetrahedra(new[] { new Vector3F(0, 0, 0), new Vector3F(1, 0, 0), new Vector3F(0, 1, 0), new Vector3F(0, 0, 1) }, new[] { 0, 1, 2, 3 });
        }
    }

    // A skin from hostile arrays: mostly sane, sometimes wrong lengths, joint numbers, weights, limits or normals. Argument errors are fine.
    private static SoftSkin? Skin(Reader r, SoftBodyMesh? mesh)
    {
        if (mesh == null || r.Byte() < 160) { return null; }
        int n = mesh.VertexCount, nj = 1 + r.Byte() % 4;
        if (r.Byte() >= 250) { nj = r.Byte() % 300; }
        var parents = new int[nj];
        for (int j = 0; j < nj; j++) { parents[j] = j == 0 ? -1 : (r.Byte() >= 245 ? r.Index(nj) : r.Byte() % j); }
        var bind = new Vector3F[r.Byte() >= 250 ? r.Byte() % 5 : nj];
        for (int j = 0; j < bind.Length; j++) { bind[j] = r.Byte() >= 240 ? r.V3F() : new Vector3F(0f, j * 0.3f, 0f); }
        int c = 4 * n + (r.Byte() >= 250 ? r.Byte() % 3 : 0);
        var idx = new int[c]; var w = new float[c];
        for (int i = 0; i < c; i++)
        {
            idx[i] = r.Byte() >= 245 ? r.Index(nj) : (r.Byte() % Math.Max(nj, 1));
            w[i] = r.Byte() >= 240 ? r.Float() : (i % 4 < 2 ? 0.5f : 0f);
        }

        var max = new float[n + (r.Byte() >= 250 ? 1 : 0)];
        for (int i = 0; i < max.Length; i++) { max[i] = r.Byte() >= 240 ? r.Float() : (i % 5 == 0 ? 0f : 0.05f); }
        float[] bd = Array.Empty<float>(), br = Array.Empty<float>();
        if (r.Bool())
        {
            bd = new float[n]; br = new float[n];
            for (int i = 0; i < n; i++) { bd[i] = r.Byte() >= 240 ? r.Float() : 0.04f; br[i] = r.Byte() >= 240 ? r.Float() : 0.03f; }
        }

        Vector3F[] nrm = Array.Empty<Vector3F>();
        if (r.Byte() >= 200)
        {
            nrm = new Vector3F[n];
            for (int i = 0; i < n; i++) { nrm[i] = r.Byte() >= 240 ? r.V3F() : new Vector3F(0f, 1f, 0f); }
        }

        return SoftSkin.Create(mesh, parents, bind, idx, w, max, bd, br, nrm);
    }

    private static SoftBodyDef Def(Reader r)
    {
        var d = new SoftBodyDef
        {
            Mesh = r.Byte() >= 250 ? null : Mesh(r),
            Position = new Vector3D(r.Double() % 30.0, r.Double() % 30.0 + 2.0, r.Double() % 30.0),
            Scale = r.Byte() >= 240 ? r.Float() : 0.5f + (r.Byte() / 128f),
            Mass = r.Byte() >= 240 ? r.Float() : 0.5f + (r.Byte() / 16f),
            ParticleRadius = r.Byte() >= 240 ? r.Float() : 0f,
            EdgeCompliance = r.Byte() >= 240 ? r.Float() : 0f,
            BendCompliance = r.Byte() >= 240 ? r.Float() : 1e-3f,
            VolumeCompliance = r.Byte() >= 240 ? r.Float() : 0f,
            InternalPressure = r.Byte() >= 240 ? r.Float() : 1f,
            Friction = r.Byte() >= 240 ? r.Float() : 0.4f,
            LinearDamping = r.Byte() >= 240 ? r.Float() : 0.1f,
            SelfCollision = r.Bool(),
            LinearVelocity = r.Byte() >= 240 ? r.V3F() : default,
        };
        d.Skin = Skin(r, d.Mesh);
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
        w.AddBox(new Vector3F(60f, 1f, 60f), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        w.AddSphere(1f, new Vector3D(0, 1, 0), type: BodyType.Static);
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

    private static void Op(PhysicsWorld3D w, List<SoftBody> softs, Reader r)
    {
        switch (r.Byte() % 14)
        {
            case 0:
            case 1: { softs.Add(w.AddSoftBody(Def(r))); break; }
            case 2: { softs.Add(w.AddCloth(Math.Abs(r.Float()) % 3f, Math.Abs(r.Float()) % 3f, r.Byte() % 12, new Vector3D(r.Float() % 10f, 2 + r.Float() % 5f, r.Float() % 10f), r.Bool())); break; }
            case 3: { softs.Add(w.AddSoftBall(Math.Abs(r.Float()) % 2f, new Vector3D(r.Float() % 10f, 2 + r.Float() % 5f, r.Float() % 10f), (SoftMaterial)(r.Byte() % 4), r.Byte() % 6)); break; }
            case 4: { int n = 1 + r.Byte() % 6; for (int i = 0; i < n; i++) { StepTolerant(w); } break; }
            case 5: { var s = Pick(softs, r); s.SetPinned(r.Index(s.IsValid ? s.ParticleCount : 4), r.Bool()); break; }
            case 6: { var s = Pick(softs, r); s.ApplyImpulse(r.Index(s.IsValid ? s.ParticleCount : 4), r.V3F()); break; }
            case 7: { var s = Pick(softs, r); s.SetPosition(r.Index(s.IsValid ? s.ParticleCount : 4), new Vector3D(r.Double(), r.Double(), r.Double())); break; }
            case 8: { var s = Pick(softs, r); s.Destroy(); break; }
            case 9:
                {
                    var s = Pick(softs, r);
                    switch (r.Byte() % 5)
                    {
                        case 0: s.EdgeCompliance = r.Float(); break;
                        case 1: s.BendCompliance = r.Float(); break;
                        case 2: s.VolumeCompliance = r.Float(); break;
                        case 3: s.InternalPressure = r.Float(); break;
                        default: s.GravityScale = r.Float(); break;
                    }

                    break;
                }
            case 10:
                {
                    // Offsets on the 2^-31 grid, small enough to keep bodies in range; huge or off-grid ones must be refused with an argument error.
                    double Off() => r.Byte() >= 250 ? r.Double() : (sbyte)r.Byte() * 0.5;
                    w.ShiftOrigin(new Vector3D(Off(), Off(), Off()));
                    break;
                }
            case 12:
                {
                    var s = Pick(softs, r);
                    if (s.IsValid && !s.HasSkin) { break; }
                    int nj = s.IsValid ? s.SkeletonJointCount : 2;
                    var pose = new QuaternionF[nj + (r.Byte() >= 250 ? 1 : 0)];
                    for (int i = 0; i < pose.Length; i++) { pose[i] = r.Byte() >= 200 ? new QuaternionF(r.Float(), r.Float(), r.Float(), r.Float()) : QuaternionF.CreateFromAxisAngle(new Vector3F(0f, 0f, 1f), r.Float() % 3f); }
                    s.SetSkeletonPose(pose, new Vector3D(r.Double() % 8.0, r.Double() % 8.0 + 1.0, r.Double() % 8.0), r.Byte() >= 200 ? new QuaternionF(r.Float(), r.Float(), r.Float(), r.Float()) : default, r.Byte() >= 200);
                    break;
                }
            case 13:
                {
                    var s = Pick(softs, r);
                    if (s.IsValid && !s.HasSkin) { break; }
                    int nj = s.IsValid ? s.SkeletonJointCount : 2;
                    var joints = new SkinJointPose[nj + (r.Byte() >= 250 ? 1 : 0)];
                    for (int i = 0; i < joints.Length; i++) { joints[i] = new SkinJointPose(new Vector3D(r.Double() % 8.0, r.Double() % 8.0 + 1.0, r.Double() % 8.0), new QuaternionF(r.Float(), r.Float(), r.Float(), r.Float())); }
                    s.SetSkeletonTransforms(joints, r.Byte() >= 200);
                    if (s.IsValid) { _ = s.GetSkinnedPosition(r.Index(s.ParticleCount)); }
                    break;
                }
            default:
                {
                    var s = Pick(softs, r);
                    s.AddVelocity(r.V3F());
                    _ = s.Volume; _ = s.CenterOfMass; _ = s.Bounds;
                    break;
                }
        }
    }

    private static byte[] Base()
    {
        if (s_base != null) { return s_base; }
        var w = NewWorld();
        w.AddBox(new Vector3F(60f, 1f, 60f), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        w.AddSphere(0.6f, new Vector3D(0, 0.6, 0), type: BodyType.Static);
        w.AddCloth(1.4f, 1.4f, 6, new Vector3D(0, 1.6, 0), pinCorners: false);
        w.AddSoftBox(new Vector3F(0.5f, 0.5f, 0.5f), new Vector3D(1.6, 1.0, 0), SoftMaterial.Jelly, 2);
        w.AddSoftBall(0.3f, new Vector3D(-1.6, 1.0, 0), SoftMaterial.Rubber, 2);
        w.AddBox(new Vector3F(0.4f, 0.4f, 0.4f), new Vector3D(1.6, 2.4, 0));
        var sheet = SoftBodyMesh.Cloth(1f, 1f, 5, 5);
        int nv = sheet.VertexCount;
        var ji = new int[4 * nv]; var jw = new float[4 * nv]; var jm = new float[nv]; var jb = new float[nv]; var jr = new float[nv];
        for (int i = 0; i < nv; i++) { ji[4 * i + 1] = 1; jw[4 * i] = 1f - (i % 6) / 5f; jw[4 * i + 1] = (i % 6) / 5f; jm[i] = i < 6 ? 0f : 0.06f; jb[i] = 0.04f; jr[i] = 0.03f; }
        var skin = SoftSkin.Create(sheet, new[] { -1, 0 }, new[] { new Vector3F(0, 0, 0), new Vector3F(0, 0, 0.5f) }, ji, jw, jm, jb, jr);
        var banner = w.AddSoftBody(new SoftBodyDef { Mesh = sheet, Skin = skin, Position = new Vector3D(0, 2.5, 3) });
        for (int s = 0; s < 30; s++)
        {
            banner.SetSkeletonPose(new[] { QuaternionF.Identity, QuaternionF.CreateFromAxisAngle(new Vector3F(1f, 0f, 0f), 0.02f * s) }, new Vector3D(0, 2.5, 3));
            w.StepChecked(1f / 60f);
        }

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
