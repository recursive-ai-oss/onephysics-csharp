using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics3D;

// Fuzz targets for 3D fluids and buoyancy.
// "world3d-fluids" turns bytes into a program: hostile fluid definitions (NaN, infinite, zero and negative density, zero and huge normals, inverted regions), buoyancy factors, stale handles, the raw ApplyBuoyancy,
// the geometry entry point Buoyancy3D.Submerged with hostile planes, stepping, origin shifts and snapshot round trips. Only argument errors and typed physics errors may come out.
// "world3d-fluid-snapshot" flips bytes in the snapshot of a world with fluids and buoyancy factors: rejected with the world unchanged, or accepted and re-serialized to the same bytes.
internal static class FuzzFluids3D
{
    private static int s_defs, s_refused, s_steps, s_accepted, s_rejected;
    private static byte[]? s_base;

    static FuzzFluids3D()
    {
        AppDomain.CurrentDomain.ProcessExit += (_, _) => Console.WriteLine("fluid targets: fluids " + s_defs + ", refused " + s_refused + ", steps " + s_steps + "; snapshots accepted " + s_accepted + ", rejected " + s_rejected);
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
    }

    private static bool Allowed(Exception e) => e is ArgumentException || e is PhysicsException;

    private static void StepTolerant(PhysicsWorld3D w)
    {
        try { w.Step(1f / 60f); s_steps++; }
        catch (BodyOutOfBoundsException e) { e.Body.Destroy(); }
    }

    private static PhysicsWorld3D NewWorld() => new(new WorldOptions3D { WorldId = 11 });

    private static FluidDef Def(Reader r)
    {
        var d = new FluidDef { SurfaceOffset = r.Byte() % 3 == 0 ? r.Double() : (r.Byte() % 8) - 2.0 };
        byte m = r.Byte();
        if ((m & 1) != 0) { d.SurfaceNormal = r.V3F(); }
        if ((m & 2) != 0) { d.Density = r.Float(); }
        if ((m & 4) != 0) { d.Flow = r.V3F(); }
        if ((m & 8) != 0) { d.LinearDrag = r.Float(); d.AngularDrag = r.Float(); }
        if ((m & 16) != 0) { d.ViscousDamping = r.Float(); }
        if ((m & 32) != 0) { d.LimitToRegion = true; d.RegionMin = new Vector3D(r.Double(), r.Double(), r.Double()); d.RegionMax = new Vector3D(r.Double(), r.Double(), r.Double()); }
        return d;
    }

    private static Body PickBody(List<Body> bs, Reader r)
    {
        byte b = r.Byte();
        if (bs.Count == 0 || b >= 240) { return default; }
        return bs[b % bs.Count];
    }

    private static Fluid PickFluid(List<Fluid> fs, Reader r)
    {
        byte b = r.Byte();
        if (fs.Count == 0 || b >= 240) { return default; }
        return fs[b % fs.Count];
    }

    public static void Run(byte[] data)
    {
        var r = new Reader(data);
        var w = NewWorld();
        w.AddBox(new Vector3F(60f, 1f, 60f), new Vector3D(0, -6.5, 0), type: BodyType.Static);
        var bs = new List<Body>();
        var fs = new List<Fluid>();
        int ops = 4 + r.Byte() % 48;
        for (int n = 0; n < ops && r.More; n++)
        {
            try { Op(w, bs, fs, r); }
            catch (Exception e) when (Allowed(e)) { s_refused++; }
        }

        for (int i = 0; i < 4; i++) { StepTolerant(w); }
        byte[] snap = w.Snapshot();
        var w2 = NewWorld();
        try { w2.Restore(snap); }
        catch (SnapshotFormatException e) when (e.Message.Contains("out of range", StringComparison.Ordinal) || e.Message.Contains("non-finite", StringComparison.Ordinal)) { return; }
        if (w2.ComputeHash() != w.ComputeHash()) { throw new InvalidOperationException("restored world hash differs"); }
        if (!w2.Snapshot().AsSpan().SequenceEqual(snap)) { throw new InvalidOperationException("restored world re-serializes differently"); }
        if (w2.FluidCount != w.FluidCount) { throw new InvalidOperationException("restored world has a different fluid count"); }
        StepTolerant(w); StepTolerant(w2);
        if (w2.ComputeHash() != w.ComputeHash()) { throw new InvalidOperationException("restored world diverged after a step"); }
    }

    private static void Op(PhysicsWorld3D w, List<Body> bs, List<Fluid> fs, Reader r)
    {
        switch (r.Byte() % 11)
        {
            case 0:
            case 1: { var f = w.AddFluid(Def(r)); fs.Add(f); s_defs++; break; }
            case 2:
                {
                    var at = new Vector3D(r.Float() % 20f, r.Float() % 10f, r.Float() % 20f);
                    float density = Math.Abs(r.Float()) % 3000f + 1f;
                    Body b = (r.Byte() % 4) switch
                    {
                        0 => w.AddSphere(0.1f + (Math.Abs(r.Float()) % 2f), at, density: density),
                        1 => w.AddBox(new Vector3F(0.2f + Math.Abs(r.Float()) % 3f, 0.2f + Math.Abs(r.Float()) % 3f, 0.2f + Math.Abs(r.Float()) % 3f), at, density: density),
                        2 => w.AddCapsule(0.1f + Math.Abs(r.Float()) % 2f, 0.1f + Math.Abs(r.Float()) % 1f, at, density: density),
                        _ => w.AddCylinder(0.1f + Math.Abs(r.Float()) % 2f, 0.1f + Math.Abs(r.Float()) % 1f, at, density: density),
                    };
                    bs.Add(b);
                    break;
                }
            case 3: { var b = PickBody(bs, r); b.BuoyancyFactor = r.Float(); break; }
            case 4: { int n = 1 + r.Byte() % 6; for (int i = 0; i < n; i++) { StepTolerant(w); } break; }
            case 5: { var f = PickFluid(fs, r); f.SetDefinition(Def(r)); break; }
            case 6: { var f = PickFluid(fs, r); if (r.Bool()) { f.Flow = r.V3F(); } else { f.SurfaceOffset = r.Double(); } break; }
            case 7: { var f = PickFluid(fs, r); f.Remove(); break; }
            case 8: { var b = PickBody(bs, r); _ = w.ApplyBuoyancy(b, Def(r), r.Float()); break; }
            case 9:
                {
                    ConvexShape3D shape = (r.Byte() % 3) switch
                    {
                        0 => ConvexShape3D.Sphere(Math.Abs(r.Float()) % 3f + 0.01f),
                        1 => ConvexShape3D.Box(new Vector3F(Math.Abs(r.Float()) % 3f + 0.01f, Math.Abs(r.Float()) % 3f + 0.01f, Math.Abs(r.Float()) % 3f + 0.01f)),
                        _ => ConvexShape3D.TaperedCapsule(Math.Abs(r.Float()) % 3f + 0.01f, Math.Abs(r.Float()) % 2f + 0.01f, Math.Abs(r.Float()) % 2f + 0.01f),
                    };
                    var pose = new RigidTransformF(r.Bool() ? QuaternionF.Identity : new QuaternionF(r.Float(), r.Float(), r.Float(), r.Float()), r.V3F());
                    var res = Buoyancy3D.Submerged(shape, pose, r.V3F(), r.Float(), r.V3F());
                    if (!(res.Volume >= 0f) && float.IsFinite(pose.Translation.X)) { throw new InvalidOperationException("negative submerged volume"); }
                    break;
                }
            case 10:
                {
                    // Offsets on the 2^-31 grid, small enough to keep bodies in range; huge or off-grid ones must be refused with an argument error.
                    double Off() => r.Byte() >= 250 ? r.Double() : (sbyte)r.Byte() * 0.5;
                    w.ShiftOrigin(new Vector3D(Off(), Off(), Off()));
                    break;
                }
            default: { StepTolerant(w); break; }
        }
    }

    private static byte[] Base()
    {
        if (s_base != null) { return s_base; }
        var w = NewWorld();
        w.AddBox(new Vector3F(60f, 1f, 60f), new Vector3D(0, -6.5, 0), type: BodyType.Static);
        w.AddFluid(new FluidDef { SurfaceOffset = 0.5, Flow = new Vector3F(0.5f, 0f, 0.2f), ViscousDamping = 1f, LimitToRegion = true, RegionMin = new Vector3D(-20, -6, -20), RegionMax = new Vector3D(20, 3, 20) });
        w.AddWater(-2.0, 1200f);
        for (int i = 0; i < 6; i++)
        {
            var at = new Vector3D(-3 + i * 1.2, 1.0 + 0.5 * i, 0.4 * i);
            var b = (i % 3) switch
            {
                0 => w.AddBox(new Vector3F(0.6f, 0.4f, 0.5f), at, density: 300f + 150f * i),
                1 => w.AddSphere(0.3f, at, density: 700f),
                _ => w.AddCapsule(0.3f, 0.15f, at, density: 900f),
            };
            if (i == 2) { b.BuoyancyFactor = 1.5f; }
            if (i == 4) { b.BuoyancyFactor = 0f; }
        }

        for (int s = 0; s < 40; s++) { w.Step(1f / 60f); }
        return s_base = w.Snapshot();
    }

    public static void SnapshotTarget(byte[] data)
    {
        var bytes = (byte[])Base().Clone();
        int mutations = data.Length > 0 ? 1 + data[0] % 5 : 0;
        // The fluid block is near the end of the snapshot: aim most flips at the last 400 bytes.
        for (int i = 1, m = 0; i + 2 < data.Length && m < mutations; i += 3, m++)
        {
            int span = (data[0] & 0x10) != 0 ? bytes.Length : Math.Min(400, bytes.Length);
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
