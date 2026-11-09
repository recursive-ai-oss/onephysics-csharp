using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Fields;
using RecursiveAI.OnePhysics.Physics3D;
using P2 = RecursiveAI.OnePhysics.Physics2D;

// Fuzz targets for fabric.
// "world3d-cloth" turns bytes into hostile ClothDef, ClothMaterial, ClothParams and StartVertices values, seams and attachments between stale and out-of-range handles and particles, steps in wind, destroys bodies
// the links name, then a snapshot round trip. Only argument errors and typed physics errors may come out.
// "world3d-cloth-snapshot" and "world2d-strip-snapshot" flip bytes in the snapshot of a world with fabric, seams and attachments (or strips): rejected with the world unchanged, or accepted and re-serialized to the same bytes.
internal static class FuzzCloth
{
    private static byte[]? s_base3, s_base2;

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

        public int Index(int count)
        {
            byte b = Byte();
            if (b >= 248) { return b == 248 ? -1 : b == 249 ? count : b == 250 ? int.MaxValue : int.MinValue; }
            return count == 0 ? 0 : b % count;
        }
    }

    private static float Wind(float v) => float.IsFinite(v) ? Math.Clamp(v, -20f, 20f) : 0f;

    private static bool Allowed(Exception e) => e is ArgumentException || e is PhysicsException || e is ArgumentOutOfRangeException;

    private static PhysicsWorld3D NewWorld3() => new(new WorldOptions3D { WorldId = 12 });

    private static ClothMaterial Material(Reader r) => new()
    {
        ArealDensity = Math.Abs(r.Float()) % 3f, WarpCompliance = r.Float() % 0.1f, WeftCompliance = r.Float() % 0.1f, ShearCompliance = r.Float() % 0.1f, BendCompliance = r.Float() % 0.1f,
        StrainLimit = Math.Abs(r.Float()) % 0.5f, Friction = r.Float() % 2f, Drag = Math.Abs(r.Float()) % 3f, LiftSlope = Math.Abs(r.Float()) % 6f, Damping = Math.Abs(r.Float()) % 0.2f, LinearDamping = Math.Abs(r.Float()) % 2f,
    };

    private static ClothDef Def(Reader r) => new()
    {
        Width = Math.Abs(r.Float()) % 3f, Depth = Math.Abs(r.Float()) % 3f, SegmentsX = 1 + r.Byte() % 12, SegmentsZ = 1 + r.Byte() % 12, Position = new Vector3D(r.Float() % 3f, 1 + Math.Abs(r.Float()) % 3f, r.Float() % 3f),
        Material = r.Byte() % 7 == 0 ? null! : Material(r), RestCurvatureX = r.Byte() % 3 == 0 ? r.Float() : 0f, StartCurved = (r.Byte() & 1) != 0, Tethers = (r.Byte() & 1) != 0, TetherStrain = r.Float() % 0.3f,
        PinnedParticles = (r.Byte() & 1) != 0 ? new List<int> { r.Index(100), r.Index(100) } : null, SelfCollision = (r.Byte() & 1) != 0, TearStrain = r.Byte() % 5 == 0 ? Math.Abs(r.Float()) % 2f : 0f,
        LinearVelocity = new Vector3F(r.Float() % 20f, r.Float() % 20f, r.Float() % 20f),
    };

    public static void Run(byte[] data)
    {
        var r = new Reader(data);
        var w = NewWorld3();
        w.AddBox(new Vector3F(60f, 1f, 60f), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var box = w.AddBox(new Vector3F(0.5f, 0.5f, 0.5f), new Vector3D(0, 1.5, 0));
        w.AddField(new FieldDefinition { Kind = FieldKind.Area, Direction = default, Minimum = new Vector3D(-30, -1, -30), Maximum = new Vector3D(30, 30, 30), Flow = new Vector3F(Wind(r.Float()), 0f, Wind(r.Float())) });
        var clothes = new List<SoftBody>();
        int ops = 3 + r.Byte() % 24;
        for (int n = 0; n < ops && r.More; n++)
        {
            try
            {
                switch (r.Byte() % 8)
                {
                    case 0: case 1: clothes.Add(w.AddCloth(Def(r))); break;
                    case 2:
                        {
                            var sd = Def(r).ToSoftBodyDef();
                            if (r.Byte() % 3 == 0) { sd.Fabric!.StrainLimit = r.Float(); sd.Fabric.WarpAxis = new Vector3F(r.Float(), r.Float(), r.Float()); sd.Fabric.TetherAnchors = new List<int> { r.Index(50), r.Index(50) }; }
                            if (r.Byte() % 3 == 0) { sd.StartVertices = new Vector3F[r.Byte() % 200]; }
                            clothes.Add(w.AddSoftBody(sd));
                            break;
                        }
                    case 3: if (clothes.Count > 1) { w.AddSeam(clothes[r.Index(clothes.Count)], r.Index(150), clothes[r.Index(clothes.Count)], r.Index(150), r.Float()); } break;
                    case 4: if (clothes.Count > 0) { w.AttachCloth(clothes[r.Index(clothes.Count)], r.Index(150), box); } break;
                    case 5: w.RemoveClothLink(r.Index(w.ClothLinkCount + 1)); break;
                    case 6: for (int i = 0; i < 1 + r.Byte() % 20; i++) { w.Step(1f / 60f); } break;
                    default: if (clothes.Count > 0) { var c = clothes[r.Index(clothes.Count)]; if (c.IsValid) { c.Destroy(); } } break;
                }
            }
            catch (Exception e) when (Allowed(e)) { }
            catch (SoftBodyOutOfBoundsException) { }
        }

        try { w.Step(1f / 60f); } catch (Exception e) when (Allowed(e) || e is SoftBodyOutOfBoundsException) { return; }
        byte[] snap;
        try { snap = w.Snapshot(); } catch (Exception e) when (Allowed(e)) { return; }
        var w2 = NewWorld3();
        try { w2.Restore(snap); }
        catch (SnapshotFormatException e) when (e.Message.Contains("out of range", StringComparison.Ordinal) || e.Message.Contains("non-finite", StringComparison.Ordinal)) { return; }
        if (w2.ComputeHash() != w.ComputeHash()) { throw new InvalidOperationException("restored world hash differs"); }
        if (!w2.Snapshot().AsSpan().SequenceEqual(snap)) { throw new InvalidOperationException("restored world re-serializes differently"); }
        if (w2.ClothLinkCount != w.ClothLinkCount) { throw new InvalidOperationException("restored world has a different cloth link count"); }
    }

    private static byte[] Base3()
    {
        if (s_base3 != null) { return s_base3; }
        var w = NewWorld3();
        w.AddBox(new Vector3F(60f, 1f, 60f), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var box = w.AddBox(new Vector3F(0.4f, 0.4f, 0.4f), new Vector3D(0, 3.2, 0));
        w.AddField(new FieldDefinition { Kind = FieldKind.Area, Direction = default, Minimum = new Vector3D(-30, -1, -30), Maximum = new Vector3D(30, 30, 30), Flow = new Vector3F(2f, 0f, 1f) });
        var a = w.AddCloth(new ClothDef { Width = 0.6f, Depth = 0.6f, SegmentsX = 5, SegmentsZ = 5, Position = new Vector3D(0, 3, 0), PinnedParticles = ClothDef.PinEdge(5, 5, 0), Tethers = true, RestCurvatureX = 1f });
        var b = w.AddCloth(new ClothDef { Width = 0.6f, Depth = 0.6f, SegmentsX = 5, SegmentsZ = 5, Position = new Vector3D(0.6, 3, 0), Material = ClothMaterial.Denim });
        w.AddSeam(a, ClothDef.PinEdge(5, 5, 1), b, ClothDef.PinEdge(5, 5, 3), 1e-4f);
        w.AttachClothEdge(a, ClothDef.PinEdge(5, 5, 2), box);
        for (int i = 0; i < 5; i++) { w.Step(1f / 60f); }
        s_base3 = w.Snapshot();
        return s_base3;
    }

    private static byte[] Base2()
    {
        if (s_base2 != null) { return s_base2; }
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 12 });
        w.AddBox(new Vector2F(40f, 1f), new Vector2D(0, -0.5), type: BodyType.Static);
        w.AddField(new FieldDefinition { Kind = FieldKind.Area, Direction = default, Minimum = new Vector3D(-30, -1, -1), Maximum = new Vector3D(30, 30, 1), Flow = new Vector3F(2f, 3f, 0f) });
        w.AddStrip(1f, 0.05f, 8, new Vector2D(0, 2), 0.1f, pinStart: true);
        w.AddStrip(0.6f, 0.05f, 6, new Vector2D(2, 2), 0.1f, pinStart: false);
        for (int i = 0; i < 5; i++) { w.Step(1f / 60f); }
        s_base2 = w.Snapshot();
        return s_base2;
    }

    private static void Flip(byte[] data, byte[] bytes)
    {
        int mutations = data.Length > 0 ? 1 + data[0] % 5 : 0;
        for (int i = 1, m = 0; i + 2 < data.Length && m < mutations; i += 3, m++)
        {
            int span = (data[0] & 0x10) != 0 ? bytes.Length : Math.Min(4000, bytes.Length);
            int at = bytes.Length - 1 - (((data[i] << 8) | data[i + 1]) % span);
            bytes[at] ^= data[i + 2];
        }
    }

    public static void SnapshotTarget3(byte[] data)
    {
        var bytes = (byte[])Base3().Clone();
        Flip(data, bytes);
        if (data.Length > 0 && (data[0] & 0x0F) == 0x0F) { Array.Resize(ref bytes, (data[^1] * bytes.Length) / 256); }
        var w = NewWorld3();
        byte[] empty = w.Snapshot();
        try { w.Restore(bytes); }
        catch (SnapshotFormatException)
        {
            if (!w.Snapshot().AsSpan().SequenceEqual(empty)) { throw new InvalidOperationException("a rejected snapshot changed the world"); }
            return;
        }

        if (!w.Snapshot().AsSpan().SequenceEqual(bytes)) { throw new InvalidOperationException("accepted snapshot does not re-serialize to the same bytes"); }
        try { w.Step(1f / 60f); w.Step(1f / 60f); } catch (Exception e) when (Allowed(e) || e is SoftBodyOutOfBoundsException) { }
    }

    public static void SnapshotTarget2(byte[] data)
    {
        var bytes = (byte[])Base2().Clone();
        Flip(data, bytes);
        if (data.Length > 0 && (data[0] & 0x0F) == 0x0F) { Array.Resize(ref bytes, (data[^1] * bytes.Length) / 256); }
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 12 });
        byte[] empty = w.Snapshot();
        try { w.Restore(bytes); }
        catch (SnapshotFormatException)
        {
            if (!w.Snapshot().AsSpan().SequenceEqual(empty)) { throw new InvalidOperationException("a rejected snapshot changed the world"); }
            return;
        }

        if (!w.Snapshot().AsSpan().SequenceEqual(bytes)) { throw new InvalidOperationException("accepted snapshot does not re-serialize to the same bytes"); }
        try { w.Step(1f / 60f); w.Step(1f / 60f); } catch (Exception e) when (e is ArgumentException || e is PhysicsException) { }
    }
}
