using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;

// Fuzz targets for the 2D world. "world2d-snapshot" damages a valid snapshot (byte flips, truncation) and restores it: the only error allowed is SnapshotFormatException, a rejected
// snapshot must leave the world exactly as it was, and an accepted one must step and re-serialize to the same bytes. "world2d-defs" reads bytes as body, shape and joint-free world
// parameters, many of them hostile (NaN, infinity, huge, tiny, negative): creation may throw only argument errors, typed physics errors or invalid-shape errors, never an index or null error,
// and whatever was created must step and survive a snapshot round trip.
internal static class Fuzz2DWorld
{
    private static int s_accepted, s_rejected, s_created, s_refused, s_stepped, s_oob;
    private static byte[]? s_base;

    static Fuzz2DWorld()
    {
        AppDomain.CurrentDomain.ProcessExit += (_, _) => Console.WriteLine("world2d targets: snapshots accepted " + s_accepted + ", rejected " + s_rejected + "; bodies created " + s_created + ", creations refused " + s_refused + ", steps run " + s_stepped + ", out-of-bounds bodies handled " + s_oob);
    }

    private static PhysicsWorld2D BaseWorld(out PhysicsWorld2D? unused)
    {
        unused = null;
        var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = 7 });
        w.AddBox(new Vector2F(40f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static);
        for (int i = 0; i < 6; i++)
        {
            w.AddBox(new Vector2F(1f, 1f), at: new Vector2D(0.2 * i, 0.5 + 1.05 * i));
        }

        w.AddCircle(0.5f, at: new Vector2D(3, 2));
        w.AddCapsule(0.5f, 0.25f, at: new Vector2D(-3, 2));
        for (int i = 0; i < 40; i++) { w.StepChecked(1f / 60f); }
        return w;
    }

    private static byte[] Base() => s_base ??= BaseWorld(out _).Snapshot();

    public static void SnapshotTarget(byte[] data)
    {
        var bytes = (byte[])Base().Clone();
        int mutations = data.Length > 0 ? 1 + data[0] % 4 : 0;
        for (int i = 1, m = 0; i + 2 < data.Length && m < mutations; i += 3, m++)
        {
            int at = ((data[i] << 8) | data[i + 1]) % bytes.Length;
            bytes[at] ^= data[i + 2];
        }

        if (data.Length > 0 && (data[0] & 0x0F) == 0x0F) { Array.Resize(ref bytes, (data[^1] * bytes.Length) / 256); }

        var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = 7 });
        byte[] empty = w.Snapshot();
        try { w.Restore(bytes); }
        catch (SnapshotFormatException)
        {
            s_rejected++;
            if (!w.Snapshot().AsSpan().SequenceEqual(empty)) { throw new InvalidOperationException("a rejected snapshot changed the world"); }
            return;
        }

        s_accepted++;
        byte[] again = w.Snapshot();
        if (!again.AsSpan().SequenceEqual(bytes)) { throw new InvalidOperationException("accepted snapshot does not re-serialize to the same bytes"); }
        StepTolerant(w);
        StepTolerant(w);
    }

    private sealed class Reader
    {
        private readonly byte[] _d;
        private int _i;

        public Reader(byte[] d) { _d = d; }

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
                case 253: return 3e6;
                case 254: return 1e-300;
                case 255: return -0.0;
            }

            int v = (short)(b | (Byte() << 8));
            return v / 64.0;
        }

        public Vector2F V2F() => new(Float(), Float());
    }

    // A body that left the supported range is reported with a typed error naming a live body and the world is unchanged; the body is destroyed and stepping continues.
    private static void StepTolerant(PhysicsWorld2D w)
    {
        try { w.StepChecked(1f / 60f); s_stepped++; }
        catch (BodyOutOfBoundsException e)
        {
            ulong before = w.ComputeHash();
            try { w.StepChecked(1f / 60f); throw new InvalidOperationException("a step after the out-of-bounds error did not raise it again"); }
            catch (BodyOutOfBoundsException) { }
            if (before != w.ComputeHash()) { throw new InvalidOperationException("the out-of-bounds error changed the world"); }
            e.Body.Destroy();
            s_oob++;
        }
    }

    private static bool Allowed(Exception e) => e is ArgumentException || e is PhysicsException;

    public static void DefsTarget(byte[] data)
    {
        var r = new Reader(data);
        var opt = new WorldOptions2D { WorldId = 9, Substeps = 1 + (r.Byte() & 3) };
        var w = new PhysicsWorld2D(opt);
        int count = 1 + (r.Byte() % 6);
        var bodies = new Body[count];
        int made = 0;
        for (int i = 0; i < count; i++)
        {
            try
            {
                var bd = new BodyDef
                {
                    Type = (BodyType)(r.Byte() % 4),
                    Position = new Vector2D(r.Double(), r.Double()),
                    Angle = r.Float(),
                    LinearVelocity = r.V2F(),
                    AngularVelocity = r.Float(),
                    LinearDamping = r.Float(),
                    AngularDamping = r.Float(),
                    GravityScale = r.Float(),
                    FixedRotation = (r.Byte() & 1) != 0,
                    IsBullet = (r.Byte() & 1) != 0,
                    StartAsleep = (r.Byte() & 1) != 0,
                };
                var b = w.CreateBody(in bd);
                bodies[made++] = b;
                s_created++;
                int shapes = r.Byte() % 3;
                for (int s = 0; s < shapes; s++)
                {
                    var sd = new ShapeDef
                    {
                        Density = r.Float(), Friction = r.Float(), Restitution = r.Float(), RollingResistance = r.Float(),
                        CategoryBits = r.Byte(), MaskBits = r.Byte(), GroupIndex = (sbyte)r.Byte(),
                    };
                    switch (r.Byte() % 6)
                    {
                        case 0: w.CreateCircle(b, r.Float(), r.V2F(), sd); break;
                        case 1: w.CreateCapsule(b, r.V2F(), r.V2F(), r.Float(), sd); break;
                        case 2:
                            {
                                var pts = new Vector2F[r.Byte() % 12];
                                for (int k = 0; k < pts.Length; k++) { pts[k] = r.V2F(); }
                                w.CreatePolygon(b, pts, r.Float(), sd);
                                break;
                            }
                        case 3: w.CreateBox(b, new Vector2F(r.Float(), r.Float()), r.V2F(), r.Float(), r.Float(), sd); break;
                        case 4: w.CreateSegment(b, r.V2F(), r.V2F(), sd); break;
                        default:
                            {
                                var pts = new Vector2F[r.Byte() % 12];
                                for (int k = 0; k < pts.Length; k++) { pts[k] = r.V2F(); }
                                w.CreateChain(b, pts, (r.Byte() & 1) != 0, (r.Byte() & 1) != 0, sd);
                                break;
                            }
                    }
                }
            }
            catch (Exception e) when (Allowed(e)) { s_refused++; }
        }

        for (int step = 0; step < 6; step++) { StepTolerant(w); }

        byte[] snap = w.Snapshot();
        var w2 = new PhysicsWorld2D(new WorldOptions2D { WorldId = 9, Substeps = opt.Substeps });
        // A snapshot is refused when a body is outside the supported range, and the next step of the original world would have raised for the same body.
        try { w2.Restore(snap); }
        catch (SnapshotFormatException)
        {
            try { w.StepChecked(1f / 60f); }
            catch (BodyOutOfBoundsException) { s_oob++; return; }
            throw new InvalidOperationException("a snapshot of a world whose next step succeeds was refused");
        }

        if (w2.ComputeHash() != w.ComputeHash()) { throw new InvalidOperationException("restored world hash differs"); }
        if (!w2.Snapshot().AsSpan().SequenceEqual(snap)) { throw new InvalidOperationException("restored world re-serializes differently"); }
    }
}
