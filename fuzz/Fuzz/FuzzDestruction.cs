using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Destruction;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

// Fuzz targets for destruction, both dimensions.
// "world3d-destruction" and "world2d-destruction" turn bytes into a program: hostile FractureDef, BreakableDef and DebrisOptions values (NaN, infinities, huge and negative numbers, cell counts out of range, seed
// points outside the body or equal), assets from hostile point sets and cells, manual fractures and slices with degenerate planes, bond breaks, stepping, destroying bodies, stale handles and snapshot round
// trips. Only argument errors, FractureException and other typed errors may come out.
// "world3d-destruction-snapshot" and "world2d-destruction-snapshot" flip bytes in the snapshot of a world with breakable compounds, fracturable bodies and fragments (the DSTR section): rejected with the world
// unchanged, or accepted and re-serialized to the same bytes.
internal static class FuzzDestruction
{
    private static int s_ops, s_refused, s_accepted, s_rejected;
    private static byte[]? s_base2, s_base3;

    static FuzzDestruction()
    {
        AppDomain.CurrentDomain.ProcessExit += (_, _) => Console.WriteLine("destruction targets: ops " + s_ops + ", refused " + s_refused + "; snapshots accepted " + s_accepted + ", rejected " + s_rejected);
    }

    private sealed class Reader
    {
        private readonly byte[] _d; private int _i;
        public Reader(byte[] d) { _d = d; }
        public bool More => _i < _d.Length;
        public byte Byte() => _i < _d.Length ? _d[_i++] : (byte)0;
        public float Float()
        {
            byte b = Byte();
            switch (b) { case 250: return float.NaN; case 251: return float.PositiveInfinity; case 252: return float.NegativeInfinity; case 253: return 3e6f; case 254: return 1e-9f; case 255: return -0f; }
            return (short)(b | (Byte() << 8)) / 256f;
        }
        public double Double() => Float();
    }

    private static bool Allowed(Exception e) => e is ArgumentException || e is PhysicsException || e is InvalidOperationException || e is FractureException;

    private static void Try(Action a)
    {
        s_ops++;
        try { a(); }
        catch (Exception e) when (Allowed(e)) { s_refused++; }
    }

    // ---- 3D

    private static P3.FractureDef Def3(Reader r)
    {
        var d = new P3.FractureDef();
        byte m = r.Byte();
        if ((m & 1) != 0) d.CellCount = r.Byte() % 44 - 4;
        if ((m & 2) != 0) d.Pattern = (FracturePattern)(r.Byte() % 4);
        if ((m & 4) != 0) d.Seed = ((ulong)r.Byte() << 8) | r.Byte();
        if ((m & 8) != 0) { var p = new Vector3F[r.Byte() % 6]; for (int i = 0; i < p.Length; i++) p[i] = new Vector3F(r.Float(), r.Float(), r.Float()); d.Points = p; }
        if ((m & 16) != 0) d.ImpactPoint = new Vector3D(r.Double(), r.Double(), r.Double());
        if ((m & 32) != 0) { d.ImpactRadius = r.Float(); d.ClusterPower = r.Float(); }
        if ((m & 64) != 0) { d.ImpulseThreshold = r.Float(); d.Depth = r.Byte() % 5 - 1; }
        if ((m & 128) != 0) { d.MinFragmentVolume = r.Float(); d.MergeSmall = (r.Byte() & 1) != 0; d.ChildThresholdScale = r.Float(); d.ChildCellCount = r.Byte() % 12 - 3; }
        return d;
    }

    private static Vector3F[] Points3(Reader r)
    {
        var p = new Vector3F[r.Byte() % 14];
        for (int i = 0; i < p.Length; i++) p[i] = new Vector3F(r.Float(), r.Float(), r.Float());
        return p;
    }

    internal static void World3D(byte[] input)
    {
        var r = new Reader(input);
        var world = new P3.PhysicsWorld3D(new() { WorldId = 63, SleepEnabled = (input.Length & 1) == 0 });
        world.AddBox(new(30, 1, 30), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var bodies = new List<P3.Body>();
        var assets = new List<P3.FractureAsset>();
        int ops = 0;
        while (r.More && ops++ < 60)
        {
            byte op = r.Byte();
            P3.Body Pick() => bodies.Count == 0 ? default : bodies[r.Byte() % bodies.Count];
            switch (op % 14)
            {
                case 0: Try(() => bodies.Add(world.AddBox(new(1 + (r.Byte() % 3), 1, 1 + (r.Byte() % 2)), new Vector3D(r.Byte() % 8 - 4, 1 + r.Byte() % 5, r.Byte() % 8 - 4)))); break;
                case 1: Try(() => Pick().Fracture(Def3(r))); break;
                case 2: Try(() => Pick().SetFracturable(Def3(r))); break;
                case 3: Try(() => Pick().Slice(new Vector3D(r.Double(), r.Double(), r.Double()), new Vector3F(r.Float(), r.Float(), r.Float()))); break;
                case 4: Try(() => assets.Add(P3.FractureAsset.FromHull(Points3(r), Def3(r)))); break;
                case 5:
                    Try(() =>
                    {
                        var cells = new List<Vector3F[]>();
                        for (int i = 0; i < r.Byte() % 4; i++) cells.Add(Points3(r));
                        assets.Add(P3.FractureAsset.FromCells(cells));
                    });
                    break;
                case 6:
                    Try(() =>
                    {
                        if (assets.Count == 0) return;
                        var def = new BreakableDef { Strength = r.Float(), ShearFactor = r.Float(), OverloadSteps = r.Byte() % 5 - 1, AnchorParts = r.Byte() % 3 == 0 ? [r.Byte() % 5] : null };
                        bodies.Add(world.AddBreakable(assets[r.Byte() % assets.Count], new Vector3D(r.Byte() % 8 - 4, 2 + r.Byte() % 5, r.Byte() % 8 - 4), def: def, type: (BodyType)(r.Byte() % 3), density: r.Float()));
                    });
                    break;
                case 7: Try(() => { var b = Pick(); var bonds = world.GetBonds(b); world.BreakBond(b, r.Byte() % (bonds.Length + 2) - 1); }); break;
                case 8: Try(() => world.Debris = new DebrisOptions { MaxFragments = r.Byte() % 30 - 2, SleepToStaticSteps = r.Byte() % 20 - 1, LifetimeSteps = r.Byte() % 100 - 3 }); break;
                case 9: Try(() => Pick().Destroy()); break;
                case 10: Try(() => { for (int i = 0; i < 1 + r.Byte() % 20; i++) world.Step(1f / 60); }); break;
                case 11:
                    Try(() =>
                    {
                        var snap = world.Snapshot();
                        var copy = new P3.PhysicsWorld3D(new() { WorldId = 63 });
                        copy.Restore(snap);
                        if (!copy.Snapshot().AsSpan().SequenceEqual(snap)) throw new InvalidOperationException("Snapshot round trip changed bytes.");
                    });
                    break;
                case 12: Try(() => { _ = world.FractureEvents.Length + world.FractureFragments.Length; _ = world.DestructionCounters; }); break;
                default: Try(() => Pick().SetPosition(new Vector3D(r.Byte() % 8 - 4, 3, r.Byte() % 8 - 4))); break;
            }
        }
    }

    // ---- 2D

    private static P2.FractureDef Def2(Reader r)
    {
        var d = new P2.FractureDef();
        byte m = r.Byte();
        if ((m & 1) != 0) d.CellCount = r.Byte() % 44 - 4;
        if ((m & 2) != 0) d.Pattern = (FracturePattern)(r.Byte() % 4);
        if ((m & 4) != 0) d.Seed = ((ulong)r.Byte() << 8) | r.Byte();
        if ((m & 8) != 0) { var p = new Vector2F[r.Byte() % 6]; for (int i = 0; i < p.Length; i++) p[i] = new Vector2F(r.Float(), r.Float()); d.Points = p; }
        if ((m & 16) != 0) d.ImpactPoint = new Vector2D(r.Double(), r.Double());
        if ((m & 32) != 0) { d.ImpactRadius = r.Float(); d.ClusterPower = r.Float(); }
        if ((m & 64) != 0) { d.ImpulseThreshold = r.Float(); d.Depth = r.Byte() % 5 - 1; }
        if ((m & 128) != 0) { d.MinFragmentVolume = r.Float(); d.MergeSmall = (r.Byte() & 1) != 0; d.ChildThresholdScale = r.Float(); d.ChildCellCount = r.Byte() % 12 - 3; }
        return d;
    }

    private static Vector2F[] Points2(Reader r)
    {
        var p = new Vector2F[r.Byte() % 12];
        for (int i = 0; i < p.Length; i++) p[i] = new Vector2F(r.Float(), r.Float());
        return p;
    }

    internal static void World2D(byte[] input)
    {
        var r = new Reader(input);
        var world = new P2.PhysicsWorld2D(new() { WorldId = 64, SleepEnabled = (input.Length & 1) == 0 });
        world.AddBox(new(30, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        var bodies = new List<P2.Body>();
        var assets = new List<P2.FractureAsset>();
        int ops = 0;
        while (r.More && ops++ < 60)
        {
            byte op = r.Byte();
            P2.Body Pick() => bodies.Count == 0 ? default : bodies[r.Byte() % bodies.Count];
            switch (op % 14)
            {
                case 0: Try(() => bodies.Add(world.AddBox(new(1 + (r.Byte() % 3), 1), new Vector2D(r.Byte() % 8 - 4, 1 + r.Byte() % 5), r.Float()))); break;
                case 1: Try(() => Pick().Fracture(Def2(r))); break;
                case 2: Try(() => Pick().SetFracturable(Def2(r))); break;
                case 3: Try(() => Pick().Slice(new Vector2D(r.Double(), r.Double()), new Vector2F(r.Float(), r.Float()))); break;
                case 4: Try(() => assets.Add(P2.FractureAsset.FromPolygon(Points2(r), Def2(r)))); break;
                case 5:
                    Try(() =>
                    {
                        var cells = new List<Vector2F[]>();
                        for (int i = 0; i < r.Byte() % 4; i++) cells.Add(Points2(r));
                        assets.Add(P2.FractureAsset.FromCells(cells));
                    });
                    break;
                case 6:
                    Try(() =>
                    {
                        if (assets.Count == 0) return;
                        var def = new BreakableDef { Strength = r.Float(), ShearFactor = r.Float(), OverloadSteps = r.Byte() % 5 - 1, AnchorParts = r.Byte() % 3 == 0 ? [r.Byte() % 5] : null };
                        bodies.Add(world.AddBreakable(assets[r.Byte() % assets.Count], new Vector2D(r.Byte() % 8 - 4, 2 + r.Byte() % 5), r.Float(), def, (BodyType)(r.Byte() % 3), null, r.Float()));
                    });
                    break;
                case 7: Try(() => { var b = Pick(); var bonds = world.GetBonds(b); world.BreakBond(b, r.Byte() % (bonds.Length + 2) - 1); }); break;
                case 8: Try(() => world.Debris = new DebrisOptions { MaxFragments = r.Byte() % 30 - 2, SleepToStaticSteps = r.Byte() % 20 - 1, LifetimeSteps = r.Byte() % 100 - 3 }); break;
                case 9: Try(() => Pick().Destroy()); break;
                case 11:
                    Try(() =>
                    {
                        var snap = world.Snapshot();
                        var copy = new P2.PhysicsWorld2D(new() { WorldId = 64 });
                        copy.Restore(snap);
                        if (!copy.Snapshot().AsSpan().SequenceEqual(snap)) throw new InvalidOperationException("Snapshot round trip changed bytes.");
                    });
                    break;
                case 12: Try(() => { _ = world.FractureEvents.Length + world.FractureFragments.Length; _ = world.DestructionCounters; }); break;
                default: Try(() => Pick().SetPosition(new Vector2D(r.Byte() % 8 - 4, 3))); break;
            }
        }
    }

    // ---- snapshots

    private static byte[] Base3()
    {
        if (s_base3 != null) return s_base3;
        var w = new P3.PhysicsWorld3D(new() { WorldId = 65 });
        w.AddBox(new(30, 1, 30), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var asset = P3.FractureAsset.FromHull([new(-1, -1, -1), new(1, -1, -1), new(-1, 1, -1), new(1, 1, -1), new(-1, -1, 1), new(1, -1, 1), new(-1, 1, 1), new(1, 1, 1)], new P3.FractureDef { CellCount = 6, Seed = 3 });
        w.AddBreakable(asset, new Vector3D(-3, 4, 0), def: new BreakableDef { Strength = 1e3f, AnchorParts = [0] });
        w.AddBox(new(1, 1, 1), new Vector3D(3, 3, 0)).SetFracturable(new P3.FractureDef { CellCount = 5, ImpulseThreshold = 2f, Depth = 1 });
        w.AddBox(new(1, 1, 1), new Vector3D(0, 3, 0)).Fracture(new P3.FractureDef { CellCount = 5 });
        w.Debris = new() { MaxFragments = 20, LifetimeSteps = 300, SleepToStaticSteps = 50 };
        for (int i = 0; i < 20; i++) w.Step(1f / 60);
        return s_base3 = w.Snapshot();
    }

    private static byte[] Base2()
    {
        if (s_base2 != null) return s_base2;
        var w = new P2.PhysicsWorld2D(new() { WorldId = 66 });
        w.AddBox(new(30, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        var asset = P2.FractureAsset.FromPolygon([new(-1, -1), new(1, -1), new(1, 1), new(-1, 1)], new P2.FractureDef { CellCount = 6, Seed = 3 });
        w.AddBreakable(asset, new Vector2D(-3, 4), def: new BreakableDef { Strength = 1e3f, AnchorParts = [0] });
        w.AddBox(new(1, 1), new Vector2D(3, 3)).SetFracturable(new P2.FractureDef { CellCount = 5, ImpulseThreshold = 2f, Depth = 1 });
        w.AddBox(new(1, 1), new Vector2D(0, 3)).Fracture(new P2.FractureDef { CellCount = 5 });
        w.Debris = new() { MaxFragments = 20, LifetimeSteps = 300, SleepToStaticSteps = 50 };
        for (int i = 0; i < 20; i++) w.Step(1f / 60);
        return s_base2 = w.Snapshot();
    }

    private static byte[] Mutate(byte[] baseline, byte[] input)
    {
        var bytes = (byte[])baseline.Clone();
        // The destruction section is the last one: bias the flips toward the end.
        for (int i = 1; i + 2 < input.Length; i += 3)
        {
            int span = Math.Min(900, bytes.Length);
            int offset = bytes.Length - 1 - (((input[i] << 8) | input[i + 1]) % span);
            bytes[offset] ^= input[i + 2];
        }
        if (input.Length > 0 && (input[0] & 15) == 15) Array.Resize(ref bytes, input[^1] * bytes.Length / 256);
        return bytes;
    }

    internal static void Snapshot3D(byte[] input)
    {
        var bytes = Mutate(Base3(), input);
        var world = new P3.PhysicsWorld3D(new() { WorldId = 65 });
        byte[] before = world.Snapshot();
        try { world.Restore(bytes); }
        catch (SnapshotFormatException)
        {
            s_rejected++;
                        if (!world.Snapshot().AsSpan().SequenceEqual(before)) throw new InvalidOperationException("Rejected destruction snapshot changed the world.");
            return;
        }
        s_accepted++;
        if (!world.Snapshot().AsSpan().SequenceEqual(bytes)) throw new InvalidOperationException("Accepted destruction snapshot changed bytes.");
        // An accepted but damaged section must not break stepping with anything but typed errors.
        try { for (int i = 0; i < 3; i++) world.Step(1f / 60); } catch (Exception e) when (e is PhysicsException || e is ArgumentException || e is InvalidOperationException) { }
    }

    internal static void Snapshot2D(byte[] input)
    {
        var bytes = Mutate(Base2(), input);
        var world = new P2.PhysicsWorld2D(new() { WorldId = 66 });
        byte[] before = world.Snapshot();
        try { world.Restore(bytes); }
        catch (SnapshotFormatException)
        {
            s_rejected++;
            if (!world.Snapshot().AsSpan().SequenceEqual(before)) throw new InvalidOperationException("Rejected destruction snapshot changed the world.");
            return;
        }
        s_accepted++;
        if (!world.Snapshot().AsSpan().SequenceEqual(bytes)) throw new InvalidOperationException("Accepted destruction snapshot changed bytes.");
        try { for (int i = 0; i < 3; i++) world.Step(1f / 60); } catch (Exception e) when (e is PhysicsException || e is ArgumentException || e is InvalidOperationException) { }
    }
}
