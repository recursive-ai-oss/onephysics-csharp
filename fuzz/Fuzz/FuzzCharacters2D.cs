using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;

// Fuzz targets for the 2D bullets and character mover.
// "world2d-characters" turns bytes into a program: bodies (some of them bullets), hostile character definitions, moves with hostile displacements and time steps, middle-tier moves from a free state,
// plane solving over hostile planes, edits of live characters, destroying characters and bodies, stepping, stale handles and snapshot round trips. Only argument errors and typed physics errors may come out.
// "world2d-character-snapshot" flips bytes in the snapshot of a world with characters, joints and services: rejected with the world unchanged, or accepted and re-serialized to the same bytes.
internal static class FuzzCharacters2D
{
    private static int s_chars, s_refused, s_moves, s_planes, s_steps, s_accepted, s_rejected;
    private static byte[]? s_base;

    static FuzzCharacters2D()
    {
        AppDomain.CurrentDomain.ProcessExit += (_, _) => Console.WriteLine("character targets: characters " + s_chars + ", refused " + s_refused + ", moves " + s_moves + ", plane solves " + s_planes + ", steps " + s_steps + "; snapshots accepted " + s_accepted + ", rejected " + s_rejected);
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
                case 253: return 3e6;
                case 254: return 1e-300;
                case 255: return -0.0;
            }

            int v = (short)(b | (Byte() << 8));
            return v / 64.0;
        }

        public Vector2F V2F() => new(Float(), Float());

        public bool Bool() => (Byte() & 1) != 0;
    }

    private static bool Allowed(Exception e) => e is ArgumentException || e is PhysicsException;

    private static void StepTolerant(PhysicsWorld2D w)
    {
        try { w.Step(1f / 60f); s_steps++; }
        catch (BodyOutOfBoundsException e) { e.Body.Destroy(); }
    }

    private static PhysicsWorld2D Scene(out Body[] bodies)
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = 9, Substeps = 4 });
        var list = new List<Body>
        {
            w.AddBox(new Vector2F(60f, 1f), new Vector2D(0, -0.5), type: BodyType.Static),
            w.AddBox(new Vector2F(0.5f, 6f), new Vector2D(6, 3), type: BodyType.Static),
        };
        for (int i = 0; i < 4; i++) { list.Add(w.AddBox(new Vector2F(0.8f, 0.8f), new Vector2D(-3 + 1.5 * i, 0.4))); }
        var kin = w.CreateBody(new BodyDef { Type = BodyType.Kinematic, Position = new Vector2D(-6, 2), LinearVelocity = new Vector2F(1f, 0f) });
        w.CreateBox(kin, new Vector2F(2f, 0.2f));
        list.Add(kin);
        var bullet = w.CreateBody(new BodyDef { Type = BodyType.Dynamic, Position = new Vector2D(-12, 2), IsBullet = true, GravityScale = 0f });
        w.CreateCircle(bullet, 0.1f);
        bullet.LinearVelocity = new Vector2F(150f, 0f);
        list.Add(bullet);
        bodies = list.ToArray();
        return w;
    }

    private static CharacterDef Def(Reader r)
    {
        switch (r.Byte() % 5)
        {
            case 0: return CharacterDef.Humanoid;
            case 1: return CharacterDef.Child;
            case 2: return CharacterDef.Giant;
            default:
                return new CharacterDef
                {
                    Radius = r.Float(), Height = r.Float(), SlopeLimitRadians = r.Float(), StepOffset = r.Float(), Skin = r.Float(),
                    LayerMask = r.Byte() < 40 ? 0u : uint.MaxValue, PushStrength = r.Float(), Layer = r.Byte() % 40 - 3, CategoryBits = (uint)r.Byte(), MaskBits = r.Bool() ? uint.MaxValue : (uint)r.Byte(),
                };
        }
    }

    public static void Run(byte[] data)
    {
        var r = new Reader(data);
        var w = Scene(out var bodies);
        var chars = new List<Character>();
        int ops = 4 + r.Byte() % 48;
        for (int n = 0; n < ops && r.More; n++)
        {
            try { Op(w, bodies, chars, r); }
            catch (Exception e) when (Allowed(e)) { s_refused++; }
        }

        for (int i = 0; i < 4; i++) { StepTolerant(w); }
        byte[] snap = w.Snapshot();
        var w2 = new PhysicsWorld2D(new WorldOptions2D { WorldId = 9, Substeps = 4 });
        try { w2.Restore(snap); }
        catch (SnapshotFormatException e) when (e.Message.Contains("out of range", StringComparison.Ordinal) || e.Message.Contains("non-finite", StringComparison.Ordinal)) { return; }
        if (w2.ComputeHash() != w.ComputeHash()) { throw new InvalidOperationException("restored world hash differs"); }
        if (!w2.Snapshot().AsSpan().SequenceEqual(snap)) { throw new InvalidOperationException("restored world re-serializes differently"); }
        if (w2.GetCharacters().Length != w.GetCharacters().Length) { throw new InvalidOperationException("restored world has a different character count"); }
        StepTolerant(w); StepTolerant(w2);
        if (w2.ComputeHash() != w.ComputeHash()) { throw new InvalidOperationException("restored world diverged after a step"); }
    }

    private static Character PickChar(List<Character> chars, Reader r)
    {
        byte b = r.Byte();
        if (chars.Count == 0 || b >= 240) { return default; }
        return chars[b % chars.Count];
    }

    private static void Op(PhysicsWorld2D w, Body[] bodies, List<Character> chars, Reader r)
    {
        switch (r.Byte() % 14)
        {
            case 0:
            case 1:
                {
                    var c = w.AddCharacter(new Vector2D(r.Double(), r.Double()), Def(r));
                    chars.Add(c); s_chars++;
                    break;
                }
            case 2:
            case 3:
            case 4:
                {
                    var c = PickChar(chars, r);
                    s_moves++;
                    c.Move(new Vector2F(r.Float(), r.Float()), r.Byte() < 30 ? r.Float() : 1f / 60f);
                    break;
                }
            case 5: { var c = PickChar(chars, r); s_moves++; c.MoveVelocity(new Vector2F(r.Float(), r.Float()), 1f / 60f); break; }
            case 6: { int n = 1 + r.Byte() % 4; for (int i = 0; i < n; i++) { StepTolerant(w); } break; }
            case 7:
                {
                    var c = PickChar(chars, r);
                    switch (r.Byte() % 4)
                    {
                        case 0: c.SlopeLimitRadians = r.Float(); break;
                        case 1: c.StepOffset = r.Float(); break;
                        case 2: c.PushStrength = r.Float(); break;
                        default: c.LayerMask = (uint)(r.Byte() << 8 | r.Byte()); break;
                    }

                    break;
                }
            case 8: { var c = PickChar(chars, r); c.Destroy(); break; }
            case 9: { byte b = r.Byte(); var body = bodies[b % bodies.Length]; if (body.IsValid && body != bodies[0]) { body.Destroy(); } break; }
            case 10:
                {
                    var def = Def(r);
                    var state = new MoverState2D { Position = new Vector2D(r.Double(), r.Double()) };
                    var hits = new MoverHit2D[r.Byte() % 5];
                    s_moves++;
                    Mover2D.Move(w, in def, ref state, new Vector2F(r.Float(), r.Float()), r.Byte() < 30 ? r.Float() : 1f / 60f, default, hits);
                    break;
                }
            case 11:
                {
                    var planes = new CollisionPlane2D[r.Byte() % 8];
                    for (int i = 0; i < planes.Length; i++) { planes[i] = new CollisionPlane2D { Normal = r.V2F(), Separation = r.Float(), PushLimit = r.Float() }; }
                    s_planes++;
                    var v = Mover2D.SolvePlanes(planes, r.V2F(), 1 + r.Byte() % 60, r.Float());
                    _ = v.X + v.Y;
                    Mover2D.ClipVelocity(planes, r.V2F(), r.Float());
                    break;
                }
            case 12:
                {
                    byte[] snap = w.Snapshot();
                    var w2 = new PhysicsWorld2D(new WorldOptions2D { WorldId = 9, Substeps = 4 });
                    try { w2.Restore(snap); }
                    catch (SnapshotFormatException e) when (e.Message.Contains("out of range", StringComparison.Ordinal) || e.Message.Contains("non-finite", StringComparison.Ordinal)) { break; }
                    if (w2.ComputeHash() != w.ComputeHash()) { throw new InvalidOperationException("restored world hash differs"); }
                    break;
                }
            default:
                {
                    var c = PickChar(chars, r);
                    if (c.IsValid) { _ = c.Position; _ = c.Velocity; _ = c.IsGrounded; _ = c.GroundBody; _ = c.GroundVelocity; }
                    else { _ = c.Position; }
                    break;
                }
        }
    }

    private static byte[] Base(bool withJoints)
    {
        if (s_base != null && !withJoints) { return s_base; }
        var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = 9, Substeps = 4 });
        var floor = w.AddBox(new Vector2F(60f, 1f), new Vector2D(0, -0.5), type: BodyType.Static);
        w.AddBox(new Vector2F(0.8f, 0.8f), new Vector2D(2, 0.4));
        if (withJoints)
        {
            var pivot = w.AddCircle(0.1f, new Vector2D(5, 6), BodyType.Static);
            var bob = w.AddCircle(0.3f, new Vector2D(7, 6));
            w.AddHinge(pivot, bob, new Vector2D(5, 6));
            w.IgnoreLayerCollision(1, 2);
            _ = floor;
        }

        for (int i = 0; i < 3; i++)
        {
            var def = i == 1 ? CharacterDef.Child : CharacterDef.Humanoid;
            def.PushStrength = i * 0.5f;
            w.AddCharacter(new Vector2D(-3 + 2 * i, 1.2), def);
        }

        for (int s = 0; s < 40; s++)
        {
            foreach (var c in w.GetCharacters()) { c.Move(new Vector2F(0.03f, c.IsGrounded ? -0.01f : -0.1f), 1f / 60f); }
            w.Step(1f / 60f);
        }

        var bytes = w.Snapshot();
        if (!withJoints) { s_base = bytes; }
        return bytes;
    }

    public static void SnapshotTarget(byte[] data)
    {
        var bytes = (byte[])Base(data.Length > 0 && (data[0] & 0x10) != 0).Clone();
        int mutations = data.Length > 0 ? 1 + data[0] % 5 : 0;
        for (int i = 1, m = 0; i + 2 < data.Length && m < mutations; i += 3, m++)
        {
            int at = ((data[i] << 8) | data[i + 1]) % bytes.Length;
            bytes[at] ^= data[i + 2];
        }

        if (data.Length > 0 && (data[0] & 0x0F) == 0x0F) { Array.Resize(ref bytes, (data[^1] * bytes.Length) / 256); }

        var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = 9, Substeps = 4 });
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
        foreach (var c in w.GetCharacters()) { _ = c.Position; _ = c.GroundBody; }
        StepTolerant(w);
        foreach (var c in w.GetCharacters()) { try { c.Move(new Vector2F(0.05f, -0.05f), 1f / 60f); } catch (Exception e) when (Allowed(e)) { } }
        StepTolerant(w);
    }
}
