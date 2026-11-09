using System;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Rods;
using RecursiveAI.OneMath;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

internal static class FuzzCharacterAbilities
{
    private static float F(byte[] input, int at)
    {
        if (at + 4 > input.Length) { return 0.1f; }
        float v = BitConverter.ToSingle(input, at);
        return v;
    }

    private static P3.PhysicsWorld3D World3D(out P3.Body car, out P3.Rod rope, out P3.Shape wall)
    {
        var w = new P3.PhysicsWorld3D(new() { WorldId = 51, SleepEnabled = false });
        w.AddBox(new(60, 1, 60), at: new(0, -0.5, 0), type: BodyType.Static);
        car = w.AddBox(new(2, 0.6f, 4), at: new(-5, 0.5, 0), density: 200f);
        var ledge = w.AddBox(new(8f, 2f, 4f), at: new(0, 1, 4), type: BodyType.Static);
        wall = ledge.GetShapes()[0];
        var pts = new Vector3D[11];
        for (int i = 0; i <= 10; i++) { pts[i] = new Vector3D(10, 5, 0) + new Vector3D(Math.Sin(0.3) * 2, -Math.Cos(0.3) * 2, 0) * (i / 10.0); }
        rope = w.AddRod(new P3.RodDef { Points = pts, Material = new RodMaterial(1e9f, 4e8f, 0.005f, 1000f), PinnedParticles = [0] });
        return w;
    }

    // Hostile sequences of character-ability calls: mounts, ledge and surface grabs, rope grabs with any position, climbs and releases with any distance, body and rod destruction while held, steps and
    // snapshot round trips. Every call either works or fails with a typed argument or state exception, and the world keeps stepping and round-trips to the same bytes.
    internal static void Calls(byte[] input)
    {
        if (input.Length < 4) { return; }
        bool planar = (input[0] & 1) == 0;
        try
        {
            if (planar) { Calls2D(input); } else { Calls3D(input); }
        }
        catch (ArgumentException) { }
        catch (InvalidOperationException) { }
    }

    private static void Calls3D(byte[] input)
    {
        var w = World3D(out var car, out var rope, out var wall);
        var def = P3.CharacterDef.Humanoid; def.CanClimb = (input[0] & 2) != 0; def.ExactPlatformCarry = (input[0] & 4) != 0;
        var c = w.AddCharacter(new Vector3D(0, 1.15, 1.65), def);
        wall.Climbable = (input[0] & 8) != 0;
        var victim = w.AddBox(new(1, 1, 1), at: new(-5, 5, 5));
        for (int i = 1; i + 5 <= input.Length && i < 200; i += 5)
        {
            byte op = (byte)(input[i] % 14);
            float a = F(input, i + 1);
            try
            {
                switch (op)
                {
                    case 0: c.Mount((input[i + 1] & 1) == 0 ? car : victim, new Vector3F(a, 0.5f, 0.1f), default); break;
                    case 1: c.Dismount(new Vector3F(a, 0, 0)); break;
                    case 2: c.TryGrabLedge(new Vector3F(a, 0, 1)); break;
                    case 3: c.TryClimb(new Vector3F(0, a, 1)); break;
                    case 4: c.ClimbUp(); break;
                    case 5: c.Climb(a); break;
                    case 6: c.Grab(rope, a); break;
                    case 7: c.TryGrab(rope, a); break;
                    case 8: c.Release(); break;
                    case 9: c.WallJump(a, 2f); break;
                    case 10: if (victim.IsValid) { victim.Destroy(); } break;
                    case 11: if (rope.IsValid && (input[i + 1] & 1) == 0) { rope.Destroy(); } break;
                    case 12: c.Move(new Vector3F(a, -0.01f, 0), 1f / 60); break;
                    default: c.Position = new Vector3D(a, 1, 1); break;
                }
            }
            catch (ArgumentException) { }
            catch (InvalidOperationException) { }

            try { w.Step(1f / 60); } catch (PhysicsException) { return; }
        }

        var bytes = w.Snapshot();
        var copy = new P3.PhysicsWorld3D(new() { WorldId = 51 });
        copy.Restore(bytes);
        if (!bytes.AsSpan().SequenceEqual(copy.Snapshot())) { throw new InvalidOperationException("Ability snapshot round trip changed bytes."); }
    }

    private static void Calls2D(byte[] input)
    {
        var w = new P2.PhysicsWorld2D(new() { WorldId = 52, SleepEnabled = false });
        w.AddBox(new(60, 1), at: new(0, -0.5), type: BodyType.Static);
        var car = w.AddBox(new(4, 0.6f), at: new(-5, 0.5), density: 200f);
        var ledge = w.AddBox(new(8f, 2f), at: new(6, 1), type: BodyType.Static);
        var wall = ledge.GetShapes()[0];
        var pts = new Vector2D[11];
        for (int i = 0; i <= 10; i++) { pts[i] = new Vector2D(10, 5) + new Vector2D(Math.Sin(0.3) * 2, -Math.Cos(0.3) * 2) * (i / 10.0); }
        var rope = w.AddRod(new P2.RodDef { Points = pts, Material = new RodMaterial(1e9f, 4e8f, 0.005f, 1000f), PinnedParticles = [0] });
        var def = P2.CharacterDef.Humanoid; def.CanClimb = (input[0] & 2) != 0; def.ExactPlatformCarry = (input[0] & 4) != 0;
        var c = w.AddCharacter(new Vector2D(1.65, 1.15), def);
        wall.Climbable = (input[0] & 8) != 0;
        var victim = w.AddBox(new(1, 1), at: new(-5, 5));
        for (int i = 1; i + 5 <= input.Length && i < 200; i += 5)
        {
            byte op = (byte)(input[i] % 14);
            float a = F(input, i + 1);
            try
            {
                switch (op)
                {
                    case 0: c.Mount((input[i + 1] & 1) == 0 ? car : victim, new Vector2F(a, 0.5f), a); break;
                    case 1: c.Dismount(new Vector2F(a, 0)); break;
                    case 2: c.TryGrabLedge(new Vector2F(a, 1)); break;
                    case 3: c.TryClimb(new Vector2F(a, 1)); break;
                    case 4: c.ClimbUp(); break;
                    case 5: c.Climb(a); break;
                    case 6: c.Grab(rope, a); break;
                    case 7: c.TryGrab(rope, a); break;
                    case 8: c.Release(); break;
                    case 9: c.WallJump(a, 2f); break;
                    case 10: if (victim.IsValid) { victim.Destroy(); } break;
                    case 11: if (rope.IsValid && (input[i + 1] & 1) == 0) { rope.Destroy(); } break;
                    case 12: c.Move(new Vector2F(a, -0.01f), 1f / 60); break;
                    default: c.Position = new Vector2D(a, 1); break;
                }
            }
            catch (ArgumentException) { }
            catch (InvalidOperationException) { }

            try { w.Step(1f / 60); } catch (PhysicsException) { return; }
        }

        var bytes = w.Snapshot();
        var copy = new P2.PhysicsWorld2D(new() { WorldId = 52 });
        copy.Restore(bytes);
        if (!bytes.AsSpan().SequenceEqual(copy.Snapshot())) { throw new InvalidOperationException("Ability snapshot round trip changed bytes."); }
    }

    private static string Diff(byte[] a, byte[] b)
    {
        int n = Math.Min(a.Length, b.Length), i = 0;
        while (i < n && a[i] == b[i]) { i++; }
        string what = "";
        foreach (var f in SnapshotDesync.Inspect(a)) { if (f.Offset <= i && i < f.Offset + f.ByteCount) { what = " field " + f.Section + "/" + f.Entity + "#" + f.Index + "/" + f.Name + " (" + f.ValueType + ")"; break; } }
        return " first difference at byte " + i + what;
    }

    private static byte[]? s_planar, s_spatial;

    private static byte[] Baseline(bool planar)
    {
        if (planar) { return s_planar ??= BuildBaseline2D(); }
        return s_spatial ??= BuildBaseline3D();
    }

    private static byte[] BuildBaseline3D()
    {
        var w = World3D(out var car, out var rope, out var wall);
        var def = P3.CharacterDef.Humanoid; def.CanClimb = true; def.ExactPlatformCarry = true;
        var seated = w.AddCharacter(new Vector3D(-5, 3, 0), def); seated.Mount(car, new Vector3F(0.4f, 0.8f, 0.2f), default);
        var hang = w.AddCharacter(new Vector3D(0, 1.15, 1.65), def); hang.TryGrabLedge(new Vector3F(0, 0, 1));
        var rope3 = w.AddCharacter(new Vector3D(10, 3, 0), def); rope3.Grab(rope, 10f);
        wall.Climbable = true;
        for (int i = 0; i < 4; i++) { w.Step(1f / 60); }
        return w.Snapshot();
    }

    private static byte[] BuildBaseline2D()
    {
        var w = new P2.PhysicsWorld2D(new() { WorldId = 52, SleepEnabled = false });
        w.AddBox(new(60, 1), at: new(0, -0.5), type: BodyType.Static);
        var car = w.AddBox(new(4, 0.6f), at: new(-5, 0.5), density: 200f);
        var ledge = w.AddBox(new(8f, 2f), at: new(6, 1), type: BodyType.Static);
        var pts = new Vector2D[11];
        for (int i = 0; i <= 10; i++) { pts[i] = new Vector2D(10, 5) + new Vector2D(Math.Sin(0.3) * 2, -Math.Cos(0.3) * 2) * (i / 10.0); }
        var rope = w.AddRod(new P2.RodDef { Points = pts, Material = new RodMaterial(1e9f, 4e8f, 0.005f, 1000f), PinnedParticles = [0] });
        var def = P2.CharacterDef.Humanoid; def.CanClimb = true; def.ExactPlatformCarry = true;
        var seated = w.AddCharacter(new Vector2D(-5, 3), def); seated.Mount(car, new Vector2F(0.4f, 0.8f), 0.1f);
        var hang = w.AddCharacter(new Vector2D(1.65, 1.15), def); hang.TryGrabLedge(new Vector2F(1, 0));
        var roper = w.AddCharacter(new Vector2D(10, 3), def); roper.Grab(rope, 10f);
        ledge.GetShapes()[0].Climbable = true;
        for (int i = 0; i < 4; i++) { w.Step(1f / 60); }
        return w.Snapshot();
    }

    // Damaged snapshots with ability state: restore either rejects them with a SnapshotFormatException leaving the world unchanged, or accepts them and writes the same bytes back.
    internal static void Snapshot(byte[] input)
    {
        bool planar = input.Length == 0 || (input[0] & 1) == 0;
        var bytes = (byte[])Baseline(planar).Clone();
        for (int i = 1; i + 2 < input.Length; i += 3)
        {
            int offset = bytes.Length - 1 - (((input[i] << 8) | input[i + 1]) % Math.Min(330, bytes.Length)); bytes[offset] ^= input[i + 2];
        }

        if (input.Length > 0 && (input[0] & 15) == 15) { Array.Resize(ref bytes, input[^1] * bytes.Length / 256); }
        if (planar)
        {
            var world = new P2.PhysicsWorld2D(new() { WorldId = 52 }); var before = world.Snapshot();
            try { world.Restore(bytes); }
            catch (SnapshotFormatException) { if (!before.AsSpan().SequenceEqual(world.Snapshot())) { throw new InvalidOperationException("Rejected ability snapshot changed world."); } return; }
            if (!bytes.AsSpan().SequenceEqual(world.Snapshot())) { throw new InvalidOperationException("Accepted ability snapshot changed bytes." + Diff(bytes, world.Snapshot())); }
            for (int i = 0; i < 2; i++) { try { world.Step(1f / 60); } catch (PhysicsException) { return; } }
        }
        else
        {
            var world = new P3.PhysicsWorld3D(new() { WorldId = 51 }); var before = world.Snapshot();
            try { world.Restore(bytes); }
            catch (SnapshotFormatException) { if (!before.AsSpan().SequenceEqual(world.Snapshot())) { throw new InvalidOperationException("Rejected ability snapshot changed world."); } return; }
            if (!bytes.AsSpan().SequenceEqual(world.Snapshot())) { throw new InvalidOperationException("Accepted ability snapshot changed bytes." + Diff(bytes, world.Snapshot())); }
            for (int i = 0; i < 2; i++) { try { world.Step(1f / 60); } catch (PhysicsException) { return; } }
        }
    }
}
