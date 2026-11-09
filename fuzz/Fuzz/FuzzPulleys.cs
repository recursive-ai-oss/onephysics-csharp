using System;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Cables;
using RecursiveAI.OneMath;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

internal static class FuzzPulleys
{
    private static float F(byte[] input, int at) => at + 4 <= input.Length ? BitConverter.ToSingle(input, at) : 0.1f;

    // A pulley system built from hostile bytes: kinds, radii, positions, friction, inertia, lengths and options. Creation either fails with an ArgumentException or ArgumentOutOfRangeException or
    // produces a system that steps, snapshots and restores to the same bytes.
    internal static void Definition(byte[] input)
    {
        if (input.Length < 8) return;
        bool planar = (input[0] & 1) == 0;
        int n = 2 + input[1] % 6;
        try
        {
            if (planar)
            {
                var w = new P2.PhysicsWorld2D(new() { WorldId = 41, SleepEnabled = false });
                var a = w.AddBox(new(.4f, .4f), at: new(0, 2), density: 10); var b = w.AddBox(new(.4f, .4f), at: new(1, 2), density: 10);
                var nodes = new P2.PulleyNodeDef[n];
                for (int i = 0; i < n; i++)
                {
                    int o = 8 + 12 * i; byte k = o < input.Length ? input[o] : (byte)(i == 0 || i == n - 1 ? 0 : 1);
                    var node = new P2.PulleyNodeDef { Kind = (CableNodeKind)(k % 4), Body = (k & 4) != 0 ? a.RawHandle : (k & 8) != 0 ? b.RawHandle : default, Point = new Vector2D(i * .5 + F(input, o + 1) % 1, 3 + F(input, o + 5) % 1), Radius = Math.Abs(F(input, o + 9)) % 1, Friction = (k >> 4) * .1f, Inertia = (k & 128) != 0 ? float.PositiveInfinity : .05f, ExtraTurns = k % 3, Direction = (k & 64) != 0 ? WrapDirection.Clockwise : WrapDirection.CounterClockwise, FixedAxle = (k & 32) != 0 };
                    if (i == 0 || i == n - 1) node.Kind = k % 2 == 0 ? CableNodeKind.Anchor : CableNodeKind.Drum;
                    nodes[i] = node;
                }
                var def = new P2.PulleySystemDef { Nodes = nodes, Closed = (input[0] & 2) != 0, Length = (input[0] & 4) != 0 ? Math.Abs(F(input, 4)) : 0, Slack = (input[0] & 8) != 0 ? Math.Abs(F(input, 4)) % 3 : 0, Preload = (input[0] & 16) != 0 ? Math.Abs(F(input, 4)) : 0, Iterations = 1 + input[2] % 40 };
                var rope = w.AddPulleySystem(def);
                for (int i = 0; i < 5; i++) w.Step(1f / 60);
                var bytes = w.Snapshot(); w.Restore(bytes);
                if (!bytes.AsSpan().SequenceEqual(w.Snapshot())) throw new InvalidOperationException("Pulley system round trip changed bytes.");
                _ = rope;
            }
            else
            {
                var w = new P3.PhysicsWorld3D(new() { WorldId = 41, SleepEnabled = false });
                var a = w.AddBox(new(.4f, .4f, .4f), at: new(0, 2, 0), density: 10); var b = w.AddBox(new(.4f, .4f, .4f), at: new(1, 2, 0), density: 10);
                var nodes = new P3.PulleyNodeDef[n];
                for (int i = 0; i < n; i++)
                {
                    int o = 8 + 12 * i; byte k = o < input.Length ? input[o] : (byte)(i == 0 || i == n - 1 ? 0 : 1);
                    var node = new P3.PulleyNodeDef { Kind = (CableNodeKind)(k % 4), Body = (k & 4) != 0 ? a.RawHandle : (k & 8) != 0 ? b.RawHandle : default, Point = new Vector3D(i * .5 + F(input, o + 1) % 1, 3 + F(input, o + 5) % 1, F(input, o + 9) % 1), Radius = Math.Abs(F(input, o + 9)) % 1, Friction = (k >> 4) * .1f, Inertia = (k & 128) != 0 ? float.PositiveInfinity : .05f, ExtraTurns = k % 3, Direction = (k & 64) != 0 ? WrapDirection.Clockwise : WrapDirection.CounterClockwise, FixedAxle = (k & 32) != 0, Axis = new Vector3F(F(input, o + 5) % 1, k % 2, F(input, o + 1) % 1) };
                    if (i == 0 || i == n - 1) node.Kind = k % 2 == 0 ? CableNodeKind.Anchor : CableNodeKind.Drum;
                    nodes[i] = node;
                }
                var def = new P3.PulleySystemDef { Nodes = nodes, Closed = (input[0] & 2) != 0, Length = (input[0] & 4) != 0 ? Math.Abs(F(input, 4)) : 0, Slack = (input[0] & 8) != 0 ? Math.Abs(F(input, 4)) % 3 : 0, Preload = (input[0] & 16) != 0 ? Math.Abs(F(input, 4)) : 0, Iterations = 1 + input[2] % 40 };
                var rope = w.AddPulleySystem(def);
                for (int i = 0; i < 5; i++) w.Step(1f / 60);
                var bytes = w.Snapshot(); w.Restore(bytes);
                if (!bytes.AsSpan().SequenceEqual(w.Snapshot())) throw new InvalidOperationException("Pulley system round trip changed bytes.");
                _ = rope;
            }
        }
        catch (ArgumentException) { }
    }

    private static string Diff(byte[] a, byte[] b)
    {
        int n = Math.Min(a.Length, b.Length), i = 0; while (i < n && a[i] == b[i]) i++;
        string what = "";
        foreach (var f in RecursiveAI.OnePhysics.SnapshotDesync.Inspect(a)) if (f.Offset <= i && i < f.Offset + f.ByteCount) { what = $"field {f.Section}/{f.Entity}#{f.Index}/{f.Name} ({f.ValueType}) = {f.Value}; "; break; } return string.Concat(what.AsSpan(0, Math.Min(400, what.Length)), $" first difference at {i} of {a.Length}/{b.Length} (from the end {a.Length - i}): {(i < n ? a[i] : -1)} vs {(i < n ? b[i] : -1)}");
    }

    private static byte[]? s_planar, s_spatial;
    private static byte[] Baseline(bool planar)
    {
        if (planar)
        {
            if (s_planar != null) return s_planar;
            var w = new P2.PhysicsWorld2D(new() { WorldId = 41, SleepEnabled = false });
            var a = w.AddBox(new(.4f, .4f), at: new(-.25, 3), density: 40); var b = w.AddBox(new(.4f, .4f), at: new(.25, 3), density: 20);
            w.AddRopeOverPulleys(P2.RopeEnd.On(a, default), [new P2.Pulley(new(0, 5), .25f, clockwise: true, friction: .3f, inertia: float.PositiveInfinity)], P2.RopeEnd.On(b, default), slack: .2);
            var load = w.AddBox(new(.4f, .4f), at: new(4, 1), density: 50);
            w.AddWinch(default, new(4.2, 6), .2f, P2.RopeEnd.On(load, default), maxTorque: 100).SetMotor(0, -2, 100);
            var g = w.AddBox(new(1, 1), at: new(8, -5), type: BodyType.Static); var wa = w.AddCircle(.12f, at: new(8, 0), density: 100); var wb = w.AddCircle(.12f, at: new(8.7, 0), density: 100);
            w.AddHinge(g, wa, new(8, 0)); w.AddHinge(g, wb, new(8.7, 0));
            w.AddBelt([new P2.Pulley(default, .12f, wa), new P2.Pulley(default, .12f, wb)], preload: 5);
            for (int i = 0; i < 4; i++) w.Step(1f / 60);
            return s_planar = w.Snapshot();
        }
        if (s_spatial != null) return s_spatial;
        var world = new P3.PhysicsWorld3D(new() { WorldId = 41, SleepEnabled = false });
        var ba = world.AddBox(new(.4f, .4f, .4f), at: new(-.25, 3, 0), density: 600); var bb = world.AddBox(new(.4f, .4f, .4f), at: new(.25, 3, 0), density: 300);
        world.AddRopeOverPulleys(P3.RopeEnd.On(ba, default), [new P3.Pulley(new(0, 5, 0), .25f, clockwise: true, friction: .3f, inertia: float.PositiveInfinity)], P3.RopeEnd.On(bb, default), slack: .2);
        var crate = world.AddBox(new(.4f, .4f, .4f), at: new(4, 1, 1), density: 800);
        world.AddWinch(default, new(4.2, 6, 1), .2f, P3.RopeEnd.On(crate, default), maxTorque: 300).SetMotor(0, -2, 300);
        for (int i = 0; i < 4; i++) world.Step(1f / 60);
        return s_spatial = world.Snapshot();
    }

    // Damaged snapshots with pulley state: restore either rejects them with a SnapshotFormatException leaving the world unchanged, or accepts them and writes the same bytes back.
    internal static void Snapshot(byte[] input)
    {
        bool planar = input.Length == 0 || (input[0] & 1) == 0;
        var bytes = (byte[])Baseline(planar).Clone();
        for (int i = 1; i + 2 < input.Length; i += 3)
        {
            int offset = bytes.Length - 1 - (((input[i] << 8) | input[i + 1]) % Math.Min(1500, bytes.Length)); bytes[offset] ^= input[i + 2];
        }
        if (input.Length > 0 && (input[0] & 15) == 15) Array.Resize(ref bytes, input[^1] * bytes.Length / 256);
        if (planar)
        {
            var world = new P2.PhysicsWorld2D(new() { WorldId = 41 }); var before = world.Snapshot();
            try { world.Restore(bytes); }
            catch (SnapshotFormatException) { if (!before.AsSpan().SequenceEqual(world.Snapshot())) throw new InvalidOperationException("Rejected pulley snapshot changed world."); return; }
            if (!bytes.AsSpan().SequenceEqual(world.Snapshot())) throw new InvalidOperationException("Accepted pulley snapshot changed bytes." + Diff(bytes, world.Snapshot()));
            for (int i = 0; i < 2; i++) { try { world.Step(1f / 60); } catch (PhysicsException) { return; } }
        }
        else
        {
            var world = new P3.PhysicsWorld3D(new() { WorldId = 41 }); var before = world.Snapshot();
            try { world.Restore(bytes); }
            catch (SnapshotFormatException) { if (!before.AsSpan().SequenceEqual(world.Snapshot())) throw new InvalidOperationException("Rejected pulley snapshot changed world."); return; }
            if (!bytes.AsSpan().SequenceEqual(world.Snapshot())) throw new InvalidOperationException("Accepted pulley snapshot changed bytes." + Diff(bytes, world.Snapshot()));
            for (int i = 0; i < 2; i++) { try { world.Step(1f / 60); } catch (PhysicsException) { return; } }
        }
    }
}
