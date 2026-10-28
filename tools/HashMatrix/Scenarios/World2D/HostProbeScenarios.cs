using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Physics2D;

namespace RecursiveAI.OnePhysics.HashMatrix;

/// <summary>
/// A world driven the way a deterministic game host drives one: a pile of mixed shapes, kinematic movers (one sliding, one spinning), static, kinematic and dynamic sensors, characters walking through
/// sensors and onto the movers, and a ray, a circle cast and an overlap every step. The host takes a snapshot at step 100. Three runs of the same inputs are compared step by step: one that is never
/// interrupted, one that carries on to step 160 and is then restored to step 100 in the same world, and one that restores the snapshot into a fresh world. A difference in any step's digest
/// (world hash, delivered events, character state, query results) fails the scenario; the manifest carries the uninterrupted run's digests.
/// </summary>
internal static class HostProbeScenarios2D
{
    private const int Steps = 300, SnapshotAt = 100, RewindAt = 160;
    private const float Dt = 1f / 60f;

    [ModuleInitializer]
    internal static void Init() => Scenarios.Register("world2d/host-probe", Run);

    private static ulong Next(ref ulong s)
    {
        ulong z = s += 0x9E3779B97F4A7C15UL;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
        return z ^ (z >> 31);
    }

    private static float Range(ref ulong s, float lo, float hi) => lo + (hi - lo) * ((Next(ref s) >> 40) * (1f / 16777216f));

    private sealed class Digest
    {
        public ulong H = 1469598103934665603UL;

        public void Add(ulong v)
        {
            H = (H ^ v) * 1099511628211UL;
            H ^= H >> 29;
        }

        public void Add(int v) => Add((ulong)(uint)v);
        public void Add(float v) => Add((ulong)(uint)(float.IsNaN(v) ? 0x7fc00000 : BitConverter.SingleToInt32Bits(v)));
        public void Add(double v) => Add((ulong)(double.IsNaN(v) ? 0x7ff8000000000000L : BitConverter.DoubleToInt64Bits(v)));
    }

    private static PhysicsWorld2D NewWorld(int threads) => new(new WorldOptions2D { ThreadCount = threads, WorldId = 31, Events = EventMask.All });

    private static void Build(PhysicsWorld2D w)
    {
        ulong s = 424242;
        w.CreateBox(w.CreateBody(BodyType.Static, new Vector2D(0, -0.5)), 30f, 0.5f);
        w.CreateBox(w.CreateBody(BodyType.Static, new Vector2D(-30, 8)), 0.5f, 9f);
        w.CreateBox(w.CreateBody(BodyType.Static, new Vector2D(30, 8)), 0.5f, 9f);
        // A static sensor zone over the floor, a sliding kinematic platform carrying a kinematic sensor, and a spinning kinematic bar.
        w.CreateBox(w.CreateBody(BodyType.Static, new Vector2D(-10, 1.5)), 2.5f, 1.5f, default, 0f, 0f, new ShapeDef { IsSensor = true });
        var slider = w.CreateBody(new BodyDef { Type = BodyType.Kinematic, Position = new Vector2D(-18, 2.5), LinearVelocity = new Vector2F(2f, 0f) });
        w.CreateBox(slider, 3f, 0.25f);
        w.CreateBox(slider, 3f, 1.5f, new Vector2F(0, 1.75f), 0f, 0f, new ShapeDef { IsSensor = true });
        var spinner = w.CreateBody(new BodyDef { Type = BodyType.Kinematic, Position = new Vector2D(8, 2.2), AngularVelocity = 1.2f });
        w.CreateBox(spinner, 3.5f, 0.2f);
        w.CreateCapsule(spinner, new Vector2F(0, -1.5f), new Vector2F(0, 1.5f), 0.15f);
        // A pile of mixed shapes; every seventh body has a dynamic sensor shape as well.
        for (int i = 0; i < 70; i++)
        {
            var b = w.CreateBody(BodyType.Dynamic, new Vector2D(Range(ref s, -22, 22), 4 + (i / 10) * 1.2), Range(ref s, -1, 1));
            switch (i % 4)
            {
                case 0: w.CreateCircle(b, 0.3f + 0.05f * (i % 3)); break;
                case 1: w.CreateBox(b, 0.4f, 0.3f); break;
                case 2: w.CreateCapsule(b, new Vector2F(-0.4f, 0), new Vector2F(0.4f, 0), 0.22f); break;
                default: w.CreatePolygon(b, [new Vector2F(0, 0.5f), new Vector2F(-0.45f, -0.3f), new Vector2F(0.45f, -0.3f)], 0.02f); break;
            }

            if (i % 7 == 3) { w.CreateCircle(b, 0.9f, default, new ShapeDef { IsSensor = true }); }
        }

        // Fast spinners thrown across the characters' paths, and a kinematic bar created after the pile (so it is the second shape of every pair it is in) that turns about a point well off its middle.
        for (int i = 0; i < 6; i++)
        {
            var t = w.CreateBody(new BodyDef { Type = BodyType.Dynamic, Position = new Vector2D(-26 + 8 * i, 2.5 + 0.4 * (i % 3)), LinearVelocity = new Vector2F(i % 2 == 0 ? 6f : -6f, 3f), AngularVelocity = i % 2 == 0 ? 14f : -17f });
            w.CreateBox(t, 0.9f, 0.12f);
        }

        var late = w.CreateBody(new BodyDef { Type = BodyType.Kinematic, Position = new Vector2D(-4, 1.8), AngularVelocity = 0.9f });
        w.CreateBox(late, 0.15f, 0.15f);
        w.CreateBox(late, 2.5f, 0.12f, new Vector2F(2.6f, 0f), 0f);

        for (int i = 0; i < 6; i++)
        {
            var def = i % 2 == 0 ? Character.Humanoid : Character.Child;
            def.PushStrength = i % 3 == 0 ? 0.5f : 0f;
            w.AddCharacter(new Vector2D(-24 + 4.5 * i, 1.2), def);
        }
    }

    // What the host does in one step: movers get their velocities, characters get their moves, then the world steps and the host queries it. Everything it does is a function of the step number and
    // of what the world reports, so a world restored to the same state sees the same calls.
    private static void Advance(PhysicsWorld2D w, int step, Digest d, Span<QueryHit2D> buffer)
    {
        int slider = 4, spinner = 5;
        w.GetBody(w.Core.HandleOfBody(slider)).LinearVelocity = new Vector2F(((step / 90) & 1) == 0 ? 2f : -2f, 0f);
        w.GetBody(w.Core.HandleOfBody(spinner)).AngularVelocity = ((step / 120) & 1) == 0 ? 1.2f : -0.7f;
        var chars = w.GetCharacters();
        for (int i = 0; i < chars.Length; i++)
        {
            var c = chars[i];
            float vx = (((step / 55) + i) & 1) == 0 ? 2.8f : -2.2f;
            float vy = c.IsGrounded ? ((step + 13 * i) % 71 == 0 ? 5f : -1f) : c.Velocity.Y - 9.81f * Dt;
            c.Move(new Vector2F(vx * Dt, vy * Dt), Dt);
            d.Add(c.Position.X); d.Add(c.Position.Y); d.Add(c.IsGrounded ? 1 : 0); d.Add(c.Velocity.X); d.Add(c.Velocity.Y);
        }

        w.Step(Dt);
        d.Add((ulong)w.ComputeHash());
        d.Add(w.Events.Length);
        foreach (var e in w.Events)
        {
            d.Add((int)e.Kind); d.Add(e.BodyA.Index); d.Add(e.BodyB.Index); d.Add(e.ShapeA.Index); d.Add(e.ShapeB.Index); d.Add(e.Value);
        }

        for (int i = 0; i < 8; i++)
        {
            var o = new Vector2D(-24 + i * 6.5, 3 + (i % 3) * 1.5);
            var dir = new Vector2F(i % 2 == 0 ? 1f : -1f, -0.15f);
            bool hit = w.Raycast(o, dir, out var h, 50f);
            Add(d, hit, h);
            hit = w.CircleCast(o, 0.3f, dir, out h, 50f, new QueryFilter2D { IncludeSensors = true });
            Add(d, hit, h);
            hit = w.OverlapCircle(o, 1.2f, out h);
            Add(d, hit, h);
            int n = w.OverlapBoxAll(o, new Vector2F(3f, 2f), 0.2f * i, buffer);
            d.Add(n);
            for (int j = 0; j < n; j++) { d.Add(buffer[j].Shape.RawHandle.Index); }
        }
    }

    private static void Add(Digest d, bool hit, in QueryHit2D h)
    {
        d.Add(hit ? 1 : 0);
        if (!hit) { return; }
        d.Add(h.Shape.RawHandle.Index); d.Add(h.Point.X); d.Add(h.Point.Y); d.Add(h.Normal.X); d.Add(h.Normal.Y); d.Add(h.Fraction); d.Add(h.Distance);
    }

    private static void Run(ScenarioContext ctx)
    {
        var buffer = new QueryHit2D[16];
        var reference = new ulong[Steps];
        var uninterrupted = NewWorld(ctx.Threads);
        Build(uninterrupted);
        byte[]? snapshot = null;
        var periodic = new List<byte[]>();
        for (int step = 0; step < Steps; step++)
        {
            if (step % 20 == 0) { periodic.Add(uninterrupted.Snapshot()); }
            if (step == SnapshotAt) { snapshot = uninterrupted.Snapshot(); ctx.Write(snapshot); }
            var d = new Digest();
            Advance(uninterrupted, step, d, buffer);
            reference[step] = d.H;
            ctx.Write((long)d.H);
            ctx.EndStep();
        }

        byte[] finalBytes = uninterrupted.Snapshot();

        // The same world, carried past the snapshot and restored to it.
        var same = NewWorld(ctx.Threads);
        Build(same);
        for (int step = 0; step < RewindAt; step++)
        {
            var d = new Digest();
            if (step == SnapshotAt && !same.Snapshot().AsSpan().SequenceEqual(snapshot)) { throw new InvalidOperationException("host-probe 2D: the snapshot differs between runs"); }
            Advance(same, step, d, buffer);
            Check(reference, step, d, "same world before the restore");
        }

        same.Restore(snapshot!);
        for (int step = SnapshotAt; step < Steps; step++)
        {
            var d = new Digest();
            Advance(same, step, d, buffer);
            Check(reference, step, d, "same world after the restore");
        }

        if (!same.Snapshot().AsSpan().SequenceEqual(finalBytes)) { throw new InvalidOperationException("host-probe 2D: the restored world ends in different bytes"); }

        // A fresh world restored from the snapshot.
        var fresh = NewWorld(ctx.Threads);
        fresh.Restore(snapshot!);
        for (int step = SnapshotAt; step < Steps; step++)
        {
            var d = new Digest();
            Advance(fresh, step, d, buffer);
            Check(reference, step, d, "fresh world");
        }

        if (!fresh.Snapshot().AsSpan().SequenceEqual(finalBytes)) { throw new InvalidOperationException("host-probe 2D: the fresh world ends in different bytes"); }

        // Restoring a snapshot taken at any point of the run continues like the run: the pair caches the engine keeps between steps may not decide a result.
        for (int k = 0; k < periodic.Count; k++)
        {
            var other = NewWorld(ctx.Threads);
            other.Restore(periodic[k]);
            for (int step = 20 * k; step < Math.Min(20 * k + 40, Steps); step++)
            {
                var d = new Digest();
                Advance(other, step, d, buffer);
                Check(reference, step, d, "a world restored at step " + (20 * k));
            }
        }

        ctx.Write(1);
    }

    private static void Check(ulong[] reference, int step, Digest d, string which)
    {
        if (reference[step] != d.H) { throw new InvalidOperationException("host-probe 2D: " + which + " differs from the uninterrupted run at step " + step); }
    }
}
