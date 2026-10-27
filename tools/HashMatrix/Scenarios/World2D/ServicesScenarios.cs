using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Collision2D;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Debug;
using RecursiveAI.OnePhysics.Physics2D;

namespace RecursiveAI.OnePhysics.HashMatrix;

/// <summary>
/// The 2D world services: layer matrix and filter changes at runtime, a struct contact filter, sensors with enter and exit events, hit events, sleep and wake events, queries of every kind after each step,
/// debug geometry, and a snapshot restored halfway. Every step writes the world hash, the delivered events, trigger and collision callback logs, query results and a digest of the debug geometry.
/// </summary>
internal static class ServicesScenarios
{
    [ModuleInitializer]
    internal static void Init() => Scenarios.Register("world2d/services", Run);

    private readonly struct RefuseOdd : IContactFilter
    {
        public bool ShouldCollide(Shape a, Shape b) => ((a.Body.RawHandle.Index + b.Body.RawHandle.Index) & 7) != 5;
    }

    private static ulong Next(ref ulong s)
    {
        ulong z = s += 0x9E3779B97F4A7C15UL;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
        return z ^ (z >> 31);
    }

    private static float Range(ref ulong s, float lo, float hi) => lo + (hi - lo) * ((Next(ref s) >> 40) * (1f / 16777216f));

    private static PhysicsWorld2D Build(int threads, ulong seed)
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { ThreadCount = threads, WorldId = 11, Events = EventMask.All, HitSpeedThreshold = 2f });
        ulong s = seed;
        var floor = w.CreateBody(BodyType.Static, new Vector2D(0, -0.5));
        w.CreateBox(floor, 30f, 0.5f, default, 0f, 0f, new ShapeDef { Layer = 1 });
        w.CreateBox(w.CreateBody(BodyType.Static, new Vector2D(-30, 10)), 0.5f, 10f);
        w.CreateBox(w.CreateBody(BodyType.Static, new Vector2D(30, 10)), 0.5f, 10f);
        // Two sensor zones, one on a static body and one on a kinematic platform.
        w.CreateBox(w.CreateBody(BodyType.Static, new Vector2D(-8, 1.5)), 3f, 1.5f, default, 0f, 0f, new ShapeDef { IsSensor = true });
        var platform = w.CreateBody(new BodyDef { Type = BodyType.Kinematic, Position = new Vector2D(0, 4), LinearVelocity = new Vector2F(1.5f, 0f) });
        w.CreateBox(platform, 4f, 0.25f);
        w.CreateBox(platform, 4f, 2f, new Vector2F(0, 2.25f), 0f, 0f, new ShapeDef { IsSensor = true });
        for (int i = 0; i < 90; i++)
        {
            var b = w.CreateBody(BodyType.Dynamic, new Vector2D(Range(ref s, -20, 20), 6 + (i / 10) * 1.3), Range(ref s, -1, 1));
            var def = new ShapeDef { Layer = i % 3 == 0 ? 2 : 0, CategoryBits = (uint)(1 << (i % 4)), MaskBits = i % 11 == 0 ? 0xFu : uint.MaxValue };
            if (i % 13 == 0) { def.IsSensor = true; }
            switch (i % 4)
            {
                case 0: w.CreateCircle(b, 0.35f + 0.1f * (i % 3), default, def); break;
                case 1: w.CreateBox(b, 0.4f, 0.3f, default, 0f, 0f, def); break;
                case 2: w.CreateCapsule(b, new Vector2F(-0.4f, 0), new Vector2F(0.4f, 0), 0.25f, def); break;
                default: w.CreatePolygon(b, [new Vector2F(0, 0.5f), new Vector2F(-0.45f, -0.3f), new Vector2F(0.45f, -0.3f)], 0f, def); break;
            }
        }

        return w;
    }

    private static void Run(ScenarioContext ctx)
    {
        var w = Build(ctx.Threads, 20260);
        var log = new List<long>();
        // Callbacks on a few bodies feed the digest in delivery order.
        for (int b = 3; b < 60; b += 7)
        {
            var body = w.GetBody(w.Core.HandleOfBody(b));
            body.OnCollisionEnter += c => { log.Add(1000 + c.Body.RawHandle.Index * 64 + c.Other.RawHandle.Index); log.Add(BitConverter.SingleToInt32Bits(c.NormalImpulse)); };
            body.OnCollisionExit += c => log.Add(2000 + c.Body.RawHandle.Index * 64 + c.Other.RawHandle.Index);
            body.OnCollisionHit += c => { log.Add(3000 + c.Body.RawHandle.Index); log.Add(BitConverter.SingleToInt32Bits(c.ApproachSpeed)); };
            body.OnTriggerEnter += t => log.Add(4000 + t.Body.RawHandle.Index * 64 + t.Other.RawHandle.Index);
            body.OnTriggerExit += t => log.Add(5000 + t.Body.RawHandle.Index * 64 + t.Other.RawHandle.Index);
            body.OnSleep += x => log.Add(6000 + x.RawHandle.Index);
            body.OnWake += x => log.Add(7000 + x.RawHandle.Index);
        }

        var buffer = new QueryHit2D[8];
        var geometry = new DebugGeometry();
        byte[]? snapshot = null;
        const float Dt = 1f / 60f;
        for (int step = 0; step < 240; step++)
        {
            if (step == 50) { w.IgnoreLayerCollision(0, 2); }
            if (step == 90) { w.IgnoreLayerCollision(0, 2, false); }
            if (step == 110) { w.SetContactFilter(new RefuseOdd()); }
            if (step == 150) { w.ClearContactFilter(); }
            if (step == 170) { w.GetBody(w.Core.HandleOfBody(20)).GetShapes()[0].MaskBits = 0; }
            if (step == 120) { snapshot = w.Snapshot(); ctx.Write(snapshot); }
            if (step == 140) { w.Restore(snapshot!); }
            if (step == 200) { w.GetBody(w.Core.HandleOfBody(30)).Destroy(); }

            w.Step(Dt);
            ctx.Write((long)w.ComputeHash());
            ctx.Write(log.Count);
            foreach (long v in log) { ctx.Write(v); }
            log.Clear();
            foreach (var e in w.Events)
            {
                ctx.Write((int)e.Kind); ctx.Write(e.BodyA.Index); ctx.Write(e.BodyB.Index); ctx.Write(e.ShapeA.Index); ctx.Write(e.ShapeB.Index); ctx.Write(e.Value);
            }

            // Queries of every kind from a fixed set of probes.
            for (int i = 0; i < 24; i++)
            {
                var o = new Vector2D(-25 + i * 2.1, 2 + (i % 5) * 2.0);
                var d = new Vector2F(i % 2 == 0 ? 1f : -1f, -0.2f + 0.05f * (i % 9));
                WriteHit(ctx, w.Raycast(o, d, out var h, 60f), h);
                WriteHit(ctx, w.CircleCast(o, 0.3f, d, out h, 60f), h);
                WriteHit(ctx, w.BoxCast(o, new Vector2F(0.6f, 0.4f), 0.3f * i, d, out h, 60f, QueryFilter2D.Layers(0b101)), h);
                WriteHit(ctx, w.OverlapCircle(o, 1.5f, out h), h);
                WriteHit(ctx, w.ClosestPoint(o, out h, 30f, new QueryFilter2D { IncludeSensors = true }), h);
                int n = w.RaycastAll(o, d, buffer, 60f);
                ctx.Write(n);
                for (int j = 0; j < n; j++) { WriteHit(ctx, true, buffer[j]); }
                ctx.Write(w.OverlapBoxAll(o, new Vector2F(2f, 2f), 0.1f * i, buffer));
            }

            if (step % 20 == 0)
            {
                w.GetDebugGeometry(DebugContent.All, geometry);
                ctx.Write(geometry.Shapes.Length); ctx.Write(geometry.Points.Length); ctx.Write(geometry.Contacts.Length); ctx.Write(geometry.Bodies.Length);
                foreach (var p in geometry.Points) { ctx.Write(p.X); ctx.Write(p.Y); }
                foreach (var c in geometry.Contacts) { ctx.Write(c.Point.X); ctx.Write(c.Point.Y); ctx.Write(c.Normal.X); ctx.Write(c.Separation); }
                foreach (var b in geometry.Bodies) { ctx.Write((int)b.State); ctx.Write(b.Island); ctx.Write(b.CenterOfMass.X); }
            }

            ctx.EndStep();
        }
    }

    private static void WriteHit(ScenarioContext ctx, bool hit, in QueryHit2D h)
    {
        ctx.Write(hit ? 1 : 0);
        if (!hit) { return; }
        ctx.Write(h.Shape.RawHandle.Index); ctx.Write(h.Point.X); ctx.Write(h.Point.Y); ctx.Write(h.Normal.X); ctx.Write(h.Normal.Y); ctx.Write(h.Fraction); ctx.Write(h.Distance);
    }
}
