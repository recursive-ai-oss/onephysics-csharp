using System;
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.HashMatrix;

internal static class FoundationScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("snapshots/forks-and-regions", Run);
        Scenarios.Register("snapshots/encoding-v2", Encoding);
    }

    private static void Encoding(ScenarioContext ctx)
    {
        var a = new byte[4096]; var b = new byte[4096];
        for (int i = 0; i < a.Length; i++) { a[i] = (byte)(i / 8); b[i] = (byte)(a[i] ^ (i % 37 == 0 ? 3 : 0)); }
        ctx.Write(SnapshotEncoding.Compress(b, true));
        var delta = SnapshotEncoding.CreateXorDelta(a, b);
        var compressed = SnapshotEncoding.Compress(delta, true);
        if (!SnapshotEncoding.ApplyDelta(a, SnapshotEncoding.Decompress(compressed)).AsSpan().SequenceEqual(b)) { throw new InvalidOperationException("XOR/plane round trip differs."); }
        ctx.Write(compressed); ctx.EndStep();
    }

    private static void Run(ScenarioContext ctx)
    {
        var w2 = new PhysicsWorld2D(new WorldOptions2D { WorldId = 95, ThreadCount = ctx.Threads });
        var w3 = new PhysicsWorld3D(new WorldOptions3D { WorldId = 96, ThreadCount = ctx.Threads });
        w2.AddBox(new Vector2F(30, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        w3.AddBox(new Vector3F(30, 1, 30), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        w2.AddCircle(0.5f, new Vector2D(0, 3));
        w3.AddSphere(0.5f, new Vector3D(0, 3, 0));
        w3.AddSoftBall(0.3f, new Vector3D(2, 3, 0), segments: 2);
        var f2 = w2.Fork(); var f3 = w3.Fork();
        for (int i = 0; i < 300; i++)
        {
            w2.Step(1f / 60); w3.Step(1f / 60); f2.Step(1f / 60); f3.Step(1f / 60);
            if (w2.ComputeHash() != f2.ComputeHash() || w3.ComputeHash() != f3.ComputeHash()) { throw new InvalidOperationException("Fork trajectory differs."); }
            ctx.Write((long)f2.ComputeHash()); ctx.Write((long)f3.ComputeHash());
            if ((i & 31) == 0)
            {
                var r2 = w2.CaptureRegion(new Vector2D(-1, -1), new Vector2D(1, 4));
                var r3 = w3.CaptureRegion(new Vector3D(-1, -1, -1), new Vector3D(3, 4, 1));
                ctx.Write((long)r2.ComputeHash()); ctx.Write((long)r3.ComputeHash());
                ctx.Write(SnapshotEncoding.Compress(r2.ToArray(), true)); ctx.Write(SnapshotEncoding.Compress(r3.ToArray(), true));
                if (r2.FindDifferences(RegionSnapshot.Read(r2.ToArray())).Count != 0 || r3.FindDifferences(RegionSnapshot.Read(r3.ToArray())).Count != 0)
                { throw new InvalidOperationException("Region read differs."); }
                if (w2.FindDifferences(f2).Count != 0 || w3.FindDifferences(f3).Count != 0) { throw new InvalidOperationException("Named fork state differs."); }
            }
            ctx.EndStep();
        }
    }
}
