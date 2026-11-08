using System;
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.HashMatrix;

internal static class EncodingScenarios
{
    [ModuleInitializer]
    internal static void Init() => Scenarios.Register("snapshots/encoding", Run);

    private static void Run(ScenarioContext ctx)
    {
        var w2 = new PhysicsWorld2D(new WorldOptions2D { ThreadCount = ctx.Threads });
        var w3 = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = ctx.Threads });
        w2.AddBox(new Vector2F(1, 1), new Vector2D(0, 4));
        w3.AddBox(new Vector3F(1, 1, 1), new Vector3D(0, 4, 0));
        w3.AddSoftBall(0.5f, new Vector3D(2, 5, 0), segments: 2);
        var base2 = w2.Snapshot();
        var base3 = w3.Snapshot();
        for (int i = 0; i < 12; i++)
        {
            w2.Step(1f / 60); w3.Step(1f / 60);
            foreach (var pair in new[] { (base2, w2.Snapshot()), (base3, w3.Snapshot()) })
            {
                var delta = SnapshotEncoding.CreateDelta(pair.Item1, pair.Item2);
                var compressed = SnapshotEncoding.Compress(pair.Item2);
                var compressedDelta = SnapshotEncoding.Compress(delta);
                if (!SnapshotEncoding.Decompress(compressed).AsSpan().SequenceEqual(pair.Item2) ||
                    !SnapshotEncoding.ApplyDelta(pair.Item1, SnapshotEncoding.Decompress(compressedDelta)).AsSpan().SequenceEqual(pair.Item2))
                { throw new InvalidOperationException("Snapshot encoding round trip differs."); }
                ctx.Write(compressed.Length); ctx.Write(compressed);
                ctx.Write(compressedDelta.Length); ctx.Write(compressedDelta);
            }
            ctx.EndStep();
        }
    }
}
