using System.Diagnostics;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace Bar;

internal static class MixedQueryTiming
{
    internal static int Run(Opts options)
    {
        int threads = int.Parse(options.Get("threads", "1"), CultureInfo.InvariantCulture);
        Console.WriteLine("scene,dimension,threads,kind,count,loop_hits,batch_hits,loop_best_us,batch_best_us");
        Run2D(threads); Run3D(threads);
        return 0;
    }

    private static double Best(Action operation)
    {
        for (int i = 0; i < 4; i++) { operation(); }
        long best = long.MaxValue;
        for (int i = 0; i < 3; i++)
        {
            long start = Stopwatch.GetTimestamp(); operation();
            best = Math.Min(best, Stopwatch.GetTimestamp() - start);
        }
        return best * (1e6 / Stopwatch.Frequency);
    }

    private static void Run2D(int threads)
    {
        var scene = Gen.All().First(s => s.Name == "queries");
        var world = SceneBuilder2D.Build(scene, threads).World;
        world.Warmup();
        for (int i = 0; i < scene.Steps; i++) { world.Step(1f / 60f); }
        var query = new P2.PhysicsQuery2D(world);
        foreach (string kind in new[] { "ray", "overlap", "cast" })
        {
            var source = scene.Queries.Where(q => q.Kind == kind).ToArray();
            var inputs = new P2.QueryInput2D[source.Length];
            var hits = new P2.QueryHit2D[source.Length];
            for (int i = 0; i < inputs.Length; i++)
            {
                var p = source[i].P;
                var position = new Vector2D(p[0], p[1]);
                if (kind == "ray") { inputs[i] = P2.QueryInput2D.Ray(position, new Vector2F((float)p[3], (float)p[4]), (float)p[4]); }
                else
                {
                    var shape = Shape2D.CreateCircle(default, (float)p[2]);
                    if (kind == "overlap") { inputs[i] = P2.QueryInput2D.Overlap(shape, position, 0f); }
                    else
                    {
                        var motion = new Vector2F((float)p[3], (float)p[4]);
                        inputs[i] = P2.QueryInput2D.Cast(shape, position, 0f, motion, motion.Length());
                    }
                }
            }
            int loopHits = 0, batchHits = 0;
            void Loop()
            {
                loopHits = 0;
                foreach (var request in source)
                {
                    var p = request.P;
                    var position = new Vector2D(p[0], p[1]);
                    bool found;
                    if (kind == "ray") { found = world.Raycast(position, new Vector2F((float)p[3], (float)p[4]), out _, (float)p[4]); }
                    else if (kind == "overlap") { found = world.OverlapCircle(position, (float)p[2], out _); }
                    else
                    {
                        var motion = new Vector2F((float)p[3], (float)p[4]);
                        found = world.CircleCast(position, (float)p[2], motion, out _, motion.Length());
                    }
                    if (found) { loopHits++; }
                }
            }
            void Batch() => batchHits = query.QueryBatch(inputs, hits);
            double loop = Best(Loop), batch = Best(Batch);
            if (loopHits != batchHits) { throw new InvalidOperationException("Batch and loop hit counts differ."); }
            Console.WriteLine(FormattableString.Invariant($"{scene.Name},2,{threads},{kind},{inputs.Length},{loopHits},{batchHits},{loop:R},{batch:R}"));
        }
    }

    private static void Run3D(int threads)
    {
        var scene = Gen3.All().First(s => s.Name == "queries-3d");
        var world = SceneBuilder3D.Build(scene, threads).World;
        world.Warmup();
        for (int i = 0; i < scene.Steps; i++) { world.Step(1f / 60f); }
        var query = new P3.PhysicsQuery3D(world);
        foreach (string kind in new[] { "ray", "overlap", "cast" })
        {
            var source = scene.Queries.Where(q => q.Kind == kind).ToArray();
            var inputs = new P3.QueryInput3D[source.Length];
            var hits = new P3.QueryHit3D[source.Length];
            for (int i = 0; i < inputs.Length; i++)
            {
                var p = source[i].P;
                var position = new Vector3D(p[0], p[1], p[2]);
                if (kind == "ray") { inputs[i] = P3.QueryInput3D.Ray(position, new Vector3F((float)p[3], (float)p[4], (float)p[5]), (float)p[6]); }
                else
                {
                    var shape = ConvexShape3D.Sphere((float)p[3]);
                    if (kind == "overlap") { inputs[i] = P3.QueryInput3D.Overlap(shape, position, default); }
                    else
                    {
                        var motion = new Vector3F((float)p[4], (float)p[5], (float)p[6]);
                        inputs[i] = P3.QueryInput3D.Cast(shape, position, default, motion, motion.Length());
                    }
                }
            }
            int loopHits = 0, batchHits = 0;
            void Loop()
            {
                loopHits = 0;
                foreach (var request in source)
                {
                    var p = request.P;
                    var position = new Vector3D(p[0], p[1], p[2]);
                    bool found;
                    if (kind == "ray") { found = world.Raycast(position, new Vector3F((float)p[3], (float)p[4], (float)p[5]), out _, (float)p[6]); }
                    else if (kind == "overlap") { found = world.OverlapSphere(position, (float)p[3], out _); }
                    else
                    {
                        var motion = new Vector3F((float)p[4], (float)p[5], (float)p[6]);
                        found = world.SphereCast(position, (float)p[3], motion, out _, motion.Length());
                    }
                    if (found) { loopHits++; }
                }
            }
            void Batch() => batchHits = query.QueryBatch(inputs, hits);
            double loop = Best(Loop), batch = Best(Batch);
            if (loopHits != batchHits) { throw new InvalidOperationException("Batch and loop hit counts differ."); }
            Console.WriteLine(FormattableString.Invariant($"{scene.Name},3,{threads},{kind},{inputs.Length},{loopHits},{batchHits},{loop:R},{batch:R}"));
        }
    }
}
