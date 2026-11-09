using System.Diagnostics;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Destruction;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace Bar;

/// <summary>Indicative timing of destruction: one 64-cell fracture (3D and 2D) and the step time of the shattering-crates scene at 1, 8 and all threads. The library reads no clock; this tool does.</summary>
internal static class DestructionTiming
{
    internal static int Run(Opts opts)
    {
        int reps = int.Parse(opts.Get("repeat", "20"), CultureInfo.InvariantCulture);
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"fracture-64-cells 3d box   best {Best(reps, () => Fracture3D(64)):F3} ms"));
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"fracture-64-cells 3d hull  best {Best(reps, () => Fracture3DHull(64)):F3} ms"));
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"fracture-64-cells 2d box   best {Best(reps, () => Fracture2D(64)):F3} ms"));
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"slice 3d box              best {Best(reps, Slice3D):F3} ms"));
        foreach (int t in new[] { 1, 8, Environment.ProcessorCount })
        {
            var times = new List<double>();
            var w = Scene(t);
            for (int i = 0; i < 300; i++)
            {
                long a = Stopwatch.GetTimestamp();
                w.Step(1f / 60);
                if (i >= 20) times.Add(Stopwatch.GetElapsedTime(a).TotalMilliseconds);
            }
            times.Sort();
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"fracture-runtime-3d threads {t,2}  step median {times[times.Count / 2]:F3} ms  p95 {times[(int)(times.Count * 0.95)]:F3} ms  bodies {w.BodyCount} fractures {w.DestructionCounters.Fractures}"));
        }
        return 0;
    }

    private static double Best(int reps, Action a)
    {
        a();
        double best = double.MaxValue;
        for (int i = 0; i < reps; i++) { long t = Stopwatch.GetTimestamp(); a(); best = Math.Min(best, Stopwatch.GetElapsedTime(t).TotalMilliseconds); }
        return best;
    }

    private static void Fracture3D(int cells)
    {
        var w = new P3.PhysicsWorld3D();
        w.AddBox(new(1, 1, 1), new Vector3D(0, 2, 0)).Fracture(new P3.FractureDef { CellCount = cells, Seed = 3 });
    }

    private static void Fracture3DHull(int cells)
    {
        var w = new P3.PhysicsWorld3D();
        var p = new Vector3F[24];
        var rng = new SeedLike(5);
        for (int i = 0; i < p.Length; i++) p[i] = new Vector3F((float)(rng.Next() - 0.5), (float)(rng.Next() - 0.5), (float)(rng.Next() - 0.5));
        w.AddConvexHull(p, new Vector3D(0, 2, 0)).Fracture(new P3.FractureDef { CellCount = cells, Seed = 3 });
    }

    private static void Fracture2D(int cells)
    {
        var w = new P2.PhysicsWorld2D();
        w.AddBox(new(1, 1), new Vector2D(0, 2)).Fracture(new P2.FractureDef { CellCount = cells, Seed = 3 });
    }

    private static void Slice3D()
    {
        var w = new P3.PhysicsWorld3D();
        w.Slice(w.AddBox(new(1, 1, 1), new Vector3D(0, 2, 0)), new Vector3D(0, 2, 0), new Vector3F(1, 1, 0));
    }

    private sealed class SeedLike(ulong s)
    {
        private ulong _s = s;
        public double Next() { _s = unchecked(_s * 6364136223846793005UL + 1442695040888963407UL); return (_s >> 11) * (1.0 / 9007199254740992.0); }
    }

    private static P3.PhysicsWorld3D Scene(int threads)
    {
        var w = new P3.PhysicsWorld3D(new() { ThreadCount = threads });
        w.AddBox(new(80, 1, 80), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        w.Debris = new() { MaxFragments = 300, SleepToStaticSteps = 120 };
        var seed = new SeedLike(3);
        for (int i = 0; i < 24; i++)
        {
            var c = w.AddBox(new(0.8f, 0.8f, 0.8f), new Vector3D(-6 + (i % 6) * 2.4, 2 + (i / 6) * 3 + seed.Next(), -4 + (i / 6) * 2.5 + seed.Next()));
            c.SetFracturable(new P3.FractureDef { CellCount = 12, ImpulseThreshold = 4f, Seed = (ulong)(100 + i), Depth = 1, Pattern = FracturePattern.Clustered });
        }
        return w;
    }
}
