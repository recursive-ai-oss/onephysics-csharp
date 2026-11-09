using System.Diagnostics;
using System.Linq;

namespace Bar;

// soft3d-timing <threads>: median, mean and p99 step time over 300 steps after 60 warm-up steps for the 3D soft-body scenes (indicative; no tiering control, one process). cloth-fold-3d exists only where
// tests/Common/SoftFracture3DScene.cs does.
internal static class SoftTiming3D
{
    internal static int Run(int threads, string surface = "default")
    {
        if (surface == "on") { SceneBuilder3D.SurfaceOverride = true; } else if (surface == "off") { SceneBuilder3D.SurfaceOverride = false; }
        _fold = surface != "off"; _surface = surface;
        var sc = Gen.All().First(s => s.Name == "cloth-drape-3d");
        var (w, _, _) = SceneBuilder3D.Build(sc, threads);
        SceneBuilder3D.AddSofts(sc, w);
        Print("cloth-drape-3d " + surface, threads, Time(w, () => w.Step((float)sc.Dt)));
        var sp = Gen.All().First(s => s.Name == "soft-pile-3d");
        var (w2, _, _) = SceneBuilder3D.Build(sp, threads);
        SceneBuilder3D.AddSofts(sp, w2);
        Print("soft-pile-3d " + surface, threads, Time(w2, () => w2.Step((float)sp.Dt)));
        return RunFold(threads);
    }

    private static readonly string[] StageNames = ["softSetup", "softPredict", "softDetect", "softPairs", "softConstraints", "softFinish", "softEnd"];
    private static readonly int[] StageIdx = [17, 18, 19, 20, 21, 24, 25];

    private static double[] Time(RecursiveAI.OnePhysics.Physics3D.PhysicsWorld3D w, Action step)
    {
        for (int i = 0; i < 60; i++) { step(); }
        var ms = new double[300];
        var st = new double[StageIdx.Length][];
        for (int k = 0; k < st.Length; k++) { st[k] = new double[300]; }
        w.Profile.TimestampSource = Stopwatch.GetTimestamp;
        int gc0 = GC.CollectionCount(0);
        var sum = new double[300];
        for (int i = 0; i < ms.Length; i++)
        {
            long t0 = Stopwatch.GetTimestamp();
            step();
            ms[i] = Stopwatch.GetElapsedTime(t0).TotalMilliseconds;
            var du = w.Profile.Durations;
            for (int k = 0; k < st.Length; k++) { st[k][i] = du[StageIdx[k]] * 1000.0 / Stopwatch.Frequency; sum[i] += st[k][i]; }
        }

        w.Profile.TimestampSource = null;
        Console.WriteLine($"  softSum median={sum.OrderBy(x => x).ElementAt(150):F3} gen0={GC.CollectionCount(0) - gc0}");
        Console.WriteLine("  stages(median ms): " + string.Join(" ", StageNames.Select((n, k) => n + "=" + st[k].OrderBy(x => x).ElementAt(150).ToString("F3"))));

        return ms;
    }

    private static void Print(string scene, int threads, double[] ms)
    {
        double mean = ms.Average();
        var s = ms.OrderBy(x => x).ToArray();
        Console.WriteLine($"{scene} threads={threads} median={s[150]:F3} ms mean={mean:F3} ms p99={s[297]:F3} ms");
    }

    private static bool _fold = true;
    private static string _surface = "default";

    private static int RunFold(int threads)
    {
        var w = SoftFracture3DScene.Build("cloth-fold-3d", threads, _fold);
        Print("cloth-fold-3d " + _surface, threads, Time(w, () => w.Step(SoftFracture3DScene.Dt)));
        return 0;
    }
}
