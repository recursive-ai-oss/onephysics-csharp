using System.Diagnostics;
using System.Linq;

namespace Bar;

// soft3d-timing <threads> [on|off]: median, mean and p99 step time over 300 steps after 60 warm-up steps for the 3D soft-body scenes (indicative; no tiering control, one process). cloth-fold-3d exists only where
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
        Print("cloth-drape-3d " + surface, threads, Time(() => w.Step((float)sc.Dt)));
        var sp = Gen.All().First(s => s.Name == "soft-pile-3d");
        var (w2, _, _) = SceneBuilder3D.Build(sp, threads);
        SceneBuilder3D.AddSofts(sp, w2);
        Print("soft-pile-3d " + surface, threads, Time(() => w2.Step((float)sp.Dt)));
        return RunFold(threads);
    }

    // soft3d-profile <threads> [scene]: per-stage means of the soft step over 600 steps, in windows of 100 steps (a settled body sleeps and stops costing anything, so the windows show where it is awake).
    internal static int Profile(int threads, string scene = "cloth-drape-3d")
    {
        var sc = Gen.All().First(s => s.Name == scene);
        var (w, _, _) = SceneBuilder3D.Build(sc, threads);
        SceneBuilder3D.AddSofts(sc, w);
        w.Profile.TimestampSource = Stopwatch.GetTimestamp;
        string[] names = ["softSetup", "softPredict", "softDetect", "softPairs", "softConstraints", "softContacts", "softPairSolve", "softFinish"];
        int[] idx = [17, 18, 19, 20, 21, 22, 23, 24];
        for (int win = 0; win < 6; win++)
        {
            var acc = new double[names.Length]; double tot = 0;
            for (int i = 0; i < 100; i++)
            {
                long t0 = Stopwatch.GetTimestamp();
                w.Step((float)sc.Dt);
                tot += Stopwatch.GetElapsedTime(t0).TotalMilliseconds;
                var d = w.Profile.Durations;
                for (int k = 0; k < idx.Length; k++) { acc[k] += d[idx[k]] * 1000.0 / Stopwatch.Frequency; }
            }

            Console.WriteLine($"{scene} threads={threads} steps {win * 100}-{win * 100 + 99}: step mean {tot / 100:F4} ms; " + string.Join(" ", names.Select((n, k) => $"{n}={acc[k] / 100:F4}")));
        }

        return 0;
    }

    private static double[] Time(Action step)
    {
        for (int i = 0; i < 60; i++) { step(); }
        var ms = new double[300];
        for (int i = 0; i < ms.Length; i++)
        {
            long t0 = Stopwatch.GetTimestamp();
            step();
            ms[i] = Stopwatch.GetElapsedTime(t0).TotalMilliseconds;
        }

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
        Print("cloth-fold-3d " + _surface, threads, Time(() => w.Step(SoftFracture3DScene.Dt)));
        return 0;
    }
}
