namespace Bar;

// Prints one stability row per 2D soft-body scene (see docs/spec/scenes.md). Exit code 1 when a limit is broken.
internal static class SoftStability2D
{
    private static readonly (string Scene, double Speed, double Pen, double Area, double Overlap, double Final)[] Limits =
    {
        ("soft-pile-2d", 25.0, 0.08, 0.05, 0.03, 0.5),
        ("soft-rigid-2d", 15.0, 0.05, 0.02, 0.10, 1.5),
        ("balloon-2d", 20.0, 0.06, 0.02, 0.05, 1.0),
        ("plastic-2d", 8.0, 0.03, 0.05, 0.01, 0.3),
    };

    internal static int Run()
    {
        int bad = 0;
        foreach (var l in Limits)
        {
            var m = SoftBodies2DScene.Measure(l.Scene);
            bool ok = m.NaN == 0 && m.Escapes == 0 && m.MaxSpeed <= l.Speed && m.MaxPenetration <= l.Pen && m.AreaError <= l.Area && m.MaxSoftOverlap <= l.Overlap && m.FinalSpeed99 <= l.Final;
            Console.WriteLine(SoftBodies2DScene.Format(m) + (ok ? " PASS" : " FAIL"));
            if (!ok) { bad++; }
        }

        return bad == 0 ? 0 : 1;
    }

    // soft2d-timing <threads>: median and mean step time over 300 steps after 60 warm-up steps, per scene (indicative; there is no managed 2D reference with soft bodies).
    internal static int Time(int threads)
    {
        foreach (var scene in SoftBodies2DScene.Names)
        {
            var w = SoftBodies2DScene.Build(scene, threads);
            for (int i = 0; i < 60; i++) { w.Step(SoftBodies2DScene.Dt); }
            var ms = new double[300];
            for (int i = 0; i < ms.Length; i++)
            {
                long t0 = System.Diagnostics.Stopwatch.GetTimestamp();
                w.Step(SoftBodies2DScene.Dt);
                ms[i] = System.Diagnostics.Stopwatch.GetElapsedTime(t0).TotalMilliseconds;
            }

            double mean = 0; foreach (var v in ms) { mean += v; }
            Array.Sort(ms);
            Console.WriteLine($"{scene} threads={threads} median={ms[150]:F3} ms mean={mean / ms.Length:F3} ms p99={ms[297]:F3} ms");
        }

        return 0;
    }
}
