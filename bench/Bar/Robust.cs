namespace Bar;

/// <summary>Order statistics behind the stability metrics. A pile has chaotic contact order: a rounding-sized change moves which body is deepest, fastest or flung furthest in one frame, without the pile being any less stable. The robust metrics therefore take a high percentile across bodies (or pairs) and a median across a short window of frames. Real instability (an explosion, sinking, sustained shaking) moves the whole distribution and still shows.</summary>
static class Robust
{
    /// <summary>Frames (or frame-to-frame intervals) in the window the robust metrics look at, counted back from the end of the run.</summary>
    public const int Window = 3;

    /// <summary>Percentile of the speeds that `jitter` reports (across dynamic bodies).</summary>
    public const double JitterQuantile = 0.99;

    /// <summary>Percentile of the per-body kinetic energy that `restKE` reports.</summary>
    public const double RestKeQuantile = 0.90;

    /// <summary>Percentile of the overlapping pairs' depths that `finalPenetration` reports.</summary>
    public const double PenetrationQuantile = 0.95;

    /// <summary>How far outside `param container` a body may be before it counts as escaped, metres: the walls' thickness.</summary>
    public const double DefaultEscapeMargin = 0.5;

    /// <summary>The margin the scene asks for with `param escapeMargin`, else <see cref="DefaultEscapeMargin"/>. Scenes where a heavy impact legitimately throws bodies out of the container ask for a margin wider than any legitimate throw.</summary>
    public static double EscapeMargin(Scene s) =>
        s.Params.TryGetValue("escapeMargin", out var m) ? double.Parse(m[0], System.Globalization.CultureInfo.InvariantCulture) : DefaultEscapeMargin;

    /// <summary>Nearest-rank percentile (q in 0..1) of the values; 0 for an empty set. Not interpolated, so the result is always one of the inputs.</summary>
    public static double Percentile(double[] values, double q)
    {
        if (values.Length == 0) return 0;
        var v = (double[])values.Clone();
        Array.Sort(v);
        int rank = (int)Math.Ceiling(q * v.Length); // 1-based nearest rank
        return v[Math.Clamp(rank, 1, v.Length) - 1];
    }

    /// <summary>Median (the lower middle value for an even count); 0 for an empty set.</summary>
    public static double Median(IEnumerable<double> values)
    {
        var v = values.ToArray();
        if (v.Length == 0) return 0;
        Array.Sort(v);
        return v[(v.Length - 1) / 2];
    }

    /// <summary>The depth of the deepest pair of a frame (pair depths are sorted deepest first); 0 when nothing overlaps.</summary>
    public static double Deepest(double[] depthsDescending) => depthsDescending.Length == 0 ? 0 : depthsDescending[0];

    /// <summary>`finalPenetration`: the median over the last frames of the 95th-percentile depth of the overlapping pairs.</summary>
    public static double FinalPenetration(List<double[]> depthsPerFrame) =>
        Median(depthsPerFrame.TakeLast(Window).Select(d => Percentile(d.Reverse().ToArray(), PenetrationQuantile)));

    /// <summary>`jitter`: over the last intervals between dumps, the 99th-percentile mean speed (distance moved over the interval) of the dynamic bodies; the median of those over the window.</summary>
    public static double Jitter(int frameCount, Func<int, double> stepOf, double dt, int[] dyn, Func<int, int, int, double> distance)
    {
        var perInterval = new List<double>();
        for (int k = Math.Max(1, frameCount - Window); k < frameCount; k++)
        {
            double secs = (stepOf(k) - stepOf(k - 1)) * dt;
            perInterval.Add(Percentile(dyn.Select(i => distance(k - 1, k, i) / secs).ToArray(), JitterQuantile));
        }
        return Median(perInterval);
    }

    /// <summary>`restKE`: the median over the last frames of the 90th-percentile specific kinetic energy of the dynamic bodies. A few bodies rolling on without rolling resistance (heads of a ragdoll pile, say) do not make a resting pile unstable, but a pile in which a tenth of the bodies still moves does.</summary>
    public static double RestKe(int frameCount, int[] dyn, Func<int, int, double> specificKe) =>
        Median(Enumerable.Range(Math.Max(0, frameCount - Window), Math.Min(Window, frameCount)).Select(f => Percentile(dyn.Select(i => specificKe(f, i)).ToArray(), RestKeQuantile)));

    /// <summary>Checks the order statistics and the window rules against hand-worked answers; returns the number of failures.</summary>
    public static int SelfTest()
    {
        int bad = 0;
        void Check(string what, double got, double want) { if (Math.Abs(got - want) > 1e-12) { Console.Error.WriteLine($"selftest robust {what}: got {got}, want {want}"); bad++; } }
        double[] hundred = Enumerable.Range(1, 100).Select(x => (double)x).ToArray();
        Check("p99 of 1..100", Percentile(hundred, 0.99), 99);
        Check("p90 of 1..100", Percentile(hundred, 0.90), 90);
        Check("p95 of one value", Percentile([7], 0.95), 7);
        Check("percentile of nothing", Percentile([], 0.5), 0);
        Check("median odd", Median([5, 1, 3]), 3);
        Check("median even is the lower middle", Median([4, 1, 3, 2]), 2);

        // 100 bodies, 5 dumps 60 steps apart. One body jumps 6 m in the last interval (6 m/s), the rest creep 0.06 m (0.06 m/s): a single stray body must not set the jitter.
        int[] dyn = Enumerable.Range(0, 100).ToArray();
        double Pos(int frame, int body) => frame == 4 && body == 0 ? 0.06 * frame + 6 : 0.06 * frame;
        double stray = Jitter(5, k => k * 60, 1.0 / 60, dyn, (a, b, i) => Math.Abs(Pos(b, i) - Pos(a, i)));
        Check("jitter ignores one stray body", stray, 0.06);
        // The same pile with 5 bodies moving at 6 m/s in every interval is shaking: the 99th percentile sees it.
        double shaking = Jitter(5, k => k * 60, 1.0 / 60, dyn, (a, b, i) => i < 5 ? 6 : 0.06);
        Check("jitter sees a shaking pile", shaking, 6);

        // Rest energy: one rolling body in a hundred is ignored, ten are not.
        Check("restKE ignores one roller", RestKe(4, dyn, (f, i) => i == 0 ? 20 : 0), 0);
        Check("restKE sees ten movers", RestKe(4, dyn, (f, i) => i < 10 ? 20 : 0), 0); // exactly 10% moving: the 90th percentile is still a resting body
        Check("restKE sees eleven movers", RestKe(4, dyn, (f, i) => i < 11 ? 20 : 0), 20);

        // Final penetration: the median over the last three frames of the 95th percentile depth, deepest first in the input.
        double[] Depths(double deepest) => Enumerable.Range(0, 100).Select(k => k == 0 ? deepest : 0.01).ToArray();
        Check("penetration ignores one deep pair", FinalPenetration([Depths(0.5), Depths(0.5), Depths(0.5)]), 0.01);
        Check("penetration of no overlaps", FinalPenetration([[], [], []]), 0);
        return bad;
    }
}
