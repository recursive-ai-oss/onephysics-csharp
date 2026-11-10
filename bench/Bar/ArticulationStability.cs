#pragma warning disable OPHYS0002
#pragma warning disable OPHYS0003
using System.Diagnostics;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace Bar;

/// <summary>
/// Stability and timing rows for the articulation scenes (articulation-arm-3d, quadruped-3d, articulation-chain-2d; no reference engine has reduced-coordinate articulations at their defaults). Metrics over a fixed 600 steps:
/// maxSpeed (m/s, any moving body), nan, escapes (bodies outside the scene bounds), tunnelled (dynamic bodies below the floor), maxPenetration (the deepest overlap of any contact, links included), jointError (the largest
/// distance between a link and the place its joint puts it, from the stored poses), boxError (the arm's pushed box against its goal), minTorsoHeight and fallen (the quadrupeds). <c>articulation-timing</c> times 64 quadrupeds
/// on a floor and a 64-link chain with a 1000:1 end mass.
/// </summary>
internal static class ArticulationStability
{
    static string F(double v) => v.ToString("R", CultureInfo.InvariantCulture);

    internal static int Run(Opts opts)
    {
        string only = opts.Get("scene", "");
        int threads = int.Parse(opts.Get("threads", "1"), CultureInfo.InvariantCulture);
        bool timing = opts.Get("timing", "") != "";
        int failures = 0;
        foreach (var name in ArticulationScenes3D.Names.Concat(ArticulationScenes2D.Names))
        {
            if (only.Length != 0 && only != name) continue;
            failures += RunOne(name, threads, timing);
        }

        return failures == 0 ? 0 : 1;
    }

    // Thresholds: 1.5 times the worst of the x64 and arm64 runs (the two agree to the bit), rounded up; the arm's box error and the quadruped's link penetration follow the unit's own targets (1 cm, 2 cm).
    // Always: nan = escapes = tunnelled = 0, jointError at most 1e-6 (by construction it is rounding only).
    static readonly Dictionary<string, (string Metric, double Max)[]> Limits = new()
    {
        ["articulation-arm-3d"] = new[] { ("boxError", 0.007), ("maxSpeed", 0.3), ("linkPenetration", 0.003), ("maxPenetration", 0.002), ("fallen", 0) },
        ["quadruped-3d"] = new[] { ("maxSpeed", 4.2), ("linkPenetration", 0.02), ("maxPenetration", 0.05), ("fallen", 0) },
        ["articulation-chain-2d"] = new[] { ("maxSpeed", 36), ("linkPenetration", 0.03), ("maxPenetration", 0.001) },
    };

    static int RunOne(string name, int threads, bool timing)
    {
        var times = new List<double>();
        var sw = new Stopwatch();
        SortedDictionary<string, double> m;
        if (name.EndsWith("3d"))
        {
            var sc = ArticulationScenes3D.Build(name, threads);
            for (int i = 0; i < sc.Steps; i++) { sc.Script(i); sw.Restart(); sc.World.Step(ArticulationScenes3D.Dt); if (timing) times.Add(sw.Elapsed.TotalMilliseconds); sc.Measure(i); }
            m = sc.M;
        }
        else
        {
            var sc = ArticulationScenes2D.Build(name, threads);
            for (int i = 0; i < sc.Steps; i++) { sc.Script(i); sw.Restart(); sc.World.Step(ArticulationScenes2D.Dt); if (timing) times.Add(sw.Elapsed.TotalMilliseconds); sc.Measure(i); }
            m = sc.M;
        }

        var sb = new System.Text.StringBuilder($"{name} threads={threads}");
        foreach (var kv in m) sb.Append($" {kv.Key}={F(kv.Value)}");
        if (timing) { times.Sort(); sb.Append($" medianMs={F(times[times.Count / 2])} meanMs={F(times.Average())}"); }
        var bad = new List<string>();
        foreach (var k in new[] { "nan", "escapes", "tunnelled" }) { if (m.GetValueOrDefault(k) != 0) { bad.Add(k); } }
        if (m.GetValueOrDefault("jointError") > 1e-6) { bad.Add("jointError"); }
        if (name == "quadruped-3d" && m.GetValueOrDefault("minTorsoHeight") < 0.5) { bad.Add("minTorsoHeight"); }
        if (Limits.TryGetValue(name, out var lim)) { foreach (var (metric, max) in lim) { if (m.GetValueOrDefault(metric) > max) { bad.Add(metric); } } }
        sb.Append(bad.Count == 0 ? " PASS" : " FAIL(" + string.Join(",", bad) + ")");
        Console.WriteLine(sb.ToString());
        return bad.Count == 0 ? 0 : 1;
    }

    /// <summary>Median step time of 64 quadrupeds standing on a floor (3D) and of a 64-link chain with a 1000:1 end mass (2D and 3D), after a warm-up.</summary>
    internal static int Time(Opts opts)
    {
        int threads = int.Parse(opts.Get("threads", "1"), CultureInfo.InvariantCulture);
        int steps = int.Parse(opts.Get("steps", "240"), CultureInfo.InvariantCulture);
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { ThreadCount = threads });
        w.AddBox(new Vector3F(40, 1, 40), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        for (int i = 0; i < 64; i++) { w.AddArticulation(P3.ArticulationDef.Quadruped(), new Vector3D((i % 8) * 1.4 - 5, 0.6, (i / 8) * 1.4 - 5)); }
        Console.WriteLine($"quadrupeds-64-3d threads={threads} medianMs={F(Median(steps, () => w.Step(1f / 60f)))}");
        var chain3 = new P3.PhysicsWorld3D(new P3.WorldOptions3D { ThreadCount = threads });
        chain3.AddArticulation(P3.ArticulationDef.Chain(64, 0.25f, 1f, 1000f), new Vector3D(0, 30, 0));
        Console.WriteLine($"chain-64-3d threads={threads} medianMs={F(Median(steps, () => chain3.Step(1f / 60f)))}");
        var chain2 = new P2.PhysicsWorld2D(new P2.WorldOptions2D { ThreadCount = threads });
        chain2.AddArticulation(P2.ArticulationDef.Chain(64, 0.25f, 1f, 1000f), new Vector2D(0, 30));
        Console.WriteLine($"chain-64-2d threads={threads} medianMs={F(Median(steps, () => chain2.Step(1f / 60f)))}");
        return 0;
    }

    static double Median(int steps, Action step)
    {
        for (int i = 0; i < 30; i++) step();
        var times = new List<double>();
        var sw = new Stopwatch();
        for (int i = 0; i < steps; i++) { sw.Restart(); step(); times.Add(sw.Elapsed.TotalMilliseconds); }
        times.Sort();
        return times[times.Count / 2];
    }
}
