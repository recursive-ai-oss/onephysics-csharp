using System.Diagnostics;
using System.Globalization;

namespace Bar;

/// <summary>
/// Stability and timing rows for the rail and bicycle scenes (rail-3d, rail-2d, bicycles-3d, bicycles-2d; no reference engine has either). Metrics over a fixed 600 steps:
///  maxSpeed (m/s, any body), nan and escapes (bodies outside the scene bounds), derailments (rail-3d: wheelset-steps without tread contact; rail-2d: vehicle-steps with a wheelset off the rail), maxClimbRatio (rail-3d: the largest
///  flange Y/Q on a wheel carrying over 20 kN, against Nadal's limit), fallen (bicycles-3d: riders past the fall angle at the end), flips (bicycles-2d: bikes past the flip angle), maxPenetration (2D), minProgress (the
///  shortest distance any vehicle moved along the course, m) and brokenCouplers.
/// </summary>
internal static class RailBikeStability
{
    static string F(double v) => v.ToString("R", CultureInfo.InvariantCulture);

    internal static int Run(Opts opts)
    {
        string only = opts.Get("scene", "");
        int threads = int.Parse(opts.Get("threads", "1"), CultureInfo.InvariantCulture);
        bool timing = opts.Get("timing", "") != "";
        int fails = 0;
        foreach (var name in RailBikeScenes3D.Names.Concat(RailBikeScenes2D.Names))
        {
            if (only.Length != 0 && only != name) continue;
            fails += RunOne(name, threads, timing);
        }

        return fails;
    }

    // RAILBIKE_TRACE=<every n steps> prints each vehicle's position, speed and lean or track state.
    static void Trace(int step, RailBikeScenes3D.Scene sc)
    {
        if (Environment.GetEnvironmentVariable("RAILBIKE_TRACE") is not { Length: > 0 } t || step % int.Parse(t, CultureInfo.InvariantCulture) != 0) return;
        for (int i = 0; i < sc.Bikes.Length; i++) { var b = sc.Bikes[i]; var p = b.Frame.Position; Console.WriteLine($"  t{step} bike{i} z {p.Z:F1} y {p.Y:F2} x {p.X:F1} v {b.Speed:F2} lean {b.LeanAngle:F2} steer {b.SteerAngle:F2} fell {b.HasFallen}"); }
        for (int i = 0; i < sc.Rail.Length; i++) { var r = sc.Rail[i]; var p = r.Car.Position; Console.WriteLine($"  t{step} rail{i} x {p.X:F1} y {p.Y:F2} z {p.Z:F1} v {r.Speed:F2} onTrack {r.IsOnTrack}"); }
    }

    static void Trace(int step, RailBikeScenes2D.Scene sc)
    {
        if (Environment.GetEnvironmentVariable("RAILBIKE_TRACE") is not { Length: > 0 } t || step % int.Parse(t, CultureInfo.InvariantCulture) != 0) return;
        for (int i = 0; i < sc.Bikes.Length; i++) { var b = sc.Bikes[i]; var p = b.Frame.Position; Console.WriteLine($"  t{step} bike{i} x {p.X:F1} y {p.Y:F2} v {b.Speed:F2} pitch {b.PitchAngle:F2} flipped {b.HasFlipped}"); }
        for (int i = 0; i < sc.Rail.Length; i++) { var r = sc.Rail[i]; var p = r.Car.Position; Console.WriteLine($"  t{step} rail{i} x {p.X:F1} y {p.Y:F2} v {r.Speed:F2} onTrack {r.IsOnTrack}"); }
    }

    static int RunOne(string name, int threads, bool timing)
    {
        var times = new List<double>();
        var sw = new Stopwatch();
        SortedDictionary<string, double> m;
        double extra = 0;
        if (name.EndsWith("3d"))
        {
            var sc = RailBikeScenes3D.Build(name, threads);
            for (int i = 0; i < sc.Steps; i++) { sc.Script(i); sw.Restart(); sc.World.Step(RailBikeScenes3D.Dt); if (timing) times.Add(sw.Elapsed.TotalMilliseconds); sc.Measure(i); Trace(i, sc); }
            m = sc.M; extra = sc.ExpectedFalls;
        }
        else
        {
            var sc = RailBikeScenes2D.Build(name, threads);
            for (int i = 0; i < sc.Steps; i++) { sc.Script(i); sw.Restart(); sc.World.Step(RailBikeScenes2D.Dt); if (timing) times.Add(sw.Elapsed.TotalMilliseconds); sc.Measure(i); Trace(i, sc); }
            m = sc.M;
        }

        var sb = new System.Text.StringBuilder($"{name} threads={threads}");
        foreach (var kv in m) sb.Append($" {kv.Key}={F(kv.Value)}");
        if (extra > 0) sb.Append($" expectedFalls={F(extra)}");
        if (timing) { times.Sort(); sb.Append($" medianMs={F(times[times.Count / 2])} meanMs={F(times.Average())}"); }
        bool ok = Judge(name, m);
        sb.Append(ok ? " PASS" : " FAIL");
        Console.WriteLine(sb.ToString());
        return ok ? 0 : 1;
    }

    // The limits of docs/spec/scenes.md (rail and bicycle rows): every counter below is 0 unless listed, speeds and penetration are
    // maxima, progress is a minimum. A metric the scene does not report is not judged.
    static bool Judge(string name, SortedDictionary<string, double> m)
    {
        bool ok = true;
        void Max(string k, double limit) { if (m.TryGetValue(k, out var v) && !(v <= limit)) ok = false; }
        void Min(string k, double limit) { if (!m.TryGetValue(k, out var v) || !(v > limit)) ok = false; }
        foreach (var k in new[] { "nan", "escapes", "derailments", "brokenCouplers", "flips" }) Max(k, 0);
        switch (name)
        {
            case "rail-3d": Max("fallen", 0); Max("maxSpeed", 25); Max("maxClimbRatio", 0.5); Min("minProgress", 50); break;
            case "bicycles-3d": if (!m.TryGetValue("fallen", out var fallen) || fallen != 2) ok = false; Max("maxPenetration", 0.1); Max("maxSpeed", 10); Min("minProgress", 10); break;
            case "rail-2d": Max("maxSpeed", 17.5); Min("minProgress", 50); break;
            case "bicycles-2d": Max("maxPenetration", 0.05); Max("maxSpeed", 18); Min("minProgress", 10); break;
        }

        return ok;
    }
}
