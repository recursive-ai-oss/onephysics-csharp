#pragma warning disable OPHYS0002, OPHYS0003
using System.Diagnostics;
using System.Globalization;

namespace Bar;

/// <summary>
/// Stability and timing rows for the terrain-edit scenes (bodies on a signed distance field terrain, a voxel terrain dug under bodies and characters, a bitmask terrain carved by projectiles; no
/// reference engine has any of them). Each plays 600 steps and prints the scene's own measurements against its limits (docs/spec/scenes.md).
/// </summary>
internal static class TerrainEditStability
{
    static string F(double v) => v.ToString("R", CultureInfo.InvariantCulture);

    internal static int Run(Opts opts)
    {
        string only = opts.Get("scene", "");
        int threads = int.Parse(opts.Get("threads", "1"), CultureInfo.InvariantCulture);
        bool timing = opts.Get("timing", "") != "";
        int fails = 0;
        foreach (var name in TerrainEditScenes.Names)
        {
            if (only.Length != 0 && only != name) continue;
            var s = TerrainEditScenes.Build(name, threads);
            var times = new List<double>(s.Steps);
            for (int i = 0; i < s.Steps; i++)
            {
                s.Script(i);
                long t0 = Stopwatch.GetTimestamp();
                s.Step();
                if (timing) times.Add(Stopwatch.GetElapsedTime(t0).TotalMilliseconds);
                s.Observe(i);
            }

            bool ok = true;
            foreach (var (metric, min, max) in TerrainEditScenes.Limits[name])
                if (!s.Metrics.TryGetValue(metric, out double v) || !(v >= min && v <= max)) ok = false;
            Console.WriteLine($"{name} steps={s.Steps} threads={threads} " + string.Join(' ', s.Metrics.Select(kv => kv.Key + "=" + F(kv.Value))) + (ok ? " PASS" : " FAIL"));
            if (timing && times.Count != 0)
            {
                times.Sort();
                Console.WriteLine($"{name} threads={threads} stepMs median={F(times[times.Count / 2])} p90={F(times[(int)(0.9 * (times.Count - 1))])} mean={F(times.Average())}");
            }

            fails += ok ? 0 : 1;
        }

        return fails;
    }
}
