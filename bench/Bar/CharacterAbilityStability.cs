using System.Diagnostics;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;

namespace Bar;

/// <summary>
/// Stability and timing rows for the character-ability scenes (characters on soft bodies, ropes, ledges and ladders, vehicles and decks; no reference engine has any of them). Metrics over a fixed 600 steps:
/// maxSpeed (m/s, any body or character), nan and escapes (bodies outside the scene's bounds), falls (characters below the floor at any step), grounded (characters standing at the end), standerJitter (mean
/// absolute vertical change per step of the character standing on a soft body over the last 200 steps), curtainDeflection and pusherX (the walked-through hanging rope and how far the pusher got),
/// swingerLanded (the rope swinger let go and stands), grabs, falseGrabs, onTop and ladderOnTop (the climbing course), seatError (largest distance of the driver from the seat), deckDrift and platformDrift
/// (largest drift of a standing passenger in the frame of the barge and of the turning platform).
/// </summary>
internal static class CharacterAbilityStability
{
    static string F(double v) => v.ToString("R", CultureInfo.InvariantCulture);

    /// <summary>Thresholds: our own, 1.5 times what the scenes measured on the worker and on the Apple M2, rounded up (docs/spec/scenes.md).</summary>
    static readonly Dictionary<string, (string Metric, double Min, double Max)[]> Limits = new()
    {
        ["characters-soft-3d"] = [("grounded", 3, 3), ("falls", 0, 0), ("standerJitter", 0, 0.001), ("maxSpeed", 0, 7)],
        ["characters-soft-2d"] = [("grounded", 2, 3), ("falls", 0, 0), ("standerJitter", 0, 0.001), ("maxSpeed", 0, 30)],
        ["characters-ropes-3d"] = [("falls", 0, 0), ("walkerGrounded", 1, 1), ("swingerLanded", 1, 1), ("pusherX", 14.5, 1e9), ("curtainDeflection", 0.1, 3.0), ("maxSpeed", 0, 50)],
        ["climbing-3d"] = [("grabs", 5, 5), ("falseGrabs", 0, 0), ("onTop", 4, 4), ("ladderOnTop", 1, 1), ("falls", 0, 0), ("maxSpeed", 0, 15)],
        ["climbing-2d"] = [("grabs", 2, 2), ("falseGrabs", 0, 0), ("onTop", 2, 2), ("ladderOnTop", 1, 1), ("falls", 0, 0), ("maxSpeed", 0, 15)],
        ["riders-3d"] = [("seatError", 0, 1e-4), ("deckDrift", 0, 1e-3), ("platformDrift", 0, 1e-3), ("falls", 0, 0), ("maxSpeed", 0, 50)],
    };

    internal static int Run(Opts opts)
    {
        string only = opts.Get("scene", "");
        int threads = int.Parse(opts.Get("threads", "1"), CultureInfo.InvariantCulture);
        bool timing = opts.Get("timing", "") != "";
        int fails = 0;
        foreach (var name in CharacterAbilityScenes3D.Names.Concat(CharacterAbilityScenes2D.Names))
        {
            if (only.Length != 0 && only != name) continue;
            fails += name.EndsWith("-3d", StringComparison.Ordinal) ? Run3D(name, threads, timing) : Run2D(name, threads, timing);
        }

        return fails;
    }

    static int Report(string name, int steps, int threads, SortedDictionary<string, double> r, bool timing, List<double> times)
    {
        bool ok = true;
        foreach (var (metric, min, max) in Limits[name])
        {
            if (!r.TryGetValue(metric, out double v) || !(v >= min && v <= max)) ok = false;
        }

        if (!(r["nan"] == 0 && r["escapes"] == 0)) ok = false;
        Console.WriteLine($"{name} steps={steps} threads={threads} " + string.Join(' ', r.Select(kv => kv.Key + "=" + F(kv.Value))) + (ok ? " PASS" : " FAIL"));
        if (timing && times.Count != 0)
        {
            times.Sort();
            Console.WriteLine($"{name} threads={threads} stepMs median={F(times[times.Count / 2])} p90={F(times[(int)(0.9 * (times.Count - 1))])} mean={F(times.Average())}");
        }

        return ok ? 0 : 1;
    }

    static int Run3D(string name, int threads, bool timing)
    {
        var sc = CharacterAbilityScenes3D.Build(name, threads);
        var w = sc.World;
        double maxSpeed = 0, nan = 0, escapes = 0;
        var times = new List<double>();
        var sw = new Stopwatch();
        for (int step = 0; step < sc.Steps; step++)
        {
            sc.Script(step);
            sw.Restart();
            w.Step(CharacterAbilityScenes3D.Dt);
            if (timing) times.Add(sw.Elapsed.TotalMilliseconds);
            sc.Observe(step);
            for (int b = 0; b < w.Core.Bodies.HighWater; b++)
            {
                if (!w.Core.Bodies.IsLive(b)) continue;
                var body = w.GetBody(w.Core.HandleOfBody(b));
                if (body.Type == BodyType.Static) continue;
                var v = body.LinearVelocity; var p = body.Position;
                if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || !double.IsFinite(p.Z) || !float.IsFinite(v.X) || !float.IsFinite(v.Y) || !float.IsFinite(v.Z)) { nan++; continue; }
                maxSpeed = Math.Max(maxSpeed, Math.Sqrt((double)v.X * v.X + (double)v.Y * v.Y + (double)v.Z * v.Z));
                if (Math.Abs(p.X) > 400 || Math.Abs(p.Z) > 400 || p.Y < -12 || p.Y > 60) escapes++;
            }
        }

        var r = new SortedDictionary<string, double>(sc.Metrics) { ["maxSpeed"] = maxSpeed, ["nan"] = nan, ["escapes"] = escapes };
        return Report(name, sc.Steps, threads, r, timing, times);
    }

    static int Run2D(string name, int threads, bool timing)
    {
        var sc = CharacterAbilityScenes2D.Build(name, threads);
        var w = sc.World;
        double maxSpeed = 0, nan = 0, escapes = 0;
        var times = new List<double>();
        var sw = new Stopwatch();
        for (int step = 0; step < sc.Steps; step++)
        {
            sc.Script(step);
            sw.Restart();
            w.Step(CharacterAbilityScenes2D.Dt);
            if (timing) times.Add(sw.Elapsed.TotalMilliseconds);
            sc.Observe(step);
            for (int b = 0; b < w.Core.Bodies.HighWater; b++)
            {
                if (!w.Core.Bodies.IsLive(b)) continue;
                var body = w.GetBody(w.Core.HandleOfBody(b));
                if (body.Type == BodyType.Static) continue;
                var v = body.LinearVelocity; var p = body.Position;
                if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || !float.IsFinite(v.X) || !float.IsFinite(v.Y)) { nan++; continue; }
                maxSpeed = Math.Max(maxSpeed, Math.Sqrt((double)v.X * v.X + (double)v.Y * v.Y));
                if (Math.Abs(p.X) > 400 || p.Y < -12 || p.Y > 60) escapes++;
            }
        }

        var r = new SortedDictionary<string, double>(sc.Metrics) { ["maxSpeed"] = maxSpeed, ["nan"] = nan, ["escapes"] = escapes };
        return Report(name, sc.Steps, threads, r, timing, times);
    }
}
