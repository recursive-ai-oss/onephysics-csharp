#pragma warning disable OPHYS0002, OPHYS0003
using System.Diagnostics;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;

namespace Bar;

/// <summary>
/// Stability and timing rows for the boat, hovercraft, glider and thruster scenes (no reference engine has any of them). Metrics, over a fixed 600 steps:
///  maxSpeed (m/s, any body), nan and escapes (bodies outside the scene's bounds), maxPenetration (m, deepest contact of a boat, hovercraft or lander with the fixed scenery),
///  capsized (boats whose up axis is more than 78° from vertical at any step), sunk (boats whose origin falls more than 1.5 m under the water),
///  flipped (hovercraft whose up axis is more than 60° from vertical), minGap (hovercraft: the smallest gap under any fan once the fans have spooled up, m),
///  tumbled (gliders more than 70° from level in pitch or roll), maxSpin (gliders: largest angular speed, rad/s),
///  heightError (thrusters-2d: the root-mean-square distance from the commanded height over the hold, m), landingSpeed (m/s at the last step) and toppled (the lander's tilt at the end beyond 0.3 rad).
/// </summary>
internal static class ActuatorStability
{
    static string F(double v) => v.ToString("R", CultureInfo.InvariantCulture);

    internal static int Run(Opts opts)
    {
        string only = opts.Get("scene", "");
        int threads = int.Parse(opts.Get("threads", "1"), CultureInfo.InvariantCulture);
        bool timing = opts.Get("timing", "") != "";
        int fails = 0;
        foreach (var name in Actuators3DScene.Names.Concat(Actuators2DScene.Names))
        {
            if (only.Length != 0 && only != name) continue;
            fails += name.EndsWith("-3d", StringComparison.Ordinal) ? Run3D(name, threads, timing) : Run2D(name, threads, timing);
        }

        return fails;
    }

    /// <summary>Penetration counts against the fixed scenery only: a crate crushed between two craft is the scene's own doing, not the contact solver's.</summary>
    static bool OtherIsStatic(RecursiveAI.OnePhysics.LowLevel.WorldCore core, RecursiveAI.OnePhysics.LowLevel.ContactView c, int body)
    {
        int other = c.BodyA.Index == body ? c.BodyB.Index : c.BodyA.Index;
        return core.Bodies.TypeOf(other) == BodyType.Static;
    }

    static void Report(string name, int steps, int threads, SortedDictionary<string, double> r, bool ok, bool timing, List<double> times)
    {
        Console.WriteLine($"{name} steps={steps} threads={threads} " + string.Join(' ', r.Select(kv => kv.Key + "=" + F(kv.Value))) + (ok ? " PASS" : " FAIL"));
        if (timing && times.Count != 0)
        {
            times.Sort();
            Console.WriteLine($"{name} threads={threads} stepMs median={F(times[times.Count / 2])} p90={F(times[(int)(0.9 * (times.Count - 1))])} mean={F(times.Average())}");
        }
    }

    static int Run3D(string name, int threads, bool timing)
    {
        var sc = Actuators3DScene.Build(name, threads);
        var w = sc.World;
        var r = new SortedDictionary<string, double>();
        double maxSpeed = 0, nan = 0, escapes = 0, maxPen = 0, maxSpin = 0, minGap = double.MaxValue;
        var capsized = new HashSet<int>(); var sunk = new HashSet<int>(); var flipped = new HashSet<int>(); var tumbled = new HashSet<int>();
        var times = new List<double>();
        var sw = new Stopwatch();
        var craftSlots = new HashSet<int>(sc.Craft.Select(c => c.RawHandle.Index));
        for (int step = 0; step < sc.Steps; step++)
        {
            sc.Script(step);
            sw.Restart();
            w.Step(Actuators3DScene.Dt);
            if (timing) times.Add(sw.Elapsed.TotalMilliseconds);
            int hw = w.Core.Bodies.HighWater;
            for (int b = 0; b < hw; b++)
            {
                if (!w.Core.Bodies.IsLive(b)) continue;
                var body = w.GetBody(w.Core.HandleOfBody(b));
                if (body.Type != BodyType.Dynamic) continue;
                var v = body.LinearVelocity; var p = body.Position;
                if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || !float.IsFinite(v.X) || !float.IsFinite(v.Y) || !float.IsFinite(v.Z)) { nan++; continue; }
                maxSpeed = Math.Max(maxSpeed, Math.Sqrt((double)v.X * v.X + (double)v.Y * v.Y + (double)v.Z * v.Z));
                double lim = name == "gliders-3d" ? 1500 : 100;
                if (Math.Abs(p.X) > lim || Math.Abs(p.Z) > lim || p.Y < (name == "gliders-3d" ? 0 : -10) || p.Y > (name == "gliders-3d" ? 600 : 30)) escapes++;
                if (name != "gliders-3d" && craftSlots.Contains(b)) { foreach (var c in body.Contacts) { if (!OtherIsStatic(w.Core, c, b)) continue; for (int i = 0; i < c.PointCount; i++) maxPen = Math.Max(maxPen, -(double)c.Separation(i)); } }
            }

            if (Environment.GetEnvironmentVariable("ACT_TRACE") is { Length: > 0 } tv && step % 20 == 0)
                for (int k = 0; k < sc.Craft.Length; k++)
                {
                    var c = sc.Craft[k]; var up0 = c.Rotation.Rotate(new Vector3F(0f, 1f, 0f));
                    Console.WriteLine($"  t{step} craft{k} p=({c.Position.X:F1},{c.Position.Y:F2},{c.Position.Z:F1}) v=({c.LinearVelocity.X:F1},{c.LinearVelocity.Y:F1},{c.LinearVelocity.Z:F1}) up.y={up0.Y:F2} w=({c.AngularVelocity.X:F1},{c.AngularVelocity.Y:F1},{c.AngularVelocity.Z:F1})" + (k < sc.Hovers.Length ? $" gaps={sc.Hovers[k].GetFanGap(0):F2},{sc.Hovers[k].GetFanGap(1):F2},{sc.Hovers[k].GetFanGap(2):F2},{sc.Hovers[k].GetFanGap(3):F2}" : ""));
                }

            for (int k = 0; k < sc.Craft.Length; k++)
            {
                var c = sc.Craft[k];
                var up = c.Rotation.Rotate(new Vector3F(0f, 1f, 0f));
                if (name == "boats-3d")
                {
                    if (up.Y < 0.2f) capsized.Add(k);
                    if (c.Position.Y < -1.5) sunk.Add(k);
                }
                else if (name == "hovercraft-3d")
                {
                    if (up.Y < 0.5f) flipped.Add(k);
                    if (step >= 90) { for (int i = 0; i < sc.Hovers[k].FanCount; i++) minGap = Math.Min(minGap, sc.Hovers[k].GetFanGap(i)); }
                }
                else
                {
                    var f = c.Rotation.Rotate(new Vector3F(0f, 0f, 1f));
                    var rt = c.Rotation.Rotate(new Vector3F(1f, 0f, 0f));
                    if (Math.Abs(f.Y) > 0.94f || Math.Abs(rt.Y) > 0.94f) tumbled.Add(k);
                    var wv = c.AngularVelocity;
                    maxSpin = Math.Max(maxSpin, Math.Sqrt((double)wv.X * wv.X + (double)wv.Y * wv.Y + (double)wv.Z * wv.Z));
                }
            }
        }

        r["maxSpeed"] = maxSpeed; r["nan"] = nan; r["escapes"] = escapes;
        bool ok = nan == 0 && escapes == 0;
        switch (name)
        {
            case "boats-3d":
                r["maxPenetration"] = maxPen; r["capsized"] = capsized.Count; r["sunk"] = sunk.Count;
                ok &= maxSpeed <= 12 && maxPen <= 0.1 && capsized.Count == 0 && sunk.Count == 0;
                break;
            case "hovercraft-3d":
                r["maxPenetration"] = maxPen; r["flipped"] = flipped.Count; r["minGap"] = minGap;
                ok &= maxSpeed <= 10 && maxPen <= 0.07 && flipped.Count == 0 && minGap >= 0.07;
                break;
            default:
                r["tumbled"] = tumbled.Count; r["maxSpin"] = maxSpin;
                ok &= maxSpeed <= 27 && tumbled.Count == 0 && maxSpin <= 3.5;
                break;
        }

        Report(name, sc.Steps, threads, r, ok, timing, times);
        return ok ? 0 : 1;
    }

    static int Run2D(string name, int threads, bool timing)
    {
        var sc = Actuators2DScene.Build(name, threads);
        var w = sc.World;
        var r = new SortedDictionary<string, double>();
        double maxSpeed = 0, nan = 0, escapes = 0, maxPen = 0, heightSq = 0; int heightN = 0;
        var capsized = new HashSet<int>(); var sunk = new HashSet<int>();
        var times = new List<double>();
        var sw = new Stopwatch();
        var craftSlots = new HashSet<int>(sc.Craft.Select(c => c.RawHandle.Index));
        float lastLandingSpeed = 0f;
        for (int step = 0; step < sc.Steps; step++)
        {
            sc.Script(step);
            sw.Restart();
            w.Step(Actuators2DScene.Dt);
            if (timing) times.Add(sw.Elapsed.TotalMilliseconds);
            int hw = w.Core.Bodies.HighWater;
            for (int b = 0; b < hw; b++)
            {
                if (!w.Core.Bodies.IsLive(b)) continue;
                var body = w.GetBody(w.Core.HandleOfBody(b));
                if (body.Type != BodyType.Dynamic) continue;
                var v = body.LinearVelocity; var p = body.Position;
                if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || !float.IsFinite(v.X) || !float.IsFinite(v.Y)) { nan++; continue; }
                maxSpeed = Math.Max(maxSpeed, Math.Sqrt((double)v.X * v.X + (double)v.Y * v.Y));
                if (Math.Abs(p.X) > 100 || p.Y < -10 || p.Y > 80) escapes++;
                if (craftSlots.Contains(b)) { foreach (var c in w.GetContacts(body)) { if (!OtherIsStatic(w.Core, c, b)) continue; for (int i = 0; i < c.PointCount; i++) maxPen = Math.Max(maxPen, -(double)c.Separation(i)); } }
            }

            if (name == "boats-2d")
            {
                for (int k = 0; k < sc.Craft.Length; k++)
                {
                    var c = sc.Craft[k];
                    if (Math.Abs(c.Angle) > 1.2f) capsized.Add(k);
                    if (c.Position.Y < -1.5) sunk.Add(k);
                }
            }
            else
            {
                var l = sc.Craft[0];
                if (step >= 150 && step < 300) { double e = l.Position.Y - sc.TargetHeight; heightSq += e * e; heightN++; }
                lastLandingSpeed = (float)Math.Sqrt((double)l.LinearVelocity.X * l.LinearVelocity.X + (double)l.LinearVelocity.Y * l.LinearVelocity.Y);
            }
        }

        r["maxSpeed"] = maxSpeed; r["nan"] = nan; r["escapes"] = escapes; r["maxPenetration"] = maxPen;
        bool ok = nan == 0 && escapes == 0;
        if (name == "boats-2d")
        {
            r["capsized"] = capsized.Count; r["sunk"] = sunk.Count;
            ok &= maxSpeed <= 6 && maxPen <= 0.05 && capsized.Count == 0 && sunk.Count == 0;
        }
        else
        {
            double heightError = Math.Sqrt(heightSq / Math.Max(1, heightN));
            double tilt = Math.Abs(sc.Craft[0].Angle);
            r["heightError"] = heightError; r["landingSpeed"] = lastLandingSpeed; r["toppled"] = tilt > 0.3 ? 1 : 0; r["finalHeight"] = sc.Craft[0].Position.Y;
            ok &= maxSpeed <= 20 && maxPen <= 0.2 && heightError <= 0.5 && lastLandingSpeed <= 1.0 && tilt <= 0.3;
        }

        Report(name, sc.Steps, threads, r, ok, timing, times);
        return ok ? 0 : 1;
    }
}
