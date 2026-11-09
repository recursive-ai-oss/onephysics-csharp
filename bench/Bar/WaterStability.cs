#pragma warning disable OPHYS0002, OPHYS0003
using System.Diagnostics;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;

namespace Bar;

/// <summary>
/// Stability and timing rows for the shallow-water wave tanks (no reference engine has shallow water coupled to bodies). Metrics, over a fixed 900 steps:
///  maxSpeed (m/s, any body), nan and escapes (bodies outside the basin), capsized (boats whose up axis is more than 78° from vertical at any step), sunk (boats whose origin falls more than 1.2 m under the starting level),
///  volumeError (the water's volume against what it started with plus the pulses added, relative), lostVolume (water pushed below zero depth, m³ or m²), maxWave (the largest height of the surface above its starting level, m)
///  and waterSpeed (the largest water speed at the end of the run, m/s).
/// </summary>
internal static class WaterStability
{
    static string F(double v) => v.ToString("R", CultureInfo.InvariantCulture);

    internal static int Run(Opts opts)
    {
        string only = opts.Get("scene", "");
        int threads = int.Parse(opts.Get("threads", "1"), CultureInfo.InvariantCulture);
        bool timing = opts.Get("timing", "") != "";
        int fails = 0;
        foreach (var name in WaterScenes.Names)
        {
            if (only.Length != 0 && only != name) continue;
            fails += name.EndsWith("-3d", StringComparison.Ordinal) ? Run3D(threads, timing) : Run2D(threads, timing);
        }

        return fails;
    }

    /// <summary>Indicative timing: a 256 x 256 grid (0.5 m cells) of 3D water with 20 floating bodies, and a 2048-cell planar channel with 20 bodies; median and mean step time over 300 steps after 100 of warm-up, and the water's share of it.</summary>
    internal static int Timing(Opts opts)
    {
        int threads = int.Parse(opts.Get("threads", "1"), CultureInfo.InvariantCulture);
        var w = new RecursiveAI.OnePhysics.Physics3D.PhysicsWorld3D(new RecursiveAI.OnePhysics.Physics3D.WorldOptions3D { ThreadCount = threads, WorldId = 140 });
        w.AddBox(new Vector3F(140f, 1f, 140f), new Vector3D(64, -0.5, 64), type: BodyType.Static);
        var water = w.CreateShallowWater(new RecursiveAI.OnePhysics.Physics3D.ShallowWaterDef { CellsX = 256, CellsZ = 256, CellSize = 0.5f, Level = 2.0, FloorHeight = -1.0, SampleBed = false, Bed = new float[256 * 256] });
        for (int i = 0; i < 20; i++)
        {
            double x = 20 + 18 * (i % 5), z = 20 + 20 * (i / 5);
            if (i % 2 == 0) w.AddBox(new Vector3F(1.5f, 0.6f, 3f), new Vector3D(x, 2.5, z), density: 400f); else w.AddSphere(0.6f, new Vector3D(x, 3.0, z), density: 500f);
        }

        var times = new List<double>(); var sw = new Stopwatch();
        for (int step = 0; step < 400; step++)
        {
            if (step % 100 == 50) water.AddVolume(2, 2, 6, 126, 80f);
            sw.Restart(); w.Step(WaterScenes.Dt); if (step >= 100) times.Add(sw.Elapsed.TotalMilliseconds);
        }

        times.Sort();
        Console.WriteLine($"water-3d-256x256 bodies=20 threads={threads} stepMs median={F(times[times.Count / 2])} p90={F(times[(int)(0.9 * (times.Count - 1))])} mean={F(times.Average())} substeps={w.Engine.Water.LastSubsteps}");

        var w2 = new RecursiveAI.OnePhysics.Physics2D.PhysicsWorld2D(new RecursiveAI.OnePhysics.Physics2D.WorldOptions2D { ThreadCount = threads, WorldId = 141 });
        w2.AddBox(new Vector2F(600f, 1f), new Vector2D(256, -0.5), type: BodyType.Static);
        var water2 = w2.CreateShallowWater(new RecursiveAI.OnePhysics.Physics2D.ShallowWaterDef { Cells = 2048, CellSize = 0.25f, Level = 2.0, FloorHeight = -1.0, SampleBed = false, Bed = new float[2048] });
        for (int i = 0; i < 20; i++) { if (i % 2 == 0) w2.AddBox(new Vector2F(1.5f, 0.6f), new Vector2D(20 + 22 * i, 2.5), density: 400f); else w2.AddCircle(0.6f, new Vector2D(20 + 22 * i, 3.0), density: 500f); }
        times.Clear();
        for (int step = 0; step < 400; step++)
        {
            if (step % 100 == 50) water2.AddVolume(2, 6, 4f);
            sw.Restart(); w2.Step(WaterScenes.Dt); if (step >= 100) times.Add(sw.Elapsed.TotalMilliseconds);
        }

        times.Sort();
        Console.WriteLine($"water-2d-2048 bodies=20 threads={threads} stepMs median={F(times[times.Count / 2])} p90={F(times[(int)(0.9 * (times.Count - 1))])} mean={F(times.Average())}");
        return 0;
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

    static int Run3D(int threads, bool timing)
    {
        var sc = WaterScenes.Build3D(threads);
        var w = sc.World;
        var r = new SortedDictionary<string, double>();
        double maxSpeed = 0, nan = 0, escapes = 0, maxWave = 0;
        var capsized = new HashSet<int>(); var sunk = new HashSet<int>();
        var times = new List<double>();
        var sw = new Stopwatch();
        for (int step = 0; step < sc.Steps; step++)
        {
            sc.Script(step);
            sw.Restart();
            w.Step(WaterScenes.Dt);
            if (timing) times.Add(sw.Elapsed.TotalMilliseconds);
            int hw = w.Core.Bodies.HighWater;
            for (int b = 0; b < hw; b++)
            {
                if (!w.Core.Bodies.IsLive(b)) continue;
                var body = w.GetBody(w.Core.HandleOfBody(b));
                if (body.Type != BodyType.Dynamic) continue;
                var v = body.LinearVelocity; var p = body.Position;
                if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || !double.IsFinite(p.Z) || !float.IsFinite(v.X) || !float.IsFinite(v.Y) || !float.IsFinite(v.Z)) { nan++; continue; }
                maxSpeed = Math.Max(maxSpeed, Math.Sqrt((double)v.X * v.X + (double)v.Y * v.Y + (double)v.Z * v.Z));
                if (p.X < sc.MinX || p.X > sc.MaxX || p.Z < sc.MinZ || p.Z > sc.MaxZ || p.Y < -2 || p.Y > 12) escapes++;
            }

            for (int k = 0; k < sc.Craft.Length; k++)
            {
                var c = sc.Craft[k];
                var up = c.Rotation.Rotate(new Vector3F(0f, 1f, 0f));
                if (up.Y < 0.2f) capsized.Add(k);
                if (c.Position.Y < 0.8) sunk.Add(k);
            }

            if (step % 5 == 0)
            {
                for (int iz = 0; iz < 48; iz += 3) for (int ix = 0; ix < 80; ix += 3) if (sc.Water.CellDepth(ix, iz) > 0.05f) maxWave = Math.Max(maxWave, sc.Water.CellSurface(ix, iz) - 2.0);
            }
        }

        double v1 = sc.Water.Volume, expected = sc.InitialVolume + sc.Added;
        float waterSpeed = 0f;
        for (int iz = 0; iz < 48; iz++) for (int ix = 0; ix < 80; ix++)
            if (sc.Water.CellDepth(ix, iz) > 0.05f && sc.Water.TryGetSample(0.25 + 0.5 * ix, 0.25 + 0.5 * iz, out var s)) waterSpeed = Math.Max(waterSpeed, s.Velocity.Length());
        r["maxSpeed"] = maxSpeed; r["nan"] = nan; r["escapes"] = escapes; r["capsized"] = capsized.Count; r["sunk"] = sunk.Count;
        r["volumeError"] = Math.Abs(v1 - expected) / sc.InitialVolume; r["lostVolume"] = sc.Water.LostVolume; r["maxWave"] = maxWave; r["waterSpeed"] = waterSpeed;
        bool ok = nan == 0 && escapes == 0 && capsized.Count == 0 && sunk.Count == 0 && maxSpeed <= Thresholds.Speed3 && r["volumeError"] <= Thresholds.Volume && r["lostVolume"] <= Thresholds.Lost && maxWave <= Thresholds.Wave3 && waterSpeed <= Thresholds.WaterSpeed3;
        Report(sc.Name, sc.Steps, threads, r, ok, timing, times);
        return ok ? 0 : 1;
    }

    static int Run2D(int threads, bool timing)
    {
        var sc = WaterScenes.Build2D(threads);
        var w = sc.World;
        var r = new SortedDictionary<string, double>();
        double maxSpeed = 0, nan = 0, escapes = 0, maxWave = 0;
        var capsized = new HashSet<int>(); var sunk = new HashSet<int>();
        var times = new List<double>();
        var sw = new Stopwatch();
        for (int step = 0; step < sc.Steps; step++)
        {
            sc.Script(step);
            sw.Restart();
            w.Step(WaterScenes.Dt);
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
                if (p.X < sc.MinX || p.X > sc.MaxX || p.Y < -2 || p.Y > 12) escapes++;
            }

            for (int k = 0; k < sc.Craft.Length; k++)
            {
                var c = sc.Craft[k];
                if (Math.Abs(c.Angle) > 1.2f) capsized.Add(k);
                if (c.Position.Y < 0.8) sunk.Add(k);
            }

            if (step % 5 == 0) { for (int ix = 0; ix < sc.Water.Cells; ix += 2) if (sc.Water.CellDepth(ix) > 0.05f) maxWave = Math.Max(maxWave, sc.Water.CellSurface(ix) - 2.0); }
            if (Environment.GetEnvironmentVariable("WATER_TRACE") is { Length: > 0 } && step % 30 == 0)
                for (int k = 0; k < sc.Craft.Length; k++) { var c = sc.Craft[k]; Console.WriteLine($"  t{step} craft{k} p=({c.Position.X:F2},{c.Position.Y:F2}) angle={c.Angle:F2} v=({c.LinearVelocity.X:F2},{c.LinearVelocity.Y:F2})"); }
        }

        double v1 = sc.Water.Volume, expected = sc.InitialVolume + sc.Added;
        float waterSpeed = 0f;
        for (int ix = 0; ix < sc.Water.Cells; ix++)
            if (sc.Water.CellDepth(ix) > 0.05f && sc.Water.TryGetSample(0.125 + 0.25 * ix, out var s)) waterSpeed = Math.Max(waterSpeed, Math.Abs(s.Velocity));
        r["maxSpeed"] = maxSpeed; r["nan"] = nan; r["escapes"] = escapes; r["capsized"] = capsized.Count; r["sunk"] = sunk.Count;
        r["volumeError"] = Math.Abs(v1 - expected) / sc.InitialVolume; r["lostVolume"] = sc.Water.LostVolume; r["maxWave"] = maxWave; r["waterSpeed"] = waterSpeed;
        bool ok = nan == 0 && escapes == 0 && capsized.Count == 0 && sunk.Count == 0 && maxSpeed <= Thresholds.Speed2 && r["volumeError"] <= Thresholds.Volume && r["lostVolume"] <= Thresholds.Lost && maxWave <= Thresholds.Wave2 && waterSpeed <= Thresholds.WaterSpeed2;
        Report(sc.Name, sc.Steps, threads, r, ok, timing, times);
        return ok ? 0 : 1;
    }

    // Thresholds are 1.5 times the worse of the x64 and arm64 runs, which agree to every digit (see docs/spec/scenes.md).
    static class Thresholds
    {
        internal const double Speed3 = 12, Speed2 = 9, Volume = 1.5e-6, Lost = 1e-9, Wave3 = 0.75, Wave2 = 1.2, WaterSpeed3 = 0.5, WaterSpeed2 = 2.1;
    }
}
