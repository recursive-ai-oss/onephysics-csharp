#pragma warning disable OPHYS0002, OPHYS0003
using System.Diagnostics;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace Bar;

/// <summary>
/// Stability and timing rows for world scale: streamed tiles in 3D and 2D (a moving window over a 16-tile field with bodies crossing seams), simulation level of detail (box walls with a walking focus) and local time scale (a slow-motion
/// bubble beside normal time). No reference engine has these features, so every scene is gated on its own thresholds. The streaming rows also run the same scene on one unsplit field: seam steps must be no worse than there.
/// </summary>
internal static class WorldScaleStability
{
    static string F(double v) => v.ToString("R", CultureInfo.InvariantCulture);

    internal static int Run(Opts opts)
    {
        string only = opts.Get("scene", "");
        int threads = int.Parse(opts.Get("threads", "1"), CultureInfo.InvariantCulture);
        int fails = 0;
        if (only is "" or "streaming-3d") fails += Streaming3D(threads);
        if (only is "" or "streaming-2d") fails += Streaming2D(threads);
        if (only is "" or "lod-3d") fails += Lod3D(threads);
        if (only is "" or "timescale-2d") fails += TimeScale2D(threads);
        return fails;
    }

    static void Report(string name, int steps, int threads, SortedDictionary<string, double> r, bool ok)
        => Console.WriteLine($"{name} steps={steps} threads={threads} " + string.Join(' ', r.Select(kv => kv.Key + "=" + F(kv.Value))) + (ok ? " PASS" : " FAIL"));

    static int Streaming3D(int threads)
    {
        var tiled = WorldScaleScenes.BuildStreaming3D(threads);
        var whole = WorldScaleScenes.BuildStreaming3D(threads, 1401, true);
        for (int i = 0; i < tiled.Steps; i++) { tiled.Step(i); whole.Step(i); }
        var r = new SortedDictionary<string, double>
        {
            ["maxSpeed"] = tiled.MaxSpeed, ["nan"] = tiled.NonFinite, ["tunnelled"] = tiled.Tunnelled, ["escapes"] = tiled.Escapes, ["seamStep"] = tiled.SeamStep, ["unsplitSeamStep"] = whole.SeamStep,
            ["loads"] = tiled.Loads, ["unloads"] = tiled.Unloads, ["maxLoaded"] = tiled.MaxLoaded, ["failedLoads"] = tiled.World.FailedTileLoads,
        };
        bool ok = tiled.NonFinite == 0 && tiled.Tunnelled == 0 && tiled.Escapes == 0 && tiled.World.FailedTileLoads == 0 && tiled.MaxSpeed <= Th.StreamSpeed3 && tiled.SeamStep <= whole.SeamStep * 1.25 + 0.1 && tiled.MaxLoaded <= 30;
        Report("streaming-3d", tiled.Steps, threads, r, ok);
        return ok ? 0 : 1;
    }

    static int Streaming2D(int threads)
    {
        var tiled = WorldScaleScenes.BuildStreaming2D(threads);
        var whole = WorldScaleScenes.BuildStreaming2D(threads, 1411, true);
        for (int i = 0; i < tiled.Steps; i++) { tiled.Step(i); whole.Step(i); }
        var r = new SortedDictionary<string, double>
        {
            ["maxSpeed"] = tiled.MaxSpeed, ["nan"] = tiled.NonFinite, ["tunnelled"] = tiled.Tunnelled, ["seamStep"] = tiled.SeamStep, ["unsplitSeamStep"] = whole.SeamStep,
            ["loads"] = tiled.Loads, ["unloads"] = tiled.Unloads, ["maxLoaded"] = tiled.MaxLoaded, ["failedLoads"] = tiled.World.FailedTileLoads,
        };
        bool ok = tiled.NonFinite == 0 && tiled.Tunnelled == 0 && tiled.World.FailedTileLoads == 0 && tiled.MaxSpeed <= Th.StreamSpeed2 && tiled.SeamStep <= whole.SeamStep * 1.25 + 0.1;
        Report("streaming-2d", tiled.Steps, threads, r, ok);
        return ok ? 0 : 1;
    }

    static int Lod3D(int threads)
    {
        var lod = WorldScaleScenes.BuildLod3D(threads);
        var full = WorldScaleScenes.BuildLod3D(threads, 1421, false, false);
        for (int i = 0; i < lod.Steps; i++) { lod.Step(i); lod.Measure(); full.World.Step(WorldScaleScenes.Dt); full.Measure(); }
        double drift = 0;
        for (int i = 0; i < lod.Bodies.Count; i++) { var a = lod.Bodies[i].Position; var b = full.Bodies[i].Position; drift = Math.Max(drift, Math.Abs(a.X - b.X) + Math.Abs(a.Y - b.Y) + Math.Abs(a.Z - b.Z)); }
        var r = new SortedDictionary<string, double>
        {
            ["maxSpeed"] = lod.MaxSpeed, ["nan"] = lod.NonFinite, ["maxDrift"] = lod.MaxDrift, ["driftFromFullRate"] = drift, ["asleepIslands"] = lod.World.Counters.AsleepIslands, ["fullRateAsleepIslands"] = full.World.Counters.AsleepIslands,
        };
        bool ok = lod.NonFinite == 0 && lod.MaxSpeed <= Th.LodSpeed && lod.MaxDrift <= Th.LodDrift && drift <= Th.LodFullDrift && lod.World.Counters.AsleepIslands >= 20;
        Report("lod-3d", lod.Steps, threads, r, ok);
        return ok ? 0 : 1;
    }

    static int TimeScale2D(int threads)
    {
        var s = WorldScaleScenes.BuildTimeScale2D(threads);
        for (int i = 0; i < s.Steps; i++) { s.Step(i); s.Measure(); }
        double bubbleDrift = 0, normalDrift = 0;
        // boxes were added two at a time: even entries stand in normal time, odd ones in the bubble
        for (int i = 0; i < 42; i++)
        {
            var p = s.Bodies[i].Position;
            int level = 0, idx = i / 2, left = 6;
            while (idx >= left) { idx -= left; left--; level++; }
            double x0 = (i % 2 == 0 ? -14 : 24) + idx * 1.05 + level * 0.525, y0 = 0.5 + level * 1.01;
            double d = Math.Abs(p.X - x0) + Math.Abs(p.Y - y0);
            if (i % 2 == 0) normalDrift = Math.Max(normalDrift, d); else bubbleDrift = Math.Max(bubbleDrift, d);
        }

        var r = new SortedDictionary<string, double> { ["maxSpeed"] = s.MaxSpeed, ["nan"] = s.NonFinite, ["normalPyramidDrift"] = normalDrift, ["bubblePyramidDrift"] = bubbleDrift, ["ballX"] = s.Ball.Position.X };
        bool ok = s.NonFinite == 0 && s.MaxSpeed <= Th.ScaleSpeed && normalDrift <= Th.NormalDrift && bubbleDrift <= Th.BubbleDrift && s.Ball.Position.X > 15;
        Report("timescale-2d", s.Steps, threads, r, ok);
        return ok ? 0 : 1;
    }

    /// <summary>Indicative timing: loading and unloading a tile of 64 x 64 cells in a world with 24 tiles built, and the step cost of a level-of-detail world against full rate.</summary>
    internal static int Timing(Opts opts)
    {
        int threads = int.Parse(opts.Get("threads", "1"), CultureInfo.InvariantCulture);
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { ThreadCount = threads, WorldId = 1440 });
        for (int tx = 0; tx < 5; tx++) { for (int tz = 0; tz < 5; tz++) { if (tx == 2 && tz == 2) continue; w.LoadTile(new TileKey(tx, tz), WorldScaleScenes.Tile3D(tx, tz, true, 64)); } }
        w.FlushTiles();
        var load = new List<double>(); var unload = new List<double>(); var build = new List<double>();
        var sw = new Stopwatch();
        for (int rep = 0; rep < 24; rep++)
        {
            sw.Restart(); var def = WorldScaleScenes.Tile3D(2, 2, true, 64); build.Add(sw.Elapsed.TotalMilliseconds);
            sw.Restart(); w.LoadTile(new TileKey(2, 2), def); w.FlushTiles(); load.Add(sw.Elapsed.TotalMilliseconds);
            w.Step(WorldScaleScenes.Dt);
            sw.Restart(); w.UnloadTile(new TileKey(2, 2)); w.FlushTiles(); unload.Add(sw.Elapsed.TotalMilliseconds);
        }

        {
            var tdef = WorldScaleScenes.Tile3D(2, 2, true, 64); var part = tdef.HeightFields[0]; var create = new List<double>();
            for (int rep = 0; rep < 20; rep++) { sw.Restart(); _ = RecursiveAI.OnePhysics.LowLevel.HeightField3D.Create(part.Heights, part.SamplesX, part.SamplesZ, part.Scale, default, default, part.Edges); create.Add(sw.Elapsed.TotalMilliseconds); }
            create.RemoveRange(0, 5); create.Sort();
            Console.WriteLine($"height-field-65x65 createMs median={F(create[create.Count / 2])}");
        }

        load.RemoveRange(0, 4); unload.RemoveRange(0, 4); load.Sort(); unload.Sort(); build.Sort();
        Console.WriteLine($"tile-64x64 threads={threads} loadMs median={F(load[load.Count / 2])} max={F(load[^1])} unloadMs median={F(unload[unload.Count / 2])} max={F(unload[^1])} defBuildMs median={F(build[build.Count / 2])}");

        double Step(bool lod)
        {
            var s = WorldScaleScenes.BuildLod3D(threads, 1441, true, false);
            if (lod)
            {
                var regions = new List<P3.LodRegion3D>();
                regions.Add(new P3.LodRegion3D(new(-20, -5, -20), new(6 * 14 - 2, 20, 20), 1));
                regions.Add(new P3.LodRegion3D(new(6 * 14 - 2, -5, -20), new(24 * 14 + 20, 20, 20), 4));
                s.World.SetSimulationLod(default, default, 1, regions.ToArray(), 1);
            }

            for (int i = 0; i < 96; i++) s.World.Step(WorldScaleScenes.Dt);
            var t = new List<double>();
            for (int i = 0; i < 128; i++) { sw.Restart(); s.World.Step(WorldScaleScenes.Dt); t.Add(sw.Elapsed.TotalMilliseconds); }
            return t.Average();
        }

        double full = Step(false), scaled = Step(true);
        Console.WriteLine($"lod-3d-cost threads={threads} bodies=600 fullRateMeanStepMs={F(full)} quarterRateMeanStepMs={F(scaled)} ratio={F(scaled / full)}");
        return 0;
    }

    static class Th
    {
        // 1.5 times the worst of the x64 and arm64 runs (the two agree to the last digit): streaming-3d maxSpeed 6.59, streaming-2d 6.01, lod-3d maxSpeed 0.703 / maxDrift 0.145 / driftFromFullRate 0.181, timescale-2d maxSpeed 8.92 / bubble drift 0.121 / normal drift 0.055.
        internal const double StreamSpeed3 = 9.9, StreamSpeed2 = 9.1, LodSpeed = 1.06, LodDrift = 0.22, LodFullDrift = 0.28, ScaleSpeed = 13.4, BubbleDrift = 0.19, NormalDrift = 0.09;
    }
}
