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

    /// <summary>
    /// Indicative cost of an edit against the grid size: a radius-8-cell carve of a 2D bitmask terrain and a radius-4-cell sphere dig of a 3D voxel grid, alternating carve and fill at one place so every call changes cells,
    /// against building the same grid from scratch. Rows: median and p90 over 200 edits, in microseconds.
    /// </summary>
    internal static int Time(Opts opts)
    {
        const int Edits = 200;
        double Median(List<double> v) { v.Sort(); return v[v.Count / 2]; }
        double P90(List<double> v) => v[(int)(0.9 * (v.Count - 1))];
        foreach (var (w, h) in new[] { (128, 64), (256, 64), (512, 128), (1024, 64) })
        {
            var mask = new bool[w * h];
            for (int x = 0; x < w; x++) { int top = h / 2 + (x * 7 % 5); for (int y = 0; y < top; y++) mask[y * w + x] = true; }
            var world = new RecursiveAI.OnePhysics.Physics2D.PhysicsWorld2D(new() { WorldId = 150 });
            var terrain = world.AddBitmaskTerrain(w, h, mask, 0.25f);
            var center = new RecursiveAI.OneMath.Vector2D(w * 0.125, h / 2 * 0.25);
            var edit = new List<double>(); var ticks = new List<double>();
            for (int i = 0; i < Edits; i++)
            {
                long t0 = Stopwatch.GetTimestamp();
                int n = (i & 1) == 0 ? world.CarveCircle(terrain, center, 2.0) : world.FillCircle(terrain, center, 2.0);
                edit.Add(Stopwatch.GetElapsedTime(t0).TotalMilliseconds * 1000);
            }

            var tiles = world.GetTileGrid(terrain).Tiles.ToArray();
            for (int i = 0; i < 20; i++) { long t0 = Stopwatch.GetTimestamp(); _ = new RecursiveAI.OnePhysics.TileGrid2D(w, h, tiles, 0.25f, default, true); ticks.Add(Stopwatch.GetElapsedTime(t0).TotalMilliseconds * 1000); }
            Console.WriteLine($"carve2d grid={w}x{h} cells={w * h} segments={world.GetTileGrid(terrain).SegmentCount} radiusCells=8 editUs median={F(Median(edit))} p90={F(P90(edit))} fullBuildUs median={F(Median(ticks))}");
        }

        foreach (var (x, y, z) in new[] { (48, 48, 48), (64, 32, 64), (96, 32, 96), (100, 100, 100) })
        {
            var cells = new bool[x * y * z];
            for (int k = 0; k < z; k++) for (int j = 0; j < y / 2; j++) for (int i = 0; i < x; i++) cells[(k * y + j) * x + i] = true;
            var world = new RecursiveAI.OnePhysics.Physics3D.PhysicsWorld3D(new() { WorldId = 151 });
            var shape = world.AddVoxelGrid(x, y, z, cells, 0.5f).Voxels;
            var center = new RecursiveAI.OneMath.Vector3D(x * 0.25, y / 2 * 0.5, z * 0.25);
            var edit = new List<double>(); var full = new List<double>();
            for (int i = 0; i < Edits; i++)
            {
                long t0 = Stopwatch.GetTimestamp();
                int n = (i & 1) == 0 ? shape.DigSphere(center, 2.0) : shape.FillSphere(center, 2.0);
                edit.Add(Stopwatch.GetElapsedTime(t0).TotalMilliseconds * 1000);
            }

            var occupancy = world.GetVoxelOccupancy(shape);
            for (int i = 0; i < 5; i++) { long t0 = Stopwatch.GetTimestamp(); _ = new RecursiveAI.OnePhysics.LowLevel.VoxelGrid3D(x, y, z, occupancy, 0.5f); full.Add(Stopwatch.GetElapsedTime(t0).TotalMilliseconds * 1000); }
            Console.WriteLine($"voxelstamp3d grid={x}x{y}x{z} cells={x * y * z} radiusCells=4 editUs median={F(Median(edit))} p90={F(P90(edit))} fullBuildUs median={F(Median(full))}");
        }

        return 0;
    }
}
