using System.Diagnostics;
using System.Globalization;
using System.Numerics;
using System.Text;

namespace Bar;

/// <summary>What an in-process 3D engine adapter provides to the shared driver loop.</summary>
interface ISim3
{
    string Name { get; }
    string Version { get; }
    string Config { get; }
    void Build(Scene s, int threads);
    void Step(double dt);
    void WakeAll(Scene s);
    /// <summary>Fills 14 doubles per body: x y z qx qy qz qw vx vy vz wx wy wz awake.</summary>
    void Snapshot(double[] d);
    /// <summary>Runs the scene's queries (closest ray hits, sphere overlaps counting shapes, sphere casts to the closest hit) and appends query_* lines; engines without queries append nothing.</summary>
    void Queries(Scene s, StringBuilder sb) { }
}

static class Driver3
{
    public static int Run(Opts o, ISim3 sim)
    {
        var s = Scene.Load(o.Get("scene", ""));
        bool timing = o.Get("mode", "stability") == "timing";
        int threads = int.Parse(o.Get("threads", "1"), CultureInfo.InvariantCulture);
        sim.Build(s, threads);
        int n = s.Bodies.Count; var d = new double[n * 14];
        BinaryWriter? dump = null;
        if (!timing && o.Has("dump"))
        {
            dump = new BinaryWriter(File.Create(o.Get("dump", "")));
            dump.Write(Encoding.ASCII.GetBytes("OPDUMP01")); dump.Write((uint)n); dump.Write((uint)3);
        }
        void Frame(int step) { sim.Snapshot(d); dump!.Write((uint)step); dump.Write((uint)0); foreach (var v in d) dump.Write(v); }
        if (dump != null) Frame(0);
        var stepNs = new double[s.Steps]; var allocB = new long[s.Steps]; double nsPerTick = 1e9 / Stopwatch.Frequency;
        for (int step = 0; step < s.Steps; step++)
        {
            if (s.Events.Any(e => e.Step == step)) sim.WakeAll(s);
            long t0 = Stopwatch.GetTimestamp();
            long a0 = GC.GetAllocatedBytesForCurrentThread();
            sim.Step(s.Dt);
            allocB[step] = GC.GetAllocatedBytesForCurrentThread() - a0;
            stepNs[step] = (Stopwatch.GetTimestamp() - t0) * nsPerTick;
            if (dump != null && ((step + 1) % s.DumpEvery == 0 || step + 1 == s.Steps)) Frame(step + 1);
        }
        dump?.Dispose();
        var sb = new StringBuilder();
        if (timing && s.Queries.Count > 0) sim.Queries(s, sb);
        sb.Append(CultureInfo.InvariantCulture, $"engine={sim.Name}\nversion={sim.Version}\nconfig={sim.Config}\nscene={s.Name}\nthreads={threads}\nsteps={s.Steps}\nwarmup={s.Warmup}\nbodies={n}\n");
        sb.Append("step_ns=").Append(string.Join(',', stepNs.Select(v => v.ToString("F0", CultureInfo.InvariantCulture)))).Append('\n');
        sb.Append("alloc_bytes=").Append(string.Join(',', allocB)).Append('\n');
        File.WriteAllText(o.Get("result", ""), sb.ToString());
        return 0;
    }

    /// <summary>Volume of the convex hull of a point set (used to turn a density into a mass).</summary>
    public static double HullVolume(Vector3[] p)
    {
        var c = Vector3.Zero; foreach (var q in p) c += q; c /= p.Length;
        var planes = new Dictionary<(long, long, long, long), (Vector3 n, float d, List<Vector3> pts)>();
        for (int i = 0; i < p.Length; i++)
            for (int j = i + 1; j < p.Length; j++)
                for (int k = j + 1; k < p.Length; k++)
                {
                    var n = Vector3.Cross(p[j] - p[i], p[k] - p[i]); float l = n.Length(); if (l < 1e-9f) continue; n /= l;
                    float dd = Vector3.Dot(n, p[i]); bool pos = false, neg = false;
                    foreach (var q in p) { float s = Vector3.Dot(n, q) - dd; if (s > 1e-5f) pos = true; else if (s < -1e-5f) neg = true; }
                    if (pos && neg) continue;
                    if (Vector3.Dot(n, c) - dd > 0) { n = -n; dd = -dd; } // outward
                    var key = ((long)MathF.Round(n.X * 1e4f), (long)MathF.Round(n.Y * 1e4f), (long)MathF.Round(n.Z * 1e4f), (long)MathF.Round(dd * 1e4f));
                    if (!planes.ContainsKey(key)) planes[key] = (n, dd, p.Where(q => MathF.Abs(Vector3.Dot(n, q) - dd) < 1e-5f).ToList());
                }
        double vol = 0;
        foreach (var (n, dd, pts) in planes.Values)
        {
            var fc = Vector3.Zero; foreach (var q in pts) fc += q; fc /= pts.Count;
            var u = Vector3.Normalize(pts[0] - fc); var w = Vector3.Cross(n, u);
            var sorted = pts.OrderBy(q => Math.Atan2(Vector3.Dot(q - fc, w), Vector3.Dot(q - fc, u))).ToList();
            double area = 0;
            for (int i = 0; i < sorted.Count; i++) area += 0.5 * Vector3.Cross(sorted[i] - fc, sorted[(i + 1) % sorted.Count] - fc).Length();
            vol += area * (-(Vector3.Dot(n, c) - dd)) / 3;
        }
        return vol;
    }

    public static double ShapeVolume(ShapeDef sh)
    {
        var p = sh.P; double r = sh.Radius;
        return sh.Kind switch
        {
            "sphere" => 4.0 / 3 * Math.PI * r * r * r,
            "box" => 8 * p[0] * p[1] * p[2],
            "capsule" => Math.PI * r * r * Math.Sqrt((p[3] - p[0]) * (p[3] - p[0]) + (p[4] - p[1]) * (p[4] - p[1]) + (p[5] - p[2]) * (p[5] - p[2])) + 4.0 / 3 * Math.PI * r * r * r,
            "hull" => HullVolume(Enumerable.Range(0, p.Length / 3).Select(i => new Vector3((float)p[3 * i], (float)p[3 * i + 1], (float)p[3 * i + 2])).ToArray()),
            _ => 0,
        };
    }
}
