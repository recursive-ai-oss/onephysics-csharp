using System.Diagnostics;
using System.Globalization;
using System.Text;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Physics2D;
using P2 = RecursiveAI.OnePhysics.Physics2D;

namespace Bar;

/// <summary>Runs a scene through this library's 2D world, following the same driver protocol as the reference drivers. Queries are not supported yet.</summary>
static class OnePhysicsDriver
{
    public static int Run(Opts o)
    {
        var s = Scene.Load(o.Get("scene", ""));
        bool timing = o.Get("mode", "stability") == "timing";
        string result = o.Get("result", ""), dumpPath = o.Get("dump", "");
        int threads = int.Parse(o.Get("threads", "1"), CultureInfo.InvariantCulture);

        var (world, bodies, opt) = SceneBuilder2D.Build(s, threads);
        int n = bodies.Length;
        var d = new double[n * 7];
        void Snap()
        {
            for (int i = 0; i < n; i++)
            {
                var p = bodies[i].Position; var v = bodies[i].LinearVelocity;
                d[i * 7] = p.X; d[i * 7 + 1] = p.Y; d[i * 7 + 2] = bodies[i].Angle; d[i * 7 + 3] = v.X; d[i * 7 + 4] = v.Y;
                d[i * 7 + 5] = bodies[i].AngularVelocity; d[i * 7 + 6] = bodies[i].IsAwake ? 1 : 0;
            }
        }
        BinaryWriter? dump = null;
        if (!timing && dumpPath != "")
        {
            dump = new BinaryWriter(File.Create(dumpPath));
            dump.Write(Encoding.ASCII.GetBytes("OPDUMP01")); dump.Write((uint)n); dump.Write((uint)0);
        }
        void Frame(int step) { Snap(); dump!.Write((uint)step); dump.Write((uint)0); foreach (var v in d) dump.Write(v); }
        if (dump != null) Frame(0);

        var stepNs = new double[s.Steps];
        double nsPerTick = 1e9 / Stopwatch.Frequency;
        float dt = (float)s.Dt;
        for (int step = 0; step < s.Steps; step++)
        {
            foreach (var e in s.Events) if (e.Step == step) for (int i = 0; i < n; i++) if (s.Bodies[i].Type == "dynamic") bodies[i].Wake();
            long t0 = Stopwatch.GetTimestamp();
            world.Step(dt);
            stepNs[step] = (Stopwatch.GetTimestamp() - t0) * nsPerTick;
            if (dump != null && ((step + 1) % s.DumpEvery == 0 || step + 1 == s.Steps)) Frame(step + 1);
        }
        dump?.Dispose();

        var sb = new StringBuilder();
        sb.Append(CultureInfo.InvariantCulture, $"engine=onephysics\nversion=0.1.0-dev\nconfig=threads={threads}, substeps={opt.Substeps}, default options\nscene={s.Name}\nthreads={threads}\nsteps={s.Steps}\nwarmup={s.Warmup}\nbodies={n}\n");
        sb.Append("step_ns=").Append(string.Join(',', stepNs.Select(v => v.ToString("F0", CultureInfo.InvariantCulture)))).Append('\n');
        sb.Append(CultureInfo.InvariantCulture, $"hash={world.ComputeHash():x16}\n");
        File.WriteAllText(result, sb.ToString());
        return 0;
    }
}
