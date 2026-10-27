using System.Diagnostics;
using System.Globalization;
using System.Text;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Physics2D;
using P2 = RecursiveAI.OnePhysics.Physics2D;

namespace Bar;

/// <summary>Runs a scene through this library's 2D world, following the same driver protocol as the reference drivers. Revolute joints are built from the scene; other joint types are skipped (reported on stderr). Queries run through the world's query API, single-threaded, like the reference drivers.</summary>
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
        if (s.Queries.Count > 0) Queries(world, s, sb);
        sb.Append(CultureInfo.InvariantCulture, $"hash={world.ComputeHash():x16}\n");
        File.WriteAllText(result, sb.ToString());
        return 0;
    }

    // The same protocol as the reference drivers: closest ray hits, circle overlaps (shapes counted), circle sweeps (closest hit); the first repetition is untimed when there are several.
    static void Queries(PhysicsWorld2D world, Scene s, StringBuilder sb)
    {
        var ns = new double[3]; var count = new int[3]; var hits = new long[3];
        double nsPerTick = 1e9 / Stopwatch.Frequency;
        var buffer = new QueryHit2D[256];
        string[] names = ["ray", "overlap", "cast"];
        for (int rep = 0; rep < s.QueryReps; rep++)
        {
            bool measure = s.QueryReps == 1 || rep > 0;
            for (int kind = 0; kind < 3; kind++)
            {
                long h = 0; int c = 0; long t0 = Stopwatch.GetTimestamp();
                foreach (var q in s.Queries)
                {
                    if (q.Kind != names[kind]) continue;
                    c++;
                    var P = q.P;
                    if (kind == 0)
                    {
                        if (world.Raycast(new Vector2D(P[0], P[1]), new Vector2F((float)P[2], (float)P[3]), out _, (float)P[4])) h++;
                    }
                    else if (kind == 1)
                    {
                        h += world.OverlapCircleAll(new Vector2D(P[0], P[1]), (float)P[2], buffer);
                    }
                    else
                    {
                        float tx = (float)P[3], ty = (float)P[4];
                        float len = float.Sqrt(tx * tx + ty * ty);
                        if (world.CircleCast(new Vector2D(P[0], P[1]), (float)P[2], new Vector2F(tx, ty), out _, len)) h++;
                    }
                }
                if (measure) { ns[kind] += (Stopwatch.GetTimestamp() - t0) * nsPerTick; count[kind] += c; hits[kind] = h; }
            }
        }
        for (int k = 0; k < 3; k++) sb.Append(CultureInfo.InvariantCulture, $"query_{names[k]}_ns={ns[k]:F0}\nquery_{names[k]}_count={count[k]}\nquery_{names[k]}_hits={hits[k]}\n");
    }
}
