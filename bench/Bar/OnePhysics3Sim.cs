using System.Diagnostics;
using System.Globalization;
using System.Text;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace Bar;

/// <summary>This library's 3D world behind the shared 3D driver loop.</summary>
sealed class OnePhysics3Sim : ISim3
{
    P3.PhysicsWorld3D world = null!;
    P3.Body[] bodies = [];
    int threads;

    public string Name => "onephysics";
    public string Version => "0.1.0-dev";
    public string Config => $"threads={threads}, substeps=4, default options";

    public void Build(Scene s, int threads)
    {
        this.threads = threads;
        (world, bodies, _) = SceneBuilder3D.Build(s, threads);
    }

    public void Step(double dt) => world.Step((float)dt);

    public void WakeAll(Scene s)
    {
        for (int i = 0; i < bodies.Length; i++) if (s.Bodies[i].Type == "dynamic") bodies[i].Wake();
    }

    public void Snapshot(double[] d)
    {
        for (int i = 0; i < bodies.Length; i++)
        {
            int o = i * 14; var b = bodies[i];
            var p = b.Position; var q = b.Rotation; var v = b.LinearVelocity; var w = b.AngularVelocity;
            d[o] = p.X; d[o + 1] = p.Y; d[o + 2] = p.Z; d[o + 3] = q.X; d[o + 4] = q.Y; d[o + 5] = q.Z; d[o + 6] = q.W;
            d[o + 7] = v.X; d[o + 8] = v.Y; d[o + 9] = v.Z; d[o + 10] = w.X; d[o + 11] = w.Y; d[o + 12] = w.Z;
            d[o + 13] = b.IsAwake ? 1 : 0;
        }
    }

    // The same protocol as the reference drivers; the first repetition is untimed when there are several.
    public void Queries(Scene s, StringBuilder sb)
    {
        var ns = new double[3]; var count = new int[3]; var hits = new long[3];
        double nsPerTick = 1e9 / Stopwatch.Frequency;
        var buffer = new P3.QueryHit3D[256];
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
                        if (world.Raycast(new Vector3D(P[0], P[1], P[2]), new Vector3F((float)P[3], (float)P[4], (float)P[5]), out _, (float)P[6])) h++;
                    }
                    else if (kind == 1)
                    {
                        h += world.OverlapSphereAll(new Vector3D(P[0], P[1], P[2]), (float)P[3], buffer);
                    }
                    else
                    {
                        float tx = (float)P[4], ty = (float)P[5], tz = (float)P[6];
                        float len = float.Sqrt(tx * tx + ty * ty + tz * tz);
                        if (world.SphereCast(new Vector3D(P[0], P[1], P[2]), (float)P[3], new Vector3F(tx, ty, tz), out _, len)) h++;
                    }
                }
                if (measure) { ns[kind] += (Stopwatch.GetTimestamp() - t0) * nsPerTick; count[kind] += c; hits[kind] = h; }
            }
        }
        for (int k = 0; k < 3; k++) sb.Append(CultureInfo.InvariantCulture, $"query_{names[k]}_ns={ns[k]:F0}\nquery_{names[k]}_count={count[k]}\nquery_{names[k]}_hits={hits[k]}\n");
    }
}
