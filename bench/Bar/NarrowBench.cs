using System.Diagnostics;
using System.Globalization;
using RecursiveAI.OnePhysics.LowLevel;

namespace Bar;

using Scene = RecursiveAI.OnePhysics.Scene.Scene;

/// <summary>
/// Times the contact stage (pair revisit, narrowphase and apply) alone: a scene is stepped to a chosen step, then the stage runs again and again on the unchanged bodies, which is deterministic work
/// (every pair gives the same answer each time), so the minimum over many repetitions is steady even on a loaded host. Used to compare two builds of the engine on the same scene.
/// </summary>
static class NarrowBench
{
    public static int Run(Opts o)
    {
        var s = Scene.Load(o.Get("scene", ""));
        int at = int.Parse(o.Get("at", "200"), CultureInfo.InvariantCulture), reps = int.Parse(o.Get("reps", "300"), CultureInfo.InvariantCulture);
        var sim = new OnePhysics3Sim();
        sim.Build(s, 1);
        for (int step = 0; step < at; step++)
        {
            if (s.Events.Any(e => e.Step == step)) sim.WakeAll(s);
            sim.Step(s.Dt);
        }

        if (o.Has("steps")) { return Steps(sim, s, at, int.Parse(o.Get("steps", "10"), CultureInfo.InvariantCulture), reps); }
        var e3 = sim.World.Engine; var core = e3.Core;
        // Fresh mode: no manifold is reused and every pair that was found apart is searched again (a changed speculative distance empties that memory), so each touching pair runs the whole narrowphase.
        bool fresh = o.Has("fresh");
        if (fresh) { e3.RecycleManifolds = false; }
        var ns = new double[reps]; double tick = 1e9 / Stopwatch.Frequency;
        for (int r = 0; r < reps + 20; r++)
        {
            if (fresh) { e3.Opt.SpeculativeDistance = (r & 1) == 0 ? 0.02f : 0.020001f; }
            long t0 = Stopwatch.GetTimestamp();
            e3.UpdateContacts(core);
            double t = (Stopwatch.GetTimestamp() - t0) * tick;
            if (r >= 20) ns[r - 20] = t;
        }

        Array.Sort(ns);
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"{s.Name} at={at} awake={core.AwakeBodies.Length} pairs={e3.PairHigh} updated={e3.StatUpdated} reached={e3.StatReached} fresh={e3.StatFresh} {(fresh ? "FRESH " : "")}contactstage min={ns[0] / 1e3:F1}us p10={ns[reps / 10] / 1e3:F1}us median={ns[reps / 2] / 1e3:F1}us"));
        return 0;
    }

    // Whole steps from one saved state: every repetition restores the same snapshot and takes the same few steps, so each step does identical work and the minimum over repetitions is its cost without the noise
    // of a shared host. The first steps after a restore rebuild caches the snapshot does not hold and are not measured.
    static int Steps(OnePhysics3Sim sim, Scene s, int at, int k, int reps)
    {
        const int Warm = 3;
        var snap = sim.World.Snapshot();
        var best = new double[k]; Array.Fill(best, double.MaxValue); var sums = new double[reps];
        double tick = 1e9 / Stopwatch.Frequency;
        for (int r = 0; r < reps + 3; r++)
        {
            sim.World.Restore(snap);
            double sum = 0;
            for (int j = 0; j < Warm + k; j++)
            {
                long t0 = Stopwatch.GetTimestamp();
                sim.Step(s.Dt);
                double t = (Stopwatch.GetTimestamp() - t0) * tick;
                if (j >= Warm) { if (r >= 3 && t < best[j - Warm]) best[j - Warm] = t; sum += t; }
            }

            if (r >= 3) sums[r - 3] = sum;
        }

        Array.Sort(sums);
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"{s.Name} at={at} STEPS k={k} sum-of-min-steps={best.Sum() / 1e3:F1}us per-step-min=[{string.Join(' ', best.Select(b => (b / 1e3).ToString("F0", CultureInfo.InvariantCulture)))}] p25-rep-sum={sums[reps / 4] / 1e3:F1}us"));
        return 0;
    }
}
