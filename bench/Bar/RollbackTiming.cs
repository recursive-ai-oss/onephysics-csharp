using System.Diagnostics;
using System.Globalization;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Physics3D;

namespace Bar;

// Indicative rollback costs on the rollback scenes: snapshot plus restore (what a host does today), Mark plus RollbackTo, and Fork,
// after 1, 7 and 60 steps since the mark. Times are medians of repeated trials in milliseconds.
internal static class RollbackTiming
{
    private static string Load() => File.ReadAllText("/proc/loadavg").Split(' ')[0];

    internal static int Run(Opts options)
    {
        int threads = int.Parse(options.Get("threads", "1"), CultureInfo.InvariantCulture);
        int trials = int.Parse(options.Get("trials", "15"), CultureInfo.InvariantCulture);
        string only = options.Get("scene", "");
        Console.WriteLine("scene,threads,steps,snapshot_ms,restore_ms,snap_restore_ms,mark_ms,rollback_ms,mark_rollback_ms,fork_ms,snapshot_bytes,load_before,load_after");
        foreach (var name in new[] { "large-pile", "large-pile-3d", "ragdoll-pile-3d", "sleep-wake-3d" })
        {
            if (only.Length > 0 && only != name) { continue; }
            string before = Load();
            foreach (int k in new[] { 1, 7, 60 })
            {
                if (name.EndsWith("-3d", StringComparison.Ordinal)) { Measure3(name, threads, trials, k, before); } else { Measure2(name, threads, trials, k, before); }
            }
        }
        return 0;
    }

    private static double Median(List<double> xs) { xs.Sort(); return xs[xs.Count / 2]; }

    private static void Measure2(string name, int threads, int trials, int k, string before)
    {
        var scene = Gen.All().First(s => s.Name == name);
        // One world per kind of measurement: a restore swaps a world's storage for its spare, which changes capacities, so a mark taken in a world that also restores would be measured against that churn.
        PhysicsWorld2D Make() { var w = SceneBuilder2D.Build(scene, threads).World; w.Warmup(); for (int i = 0; i < 120; i++) { w.Step(1f / 60); } return w; }
        var a = Make(); var b = Make(); var c = Make();
        var buffer = new byte[a.GetSnapshotSizeBound() * 2];
        Rows(name, threads, k, trials, before,
            () => { a.TrySnapshot(buffer, out int n); return n; }, n => a.Restore(buffer.AsSpan(0, n)), () => a.Step(1f / 60),
            () => b.Mark(), m => { b.RollbackTo((RecursiveAI.OnePhysics.WorldMark)m); b.ReleaseMark((RecursiveAI.OnePhysics.WorldMark)m); }, () => b.Step(1f / 60),
            () => { var f = c.Fork(); GC.KeepAlive(f); }, () => c.Step(1f / 60));
    }

    private static void Measure3(string name, int threads, int trials, int k, string before)
    {
        var scene = Gen3.All().First(s => s.Name == name);
        PhysicsWorld3D Make() { var w = SceneBuilder3D.Build(scene, threads).World; w.Warmup(); for (int i = 0; i < 120; i++) { w.Step(1f / 60); } return w; }
        var a = Make(); var b = Make(); var c = Make();
        var buffer = new byte[a.GetSnapshotSizeBound() * 2];
        Rows(name, threads, k, trials, before,
            () => { a.TrySnapshot(buffer, out int n); return n; }, n => a.Restore(buffer.AsSpan(0, n)), () => a.Step(1f / 60),
            () => b.Mark(), m => { b.RollbackTo((RecursiveAI.OnePhysics.WorldMark)m); b.ReleaseMark((RecursiveAI.OnePhysics.WorldMark)m); }, () => b.Step(1f / 60),
            () => { var f = c.Fork(); GC.KeepAlive(f); }, () => c.Step(1f / 60));
    }

    private static void Rows(string name, int threads, int k, int trials, string before,
        Func<int> snapshot, Action<int> restore, Action stepA, Func<object> mark, Action<object> rollback, Action stepB, Action fork, Action stepC)
    {
        var snap = new List<double>(); var rest = new List<double>(); var both = new List<double>();
        var mk = new List<double>(); var rb = new List<double>(); var mr = new List<double>(); var fk = new List<double>();
        int bytes = 0;
        double Time(Action a) { long t0 = Stopwatch.GetTimestamp(); a(); return 1000.0 * (Stopwatch.GetTimestamp() - t0) / Stopwatch.Frequency; }
        for (int t = 0; t < trials + 3; t++)
        {
            int n = 0;
            double s = Time(() => n = snapshot());
            bytes = n;
            for (int i = 0; i < k; i++) { stepA(); }
            double r = Time(() => restore(n));
            if (t >= 3) { snap.Add(s); rest.Add(r); both.Add(s + r); }

            object m = null!;
            double a = Time(() => m = mark());
            for (int i = 0; i < k; i++) { stepB(); }
            double b = Time(() => rollback(m));
            if (t >= 3) { mk.Add(a); rb.Add(b); mr.Add(a + b); }

            for (int i = 0; i < k; i++) { stepC(); }
            double f = Time(fork);
            if (t >= 3) { fk.Add(f); }
        }

        string F(List<double> xs) => xs.Count == 0 ? "" : Median(xs).ToString("F3", CultureInfo.InvariantCulture);
        Console.WriteLine($"{name},{threads},{k},{F(snap)},{F(rest)},{F(both)},{F(mk)},{F(rb)},{F(mr)},{F(fk)},{bytes},{before},{Load()}");
    }
}
