using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.HashMatrix;

/// <summary>
/// <c>--rollback-probe</c>: in the middle of every scenario, for every world it steps, marks the world, changes its inputs and steps on (1, 7 and 60 steps, then a ring of three marks),
/// rolls back and demands the snapshot bytes and the hash of the marked state. The scenario then goes on from the rolled-back world, so a probed run's manifest must equal the plain run's:
/// that checks the future after a rollback against a world that never rolled back, over every scenario family in the matrix.
/// </summary>
internal static class RollbackProbe
{
    private sealed class State { public int Stage; public bool Busy; }
    private static readonly ConditionalWeakTable<object, State> States = new();
    internal static int Probes;
    private static readonly bool LiveMode = Environment.GetEnvironmentVariable("HM_PROBE_LIVE") == "1";
    private const int FutureSteps = 12;
    private static readonly string PerturbMode = Environment.GetEnvironmentVariable("HM_PROBE_PERTURB") ?? "all";
    private static readonly bool ViaRestore = Environment.GetEnvironmentVariable("HM_PROBE_VIA_RESTORE") == "1";
    private static byte[]? SavedBytes;
    private static readonly int StageMask = int.TryParse(Environment.GetEnvironmentVariable("HM_PROBE_STAGES"), out int mask) ? mask : 15;

    internal static void Install() => StepObserver.Observer = OnStep;

    private sealed class Ops
    {
        public Func<byte[]> Snapshot = null!;
        public Func<ulong> Hash = null!;
        public Func<WorldMark> Mark = null!;
        public Action<WorldMark> Rollback = null!;
        public Action<int, float> StepSuppressed = null!;
        public Action Perturb = null!;
        public Func<WorldMark, bool> Valid = null!;
        public Action<WorldMark> Release = null!;
        public Func<long> StepCount = null!;
        public Func<int> Live = null!;
        public Action Trim = null!;
        public Func<(Action<int, float> Step, Func<object> World)> Fork = null!;
        public Func<object, object, string> Diff = null!;
    }

    [ThreadStatic] private static bool _inProbe;

    private static Ops? Make(object world)
    {
        Ops ops;
        if (world is PhysicsWorld3D w3)
        {
            if (w3.IsRecording) { return null; }
            ops = new Ops
            {
                Snapshot = w3.Snapshot, Hash = w3.ComputeHash, Mark = w3.Mark, Rollback = m => { if (ViaRestore) { w3.Restore(SavedBytes!); } else { w3.RollbackTo(m); } }, Valid = w3.IsMarkValid, Release = m => w3.ReleaseMark(m),
                StepCount = () => w3.StepCount, Live = () => w3.LiveMarkCount, Trim = w3.ReleaseAllMarks,
                Fork = () => { var f = w3.Fork(); return ((n, d) => { using var s = f.SuppressCallbacks(); for (int i = 0; i < n; i++) { f.Step(d); } }, () => f); },
                Diff = (a, b) => { var r = ((PhysicsWorld3D)a).FindDifferences((PhysicsWorld3D)b); return r.Count == 0 ? "" : r.ToString(); },
                StepSuppressed = (n, d) => { using var s = w3.SuppressCallbacks(); for (int i = 0; i < n; i++) { w3.Step(d); } },
                Perturb = () =>
                {
                    var g = w3.Options.Gravity; w3.Options.Gravity = new Vector3F(g.X + 0.5f, g.Y, g.Z);
                    var extra = w3.AddSphere(0.25f, at: new Vector3D(1.5, 30, 1.5));
                    extra.LinearVelocity = new Vector3F(3, 1, -2);
                },
            };
        }
        else if (world is PhysicsWorld2D w2)
        {
            if (w2.IsRecording) { return null; }
            ops = new Ops
            {
                Snapshot = w2.Snapshot, Hash = w2.ComputeHash, Mark = w2.Mark, Rollback = m => { if (ViaRestore) { w2.Restore(SavedBytes!); } else { w2.RollbackTo(m); } }, Valid = w2.IsMarkValid, Release = m => w2.ReleaseMark(m),
                StepCount = () => w2.StepCount, Live = () => w2.LiveMarkCount, Trim = w2.ReleaseAllMarks,
                Fork = () => { var f = w2.Fork(); return ((n, d) => { using var s = f.SuppressCallbacks(); for (int i = 0; i < n; i++) { f.Step(d); } }, () => f); },
                Diff = (a, b) => { var r = ((PhysicsWorld2D)a).FindDifferences((PhysicsWorld2D)b); return r.Count == 0 ? "" : r.ToString(); },
                StepSuppressed = (n, d) => { using var s = w2.SuppressCallbacks(); for (int i = 0; i < n; i++) { w2.Step(d); } },
                Perturb = () =>
                {
                    var g = w2.Options.Gravity; w2.Options.Gravity = new Vector2F(g.X + 0.5f, g.Y);
                    var extra = w2.AddCircle(0.25f, at: new Vector2D(1.5, 30));
                    extra.LinearVelocity = new Vector2F(3, 1);
                },
            };
        }
        else { return null; }
        return ops;
    }


    private static void OnStep(object world, float dt)
    {
        if (_inProbe) { return; }
        var live = Make(world);
        if (live == null) { return; }
        var state = States.GetOrCreateValue(world);
        if (state.Busy || state.Stage > 3) { return; }
        long c = live.StepCount();
        int stage = state.Stage;
        if ((stage == 0 && c < 3) || (stage == 1 && c < 17) || (stage == 2 && c < 61) || (stage == 3 && c < 101)) { return; }
        state.Busy = true; _inProbe = true;
        try
        {
            state.Stage++;
            if (((StageMask >> stage) & 1) == 0) { return; }
            // By default the probe works on a structural fork, so the scenario's own world (and what it reads from it after the step) is untouched and its manifest must not move.
            object target = LiveMode ? world : (world is PhysicsWorld3D f3 ? f3.Fork() : ((PhysicsWorld2D)world).Fork());
            var ops = LiveMode ? live : Make(target)!;
            if (stage < 3) { Experiment(ops, stage == 0 ? 1 : stage == 1 ? 7 : 60, dt, target); }
            else { Ring(ops, dt, target); }
            Probes++;
        }
        finally { state.Busy = false; _inProbe = false; }
    }

    private static void Check(Ops ops, byte[] bytes, ulong hash, string what, object world)
    {
        ulong h = ops.Hash();
        if (h != hash) { throw new InvalidOperationException($"rollback probe ({world.GetType().Name}): {what}: hash {h:x16} differs from the marked state's {hash:x16}"); }
        var now = ops.Snapshot();
        if (!now.AsSpan().SequenceEqual(bytes))
        {
            int at = 0; while (at < now.Length && at < bytes.Length && now[at] == bytes[at]) { at++; }
            throw new InvalidOperationException($"rollback probe ({world.GetType().Name}): {what}: snapshot differs from the marked state's, first at byte {at} (lengths {now.Length} and {bytes.Length})");
        }
    }

    private static void Experiment(Ops ops, int k, float dt, object world)
    {
        var bytes = ops.Snapshot(); SavedBytes = bytes;
        ulong hash = ops.Hash();
        var never = ops.Fork();
        var m = ops.Mark();
        if (PerturbMode != "none") { ops.Perturb(); }
        ops.StepSuppressed(k, dt);
        ops.Rollback(m);
        Check(ops, bytes, hash, $"after {k} steps", world);
        // Rolling back again to the same mark works, and the step after a rollback does not depend on what the abandoned future did.
        ops.StepSuppressed(1, dt);
        ops.Rollback(m);
        Check(ops, bytes, hash, "a second rollback to the same mark", world);
        ops.Release(m);
        if (ops.Valid(m)) { throw new InvalidOperationException("a released mark is still valid"); }
        // The future: a fork of the rolled-back world and the fork taken before the experiment, which never rolled back, stepped alike, stay equal.
        var after = ops.Fork();
        after.Step(FutureSteps, dt); never.Step(FutureSteps, dt);
        string report = ops.Diff(after.World(), never.World());
        if (report.Length > 0) { throw new InvalidOperationException($"rollback probe ({world.GetType().Name}): the future after {k} steps and a rollback differs from the world that never rolled back:\n" + report); }
    }

    private static void Ring(Ops ops, float dt, object world)
    {
        var b0 = ops.Snapshot(); ulong h0 = ops.Hash(); var a = ops.Mark();
        ops.StepSuppressed(2, dt);
        var b1 = ops.Snapshot(); ulong h1 = ops.Hash(); var b = ops.Mark();
        ops.Perturb();
        ops.StepSuppressed(3, dt);
        var c = ops.Mark();
        ops.StepSuppressed(2, dt);
        ops.Rollback(b);
        Check(ops, b1, h1, "ring rollback to the middle mark", world);
        if (ops.Valid(c)) { throw new InvalidOperationException("a mark taken after the rollback target is still valid"); }
        if (!ops.Valid(a) || !ops.Valid(b)) { throw new InvalidOperationException("a mark at or before the rollback target was released"); }
        ops.StepSuppressed(1, dt);
        ops.Rollback(a);
        Check(ops, b0, h0, "ring rollback to the oldest mark", world);
        if (ops.Valid(b)) { throw new InvalidOperationException("the middle mark survived a rollback to an older one"); }
        ops.Trim();
        if (ops.Live() != 0) { throw new InvalidOperationException("marks remain after ReleaseAllMarks"); }
    }
}
