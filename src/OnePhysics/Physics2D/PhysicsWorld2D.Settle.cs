using System;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    /// <summary>
    /// Runs the world to equilibrium and reports how close it got. Sleeping is switched off for the run, so a body that is still moving cannot hide by sleeping, and switched back on after it (bodies fall asleep in later steps as usual).
    /// Each step is an ordinary <see cref="Step"/> at <see cref="SettleOptions2D.Dt"/>, followed by the optional extra damping; the run stops after <see cref="SettleOptions2D.StableSteps"/> consecutive steps within the tolerances or at <see cref="SettleOptions2D.MaxSteps"/>.
    /// Callbacks and events of the intermediate steps are suppressed; <see cref="Events"/> holds the last step's. <see cref="StepCount"/> advances by <see cref="SettleReport.StepsUsed"/>.
    /// A recorder sees one Settle record, not the steps: playback runs the same settle and checks the report and the world hash. Deterministic at every thread count and vector width.
    /// Allocates scratch proportional to the body count for the call (and the per-body arrays in the report only when <see cref="SettleOptions2D.ReportBodies"/> is set); it is an offline operation, not a per-frame one.
    /// </summary>
    /// <param name="options">Options; defaults when null. Copied.</param>
    /// <returns>The report. A world with no moving parts converges after <see cref="SettleOptions2D.StableSteps"/> steps.</returns>
    /// <exception cref="ArgumentOutOfRangeException">An option is out of range.</exception>
    /// <exception cref="InvalidOperationException">Called from inside a step, a callback or another settle.</exception>
    /// <exception cref="BodyOutOfBoundsException">A step rejected a body outside the supported range; the steps already taken stay.</exception>
    public SettleReport Settle(SettleOptions2D? options = null)
    {
        var o = options ?? new SettleOptions2D();
        o.Validate();
        if (_settling || _stepping) throw new InvalidOperationException("Settle cannot run inside a step, a callback or another settle.");
        _core.FlushPostStep();
        var encoded = new byte[SettleOptions2D.EncodedSize];
        o.Write(encoded);
        var report = RunSettle(o);
        _recorder?.RecordSettle(encoded, report);
        return report;
    }

    private bool _settling, _stepping;

    private SettleReport RunSettle(SettleOptions2D o)
    {
        var p = o.ToParams();
        bool sleep = _engine.Opt.SleepEnabled;
        _settling = true;
        using var pause = _recorder?.Pause() ?? default;
        using var suppression = SuppressCallbacks();
        try
        {
            _engine.Opt.SleepEnabled = false;
            return SettleRunner.Run(new SettleHost(this), in p);
        }
        finally
        {
            _engine.Opt.SleepEnabled = sleep;
            // Step copied the live options into the core; copy them back so the saved options are the caller's, not the ones the run used.
            SyncCoreOptions();
            _settling = false;
        }
    }

    // Replays a recorded settle: the same options, then the report must be the recorded one.
    internal SettleReport ReplaySettle(ReadOnlySpan<byte> options) => RunSettle(SettleOptions2D.Read(options));

    private readonly struct SettleHost(PhysicsWorld2D world) : ISettleHost
    {
        public WorldCore Core => world._core;
        public float MassOf(int slot) => world._engine.Mass[slot];
        public void Step(float dt) => world.Step(dt);

        public float DampAndScanDeformables(float damping, float dt)
        {
            float factor = damping > 0 ? 1f / (1f + damping * dt) : 1f;
            double worst = 0;
            var e = world._engine;
            for (int i = 0; i < e.SoftHandles.HighWater; i++)
            {
                if (!e.SoftHandles.IsLive(i)) { continue; }
                var d = e.SoftData[i]!;
                if (d.Asleep) { continue; }
                var v = d.Vel;
                for (int k = 0; k < d.N; k++)
                {
                    if (factor != 1f) { v[2 * k] *= factor; v[2 * k + 1] *= factor; }
                    double s = (double)v[2 * k] * v[2 * k] + (double)v[2 * k + 1] * v[2 * k + 1];
                    if (s > worst) { worst = s; }
                }
            }
            var rods = e.Rods.Data;
            for (int i = 0; i < rods.Length; i++)
            {
                var r = rods[i];
                if (r == null) { continue; }
                var v = r.VelocityData;
                for (int k = 0; k < v.Length; k++)
                {
                    if (factor != 1f) { v[k] = new RecursiveAI.OneMath.Vector3F(v[k].X * factor, v[k].Y * factor, v[k].Z * factor); }
                    double s = (double)v[k].X * v[k].X + (double)v[k].Y * v[k].Y + (double)v[k].Z * v[k].Z;
                    if (s > worst) { worst = s; }
                }
            }
            return (float)Math.Sqrt(worst);
        }
    }
}
