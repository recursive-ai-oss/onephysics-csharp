using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Articulations;

internal sealed partial class ArticulationStore
{
    // ------------------------------------------------------------------ the stage

    /// <summary>Refuses to step an articulation whose coordinates or speeds are not finite or are beyond the domain the step is written for; nothing has changed yet.</summary>
    public void CheckBounds()
    {
        for (int i = 0; i < Handles.HighWater; i++)
        {
            if (!Handles.IsLive(i)) { continue; }
            var m = States[i]!.Model;
            for (int k = 0; k < m.Q.Length; k++)
            {
                if (!(Math.Abs(m.Q[k]) <= 4194304.0)) { throw new ArticulationStateException(i, k, "positions", "Articulation " + i.ToString(System.Globalization.CultureInfo.InvariantCulture) + ", position value " + k.ToString(System.Globalization.CultureInfo.InvariantCulture) + " must be finite and within ±4194304."); }
            }

            for (int k = 0; k < m.Qd.Length; k++)
            {
                if (!(Math.Abs(m.Qd[k]) <= 1e6)) { throw new ArticulationStateException(i, k, "velocities", "Articulation " + i.ToString(System.Globalization.CultureInfo.InvariantCulture) + ", velocity value " + k.ToString(System.Globalization.CultureInfo.InvariantCulture) + " must be finite and within ±1e6."); }
            }
        }
    }

    /// <summary>
    /// Runs every articulation for one step. Called once per world step after the contact stage has made this step's manifolds. Serial in slot order: gather each articulation's contacts and the bodies it touches;
    /// then simulate the articulations in parallel (each touches only its own state); then publish link velocities and pushed-body velocity changes in slot order. Every articulation reads the engine as it was before any of them
    /// ran, so the result does not depend on the thread count.
    /// </summary>
    public void Step(float dt)
    {
        if (Handles.Count == 0) { return; }
        // One scan of the constraint table, then the contacts are bucketed by articulation (counting sort, so each keeps its constraint order).
        _all.Clear();
        _host.GatherLinkContacts(_all);
        int slots = Handles.HighWater;
        ArticulationWork.Fit(ref _start, slots + 2);
        Array.Clear(_start, 0, slots + 2);
        foreach (var c in _all) { _start[ArticulationOfBody(c.LinkBody) + 1]++; }
        for (int i = 1; i <= slots; i++) { _start[i] += _start[i - 1]; }
        ArticulationWork.Fit(ref _sorted, _all.Count);
        ArticulationWork.Fit(ref _fill, slots + 1);
        Array.Copy(_start, _fill, slots + 1);
        for (int k = 0; k < _all.Count; k++) { _sorted[_fill[ArticulationOfBody(_all[k].LinkBody)]++] = k; }
        ArticulationWork.Fit(ref _live, slots);
        int live = 0;
        for (int i = 0; i < slots; i++)
        {
            if (!Handles.IsLive(i)) { continue; }
            var s = States[i]!;
            s.Work ??= new ArticulationWork(this, _host);
            s.Work.Prepare(s, _all, _sorted, _start[i], _start[i + 1], i);
            _live[live++] = i;
        }

        _stepDt = dt; _stepGravity = _host.Gravity; _stepSlop = _host.ContactSlop; _stepPush = _host.PushOutVelocity;
        _host.Core.Executor.For(live, new SimulateTask(this, live));
        for (int k = 0; k < live; k++) { var s = States[_live[k]]!; s.Work!.Publish(s); }
    }

    private readonly List<HostContact> _all = new();
    private int[] _start = new int[8], _sorted = new int[8], _fill = new int[8], _live = new int[8];
    private float _stepDt;
    private D3 _stepGravity;
    private double _stepSlop, _stepPush;

    private readonly struct SimulateTask(ArticulationStore store, int count) : IParallelBody
    {
        public void Run(int task)
        {
            if ((uint)task >= (uint)count) { return; }
            var s = store.States[store._live[task]]!;
            s.Work!.Simulate(s, store._stepDt, store._stepGravity, store._stepSlop, store._stepPush);
        }
    }

    /// <summary>Puts every link body at the pose its joint coordinates give, at the end of the step.</summary>
    public void EndStep()
    {
        for (int i = 0; i < Handles.HighWater; i++)
        {
            if (!Handles.IsLive(i)) { continue; }
            Place(i);
        }
    }

    // Positions of bodies sit on a 2^-32 grid (see the engine's integration), so a shifted world steps like an unshifted one.
    private const double GridScale = 4294967296.0;

    internal static double OnGrid(double x) => Math.Floor(x * GridScale + 0.5) / GridScale;

    internal static D3 Grid(D3 p) => new(Math.Floor(p.X * GridScale + 0.5) / GridScale, Math.Floor(p.Y * GridScale + 0.5) / GridScale, Math.Floor(p.Z * GridScale + 0.5) / GridScale);

}
