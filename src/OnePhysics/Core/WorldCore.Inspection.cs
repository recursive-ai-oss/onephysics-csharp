using System;
using System.Runtime.CompilerServices;

namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class WorldCore
{
    private bool _collectTransitions;

    /// <summary>Unsaved body input and damping impulses of the last step, in world axes.</summary>
    public BodyReadback BodyReadback { get; }

    /// <summary>Unsaved whole-step joint impulses and last-pass row data, shared by both engines.</summary>
    public JointReadback JointReadback { get; }

    /// <summary>Always-on diagnostics of the last step. Reused; not saved or hashed.</summary>
    /// <remarks>The population counters (awake and asleep islands, continuous-collision and bullet bodies) are counted on their first read after a step, or before the first change to the world after it.</remarks>
    public StepCounters Counters { get; }

    // Post-step work left for the first reader. _countersPending: CapturePopulationCounters is owed (set at the end of Step; the counters' getters ask through CapturePendingCounters). _viewPending: the
    // engine's world owes a view built from the state after the step (PhysicsWorld3D's public events), built by PostStepFlush. Both are dropped at the start of the next step. Everything that changes
    // what they read (bodies, shapes, constraints, islands, sleep) calls FlushPostStep first, so a late result equals the one computed at the end of the step: the core's own mutators do, and the
    // worlds' public mutators do through RecordMutation. Raw writes to the stores' spans are not seen.
    private bool _countersPending, _viewPending;

    /// <summary>Builds the engine world's post-step view; called at most once per step, before the first change to the world after it.</summary>
    internal Action? PostStepFlush;

    /// <summary>The engine world has a view of the step that just ended still to build (see <see cref="PostStepFlush"/>).</summary>
    internal void MarkViewPending() => _viewPending = true;

    /// <summary>Counts the population counters of the last step if nothing has read them yet.</summary>
    internal void CapturePendingCounters()
    {
        if (_countersPending) { _countersPending = false; CapturePopulationCounters(); }
    }

    /// <summary>Computes whatever post-step result is still owed, before the world changes.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void FlushPostStep()
    {
        if (_countersPending | _viewPending) { FlushPostStepSlow(); }
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private void FlushPostStepSlow()
    {
        CapturePendingCounters();
        if (_viewPending) { _viewPending = false; PostStepFlush?.Invoke(); }
    }

    /// <summary>Per-world stage durations using an optional host timestamp source. Not saved or hashed.</summary>
    public StepProfile Profile { get; } = new();

    /// <summary>Returns a read-only body inspection view. Throws for an invalid or stale handle. Read only between steps.</summary>
    public BodyInspection InspectBody(BodyHandle body)
    {
        Resolve(body);
        return new BodyInspection(this, body);
    }

    private byte[] _newContacts = Array.Empty<byte>();
    private byte[] _newContactPoints = Array.Empty<byte>();

    private void ReserveInspectionContacts(int count)
    {
        if (_newContacts.Length >= count) { return; }
        int n = Math.Max(count, Math.Max(16, _newContacts.Length * 2));
        Array.Resize(ref _newContacts, n); Array.Resize(ref _newContactPoints, n);
    }

    private void CaptureContactsForInspection()
    {
        ReserveInspectionContacts(_constraints.HighWater);
        Array.Clear(_newContacts); Array.Clear(_newContactPoints);
    }

    internal void InspectContactBirth(int c, int count, int matchedMask)
    {
        // The existing warm-start match produced this mask; reduce it once per refreshed manifold.
        int matched = (matchedMask & 1) + ((matchedMask >> 1) & 1) + ((matchedMask >> 2) & 1) + ((matchedMask >> 3) & 1);
        _newContactPoints[c] = (byte)(count - matched);
    }

    private void CaptureSolverCounters()
    {
        Counters.AwakeBodies = _awakeCount;
        Counters.SerialOverflow = _coloring.Overflow.Length;
        for (int i = 0; i < ConstraintColoring.ColorCount; i++) { Counters.Colors[i] = _coloring.Color(i).Length; }
        _coloring.CaptureCounts(Counters);
        if (_options.DetailedInspection)
        {
            foreach (int c in _coloring.Active)
            {
                if (_constraints.KindOf(c) == ConstraintKind.Joint) { JointReadback.Track(c, true); }
            }
        }
        Counters.PersistedManifolds = Counters.Manifolds - Counters.NewManifolds - Counters.RecycledManifolds;
        Counters.PersistedContactPoints = Counters.ContactPoints - Counters.NewContactPoints - Counters.RecycledContactPoints;
    }

    private void CapturePopulationCounters()
    {
        foreach (int b in AwakeBodies)
        {
            byte state = BodyReadback.CcdState[b];
            if ((state & 1) != 0) { Counters.CcdBodies++; }
            if ((state & 2) != 0) { Counters.CcdHits++; }
            if ((state & 4) != 0) { Counters.BulletBodies++; }
        }
        for (int i = 0; i < _islands.HighWater; i++)
        {
            if (!_islands.IsLive(i)) { continue; }
            if (_islands.IsAsleep(i)) { Counters.AsleepIslands++; }
            else { Counters.AwakeIslands++; }
        }
    }
}
