using System;

namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class WorldCore
{
    private bool _collectTransitions;

    /// <summary>Unsaved body input and damping impulses of the last step, in world axes.</summary>
    public BodyReadback BodyReadback { get; }

    /// <summary>Unsaved whole-step joint impulses and last-pass row data, shared by both engines.</summary>
    public JointReadback JointReadback { get; }

    /// <summary>Always-on diagnostics of the last step. Reused; not saved or hashed.</summary>
    public StepCounters Counters { get; } = new();

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
        foreach (int c in _coloring.Active)
        {
            if (_constraints.KindOf(c) == ConstraintKind.Joint) { if (_options.DetailedInspection) { JointReadback.Track(c, true); } Counters.Joints++; }
            else if (_constraints.KindOf(c) == ConstraintKind.Contact)
            {
                Counters.Manifolds++;
                int count = _constraints.PointCount[c];
                Counters.ContactPoints += count;
                if (_newContacts[c] != 0) { Counters.NewManifolds++; }
                Counters.NewContactPoints += _newContactPoints[c];
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
