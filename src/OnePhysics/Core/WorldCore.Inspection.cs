namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class WorldCore
{
    private bool _collectTransitions;

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

    private void CaptureSolverCounters()
    {
        Counters.AwakeBodies = _awakeCount;
        Counters.SerialOverflow = _coloring.Overflow.Length;
        for (int i = 0; i < ConstraintColoring.ColorCount; i++) { Counters.Colors[i] = _coloring.Color(i).Length; }
        foreach (int c in _coloring.Active)
        {
            if (_constraints.KindOf(c) == ConstraintKind.Joint) { Counters.Joints++; }
            else if (_constraints.KindOf(c) == ConstraintKind.Contact)
            {
                Counters.Manifolds++;
                Counters.ContactPoints += _constraints.PointCount[c];
            }
        }
    }

    private void CapturePopulationCounters()
    {
        for (int i = 0; i < _islands.HighWater; i++)
        {
            if (!_islands.IsLive(i)) { continue; }
            if (_islands.IsAsleep(i)) { Counters.AsleepIslands++; }
            else { Counters.AwakeIslands++; }
        }
    }
}
