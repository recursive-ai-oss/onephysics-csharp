using System;

namespace RecursiveAI.OnePhysics;

/// <summary>Serial stage boundaries shared by both engines. A duration belongs to the stage that starts at that boundary.</summary>
public enum StepStage
{
    /// <summary>Bounds update.</summary>
    Boxes,
    /// <summary>Candidate pairs.</summary>
    Pairs,
    /// <summary>Collision detection.</summary>
    Narrowphase,
    /// <summary>Ordered manifold updates.</summary>
    Apply,
    /// <summary>Active constraints and schedules.</summary>
    Coloring,
    /// <summary>Constraint preparation.</summary>
    Prepare,
    /// <summary>Constraint passes and integration.</summary>
    Solve,
    /// <summary>Final body work, including CCD.</summary>
    Finalize,
    /// <summary>Solver result storage.</summary>
    Store,
    /// <summary>Break processing and sleeping.</summary>
    Sleep,
    /// <summary>Event collection.</summary>
    Events,
    /// <summary>End of rigid step; not a timed stage.</summary>
    End,
    /// <summary>Broadphase moves.</summary>
    Move,
    /// <summary>Start pose initialization.</summary>
    Init,
    /// <summary>Broadphase search.</summary>
    Search,
    /// <summary>Pair sorting.</summary>
    Sort,
    /// <summary>Pair buffer finalization.</summary>
    Alloc,
    /// <summary>Soft-body setup.</summary>
    SoftSetup,
    /// <summary>Soft-body prediction.</summary>
    SoftPredict,
    /// <summary>Soft-body collision detection.</summary>
    SoftDetect,
    /// <summary>Soft-body particle pairs.</summary>
    SoftPairs,
    /// <summary>Soft-body constraint solving.</summary>
    SoftConstraints,
    /// <summary>Soft-body contacts.</summary>
    SoftContacts,
    /// <summary>Soft-body pair solving.</summary>
    SoftPairSolve,
    /// <summary>Soft-body velocity and position finalization.</summary>
    SoftFinish,
    /// <summary>End of soft step; not a timed stage.</summary>
    SoftEnd,
}

/// <summary>
/// Reused, per-world stage durations in host timestamp units. Not snapshotted or hashed. Reads no clock itself.
/// Read between steps; the next step clears the durations. Timing excludes user event callbacks.
/// </summary>
public sealed class StepProfile
{
    private readonly long[] _durations = new long[26];
    private Func<long>? _source, _captured;
    private long _previous;
    private int _stage = -1;

    /// <summary>
    /// Optional monotonic timestamp source; null disables timing (default). Set between steps.
    /// Must not throw, mutate a world, or call Step. Differences are in its own units; the host supplies conversion to seconds.
    /// A decreasing timestamp contributes zero and increments InvalidTimestampCount rather than affecting simulation.
    /// </summary>
    public Func<long>? TimestampSource { get; set; }

    /// <summary>Durations indexed by StepStage, in host timestamp units, accumulated over repeated stages. End sentinels stay zero.</summary>
    public ReadOnlySpan<long> Durations => _durations;

    /// <summary>Number of decreasing timestamps in the last step; zero when timing is disabled.</summary>
    public int InvalidTimestampCount { get; private set; }

    internal void Begin()
    {
        Array.Clear(_durations);
        InvalidTimestampCount = 0;
        _source = _captured = TimestampSource;
        _stage = -1;
    }

    internal void BeginSoft() { _source = _captured; _stage = -1; }

    internal void Mark(StepStage stage)
    {
        if (_source == null) { return; }
        long now = _source();
        if (_stage >= 0)
        {
            if (now < _previous) { InvalidTimestampCount++; }
            else { _durations[_stage] += now - _previous; }
        }
        _previous = now;
        _stage = stage is StepStage.End or StepStage.SoftEnd ? -1 : (int)stage;
        if (_stage < 0) { _source = null; }
    }
}
