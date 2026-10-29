using System;

namespace RecursiveAI.OnePhysics;

/// <summary>
/// The world options a snapshot carries (they are part of the saved state, so a restore overwrites the ones the world was built with). <c>Restore</c> returns the ones whose value it changed.
/// Options that are not in snapshots (thread count, capacities, solver tuning) are never touched by a restore.
/// </summary>
[Flags]
public enum OptionChanges
{
    /// <summary>The restore left every saved option as it was.</summary>
    None = 0,

    /// <summary><c>Gravity</c> changed.</summary>
    Gravity = 1,

    /// <summary><c>Substeps</c> changed.</summary>
    Substeps = 2,

    /// <summary><c>VelocityIterations</c> changed.</summary>
    VelocityIterations = 4,

    /// <summary><c>RelaxIterations</c> changed.</summary>
    RelaxIterations = 8,

    /// <summary><c>SleepEnabled</c> changed.</summary>
    SleepEnabled = 16,

    /// <summary><c>TimeToSleep</c> changed.</summary>
    TimeToSleep = 32,

    /// <summary><c>SleepThreshold</c> changed.</summary>
    SleepThreshold = 64,

    /// <summary><c>Events</c> (the event mask) changed.</summary>
    Events = 128,
}
