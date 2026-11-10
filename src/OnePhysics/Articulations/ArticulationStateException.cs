namespace RecursiveAI.OnePhysics;

/// <summary>An articulation's joint coordinates or speeds are outside the supported domain (not finite, a position beyond ±4194304 or a speed beyond 1e6 radians or metres per second). Raised before a step changes the world; repair the state with <c>SetPositions</c>/<c>SetVelocities</c> or destroy the articulation.</summary>
public sealed class ArticulationStateException : PhysicsException
{
    internal ArticulationStateException(int articulation, int index, string field, string message) : base(message)
    { Articulation = articulation; Index = index; Field = field; }

    /// <summary>Articulation slot.</summary>
    public int Articulation { get; }

    /// <summary>Index into the position or velocity array named by <see cref="Field"/>.</summary>
    public int Index { get; }

    /// <summary>"positions" or "velocities".</summary>
    public string Field { get; }
}
