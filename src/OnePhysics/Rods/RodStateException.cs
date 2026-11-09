namespace RecursiveAI.OnePhysics.Rods;

/// <summary>A rod particle or frame is outside the supported simulation domain. Raised before a step changes the world; repair the raw data, move or destroy the rod.</summary>
public sealed class RodStateException : PhysicsException
{
    internal RodStateException(int rod, int element, string field, string message) : base(message)
    { Rod = rod; Element = element; Field = field; }
    /// <summary>Rod slot.</summary>
    public int Rod { get; }
    /// <summary>Particle or segment index, as named by Field.</summary>
    public int Element { get; }
    /// <summary>Name of the invalid field.</summary>
    public string Field { get; }
}
