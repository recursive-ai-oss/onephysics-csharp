using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics;

/// <summary>Gravity helpers. This is the first public type of the package; it exists so the dependency on the deterministic math library is real.</summary>
public static class Gravity
{
    /// <summary>Standard gravity on Earth in metres per second squared, pointing along -Y: (0, -9.81, 0).</summary>
    public static Vector3F Standard => new(0f, -9.81f, 0f);

    /// <summary>Returns the strength (length) of an acceleration vector, in the vector's own units. Non-finite components follow IEEE arithmetic.</summary>
    public static float Strength(Vector3F acceleration) => acceleration.Length();
}
