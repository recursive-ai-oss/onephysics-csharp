namespace RecursiveAI.OnePhysics.Core;

/// <summary>Bounds on material coefficients, so combining two materials (a geometric mean needs their product) cannot overflow float32.</summary>
public static class MaterialLimits
{
    /// <summary>The largest friction or rolling resistance coefficient a shape or material accepts, 1e15.</summary>
    public const float MaxCoefficient = 1e15f;
}
