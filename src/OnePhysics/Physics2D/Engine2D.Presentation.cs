namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D
{
    /// <summary>True when the world has particle fluids (asking does not create the fluid store).</summary>
    internal bool HasParticleFluids => _particleFluids != null && _particleFluids.HasWork;
}
