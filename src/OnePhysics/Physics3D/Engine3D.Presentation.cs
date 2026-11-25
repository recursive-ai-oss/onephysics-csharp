namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D
{
    /// <summary>True when the world has particle fluids (asking does not create the fluid store).</summary>
    internal bool HasParticleFluids => _particleFluids != null && _particleFluids.HasWork;
}
