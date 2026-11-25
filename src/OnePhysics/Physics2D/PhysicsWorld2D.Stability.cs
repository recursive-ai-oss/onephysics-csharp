using System;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    /// <summary>
    /// Measures how hard it is to tip an assembly of bodies resting on other bodies: support polygon, margin, tip angle and tipping push per horizontal direction. Read-only; it changes no state, so call it after <see cref="Settle"/> or any step.
    /// The support is the contact points between the assembly and everything outside it as the last step left them (see <see cref="StabilityMargin2D"/> for the estimate). Allocates the result arrays.
    /// </summary>
    /// <param name="assembly">One or more dynamic bodies of this world.</param>
    /// <param name="options">Defaults when null.</param>
    /// <exception cref="ArgumentException">The assembly is empty or holds a static or kinematic body.</exception>
    /// <exception cref="ArgumentOutOfRangeException">An option is out of range.</exception>
    /// <exception cref="InvalidHandleException">A body is null or belongs to another world.</exception>
    /// <exception cref="StaleHandleException">A body was destroyed.</exception>
    /// <exception cref="InvalidOperationException">The world has no gravity.</exception>
    public StabilityMargin2D GetStabilityMargin(ReadOnlySpan<Body> assembly, StabilityOptions2D? options = null)
    {
        _core.FlushPostStep();
        var handles = new BodyHandle[assembly.Length];
        for (int i = 0; i < handles.Length; i++)
        {
            if (!ReferenceEquals(assembly[i].World, this)) throw new InvalidHandleException("A body of the assembly is null or belongs to a different world.");
            handles[i] = assembly[i].RawHandle;
        }
        return StabilityMargin2D.Compute(this, handles, (options ?? new StabilityOptions2D()).WithDefaultsFor(Options.LengthUnit));
    }
}

public readonly partial struct Body
{
    /// <summary>The stability margin of this body alone; see <see cref="PhysicsWorld2D.GetStabilityMargin"/>.</summary>
    public StabilityMargin2D GetStabilityMargin(StabilityOptions2D? options = null)
    {
        var (world, _) = Resolve();
        return world.GetStabilityMargin([this], options);
    }
}
