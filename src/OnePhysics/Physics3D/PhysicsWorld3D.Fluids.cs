using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
{
    /// <summary>Adds a level pool of water filling everything below a height (3D worlds only).</summary>
    /// <param name="surfaceHeight">Height of the surface along the world's y axis, metres.</param>
    /// <param name="density">Fluid density, kg/m³. Default 1000.</param>
    /// <param name="flow">Velocity of the water, m/s. Default still.</param>
    /// <param name="damping">The fluid's <see cref="FluidDef.ViscousDamping"/>, 1/s. Default 1, which settles floating bodies in a few seconds.</param>
    /// <returns>A handle to the fluid.</returns>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite or is out of range.</exception>
    public Fluid AddWater(double surfaceHeight, float density = 1000f, Vector3F flow = default, float damping = 1f) =>
        _engine.AddFluid(new FluidDef { SurfaceOffset = surfaceHeight, Density = density, Flow = flow, ViscousDamping = damping });

    /// <summary>Adds a fluid from a definition (middle tier; 3D worlds only). The definition is copied; sleeping bodies wake.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A field is out of range (see <see cref="FluidDef"/>).</exception>
    public Fluid AddFluid(FluidDef def)
    {
        ArgumentNullException.ThrowIfNull(def);
        return _engine.AddFluid(def);
    }

    /// <summary>Number of fluids in the world.</summary>
    public int FluidCount => _engine.FluidHandles.Count;

    /// <summary>
    /// Raw tier, for hosts with their own water: adds the buoyancy and drag a body would get from the fluid described by <paramref name="fluid"/> to its force and torque for the next step, whether or not the fluid is
    /// part of the world. Call it once per step per fluid, before <see cref="Step"/>; the forces are cleared by the step like <see cref="Body.ApplyForce"/>'s. The body wakes when any part of it is submerged.
    /// </summary>
    /// <param name="body">A dynamic body.</param>
    /// <param name="fluid">The fluid; validated like <see cref="AddFluid"/>.</param>
    /// <param name="dt">The step length that will follow, seconds; the drag is limited so that it never reverses the motion within one step. Finite and positive.</param>
    /// <returns>The submerged volume of the body in m³, 0 when it is clear of the fluid.</returns>
    /// <exception cref="InvalidOperationException">The body is not dynamic.</exception>
    /// <exception cref="ArgumentOutOfRangeException">The fluid or <paramref name="dt"/> is out of range.</exception>
    public float ApplyBuoyancy(Body body, FluidDef fluid, float dt)
    {
        ArgumentNullException.ThrowIfNull(fluid);
        if (!(dt > 0f) || !float.IsFinite(dt)) { throw new ArgumentOutOfRangeException(nameof(dt), dt, "dt must be finite and positive."); }
        int b = ResolveBody(body.RawHandle);
        if (_core.Bodies.TypeOf(b) != BodyType.Dynamic) { throw new BodyTypeException(nameof(ApplyBuoyancy), body.RawHandle, _core.Bodies.TypeOf(b), true); }
        var prepared = Engine3D.Prepare(fluid);
        float v = _engine.ApplyFluidToBody(b, prepared, dt);
        if (v > 0f) { _core.WakeIsland(_core.Bodies.Island[b]); _core.Bodies.SleepTime[b] = 0f; }
        return v;
    }
}

public readonly partial struct Body
{
    /// <summary>How strongly fluids lift this body: its buoyant force is multiplied by this factor. 1 (default) is true Archimedes, 0 makes the body ignore buoyancy (drag still acts), above 1 floats it higher.
    /// Finite, 0 to 10^6. Changing it wakes the body. 3D bodies only.</summary>
    /// <exception cref="ArgumentOutOfRangeException">Setting a value that is NaN, infinite or outside the range.</exception>
    public float BuoyancyFactor
    {
        get { var (w, b) = Resolve(); return w.Engine.BuoyancyFactorOf(b); }
        set
        {
            var (w, b) = Resolve();
            if (!(value >= 0f && value <= 1e6f)) { throw new ArgumentOutOfRangeException(nameof(value), value, "BuoyancyFactor must be within 0 to 1000000."); }
            w.Engine.SetBuoyancyFactor(b, value);
            WakeBody(w, b);
        }
    }
}
