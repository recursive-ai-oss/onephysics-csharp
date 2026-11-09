using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Fields;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    /// <summary>Adds water below a height in metres; density kg/m² defaults to 1000, flow to still and viscous damping to 1/s. Values are validated before creation.</summary>
    public Fluid AddWater(double surfaceHeight,float density=1000,Vector2F flow=default,float damping=1)=>AddFluid(new(){SurfaceOffset=surfaceHeight,Density=density,Flow=flow,ViscousDamping=damping});
    /// <summary>Adds a copied, validated fluid definition. Invalid input throws ArgumentNullException or ArgumentOutOfRangeException.</summary>
    public Fluid AddFluid(FluidDef definition) {ArgumentNullException.ThrowIfNull(definition);return new Fluid(AddField(definition.ToField()));}
    /// <summary>Number of live buoyancy fields, including fluids created with AddWater.</summary>
    public int FluidCount { get {int count=0;foreach(var definition in _engine.Fields.Definitions)if(definition?.Kind==FieldKind.Buoyancy)count++;return count;} }
    /// <summary>Raw tier: applies buoyancy/drag once for the next step and returns submerged area, m². Dynamic body required; dt must be finite positive seconds. Invalid input throws before applying forces.</summary>
    public float ApplyBuoyancy(Body body,FluidDef fluid,float dt)
    {
        ArgumentNullException.ThrowIfNull(fluid);FieldDefinition.Positive(dt,nameof(dt));var definition=fluid.ToField();definition.Validate();
        int b=_core.Resolve(body.RawHandle);if(!_core.Bodies.IsDynamic(b))throw new BodyTypeException(nameof(ApplyBuoyancy),body.RawHandle,_core.Bodies.TypeOf(b),true);
        using var mutation=RecordMutation("World.ApplyBuoyancy");return _engine.Fields.Matches(b,definition)?_engine.ApplyFieldBuoyancy(b,definition,dt):0;
    }
}
