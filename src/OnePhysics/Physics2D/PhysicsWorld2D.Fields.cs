using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Fields;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    /// <summary>Number of live fields and effectors. Default zero; unused fields cost no force passes.</summary>
    public int FieldCount => _engine.Fields.Count;
    /// <summary>Adds a copied field definition. Throws ArgumentNullException or ArgumentOutOfRangeException before changing the world. Body-origin region selection is inclusive.</summary>
    public Field AddField(FieldDefinition definition) => _engine.Fields.Add(definition);
    /// <summary>Adds a constant area acceleration in m/s², over the inclusive region. Defaults: no drag, all layers.</summary>
    public Field AddAreaEffector(Vector2F acceleration,Vector2D minimum,Vector2D maximum,float drag=0) => AddField(new FieldDefinition { Direction=new Vector3F(acceleration.X,acceleration.Y,0),Minimum=new Vector3D(minimum.X,minimum.Y,0),Maximum=new Vector3D(maximum.X,maximum.Y,0),Drag=drag });
    /// <summary>Adds a point effector; positive strength repels, negative attracts. Strength is m/s² times metres raised to the falloff exponent; default constant. Radius is a hard inclusive cutoff.</summary>
    public Field AddPointEffector(Vector2D center,float strength,float radius,FieldFalloff falloff=FieldFalloff.Constant) => AddField(new FieldDefinition {Kind=FieldKind.Point,Center=new Vector3D(center.X,center.Y,0),Strength=strength,Radius=radius,Falloff=falloff});
    /// <summary>Adds inward radial gravity, strength in m/s² times the falloff distance power. Default inverse-square, softening 0.1 m.</summary>
    public Field AddRadialGravity(Vector2D center,float strength,float radius,FieldFalloff falloff=FieldFalloff.InverseSquare) => AddField(new FieldDefinition {Kind=FieldKind.RadialGravity,Center=new Vector3D(center.X,center.Y,0),Strength=strength,Radius=radius,Falloff=falloff});
    /// <summary>Adds a replacement gravity zone. Highest priority wins; equal highest priorities blend with normalized positive weights. Gravity scale still applies.</summary>
    public Field AddGravityZone(Vector2F gravity,Vector2D minimum,Vector2D maximum,int priority=0,float weight=1) => AddField(new FieldDefinition {Kind=FieldKind.RegionalGravity,Direction=new Vector3F(gravity.X,gravity.Y,0),Minimum=new Vector3D(minimum.X,minimum.Y,0),Maximum=new Vector3D(maximum.X,maximum.Y,0),Priority=priority,Weight=weight});
    /// <summary>Adds an XY vortex (+Z axis), tangential and inward acceleration in m/s², positive tangential strength counterclockwise. Spatial axes can be changed through Definition.</summary>
    public Field AddVortex(Vector2D center,float tangential,float inward,float radius) => AddField(new FieldDefinition {Kind=FieldKind.Vortex,Center=new Vector3D(center.X,center.Y,0),Axis=new(0,0,1),Strength=tangential,Inward=inward,Radius=radius});
    /// <summary>Adds magnetic interactions between tagged dipoles within cutoff metres, with minimum distance 0.1 m. Untagged bodies are unaffected. Cutoff must be within 0.0001..2097152 m; Step throws if any cutoff-sized cell contains over 64 dipoles.</summary>
    public Field AddMagnetism(float strength,float cutoff) => AddField(new FieldDefinition {Kind=FieldKind.Magnetism,Strength=strength,Radius=cutoff});
    /// <summary>Sets a world-oriented dipole moment; zero removes the magnetic tag. Throws for stale or foreign handles or nonfinite moments. Raw tier.</summary>
    public void SetMagneticMoment(BodyHandle body,Vector2F moment)
    {
        _=GetBody(body);FieldDefinition.CheckVector(new Vector3F(moment.X,moment.Y,0),nameof(moment));
        using var mutation=RecordMutation("World.SetMagneticMoment");_engine.Fields.SetMoment(body.Index,new Vector3F(moment.X,moment.Y,0));
    }
    /// <summary>Binds host gravity context. Callbacks must be pure and identical on peers; rebind after scene load, fork or replay construction. Null removes a binding. Binding changes are not simulation state.</summary>
    public void BindGravityField(int callbackId,GravityField? callback) => _engine.Fields.Bind(callbackId,callback);
    /// <summary>Middle-tier struct callback adapter. Evaluations occur serially in field then body order; the host owns callback state.</summary>
    public void BindGravityField<T>(int callbackId,T callback) where T:struct,IGravityField => BindGravityField(callbackId,callback.Evaluate);
    /// <summary>Adds a buoyancy effector, density in kg/m^2, height in metres, viscous drag in 1/s. Defaults: still water, density 1000, damping 1. Invalid values throw ArgumentOutOfRangeException.</summary>
    public Field AddBuoyancyEffector(double surfaceHeight,float density=1000,Vector2F flow=default,float damping=1) => AddField(new FieldDefinition {Kind=FieldKind.Buoyancy,SurfaceHeight=surfaceHeight,Density=density,Flow=new Vector3F(flow.X,flow.Y,0),Drag=damping,AngularDrag=damping});
    /// <summary>Adds a contact conveyor in a region, direction (unit length recommended), target speed m/s. A contact body is accelerated toward the target at Drag (1/s), default 20.</summary>
    public Field AddSurfaceEffector(Vector2F direction,float speed,Vector2D minimum,Vector2D maximum) => AddField(new FieldDefinition {Kind=FieldKind.Surface,Direction=new Vector3F(direction.X,direction.Y,0),Strength=speed,Minimum=new Vector3D(minimum.X,minimum.Y,0),Maximum=new Vector3D(maximum.X,maximum.Y,0),Drag=20});
    /// <summary>Adds a one-way platform region, solid toward its unit normal. Side friction defaults to zero and bounce to zero. Invalid values throw ArgumentOutOfRangeException.</summary>
    public Field AddPlatformEffector(Vector2F normal,Vector2D minimum,Vector2D maximum,float sideFriction=0,float sideBounce=0) => AddField(new FieldDefinition {Kind=FieldKind.Platform,Axis=new Vector3F(normal.X,normal.Y,0),Minimum=new Vector3D(minimum.X,minimum.Y,0),Maximum=new Vector3D(maximum.X,maximum.Y,0),SideFriction=sideFriction,SideBounce=sideBounce});

    /// <summary>Adds host-defined gravity on the happy path. The callback must be pure and is not serialized. Rebind the returned Definition.CallbackId on a loaded or forked world.</summary>
    public Field AddGravityField(GravityField callback)
    {
        ArgumentNullException.ThrowIfNull(callback);
        int id=0;
        foreach(var definition in _engine.Fields.Definitions) if(definition is { Kind:FieldKind.CustomGravity })id=Math.Max(id,definition.CallbackId+1);
        var field=AddField(new(){Kind=FieldKind.CustomGravity,CallbackId=id});BindGravityField(id,callback);return field;
    }
    /// <summary>Raw tier: returns the live field at a stable slot. Throws ArgumentOutOfRangeException for a dead or out-of-range slot.</summary>
    public Field GetField(int index)
    {
        var store=_engine.Fields;
        if((uint)index>=(uint)store.Definitions.Length||store.Definitions[index]==null)throw new ArgumentOutOfRangeException(nameof(index),index,"No live field at this slot.");
        return new Field(store,index,store.Generations[index]);
    }
}
