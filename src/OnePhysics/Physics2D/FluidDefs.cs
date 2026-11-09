using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Fields;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>Planar fluid, with unit-depth density in kg/m², a flat surface and drag. The world copies definitions. Polygon-clipped area and centroid determine buoyancy.</summary>
public sealed class FluidDef
{
    /// <summary>Coordinate bound for new fluids, inclusive ±2^21 metres.</summary>
    public const double MaxCoordinate=2097152;
    /// <summary>Outward normal, default +Y; any finite nonzero vector is normalized.</summary>
    public Vector2F SurfaceNormal { get; set; }=new(0,1);
    /// <summary>Surface dot(SurfaceNormal,position), metres; scaled with normalization. Default zero.</summary>
    public double SurfaceOffset { get; set; }
    /// <summary>Density kg/m², default 1000, finite nonnegative.</summary>
    public float Density { get; set; }=1000;
    /// <summary>Flow velocity m/s, default still.</summary>
    public Vector2F Flow { get; set; }
    /// <summary>Quadratic drag coefficient, default 0.5, finite nonnegative; projected width is sqrt(submerged area).</summary>
    public float LinearDrag { get; set; }=0.5f;
    /// <summary>Angular damping rate 1/s, default 0.5, finite nonnegative.</summary>
    public float AngularDrag { get; set; }=0.5f;
    /// <summary>Viscous damping 1/s weighted by displaced mass, default zero.</summary>
    public float ViscousDamping { get; set; }
    /// <summary>Restricts body centres of mass to RegionMin..RegionMax when true; default false.</summary>
    public bool LimitToRegion { get; set; }
    /// <summary>Inclusive region minimum, metres.</summary>
    public Vector2D RegionMin { get; set; }
    /// <summary>Inclusive region maximum, metres.</summary>
    public Vector2D RegionMax { get; set; }
    /// <summary>Returns a copy.</summary>
    public FluidDef Clone()=>(FluidDef)MemberwiseClone();
    /// <summary>Validates values, throwing ArgumentOutOfRangeException for a named invalid property.</summary>
    public void Validate() => ToField().Validate();
    internal FieldDefinition ToField()
    {
        double length=Math.Sqrt((double)SurfaceNormal.X*SurfaceNormal.X+(double)SurfaceNormal.Y*SurfaceNormal.Y);
        if(!double.IsFinite(length)||length<=1e-12)throw new ArgumentOutOfRangeException(nameof(SurfaceNormal),"Normal must be finite and nonzero.");
        var d=new FieldDefinition {Kind=FieldKind.Buoyancy,Axis=new((float)(SurfaceNormal.X/length),(float)(SurfaceNormal.Y/length),0),SurfaceHeight=SurfaceOffset/length,Density=Density,Flow=new(Flow.X,Flow.Y,0),Drag=ViscousDamping,QuadraticDrag=LinearDrag,AngularDrag=AngularDrag};
        if(LimitToRegion) {d.Minimum=new(RegionMin.X,RegionMin.Y,0);d.Maximum=new(RegionMax.X,RegionMax.Y,0);}
        return d;
    }
    internal static FluidDef FromField(FieldDefinition d)=>new(){SurfaceNormal=new(d.Axis.X,d.Axis.Y),SurfaceOffset=d.SurfaceHeight,Density=d.Density,Flow=new(d.Flow.X,d.Flow.Y),LinearDrag=d.QuadraticDrag,AngularDrag=d.AngularDrag,ViscousDamping=d.Drag,LimitToRegion=true,RegionMin=new(d.Minimum.X,d.Minimum.Y),RegionMax=new(d.Maximum.X,d.Maximum.Y)};
}

/// <summary>Generation-checked planar fluid handle. A removed or default fluid throws InvalidOperationException when accessed.</summary>
public readonly struct Fluid
{
    private readonly Field _field;
    internal Fluid(Field field)=>_field=field;
    /// <summary>True while live.</summary>
    public bool IsValid=>_field.IsValid;
    /// <summary>A copy of the definition.</summary>
    public FluidDef Definition=>FluidDef.FromField(_field.Definition);
    /// <summary>Replaces a copied, validated definition; invalid input leaves the world unchanged.</summary>
    public void SetDefinition(FluidDef definition) {ArgumentNullException.ThrowIfNull(definition);var d=definition.ToField();var field=_field;field.Definition=d;}
    /// <summary>Surface plane offset, metres; setting validates first.</summary>
    public double SurfaceOffset {get=>_field.Definition.SurfaceHeight;set{var d=Definition;d.SurfaceOffset=value;SetDefinition(d);}}
    /// <summary>Flow m/s; setting validates first.</summary>
    public Vector2F Flow {get{var d=_field.Definition;return new(d.Flow.X,d.Flow.Y);}set{var d=Definition;d.Flow=value;SetDefinition(d);}}
    /// <summary>Removes the fluid; stale handles throw.</summary>
    public void Remove()=>_field.Destroy();
}
