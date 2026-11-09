using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Fields;

/// <summary>Force or contact effect evaluated in field-slot order before each fixed step.</summary>
public enum FieldKind
{
    /// <summary>Constant vector force, or acceleration when Acceleration is true.</summary>
    Area,
    /// <summary>Radial force; positive Strength repels, negative attracts.</summary>
    Point,
    /// <summary>Gravity toward a point, or toward the line through Center parallel to Axis.</summary>
    RadialGravity,
    /// <summary>Replacement gravity zone, blended among equal highest priorities.</summary>
    RegionalGravity,
    /// <summary>Host-supplied acceleration; bind by CallbackId after loading.</summary>
    CustomGravity,
    /// <summary>Pairwise magnetic dipoles, bounded by Radius.</summary>
    Magnetism,
    /// <summary>Right-handed tangential acceleration around Axis plus inward acceleration.</summary>
    Vortex,
    /// <summary>Fluid below the plane dot(Axis, position) = SurfaceHeight.</summary>
    Buoyancy,
    /// <summary>One-way contacts, solid in the Axis direction.</summary>
    Platform,
    /// <summary>Contact conveyor along Direction with speed Strength.</summary>
    Surface,
}

/// <summary>Distance law outside Softening; no singularity exists at the source.</summary>
public enum FieldFalloff
{
    /// <summary>Strength, independent of distance.</summary>
    Constant,
    /// <summary>Strength / max(distance, Softening).</summary>
    InverseLinear,
    /// <summary>Strength / max(distance, Softening)^2.</summary>
    InverseSquare,
}

/// <summary>
/// Shared definition for both dimensions. Planar fields use XY coordinates and +Z vortex axes.
/// Defaults describe a zero area acceleration in an inclusive box spanning ±2^21 metres.
/// Definitions are copied on creation/update. All numeric values must be finite; coordinates
/// within ±2^21, nonnegative drag/density/blend weight, positive radius/softening.
/// </summary>
public sealed class FieldDefinition
{
    /// <summary>Effect, default Area.</summary>
    public FieldKind Kind { get; set; }
    /// <summary>Inclusive body-centre-of-mass region minimum, metres.</summary>
    public Vector3D Minimum { get; set; } = new(-2097152, -2097152, -2097152);
    /// <summary>Inclusive body-centre-of-mass region maximum, metres. Buoyancy selects shapes by body origin too.</summary>
    public Vector3D Maximum { get; set; } = new(2097152, 2097152, 2097152);
    /// <summary>Radial source or axis point, metres.</summary>
    public Vector3D Center { get; set; }
    /// <summary>Constant force (N), acceleration (m/s²), gravity or conveyor direction. Default zero.</summary>
    public Vector3F Direction { get; set; }
    /// <summary>Unit axis: fluid/platform normal, vortex axis or gravity line. Default +Y. Must be unit length within 1e-5.</summary>
    public Vector3F Axis { get; set; } = new(0, 1, 0);
    /// <summary>Strength in N or m/s² (magnetism uses N m^4 per squared dipole unit); default zero.</summary>
    public float Strength { get; set; }
    /// <summary>Radial cutoff distance in metres, inclusive; default 100. Magnetic cutoff is restricted to 0.0001..2097152 m.</summary>
    public float Radius { get; set; } = 100;
    /// <summary>Minimum distance in distance laws, metres; default 0.1.</summary>
    public float Softening { get; set; } = 0.1f;
    /// <summary>Distance law, default constant.</summary>
    public FieldFalloff Falloff { get; set; }
    /// <summary>Area/point vectors are accelerations rather than forces by default.</summary>
    public bool Acceleration { get; set; } = true;
    /// <summary>Radial gravity attracts toward a line instead of a point; default false.</summary>
    public bool Line { get; set; }
    /// <summary>Linear velocity damping rate, 1/s; default zero. Uses a non-reversing implicit drag step.</summary>
    public float Drag { get; set; }
    /// <summary>Quadratic fluid drag coefficient, nonnegative, default zero.</summary>
    public float QuadraticDrag { get; set; }
    /// <summary>Angular velocity damping rate, 1/s; default zero.</summary>
    public float AngularDrag { get; set; }
    /// <summary>Ambient velocity, m/s; default zero.</summary>
    public Vector3F Flow { get; set; }
    /// <summary>Fluid density (kg/m² for planar unit-depth bodies, kg/m³ spatially), default 1000.</summary>
    public float Density { get; set; } = 1000;
    /// <summary>Fluid plane offset in metres along Axis; default zero.</summary>
    public double SurfaceHeight { get; set; }
    /// <summary>Inward vortex acceleration m/s²; default zero.</summary>
    public float Inward { get; set; }
    /// <summary>Gravity-zone priority; largest wins. Equal priorities blend in slot order.</summary>
    public int Priority { get; set; }
    /// <summary>Gravity-zone weight; default 1, positive. Weights are normalized across matching top-priority zones.</summary>
    public float Weight { get; set; } = 1;
    /// <summary>Affected shape-layer bit mask; default all. A body matches when any nonsensor shape matches.</summary>
    public uint Layers { get; set; } = uint.MaxValue;
    /// <summary>Optional sensor shape slot, -1 by default. Requires current sensor overlap with the body.</summary>
    public int Sensor { get; set; } = -1;
    /// <summary>Generation of Sensor; zero captures its current generation at field creation/update. Stale sensors disable the field.</summary>
    public int SensorGeneration { get; set; }
    /// <summary>Host callback key for CustomGravity; nonnegative, default zero. Bindings are not serialized.</summary>
    public int CallbackId { get; set; }
    /// <summary>Platform side friction (dimensionless), default zero.</summary>
    public float SideFriction { get; set; }
    /// <summary>Platform side restitution, 0..1, default zero.</summary>
    public float SideBounce { get; set; }
    /// <summary>Returns an independent definition.</summary>
    public FieldDefinition Clone() => (FieldDefinition)MemberwiseClone();
    /// <summary>Validates all values. Throws ArgumentOutOfRangeException naming the invalid property.</summary>
    public void Validate() => ValidateCore(2097152);
    internal void ValidateStored() => ValidateCore(4194304);
    private void ValidateCore(double coordinateBound)
    {
        CheckPoint(Minimum, nameof(Minimum),coordinateBound); CheckPoint(Maximum, nameof(Maximum),coordinateBound); CheckPoint(Center, nameof(Center),coordinateBound);
        if (Minimum.X > Maximum.X || Minimum.Y > Maximum.Y || Minimum.Z > Maximum.Z) throw Bad(nameof(Maximum), "Region bounds must be ordered.");
        CheckVector(Direction, nameof(Direction)); CheckVector(Axis, nameof(Axis)); CheckVector(Flow, nameof(Flow));
        float len = Axis.X*Axis.X + Axis.Y*Axis.Y + Axis.Z*Axis.Z;
        if (Math.Abs(len - 1) > 1e-5f) throw Bad(nameof(Axis), "Axis must be unit length.");
        if (!Enum.IsDefined(Kind)) throw Bad(nameof(Kind), "Field property is outside its supported range.");
        if (!Enum.IsDefined(Falloff)) throw Bad(nameof(Falloff), "Field property is outside its supported range.");
        Check(Strength, nameof(Strength)); Check(Inward, nameof(Inward));
        if(Kind==FieldKind.Magnetism && (Radius<0.0001f || Radius>2097152))throw Bad(nameof(Radius),"Magnetic cutoff must be within 0.0001..2097152 metres.");
        Positive(Radius, nameof(Radius)); Positive(Softening, nameof(Softening)); Positive(Weight, nameof(Weight));
        Nonnegative(Drag, nameof(Drag)); Nonnegative(QuadraticDrag,nameof(QuadraticDrag)); Nonnegative(AngularDrag, nameof(AngularDrag)); Nonnegative(Density, nameof(Density)); Nonnegative(SideFriction, nameof(SideFriction));
        Nonnegative(SideBounce, nameof(SideBounce));
        if (SideBounce > 1) throw Bad(nameof(SideBounce), "Field property is outside its supported range.");
        if (!double.IsFinite(SurfaceHeight) || Math.Abs(SurfaceHeight)>coordinateBound) throw Bad(nameof(SurfaceHeight), "Field property is outside its supported range.");
        if (Sensor < -1) throw Bad(nameof(Sensor), "Field property is outside its supported range.");
        if (SensorGeneration < 0) throw Bad(nameof(SensorGeneration), "Sensor generation must be nonnegative.");
        if (CallbackId < 0 || CallbackId == int.MaxValue) throw Bad(nameof(CallbackId), "Field property is outside its supported range.");
    }
    private static ArgumentOutOfRangeException Bad(string name,string message) => new(name,message);
    internal static void Check(float v, string name) { if (!float.IsFinite(v) || Math.Abs(v)>1e12f) throw new ArgumentOutOfRangeException(name, v, "Value must be finite within ±1e12."); }
    internal static void Positive(float v, string name) { Check(v,name); if (v<=0) throw new ArgumentOutOfRangeException(name, v, "Value must be positive."); }
    internal static void Nonnegative(float v, string name) { Check(v,name); if (v<0) throw new ArgumentOutOfRangeException(name, v, "Value must be nonnegative."); }
    internal static void CheckVector(Vector3F v, string name) { Check(v.X,name);Check(v.Y,name);Check(v.Z,name); }
    internal static void CheckPoint(Vector3D v, string name,double bound=2097152) { if (!double.IsFinite(v.X)||!double.IsFinite(v.Y)||!double.IsFinite(v.Z)||Math.Abs(v.X)>bound||Math.Abs(v.Y)>bound||Math.Abs(v.Z)>bound) throw new ArgumentOutOfRangeException(name,v,"Coordinates must be finite within the supported coordinate bound."); }
}

/// <summary>Acceleration callback for deterministic gravity, evaluated in field-slot then body-slot order. Must be pure and return finite m/s². Bindings are host context, like collision filter delegates, not snapshot state.</summary>
public delegate Vector3F GravityField(Vector3D position, long stepIndex);

/// <summary>Middle-tier gravity callback. Mutable callback state is host-owned and must be restored by the host.</summary>
public interface IGravityField
{
    /// <summary>Returns acceleration in m/s² at a world-relative position and completed-step index.</summary>
    Vector3F Evaluate(Vector3D position, long stepIndex);
}
