using System;
using System.Globalization;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>
/// What turns a 2D triangle-mesh soft body into a strip of fabric or a membrane: direction-dependent stretch, a hard strain limit and wind. The planar analogue of the 3D cloth settings; set on <see cref="SoftBodyDef.Strip"/>.
/// </summary>
/// <remarks>
/// An edge of the rest mesh within about 26° of the <see cref="AlongAxis"/> is an <i>along</i> edge, within 26° of the perpendicular an <i>across</i> edge, any other a <i>shear</i> edge (Provot 1995).
/// Wind acts on the boundary: the windward face of every boundary edge takes the flat-plate force of the library's aerodynamic model (normal force and lift from the angle of attack), the lee face none; skin friction acts on both.
/// </remarks>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
public sealed class StripParams
{
    /// <summary>Largest strain limit accepted.</summary>
    public const float MaxStrain = 10f;

    /// <summary>Direction of the threads that run along the strip, in the mesh frame; any finite non-zero vector (normalised on use). Default +x.</summary>
    public Vector2F AlongAxis { get; set; } = new(1f, 0f);

    /// <summary>Compliance of along edges, m/N; a negative value (default -1) takes <see cref="SoftBodyDef.EdgeCompliance"/>.</summary>
    public float AlongCompliance { get; set; } = -1f;

    /// <summary>Compliance of across edges, m/N; a negative value (default -1) takes <see cref="SoftBodyDef.EdgeCompliance"/>.</summary>
    public float AcrossCompliance { get; set; } = -1f;

    /// <summary>Compliance of shear (diagonal) edges, m/N; a negative value (default -1) takes <see cref="SoftBodyDef.EdgeCompliance"/>.</summary>
    public float ShearCompliance { get; set; } = -1f;

    /// <summary>No edge may be longer than (1 + this) times its rest length at the end of a pass. 0 (default) is off. Finite, 0 to 10.</summary>
    public float StrainLimit { get; set; }

    /// <summary>Broadside drag coefficient of the windward face (a flat plate normal to the flow, 1.1 to 2); 0 (default) turns aerodynamics off. Finite, 0 to 100.</summary>
    public float Drag { get; set; }

    /// <summary>Lift slope dC<sub>L</sub>/dα (1/rad) at small angles of attack; default 2.</summary>
    public float LiftSlope { get; set; } = 2f;

    /// <summary>A copy of these settings.</summary>
    public StripParams Clone() => (StripParams)MemberwiseClone();

    internal bool Anisotropic => AlongCompliance >= 0f || AcrossCompliance >= 0f || ShearCompliance >= 0f;

    /// <summary>Checks every field.</summary>
    /// <exception cref="SoftBodyException">A field is NaN, infinite or out of range; the message names it.</exception>
    public void Validate()
    {
        Check(AlongCompliance, nameof(AlongCompliance), -1f, SoftBodyDef.MaxCompliance);
        Check(AcrossCompliance, nameof(AcrossCompliance), -1f, SoftBodyDef.MaxCompliance);
        Check(ShearCompliance, nameof(ShearCompliance), -1f, SoftBodyDef.MaxCompliance);
        Check(StrainLimit, nameof(StrainLimit), 0f, MaxStrain);
        Check(Drag, nameof(Drag), 0f, 100f);
        Check(LiftSlope, nameof(LiftSlope), 0f, 100f);
        float l2 = AlongAxis.X * AlongAxis.X + AlongAxis.Y * AlongAxis.Y;
        if (!float.IsFinite(l2) || !(l2 > 1e-12f)) { throw new SoftBodyException(SoftBodyError.InvalidDefinition, "StripParams.AlongAxis is " + AlongAxis + "; it must be finite and not zero.", nameof(AlongAxis)); }
    }

    private static void Check(float v, string name, float lo, float hi)
    {
        if (!float.IsFinite(v)) { throw new SoftBodyException(SoftBodyError.NonFinite, "StripParams." + name + " is " + v.ToString("R", CultureInfo.InvariantCulture) + "; it must be finite.", name); }
        if (v < lo || v > hi) { throw new SoftBodyException(SoftBodyError.OutOfRange, "StripParams." + name + " is " + v.ToString("R", CultureInfo.InvariantCulture) + "; expected " + lo.ToString("R", CultureInfo.InvariantCulture) + " to " + hi.ToString("R", CultureInfo.InvariantCulture) + ".", name); }
    }
}
