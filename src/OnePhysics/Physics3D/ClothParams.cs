using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// What turns a triangle-mesh soft body into a fabric: direction-dependent stretch and shear, a hard strain limit, bending against a rest curvature, wind, and long-range tethers. Set on
/// <see cref="SoftBodyDef.Fabric"/>; <see cref="ClothDef"/> fills it from a material. Every field is part of the world's state.
/// </summary>
/// <remarks>
/// The weave axes are given in the mesh's own frame. An edge of the rest mesh within about 26° of the warp axis is a <i>warp</i> edge, within 26° of the weft axis a <i>weft</i> edge, any other a <i>shear</i> edge
/// (the diagonals of a square grid), and each class has its own compliance (Provot 1995, structural and shear springs).
/// </remarks>
public sealed class ClothParams
{
    /// <summary>Largest strain limit accepted.</summary>
    public const float MaxStrain = 10f;

    /// <summary>Direction of the warp threads in the mesh frame; a unit vector (normalised on use). Default +x.</summary>
    public Vector3F WarpAxis { get; set; } = new(1f, 0f, 0f);

    /// <summary>Direction of the weft threads in the mesh frame; a unit vector (normalised on use). Default +z.</summary>
    public Vector3F WeftAxis { get; set; } = new(0f, 0f, 1f);

    /// <summary>Compliance of warp edges, m/N; a negative value (default -1) takes <see cref="SoftBodyDef.EdgeCompliance"/>.</summary>
    public float WarpCompliance { get; set; } = -1f;

    /// <summary>Compliance of weft edges, m/N; a negative value (default -1) takes <see cref="SoftBodyDef.EdgeCompliance"/>.</summary>
    public float WeftCompliance { get; set; } = -1f;

    /// <summary>Compliance of shear (diagonal) edges, m/N; a negative value (default -1) takes <see cref="SoftBodyDef.EdgeCompliance"/>.</summary>
    public float ShearCompliance { get; set; } = -1f;

    /// <summary>Strain limiting: no edge may be longer than (1 + this) times its rest length at the end of a pass (Provot 1995; Goldenthal et al. 2007). 0 (default) is off. Finite, 0 to 10.</summary>
    public float StrainLimit { get; set; }

    /// <summary>Bend against the rest shape's dihedral angles instead of the isometric (flat-resting) model, so a mesh whose rest shape is curved holds that curvature (Bridson et al. 2003; Müller et al. 2007).
    /// The bend compliance is then per radian of angle error. Default false.</summary>
    public bool DihedralBend { get; set; }

    /// <summary>Broadside drag coefficient C<sub>D</sub> of the fabric in wind and in fall (a flat plate normal to the flow, 1.1 to 2); 0 (default) turns aerodynamics off. Finite, 0 to 100.</summary>
    public float Drag { get; set; }

    /// <summary>Lift slope dC<sub>L</sub>/dα (1/rad) of the fabric at small angles of attack; default 2.</summary>
    public float LiftSlope { get; set; } = 2f;

    /// <summary>Tether anchors: particles (typically the pinned ones) that every other particle is limited to stay within (1 + <see cref="TetherStrain"/>) times its rest distance of its nearest anchor (Kim, Chentanez and Müller 2012,
    /// long range attachments). Null or empty (default) means none. Particle numbers of the mesh; duplicates are ignored.</summary>
    public IReadOnlyList<int>? TetherAnchors { get; set; }

    /// <summary>Tether slack as a strain, 0 to 10; used with <see cref="TetherAnchors"/>. Default 0.02.</summary>
    public float TetherStrain { get; set; } = 0.02f;

    /// <summary>A copy of these settings.</summary>
    public ClothParams Clone()
    {
        var c = (ClothParams)MemberwiseClone();
        c.TetherAnchors = TetherAnchors == null ? null : new List<int>(TetherAnchors);
        return c;
    }

    internal bool Anisotropic => WarpCompliance >= 0f || WeftCompliance >= 0f || ShearCompliance >= 0f;

    internal bool HasLimits => StrainLimit > 0f || (TetherAnchors != null && TetherAnchors.Count > 0);

    /// <summary>Checks every field.</summary>
    /// <param name="vertexCount">The mesh's vertex count, for the tether anchors.</param>
    /// <exception cref="SoftBodyException">A field is NaN, infinite or out of range; the message names it.</exception>
    public void Validate(int vertexCount)
    {
        Check(WarpCompliance, nameof(WarpCompliance), -1f, SoftBodyDef.MaxCompliance);
        Check(WeftCompliance, nameof(WeftCompliance), -1f, SoftBodyDef.MaxCompliance);
        Check(ShearCompliance, nameof(ShearCompliance), -1f, SoftBodyDef.MaxCompliance);
        Check(StrainLimit, nameof(StrainLimit), 0f, MaxStrain);
        Check(Drag, nameof(Drag), 0f, 100f);
        Check(LiftSlope, nameof(LiftSlope), 0f, 100f);
        Check(TetherStrain, nameof(TetherStrain), 0f, MaxStrain);
        Axis(WarpAxis, nameof(WarpAxis)); Axis(WeftAxis, nameof(WeftAxis));
        var a = Norm(WarpAxis); var b = Norm(WeftAxis);
        if (Math.Abs(a.X * b.X + a.Y * b.Y + a.Z * b.Z) > 0.5f) { throw new SoftBodyException(SoftBodyError.InvalidDefinition, "ClothParams.WarpAxis and WeftAxis are within 60° of each other; the weave axes must be roughly perpendicular.", nameof(WeftAxis)); }
        if (TetherAnchors != null)
        {
            for (int i = 0; i < TetherAnchors.Count; i++)
            {
                int k = TetherAnchors[i];
                if ((uint)k >= (uint)vertexCount) { throw new SoftBodyException(SoftBodyError.BadIndex, "ClothParams.TetherAnchors[" + i.ToString(CultureInfo.InvariantCulture) + "] is " + k.ToString(CultureInfo.InvariantCulture) + "; particle indices must be in 0 to " + (vertexCount - 1).ToString(CultureInfo.InvariantCulture) + ".", nameof(TetherAnchors)); }
            }
        }
    }

    internal static Vector3F Norm(Vector3F v)
    {
        float l = float.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z);
        return new Vector3F(v.X / l, v.Y / l, v.Z / l);
    }

    private static void Axis(Vector3F v, string name)
    {
        float l2 = v.X * v.X + v.Y * v.Y + v.Z * v.Z;
        if (!float.IsFinite(l2) || !(l2 > 1e-12f)) { throw new SoftBodyException(SoftBodyError.InvalidDefinition, "ClothParams." + name + " is " + v + "; it must be finite and not zero.", name); }
    }

    private static void Check(float v, string name, float lo, float hi)
    {
        if (!float.IsFinite(v)) { throw new SoftBodyException(SoftBodyError.NonFinite, "ClothParams." + name + " is " + v.ToString("R", CultureInfo.InvariantCulture) + "; it must be finite.", name); }
        if (v < lo || v > hi) { throw new SoftBodyException(SoftBodyError.OutOfRange, "ClothParams." + name + " is " + v.ToString("R", CultureInfo.InvariantCulture) + "; expected " + lo.ToString("R", CultureInfo.InvariantCulture) + " to " + hi.ToString("R", CultureInfo.InvariantCulture) + ".", name); }
    }
}
