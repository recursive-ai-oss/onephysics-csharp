using System;
using RecursiveAI.OneMath;
using System.Diagnostics.CodeAnalysis;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>How much freedom a shape has when it is scaled.</summary>
[Experimental("OPHYS0001")]
public enum ScaleSupport3D : byte
{
    /// <summary>Only equal scale on all three axes keeps the shape what it is: sphere, capsule, tapered capsule, and any rounded box, cylinder or inflated hull.</summary>
    Uniform,

    /// <summary>The X and Z scales must be equal, Y is free: a sharp cylinder (its axis is Y, its cross-section a circle).</summary>
    AxisSymmetricY,

    /// <summary>Any positive scale per axis: a sharp box and a sharp hull.</summary>
    PerAxis,
}

/// <summary>
/// Scaled shapes. Scaling is applied when the shape is made: the result is an ordinary <see cref="ConvexShape3D"/> with the scaled dimensions, so GJK, EPA, casts,
/// mass properties and the manifold generator (including its analytic routes and feature ids) treat it exactly as the equivalent unscaled shape. A scaled hull is a new hull
/// built from the scaled vertices (a linear map keeps a polytope convex; the face planes follow the inverse-transpose map, which a rebuild recomputes).
/// <para>
/// Which shapes accept which scales is a question of staying the same kind of shape: a sphere scaled unevenly is an ellipsoid, a capsule scaled unevenly has elliptical caps, and a convex
/// radius is a sphere offset that a non-uniform scale would turn into an ellipsoid offset, so those need equal scale on all axes. A sharp cylinder keeps being a cylinder when its
/// circle is scaled evenly (X equal to Z); a sharp box or hull accepts any positive scale. Negative scale (a mirror) is refused: it would flip hull windings, and a mirrored
/// placement is better expressed in the transform of the child. See <see cref="GetScaleSupport"/>.
/// </para>
/// </summary>
[Experimental("OPHYS0001")]
public static class ScaledShape3D
{
    /// <summary>The scales <paramref name="shape"/> accepts.</summary>
    public static ScaleSupport3D GetScaleSupport(in ConvexShape3D shape)
    {
        bool sharp = shape.ConvexRadius == 0f;
        switch (shape.Kind)
        {
            case ConvexShapeKind3D.Box:
            case ConvexShapeKind3D.Hull:
                return sharp ? ScaleSupport3D.PerAxis : ScaleSupport3D.Uniform;
            case ConvexShapeKind3D.Cylinder:
                return sharp ? ScaleSupport3D.AxisSymmetricY : ScaleSupport3D.Uniform;
            default:
                return ScaleSupport3D.Uniform;
        }
    }

    /// <summary>Scales a shape by the same factor on every axis.</summary>
    /// <param name="shape">The shape; a default (point) shape cannot be scaled.</param>
    /// <param name="scale">Finite and positive.</param>
    /// <exception cref="ShapeCompositionException">The scale or the shape is not usable, or a scaled dimension leaves the range <see cref="ConvexShape3D"/> accepts.</exception>
    public static ConvexShape3D Create(in ConvexShape3D shape, float scale) => Create(shape, new Vector3F(scale, scale, scale));

    /// <summary>Scales a shape by one factor per axis, where the shape allows it (see <see cref="GetScaleSupport"/>).</summary>
    /// <param name="shape">The shape; a default (point) shape cannot be scaled.</param>
    /// <param name="scale">Each component finite and positive. Equal components are always accepted; unequal ones only where the shape kind allows.</param>
    /// <exception cref="ShapeCompositionException">The scale is NaN, infinite, zero or negative (<see cref="ShapeCompositionError.InvalidScale"/>), unequal where the shape needs equal scale
    /// (<see cref="ShapeCompositionError.UnsupportedScale"/>), or the result cannot be built (<see cref="ShapeCompositionError.ShapeBuildFailed"/>).</exception>
    public static ConvexShape3D Create(in ConvexShape3D shape, Vector3F scale)
    {
        CheckComponent(scale.X, "scale.X");
        CheckComponent(scale.Y, "scale.Y");
        CheckComponent(scale.Z, "scale.Z");
        if (shape.Kind == ConvexShapeKind3D.Sphere && shape.ConvexRadius == 0f)
        {
            throw new ShapeCompositionException(ShapeCompositionError.InvalidShape, "shape is the zero-size point shape; a sphere, box, capsule, cylinder or hull is expected.", nameof(shape));
        }

        bool uniform = scale.X == scale.Y && scale.Y == scale.Z;
        if (uniform && scale.X == 1f)
        {
            return shape;
        }

        ScaleSupport3D support = GetScaleSupport(shape);
        bool ok = support switch
        {
            ScaleSupport3D.PerAxis => true,
            ScaleSupport3D.AxisSymmetricY => scale.X == scale.Z,
            _ => uniform,
        };
        if (!ok)
        {
            string need = support == ScaleSupport3D.AxisSymmetricY ? "equal X and Z scale" : "equal scale on all three axes";
            throw new ShapeCompositionException(
                ShapeCompositionError.UnsupportedScale,
                $"scale is {scale} but a {shape.Kind} shape{(shape.ConvexRadius > 0f && shape.Kind != ConvexShapeKind3D.Sphere && shape.Kind != ConvexShapeKind3D.Capsule ? " with a convex radius" : string.Empty)} needs {need}.",
                nameof(scale));
        }

        try
        {
            return Build(shape, scale);
        }
        catch (ArgumentException e) when (e is ArgumentOutOfRangeException || e is HullBuildException)
        {
            throw new ShapeCompositionException(
                ShapeCompositionError.ShapeBuildFailed,
                $"scaling the {shape.Kind} shape by {scale} gave a shape that cannot be built: {e.Message}",
                nameof(scale),
                e);
        }
    }

    private static void CheckComponent(float v, string name)
    {
        if (!(v > 0f) || !float.IsFinite(v))
        {
            throw new ShapeCompositionException(ShapeCompositionError.InvalidScale, $"{name} is {v}; a finite value above 0 is expected.");
        }
    }

    private static ConvexShape3D Build(in ConvexShape3D shape, Vector3F s)
    {
        float r = shape.ConvexRadius;
        Vector3F z = shape.CoreSize;
        switch (shape.Kind)
        {
            case ConvexShapeKind3D.Sphere:
                return ConvexShape3D.Sphere(r * s.X);
            case ConvexShapeKind3D.Box:
                {
                    var outer = new Vector3F((z.X + r) * s.X, (z.Y + r) * s.Y, (z.Z + r) * s.Z);
                    float cr = r * s.X;
                    return ConvexShape3D.Box(outer, Math.Min(cr, Math.Min(outer.X, Math.Min(outer.Y, outer.Z))));
                }

            case ConvexShapeKind3D.Capsule:
                return ConvexShape3D.Capsule(z.Y * s.X, r * s.X);
            case ConvexShapeKind3D.TaperedCapsule:
                return ConvexShape3D.TaperedCapsule(z.Z * s.X, z.X * s.X, z.Y * s.X);
            case ConvexShapeKind3D.Cylinder:
                {
                    float radius = (z.X + r) * s.X, half = (z.Y + r) * s.Y;
                    float cr = r * s.X;
                    return ConvexShape3D.Cylinder(half, radius, Math.Min(cr, Math.Min(half, radius)));
                }

            default:
                {
                    ConvexHull3D hull = shape.Hull!;
                    var v = new Vector3F[hull.Vertices.Length];
                    for (int i = 0; i < v.Length; i++)
                    {
                        v[i] = hull.Vertices[i] * s;
                    }

                    return ConvexShape3D.FromHull(ConvexHull3D.Build(v), r * s.X);
                }
        }
    }
}
