using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics;

public readonly partial struct ConvexShape3D
{
    private ConvexShape3D(in ConvexShape3D shape, Vector3F offset)
    {
        Kind = shape.Kind; _size = shape._size; _radius = shape._radius; _hull = shape._hull; _massOffset = offset;
    }

    /// <summary>Offsets the mass centre in metres without moving geometry; inertia follows the parallel-axis theorem. Nested offsets add before computing inertia. Zero-mass shapes remain massless.</summary>
    /// <exception cref="ArgumentOutOfRangeException">An offset component or its accumulated value is not finite or exceeds ±2^21.</exception>
    public ConvexShape3D WithCenterOfMassOffset(Vector3F offset)
    {
        CheckOffset(offset);
        var total = _massOffset + offset;
        CheckOffset(total);
        return new ConvexShape3D(this, total);
    }

    /// <summary>Mass-centre offset in metres, zero by default; it does not move collision geometry.</summary>
    public Vector3F CenterOfMassOffset => _massOffset;

    internal static void CheckOffset(Vector3F offset)
    {
        if (!float.IsFinite(offset.X) || !float.IsFinite(offset.Y) || !float.IsFinite(offset.Z) ||
            Math.Abs(offset.X) > MaxDimension || Math.Abs(offset.Y) > MaxDimension || Math.Abs(offset.Z) > MaxDimension)
            throw new ArgumentOutOfRangeException(nameof(offset), offset, "Offset components must be finite within ±2097152 metres.");
    }

    /// <summary>An infinite half-space dot(normal, point) ≤ offset. Normal must be finite and nonzero; it is normalized, with offset divided by the same length. Offset must be finite within ±2^21 metres after normalization. May only be attached to static bodies, never compound children.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The normal is zero/non-finite or the normalized offset exceeds ±2^21.</exception>
    public static ConvexShape3D Plane(Vector3F normal, float offset = 0f)
    {
        double length = double.Sqrt((double)normal.X * normal.X + (double)normal.Y * normal.Y + (double)normal.Z * normal.Z);
        double distance = offset / length;
        if (!(length > 0) || !double.IsFinite(length) || !double.IsFinite(distance) || Math.Abs(distance) > MaxDimension)
            throw new ArgumentOutOfRangeException(nameof(normal), normal, "Plane normal must be finite and nonzero, and normalized offset finite within ±2097152 metres.");
        return new ConvexShape3D(ConvexShapeKind3D.Plane, new Vector3F((float)(normal.X / length), (float)(normal.Y / length), (float)(normal.Z / length)), (float)distance, null);
    }

    /// <summary>Outward unit normal of a plane, zero for other shapes.</summary>
    public Vector3F PlaneNormal => Kind == ConvexShapeKind3D.Plane ? _size : default;

    /// <summary>Offset in dot(normal, point) ≤ offset, in metres; zero for other shapes.</summary>
    public float PlaneOffset => Kind == ConvexShapeKind3D.Plane ? _radius : 0f;
}
