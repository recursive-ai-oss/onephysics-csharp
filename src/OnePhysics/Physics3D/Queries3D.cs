using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// Which shapes a query may report. The default value (<c>default</c> or <c>new QueryFilter3D()</c>) accepts every shape except sensors.
/// A shape passes when its layer is in <see cref="LayerMask"/>, its category is in <see cref="MaskBits"/> and the query's <see cref="CategoryBits"/> intersect the shape's mask,
/// it is not a sensor (unless <see cref="IncludeSensors"/>), and <see cref="Predicate"/> (if any) returns true.
/// </summary>
public struct QueryFilter3D
{
    private uint _noLayers, _noCategory, _noMask;

    /// <summary>Bit i set: shapes on layer i may be reported (Unity's layer mask). Default all layers.</summary>
    public uint LayerMask { readonly get => ~_noLayers; set => _noLayers = ~value; }

    /// <summary>The query's own category bits, tested against each shape's mask. Default all.</summary>
    public uint CategoryBits { readonly get => ~_noCategory; set => _noCategory = ~value; }

    /// <summary>Categories of shapes the query accepts, tested against each shape's category. Default all.</summary>
    public uint MaskBits { readonly get => ~_noMask; set => _noMask = ~value; }

    /// <summary>Whether sensor shapes are reported. Default false.</summary>
    public bool IncludeSensors { get; set; }

    /// <summary>Optional extra test, called for every candidate that passed the bit tests, in traversal order (so the call order is not specified; make it a pure function). Batch queries call it from several threads.</summary>
    public Func<Shape, bool>? Predicate { get; set; }

    /// <summary>A filter accepting only the given layer bits.</summary>
    public static QueryFilter3D Layers(uint layerMask) => new() { LayerMask = layerMask };

    /// <summary>A filter accepting only one layer (0..31).</summary>
    /// <exception cref="ArgumentOutOfRangeException">The layer is outside 0..31.</exception>
    public static QueryFilter3D OnLayer(int layer)
    {
        Engine3D.CheckLayer(layer, nameof(layer));
        return new QueryFilter3D { LayerMask = 1u << layer };
    }
}

/// <summary>A query result. Which fields are meaningful depends on the query; see each method.</summary>
[System.Diagnostics.CodeAnalysis.SuppressMessage("Design", "CA1051:Do not declare visible instance fields", Justification = "Handle fields, not properties, so that `hit.Body.LinearVelocity = v` compiles: a property result is not a variable (CS1612), a readonly field is.")]
public readonly struct QueryHit3D
{
    internal QueryHit3D(Shape shape, Vector3D point, Vector3F normal, float fraction, float distance, int subShape)
    {
        Shape = shape;
        Body = shape.Body;
        Point = point;
        Normal = normal;
        Fraction = fraction;
        Distance = distance;
        SubShapeId = subShape;
        IsHit = true;
    }

    /// <summary>False for the default value (a batch entry without a hit).</summary>
    public bool IsHit { get; }

    /// <summary>The shape that was hit.</summary>
    public readonly Shape Shape;

    /// <summary>The body of <see cref="Shape"/>.</summary>
    public readonly Body Body;

    /// <summary>Ray and casts: the hit point on the shape (for a ray that starts inside, its origin). Overlaps and closest point: the point of the shape nearest the query's centre, or the centre itself when it is inside.</summary>
    public Vector3D Point { get; }

    /// <summary>Ray and casts: the surface normal at the hit, facing the ray or mover; zero when the query started inside the shape. Closest point: unit vector from the closest point towards the query point (zero when inside). Overlaps: zero.</summary>
    public Vector3F Normal { get; }

    /// <summary>Ray and casts: the fraction of the cast distance (0..1). Otherwise 0.</summary>
    public float Fraction { get; }

    /// <summary>Ray and casts: distance travelled to the hit, in metres. Overlaps and closest point: distance from the query centre to the shape's surface (0 inside).</summary>
    public float Distance { get; }

    /// <summary>The compound child or mesh or height-field triangle that was hit (the id the contact events use); 0 for a convex shape; -1 for overlaps, which report the shape only.</summary>
    public int SubShapeId { get; }

    /// <inheritdoc/>
    public override string ToString() =>
        IsHit ? "QueryHit(" + Body + ", distance " + Distance.ToString(System.Globalization.CultureInfo.InvariantCulture) + ")" : "QueryHit(none)";
}

/// <summary>One ray of a batch.</summary>
public readonly struct RayInput3D
{
    /// <summary>Creates a ray; validated when the batch runs.</summary>
    public RayInput3D(Vector3D origin, Vector3F direction, float maxDistance = float.PositiveInfinity)
    {
        Origin = origin;
        Direction = direction;
        MaxDistance = maxDistance;
    }

    /// <summary>Start point.</summary>
    public Vector3D Origin { get; }

    /// <summary>Direction; need not be a unit vector, must not be zero.</summary>
    public Vector3F Direction { get; }

    /// <summary>Length, positive; infinity means the largest supported distance.</summary>
    public float MaxDistance { get; }
}

/// <summary>One sphere of a batch (an overlap, or a cast when <see cref="Direction"/> is used).</summary>
public readonly struct SphereInput3D
{
    /// <summary>Creates a sphere query; validated when the batch runs.</summary>
    public SphereInput3D(Vector3D center, float radius, Vector3F direction = default, float maxDistance = 0f)
    {
        Center = center;
        Radius = radius;
        Direction = direction;
        MaxDistance = maxDistance;
    }

    /// <summary>Sphere centre.</summary>
    public Vector3D Center { get; }

    /// <summary>Radius, at least 0.</summary>
    public float Radius { get; }

    /// <summary>Cast direction (ignored by overlaps).</summary>
    public Vector3F Direction { get; }

    /// <summary>Cast length (ignored by overlaps).</summary>
    public float MaxDistance { get; }
}
