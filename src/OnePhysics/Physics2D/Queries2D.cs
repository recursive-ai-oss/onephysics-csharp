using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>
/// Which shapes a query may report. The default value (<c>default</c> or <c>new QueryFilter2D()</c>) accepts every shape except sensors.
/// A shape passes when its layer is in <see cref="LayerMask"/>, its category is in <see cref="MaskBits"/> and the query's <see cref="CategoryBits"/> intersect the shape's mask,
/// it is not a sensor (unless <see cref="IncludeSensors"/>), and <see cref="Predicate"/> (if any) returns true.
/// </summary>
public struct QueryFilter2D
{
    private uint _noLayers, _noCategory, _noMask;

    /// <summary>Bit i set: shapes on layer i may be reported (Unity's layer mask). Default all layers.</summary>
    public uint LayerMask { readonly get => ~_noLayers; set => _noLayers = ~value; }

    /// <summary>The query's own category bits, tested against each shape's mask (Box2D-style). Default all.</summary>
    public uint CategoryBits { readonly get => ~_noCategory; set => _noCategory = ~value; }

    /// <summary>Categories of shapes the query accepts, tested against each shape's category. Default all.</summary>
    public uint MaskBits { readonly get => ~_noMask; set => _noMask = ~value; }

    /// <summary>Whether sensor shapes are reported. Default false.</summary>
    public bool IncludeSensors { get; set; }

    /// <summary>Optional extra test, called for every candidate that passed the bit tests, in traversal order (so the call order is not specified; make it a pure function). Batch queries call it from several threads.</summary>
    public Func<Shape, bool>? Predicate { get; set; }

    /// <summary>A filter accepting only the given layer bits.</summary>
    public static QueryFilter2D Layers(uint layerMask) => new() { LayerMask = layerMask };

    /// <summary>A filter accepting only one layer (0..31).</summary>
    /// <exception cref="ArgumentOutOfRangeException">The layer is outside 0..31.</exception>
    public static QueryFilter2D OnLayer(int layer)
    {
        Engine2D.CheckLayer(layer, nameof(layer));
        return new QueryFilter2D { LayerMask = 1u << layer };
    }
}

/// <summary>A query result. Which fields are meaningful depends on the query; see each method.</summary>
public readonly struct QueryHit2D
{
    internal QueryHit2D(Shape shape, Vector2D point, Vector2F normal, float fraction, float distance)
    {
        Shape = shape;
        Point = point;
        Normal = normal;
        Fraction = fraction;
        Distance = distance;
        IsHit = true;
    }

    /// <summary>False for the default value (a batch entry without a hit).</summary>
    public bool IsHit { get; }

    /// <summary>The shape that was hit.</summary>
    public Shape Shape { get; }

    /// <summary>The body of <see cref="Shape"/>.</summary>
    public Body Body => Shape.Body;

    /// <summary>Ray and casts: the hit point on the shape (for a ray that starts inside, its origin). Overlaps and closest point: the point of the shape nearest the query's centre, or the centre itself when it is inside.</summary>
    public Vector2D Point { get; }

    /// <summary>Ray and casts: the surface normal at the hit, facing the ray or mover; zero when the query started inside the shape. Closest point: unit vector from the closest point towards the query point (zero when inside). Overlaps: zero.</summary>
    public Vector2F Normal { get; }

    /// <summary>Ray and casts: the fraction of the cast distance (0..1). Otherwise 0.</summary>
    public float Fraction { get; }

    /// <summary>Ray and casts: distance travelled to the hit, in metres. Overlaps and closest point: distance from the query centre to the shape's surface (0 inside).</summary>
    public float Distance { get; }
}

/// <summary>One ray of a batch.</summary>
public readonly struct RayInput2D
{
    /// <summary>Creates a ray; validated when the batch runs.</summary>
    public RayInput2D(Vector2D origin, Vector2F direction, float maxDistance = float.PositiveInfinity)
    {
        Origin = origin;
        Direction = direction;
        MaxDistance = maxDistance;
    }

    /// <summary>Start point.</summary>
    public Vector2D Origin { get; }

    /// <summary>Direction; need not be a unit vector, must not be zero.</summary>
    public Vector2F Direction { get; }

    /// <summary>Length, positive; infinity means the largest supported distance.</summary>
    public float MaxDistance { get; }
}

/// <summary>One circle of a batch (an overlap, or a cast when <see cref="Direction"/> is used).</summary>
public readonly struct CircleInput2D
{
    /// <summary>Creates a circle query; validated when the batch runs.</summary>
    public CircleInput2D(Vector2D center, float radius, Vector2F direction = default, float maxDistance = 0f)
    {
        Center = center;
        Radius = radius;
        Direction = direction;
        MaxDistance = maxDistance;
    }

    /// <summary>Circle centre.</summary>
    public Vector2D Center { get; }

    /// <summary>Radius, at least 0.</summary>
    public float Radius { get; }

    /// <summary>Cast direction (ignored by overlaps).</summary>
    public Vector2F Direction { get; }

    /// <summary>Cast length (ignored by overlaps).</summary>
    public float MaxDistance { get; }
}
