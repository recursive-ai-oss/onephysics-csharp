using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Rods;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>Which deformables a query looks at: kinds, layers and categories (the same bit rules as <see cref="QueryFilter2D"/>, applied to the layer and category and mask bits of the soft body or rod).</summary>
public struct DeformableFilter2D
{
    private uint _noLayers, _noCategory, _noMask;
    private byte _noKinds;

    /// <summary>Kinds of deformable reported. Default all.</summary>
    public DeformableKinds Kinds { readonly get => (DeformableKinds)(~_noKinds & 7); set => _noKinds = (byte)(~(byte)value & 7); }

    /// <summary>Bit i set: deformables on layer i may be reported. Default all layers.</summary>
    public uint LayerMask { readonly get => ~_noLayers; set => _noLayers = ~value; }

    /// <summary>The query's own category bits, tested against each deformable's mask. Default all.</summary>
    public uint CategoryBits { readonly get => ~_noCategory; set => _noCategory = ~value; }

    /// <summary>Categories of deformables the query accepts, tested against each deformable's category. Default all.</summary>
    public uint MaskBits { readonly get => ~_noMask; set => _noMask = ~value; }

    /// <summary>A filter for the given kinds only.</summary>
    public static DeformableFilter2D Of(DeformableKinds kinds) => new() { Kinds = kinds };
}

/// <summary>A hit on a deformable: which one (kind, slot and generation: <c>GetSoftBody(slot)</c> or <c>GetRod(slot)</c>), which element of it, where, and the weights of the point on the element.</summary>
public readonly struct DeformableHit2D
{
    internal DeformableHit2D(DeformableKind kind, int slot, int generation, DeformableElement element, int index, int a, int b, Vector2F weights, Vector2D point, Vector2F normal, float fraction, float distance)
    {
        IsHit = true; Kind = kind; Slot = slot; Generation = generation; Element = element; Index = index; ParticleA = a; ParticleB = b;
        Barycentric = weights; Point = point; Normal = normal; Fraction = fraction; Distance = distance;
    }

    /// <summary>False for the default value (no hit).</summary>
    public bool IsHit { get; }

    /// <summary>What kind of deformable was hit.</summary>
    public DeformableKind Kind { get; }

    /// <summary>The soft body's or rod's slot.</summary>
    public int Slot { get; }

    /// <summary>The generation of the soft body or rod at that slot.</summary>
    public int Generation { get; }

    /// <summary>The kind of element hit: a particle or a segment (a rod segment or an edge of a soft body).</summary>
    public DeformableElement Element { get; }

    /// <summary>The element's index: the particle, the rod segment (segment i joins particles i and i + 1), the outline edge of a soft body (edge i joins outline vertices i and i + 1, wrapping) or the edge of a strip in the body's edge list.</summary>
    public int Index { get; }

    /// <summary>The first particle of the element.</summary>
    public int ParticleA { get; }

    /// <summary>The second particle of a segment, -1 for a particle.</summary>
    public int ParticleB { get; }

    /// <summary>Weights of the hit point on <see cref="ParticleA"/> and <see cref="ParticleB"/>, summing to 1: (1, 0) for a particle; for a segment the point projected on it.</summary>
    public Vector2F Barycentric { get; }

    /// <summary>The point on the element: where a ray entered, where a cast touched, the nearest point for a closest-point query, a point of contact for an overlap.</summary>
    public Vector2D Point { get; }

    /// <summary>Unit normal of the element's surface at the point, pointing towards the query (zero when the query starts inside, or for an overlap).</summary>
    public Vector2F Normal { get; }

    /// <summary>Distance travelled along the ray or cast as a fraction of its length; 0 for overlaps and closest points.</summary>
    public float Fraction { get; }

    /// <summary>Distance from the ray origin or cast start to the hit, or from the query point to the element (closest point); 0 for overlaps.</summary>
    public float Distance { get; }
}

internal struct DeformElem2
{
    public DeformableKind Kind;
    public int Slot, Generation;
    public DeformableElement Element;
    public int Index, PA, PB;
    public double Ax, Ay, Bx, By;
    public float Radius;
    public int Points;
}

internal interface IDeformVisitor2
{
    bool Visit(in DeformElem2 e);
}

internal static class DeformableScan2D
{
    public static bool Scan<V>(Engine2D e, in DeformableFilter2D f, double lox, double loy, double hix, double hiy, ref V v) where V : struct, IDeformVisitor2
    {
        var kinds = f.Kinds;
        uint layers = f.LayerMask, cat = f.CategoryBits, mask = f.MaskBits;
        var el = new DeformElem2();
        for (int slot = 0; slot < e.SoftHandles.HighWater; slot++)
        {
            if (!e.SoftHandles.IsLive(slot)) { continue; }
            var d = e.SoftData[slot]!;
            // Cloth: a strip or membrane, or a mesh with no area (no triangles and no enclosed area held by pressure).
            bool cloth = Engine2D.IsClothLike(d);
            if ((kinds & (cloth ? DeformableKinds.Cloth : DeformableKinds.SoftBodies)) == 0) { continue; }
            if ((layers & (1u << d.Def.Layer)) == 0 || (cat & d.Def.MaskBits) == 0 || (d.Def.CategoryBits & mask) == 0) { continue; }
            int n = d.N;
            if (n == 0) { continue; }
            double r = d.Radius;
            double mnx = double.MaxValue, mny = double.MaxValue, mxx = -double.MaxValue, mxy = -double.MaxValue;
            var pos = d.Pos;
            for (int p = 0; p < n; p++)
            {
                double x = pos[2 * p], y = pos[2 * p + 1];
                if (x < mnx) { mnx = x; } if (x > mxx) { mxx = x; } if (y < mny) { mny = y; } if (y > mxy) { mxy = y; }
            }

            if (mxx + r < lox || mnx - r > hix || mxy + r < loy || mny - r > hiy) { continue; }
            el.Kind = cloth ? DeformableKind.Cloth : DeformableKind.SoftBody;
            el.Slot = slot; el.Generation = e.SoftHandles.GenerationOf(slot);
            el.Radius = d.Radius;
            bool outline = !cloth && d.Outline.Length >= 3;
            int edges = outline ? d.Outline.Length : d.EdgeA.Length;
            if (edges > 0)
            {
                el.Element = DeformableElement.Segment; el.Points = 2;
                for (int i = 0; i < edges; i++)
                {
                    int a = outline ? d.Outline[i] : d.EdgeA[i], b = outline ? d.Outline[(i + 1) % edges] : d.EdgeB[i];
                    el.Ax = pos[2 * a]; el.Ay = pos[2 * a + 1]; el.Bx = pos[2 * b]; el.By = pos[2 * b + 1];
                    if (Math.Max(el.Ax, el.Bx) + r < lox || Math.Min(el.Ax, el.Bx) - r > hix || Math.Max(el.Ay, el.By) + r < loy || Math.Min(el.Ay, el.By) - r > hiy) { continue; }
                    el.Index = i; el.PA = a; el.PB = b;
                    if (v.Visit(in el)) { return true; }
                }
            }
            else
            {
                el.Element = DeformableElement.Particle; el.Points = 1; el.PB = -1;
                for (int p = 0; p < n; p++)
                {
                    el.Ax = pos[2 * p]; el.Ay = pos[2 * p + 1];
                    if (el.Ax + r < lox || el.Ax - r > hix || el.Ay + r < loy || el.Ay - r > hiy) { continue; }
                    el.Index = p; el.PA = p;
                    if (v.Visit(in el)) { return true; }
                }
            }
        }

        if ((kinds & DeformableKinds.Rods) == 0) { return false; }
        var rods = e.RodsOrNull;
        if (rods == null) { return false; }
        for (int slot = 0; slot < rods.Data.Length; slot++)
        {
            var rod = rods.Data[slot];
            if (rod == null) { continue; }
            if ((layers & (1u << rod.Layer)) == 0 || (cat & rod.Mask) == 0 || (rod.Category & mask) == 0) { continue; }
            double r = rod.Material.Radius;
            el.Kind = DeformableKind.Rod; el.Slot = slot; el.Generation = rods.Generations[slot];
            el.Element = DeformableElement.Segment; el.Points = 2; el.Radius = (float)r;
            var pp = rod.PositionData;
            for (int i = 0; i + 1 < pp.Length; i++)
            {
                var a = pp[i]; var b = pp[i + 1];
                if (Math.Max(a.X, b.X) + r < lox || Math.Min(a.X, b.X) - r > hix || Math.Max(a.Y, b.Y) + r < loy || Math.Min(a.Y, b.Y) - r > hiy) { continue; }
                el.Ax = a.X; el.Ay = a.Y; el.Bx = b.X; el.By = b.Y; el.Index = i; el.PA = i; el.PB = i + 1;
                if (v.Visit(in el)) { return true; }
            }
        }

        return false;
    }

    /// <summary>The element as a shape in a frame whose origin is <paramref name="ox"/>, <paramref name="oy"/> and whose axes are the world's.</summary>
    public static Shape2D Shape(in DeformElem2 e, double ox, double oy)
    {
        float r = Math.Max(e.Radius, 1e-5f);
        var a = new Vec2((float)(e.Ax - ox), (float)(e.Ay - oy));
        if (e.Points == 1) { return Shape2D.CreateCircle(a, r); }
        var b = new Vec2((float)(e.Bx - ox), (float)(e.By - oy));
        if ((b - a).Length < Constants2D.MinShapeExtent * 4f) { return Shape2D.CreateCircle(new Vec2(0.5f * (a.X + b.X), 0.5f * (a.Y + b.Y)), r); }
        return Shape2D.CreateCapsule(a, b, r);
    }

    public static Vector2F Weights(in DeformElem2 e, Vector2D p)
    {
        if (e.Points == 1) { return new Vector2F(1f, 0f); }
        double dx = e.Bx - e.Ax, dy = e.By - e.Ay, l2 = dx * dx + dy * dy;
        double t = l2 > 0.0 ? ((p.X - e.Ax) * dx + (p.Y - e.Ay) * dy) / l2 : 0.0;
        t = t < 0.0 ? 0.0 : t > 1.0 ? 1.0 : t;
        return new Vector2F((float)(1.0 - t), (float)t);
    }

    public static DeformableHit2D Make(in DeformElem2 e, Vector2D point, Vector2F normal, float fraction, float distance) =>
        new(e.Kind, e.Slot, e.Generation, e.Element, e.Index, e.PA, e.Points == 2 ? e.PB : -1, Weights(in e, point), point, normal, fraction, distance);

    public static bool Before(in DeformableHit2D a, in DeformableHit2D b)
    {
        if (a.Distance != b.Distance) { return a.Distance < b.Distance; }
        if (a.Kind != b.Kind) { return a.Kind < b.Kind; }
        if (a.Slot != b.Slot) { return a.Slot < b.Slot; }
        if (a.Element != b.Element) { return a.Element < b.Element; }
        return a.Index < b.Index;
    }
}

internal struct DeformableVisitor2 : IDeformVisitor2
{
    public const int Ray = 0, Cast = 1, Overlap = 2, Closest = 3, CountOverlap = 4;

    public int Mode, Total;
    public double Ox, Oy;
    public Rot2 Rotation;
    public Vec2 Direction;            // unit
    public float Length;
    public Shape2D Shape;
    public bool Any;
    public DeformableHit2D[] Buffer;
    public int Capacity, Count;

    public bool Visit(in DeformElem2 e)
    {
        var shape = DeformableScan2D.Shape(in e, Ox, Oy);
        if (Mode == Ray)
        {
            var o = shape.RayCast(default, new Vec2(Direction.X * Length, Direction.Y * Length), 1f);
            if (!o.Hit) { return false; }
            return Add(DeformableScan2D.Make(in e, new Vector2D(Ox + o.Point.X, Oy + o.Point.Y), new Vector2F(o.Normal.X, o.Normal.Y), o.Fraction, o.Fraction * Length));
        }

        if (Mode == Cast)
        {
            var o = Distance2D.ShapeCast(shape, Shape, new Transform2(default, Rotation), new Vec2(Direction.X * Length, Direction.Y * Length), 1f);
            if (!o.Hit) { return false; }
            return Add(DeformableScan2D.Make(in e, new Vector2D(Ox + o.Point.X, Oy + o.Point.Y), new Vector2F(o.Normal.X, o.Normal.Y), o.Fraction, o.Fraction * Length));
        }

        var cache = new SimplexCache2D();
        var d = Distance2D.Compute(shape, Shape, new Transform2(default, Rotation), ref cache, true);
        bool overlap = d.Distance <= 0f;
        if (Mode == CountOverlap) { if (overlap) { Total++; } return false; }
        var pt = new Vector2D(Ox + d.PointA.X, Oy + d.PointA.Y);
        if (Mode == Overlap) { return overlap && Add(DeformableScan2D.Make(in e, pt, default, 0f, 0f)); }
        if (!(d.Distance <= Length)) { return false; }
        Vector2F n = default;
        if (!overlap)
        {
            float dx = d.PointB.X - d.PointA.X, dy = d.PointB.Y - d.PointA.Y, l = float.Sqrt(dx * dx + dy * dy);
            if (l > 0f) { n = new Vector2F(dx / l, dy / l); }
        }

        return Add(DeformableScan2D.Make(in e, pt, n, 0f, overlap ? 0f : d.Distance));
    }

    private bool Add(in DeformableHit2D h)
    {
        if (Any) { Buffer[0] = h; Count = 1; return true; }
        int lo = 0, hi = Count;
        while (lo < hi)
        {
            int mid = (lo + hi) >> 1;
            if (DeformableScan2D.Before(in Buffer[mid], in h)) { lo = mid + 1; } else { hi = mid; }
        }

        if (lo >= Capacity) { return false; }
        int last = Count < Capacity ? Count : Capacity - 1;
        if (last > lo) { Array.Copy(Buffer, lo, Buffer, lo + 1, last - lo); }
        Buffer[lo] = h;
        if (Count < Capacity) { Count++; }
        return false;
    }
}

internal sealed partial class Engine2D
{
    internal RodStore? RodsOrNull => _rods;

    internal static bool IsClothLike(SoftBodyData d) => d.Def.Strip != null || (d.Tris.Length == 0 && d.PressureRest <= 0f);

    internal static DeformableKind SoftKindOf(SoftBodyData d) => IsClothLike(d) ? DeformableKind.Cloth : DeformableKind.SoftBody;

    [ThreadStatic]
    private static DeformableHit2D[]? t_deformHits;

    internal static DeformableHit2D[] DeformScratch(int capacity)
    {
        var b = t_deformHits;
        if (b == null || b.Length < capacity) { b = t_deformHits = new DeformableHit2D[Math.Max(capacity, 16)]; }
        return b;
    }
}
