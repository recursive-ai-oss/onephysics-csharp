using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Rods;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>Which deformables a query looks at: kinds, layers and categories (the same bit rules as <see cref="QueryFilter3D"/>, applied to the layer and category and mask bits of the soft body or rod).</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public struct DeformableFilter3D
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
    public static DeformableFilter3D Of(DeformableKinds kinds) => new() { Kinds = kinds };
}

/// <summary>
/// A hit on a deformable: which one (kind, slot and generation: <c>GetSoftBody(slot)</c> or <c>GetRod(slot)</c>), which element of it, where, and the barycentric weights of the point on the element.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public readonly struct DeformableHit3D
{
    internal DeformableHit3D(DeformableKind kind, int slot, int generation, DeformableElement element, int index, int a, int b, int c, Vector3F weights, Vector3D point, Vector3F normal, float fraction, float distance)
    {
        IsHit = true; Kind = kind; Slot = slot; Generation = generation; Element = element; Index = index; ParticleA = a; ParticleB = b; ParticleC = c;
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

    /// <summary>The kind of element hit.</summary>
    public DeformableElement Element { get; }

    /// <summary>The element's index: the particle (particles), the surface triangle (soft bodies and cloth: triangle i is particles 3i..3i+2 of the body's surface list) or the segment (segment i joins particles i and i + 1 of a rod).</summary>
    public int Index { get; }

    /// <summary>The first particle of the element (the only one for a particle).</summary>
    public int ParticleA { get; }

    /// <summary>The second particle of a segment or triangle, -1 for a particle.</summary>
    public int ParticleB { get; }

    /// <summary>The third particle of a triangle, -1 otherwise.</summary>
    public int ParticleC { get; }

    /// <summary>Weights of the hit point on the element's particles (<see cref="ParticleA"/>, <see cref="ParticleB"/>, <see cref="ParticleC"/>), summing to 1: (1, 0, 0) for a particle; for a segment the point projected on it; for a triangle the point projected on its plane (outside the triangle a weight is negative).</summary>
    public Vector3F Barycentric { get; }

    /// <summary>The point on the element: where a ray entered, where a cast touched, the nearest point for a closest-point query, a point of contact for an overlap.</summary>
    public Vector3D Point { get; }

    /// <summary>Unit normal of the element's surface at the point, pointing towards the query (zero when the query starts inside, or for an overlap of a flat element).</summary>
    public Vector3F Normal { get; }

    /// <summary>Distance travelled along the ray or cast as a fraction of its length; 0 for overlaps and closest points.</summary>
    public float Fraction { get; }

    /// <summary>Distance from the ray origin or cast start to the hit, or from the query point to the element (closest point); 0 for overlaps.</summary>
    public float Distance { get; }
}

/// <summary>One element of a deformable as the queries see it: one to three points (double, world coordinates) and a radius.</summary>
internal struct DeformElem3
{
    public DeformableKind Kind;
    public int Slot, Generation;
    public DeformableElement Element;
    public int Index, PA, PB, PC;
    public Vector3D A, B, C;
    public float Radius;
    public int Points;
}

internal interface IDeformVisitor3
{
    /// <summary>Looks at one element; true stops the scan.</summary>
    bool Visit(in DeformElem3 e);
}

internal static class DeformableScan3D
{
    internal const float Thin = 1e-5f;

    /// <summary>
    /// Visits the elements whose boxes meet [lo, hi]: soft bodies in slot order (their surface triangles, or their particles when they have none), then rods in slot order (their segments), with the filter applied.
    /// </summary>
    public static bool Scan<V>(Engine3D e, in DeformableFilter3D f, double lox, double loy, double loz, double hix, double hiy, double hiz, ref V v) where V : struct, IDeformVisitor3
    {
        var kinds = f.Kinds;
        uint layers = f.LayerMask, cat = f.CategoryBits, mask = f.MaskBits;
        var el = new DeformElem3();
        for (int slot = 0; slot < e.SoftHandles.HighWater; slot++)
        {
            if (!e.SoftHandles.IsLive(slot)) { continue; }
            var d = e.SoftData[slot]!;
            // Cloth: fabric, or a surface with no volume (no tetrahedra and no enclosed volume held by pressure).
            bool cloth = Engine3D.IsClothLike(d);
            if ((kinds & (cloth ? DeformableKinds.Cloth : DeformableKinds.SoftBodies)) == 0) { continue; }
            if ((layers & (1u << d.Def.Layer)) == 0 || (cat & d.Def.MaskBits) == 0 || (d.Def.CategoryBits & mask) == 0) { continue; }
            int n = d.N;
            if (n == 0) { continue; }
            double r = d.Radius;
            double mnx = double.MaxValue, mny = double.MaxValue, mnz = double.MaxValue, mxx = -double.MaxValue, mxy = -double.MaxValue, mxz = -double.MaxValue;
            var pos = d.Pos;
            for (int p = 0; p < n; p++)
            {
                double x = pos[3 * p], y = pos[3 * p + 1], z = pos[3 * p + 2];
                if (x < mnx) { mnx = x; } if (x > mxx) { mxx = x; } if (y < mny) { mny = y; } if (y > mxy) { mxy = y; } if (z < mnz) { mnz = z; } if (z > mxz) { mxz = z; }
            }

            if (mxx + r < lox || mnx - r > hix || mxy + r < loy || mny - r > hiy || mxz + r < loz || mnz - r > hiz) { continue; }
            el.Kind = cloth ? DeformableKind.Cloth : DeformableKind.SoftBody;
            el.Slot = slot; el.Generation = e.SoftHandles.GenerationOf(slot);
            var tris = d.SurfTris;
            if (tris.Length >= 3)
            {
                el.Element = DeformableElement.Triangle; el.Points = 3; el.Radius = 0f;
                for (int t = 0; t < tris.Length / 3; t++)
                {
                    int a = tris[3 * t], b = tris[3 * t + 1], c = tris[3 * t + 2];
                    el.A = new Vector3D(pos[3 * a], pos[3 * a + 1], pos[3 * a + 2]); el.B = new Vector3D(pos[3 * b], pos[3 * b + 1], pos[3 * b + 2]); el.C = new Vector3D(pos[3 * c], pos[3 * c + 1], pos[3 * c + 2]);
                    if (Math.Max(el.A.X, Math.Max(el.B.X, el.C.X)) < lox || Math.Min(el.A.X, Math.Min(el.B.X, el.C.X)) > hix
                        || Math.Max(el.A.Y, Math.Max(el.B.Y, el.C.Y)) < loy || Math.Min(el.A.Y, Math.Min(el.B.Y, el.C.Y)) > hiy
                        || Math.Max(el.A.Z, Math.Max(el.B.Z, el.C.Z)) < loz || Math.Min(el.A.Z, Math.Min(el.B.Z, el.C.Z)) > hiz) { continue; }
                    el.Index = t; el.PA = a; el.PB = b; el.PC = c;
                    if (v.Visit(in el)) { return true; }
                }
            }
            else
            {
                el.Element = DeformableElement.Particle; el.Points = 1; el.Radius = d.Radius; el.PB = -1; el.PC = -1;
                for (int p = 0; p < n; p++)
                {
                    el.A = new Vector3D(pos[3 * p], pos[3 * p + 1], pos[3 * p + 2]);
                    if (el.A.X + r < lox || el.A.X - r > hix || el.A.Y + r < loy || el.A.Y - r > hiy || el.A.Z + r < loz || el.A.Z - r > hiz) { continue; }
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
            el.Element = DeformableElement.Segment; el.Points = 2; el.Radius = (float)r; el.PC = -1;
            var pp = rod.PositionData;
            for (int i = 0; i + 1 < pp.Length; i++)
            {
                var a = pp[i]; var b = pp[i + 1];
                if (Math.Max(a.X, b.X) + r < lox || Math.Min(a.X, b.X) - r > hix || Math.Max(a.Y, b.Y) + r < loy || Math.Min(a.Y, b.Y) - r > hiy
                    || Math.Max(a.Z, b.Z) + r < loz || Math.Min(a.Z, b.Z) - r > hiz) { continue; }
                el.A = a; el.B = b; el.Index = i; el.PA = i; el.PB = i + 1;
                if (v.Visit(in el)) { return true; }
            }
        }

        return false;
    }

    /// <summary>Places an element in a frame whose origin is <paramref name="origin"/> and whose axes are the world's turned by the inverse of <paramref name="inverse"/>'s inverse (that is, <paramref name="inverse"/> takes world vectors into the frame).</summary>
    public static ConvexShape3D Shape(in DeformElem3 e, Vector3D origin, QuaternionF inverse, out RigidTransformF pose)
    {
        switch (e.Points)
        {
            case 1:
            {
                var c = inverse.Rotate(Rel(e.A, origin));
                pose = new RigidTransformF(QuaternionF.Identity, c);
                return Radius(e.Radius) > 0f ? ConvexShape3D.Sphere(Radius(e.Radius)) : ConvexShape3D.Point;
            }
            case 2:
            {
                var a = inverse.Rotate(Rel(e.A, origin)); var b = inverse.Rotate(Rel(e.B, origin));
                var axis = b - a;
                float len = float.Sqrt(axis.X * axis.X + axis.Y * axis.Y + axis.Z * axis.Z);
                var mid = new Vector3F(0.5f * (a.X + b.X), 0.5f * (a.Y + b.Y), 0.5f * (a.Z + b.Z));
                float rad = Math.Max(Radius(e.Radius), Thin);
                if (!(len > 0f) || !float.IsFinite(len))
                {
                    pose = new RigidTransformF(QuaternionF.Identity, mid);
                    return ConvexShape3D.Sphere(rad);
                }

                float half = 0.5f * len;
                // The capsule must not be absurdly thin for its length; a finer rod than that is treated as one a millionth as thick as it is long.
                if (rad < len * 1e-6f) { rad = len * 1e-6f; }
                pose = new RigidTransformF(Engine3D.AlignY(new Vector3F(axis.X / len, axis.Y / len, axis.Z / len)), mid);
                return ConvexShape3D.Capsule(half, rad);
            }
            default:
            {
                var a = inverse.Rotate(Rel(e.A, origin)); var b = inverse.Rotate(Rel(e.B, origin)); var c = inverse.Rotate(Rel(e.C, origin));
                pose = new RigidTransformF(QuaternionF.Identity, default);
                return NarrowScratch.TriBuf.Set(a, b, c);
            }
        }
    }

    private static float Radius(float r) => r > 0f ? r : 0f;

    public static Vector3F Rel(Vector3D p, Vector3D origin) => new((float)(p.X - origin.X), (float)(p.Y - origin.Y), (float)(p.Z - origin.Z));

    /// <summary>Weights of a world point on the element's particles (see <see cref="DeformableHit3D.Barycentric"/>).</summary>
    public static Vector3F Weights(in DeformElem3 e, Vector3D p)
    {
        if (e.Points == 1) { return new Vector3F(1f, 0f, 0f); }
        if (e.Points == 2)
        {
            var d = e.B - e.A;
            double l2 = d.X * d.X + d.Y * d.Y + d.Z * d.Z;
            double t = l2 > 0.0 ? ((p.X - e.A.X) * d.X + (p.Y - e.A.Y) * d.Y + (p.Z - e.A.Z) * d.Z) / l2 : 0.0;
            t = t < 0.0 ? 0.0 : t > 1.0 ? 1.0 : t;
            return new Vector3F((float)(1.0 - t), (float)t, 0f);
        }

        // Ericson, Real-Time Collision Detection, 3.4: barycentric coordinates of a point with respect to a triangle.
        var v0 = e.B - e.A; var v1 = e.C - e.A; var v2 = p - e.A;
        double d00 = Dot(v0, v0), d01 = Dot(v0, v1), d11 = Dot(v1, v1), d20 = Dot(v2, v0), d21 = Dot(v2, v1);
        double denom = d00 * d11 - d01 * d01;
        if (!(Math.Abs(denom) > 1e-300)) { return new Vector3F(1f, 0f, 0f); }
        double vv = (d11 * d20 - d01 * d21) / denom, ww = (d00 * d21 - d01 * d20) / denom;
        return new Vector3F((float)(1.0 - vv - ww), (float)vv, (float)ww);
    }

    private static double Dot(Vector3D a, Vector3D b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z;

    public static DeformableHit3D Make(in DeformElem3 e, Vector3D point, Vector3F normal, float fraction, float distance) =>
        new(e.Kind, e.Slot, e.Generation, e.Element, e.Index, e.PA, e.Points >= 2 ? e.PB : -1, e.Points == 3 ? e.PC : -1, Weights(in e, point), point, normal, fraction, distance);

    /// <summary>True when hit <paramref name="a"/> sorts before <paramref name="b"/>: nearer first, then by kind, slot, element and index.</summary>
    public static bool Before(in DeformableHit3D a, in DeformableHit3D b)
    {
        if (a.Distance != b.Distance) { return a.Distance < b.Distance; }
        if (a.Kind != b.Kind) { return a.Kind < b.Kind; }
        if (a.Slot != b.Slot) { return a.Slot < b.Slot; }
        if (a.Element != b.Element) { return a.Element < b.Element; }
        return a.Index < b.Index;
    }
}

/// <summary>The one visitor all the deformable queries share: a mode, the query's own data and a small sorted buffer of the best hits.</summary>
internal struct DeformableVisitor3 : IDeformVisitor3
{
    public const int Ray = 0, Cast = 1, Overlap = 2, Closest = 3, CountOverlap = 4;

    /// <summary>Elements counted by <see cref="CountOverlap"/> mode.</summary>
    public int Total;

    public int Mode;
    public Vector3D Origin;           // ray origin, cast or overlap position, or the closest-point query's point
    public QuaternionF Rotation, Inverse;
    public Vector3F Direction;        // unit, world frame
    public float Length;              // ray or cast length; the search distance for closest points
    public ConvexShape3D Shape;
    public bool Any;
    public DeformableHit3D[] Buffer;
    public int Capacity, Count;

    public bool Visit(in DeformElem3 e)
    {
        if (Mode == Ray) { return VisitRay(in e); }
        if (Mode == Cast) { return VisitCast(in e); }
        return VisitDistance(in e);
    }

    private bool VisitRay(in DeformElem3 e)
    {
        var shape = DeformableScan3D.Shape(in e, Origin, QuaternionF.Identity, out var pose);
        if (!Gjk3D.RayCast(shape, pose, default, Direction, Length, out var h)) { return false; }
        var local = h.PointB;
        var point = new Vector3D(Origin.X + local.X, Origin.Y + local.Y, Origin.Z + local.Z);
        return Add(DeformableScan3D.Make(in e, point, h.Normal, Length > 0f ? h.Fraction / Length : 0f, h.Fraction));
    }

    private bool VisitCast(in DeformElem3 e)
    {
        var shape = DeformableScan3D.Shape(in e, Origin, Inverse, out var pose);
        var motion = Inverse.Rotate(new Vector3F(Direction.X * Length, Direction.Y * Length, Direction.Z * Length));
        if (!Gjk3D.ShapeCast(Shape, shape, pose, motion, out var h)) { return false; }
        var local = Rotation.Rotate(h.PointB);
        var point = new Vector3D(Origin.X + local.X, Origin.Y + local.Y, Origin.Z + local.Z);
        return Add(DeformableScan3D.Make(in e, point, Rotation.Rotate(h.Normal), h.Fraction, h.Fraction * Length));
    }

    private bool VisitDistance(in DeformElem3 e)
    {
        var shape = DeformableScan3D.Shape(in e, Origin, Inverse, out var pose);
        var d = Gjk3D.Distance(Shape, shape, pose);
        bool overlap = d.Overlapping || d.Distance <= 0f;
        if (Mode == CountOverlap) { if (overlap) { Total++; } return false; }
        if (Mode == Overlap)
        {
            if (!overlap) { return false; }
            var lp = Rotation.Rotate(d.PointB);
            return Add(DeformableScan3D.Make(in e, new Vector3D(Origin.X + lp.X, Origin.Y + lp.Y, Origin.Z + lp.Z), default, 0f, 0f));
        }

        float dist = overlap ? 0f : d.Distance;
        if (!(dist <= Length)) { return false; }
        var cp = Rotation.Rotate(d.PointB);
        var n = Rotation.Rotate(-d.Normal);
        return Add(DeformableScan3D.Make(in e, new Vector3D(Origin.X + cp.X, Origin.Y + cp.Y, Origin.Z + cp.Z), overlap ? default : n, 0f, dist));
    }

    // Keeps the best Capacity hits in order; returns true (stop) for an "any" query as soon as there is one.
    private bool Add(in DeformableHit3D h)
    {
        if (Any) { Buffer[0] = h; Count = 1; return true; }
        int lo = 0, hi = Count;
        while (lo < hi)
        {
            int mid = (lo + hi) >> 1;
            if (DeformableScan3D.Before(in Buffer[mid], in h)) { lo = mid + 1; } else { hi = mid; }
        }

        if (lo >= Capacity) { return false; }
        int last = Count < Capacity ? Count : Capacity - 1;
        if (last > lo) { Array.Copy(Buffer, lo, Buffer, lo + 1, last - lo); }
        Buffer[lo] = h;
        if (Count < Capacity) { Count++; }
        return false;
    }
}

internal sealed partial class Engine3D
{
    internal RodStore? RodsOrNull => _rods;

    internal static bool IsClothLike(SoftBodyData d) => d.Def.Fabric != null || (d.Tets.Length == 0 && d.PressureRest <= 0f);

    internal static DeformableKind SoftKindOf(SoftBodyData d) => IsClothLike(d) ? DeformableKind.Cloth : DeformableKind.SoftBody;

    [ThreadStatic]
    private static DeformableHit3D[]? t_deformHits;

    /// <summary>A thread's scratch buffer of at least <paramref name="capacity"/> hits.</summary>
    internal static DeformableHit3D[] DeformScratch(int capacity)
    {
        var b = t_deformHits;
        if (b == null || b.Length < capacity) { b = t_deformHits = new DeformableHit3D[Math.Max(capacity, 16)]; }
        return b;
    }
}
