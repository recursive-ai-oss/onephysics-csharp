using System;
using RecursiveAI.OneMath;
using System.Diagnostics.CodeAnalysis;

namespace RecursiveAI.OnePhysics.LowLevel;

// Feature codes: 16 bits, kind in the top two bits, index in the low fourteen. Two codes (shape A's high, shape B's low) make a manifold point id.
internal static class FCode
{
    public const ushort EdgeKind = 0x4000, VertexKind = 0x8000, DerivedKind = 0xC000;

    public static ushort Face(int i) => (ushort)(i & 0x3FFF);

    public static ushort Edge(int i) => (ushort)(EdgeKind | (i & 0x3FFF));

    public static ushort Vertex(int i) => (ushort)(VertexKind | (i & 0x3FFF));

    public static ushort Derived(int i) => (ushort)(DerivedKind | (i & 0x3FFF));

    public static uint Pack(ushort a, ushort b) => ((uint)a << 16) | b;
}

/// <summary>
/// Builds contact manifolds for pairs of convex shapes (<see cref="ConvexShape3D"/>): up to four points with a normal, separations and stable feature ids.
/// Sphere and capsule pairs and sphere against box or hull are solved analytically; box and hull pairs use the separating axis test with Gauss-map edge pruning
/// (Gregorius, GDC 2013) and reference/incident face clipping (Ericson, <i>Real-Time Collision Detection</i>, 8.x and 5.x); every other pair, and the corner cases the axis test
/// cannot see, use GJK/EPA for the normal and clip the supporting features. More than four points are reduced as in Gregorius, GDC 2015.
/// An instance owns its working storage and reuses it, so queries do not allocate once warm; it is not safe to share between threads.
/// </summary>
public sealed partial class ManifoldGenerator3D
{
    /// <summary>Default speculative distance: surfaces closer than this get a contact point before they touch.</summary>
    public const float DefaultSpeculativeDistance = 0.02f;

    /// <summary>Largest translation component accepted in the pose of B relative to A, 2^22.</summary>
    public const float MaxOffset = 4194304f;

    private readonly Epa3D _epa = new();
    private readonly Feat _fa = new();
    private readonly Feat _fb = new();
    private OutPt[] _pts = new OutPt[16];
    private int _count;
    private readonly int[] _sel = new int[4];

    /// <summary>
    /// Temporal coherence for the polyhedron separating-axis test (Gregorius, "Robust Contact Creation for Physics Simulations", GDC 2015, on caching the separating feature between frames;
    /// Catto, "Contact Manifolds", GDC 2007 and "Solver2D" on persistent contacts). The caller passes what the last full query found about the pair in <see cref="HintIn"/> and reads what to keep in
    /// <see cref="HintOut"/>: the margin by which the face axes beat every edge-pair axis and the pose of B in A at the last full query. While the pose has moved so little since then that no edge pair can
    /// have closed that margin, the edge-pair search is skipped.
    /// </summary>
    internal EdgeHint3D HintIn, HintOut;
    private EdgeHint3D _edgeHint;

    /// <summary>
    /// Set by the convex Collide query: when the manifold is empty because the shapes are farther apart than the speculative distance and the query could say by how much, a lower bound on how much farther
    /// (beyond the speculative distance, shape surfaces included); otherwise zero. A caller that knows the shapes' relative motion can skip the next queries until that motion has used the bound up.
    /// </summary>
    internal float ClearanceOut;

    /// <summary>
    /// Builds the manifold of two shapes. Everything is in the frame of A; the normal points from A to B and separations are surface distances (negative when overlapping).
    /// </summary>
    /// <param name="a">Shape A, in its own frame.</param>
    /// <param name="b">Shape B, in its own frame.</param>
    /// <param name="bInA">Placement of B's frame in A's frame, computed by the caller from double positions; a unit rotation and a translation within <see cref="MaxOffset"/>.</param>
    /// <param name="speculativeDistance">Points whose surfaces are farther apart than this are dropped; finite and non-negative.</param>
    /// <returns>The manifold; empty when the surfaces are farther apart than <paramref name="speculativeDistance"/>.</returns>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="speculativeDistance"/> is negative or not finite, <paramref name="bInA"/> has a non-unit rotation, or a translation component is NaN, infinite or beyond <see cref="MaxOffset"/>.</exception>
    public Manifold3D Collide(in ConvexShape3D a, in ConvexShape3D b, in RigidTransformF bInA, float speculativeDistance = DefaultSpeculativeDistance)
    {
        if (!(speculativeDistance >= 0f) || !float.IsFinite(speculativeDistance))
        {
            throw new ArgumentOutOfRangeException(nameof(speculativeDistance), speculativeDistance, $"speculativeDistance is {speculativeDistance}; a finite value of at least 0 is expected.");
        }

        QuaternionF q = bInA.Rotation;
        Vector3F t = bInA.Translation;
        float ql = (q.X * q.X) + (q.Y * q.Y) + (q.Z * q.Z) + (q.W * q.W);
        if (!(ql >= 0.99f && ql <= 1.01f) || !(Math.Abs(t.X) <= MaxOffset && Math.Abs(t.Y) <= MaxOffset && Math.Abs(t.Z) <= MaxOffset))
        {
            throw new ArgumentOutOfRangeException(nameof(bInA), bInA, $"bInA is {bInA}; a unit rotation (squared length within 1% of 1) and a translation with each component at most {MaxOffset} in magnitude are expected.");
        }

        _count = 0;
        _edgeHint = HintIn; HintIn = default; // a hint applies to one call only, so a leftover cannot reach another pair that happens to run on this thread next
        ClearanceOut = 0f;
        HintOut = default; // nothing to keep unless the polyhedron test sets it; a value left by the previous pair would depend on which thread ran it
        ConvexShapeKind3D ka = a.Kind, kb = b.Kind;
        bool segA = IsSegmentLike(ka), segB = IsSegmentLike(kb);
        bool polyA = IsPoly(ka), polyB = IsPoly(kb);
        if (segA && segB)
        {
            return SegmentSegment(a, b, bInA, speculativeDistance);
        }

        if (ka == ConvexShapeKind3D.Sphere && polyB)
        {
            return SpherePoly(a, b, true, bInA, speculativeDistance);
        }

        if (kb == ConvexShapeKind3D.Sphere && polyA)
        {
            return SpherePoly(b, a, false, bInA, speculativeDistance);
        }

        if (polyA && polyB)
        {
            // Bounding spheres around the shape origins: when they are farther apart than the speculative distance, so are the shapes. Most broadphase pairs in a pile end here.
            float reach = Extent(a) + Extent(b) + speculativeDistance;
            float t2 = t.LengthSquared();
            if (t2 > reach * reach)
            {
                ClearanceOut = ConvexShape3D.Sqrt(t2) - reach;
                return Empty;
            }

            return PolyPoly(a, b, bInA, speculativeDistance);
        }

        if (((ka == ConvexShapeKind3D.Capsule && polyB) || (kb == ConvexShapeKind3D.Capsule && polyA)) && CapsulePolyApart(a, b, kb == ConvexShapeKind3D.Capsule, bInA, speculativeDistance))
        {
            return Empty;
        }

        ConvexContact3D c;
        bool capBox = (ka == ConvexShapeKind3D.Capsule && kb == ConvexShapeKind3D.Box) || (ka == ConvexShapeKind3D.Box && kb == ConvexShapeKind3D.Capsule);
        if (!capBox || GeneralCapsuleBox || !CapsuleBoxContact(a, b, kb == ConvexShapeKind3D.Capsule, bInA, out c))
        {
            c = _epa.Collide(a, b, bInA);
        }

        return Generic(a, b, bInA, c, speculativeDistance);
    }

    /// <summary>Builds a manifold with a new generator; allocates. Prefer reusing a <see cref="ManifoldGenerator3D"/> instance.</summary>
    public static Manifold3D CollideOnce(in ConvexShape3D a, in ConvexShape3D b, in RigidTransformF bInA, float speculativeDistance = DefaultSpeculativeDistance) =>
        new ManifoldGenerator3D().Collide(a, b, bInA, speculativeDistance);

    private static bool IsSegmentLike(ConvexShapeKind3D k) => k == ConvexShapeKind3D.Sphere || k == ConvexShapeKind3D.Capsule;

    private static bool IsPoly(ConvexShapeKind3D k) => k == ConvexShapeKind3D.Box || k == ConvexShapeKind3D.Hull;

    private struct OutPt
    {
        public Vector3F P;
        public float Sep;
        public uint Id;
    }

    private void Push(Vector3F p, float sep, uint id)
    {
        if (_count == _pts.Length)
        {
            Array.Resize(ref _pts, _count * 2);
        }

        _pts[_count++] = new OutPt { P = p, Sep = sep, Id = id };
    }

    private static Manifold3D Empty => default;

    // Reduces the collected points to at most four (Gregorius, GDC 2015, "Robust Contact Creation for Physics Simulations"): the deepest point, the point farthest from it,
    // the point that makes the largest triangle, and the point that adds the most area on the other side of that triangle. Ties go to the lowest index.
    // The survivors are ordered by feature id so a point keeps its slot while its feature stays in contact.
    private Manifold3D Finish(Vector3F normal)
    {
        int k = _count;
        if (k == 0)
        {
            return Empty;
        }

        int[] sel = _sel;
        int n = 0;
        if (k <= 4)
        {
            for (int i = 0; i < k; i++)
            {
                sel[n++] = i;
            }
        }
        else
        {
            int i0 = 0;
            for (int i = 1; i < k; i++)
            {
                if (_pts[i].Sep < _pts[i0].Sep)
                {
                    i0 = i;
                }
            }

            int i1 = -1;
            float best = -1f;
            for (int i = 0; i < k; i++)
            {
                if (i == i0)
                {
                    continue;
                }

                float d = (_pts[i].P - _pts[i0].P).LengthSquared();
                if (d > best)
                {
                    best = d;
                    i1 = i;
                }
            }

            int i2 = -1;
            best = -1f;
            Vector3F e01 = _pts[i1].P - _pts[i0].P;
            for (int i = 0; i < k; i++)
            {
                if (i == i0 || i == i1)
                {
                    continue;
                }

                float area = Math.Abs(Vector3F.Dot(normal, Vector3F.Cross(e01, _pts[i].P - _pts[i0].P)));
                if (area > best)
                {
                    best = area;
                    i2 = i;
                }
            }

            // Orient the triangle counter-clockwise about the normal, then the fourth point is the one farthest outside any of its edges.
            float orient = Vector3F.Dot(normal, Vector3F.Cross(e01, _pts[i2].P - _pts[i0].P));
            int ta = i0, tb = i1, tc = i2;
            if (orient < 0f)
            {
                (tb, tc) = (tc, tb);
            }

            int i3 = -1;
            best = 0f;
            for (int i = 0; i < k; i++)
            {
                if (i == i0 || i == i1 || i == i2)
                {
                    continue;
                }

                float o = Math.Max(OutsideArea(normal, ta, tb, i), Math.Max(OutsideArea(normal, tb, tc, i), OutsideArea(normal, tc, ta, i)));
                if (o > best)
                {
                    best = o;
                    i3 = i;
                }
            }

            sel[n++] = i0;
            sel[n++] = i1;
            sel[n++] = i2;
            if (i3 >= 0)
            {
                sel[n++] = i3;
            }
        }

        // Insertion sort by id; equal ids keep selection order.
        for (int i = 1; i < n; i++)
        {
            int v = sel[i];
            int j = i - 1;
            while (j >= 0 && _pts[sel[j]].Id > _pts[v].Id)
            {
                sel[j + 1] = sel[j];
                j--;
            }

            sel[j + 1] = v;
        }

        ManifoldPoint3D p0 = default, p1 = default, p2 = default, p3 = default;
        for (int i = 0; i < n; i++)
        {
            OutPt o = _pts[sel[i]];
            var mp = new ManifoldPoint3D(o.P, o.Sep, o.Id);
            switch (i)
            {
                case 0: p0 = mp; break;
                case 1: p1 = mp; break;
                case 2: p2 = mp; break;
                default: p3 = mp; break;
            }
        }

        return new Manifold3D(UnitNormal(normal), n, p0, p1, p2, p3);
    }

    // The depth and direction searches (SAT axes, EPA faces) can hand back a normal that is off by more than rounding when the shapes are tiny, denormal or hugely different in size;
    // a normal that is not within 1e-3 of unit squared length is rescaled in double so every manifold leaves here with a unit normal. A zero or non-finite normal falls back to the +Y axis.
    private static Vector3F UnitNormal(Vector3F n)
    {
        double x = n.X, y = n.Y, z = n.Z;
        double l2 = (x * x) + (y * y) + (z * z);
        if (Math.Abs(l2 - 1.0) <= 1e-3)
        {
            return n;
        }

        if (!(l2 > 0.0) || double.IsInfinity(l2))
        {
            // Overflowed squares: scale by the largest component first.
            double m = Math.Max(Math.Abs(x), Math.Max(Math.Abs(y), Math.Abs(z)));
            if (!(m > 0.0) || double.IsInfinity(m) || double.IsNaN(m))
            {
                return Vector3F.UnitY;
            }

            x /= m; y /= m; z /= m;
            l2 = (x * x) + (y * y) + (z * z);
        }

        double inv = 1.0 / Math.Sqrt(l2);
        return new Vector3F((float)(x * inv), (float)(y * inv), (float)(z * inv));
    }

    // Twice the signed area of the triangle (edge a->b, point x) about the normal, positive when x lies to the right of a->b, i.e. outside a counter-clockwise triangle.
    private float OutsideArea(Vector3F normal, int a, int b, int x) =>
        -Vector3F.Dot(normal, Vector3F.Cross(_pts[b].P - _pts[a].P, _pts[x].P - _pts[a].P));

    // A single-point manifold from a GJK/EPA contact, for the rare case where feature clipping finds nothing.
    private Manifold3D FromContact(in ConvexContact3D c, ushort codeA, ushort codeB, float margin)
    {
        if (c.Separation > margin)
        {
            return Empty;
        }

        _count = 0;
        Push((c.PointA + c.PointB) * 0.5f, c.Separation, FCode.Pack(codeA, codeB));
        return Finish(c.Normal);
    }

    private static float Extent(in ConvexShape3D s)
    {
        float r = s.ConvexRadius;
        Vector3F z = s.CoreSize;
        switch (s.Kind)
        {
            case ConvexShapeKind3D.Sphere: return r;
            case ConvexShapeKind3D.Box: return ConvexShape3D.Sqrt(z.LengthSquared()) + r;
            case ConvexShapeKind3D.Capsule: return z.Y + r;
            case ConvexShapeKind3D.TaperedCapsule: return z.Z + Math.Max(z.X, z.Y);
            case ConvexShapeKind3D.Cylinder: return ConvexShape3D.Sqrt((z.X * z.X) + (z.Y * z.Y)) + r;
            default:
                {
                    float m = 0f;
                    foreach (Vector3F v in s.Hull!.Vertices)
                    {
                        m = Math.Max(m, v.LengthSquared());
                    }

                    return ConvexShape3D.Sqrt(m) + r;
                }
        }
    }
}
