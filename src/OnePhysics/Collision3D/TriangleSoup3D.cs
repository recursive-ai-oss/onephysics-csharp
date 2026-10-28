using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OneMath.Physics;

namespace RecursiveAI.OnePhysics.Collision3D;

/// <summary>One triangle of a <see cref="TriangleSoup3D"/>, with its stable id and which of its edges are active.</summary>
public readonly struct SoupTriangle3D
{
    internal SoupTriangle3D(int id, Vector3F v0, Vector3F v1, Vector3F v2, byte activeEdges)
    {
        Id = id;
        V0 = v0;
        V1 = v1;
        V2 = v2;
        ActiveEdges = activeEdges;
    }

    /// <summary>The triangle's id: its index among the kept triangles of a mesh, or <c>2 * cell + k</c> in a height field. Stable for the lifetime of the shape.</summary>
    public int Id { get; }

    /// <summary>First vertex, in the shape's frame.</summary>
    public Vector3F V0 { get; }

    /// <summary>Second vertex.</summary>
    public Vector3F V1 { get; }

    /// <summary>Third vertex.</summary>
    public Vector3F V2 { get; }

    /// <summary>Bit <c>e</c> is set when edge <c>e</c> is active: bit 0 is V0-V1, bit 1 is V1-V2, bit 2 is V2-V0. Boundary and non-manifold edges are always active.</summary>
    public byte ActiveEdges { get; }

    /// <summary>True when the given edge (0, 1 or 2) is active.</summary>
    /// <param name="edge">The edge index.</param>
    public bool IsEdgeActive(int edge) => (uint)edge <= 2u && ((ActiveEdges >> edge) & 1) != 0;

    /// <summary>The unit normal of the front side, by the winding V0, V1, V2. Computed in double and rounded once.</summary>
    public Vector3F Normal
    {
        get
        {
            Vector3D n = Vector3D.Cross(V1.ToDouble() - V0.ToDouble(), V2.ToDouble() - V0.ToDouble());
            double len = Math.Sqrt(n.LengthSquared());
            return len > 0.0 ? new Vector3F((float)(n.X / len), (float)(n.Y / len), (float)(n.Z / len)) : default;
        }
    }

    /// <summary>The triangle as a flat convex shape for GJK. Allocates; use <see cref="TriangleShapeBuffer3D"/> in loops.</summary>
    public ConvexShape3D ToShape() => new TriangleShapeBuffer3D().Set(V0, V1, V2);
}

/// <summary>A reusable flat convex shape standing in for one triangle at a time, so loops over candidate triangles allocate nothing. Not thread-safe; each <see cref="Set"/> invalidates shapes returned earlier.</summary>
public sealed class TriangleShapeBuffer3D
{
    private static readonly int[] s_vertexEdges = { 0, 2, 4 };
    private readonly Vector3F[] _vertices = new Vector3F[3];
    private readonly HullFace3D[] _faces = new HullFace3D[2];
    private readonly ConvexShape3D _shape;

    /// <summary>Creates the buffer holding a degenerate triangle at the origin.</summary>
    public TriangleShapeBuffer3D()
    {
        // A two-sided triangle as a hull with no volume: GJK and the casts only use the vertices (support mapping); faces carry the plane for code that reads them.
        // Half-edge twins sit at e ^ 1: edges 0,2,4 walk the front face, 1,3,5 the back face.
        var edges = new HullEdge3D[]
        {
            new(0, 2, 0), new(1, 5, 1), new(1, 4, 0), new(2, 1, 1), new(2, 0, 0), new(0, 3, 1),
        };
        _faces[0] = new HullFace3D(default, 0f, 0, 3);
        _faces[1] = new HullFace3D(default, 0f, 1, 3);
        _shape = ConvexShape3D.FromHull(new ConvexHull3D(_vertices, _faces, edges, s_vertexEdges, default), 0f);
    }

    /// <summary>Loads a triangle and returns the shape; the vertices are used as given, so the shape lives in whatever frame they are expressed in.</summary>
    /// <param name="a">First vertex.</param>
    /// <param name="b">Second vertex.</param>
    /// <param name="c">Third vertex.</param>
    public ConvexShape3D Set(Vector3F a, Vector3F b, Vector3F c)
    {
        _vertices[0] = a;
        _vertices[1] = b;
        _vertices[2] = c;
        Vector3D n = Vector3D.Cross(b.ToDouble() - a.ToDouble(), c.ToDouble() - a.ToDouble());
        double len = Math.Sqrt(n.LengthSquared());
        Vector3F nf = len > 0.0 ? new Vector3F((float)(n.X / len), (float)(n.Y / len), (float)(n.Z / len)) : default;
        float off = Vector3F.Dot(nf, a);
        _faces[0] = new HullFace3D(nf, off, 0, 3);
        _faces[1] = new HullFace3D(-nf, -off, 1, 3);
        return _shape;
    }
}

/// <summary>A ray hit on a triangle soup.</summary>
public readonly struct TriangleRayHit3D
{
    internal TriangleRayHit3D(int id, float fraction, Vector3F point, Vector3F normal, bool backFace)
    {
        TriangleId = id;
        Fraction = fraction;
        Point = point;
        Normal = normal;
        BackFace = backFace;
    }

    /// <summary>The id of the triangle that was hit.</summary>
    public int TriangleId { get; }

    /// <summary>Distance along the ray in multiples of the direction vector, in [0, maxFraction].</summary>
    public float Fraction { get; }

    /// <summary>The hit point in the shape's frame.</summary>
    public Vector3F Point { get; }

    /// <summary>The unit normal of the triangle's front side (by winding), whichever side was hit.</summary>
    public Vector3F Normal { get; }

    /// <summary>True when the ray reached the triangle from behind (travelling along the normal).</summary>
    public bool BackFace { get; }
}

/// <summary>The first contact of a convex shape swept through a triangle soup.</summary>
public readonly struct SoupCastHit3D
{
    internal SoupCastHit3D(int id, float fraction, Vector3F normal, Vector3F point, bool startsOverlapping)
    {
        TriangleId = id;
        Fraction = fraction;
        Normal = normal;
        Point = point;
        StartsOverlapping = startsOverlapping;
    }

    /// <summary>The id of the triangle touched first; the lowest id among equal fractions.</summary>
    public int TriangleId { get; }

    /// <summary>The fraction of the motion at first contact, in [0, 1].</summary>
    public float Fraction { get; }

    /// <summary>Unit direction at the contact from the triangle towards the moving shape, in the soup's frame.</summary>
    public Vector3F Normal { get; }

    /// <summary>The contact point on the triangle, in the soup's frame.</summary>
    public Vector3F Point { get; }

    /// <summary>True when the shape already touched or overlapped the triangle at fraction 0.</summary>
    public bool StartsOverlapping { get; }
}

internal readonly struct RayD
{
    public readonly double Ox, Oy, Oz, Dx, Dy, Dz, Ix, Iy, Iz;

    public RayD(Vector3F o, Vector3F d)
    {
        Ox = o.X;
        Oy = o.Y;
        Oz = o.Z;
        Dx = d.X;
        Dy = d.Y;
        Dz = d.Z;
        Ix = 1.0 / Dx;
        Iy = 1.0 / Dy;
        Iz = 1.0 / Dz;
    }

    public Vector3D Origin => new(Ox, Oy, Oz);

    public Vector3D Direction => new(Dx, Dy, Dz);

    // Slab test against a box padded by about 10^-8 of the scale of the ray origin and the box (not only of the box's own coordinates, since a flat box lies at height 0 or thereabouts),
    // so a hit on a triangle inside the box is never lost to rounding in the slab arithmetic.
    // Returns the entry fraction (at least 0), or false when the ray misses within maxT.
    public bool HitBox(float minX, float minY, float minZ, float maxX, float maxY, float maxZ, double maxT, out double enter)
    {
        double t0 = 0.0, t1 = maxT;
        bool hit = Axis(Ox, Dx, Ix, minX, maxX, ref t0, ref t1) && Axis(Oy, Dy, Iy, minY, maxY, ref t0, ref t1) && Axis(Oz, Dz, Iz, minZ, maxZ, ref t0, ref t1);
        enter = t0;
        return hit;
    }

    private static bool Axis(double o, double d, double inv, float lo, float hi, ref double t0, ref double t1)
    {
        double pad = 1.5e-8 * (Math.Abs(o) + Math.Abs((double)lo) + Math.Abs((double)hi) + ((double)hi - lo));
        double l = lo - pad, h = hi + pad;
        if (d == 0.0 || !double.IsFinite(inv))
        {
            return o >= l && o <= h;
        }

        double a = (l - o) * inv, b = (h - o) * inv;
        if (a > b)
        {
            (a, b) = (b, a);
        }

        if (a > t0)
        {
            t0 = a;
        }

        if (b < t1)
        {
            t1 = b;
        }

        return t0 <= t1;
    }
}

internal struct RayCollector
{
    public int Mode; // 0 closest, 1 any, 2 all
    public double Limit;
    public bool CullBack;
    public bool Found;
    public int BestId;
    public double BestT;
    public bool BestBack;
    public List<(int Id, double T, bool Back)>? All;

    public readonly bool Done => Mode == 1 && Found;

    public void Offer(int id, double t, bool back)
    {
        if (t > Limit)
        {
            return;
        }

        switch (Mode)
        {
            case 0:
                if (!Found || t < BestT || (t == BestT && id < BestId))
                {
                    Found = true;
                    BestId = id;
                    BestT = t;
                    BestBack = back;
                    Limit = t;
                }

                break;
            case 1:
                if (!Found)
                {
                    Found = true;
                    BestId = id;
                    BestT = t;
                    BestBack = back;
                }

                break;
            default:
                All!.Add((id, t, back));
                Found = true;
                break;
        }
    }
}

/// <summary>
/// Common surface of the static, non-convex shapes made of triangles: <see cref="TriangleMesh3D"/> and <see cref="HeightField3D"/>. Both live in their own local frame, in float32, are immutable and
/// can only be static: they have no mass properties. Queries are deterministic, with stated tie rules, and agree exactly with a brute-force loop over <see cref="TryGetTriangle"/> and <see cref="TriangleTests3D"/>.
/// </summary>
public abstract class TriangleSoup3D
{
    internal TriangleSoup3D()
    {
    }

    /// <summary>The exclusive upper bound of triangle ids. A height field's ids have gaps where cells are holes.</summary>
    public abstract int TriangleIdCount { get; }

    /// <summary>The number of triangles that exist (ids that <see cref="TryGetTriangle"/> accepts).</summary>
    public abstract int TriangleCount { get; }

    /// <summary>The bounds of all triangles, in the shape's frame.</summary>
    public abstract Bounds3F Bounds { get; }

    /// <summary>Always true: triangle meshes and height fields have no mass and cannot be simulated as moving bodies.</summary>
    public virtual bool IsStaticOnly => true;

    /// <summary>Meshes and height fields are static-only.</summary>
    /// <exception cref="StaticOnlyShapeException">Always.</exception>
    public virtual MassPropertiesF GetMassProperties() =>
        throw new StaticOnlyShapeException("A triangle mesh or height field has no mass properties: it is static-only. Attach it to a static body, or build a convex hull or compound for a moving one.");

    /// <summary>Looks up a triangle by id.</summary>
    /// <param name="id">A triangle id in <c>[0, TriangleIdCount)</c>.</param>
    /// <param name="triangle">The triangle; default when false is returned.</param>
    /// <returns>False when the id is out of range or names a hole.</returns>
    public abstract bool TryGetTriangle(int id, out SoupTriangle3D triangle);

    internal abstract void RayWalk(in RayD ray, ref RayCollector collector);

    internal abstract void BoxWalk(in Bounds3F box, List<int> ids);

    private static void CheckRay(Vector3F origin, Vector3F direction, float maxFraction)
    {
        CheckFinite(origin, nameof(origin));
        CheckFinite(direction, nameof(direction));
        if (!(maxFraction >= 0f) || !float.IsFinite(maxFraction))
        {
            throw new ArgumentOutOfRangeException(nameof(maxFraction), maxFraction, $"maxFraction is {maxFraction}; a finite value of at least 0 is expected.");
        }
    }

    private static void CheckFinite(Vector3F v, string name)
    {
        if (!float.IsFinite(v.X) || !float.IsFinite(v.Y) || !float.IsFinite(v.Z))
        {
            throw new ArgumentOutOfRangeException(name, v.ToString(), $"{name} is {v}; finite components are expected.");
        }
    }

    private TriangleRayHit3D MakeHit(in RayD ray, int id, double t, bool back)
    {
        TryGetTriangle(id, out SoupTriangle3D tri);
        Vector3D p = ray.Origin + (ray.Direction * t);
        return new TriangleRayHit3D(id, (float)t, new Vector3F((float)p.X, (float)p.Y, (float)p.Z), tri.Normal, back);
    }

    /// <summary>Casts a ray and returns the closest hit: the smallest fraction, the lowest triangle id among equal fractions.</summary>
    /// <param name="origin">Ray origin in the shape's frame; finite.</param>
    /// <param name="direction">Ray direction; finite, any length. A zero direction never hits.</param>
    /// <param name="maxFraction">The ray covers fractions 0 to this value of the direction vector; finite, non-negative.</param>
    /// <param name="hit">The closest hit; default when false is returned.</param>
    /// <param name="cullBackFaces">When true, triangles seen from behind are ignored.</param>
    /// <returns>True when a triangle was hit.</returns>
    /// <exception cref="ArgumentOutOfRangeException">An argument is not finite, or <paramref name="maxFraction"/> is negative.</exception>
    public bool RayCast(Vector3F origin, Vector3F direction, float maxFraction, out TriangleRayHit3D hit, bool cullBackFaces = false)
    {
        CheckRay(origin, direction, maxFraction);
        hit = default;
        if (direction == default)
        {
            return false;
        }

        var ray = new RayD(origin, direction);
        var c = new RayCollector { Mode = 0, Limit = maxFraction, CullBack = cullBackFaces };
        RayWalk(ray, ref c);
        if (c.Found)
        {
            hit = MakeHit(ray, c.BestId, c.BestT, c.BestBack);
        }

        return c.Found;
    }

    /// <summary>Casts a ray and returns whether anything is hit, with some hit (the first one the traversal finds, which is deterministic but not the closest).</summary>
    /// <param name="origin">Ray origin in the shape's frame; finite.</param>
    /// <param name="direction">Ray direction; finite.</param>
    /// <param name="maxFraction">Largest fraction; finite, non-negative.</param>
    /// <param name="hit">Some hit; default when false is returned.</param>
    /// <param name="cullBackFaces">When true, triangles seen from behind are ignored.</param>
    /// <returns>True when a triangle was hit.</returns>
    /// <exception cref="ArgumentOutOfRangeException">An argument is not finite, or <paramref name="maxFraction"/> is negative.</exception>
    public bool RayCastAny(Vector3F origin, Vector3F direction, float maxFraction, out TriangleRayHit3D hit, bool cullBackFaces = false)
    {
        CheckRay(origin, direction, maxFraction);
        hit = default;
        if (direction == default)
        {
            return false;
        }

        var ray = new RayD(origin, direction);
        var c = new RayCollector { Mode = 1, Limit = maxFraction, CullBack = cullBackFaces };
        RayWalk(ray, ref c);
        if (c.Found)
        {
            hit = MakeHit(ray, c.BestId, c.BestT, c.BestBack);
        }

        return c.Found;
    }

    /// <summary>Appends every triangle the ray crosses to <paramref name="hits"/>, ordered by fraction and then triangle id.</summary>
    /// <param name="origin">Ray origin in the shape's frame; finite.</param>
    /// <param name="direction">Ray direction; finite.</param>
    /// <param name="maxFraction">Largest fraction; finite, non-negative.</param>
    /// <param name="hits">Receives the hits; existing entries are kept and not reordered.</param>
    /// <param name="cullBackFaces">When true, triangles seen from behind are ignored.</param>
    /// <returns>The number of hits appended.</returns>
    /// <exception cref="ArgumentOutOfRangeException">An argument is not finite, or <paramref name="maxFraction"/> is negative.</exception>
    public int RayCastAll(Vector3F origin, Vector3F direction, float maxFraction, List<TriangleRayHit3D> hits, bool cullBackFaces = false)
    {
        ArgumentNullException.ThrowIfNull(hits);
        CheckRay(origin, direction, maxFraction);
        if (direction == default)
        {
            return 0;
        }

        var ray = new RayD(origin, direction);
        var found = new List<(int Id, double T, bool Back)>();
        var c = new RayCollector { Mode = 2, Limit = maxFraction, CullBack = cullBackFaces, All = found };
        RayWalk(ray, ref c);
        found.Sort(static (x, y) => x.T != y.T ? x.T.CompareTo(y.T) : x.Id.CompareTo(y.Id));
        foreach (var (id, t, back) in found)
        {
            hits.Add(MakeHit(ray, id, t, back));
        }

        return found.Count;
    }

    /// <summary>Appends the ids of every triangle that touches the box (<see cref="TriangleTests3D.BoxTriangle"/>) to <paramref name="ids"/>, in ascending order.</summary>
    /// <param name="box">The query box in the shape's frame; finite and not empty.</param>
    /// <param name="ids">Receives the ids; existing entries are kept.</param>
    /// <returns>The number of ids appended.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The box has a non-finite bound or is empty.</exception>
    public int Overlap(Bounds3F box, List<int> ids)
    {
        ArgumentNullException.ThrowIfNull(ids);
        CheckBox(box);
        int before = ids.Count;
        BoxWalk(box, ids);
        ids.Sort(before, ids.Count - before, null);
        return ids.Count - before;
    }

    private static void CheckBox(Bounds3F box)
    {
        CheckFinite(box.Min, "box.Min");
        CheckFinite(box.Max, "box.Max");
        if (box.IsEmpty)
        {
            throw new ArgumentOutOfRangeException(nameof(box), box.ToString(), $"The box {box} is empty (a minimum exceeds its maximum); Min <= Max on every axis is expected.");
        }
    }

    // The box of a convex shape placed in the soup's frame, from six support points.
    private static Bounds3F ShapeBounds(in ConvexShape3D shape, in RigidTransformF placement)
    {
        QuaternionF inv = placement.Rotation.Conjugate();
        Vector3F px = placement.TransformPoint(shape.Support(inv.Rotate(Vector3F.UnitX)));
        Vector3F nx = placement.TransformPoint(shape.Support(inv.Rotate(-Vector3F.UnitX)));
        Vector3F py = placement.TransformPoint(shape.Support(inv.Rotate(Vector3F.UnitY)));
        Vector3F ny = placement.TransformPoint(shape.Support(inv.Rotate(-Vector3F.UnitY)));
        Vector3F pz = placement.TransformPoint(shape.Support(inv.Rotate(Vector3F.UnitZ)));
        Vector3F nz = placement.TransformPoint(shape.Support(inv.Rotate(-Vector3F.UnitZ)));
        return new Bounds3F(new Vector3F(nx.X, ny.Y, nz.Z), new Vector3F(px.X, py.Y, pz.Z));
    }

    private static void CheckPlacement(in RigidTransformF t)
    {
        CheckFinite(t.Translation, "shapeInSoup.Translation");
        QuaternionF q = t.Rotation;
        float l = q.LengthSquared();
        if (!float.IsFinite(l) || Math.Abs(l - 1f) > 1e-3f)
        {
            throw new ArgumentOutOfRangeException(nameof(t), q.ToString(), $"The rotation {q} has squared length {l}; a unit quaternion is expected.");
        }
    }

    private static Bounds3F Grow(Bounds3F b)
    {
        float m = Math.Max(Math.Max(Math.Abs(b.Min.X), Math.Abs(b.Max.X)), Math.Max(Math.Max(Math.Abs(b.Min.Y), Math.Abs(b.Max.Y)), Math.Max(Math.Abs(b.Min.Z), Math.Abs(b.Max.Z))));
        return b.Inflate(1e-5f * (m + 1e-3f));
    }

    /// <summary>
    /// Enumerates the triangles that may touch a convex shape: every triangle that touches the shape's bounding box, grown by <paramref name="margin"/>, in ascending id order. This is the input of
    /// a convex-versus-soup narrowphase; each triangle carries its active-edge flags and can become a convex shape with <see cref="SoupTriangle3D.ToShape"/>.
    /// </summary>
    /// <param name="shape">The convex shape.</param>
    /// <param name="shapeInSoup">Placement of the shape's frame in the soup's frame; a unit quaternion and a finite translation.</param>
    /// <param name="margin">Extra distance to grow the shape's bounds by, for speculative contacts; finite, non-negative.</param>
    /// <param name="triangles">Receives the candidates; existing entries are kept.</param>
    /// <param name="scratch">Optional reusable id list, to avoid an allocation.</param>
    /// <returns>The number of triangles appended.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The placement is not a rigid transform or the margin is invalid.</exception>
    public int GetCandidates(in ConvexShape3D shape, in RigidTransformF shapeInSoup, float margin, List<SoupTriangle3D> triangles, List<int>? scratch = null)
    {
        ArgumentNullException.ThrowIfNull(triangles);
        CheckPlacement(shapeInSoup);
        if (!(margin >= 0f) || !float.IsFinite(margin))
        {
            throw new ArgumentOutOfRangeException(nameof(margin), margin, $"margin is {margin}; a finite value of at least 0 is expected.");
        }

        Bounds3F box = ShapeBounds(shape, shapeInSoup).Inflate(margin);
        List<int> ids = scratch ?? new List<int>();
        ids.Clear();
        Overlap(box, ids);
        foreach (int id in ids)
        {
            TryGetTriangle(id, out SoupTriangle3D t);
            triangles.Add(t);
        }

        return ids.Count;
    }

    /// <summary>
    /// Sweeps a convex shape through the soup and returns the first contact, by running the convex shape cast (<see cref="Gjk3D.ShapeCast"/>) against each triangle whose bounds meet the swept box.
    /// The earliest fraction wins; equal fractions go to the lowest triangle id. The triangles are one-sided in no way: a shape touches either face.
    /// A hit is within the cast's tolerance (fraction error below 2e-4 for a motion of length 8) of the exact time of impact, and the swept-box filter is grown by 1e-5 of the scene's size so a touching triangle is never filtered out.
    /// </summary>
    /// <param name="shape">The moving convex shape.</param>
    /// <param name="shapeInSoup">Start placement of the shape's frame in the soup's frame; a unit quaternion and a finite translation.</param>
    /// <param name="motion">How far the shape moves, in the soup's frame; finite.</param>
    /// <param name="hit">The first contact; default when false is returned.</param>
    /// <param name="scratch">Optional reusable id list, to avoid an allocation.</param>
    /// <returns>True when the shape touches a triangle within the motion.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The placement or the motion is invalid.</exception>
    public bool ShapeCast(in ConvexShape3D shape, in RigidTransformF shapeInSoup, Vector3F motion, out SoupCastHit3D hit, List<int>? scratch = null)
    {
        CheckPlacement(shapeInSoup);
        CheckFinite(motion, nameof(motion));
        hit = default;
        Bounds3F start = ShapeBounds(shape, shapeInSoup);
        Bounds3F sweep = Grow(Bounds3F.Union(start, new Bounds3F(start.Min + motion, start.Max + motion)));
        List<int> ids = scratch ?? new List<int>();
        ids.Clear();
        Overlap(sweep, ids);
        return CastTriangles(shape, shapeInSoup, motion, ids, out hit);
    }

    [ThreadStatic]
    private static TriangleShapeBuffer3D? s_castBuffer;

    /// <summary>The cast over an explicit list of triangle ids, ascending: the filtered path of <see cref="ShapeCast"/>, and with every id the brute-force reference.</summary>
    /// <param name="shape">The moving convex shape.</param>
    /// <param name="shapeInSoup">Start placement of the shape in the soup's frame.</param>
    /// <param name="motion">The motion in the soup's frame.</param>
    /// <param name="ids">The triangle ids to test, in ascending order; ids that name no triangle are skipped.</param>
    /// <param name="hit">The first contact; default when false is returned.</param>
    /// <returns>True when the shape touches one of the triangles within the motion.</returns>
    public bool CastTriangles(in ConvexShape3D shape, in RigidTransformF shapeInSoup, Vector3F motion, IReadOnlyList<int> ids, out SoupCastHit3D hit)
    {
        ArgumentNullException.ThrowIfNull(ids);
        CheckPlacement(shapeInSoup);
        CheckFinite(motion, nameof(motion));
        hit = default;
        QuaternionF inv = shapeInSoup.Rotation.Conjugate();
        Vector3F motionA = inv.Rotate(motion);
        // One scratch triangle per thread, so repeated casts allocate nothing.
        var buffer = s_castBuffer ??= new TriangleShapeBuffer3D();
        bool found = false;
        float bestFraction = 0f;
        for (int k = 0; k < ids.Count; k++)
        {
            if (!TryGetTriangle(ids[k], out SoupTriangle3D t))
            {
                continue;
            }

            // The shape stays at its own origin; the triangle is brought into its frame, so the pair transform is the identity.
            ConvexShape3D tri = buffer.Set(
                inv.Rotate(t.V0 - shapeInSoup.Translation),
                inv.Rotate(t.V1 - shapeInSoup.Translation),
                inv.Rotate(t.V2 - shapeInSoup.Translation));
            if (Gjk3D.ShapeCast(shape, tri, RigidTransformF.Identity, motionA, out ConvexCastHit3D h) && (!found || h.Fraction < bestFraction))
            {
                found = true;
                bestFraction = h.Fraction;
                hit = new SoupCastHit3D(t.Id, h.Fraction, shapeInSoup.Rotation.Rotate(-h.Normal), shapeInSoup.TransformPoint(h.PointB), h.StartsOverlapping);
            }
        }

        return found;
    }
}
