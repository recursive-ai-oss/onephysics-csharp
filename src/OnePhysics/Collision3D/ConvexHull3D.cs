using System;
using RecursiveAI.OneMath;
using RecursiveAI.OneMath.Physics;

namespace RecursiveAI.OnePhysics.Collision3D;

/// <summary>Why a convex hull could not be built.</summary>
public enum HullBuildError
{
    /// <summary>Fewer than four points were given.</summary>
    TooFewPoints,

    /// <summary>More than <see cref="ConvexHull3D.MaxInputPoints"/> points were given.</summary>
    TooManyPoints,

    /// <summary>A coordinate is NaN or infinite.</summary>
    NonFinite,

    /// <summary>A coordinate's magnitude exceeds <see cref="ConvexHull3D.MaxCoordinate"/>.</summary>
    OutOfRange,

    /// <summary>The points do not span a volume: they are coincident, collinear or coplanar within the tolerance.</summary>
    Degenerate,

    /// <summary>The options are invalid (for example a vertex limit below four).</summary>
    InvalidOptions,
}

/// <summary>Thrown by <see cref="ConvexHull3D.Build"/> when the input cannot make a convex hull. The message names the input and what was expected.</summary>
public sealed class HullBuildException : ArgumentException
{
    /// <summary>Creates the exception.</summary>
    /// <param name="error">The kind of failure.</param>
    /// <param name="message">What was wrong and what was expected.</param>
    /// <param name="paramName">The offending parameter.</param>
    public HullBuildException(HullBuildError error, string message, string? paramName = null)
        : base(message, paramName)
    {
        Error = error;
    }

    /// <summary>The kind of failure.</summary>
    public HullBuildError Error { get; }
}

/// <summary>Options for <see cref="ConvexHull3D.Build"/>. The default value (all zeros) means no limits and an automatic tolerance.</summary>
public readonly struct HullBuildOptions
{
    /// <summary>The most vertices the hull may have; 0 means no limit, otherwise at least 4. When the limit binds, construction stops adding the furthest points and the hull is the hull of the points chosen so far, so some input points may lie outside it.</summary>
    public int MaxVertices { get; init; }

    /// <summary>The most faces the hull may have; 0 means no limit, otherwise at least 4. The same early stop as <see cref="MaxVertices"/> applies.</summary>
    public int MaxFaces { get; init; }

    /// <summary>The distance within which a point counts as lying on a face plane and neighbouring faces are merged into one polygon, in the points' units. 0 chooses 3 * 2^-23 times the sum of the three axis extents of the input; otherwise it must be finite and positive.</summary>
    public float Tolerance { get; init; }
}

/// <summary>A polygonal face of a <see cref="ConvexHull3D"/>.</summary>
public readonly struct HullFace3D
{
    internal HullFace3D(Vector3F normal, float offset, int firstEdge, int edgeCount)
    {
        Normal = normal;
        Offset = offset;
        FirstEdge = firstEdge;
        EdgeCount = edgeCount;
    }

    /// <summary>The outward unit normal.</summary>
    public Vector3F Normal { get; }

    /// <summary>The plane offset: points of the face satisfy <c>Dot(Normal, p) = Offset</c>; interior points give smaller values.</summary>
    public float Offset { get; }

    /// <summary>One half-edge of the face; follow <see cref="HullEdge3D.Next"/> to walk the boundary counter-clockwise seen from outside.</summary>
    public int FirstEdge { get; }

    /// <summary>The number of edges (and vertices) of the face, at least 3.</summary>
    public int EdgeCount { get; }
}

/// <summary>A half-edge of a <see cref="ConvexHull3D"/>. Its twin is index <c>e ^ 1</c>, so the two halves of an undirected edge are always adjacent.</summary>
public readonly struct HullEdge3D
{
    internal HullEdge3D(int origin, int next, int face)
    {
        Origin = origin;
        Next = next;
        Face = face;
    }

    /// <summary>The vertex the half-edge starts at.</summary>
    public int Origin { get; }

    /// <summary>The next half-edge around the same face.</summary>
    public int Next { get; }

    /// <summary>The face on the left of the half-edge (seen from outside).</summary>
    public int Face { get; }
}

/// <summary>
/// An immutable convex polyhedron stored as a half-edge structure with merged coplanar faces, ready for separating-axis tests
/// (Gregorius, GDC 2013, "The Separating Axis Test between Convex Polyhedra"). Vertices are a subset of the input points,
/// in input order; every vertex is a corner of at least three faces.
/// </summary>
public sealed class ConvexHull3D
{
    /// <summary>Largest coordinate magnitude accepted, 2^21.</summary>
    public const float MaxCoordinate = 2097152f;

    /// <summary>Largest number of input points accepted, 2^22.</summary>
    public const int MaxInputPoints = 1 << 22;

    private readonly Vector3F[] _vertices;
    private readonly HullFace3D[] _faces;
    private readonly HullEdge3D[] _edges;
    private readonly int[] _vertexEdge;
    private readonly MassPropertiesF _mass;

    internal ConvexHull3D(Vector3F[] vertices, HullFace3D[] faces, HullEdge3D[] edges, int[] vertexEdge, MassPropertiesF mass)
    {
        _vertices = vertices;
        _faces = faces;
        _edges = edges;
        _vertexEdge = vertexEdge;
        _mass = mass;
    }

    /// <summary>The hull's vertices.</summary>
    public ReadOnlySpan<Vector3F> Vertices => _vertices;

    /// <summary>The hull's faces, ordered by their lowest vertex index.</summary>
    public ReadOnlySpan<HullFace3D> Faces => _faces;

    /// <summary>The half-edges; the twin of edge <c>e</c> is <c>e ^ 1</c>.</summary>
    public ReadOnlySpan<HullEdge3D> Edges => _edges;

    /// <summary>For each vertex, one half-edge that starts at it.</summary>
    public ReadOnlySpan<int> VertexEdges => _vertexEdge;

    /// <summary>Volume, centre of mass and inertia about the centre of mass for unit density, in the hull's frame.</summary>
    public MassPropertiesF MassProperties => _mass;

    /// <summary>Builds the convex hull of a point set (Quickhull: Barber, Dobkin and Huhdanpaa 1996; Gregorius, GDC 2014).</summary>
    /// <param name="points">At least four points with finite coordinates of magnitude at most <see cref="MaxCoordinate"/>, not all coplanar within the tolerance.</param>
    /// <param name="options">Limits and tolerance; the default has none.</param>
    /// <returns>The hull. Identical input and options give identical output on every platform.</returns>
    /// <exception cref="HullBuildException">The input is too small or too large, contains NaN, infinite or out-of-range coordinates, is degenerate, or the options are invalid.</exception>
    public static ConvexHull3D Build(ReadOnlySpan<Vector3F> points, HullBuildOptions options = default)
    {
        return new QuickhullBuilder(points, options).Build();
    }

    /// <summary>The vertex index furthest along a direction; the lowest index wins ties.</summary>
    /// <param name="direction">Any direction; its length does not matter. NaN components give an unspecified vertex.</param>
    public int SupportIndex(Vector3F direction)
    {
        int best = 0;
        float bestDot = Vector3F.Dot(_vertices[0], direction);
        for (int i = 1; i < _vertices.Length; i++)
        {
            float d = Vector3F.Dot(_vertices[i], direction);
            if (d > bestDot)
            {
                bestDot = d;
                best = i;
            }
        }

        return best;
    }

    /// <summary>The vertex furthest along a direction; the lowest index wins ties.</summary>
    /// <param name="direction">Any direction.</param>
    public Vector3F Support(Vector3F direction) => _vertices[SupportIndex(direction)];

    internal static MassPropertiesF ComputeMass(Vector3F[] vertices, int[][] polygons)
    {
        // Volume integrals over the tetrahedra (reference point, polygon fan triangle); Dirichlet moments of a simplex:
        // integral of x x^T = V/20 * (sum p_k p_k^T + (sum p_k)(sum p_k)^T). Evaluated in double about the vertex mean.
        double rx = 0, ry = 0, rz = 0;
        for (int i = 0; i < vertices.Length; i++)
        {
            rx += vertices[i].X;
            ry += vertices[i].Y;
            rz += vertices[i].Z;
        }

        rx /= vertices.Length;
        ry /= vertices.Length;
        rz /= vertices.Length;

        double vol = 0, cx = 0, cy = 0, cz = 0;
        double sxx = 0, syy = 0, szz = 0, sxy = 0, sxz = 0, syz = 0;
        foreach (int[] poly in polygons)
        {
            int a = poly[0];
            for (int k = 1; k + 1 < poly.Length; k++)
            {
                int b = poly[k], c = poly[k + 1];
                double ax = vertices[a].X - rx, ay = vertices[a].Y - ry, az = vertices[a].Z - rz;
                double bx = vertices[b].X - rx, by = vertices[b].Y - ry, bz = vertices[b].Z - rz;
                double qx = vertices[c].X - rx, qy = vertices[c].Y - ry, qz = vertices[c].Z - rz;
                double v = (ax * (by * qz - bz * qy) - ay * (bx * qz - bz * qx) + az * (bx * qy - by * qx)) / 6.0;
                vol += v;
                cx += v * (ax + bx + qx) / 4.0;
                cy += v * (ay + by + qy) / 4.0;
                cz += v * (az + bz + qz) / 4.0;
                double ux = ax + bx + qx, uy = ay + by + qy, uz = az + bz + qz;
                double f = v / 20.0;
                sxx += f * (ax * ax + bx * bx + qx * qx + ux * ux);
                syy += f * (ay * ay + by * by + qy * qy + uy * uy);
                szz += f * (az * az + bz * bz + qz * qz + uz * uz);
                sxy += f * (ax * ay + bx * by + qx * qy + ux * uy);
                sxz += f * (ax * az + bx * bz + qx * qz + ux * uz);
                syz += f * (ay * az + by * bz + qy * qz + uy * uz);
            }
        }

        if (!(vol > 0))
        {
            throw new HullBuildException(HullBuildError.Degenerate, "The hull has no volume.");
        }

        double mx = cx / vol, my = cy / vol, mz = cz / vol;
        // Covariance about the centre of mass, then inertia = trace(C) * I - C.
        sxx -= vol * mx * mx;
        syy -= vol * my * my;
        szz -= vol * mz * mz;
        sxy -= vol * mx * my;
        sxz -= vol * mx * mz;
        syz -= vol * my * mz;
        double tr = sxx + syy + szz;
        var inertia = new Matrix3x3F(
            (float)(tr - sxx), (float)-sxy, (float)-sxz,
            (float)-sxy, (float)(tr - syy), (float)-syz,
            (float)-sxz, (float)-syz, (float)(tr - szz));
        return new MassPropertiesF((float)vol, new Vector3F((float)(mx + rx), (float)(my + ry), (float)(mz + rz)), inertia);
    }
}
