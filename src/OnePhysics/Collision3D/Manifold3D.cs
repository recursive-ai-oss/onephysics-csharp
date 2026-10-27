using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Collision3D;

/// <summary>One point of a 3D contact manifold.</summary>
public readonly struct ManifoldPoint3D
{
    /// <summary>Creates a point.</summary>
    public ManifoldPoint3D(Vector3F point, float separation, uint id)
    {
        Point = point;
        Separation = separation;
        Id = id;
    }

    /// <summary>The contact point, midway between the two surfaces, in the frame of shape A.</summary>
    public Vector3F Point { get; }

    /// <summary>Signed distance between the surfaces along the manifold normal: negative when the shapes overlap, positive for a speculative point that is not touching yet.</summary>
    public float Separation { get; }

    /// <summary>
    /// Feature id, stable while the same pair of geometric features stays in contact, so warm starting can match points between frames. The high 16 bits
    /// name the feature of shape A and the low 16 bits the feature of shape B. A feature code has its kind in the top two bits (0 face, 1 edge, 2 vertex, 3 derived)
    /// and an index in the low 14 bits: a hull's face, edge (half-edge index / 2) or vertex index (a box uses the indices of a unit cube hull), a sphere's single
    /// vertex 0, the ends of a capsule or tapered capsule as vertices 0 (+Y) and 1 (-Y) with the axis as edge 0, a cylinder's rim samples, side line and caps (see docs/collision3d.md).
    /// A point where a clipped edge of the incident face crosses a side of the reference face pairs the two edges; a surviving corner of the incident face pairs the
    /// reference face with that corner. Indices above 16383 wrap.
    /// </summary>
    public uint Id { get; }
}

/// <summary>
/// Contact manifold of two convex shapes: a unit normal and at most four points, all in the frame of shape A. Points farther apart than the speculative distance
/// are not included, so an empty manifold means "not close enough to matter". The manifold fits the shared core's 3D contact layout
/// (<c>DimensionLayout.Spatial.MaxManifoldPoints</c> = 4) as is: normal, then per point the position, separation and feature id.
/// </summary>
public readonly struct Manifold3D
{
    private readonly ManifoldPoint3D _p0, _p1, _p2, _p3;

    internal Manifold3D(Vector3F normal, int count, ManifoldPoint3D p0, ManifoldPoint3D p1, ManifoldPoint3D p2, ManifoldPoint3D p3)
    {
        Normal = normal;
        PointCount = count;
        _p0 = p0;
        _p1 = p1;
        _p2 = p2;
        _p3 = p3;
    }

    /// <summary>Largest number of points a manifold holds.</summary>
    public const int MaxPoints = 4;

    /// <summary>Unit normal pointing from shape A to shape B, in A's frame. Zero when there are no points.</summary>
    public Vector3F Normal { get; }

    /// <summary>Number of points, 0 to 4. Points are ordered by ascending feature id (ties in selection order).</summary>
    public int PointCount { get; }

    /// <summary>Returns point <paramref name="i"/>.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="i"/> is not in 0 to <see cref="PointCount"/> − 1.</exception>
    public ManifoldPoint3D this[int i]
    {
        get
        {
            if ((uint)i >= (uint)PointCount)
            {
                throw new ArgumentOutOfRangeException(nameof(i), i, "point index must be below PointCount (" + PointCount + ")");
            }

            return i switch { 0 => _p0, 1 => _p1, 2 => _p2, _ => _p3 };
        }
    }

    /// <summary>The smallest separation over the points, or <see cref="float.PositiveInfinity"/> when the manifold is empty.</summary>
    public float MinSeparation
    {
        get
        {
            float m = float.PositiveInfinity;
            for (int i = 0; i < PointCount; i++)
            {
                m = Math.Min(m, this[i].Separation);
            }

            return m;
        }
    }
}
