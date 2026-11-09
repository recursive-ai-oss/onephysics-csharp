using System;
using System.Globalization;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Tools;

/// <summary>Validation shared by the tools that take an indexed triangle mesh.</summary>
internal static class MeshInput
{
    /// <summary>The largest coordinate magnitude accepted; the engine's hulls take the same limit.</summary>
    public const float MaxCoordinate = ConvexHull3D.MaxCoordinate;

    /// <summary>The largest vertex and triangle counts accepted, so that every index and every product of counts fits an int.</summary>
    public const int MaxVertices = 1 << 24;

    public static string Inv(int v) => v.ToString(CultureInfo.InvariantCulture);

    public static void Check(ReadOnlySpan<Vector3F> vertices, ReadOnlySpan<int> indices, bool requireTriangles = true)
    {
        if (vertices.Length == 0)
        {
            throw new GeometryToolException(GeometryToolError.Empty, "vertices is empty; a mesh needs at least three vertices", nameof(vertices));
        }

        if (vertices.Length > MaxVertices)
        {
            throw new GeometryToolException(GeometryToolError.OutOfRange, "vertices has " + Inv(vertices.Length) + " entries, the limit is " + Inv(MaxVertices), nameof(vertices));
        }

        if (indices.Length % 3 != 0)
        {
            throw new GeometryToolException(GeometryToolError.BadIndex, "indices has " + Inv(indices.Length) + " entries, which is not a multiple of 3", nameof(indices));
        }

        if (requireTriangles && indices.Length == 0)
        {
            throw new GeometryToolException(GeometryToolError.Empty, "indices is empty; a mesh needs at least one triangle", nameof(indices));
        }

        if (indices.Length / 3 > MaxVertices)
        {
            throw new GeometryToolException(GeometryToolError.OutOfRange, "indices holds " + Inv(indices.Length / 3) + " triangles, the limit is " + Inv(MaxVertices), nameof(indices));
        }

        for (int i = 0; i < vertices.Length; i++)
        {
            Vector3F v = vertices[i];
            if (!float.IsFinite(v.X) || !float.IsFinite(v.Y) || !float.IsFinite(v.Z))
            {
                throw new GeometryToolException(GeometryToolError.NonFinite, "vertices[" + Inv(i) + "] = " + v + " is not finite", nameof(vertices));
            }

            if (Math.Abs(v.X) > MaxCoordinate || Math.Abs(v.Y) > MaxCoordinate || Math.Abs(v.Z) > MaxCoordinate)
            {
                throw new GeometryToolException(GeometryToolError.OutOfRange, "vertices[" + Inv(i) + "] = " + v + " is beyond the limit of " + MaxCoordinate.ToString(CultureInfo.InvariantCulture), nameof(vertices));
            }
        }

        for (int i = 0; i < indices.Length; i++)
        {
            if ((uint)indices[i] >= (uint)vertices.Length)
            {
                throw new GeometryToolException(GeometryToolError.BadIndex, "indices[" + Inv(i) + "] = " + Inv(indices[i]) + " is outside the " + Inv(vertices.Length) + " vertices", nameof(indices));
            }
        }
    }

    /// <summary>The longest axis extent of the vertices (0 for none), in double.</summary>
    public static double Extent(ReadOnlySpan<Vector3F> v)
    {
        if (v.Length == 0) { return 0; }
        double x0 = v[0].X, x1 = x0, y0 = v[0].Y, y1 = y0, z0 = v[0].Z, z1 = z0;
        for (int i = 1; i < v.Length; i++)
        {
            if (v[i].X < x0) { x0 = v[i].X; } else if (v[i].X > x1) { x1 = v[i].X; }
            if (v[i].Y < y0) { y0 = v[i].Y; } else if (v[i].Y > y1) { y1 = v[i].Y; }
            if (v[i].Z < z0) { z0 = v[i].Z; } else if (v[i].Z > z1) { z1 = v[i].Z; }
        }

        return Math.Max(x1 - x0, Math.Max(y1 - y0, z1 - z0));
    }
}
