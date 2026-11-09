using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePhysics.Tools;

/// <summary>Validation of a polygon with holes for the tools that mesh it: the same rules as <see cref="PolygonDecomposer"/> (simple loops that do not touch, holes inside the outline and in no other hole).</summary>
internal static class PolygonDomain
{
    /// <summary>Checks the polygon and returns its loops: the outline counter-clockwise first, then each hole clockwise, with the area of the polygon.</summary>
    public static List<(double X, double Y)[]> Prepare(ReadOnlySpan<RecursiveAI.OneMath.Vector2F> outline, IReadOnlyList<RecursiveAI.OneMath.Vector2F[]>? holes, out double area)
    {
        var loops = new List<(double X, double Y)[]>();
        var outer = PolygonInput.Prepare(outline, nameof(outline), nameof(outline));
        double a0 = PolygonInput.SignedArea(outer);
        if (a0 == 0) { throw new GeometryToolException(GeometryToolError.Degenerate, "outline has zero area", nameof(outline)); }
        if (a0 < 0) { Array.Reverse(outer); a0 = -a0; }
        area = a0;
        loops.Add(outer);
        int total = outer.Length;
        if (holes is not null)
        {
            for (int h = 0; h < holes.Count; h++)
            {
                if (holes[h] is null) { throw new GeometryToolException(GeometryToolError.Empty, "holes[" + MeshInput.Inv(h) + "] is null", nameof(holes)); }
                var loop = PolygonInput.Prepare(holes[h], "holes[" + MeshInput.Inv(h) + "]", nameof(holes));
                double ha = PolygonInput.SignedArea(loop);
                if (ha == 0) { throw new GeometryToolException(GeometryToolError.Degenerate, "holes[" + MeshInput.Inv(h) + "] has zero area", nameof(holes)); }
                if (ha > 0) { Array.Reverse(loop); } else { ha = -ha; }
                area -= ha;
                total += loop.Length;
                if (total > PolygonInput.MaxVertices)
                {
                    throw new GeometryToolException(GeometryToolError.OutOfRange, "the outline and holes have more than " + MeshInput.Inv(PolygonInput.MaxVertices) + " points together", nameof(holes));
                }

                loops.Add(loop);
            }
        }

        string? problem = PolygonInput.FindIntersection(loops);
        if (problem is not null)
        {
            throw new GeometryToolException(GeometryToolError.SelfIntersecting, "the polygon is not simple: " + problem + " (loop 0 is the outline, loop k is holes[k - 1])", nameof(outline));
        }

        for (int h = 1; h < loops.Count; h++)
        {
            if (!PolygonInput.Inside(loops[h][0], outer))
            {
                throw new GeometryToolException(GeometryToolError.SelfIntersecting, "holes[" + MeshInput.Inv(h - 1) + "] lies outside the outline", nameof(holes));
            }

            for (int g = 1; g < loops.Count; g++)
            {
                if (g != h && PolygonInput.Inside(loops[h][0], loops[g]))
                {
                    throw new GeometryToolException(GeometryToolError.SelfIntersecting, "holes[" + MeshInput.Inv(h - 1) + "] lies inside holes[" + MeshInput.Inv(g - 1) + "]; an island inside a hole is a separate polygon", nameof(holes));
                }
            }
        }

        return loops;
    }
}
