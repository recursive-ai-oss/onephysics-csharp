using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    /// <summary>
    /// Adds static terrain from a 2D signed distance field. The zero contour is extracted by marching squares and simplified within <paramref name="tolerance"/> metres
    /// (default a quarter of a cell, which keeps it within half a cell of the interpolated field), then added as one-sided closed chains with ghost vertices, so bodies slide over the joints without snagging.
    /// The contour is the only thing that collides: unlike 3D, a 2D field is not a dynamic shape (build a polygon or use the polygon tools for moving bodies).
    /// Positions are doubles within +-2^21; the body has no shapes when the field has no inside. Throws ArgumentNullException for a null field and ArgumentOutOfRangeException for a bad tolerance or pose.
    /// </summary>
    public Body AddSdf(SdfGrid2D grid, Vector2D at = default, float angleRadians = 0f, Material2D? material = null, float tolerance = -1f)
    {
        using var replayMutation = RecordMutation("PhysicsWorld2D.AddSdf");
        ArgumentNullException.ThrowIfNull(grid);
        var loops = grid.ExtractContours(tolerance < 0f ? 0.25f * grid.CellSize : tolerance);
        var body = CreateBody(BodyType.Static, at, angleRadians);
        try
        {
            foreach (var loop in loops)
            {
                var points = new Vector2F[loop.Length];
                for (int i = 0; i < points.Length; i++) points[i] = new Vector2F(loop[i].X, loop[i].Y);
                CreateChain(body, points, loop: true, twoSided: false, ShapeFor(material, 0f));
            }
        }
        catch (Exception) { body.Destroy(); throw; }
        return body;
    }
}
