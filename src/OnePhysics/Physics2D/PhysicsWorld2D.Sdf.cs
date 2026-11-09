using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    /// <summary>
    /// Adds static terrain from a 2D signed distance field. The zero contour is extracted by marching squares and simplified within <paramref name="tolerance"/> metres
    /// (default a quarter of a cell, which keeps it within half a cell of the interpolated field), then added as one-sided closed chains with ghost vertices, so bodies slide over the joints without snagging.
    /// With <paramref name="type"/> static (the default) the contour chains are the only thing that collides. A dynamic or kinematic body instead gets the inside of the contour as a cover of convex polygons
    /// (holes bridged, ear-clipped, then merged across diagonals while convex, at most 8 vertices each), so it collides and gets its mass from the contour's exact inside area times <paramref name="density"/>;
    /// the contour lies within half a cell of the interpolated field, as for static terrain. Pieces collide through the ordinary polygon contacts, not through field samples, and the cover must stay within 8192 contour vertices (raise <paramref name="tolerance"/> otherwise).
    /// Positions are doubles within +-2^21; the body has no shapes when the field has no inside. Throws ArgumentNullException for a null field and ArgumentOutOfRangeException for a bad tolerance, density or pose.
    /// </summary>
    public Body AddSdf(SdfGrid2D grid, Vector2D at = default, float angleRadians = 0f, Material2D? material = null, float tolerance = -1f, BodyType type = BodyType.Static, float density = 1f)
    {
        using var replayMutation = RecordMutation("PhysicsWorld2D.AddSdf");
        ArgumentNullException.ThrowIfNull(grid);
        var loops = grid.ExtractContours(tolerance < 0f ? 0.25f * grid.CellSize : tolerance);
        var body = CreateBody(type, at, angleRadians);
        try
        {
            if (type != BodyType.Static)
            {
                foreach (var piece in ConvexCover2D.Cover(loops))
                {
                    var points = new Vector2F[piece.Length];
                    for (int i = 0; i < points.Length; i++) points[i] = new Vector2F(piece[i].X, piece[i].Y);
                    try { CreatePolygon(body, points, 0f, ShapeFor(material, density)); }
                    catch (InvalidShapeException) { }   // a sliver below the minimum shape extent carries no area worth colliding with
                }
                return body;
            }
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
