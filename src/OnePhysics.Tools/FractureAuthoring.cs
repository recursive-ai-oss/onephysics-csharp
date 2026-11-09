using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Destruction;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tools;

/// <summary>
/// Pre-fracturing of authored meshes and outlines: the shape is first split into convex pieces (<see cref="ConvexDecomposer"/> in 3D, <see cref="PolygonDecomposer"/> in 2D), then each piece is cut into Voronoi cells with
/// its share of the requested cell count, and the cells of all pieces are bonded where their faces coincide. The result is a <see cref="P3.FractureAsset"/> (or <see cref="P2.FractureAsset"/>) the runtime turns into a
/// breakable body with <c>AddBreakable</c>. Deterministic: equal input and options give equal cells on every platform.
/// </summary>
public static class FractureAuthoring
{
    /// <summary>
    /// Decomposes a closed triangle mesh into convex pieces and fractures each piece into about <c>def.CellCount</c> cells in total, shared between the pieces by volume (a piece that would get fewer than two cells stays one cell).
    /// Each piece's seed is <c>def.Seed</c> plus its index. Pieces bond across faces only where the decomposition left them coincident; pieces that touch only loosely are left unbonded and break away as one body when
    /// the compound first splits.
    /// </summary>
    /// <exception cref="GeometryToolException">The mesh is not a usable closed mesh (see <see cref="ConvexDecomposer"/>).</exception>
    /// <exception cref="ArgumentOutOfRangeException">A fracture setting is out of range.</exception>
    public static P3.FractureAsset Prefracture(ReadOnlySpan<Vector3F> vertices, ReadOnlySpan<int> indices, P3.FractureDef? def = null, ConvexDecompositionOptions decomposition = default)
    {
        var d = def ?? new P3.FractureDef();
        var parts = ConvexDecomposer.Decompose(vertices, indices, decomposition);
        double total = 0;
        foreach (var h in parts.Hulls) total += h.MassProperties.Mass;
        var cells = new List<Vector3F[]>();
        for (int i = 0; i < parts.Hulls.Count; i++)
        {
            var hull = parts.Hulls[i];
            int share = total > 0 ? (int)Math.Floor(d.CellCount * (hull.MassProperties.Mass / total) + 0.5) : 1;
            var points = new Vector3F[hull.Vertices.Length];
            hull.Vertices.CopyTo(points);
            if (share < 2) { cells.Add(points); continue; }
            var sub = new P3.FractureDef
            {
                CellCount = Math.Min(share, 512), Pattern = d.Pattern, Seed = unchecked(d.Seed + (ulong)i), Points = d.Points, ImpactPoint = d.ImpactPoint,
                ImpactRadius = d.ImpactRadius, ClusterPower = d.ClusterPower, MinFragmentVolume = d.MinFragmentVolume, MergeSmall = d.MergeSmall,
            };
            var asset = P3.FractureAsset.FromHull(points, sub);
            for (int k = 0; k < asset.CellCount; k++) cells.Add(asset.GetCell(k).ToArray());
        }
        return P3.FractureAsset.FromCells(cells);
    }

    /// <summary>Makes an asset whose cells are the convex pieces of a decomposition, bonded where they share faces.</summary>
    /// <exception cref="ArgumentNullException">The decomposition is null.</exception>
    public static P3.FractureAsset FromDecomposition(ConvexDecomposition decomposition)
    {
        ArgumentNullException.ThrowIfNull(decomposition);
        var cells = new List<Vector3F[]>(decomposition.Hulls.Count);
        foreach (var h in decomposition.Hulls) { var p = new Vector3F[h.Vertices.Length]; h.Vertices.CopyTo(p); cells.Add(p); }
        return P3.FractureAsset.FromCells(cells);
    }

    /// <summary>
    /// Decomposes a simple polygon (optionally with holes) into convex pieces and fractures each piece into cells, sharing <c>def.CellCount</c> between the pieces by area. A piece that would get fewer than two cells
    /// stays one cell. Cells have at most 8 vertices, the 2D shape limit.
    /// </summary>
    /// <exception cref="GeometryToolException">The outline is not a usable simple polygon (see <see cref="PolygonDecomposer"/>).</exception>
    /// <exception cref="ArgumentOutOfRangeException">A fracture setting is out of range.</exception>
    public static P2.FractureAsset Prefracture(ReadOnlySpan<Vector2F> outline, IReadOnlyList<Vector2F[]>? holes = null, P2.FractureDef? def = null)
    {
        var d = def ?? new P2.FractureDef();
        var pieces = PolygonDecomposer.Decompose(outline, holes, new PolygonDecompositionOptions { MaxVerticesPerPiece = 8 });
        var areas = new double[pieces.Length];
        double total = 0;
        for (int i = 0; i < pieces.Length; i++)
        {
            double a = 0; var p = pieces[i];
            for (int k = 0; k < p.Length; k++) { var u = p[k]; var v = p[(k + 1) % p.Length]; a += (double)u.X * v.Y - (double)v.X * u.Y; }
            areas[i] = Math.Abs(a) * 0.5; total += areas[i];
        }
        var cells = new List<Vector2F[]>();
        for (int i = 0; i < pieces.Length; i++)
        {
            int share = total > 0 ? (int)Math.Floor(d.CellCount * (areas[i] / total) + 0.5) : 1;
            if (share < 2) { cells.Add(pieces[i]); continue; }
            var sub = new P2.FractureDef
            {
                CellCount = Math.Min(share, 512), Pattern = d.Pattern, Seed = unchecked(d.Seed + (ulong)i), Points = d.Points, ImpactPoint = d.ImpactPoint,
                ImpactRadius = d.ImpactRadius, ClusterPower = d.ClusterPower, MinFragmentVolume = d.MinFragmentVolume, MergeSmall = d.MergeSmall,
            };
            var asset = P2.FractureAsset.FromPolygon(pieces[i], sub);
            for (int k = 0; k < asset.CellCount; k++) cells.Add(asset.GetCell(k).ToArray());
        }
        return P2.FractureAsset.FromCells(cells);
    }
}
