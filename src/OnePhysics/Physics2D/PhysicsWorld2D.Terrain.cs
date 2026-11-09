using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>Which grid edges hold floating pieces of carvable terrain in place.</summary>
[Flags]
public enum TerrainAnchors
{
    /// <summary>Nothing anchors a piece: every connected piece detaches.</summary>
    None = 0,
    /// <summary>A piece touching the bottom row stays.</summary>
    Bottom = 1,
    /// <summary>A piece touching the leftmost column stays.</summary>
    Left = 2,
    /// <summary>A piece touching the rightmost column stays.</summary>
    Right = 4,
    /// <summary>A piece touching the top row stays.</summary>
    Top = 8,
}

public sealed partial class PhysicsWorld2D
{
    private const double TerrainLimit = 4194304.0;

    /// <summary>Adds static carvable terrain: a bitmask whose solid cells are sampled at their centres, with the boundary extracted by marching squares (octilinear contour segments, corners cut at half a cell) and rebuilt only near an edit. Cells are x-fastest, dimensions 1..1024 with at most 65536 cells, cell size at least 4e-5 metres; the body origin is the bottom-left corner. Use <see cref="CarveCircle"/>, <see cref="FillCircle"/>, <see cref="CarvePolygon"/>, <see cref="FillPolygon"/>, <see cref="StampMask"/> and <see cref="DetachIslands"/> to edit it. Throws ArgumentOutOfRangeException for invalid input.</summary>
    public Body AddBitmaskTerrain(int width, int height, ReadOnlySpan<bool> solid, float cellSize = 1f, Vector2D at = default, float angleRadians = 0f, Material2D? material = null)
    {
        using var replayMutation = RecordMutation("PhysicsWorld2D.AddBitmaskTerrain");
        if (solid.Length != (long)width * height) throw new ArgumentOutOfRangeException(nameof(solid), solid.Length, "Mask must contain width * height entries.");
        var tiles = new TileKind2D[solid.Length];
        for (int i = 0; i < tiles.Length; i++) tiles[i] = solid[i] ? TileKind2D.Solid : TileKind2D.Empty;
        var grid = new TileGrid2D(width, height, tiles, cellSize, default, contour: true);
        var body = CreateBody(BodyType.Static, at, angleRadians);
        try { CreateTileGrid(body, grid, ShapeFor(material, 0f)); }
        catch (Exception) { body.Destroy(); throw; }
        return body;
    }

    /// <summary>Clears every cell whose centre lies within <paramref name="radius"/> of a world-space point. Returns the number of cells changed. Only the boundary near the cells is rebuilt: untouched collision segments keep their handles and cached contacts, and awake or sleeping bodies near the edit wake. Recorded for replay. Works on tile grids too. Throws ArgumentOutOfRangeException for a non-finite or out-of-range centre or radius, ArgumentException for a body without a grid.</summary>
    public int CarveCircle(Body terrain, Vector2D center, double radius) => EditCircle(terrain, center, radius, TileKind2D.Empty, "PhysicsWorld2D.CarveCircle");

    /// <summary>Fills every cell whose centre lies within <paramref name="radius"/> of a world-space point with solid terrain. Same rules as <see cref="CarveCircle"/>.</summary>
    public int FillCircle(Body terrain, Vector2D center, double radius) => EditCircle(terrain, center, radius, TileKind2D.Solid, "PhysicsWorld2D.FillCircle");

    /// <summary>Clears every cell whose centre lies inside a world-space polygon (even-odd rule; 3 to 1024 finite points). Returns the number of cells changed. Same rules as <see cref="CarveCircle"/>.</summary>
    public int CarvePolygon(Body terrain, ReadOnlySpan<Vector2D> points) => EditPolygon(terrain, points, TileKind2D.Empty, "PhysicsWorld2D.CarvePolygon");

    /// <summary>Fills every cell whose centre lies inside a world-space polygon with solid terrain. Same rules as <see cref="CarvePolygon"/>.</summary>
    public int FillPolygon(Body terrain, ReadOnlySpan<Vector2D> points) => EditPolygon(terrain, points, TileKind2D.Solid, "PhysicsWorld2D.FillPolygon");

    /// <summary>Stamps a cell mask (x-fastest, width*height entries) with its bottom-left cell at grid cell (cellX, cellY): every true entry becomes solid when <paramref name="fill"/> is true, empty otherwise. The rectangle must lie inside the grid. Returns the number of cells changed. Same rules as <see cref="CarveCircle"/>; throws ArgumentOutOfRangeException for a bad rectangle or mask length.</summary>
    public int StampMask(Body terrain, int cellX, int cellY, int width, int height, ReadOnlySpan<bool> mask, bool fill)
    {
        using var replayMutation = RecordMutation("PhysicsWorld2D.StampMask");
        if (mask.Length != (long)width * height) throw new ArgumentOutOfRangeException(nameof(mask), mask.Length, "Mask must contain width * height entries.");
        return EditCells(terrain, cellX, cellY, width, height, mask, fill ? TileKind2D.Solid : TileKind2D.Empty);
    }

    /// <summary>Turns pieces of a bitmask terrain that are no longer connected to an anchored edge into dynamic bodies. Pieces are the 4-connected groups of solid cells, taken in order of their lowest cell index (row, then column); a piece touching one of <paramref name="anchors"/> stays. A detached piece becomes a body placed at its centroid, covered by boxes (greedy rectangles of cells, so its area is cells × cell size squared; the contour's chamfered corners are not reproduced), with the terrain's shape definition, using density 1 when the terrain's density is zero. Pieces smaller than <paramref name="minCells"/> are removed as dust without a body; once <paramref name="maxBodies"/> bodies exist further pieces stay in the terrain. Returns the new bodies in piece order. Only contour grids are accepted. Wakes bodies near the removed cells. Recorded for replay.</summary>
    public Body[] DetachIslands(Body terrain, TerrainAnchors anchors = TerrainAnchors.Bottom, int minCells = 1, int maxBodies = 64)
    {
        using var replayMutation = RecordMutation("PhysicsWorld2D.DetachIslands");
        if (minCells < 1) throw new ArgumentOutOfRangeException(nameof(minCells), minCells, "Pieces must have at least one cell.");
        if (maxBodies < 0 || maxBodies > 4096) throw new ArgumentOutOfRangeException(nameof(maxBodies), maxBodies, "Body limit must be in 0..4096.");
        var entry = TileEntry(terrain); var grid = entry.Grid;
        if (!grid.IsContour) throw new ArgumentException("Only bitmask (contour) terrain can detach pieces.", nameof(terrain));
        int w = grid.Width, h = grid.Height; var tiles = grid.Tiles.ToArray();
        var piece = new int[w * h]; Array.Fill(piece, -1);
        var stack = new List<int>(); var pieces = new List<List<int>>(); var anchored = new List<bool>();
        for (int start = 0; start < piece.Length; start++)
        {
            if (tiles[start] == TileKind2D.Empty || piece[start] >= 0) continue;
            int id = pieces.Count; var cells = new List<int>(); bool anchor = false;
            piece[start] = id; stack.Add(start);
            while (stack.Count != 0)
            {
                int c = stack[^1]; stack.RemoveAt(stack.Count - 1); cells.Add(c);
                int x = c % w, y = c / w;
                if ((y == 0 && (anchors & TerrainAnchors.Bottom) != 0) || (y == h - 1 && (anchors & TerrainAnchors.Top) != 0) || (x == 0 && (anchors & TerrainAnchors.Left) != 0) || (x == w - 1 && (anchors & TerrainAnchors.Right) != 0)) anchor = true;
                if (x > 0) Visit(c - 1); if (x < w - 1) Visit(c + 1); if (y > 0) Visit(c - w); if (y < h - 1) Visit(c + w);
            }
            cells.Sort(); pieces.Add(cells); anchored.Add(anchor);
            void Visit(int n) { if (tiles[n] != TileKind2D.Empty && piece[n] < 0) { piece[n] = id; stack.Add(n); } }
        }
        var bodies = new List<(List<int> Cells, int Rect)>(); var removeMask = new bool[w * h]; int kept = 0, any = 0;
        for (int i = 0; i < pieces.Count; i++)
        {
            if (anchored[i]) continue;
            if (pieces[i].Count >= minCells) { if (kept >= maxBodies) continue; kept++; bodies.Add((pieces[i], 0)); }
            foreach (int c in pieces[i]) { removeMask[c] = true; any++; }
        }
        if (any == 0) return [];
        var origin = terrain.Position; var rot = terrain.Rotation; double size = grid.TileSize;
        var created = new List<(double X, double Y, List<Shape2D> Shapes)>();
        foreach (var (cells, _) in bodies)
        {
            double cx = 0, cy = 0;
            foreach (int c in cells) { cx += (c % w + 0.5) * size; cy += (c / w + 0.5) * size; }
            cx /= cells.Count; cy /= cells.Count;
            var shapes = new List<Shape2D>();
            var inPiece = new HashSet<int>(cells); var covered = new HashSet<int>();
            foreach (int c in cells)
            {
                if (covered.Contains(c)) continue;
                int x0 = c % w, y0 = c / w, x1 = x0;
                while (x1 + 1 < w && inPiece.Contains(y0 * w + x1 + 1) && !covered.Contains(y0 * w + x1 + 1)) x1++;
                int y1 = y0;
                while (y1 + 1 < h && RowFree(y1 + 1, x0, x1)) y1++;
                for (int yy = y0; yy <= y1; yy++) for (int xx = x0; xx <= x1; xx++) covered.Add(yy * w + xx);
                double mx = (x0 + x1 + 1) * 0.5 * size - cx, my = (y0 + y1 + 1) * 0.5 * size - cy;
                shapes.Add(Shape2D.CreateBox((float)((x1 - x0 + 1) * 0.5 * size), (float)((y1 - y0 + 1) * 0.5 * size), new Vec2((float)mx, (float)my), Rot2.Identity));
                bool RowFree(int yy, int a, int b) { for (int xx = a; xx <= b; xx++) if (!inPiece.Contains(yy * w + xx) || covered.Contains(yy * w + xx)) return false; return true; }
            }
            created.Add((cx, cy, shapes));
        }
        var def = entry.Definition; if (!(def.Density > 0f)) def.Density = 1f;
        EditCells(terrain, 0, 0, w, h, removeMask, TileKind2D.Empty);
        var result = new Body[created.Count];
        for (int i = 0; i < result.Length; i++)
        {
            var (cx, cy, shapes) = created[i];
            double wx = origin.X + rot.C * cx - rot.S * cy, wy = origin.Y + rot.S * cx + rot.C * cy;
            var body = CreateBody(BodyType.Dynamic, new Vector2D(wx, wy), terrain.Angle);
            foreach (var s in shapes) AddShape(body, s, def);
            result[i] = body;
        }
        return result;
    }

    private int EditCircle(Body terrain, Vector2D center, double radius, TileKind2D kind, string operation)
    {
        using var replayMutation = RecordMutation(operation);
        CheckTerrainPoint(center, nameof(center));
        if (!(radius >= 0 && radius <= TerrainLimit)) throw new ArgumentOutOfRangeException(nameof(radius), radius, "Radius must be finite and within 0..2^22.");
        var entry = TileEntry(terrain); var grid = entry.Grid; var (lx, ly) = ToLocal(terrain, center); double size = grid.TileSize;
        int x0 = CellLow((lx - radius) / size, grid.Width), x1 = CellHigh((lx + radius) / size, grid.Width), y0 = CellLow((ly - radius) / size, grid.Height), y1 = CellHigh((ly + radius) / size, grid.Height);
        if (x0 > x1 || y0 > y1) return 0;
        int w = x1 - x0 + 1, h = y1 - y0 + 1; var mask = new bool[w * h]; double r2 = radius * radius;
        for (int j = 0; j < h; j++) for (int i = 0; i < w; i++)
        {
            double dx = (x0 + i + 0.5) * size - lx, dy = (y0 + j + 0.5) * size - ly;
            mask[j * w + i] = dx * dx + dy * dy <= r2;
        }
        return EditCells(terrain, x0, y0, w, h, mask, kind);
    }

    private int EditPolygon(Body terrain, ReadOnlySpan<Vector2D> points, TileKind2D kind, string operation)
    {
        using var replayMutation = RecordMutation(operation);
        if (points.Length < 3 || points.Length > 1024) throw new ArgumentOutOfRangeException(nameof(points), points.Length, "A polygon needs 3 to 1024 points.");
        var entry = TileEntry(terrain); var grid = entry.Grid; double size = grid.TileSize;
        var px = new double[points.Length]; var py = new double[points.Length];
        double minX = double.MaxValue, minY = double.MaxValue, maxX = double.MinValue, maxY = double.MinValue;
        for (int i = 0; i < points.Length; i++)
        {
            CheckTerrainPoint(points[i], nameof(points));
            (px[i], py[i]) = ToLocal(terrain, points[i]);
            minX = Math.Min(minX, px[i]); maxX = Math.Max(maxX, px[i]); minY = Math.Min(minY, py[i]); maxY = Math.Max(maxY, py[i]);
        }
        int x0 = CellLow(minX / size, grid.Width), x1 = CellHigh(maxX / size, grid.Width), y0 = CellLow(minY / size, grid.Height), y1 = CellHigh(maxY / size, grid.Height);
        if (x0 > x1 || y0 > y1) return 0;
        int w = x1 - x0 + 1, h = y1 - y0 + 1; var mask = new bool[w * h];
        for (int j = 0; j < h; j++) for (int i = 0; i < w; i++)
        {
            double cx = (x0 + i + 0.5) * size, cy = (y0 + j + 0.5) * size; bool inside = false;
            // Even-odd crossing count (Shimrat 1962) over half-open edges, so a ray through a vertex counts once.
            for (int a = 0, b = points.Length - 1; a < points.Length; b = a++)
                if ((py[a] > cy) != (py[b] > cy) && cx < (px[b] - px[a]) * (cy - py[a]) / (py[b] - py[a]) + px[a]) inside = !inside;
            mask[j * w + i] = inside;
        }
        return EditCells(terrain, x0, y0, w, h, mask, kind);
    }

    private int EditCells(Body terrain, int x, int y, int width, int height, ReadOnlySpan<bool> mask, TileKind2D kind)
    {
        var entry = TileEntry(terrain);
        var grid = entry.Grid.Edit(x, y, width, height, mask, kind, out var changed);
        if (changed.Count == 0) return 0;
        ReplaceTileBoundary(terrain, entry, grid, false);
        WakeNearTerrainEdit(terrain, grid, changed);
        return changed.Count;
    }

    // A body resting on a changed part of the boundary may have lost its support, so every dynamic body whose box
    // reaches the changed cells (plus a speculative margin) wakes. Bodies farther away are not touched.
    private void WakeNearTerrainEdit(Body terrain, TileGrid2D grid, TileGrid2D.Changed c)
    {
        double size = grid.TileSize, margin = 0.5 * size + 0.05;
        var rot = terrain.Rotation; var origin = terrain.Position;
        double minX = double.MaxValue, minY = double.MaxValue, maxX = double.MinValue, maxY = double.MinValue;
        for (int k = 0; k < 4; k++)
        {
            double lx = ((k & 1) != 0 ? c.X1 + 1 : c.X0) * size, ly = ((k & 2) != 0 ? c.Y1 + 1 : c.Y0) * size;
            double wx = origin.X + rot.C * lx - rot.S * ly, wy = origin.Y + rot.S * lx + rot.C * ly;
            minX = Math.Min(minX, wx); maxX = Math.Max(maxX, wx); minY = Math.Min(minY, wy); maxY = Math.Max(maxY, wy);
        }
        var scratch = _engine.QueryBounds(new Aabb2D(minX - margin, minY - margin, maxX + margin, maxY + margin), default(QueryFilter2D), int.MaxValue);
        try
        {
            for (int i = 0; i < scratch.Count; i++)
            {
                int owner = _core.Shapes.Owner[scratch.Hits[i].Slot];
                if (_core.Bodies.IsDynamic(owner)) { _core.WakeIsland(_core.Bodies.Island[owner]); _core.Bodies.SleepTime[owner] = 0f; }
            }
        }
        finally { scratch.Release(); }
    }

    private static (double X, double Y) ToLocal(Body terrain, Vector2D world)
    {
        var p = terrain.Position; var r = terrain.Rotation; double dx = world.X - p.X, dy = world.Y - p.Y;
        return (r.C * dx + r.S * dy, -r.S * dx + r.C * dy);
    }
    private static void CheckTerrainPoint(Vector2D p, string name)
    {
        if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || Math.Abs(p.X) > TerrainLimit || Math.Abs(p.Y) > TerrainLimit) throw new ArgumentOutOfRangeException(name, p, "Point must be finite and within +-2^22.");
    }
    private static int CellLow(double q, int n) => (int)Math.Clamp(Math.Floor(q - 0.5), 0, n);
    private static int CellHigh(double q, int n) => (int)Math.Clamp(Math.Ceiling(q - 0.5), -1, n - 1);
}
