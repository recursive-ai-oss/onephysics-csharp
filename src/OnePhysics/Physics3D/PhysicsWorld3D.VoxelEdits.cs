using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>Which faces of a voxel grid hold floating pieces in place.</summary>
[Flags]
public enum VoxelAnchors
{
    /// <summary>Nothing anchors a piece: every connected piece detaches.</summary>
    None = 0,
    /// <summary>A piece touching the lowest layer (y = 0) stays.</summary>
    Bottom = 1,
    /// <summary>A piece touching the x = 0 face stays.</summary>
    MinX = 2,
    /// <summary>A piece touching the last x layer stays.</summary>
    MaxX = 4,
    /// <summary>A piece touching the z = 0 face stays.</summary>
    MinZ = 8,
    /// <summary>A piece touching the last z layer stays.</summary>
    MaxZ = 16,
    /// <summary>A piece touching the top layer stays.</summary>
    Top = 32,
}

public sealed partial class PhysicsWorld3D
{
    private const double VoxelEditLimit = 4194304.0;

    /// <summary>Returns the first voxel-grid shape of a body. Throws ArgumentException when the body has none; a stale body throws a handle error.</summary>
    public Shape GetVoxelShape(Body body)
    {
        int b = ResolveBody(body.RawHandle);
        for (int s = _engine.ShapeHead[b]; s >= 0; s = _engine.NextShape[s])
            if (_engine.VoxelGridOf(s) != null) return new Shape(this, _core.HandleOfShape(s));
        throw new ArgumentException("The body has no voxel grid.", nameof(body));
    }

    /// <summary>Returns a copy of a voxel shape's occupancy (x-fastest, then y, then z). Throws ArgumentException for another kind of shape.</summary>
    public bool[] GetVoxelOccupancy(Shape voxels) => VoxelGrid(voxels, out _).Occupancy.ToArray();

    /// <summary>Returns the grid's dimensions and cell size: (sizeX, sizeY, sizeZ, cellSize). Throws ArgumentException for another kind of shape.</summary>
    public (int X, int Y, int Z, float CellSize) GetVoxelSize(Shape voxels) { var g = VoxelGrid(voxels, out _); return (g.SizeX, g.SizeY, g.SizeZ, g.CellSize); }

    /// <summary>Sets a block of cells to solid or empty; cell (x,y,z) is the lowest corner of the block and sizes are in cells. The block must lie inside the grid. Returns the number of cells that changed. Only faces next to changed cells are rebuilt and keep their ids elsewhere; contacts resting on changed faces end and are found again, bodies near the block wake. Recorded for replay. Throws ArgumentOutOfRangeException for a block outside the grid and ArgumentException for a shape that is not a voxel grid.</summary>
    public int SetVoxels(Shape voxels, int x, int y, int z, int width, int height, int depth, bool solid, int materialIndex = 0)
    {
        using var replayMutation = RecordMutation("PhysicsWorld3D.SetVoxels");
        var grid = VoxelGrid(voxels, out int slot);
        if (x < 0 || y < 0 || z < 0 || width < 0 || height < 0 || depth < 0 || (long)x + width > grid.SizeX || (long)y + height > grid.SizeY || (long)z + depth > grid.SizeZ) throw new ArgumentOutOfRangeException(nameof(x), "Voxel block must lie within the grid.");
        CheckMaterialIndex(slot, materialIndex);
        var cells = new List<int>();
        for (int k = z; k < z + depth; k++) for (int j = y; j < y + height; j++) for (int i = x; i < x + width; i++) cells.Add((k * grid.SizeY + j) * grid.SizeX + i);
        return Apply(slot, grid, cells, solid, materialIndex);
    }

    /// <summary>Sets one cell. Same rules as <see cref="SetVoxels"/>.</summary>
    public int SetVoxel(Shape voxels, int x, int y, int z, bool solid, int materialIndex = 0) => SetVoxels(voxels, x, y, z, 1, 1, 1, solid, materialIndex);

    /// <summary>Fills (<paramref name="solid"/> true) or clears every cell whose centre lies within <paramref name="radius"/> of a world-space point. Returns the number of cells changed. Same rules as <see cref="SetVoxels"/>. Throws ArgumentOutOfRangeException for a non-finite or out-of-range centre or radius.</summary>
    public int StampSphere(Shape voxels, Vector3D center, double radius, bool solid, int materialIndex = 0)
    {
        using var replayMutation = RecordMutation("PhysicsWorld3D.StampSphere");
        var grid = VoxelGrid(voxels, out int slot);
        CheckPoint(center, nameof(center));
        if (!(radius >= 0 && radius <= VoxelEditLimit)) throw new ArgumentOutOfRangeException(nameof(radius), radius, "Radius must be finite and within 0..2^22.");
        CheckMaterialIndex(slot, materialIndex);
        var c = ToGridLocal(slot, center); double cs = grid.CellSize, r2 = radius * radius;
        var cells = new List<int>();
        Range(c.X, radius, cs, grid.SizeX, out int x0, out int x1); Range(c.Y, radius, cs, grid.SizeY, out int y0, out int y1); Range(c.Z, radius, cs, grid.SizeZ, out int z0, out int z1);
        for (int k = z0; k <= z1; k++) for (int j = y0; j <= y1; j++) for (int i = x0; i <= x1; i++)
        {
            double dx = (i + 0.5) * cs - c.X, dy = (j + 0.5) * cs - c.Y, dz = (k + 0.5) * cs - c.Z;
            if (dx * dx + dy * dy + dz * dz <= r2) cells.Add((k * grid.SizeY + j) * grid.SizeX + i);
        }
        return Apply(slot, grid, cells, solid, materialIndex);
    }

    /// <summary>Fills or clears every cell whose centre lies inside an oriented box given in world space (centre, half extents, rotation; a default rotation is the identity). Returns the number of cells changed. Same rules as <see cref="SetVoxels"/>.</summary>
    public int StampBox(Shape voxels, Vector3D center, Vector3F halfExtents, QuaternionF rotation, bool solid, int materialIndex = 0)
    {
        using var replayMutation = RecordMutation("PhysicsWorld3D.StampBox");
        var grid = VoxelGrid(voxels, out int slot);
        CheckPoint(center, nameof(center));
        if (!(halfExtents.X >= 0 && halfExtents.Y >= 0 && halfExtents.Z >= 0 && halfExtents.X <= VoxelEditLimit && halfExtents.Y <= VoxelEditLimit && halfExtents.Z <= VoxelEditLimit)) throw new ArgumentOutOfRangeException(nameof(halfExtents), halfExtents, "Half extents must be finite and within 0..2^22.");
        var q = rotation == default ? QuaternionF.Identity : rotation;
        if (!float.IsFinite(q.X + q.Y + q.Z + q.W) || q.X * q.X + q.Y * q.Y + q.Z * q.Z + q.W * q.W < 0.5f) throw new ArgumentOutOfRangeException(nameof(rotation), rotation, "Rotation must be finite and nonzero.");
        q = q.Normalized(); CheckMaterialIndex(slot, materialIndex);
        var c = ToGridLocal(slot, center); double cs = grid.CellSize;
        // The box's rotation relative to the grid, so cell centres can be tested in the box's own axes.
        var (bodyRot, shapeRot) = (BodyRotation(slot), _engine.Local[slot].Rotation);
        var inGrid = (shapeRot.Conjugated() * bodyRot.Conjugated() * q).Normalized(); var inv = inGrid.Conjugated();
        double ex = 0, ey = 0, ez = 0;
        for (int a = 0; a < 3; a++)
        {
            var axis = a == 0 ? new Vector3F(1, 0, 0) : a == 1 ? new Vector3F(0, 1, 0) : new Vector3F(0, 0, 1);
            var u = inGrid.Rotate(axis); double h = a == 0 ? halfExtents.X : a == 1 ? halfExtents.Y : halfExtents.Z;
            ex += Math.Abs((double)u.X) * h; ey += Math.Abs((double)u.Y) * h; ez += Math.Abs((double)u.Z) * h;
        }
        Range(c.X, ex, cs, grid.SizeX, out int x0, out int x1); Range(c.Y, ey, cs, grid.SizeY, out int y0, out int y1); Range(c.Z, ez, cs, grid.SizeZ, out int z0, out int z1);
        var cells = new List<int>();
        for (int k = z0; k <= z1; k++) for (int j = y0; j <= y1; j++) for (int i = x0; i <= x1; i++)
        {
            var d = inv.Rotate(new Vector3F((float)((i + 0.5) * cs - c.X), (float)((j + 0.5) * cs - c.Y), (float)((k + 0.5) * cs - c.Z)));
            if (Math.Abs((double)d.X) <= halfExtents.X && Math.Abs((double)d.Y) <= halfExtents.Y && Math.Abs((double)d.Z) <= halfExtents.Z) cells.Add((k * grid.SizeY + j) * grid.SizeX + i);
        }
        return Apply(slot, grid, cells, solid, materialIndex);
    }

    /// <summary>Turns pieces of a voxel grid that are no longer connected to an anchored face into dynamic bodies. Pieces are the 6-connected groups of solid cells, taken in order of their lowest cell index; a piece touching one of <paramref name="anchors"/> stays. A detached piece becomes one body at its centroid, covered by boxes (greedy blocks of cells, so its volume is cells × cell size cubed), with density 1 when the grid's density is zero and the grid's friction, restitution and category. Pieces under <paramref name="minCells"/> cells are removed as dust without a body; past <paramref name="maxBodies"/> bodies further pieces stay in the grid. Returns the new bodies in piece order. Wakes bodies near the removed cells. Recorded for replay.</summary>
    public Body[] DetachVoxelIslands(Shape voxels, VoxelAnchors anchors = VoxelAnchors.Bottom, int minCells = 1, int maxBodies = 64)
    {
        using var replayMutation = RecordMutation("PhysicsWorld3D.DetachVoxelIslands");
        var grid = VoxelGrid(voxels, out int slot);
        if (minCells < 1) throw new ArgumentOutOfRangeException(nameof(minCells), minCells, "Pieces must have at least one cell.");
        if (maxBodies < 0 || maxBodies > 4096) throw new ArgumentOutOfRangeException(nameof(maxBodies), maxBodies, "Body limit must be in 0..4096.");
        int sx = grid.SizeX, sy = grid.SizeY, sz = grid.SizeZ; var solid = grid.Occupancy.ToArray();
        var piece = new int[solid.Length]; Array.Fill(piece, -1);
        var pieces = new List<List<int>>(); var anchored = new List<bool>(); var stack = new List<int>();
        for (int start = 0; start < solid.Length; start++)
        {
            if (!solid[start] || piece[start] >= 0) continue;
            int id = pieces.Count; var cells = new List<int>(); bool anchor = false;
            piece[start] = id; stack.Add(start);
            while (stack.Count != 0)
            {
                int c = stack[^1]; stack.RemoveAt(stack.Count - 1); cells.Add(c);
                int x = c % sx, y = c / sx % sy, z = c / (sx * sy);
                if ((y == 0 && (anchors & VoxelAnchors.Bottom) != 0) || (y == sy - 1 && (anchors & VoxelAnchors.Top) != 0) || (x == 0 && (anchors & VoxelAnchors.MinX) != 0) || (x == sx - 1 && (anchors & VoxelAnchors.MaxX) != 0) || (z == 0 && (anchors & VoxelAnchors.MinZ) != 0) || (z == sz - 1 && (anchors & VoxelAnchors.MaxZ) != 0)) anchor = true;
                if (x > 0) Visit(c - 1); if (x < sx - 1) Visit(c + 1); if (y > 0) Visit(c - sx); if (y < sy - 1) Visit(c + sx); if (z > 0) Visit(c - sx * sy); if (z < sz - 1) Visit(c + sx * sy);
            }
            cells.Sort(); pieces.Add(cells); anchored.Add(anchor);
            void Visit(int n) { if (solid[n] && piece[n] < 0) { piece[n] = id; stack.Add(n); } }
        }
        var bodies = new List<List<int>>(); var remove = new List<int>();
        for (int i = 0; i < pieces.Count; i++)
        {
            if (anchored[i]) continue;
            if (pieces[i].Count >= minCells) { if (bodies.Count >= maxBodies) continue; bodies.Add(pieces[i]); }
            remove.AddRange(pieces[i]);
        }
        if (remove.Count == 0) return [];
        remove.Sort();
        double cs = grid.CellSize; var poseBody = voxels.Body; var origin = poseBody.Position;
        var made = new List<(Vector3D Centroid, List<(Vector3F Size, Vector3F Offset)> Boxes)>();
        foreach (var cells in bodies)
        {
            double cx = 0, cy = 0, cz = 0;
            foreach (int c in cells) { cx += (c % sx + 0.5) * cs; cy += (c / sx % sy + 0.5) * cs; cz += (c / (sx * sy) + 0.5) * cs; }
            cx /= cells.Count; cy /= cells.Count; cz /= cells.Count;
            var inPiece = new HashSet<int>(cells); var covered = new HashSet<int>(); var boxes = new List<(Vector3F, Vector3F)>();
            foreach (int c in cells)
            {
                if (covered.Contains(c)) continue;
                int x0 = c % sx, y0 = c / sx % sy, z0 = c / (sx * sy), x1 = x0, y1 = y0, z1 = z0;
                while (x1 + 1 < sx && Free(x1 + 1, y0, z0)) x1++;
                while (y1 + 1 < sy && RowFree(x0, x1, y1 + 1, z0, z0)) y1++;
                while (z1 + 1 < sz && RowFree3(x0, x1, y0, y1, z1 + 1)) z1++;
                for (int k = z0; k <= z1; k++) for (int j = y0; j <= y1; j++) for (int i = x0; i <= x1; i++) covered.Add((k * sy + j) * sx + i);
                boxes.Add((new Vector3F((float)((x1 - x0 + 1) * cs), (float)((y1 - y0 + 1) * cs), (float)((z1 - z0 + 1) * cs)),
                    new Vector3F((float)((x0 + x1 + 1) * 0.5 * cs - cx), (float)((y0 + y1 + 1) * 0.5 * cs - cy), (float)((z0 + z1 + 1) * 0.5 * cs - cz))));
                bool Free(int i, int j, int k) { int n = (k * sy + j) * sx + i; return inPiece.Contains(n) && !covered.Contains(n); }
                bool RowFree(int a, int b, int j, int k0, int k1) { for (int i = a; i <= b; i++) if (!Free(i, j, k0)) return false; return true; }
                bool RowFree3(int a, int b, int j0, int j1, int k) { for (int j = j0; j <= j1; j++) for (int i = a; i <= b; i++) if (!Free(i, j, k)) return false; return true; }
            }
            made.Add((new Vector3D(cx, cy, cz), boxes));
        }
        var def = new ShapeDef { Density = _engine.Density[slot] > 0f ? _engine.Density[slot] : 1f, Friction = _engine.Friction[slot], Restitution = _engine.Restitution[slot], RollingResistance = _engine.Rolling[slot], CategoryBits = _engine.Category[slot], MaskBits = _engine.Mask[slot], GroupIndex = _engine.Group[slot] };
        Apply(slot, grid, remove, false, 0);
        var rot = BodyRotation(slot); var shapeRot = _engine.Local[slot].Rotation; var shapeAt = _engine.Local[slot].Translation;
        var result = new Body[made.Count];
        for (int i = 0; i < result.Length; i++)
        {
            var (centroid, boxes) = made[i];
            var local = new Vector3F((float)centroid.X, (float)centroid.Y, (float)centroid.Z);
            var inBody = shapeRot.Rotate(local) + shapeAt; var world = rot.Rotate(inBody);
            var body = CreateBody(BodyType.Dynamic, new Vector3D(origin.X + world.X, origin.Y + world.Y, origin.Z + world.Z), rot * shapeRot);
            foreach (var (size, offset) in boxes) { var d = def; d.LocalPosition = offset; CreateShape(body, ShapeSpec3D.Box(size), d); }
            result[i] = body;
        }
        return result;
    }

    private VoxelGrid3D VoxelGrid(Shape voxels, out int slot)
    {
        slot = _core.Resolve(voxels.RawHandle);
        return _engine.VoxelGridOf(slot) ?? throw new ArgumentException("The shape is not a voxel grid.", nameof(voxels));
    }
    private void CheckMaterialIndex(int slot, int index)
    {
        int count = _core.ContactMaterials.MaterialCount(slot);
        if (index < 0 || (count != 0 && index >= count)) throw new ArgumentOutOfRangeException(nameof(index), index, "Material index must be in 0..material count - 1 (0 when the grid has no table).");
    }
    private static void CheckPoint(Vector3D p, string name)
    {
        if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || !double.IsFinite(p.Z) || Math.Abs(p.X) > VoxelEditLimit || Math.Abs(p.Y) > VoxelEditLimit || Math.Abs(p.Z) > VoxelEditLimit) throw new ArgumentOutOfRangeException(name, p, "Point must be finite and within +-2^22.");
    }
    private QuaternionF BodyRotation(int slot) => _engine.RotationOfBody(_core.Shapes.Owner[slot]);
    private Vector3D BodyOrigin(int slot) { int b = _core.Shapes.Owner[slot]; var p = _core.Bodies.PositionOf(b); var off = _engine.RotationOfBody(b).Rotate(_engine.LocalCenterOf(b)); return new Vector3D(p[0] - off.X, p[1] - off.Y, p[2] - off.Z); }
    private Vector3D ToGridLocal(int slot, Vector3D world)
    {
        var o = BodyOrigin(slot); var l = _engine.Local[slot];
        var inBody = BodyRotation(slot).Conjugated().Rotate(new Vector3F((float)(world.X - o.X), (float)(world.Y - o.Y), (float)(world.Z - o.Z)));
        var v = l.Rotation.Conjugated().Rotate(inBody - l.Translation);
        return new Vector3D(v.X, v.Y, v.Z);
    }
    private static void Range(double centre, double reach, double cs, int n, out int lo, out int hi)
    {
        lo = (int)Math.Clamp(Math.Floor((centre - reach) / cs - 0.5), 0, n); hi = (int)Math.Clamp(Math.Ceiling((centre + reach) / cs - 0.5), -1, n - 1);
    }

    private int Apply(int slot, VoxelGrid3D grid, List<int> cells, bool solid, int materialIndex)
    {
        if (cells.Count == 0) return 0;
        // The region the edit can disturb: the changed cells' box, one cell wider, plus a speculative margin, in world space.
        int x0 = int.MaxValue, y0 = int.MaxValue, z0 = int.MaxValue, x1 = -1, y1 = -1, z1 = -1;
        foreach (int c in cells)
        {
            int x = c % grid.SizeX, y = c / grid.SizeX % grid.SizeY, z = c / (grid.SizeX * grid.SizeY);
            if (x < x0) x0 = x; if (x > x1) x1 = x; if (y < y0) y0 = y; if (y > y1) y1 = y; if (z < z0) z0 = z; if (z > z1) z1 = z;
        }
        double cs = grid.CellSize, margin = cs + 0.05;
        var o = BodyOrigin(slot); var rot = BodyRotation(slot); var l = _engine.Local[slot];
        double minX = double.MaxValue, minY = double.MaxValue, minZ = double.MaxValue, maxX = double.MinValue, maxY = double.MinValue, maxZ = double.MinValue;
        for (int k = 0; k < 8; k++)
        {
            var p = new Vector3F((float)(((k & 1) != 0 ? x1 + 1 : x0) * cs), (float)(((k & 2) != 0 ? y1 + 1 : y0) * cs), (float)(((k & 4) != 0 ? z1 + 1 : z0) * cs));
            var w = rot.Rotate(l.Rotation.Rotate(p) + l.Translation);
            minX = Math.Min(minX, o.X + w.X); maxX = Math.Max(maxX, o.X + w.X); minY = Math.Min(minY, o.Y + w.Y); maxY = Math.Max(maxY, o.Y + w.Y); minZ = Math.Min(minZ, o.Z + w.Z); maxZ = Math.Max(maxZ, o.Z + w.Z);
        }
        var region = new Aabb3D(minX - margin, minY - margin, minZ - margin, maxX + margin, maxY + margin, maxZ + margin);
        return _engine.EditVoxels(slot, cells.ToArray(), solid, materialIndex, in region);
    }
}
