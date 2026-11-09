using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

public readonly partial struct Body
{
    /// <summary>Clears the terrain cells whose centres lie within a world-space circle; see <see cref="PhysicsWorld2D.CarveCircle"/>. Returns the number of cells changed.</summary>
    public int CarveCircle(Vector2D center, double radius) { var (w, _) = Resolve(); return w.CarveCircle(this, center, radius); }

    /// <summary>Fills the terrain cells whose centres lie within a world-space circle; see <see cref="PhysicsWorld2D.FillCircle"/>.</summary>
    public int FillCircle(Vector2D center, double radius) { var (w, _) = Resolve(); return w.FillCircle(this, center, radius); }

    /// <summary>Clears the terrain cells whose centres lie inside a world-space polygon; see <see cref="PhysicsWorld2D.CarvePolygon"/>.</summary>
    public int CarvePolygon(ReadOnlySpan<Vector2D> points) { var (w, _) = Resolve(); return w.CarvePolygon(this, points); }

    /// <summary>Fills the terrain cells whose centres lie inside a world-space polygon; see <see cref="PhysicsWorld2D.FillPolygon"/>.</summary>
    public int FillPolygon(ReadOnlySpan<Vector2D> points) { var (w, _) = Resolve(); return w.FillPolygon(this, points); }

    /// <summary>Turns pieces no longer connected to an anchored edge into dynamic bodies; see <see cref="PhysicsWorld2D.DetachIslands"/>.</summary>
    public Body[] DetachIslands(TerrainAnchors anchors = TerrainAnchors.Bottom, int minCells = 1, int maxBodies = 64) { var (w, _) = Resolve(); return w.DetachIslands(this, anchors, minCells, maxBodies); }
}
