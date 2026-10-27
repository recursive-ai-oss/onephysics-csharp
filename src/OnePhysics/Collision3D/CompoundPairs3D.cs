using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Broadphase;

namespace RecursiveAI.OnePhysics.Collision3D;

/// <summary>A pair of sub-shape ids from a compound-versus-compound enumeration.</summary>
public readonly struct SubShapePair3D : IEquatable<SubShapePair3D>
{
    /// <summary>Creates a pair.</summary>
    public SubShapePair3D(int a, int b)
    {
        A = a;
        B = b;
    }

    /// <summary>Child id in the first compound.</summary>
    public int A { get; }

    /// <summary>Child id in the second compound.</summary>
    public int B { get; }

    /// <inheritdoc/>
    public bool Equals(SubShapePair3D other) => A == other.A && B == other.B;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is SubShapePair3D p && Equals(p);

    /// <inheritdoc/>
    public override int GetHashCode() => (A * 65599) ^ B;

    /// <summary>Equality of both ids.</summary>
    public static bool operator ==(SubShapePair3D x, SubShapePair3D y) => x.Equals(y);

    /// <summary>Negation of <c>==</c>.</summary>
    public static bool operator !=(SubShapePair3D x, SubShapePair3D y) => !x.Equals(y);

    /// <inheritdoc/>
    public override string ToString() => $"({A}, {B})";
}

/// <summary>
/// Child-pair enumeration for the narrowphase: which children of a compound can touch a convex shape or another compound. The test is on padded child bounds
/// (<see cref="ShapeBounds3D"/>): the bounds of the other side, grown by <c>margin</c> on every side in double precision, must overlap or touch the child's bounds.
/// Results come in a fixed order, ascending child id (ascending by first id, then second, for pairs), whatever shape the tree has.
/// </summary>
public static class CompoundPairs3D
{
    private static void CheckMargin(float margin)
    {
        if (!(margin >= 0f) || !float.IsFinite(margin))
        {
            throw new ShapeCompositionException(ShapeCompositionError.InvalidQuery, $"margin is {margin}; a finite value of at least 0 is expected.", nameof(margin));
        }
    }

    /// <summary>The padded bounds of a shape under a pose, grown by <paramref name="margin"/> in double precision; the box every pair test uses for the non-compound side.</summary>
    public static void GetQueryBox(in ConvexShape3D shape, in RigidTransformF pose, float margin, out double minX, out double minY, out double minZ, out double maxX, out double maxY, out double maxZ)
    {
        ShapeBounds3D.Compute(shape, pose, out Vector3F lo, out Vector3F hi);
        double m = margin;
        minX = lo.X - m;
        minY = lo.Y - m;
        minZ = lo.Z - m;
        maxX = hi.X + m;
        maxY = hi.Y + m;
        maxZ = hi.Z + m;
    }

    /// <summary>Children of <paramref name="compound"/> whose bounds meet those of <paramref name="shape"/> (see the class summary), ascending.</summary>
    /// <param name="compound">The compound.</param>
    /// <param name="shape">The convex shape in its own frame; not the point shape.</param>
    /// <param name="shapeInCompound">Placement of the shape's frame in the compound's frame; a unit rotation, translation components at most 2^22.</param>
    /// <param name="margin">Finite, at least 0; use the speculative distance.</param>
    /// <param name="results">Receives child ids; cleared first.</param>
    /// <exception cref="ShapeCompositionException">An argument is unusable.</exception>
    public static void FindChildren(CompoundShape3D compound, in ConvexShape3D shape, in RigidTransformF shapeInCompound, float margin, List<int> results)
    {
        ArgumentNullException.ThrowIfNull(compound);
        ArgumentNullException.ThrowIfNull(results);
        CheckMargin(margin);
        if (shape.Kind == ConvexShapeKind3D.Sphere && shape.ConvexRadius == 0f)
        {
            throw new ShapeCompositionException(ShapeCompositionError.InvalidShape, "shape is the zero-size point shape; a sphere, box, capsule, cylinder or hull is expected.", nameof(shape));
        }

        PlacedShape3D.CheckTransform(shapeInCompound, nameof(shapeInCompound), nameof(shapeInCompound), ManifoldGenerator3D.MaxOffset);
        results.Clear();
        GetQueryBox(shape, shapeInCompound, margin, out double x0, out double y0, out double z0, out double x1, out double y1, out double z1);
        compound.QueryBox(new Aabb3D(x0, y0, z0, x1, y1, z1), results);
        results.Sort();
    }

    /// <summary>All pairs of a child of <paramref name="a"/> and a child of <paramref name="b"/> whose bounds meet, sorted by (A, B).</summary>
    /// <param name="a">The first compound.</param>
    /// <param name="b">The second compound.</param>
    /// <param name="bInA">Placement of B's frame in A's frame; a unit rotation, translation components at most 2^22.</param>
    /// <param name="margin">Finite, at least 0.</param>
    /// <param name="results">Receives the pairs; cleared first.</param>
    /// <param name="scratch">Optional working list to reuse between calls so steady-state calls do not allocate; cleared first.</param>
    /// <exception cref="ShapeCompositionException">An argument is unusable.</exception>
    public static void FindPairs(CompoundShape3D a, CompoundShape3D b, in RigidTransformF bInA, float margin, List<SubShapePair3D> results, List<int>? scratch = null)
    {
        ArgumentNullException.ThrowIfNull(a);
        ArgumentNullException.ThrowIfNull(b);
        ArgumentNullException.ThrowIfNull(results);
        CheckMargin(margin);
        PlacedShape3D.CheckTransform(bInA, nameof(bInA), nameof(bInA), ManifoldGenerator3D.MaxOffset);
        results.Clear();
        List<int> hits = scratch ?? new List<int>();
        for (int j = 0; j < b.SlotCount; j++)
        {
            if (!b.IsLive(j))
            {
                continue;
            }

            PlacedShape3D child = b.GetChild(j);
            GetQueryBox(child.Shape, bInA * child.Local, margin, out double x0, out double y0, out double z0, out double x1, out double y1, out double z1);
            hits.Clear();
            a.QueryBox(new Aabb3D(x0, y0, z0, x1, y1, z1), hits);
            foreach (int i in hits)
            {
                results.Add(new SubShapePair3D(i, j));
            }
        }

        results.Sort(static (p, q) => p.A != q.A ? p.A.CompareTo(q.A) : p.B.CompareTo(q.B));
    }
}
