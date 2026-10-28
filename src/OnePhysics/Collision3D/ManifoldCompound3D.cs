using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Collision3D;

/// <summary>
/// The contact manifold of one child pair, with the sub-shape ids that go with it. A contact between two shapes with compounds is one constraint per child pair: the pair
/// <c>(SubShapeA, SubShapeB)</c> is part of the constraint's key (the shared core stores both ids per constraint), and the manifold's own 32-bit feature ids match points between frames
/// within that constraint. A shape that is not a compound has sub-shape id 0.
/// </summary>
public readonly struct ChildContact3D
{
    internal ChildContact3D(int subShapeA, int subShapeB, Manifold3D manifold)
    {
        SubShapeA = subShapeA;
        SubShapeB = subShapeB;
        Manifold = manifold;
    }

    /// <summary>Child id in shape A's compound, or 0 when A is not a compound.</summary>
    public int SubShapeA { get; }

    /// <summary>Child id in shape B's compound, or 0 when B is not a compound.</summary>
    public int SubShapeB { get; }

    /// <summary>The manifold in the frame of shape A (the compound's frame when A is a compound); its feature ids are those of the child shapes.</summary>
    public Manifold3D Manifold { get; }

    /// <summary>
    /// One 64-bit key per point that is unique across the child pairs of a shape pair: <c>(SubShapeA &lt;&lt; 48) | (SubShapeB &lt;&lt; 32) | featureId</c>. Child ids are below 2^16
    /// (<see cref="CompoundShape3D.MaxChildren"/>), so nothing is lost; a host that merges all child pairs into one constraint can match points across frames with this key.
    /// </summary>
    /// <param name="i">Point index below <c>Manifold.PointCount</c>.</param>
    public ulong GetPointKey(int i) => ((ulong)(ushort)SubShapeA << 48) | ((ulong)(ushort)SubShapeB << 32) | Manifold[i].Id;
}

public sealed partial class ManifoldGenerator3D
{
    private static void CheckBodyPose(in RigidTransformF bInA)
    {
        QuaternionF q = bInA.Rotation;
        Vector3F t = bInA.Translation;
        float ql = (q.X * q.X) + (q.Y * q.Y) + (q.Z * q.Z) + (q.W * q.W);
        if (!(ql >= 0.99f && ql <= 1.01f) || !(Math.Abs(t.X) <= MaxOffset && Math.Abs(t.Y) <= MaxOffset && Math.Abs(t.Z) <= MaxOffset))
        {
            throw new ArgumentOutOfRangeException(nameof(bInA), bInA, $"bInA is {bInA}; a unit rotation (squared length within 1% of 1) and a translation with each component at most {MaxOffset} in magnitude are expected.");
        }
    }

    // A manifold computed in a placed shape's own frame, expressed in its holder's frame: the points and the normal move by the local transform; separations and ids are unchanged.
    private static Manifold3D ToHolderFrame(in Manifold3D m, in RigidTransformF local)
    {
        int n = m.PointCount;
        if (n == 0)
        {
            return m;
        }

        ManifoldPoint3D p0 = default, p1 = default, p2 = default, p3 = default;
        for (int i = 0; i < n; i++)
        {
            ManifoldPoint3D s = m[i];
            var mapped = new ManifoldPoint3D(local.TransformPoint(s.Point), s.Separation, s.Id);
            switch (i)
            {
                case 0: p0 = mapped; break;
                case 1: p1 = mapped; break;
                case 2: p2 = mapped; break;
                default: p3 = mapped; break;
            }
        }

        return new Manifold3D(local.TransformVector(m.Normal), n, p0, p1, p2, p3);
    }

    /// <summary>
    /// The manifold of two placed shapes: the local transforms are composed into the pose of B's shape frame in A's shape frame, the usual manifold is built there and the result is moved back into
    /// A's holder frame. Feature ids are those of the shapes, so a shape placed anywhere has the ids it would have unplaced.
    /// </summary>
    /// <param name="a">Shape A with its local transform.</param>
    /// <param name="b">Shape B with its local transform.</param>
    /// <param name="bInA">Placement of B's holder in A's holder; a unit rotation, translation components at most <see cref="MaxOffset"/>.</param>
    /// <param name="speculativeDistance">As for the plain overload.</param>
    /// <exception cref="ArgumentOutOfRangeException">An argument is out of range (see the plain overload).</exception>
    public Manifold3D Collide(in PlacedShape3D a, in PlacedShape3D b, in RigidTransformF bInA, float speculativeDistance = DefaultSpeculativeDistance)
    {
        CheckBodyPose(bInA);
        return ChildManifold(a.Shape, a.Local, true, b.Shape, b.Local, true, bInA, speculativeDistance);
    }

    private Manifold3D ChildManifold(in ConvexShape3D a, in RigidTransformF aLocal, bool aPlaced, in ConvexShape3D b, in RigidTransformF bLocal, bool bPlaced, in RigidTransformF bInA, float speculativeDistance)
    {
        RigidTransformF rel = bInA;
        if (aPlaced)
        {
            rel = aLocal.Inverted() * rel;
        }

        if (bPlaced)
        {
            rel = rel * bLocal;
        }

        Manifold3D m = Collide(a, b, rel, speculativeDistance);
        return aPlaced ? ToHolderFrame(m, aLocal) : m;
    }

    /// <summary>
    /// Manifolds of a compound A against a convex shape B, one per child whose bounds meet B's (see <see cref="CompoundPairs3D"/>, margin = the speculative distance), in ascending child id.
    /// Children with an empty manifold are left out. Manifolds are in the compound's frame; <c>SubShapeA</c> is the child id and <c>SubShapeB</c> is 0.
    /// </summary>
    /// <param name="a">The compound.</param>
    /// <param name="b">The convex shape, in its own frame.</param>
    /// <param name="bInA">Placement of B's frame in the compound's frame; limits as for <see cref="Collide(in ConvexShape3D, in ConvexShape3D, in RigidTransformF, float)"/>.</param>
    /// <param name="results">Receives the contacts; cleared first. Reuse the list to avoid allocation.</param>
    /// <param name="speculativeDistance">As for the plain overload.</param>
    /// <returns>The number of contacts.</returns>
    public int CollideCompoundConvex(CompoundShape3D a, in ConvexShape3D b, in RigidTransformF bInA, List<ChildContact3D> results, float speculativeDistance = DefaultSpeculativeDistance)
    {
        ArgumentNullException.ThrowIfNull(a);
        ArgumentNullException.ThrowIfNull(results);
        CheckBodyPose(bInA);
        results.Clear();
        List<int> kids = _scratchIds;
        CompoundPairs3D.FindChildren(a, b, bInA, speculativeDistance, kids);
        foreach (int i in kids)
        {
            PlacedShape3D c = a.GetChild(i);
            Manifold3D m = ChildManifold(c.Shape, c.Local, true, b, default, false, bInA, speculativeDistance);
            if (m.PointCount > 0)
            {
                results.Add(new ChildContact3D(i, 0, m));
            }
        }

        return results.Count;
    }

    /// <summary>
    /// Manifolds of a convex shape A against a compound B, one per child whose bounds meet A's, in ascending child id. Manifolds are in A's frame; <c>SubShapeA</c> is 0 and <c>SubShapeB</c> is the child id.
    /// </summary>
    /// <param name="a">The convex shape, in its own frame.</param>
    /// <param name="b">The compound.</param>
    /// <param name="bInA">Placement of the compound's frame in A's frame.</param>
    /// <param name="results">Receives the contacts; cleared first.</param>
    /// <param name="speculativeDistance">As for the plain overload.</param>
    /// <returns>The number of contacts.</returns>
    public int CollideConvexCompound(in ConvexShape3D a, CompoundShape3D b, in RigidTransformF bInA, List<ChildContact3D> results, float speculativeDistance = DefaultSpeculativeDistance)
    {
        ArgumentNullException.ThrowIfNull(b);
        ArgumentNullException.ThrowIfNull(results);
        CheckBodyPose(bInA);
        results.Clear();
        List<int> kids = _scratchIds;
        CompoundPairs3D.FindChildren(b, a, bInA.Inverted(), speculativeDistance, kids);
        foreach (int j in kids)
        {
            PlacedShape3D c = b.GetChild(j);
            Manifold3D m = ChildManifold(a, default, false, c.Shape, c.Local, true, bInA, speculativeDistance);
            if (m.PointCount > 0)
            {
                results.Add(new ChildContact3D(0, j, m));
            }
        }

        return results.Count;
    }

    /// <summary>
    /// Manifolds of every child pair of two compounds whose bounds meet, in ascending (A, B) order. Manifolds are in A's compound frame; the sub-shape ids are the child ids.
    /// </summary>
    /// <param name="a">The first compound.</param>
    /// <param name="b">The second compound.</param>
    /// <param name="bInA">Placement of B's frame in A's frame.</param>
    /// <param name="results">Receives the contacts; cleared first.</param>
    /// <param name="speculativeDistance">As for the plain overload.</param>
    /// <returns>The number of contacts.</returns>
    public int CollideCompoundCompound(CompoundShape3D a, CompoundShape3D b, in RigidTransformF bInA, List<ChildContact3D> results, float speculativeDistance = DefaultSpeculativeDistance)
    {
        ArgumentNullException.ThrowIfNull(a);
        ArgumentNullException.ThrowIfNull(b);
        ArgumentNullException.ThrowIfNull(results);
        CheckBodyPose(bInA);
        results.Clear();
        List<SubShapePair3D> pairs = _scratchPairs;
        CompoundPairs3D.FindPairs(a, b, bInA, speculativeDistance, pairs, _scratchIds);
        foreach (SubShapePair3D p in pairs)
        {
            PlacedShape3D ca = a.GetChild(p.A), cb = b.GetChild(p.B);
            Manifold3D m = ChildManifold(ca.Shape, ca.Local, true, cb.Shape, cb.Local, true, bInA, speculativeDistance);
            if (m.PointCount > 0)
            {
                results.Add(new ChildContact3D(p.A, p.B, m));
            }
        }

        return results.Count;
    }

    private readonly List<int> _scratchIds = new();
    private readonly List<SubShapePair3D> _scratchPairs = new();
}
