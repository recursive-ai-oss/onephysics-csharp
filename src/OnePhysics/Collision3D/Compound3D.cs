using System;

namespace RecursiveAI.OnePhysics.Collision3D;

/// <summary>
/// A compound of convex children fixed at construction. The children keep the ids <c>0 .. Count-1</c> in the order given, and the tree is built once by a binned surface-area-heuristic
/// rebuild (<see cref="Broadphase.DynamicTree{TBox}.Rebuild"/>) over the children inserted in id order, so the same children give the same tree bytes (<see cref="CompoundShape3D.WriteBvh"/>) on every platform.
/// </summary>
public sealed class Compound3D : CompoundShape3D
{
    private Compound3D()
    {
    }

    /// <summary>Builds a compound.</summary>
    /// <param name="children">One to <see cref="CompoundShape3D.MaxChildren"/> children. Each needs a real shape (not the point shape) and a valid local transform (a unit rotation, translation components at most
    /// <see cref="PlacedShape3D.MaxLocalOffset"/>), and its transformed bounds must lie within ±2^22.</param>
    /// <exception cref="ShapeCompositionException">There are no children, too many, or a child is unusable; the message names <c>children[i]</c>.</exception>
    public static Compound3D Create(ReadOnlySpan<CompoundChild3D> children)
    {
        if (children.Length == 0)
        {
            throw new ShapeCompositionException(ShapeCompositionError.EmptyCompound, "children is empty; at least one child is expected.", nameof(children));
        }

        if (children.Length > MaxChildren)
        {
            throw new ShapeCompositionException(ShapeCompositionError.TooManyChildren, $"children has {children.Length} entries; at most {MaxChildren} are allowed.", nameof(children));
        }

        var c = new Compound3D();
        for (int i = 0; i < children.Length; i++)
        {
            c.AddCore(children[i], $"children[{i}]");
        }

        c.RebuildCore();
        return c;
    }
}
