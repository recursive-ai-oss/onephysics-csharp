using System;
using RecursiveAI.OneMath;
using System.Diagnostics.CodeAnalysis;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>
/// A compound whose children can be added, removed and moved between steps. Edits update the tree incrementally (removal, then reinsertion by the surface-area heuristic with rotations, see
/// <c>docs/broadphase.md</c>), so the tree's bytes depend on the whole edit sequence but on nothing else: the same edits give the same tree on every platform. Query results depend only on the set
/// of children (see <see cref="CompoundShape3D"/>), so they equal those of a <see cref="Compound3D"/> built from the same children; <see cref="Optimize"/> rebuilds the tree to the same shape a static compound
/// of the same children would have. Child ids are stable: a child keeps its id until removed, and freed ids are reused lowest first.
/// Not thread-safe against concurrent edits and queries.
/// </summary>
[Experimental("OPHYS0001")]
public sealed class MutableCompound3D : CompoundShape3D
{
    /// <summary>Creates an empty compound. It has no mass properties until it has a child.</summary>
    public MutableCompound3D()
    {
    }

    /// <summary>Creates a compound holding <paramref name="children"/> with ids <c>0 .. n-1</c>, inserted in order (not rebuilt; call <see cref="Optimize"/> for the static layout).</summary>
    /// <exception cref="ShapeCompositionException">There are too many children or one is unusable; the message names <c>children[i]</c>.</exception>
    public MutableCompound3D(ReadOnlySpan<CompoundChild3D> children)
    {
        if (children.Length > MaxChildren)
        {
            throw new ShapeCompositionException(ShapeCompositionError.TooManyChildren, $"children has {children.Length} entries; at most {MaxChildren} are allowed.", nameof(children));
        }

        for (int i = 0; i < children.Length; i++)
        {
            AddCore(children[i], $"children[{i}]");
        }
    }

    /// <summary>Adds a child and returns its handle. The child takes the lowest free id.</summary>
    /// <exception cref="ShapeCompositionException">The child is unusable (<c>child</c> names it) or the compound is full; nothing changes.</exception>
    public CompoundChildHandle3D Add(in CompoundChild3D child)
    {
        int slot = AddCore(child, nameof(child));
        return GetHandle(slot);
    }

    /// <summary>Removes a child; its id may be given to a later child, with a new generation.</summary>
    /// <exception cref="ShapeCompositionException"><paramref name="handle"/> is not a live child of this compound.</exception>
    public void Remove(CompoundChildHandle3D handle) => RemoveCore(CheckHandle(handle, nameof(handle)));

    /// <summary>Moves a child to a new local transform.</summary>
    /// <exception cref="ShapeCompositionException">The handle is stale or the transform is unusable; nothing changes.</exception>
    public void Move(CompoundChildHandle3D handle, in RigidTransformF local) => MoveCore(CheckHandle(handle, nameof(handle)), local, nameof(local));

    /// <summary>Changes a child's material id.</summary>
    /// <exception cref="ShapeCompositionException"><paramref name="handle"/> is not a live child of this compound.</exception>
    public void SetMaterialId(CompoundChildHandle3D handle, int materialId) => SetMaterialCore(CheckHandle(handle, nameof(handle)), materialId);

    /// <summary>Rebuilds the tree from scratch with the binned surface-area-heuristic builder; the result depends only on the children and their ids, not on the edit history.</summary>
    public void Optimize() => RebuildCore();
}
