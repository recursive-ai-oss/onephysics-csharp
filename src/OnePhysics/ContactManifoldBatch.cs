using System;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>Raw pre-solve batch hook, invoked once on the calling thread after built-in rules and individual modifiers. Only edit listed manifolds; do not mutate world topology or retain views.</summary>
public delegate void ContactBatchModifier(ContactManifoldBatch batch);

/// <summary>Stack-only ordered mutable contact batch. All views expire when the callback returns. The core exposes raw payload storage; GetManifold offers validated coefficient/separation/response setters.</summary>
public readonly ref struct ContactManifoldBatch
{
    private readonly WorldCore _core;
    private readonly ReadOnlySpan<int> _slots;
    private readonly ReadOnlySpan<byte> _counts;
    private readonly Span<float> _extra;
    internal ContactManifoldBatch(WorldCore core, ReadOnlySpan<int> slots, ReadOnlySpan<byte> counts, Span<float> extra) { _core = core; _slots = slots; _counts = counts; _extra = extra; }
    /// <summary>The owning raw world. Only listed contact payloads may be edited during this callback.</summary>
    public WorldCore Core => _core ?? throw new InvalidOperationException("The default contact batch has no world.");
    /// <summary>Number of non-sensor manifolds, including manifolds whose response a preceding modifier disabled.</summary>
    public int Count => _slots.Length;
    /// <summary>Constraint slots in shape-pair/subshape/constraint order; never scheduler/color order.</summary>
    public ReadOnlySpan<int> ConstraintSlots => _slots;
    /// <summary>Returns a mutable manifold view. Throws ArgumentOutOfRangeException for an invalid batch index.</summary>
    public ContactModification GetManifold(int index)
    {
        if ((uint)index >= (uint)_slots.Length) throw new ArgumentOutOfRangeException(nameof(index), index, "Batch index must be within 0..Count-1.");
        int c = _slots[index]; return new ContactModification(Core, c, _extra.Slice(ContactModificationPipeline.ExtraStride * c, ContactModificationPipeline.ExtraStride), _counts[index]);
    }
}
