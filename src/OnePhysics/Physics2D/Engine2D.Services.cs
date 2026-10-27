using System;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>Decides whether two shapes may collide, as a struct so the call is specialized and allocation-free. Middle tier only: the happy path uses the layer matrix and <see cref="PairPredicate"/>.</summary>
/// <remarks>Install one with <see cref="PhysicsWorld2D.SetContactFilter{TFilter}"/>. It runs serially, once when a pair of overlapping shapes is first found and again when filters are re-evaluated, so it must be a pure function of the shapes' properties.</remarks>
public interface IContactFilter
{
    /// <summary>True when the two shapes may collide. <paramref name="a"/> has the lower shape slot.</summary>
    bool ShouldCollide(Shape a, Shape b);
}

internal abstract class ContactFilterHolder
{
    public abstract bool Test(Shape a, Shape b);
}

internal sealed class ContactFilterHolder<TFilter> : ContactFilterHolder where TFilter : struct, IContactFilter
{
    private TFilter _filter;

    public ContactFilterHolder(TFilter filter) => _filter = filter;

    public override bool Test(Shape a, Shape b) => _filter.ShouldCollide(a, b);
}

internal sealed partial class Engine2D
{
    public const int LayerCount = 32;
    private const uint Services = 2;

    public const byte FlagSensor = 1;

    // Per shape (index = shape slot).
    public byte[] ShapeFlags = new byte[8];
    public byte[] ShapeLayer = new byte[8];

    /// <summary>Row i holds the layers layer i collides with (bit j). Symmetric. Default: everything collides with everything.</summary>
    public uint[] LayerMatrix = NewMatrix();

    /// <summary>Set when anything that decides whether pairs may collide changed; the next step re-evaluates every pair. Saved in snapshots.</summary>
    public bool FilterDirty;

    public ContactFilterHolder? ContactFilter;
    private PairPredicate? _seenPairFilter;
    private ContactFilterHolder? _seenContactFilter;
    private bool[] _fresh = new bool[8];

    private static uint[] NewMatrix()
    {
        var m = new uint[LayerCount];
        Array.Fill(m, uint.MaxValue);
        return m;
    }

    private void EnsureServiceShapes(int n)
    {
        Grow(ref ShapeFlags, n);
        Grow(ref ShapeLayer, n);
    }

    public bool IsSensor(int shape) => (ShapeFlags[shape] & FlagSensor) != 0;

    // ------------------------------------------------------------------ filtering

    /// <summary>
    /// The pure part of the collision rule, safe to call from the parallel pair search: no pair of sensors, then the layer matrix, then Catto's category/mask/group rule
    /// (a shared non-zero group decides: positive collides, negative never).
    /// </summary>
    public bool CanCollide(int sa, int sb)
    {
        if ((ShapeFlags[sa] & ShapeFlags[sb] & FlagSensor) != 0) { return false; }
        if (((LayerMatrix[ShapeLayer[sa]] >> ShapeLayer[sb]) & 1u) == 0) { return false; }
        if (Group[sa] == Group[sb] && Group[sa] != 0) { return Group[sa] > 0; }
        return (Mask[sa] & Category[sb]) != 0 && (Category[sa] & Mask[sb]) != 0;
    }

    /// <summary>The whole rule: <see cref="CanCollide"/> plus the pair predicate and the struct contact filter (both serial).</summary>
    private bool PairAllowed(int a, int b)
    {
        if (Core.Shapes.Owner[a] == Core.Shapes.Owner[b] || !CanCollide(a, b)) { return false; }
        var predicate = Opt.PairFilter;
        if (predicate != null && !predicate(WrapShape(a), WrapShape(b))) { return false; }
        var filter = ContactFilter;
        return filter == null || filter.Test(WrapShape(a), WrapShape(b));
    }

    /// <summary>
    /// Start of the contact stage. If a filter changed since the last step (shape category, mask, group or layer, the layer matrix, a new pair predicate or contact filter, or an explicit request),
    /// every pair in the table is evaluated again in ascending pair index: pairs that may no longer collide lose their constraint (an end event) and leave the table, and every proxy is
    /// reported to the broadphase again so pairs that are newly allowed are found in this same step.
    /// </summary>
    private void Refilter()
    {
        if (!ReferenceEquals(Opt.PairFilter, _seenPairFilter) || !ReferenceEquals(ContactFilter, _seenContactFilter)) { FilterDirty = true; }
        _seenPairFilter = Opt.PairFilter;
        _seenContactFilter = ContactFilter;
        if (!FilterDirty) { return; }
        FilterDirty = false;
        for (int p = 0; p < PairHigh; p++)
        {
            if (!PairLive[p] || PairAllowed(PairA[p], PairB[p])) { continue; }
            if (PairCons[p] >= 0) { Core.RemoveConstraint(PairCons[p]); }
            FreePair(p);
        }

        Bp.TouchAll();
    }

    /// <summary>After a restore the delegates are whatever the host installed; they are not a change.</summary>
    private void ResetSeenFilters()
    {
        _seenPairFilter = Opt.PairFilter;
        _seenContactFilter = ContactFilter;
    }

    // ------------------------------------------------------------------ snapshot: filter, layer and sensor state

    /// <summary>True when any service state differs from a world that never used these features; only then is the extra block written (so such a world's bytes and hash do not change).</summary>
    private bool HasServiceState()
    {
        if (FilterDirty || Opt.HitSpeedThreshold != 1f) { return true; }
        foreach (uint row in LayerMatrix) { if (row != uint.MaxValue) { return true; } }
        var shapes = Core.Shapes;
        for (int s = 0; s < shapes.HighWater; s++)
        {
            if (shapes.Handles.IsLive(s) && (ShapeFlags[s] != 0 || ShapeLayer[s] != 0)) { return true; }
        }

        return false;
    }

    private void WriteServices(StateWriter w, int shapeCount)
    {
        w.WriteUInt32s(LayerMatrix);
        w.WriteSingle(Opt.HitSpeedThreshold);
        w.WriteBool(FilterDirty);
        for (int s = 0; s < shapeCount; s++)
        {
            bool live = Core.Shapes.Handles.IsLive(s);
            w.WriteByte(live ? ShapeFlags[s] : (byte)0);
            w.WriteByte(live ? ShapeLayer[s] : (byte)0);
        }
    }

    private sealed class ServiceBlock
    {
        public uint[] Matrix = NewMatrix();
        public float Hit = 1f;
        public bool Dirty;
        public byte[] Flags = Array.Empty<byte>();
        public byte[] Layers = Array.Empty<byte>();
    }

    private static ServiceBlock ReadServices(StateReader r, int shapeCount, bool[] live)
    {
        var b = new ServiceBlock();
        r.ReadUInt32s(b.Matrix);
        b.Hit = r.ReadSingle();
        if (!(b.Hit >= 0f) || !float.IsFinite(b.Hit)) { throw r.Error("invalid hit speed threshold"); }
        b.Dirty = r.ReadBool();
        b.Flags = new byte[shapeCount];
        b.Layers = new byte[shapeCount];
        for (int s = 0; s < shapeCount; s++)
        {
            b.Flags[s] = r.ReadByte();
            b.Layers[s] = r.ReadByte();
            if ((b.Flags[s] & ~FlagSensor) != 0 || b.Layers[s] >= LayerCount) { throw r.Error("invalid shape flags or layer"); }
            if (!live[s] && (b.Flags[s] != 0 || b.Layers[s] != 0)) { throw r.Error("service state on a dead shape slot"); }
        }

        return b;
    }

    private void CommitServices(ServiceBlock? b, int shapeCount)
    {
        if (b == null)
        {
            LayerMatrix = NewMatrix();
            Opt.HitSpeedThreshold = 1f;
            FilterDirty = false;
            ShapeFlags = new byte[Math.Max(shapeCount, 1)];
            ShapeLayer = new byte[Math.Max(shapeCount, 1)];
        }
        else
        {
            LayerMatrix = b.Matrix;
            Opt.HitSpeedThreshold = b.Hit;
            FilterDirty = b.Dirty;
            ShapeFlags = b.Flags.Length > 0 ? b.Flags : new byte[1];
            ShapeLayer = b.Layers.Length > 0 ? b.Layers : new byte[1];
        }

        ResetSeenFilters();
    }

    // ------------------------------------------------------------------ mutation used by the handles

    public void SetLayerCollision(int a, int b, bool collide)
    {
        CheckLayer(a, nameof(a)); CheckLayer(b, nameof(b));
        uint ba = 1u << b, bb = 1u << a;
        if (collide) { LayerMatrix[a] |= ba; LayerMatrix[b] |= bb; } else { LayerMatrix[a] &= ~ba; LayerMatrix[b] &= ~bb; }
        FilterDirty = true;
    }

    public bool GetLayerCollision(int a, int b)
    {
        CheckLayer(a, nameof(a)); CheckLayer(b, nameof(b));
        return ((LayerMatrix[a] >> b) & 1u) != 0;
    }

    public static void CheckLayer(int layer, string name)
    {
        if ((uint)layer >= LayerCount) { throw new ArgumentOutOfRangeException(name, layer, "A layer must be within 0..31."); }
    }
}
