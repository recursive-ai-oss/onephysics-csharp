using System;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>Decides whether two shapes may collide, as a struct so the call is specialized and allocation-free. Middle tier only: the happy path uses the layer matrix and <see cref="PairPredicate"/>.</summary>
/// <remarks>Install one with <see cref="PhysicsWorld3D.SetContactFilter{TFilter}"/>. It runs serially, once when a pair of overlapping shapes is first found and again when filters are re-evaluated, so it must be a pure function of the shapes' properties.</remarks>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
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

internal sealed partial class Engine3D
{
    public const int LayerCount = 32;
    private const uint BlockServices = 2;
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
    /// The pure part of the collision rule, safe to call from the parallel pair search: no pair of sensors, then the layer matrix, then the category/mask/group rule
    /// (a shared non-zero group decides: positive collides, negative never).
    /// </summary>
    public bool CanCollide(int sa, int sb)
    {
        if ((ShapeFlags[sa] & ShapeFlags[sb] & FlagSensor) != 0) { return false; }
        if (_virtualCharacters != 0 && CharacterPairBlocked(sa, sb)) { return false; }
        if (((LayerMatrix[ShapeLayer[sa]] >> ShapeLayer[sb]) & 1u) == 0) { return false; }
        if (Group[sa] == Group[sb] && Group[sa] != 0) { return Group[sa] > 0; }
        return (Mask[sa] & Category[sb]) != 0 && (Category[sa] & Mask[sb]) != 0;
    }

    /// <summary>The whole rule: different bodies, <see cref="CanCollide"/>, joints without collide-connected, then the pair predicate and the struct contact filter (both serial).</summary>
    private bool PairAllowed(int a, int b)
    {
        var owner = Core.Shapes.Owner;
        if (owner[a] == owner[b] || !CanCollide(a, b) || JointsPreventCollision(owner[a], owner[b])) { return false; }
        var predicate = Opt.PairFilter;
        if (predicate != null && !predicate(WrapShape(a), WrapShape(b))) { return false; }
        var filter = ContactFilter;
        return filter == null || filter.Test(WrapShape(a), WrapShape(b));
    }

    /// <summary>
    /// Start of the contact stage. If a filter changed since the last step (shape category, mask, group or layer, the layer matrix, a new pair predicate or contact filter, or an explicit request),
    /// every pair in the table is evaluated again in ascending pair index: pairs that may no longer collide lose their constraints (end events) and leave the table, and every proxy is
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
            EndSensorOverlap(p);
            for (int c = PairCons[p]; c >= 0;)
            {
                int next = ConsNext[c];
                Core.RemoveConstraint(c);
                c = next;
            }

            FreePair(p);
        }

        Bp.TouchAll();
    }

    private void ResetSeenFilters()
    {
        _seenPairFilter = Opt.PairFilter;
        _seenContactFilter = ContactFilter;
    }

    // ------------------------------------------------------------------ sensors in the contact stage

    /// <summary>
    /// A sensor pair is marked while the shapes overlap (some manifold point with separation at most 0) and gives the enter and exit events. The mark lives in the pair table, not in a constraint,
    /// so a sensor never takes a constraint slot or a color that a contact could have had.
    /// </summary>
    private void ApplySensorPair(WorldCore world, int p, NarrowScratch sc, int first, int count, ReadOnlySpan<int> owner)
    {
        int sa = PairA[p], sb = PairB[p];
        bool overlap = false;
        for (int i = 0; i < count && !overlap; i++)
        {
            ref ContactOut o = ref sc.Out[first + i];
            for (int k = 0; k < o.Count; k++)
            {
                if (o.P[7 * k + 6] <= 0f) { overlap = true; break; }
            }
        }

        int c = PairCons[p];
        if (overlap && c == -1)
        {
            PairCons[p] = SensorOverlap;
            world.EmitSensorEvent(true, sa, sb);
        }
        else if (!overlap && c == SensorOverlap) { EndSensorOverlap(p); }
    }

    // ------------------------------------------------------------------ hit events

    /// <summary>
    /// Hit events: after the last substep, a contact whose fastest approach at the start of the step exceeded <see cref="WorldOptions3D.HitSpeedThreshold"/> and that pushed with a positive impulse reports it.
    /// Each chunk of constraints writes to its own event slot, so the delivered order does not depend on threads.
    /// </summary>
    public void EmitHits(WorldCore world, ReadOnlySpan<int> constraints, int eventSlot)
    {
        var events = world.Events;
        if (!events.Wants(EventKind.ContactHit)) { return; }
        var cs = world.Constraints;
        float threshold = Opt.HitSpeedThreshold;
        foreach (int c in constraints)
        {
            if (cs.KindOf(c) != ConstraintKind.Contact) { continue; }
            int slot = SlotOf[c];
            if (slot < 0) { continue; }
            // The relative normal velocity from the start of the step is still in the contact's wide group: the kernel only reads it.
            var w = Wide.AsSpan((slot >> 3) * W3D.GroupFloats, W3D.GroupFloats);
            int lane = slot & 7;
            var pl = cs.Payload(c);
            int count = cs.PointCount[c];
            float best = 0f;
            for (int i = 0; i < count; i++)
            {
                if (!(pl[ManifoldLayout.MaxNormalImpulse(i)] > 0f)) { continue; }
                float approach = 0f - w[(W3D.Point0 + i * W3D.PointStride + W3D.RelativeVelocity) * 8 + lane];
                if (approach > best) { best = approach; }
            }

            if (!(best > threshold)) { continue; }
            events.Emit(eventSlot, new PhysicsEvent(EventKind.ContactHit, world.HandleOfBody(cs.BodyA[c]), world.HandleOfBody(cs.BodyB[c]),
                new ShapeHandle(world.Id, cs.ShapeA[c], cs.ShapeGenerationA(c)), new ShapeHandle(world.Id, cs.ShapeB[c], cs.ShapeGenerationB(c)), cs.SubShapeA[c], cs.SubShapeB[c], default, best));
        }
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
        public int Count;
        public byte[] Flags = new byte[8];
        public byte[] Layers = new byte[8];

        public bool IsDefault()
        {
            if (Dirty || Hit != 1f) { return false; }
            foreach (uint row in Matrix) { if (row != uint.MaxValue) { return false; } }
            for (int i = 0; i < Count; i++) { if (Flags[i] != 0 || Layers[i] != 0) { return false; } }
            return true;
        }
    }

    private readonly ServiceBlock _svcStage = new();

    // Reads into the engine's staging block (kept between restores, so nothing is allocated once it has grown).
    private ServiceBlock ReadServices(StateReader r, int shapeCount, bool[] live)
    {
        var b = _svcStage;
        r.ReadUInt32s(b.Matrix);
        b.Hit = r.ReadSingle();
        if (!(b.Hit >= 0f) || !float.IsFinite(b.Hit)) { throw r.Error("invalid hit speed threshold"); }
        b.Dirty = r.ReadBool();
        b.Count = shapeCount;
        if (b.Flags.Length < shapeCount) { b.Flags = new byte[Math.Max(shapeCount, 8)]; b.Layers = new byte[Math.Max(shapeCount, 8)]; }
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
        int n = Math.Max(shapeCount, 1);
        if (b == null)
        {
            Array.Fill(LayerMatrix, uint.MaxValue);
            Opt.HitSpeedThreshold = 1f;
            FilterDirty = false;
            if (ShapeFlags.Length < n) { ShapeFlags = new byte[n]; ShapeLayer = new byte[n]; }
            Array.Clear(ShapeFlags);
            Array.Clear(ShapeLayer);
        }
        else
        {
            (LayerMatrix, b.Matrix) = (b.Matrix, LayerMatrix);
            Opt.HitSpeedThreshold = b.Hit;
            FilterDirty = b.Dirty;
            // Slots beyond the shape count hold older values; creating a shape sets its slot explicitly.
            (ShapeFlags, b.Flags) = (b.Flags, ShapeFlags);
            (ShapeLayer, b.Layers) = (b.Layers, ShapeLayer);
            if (ShapeFlags.Length < n) { ShapeFlags = new byte[n]; ShapeLayer = new byte[n]; }
        }

        ResetSeenFilters();
    }

    // ------------------------------------------------------------------ mutation used by the handles

    public void SetLayerCollision(int layerA, int layerB, bool collide)
    {
        CheckLayer(layerA, nameof(layerA)); CheckLayer(layerB, nameof(layerB));
        int a = layerA, b = layerB;
        uint ba = 1u << b, bb = 1u << a;
        if (collide) { LayerMatrix[a] |= ba; LayerMatrix[b] |= bb; } else { LayerMatrix[a] &= ~ba; LayerMatrix[b] &= ~bb; }
        FilterDirty = true;
    }

    public bool GetLayerCollision(int layerA, int layerB)
    {
        CheckLayer(layerA, nameof(layerA)); CheckLayer(layerB, nameof(layerB));
        int a = layerA, b = layerB;
        return ((LayerMatrix[a] >> b) & 1u) != 0;
    }

    public static void CheckLayer(int layer, string name)
    {
        if ((uint)layer >= LayerCount) { throw new ArgumentOutOfRangeException(name, layer, "A layer must be within 0..31."); }
    }
}
