using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
{
    private readonly Dictionary<int, Handlers> _handlers = new();
    private PhysicsEvent[] _publicEvents = new PhysicsEvent[16];
    private int _publicEventCount;
    private readonly Dictionary<(int, int, int, int), int> _pairIndex = new();
    private PairAgg[] _aggs = new PairAgg[16];

    internal sealed class Handlers
    {
        public int Generation;
        public Action<Collision>? Enter, Stay, Exit, Hit;
        public Action<Trigger>? TriggerEnter, TriggerExit;
        public Action<Body>? Sleep, Wake;

        public bool IsEmpty => Enter == null && Stay == null && Exit == null && Hit == null && TriggerEnter == null && TriggerExit == null && Sleep == null && Wake == null;
    }

    internal const int KindEnter = 0, KindStay = 1, KindExit = 2, KindHit = 3, KindTriggerEnter = 4, KindTriggerExit = 5, KindSleep = 6, KindWake = 7;

    private struct PairAgg
    {
        public int Begins, Ends;
        public bool BeginDone, EndDone, PersistDone, HitDone;
        public float Hit;
        public int Chain;
    }

    // ------------------------------------------------------------------ events: one per pair of shapes

    /// <summary>
    /// Builds <see cref="Events"/> from the core's delivered list. The core reports one contact event per sub-shape pair (a body over N mesh triangles has N constraints); the world reports one per pair of
    /// shapes: a begin when the pair had no contact constraint at the start of the step and has one now, an end for the opposite, one persist and one hit (the largest approach speed) per pair.
    /// The order is the core's order with the dropped events removed. Sensor, joint, sleep and wake events pass through unchanged.
    /// </summary>
    private void BuildPublicEvents()
    {
        var all = _core.Events.Delivered;
        _publicEventCount = 0;
        if (all.Length == 0) { return; }
        if (_publicEvents.Length < all.Length) { _publicEvents = new PhysicsEvent[Math.Max(all.Length, _publicEvents.Length * 2)]; }
        _pairIndex.Clear();
        int n = 0;
        foreach (var e in all)
        {
            if (!IsContactKind(e.Kind)) { continue; }
            var key = (e.ShapeA.Index, e.ShapeA.Generation, e.ShapeB.Index, e.ShapeB.Generation);
            if (!_pairIndex.TryGetValue(key, out int idx))
            {
                idx = _pairIndex.Count;
                _pairIndex[key] = idx;
                if (_aggs.Length <= idx) { Array.Resize(ref _aggs, _aggs.Length * 2); }
                _aggs[idx] = new PairAgg { Chain = ChainLength(e) };
            }

            ref PairAgg a = ref _aggs[idx];
            if (e.Kind == EventKind.ContactBegin) { a.Begins++; }
            else if (e.Kind == EventKind.ContactEnd) { a.Ends++; }
            else if (e.Kind == EventKind.ContactHit && e.Value > a.Hit) { a.Hit = e.Value; }
        }

        foreach (var e in all)
        {
            if (!IsContactKind(e.Kind)) { _publicEvents[n++] = e; continue; }
            ref PairAgg a = ref _aggs[_pairIndex[(e.ShapeA.Index, e.ShapeA.Generation, e.ShapeB.Index, e.ShapeB.Generation)]];
            bool after = a.Chain > 0;
            bool before = a.Chain - a.Begins + a.Ends > 0;
            switch (e.Kind)
            {
                case EventKind.ContactBegin:
                    if (!a.BeginDone && !before && after) { _publicEvents[n++] = e; }
                    a.BeginDone = true;
                    break;
                case EventKind.ContactEnd:
                    if (!a.EndDone && before && !after) { _publicEvents[n++] = e; }
                    a.EndDone = true;
                    break;
                case EventKind.ContactPersist:
                    if (!a.PersistDone) { _publicEvents[n++] = e; }
                    a.PersistDone = true;
                    break;
                default:
                    if (!a.HitDone) { _publicEvents[n++] = new PhysicsEvent(e.Kind, e.BodyA, e.BodyB, e.ShapeA, e.ShapeB, e.SubShapeA, e.SubShapeB, e.Joint, a.Hit); }
                    a.HitDone = true;
                    break;
            }
        }

        _publicEventCount = n;
    }

    private static bool IsContactKind(EventKind k) => k == EventKind.ContactBegin || k == EventKind.ContactPersist || k == EventKind.ContactEnd || k == EventKind.ContactHit;

    /// <summary>Contact constraints the pair of shapes has now (0 when either shape is gone).</summary>
    private int ChainLength(in PhysicsEvent e)
    {
        var shapes = _core.Shapes.Handles;
        if (!shapes.IsAlive(e.ShapeA.Index, e.ShapeA.Generation) || !shapes.IsAlive(e.ShapeB.Index, e.ShapeB.Generation)) { return 0; }
        var cs = _core.Constraints;
        int n = 0;
        for (int c = _engine.ContactOf(e.ShapeA.Index, e.ShapeB.Index); c >= 0 && c < cs.HighWater && cs.IsLive(c) && cs.KindOf(c) == ConstraintKind.Contact && n <= cs.HighWater; c = _engine.ConsNext[c]) { n++; }
        return n;
    }

    // ------------------------------------------------------------------ polled events (middle tier)

    /// <summary>The delivered events of one kind, a slice of <see cref="Events"/> (events are sorted by kind). Valid until the next <see cref="Step"/>.</summary>
    public ReadOnlySpan<PhysicsEvent> GetEvents(EventKind kind)
    {
        var all = Events;
        int lo = 0;
        while (lo < all.Length && all[lo].Kind < kind) { lo++; }
        int hi = lo;
        while (hi < all.Length && all[hi].Kind == kind) { hi++; }
        return all.Slice(lo, hi - lo);
    }

    /// <summary>Resolves a contact begin, persist, end or hit event into the <see cref="Collision"/> a callback receives, seen from body A. An end event, or an event for a contact that has gone since, has no points.</summary>
    /// <exception cref="ArgumentException">The event is not a contact event.</exception>
    public Collision GetCollision(in PhysicsEvent e)
    {
        if (!IsContactKind(e.Kind)) { throw new ArgumentException("A " + e.Kind + " event is not a contact event.", nameof(e)); }
        return MakeCollision(e, e.BodyA, e.BodyB, e.ShapeA, e.ShapeB, false);
    }

    /// <summary>Resolves a sensor enter or exit event into a <see cref="Trigger"/> seen from body A.</summary>
    /// <exception cref="ArgumentException">The event is not a sensor event.</exception>
    public Trigger GetTrigger(in PhysicsEvent e)
    {
        if (e.Kind != EventKind.SensorEnter && e.Kind != EventKind.SensorExit) { throw new ArgumentException("A " + e.Kind + " event is not a sensor event.", nameof(e)); }
        return MakeTrigger(e.BodyA, e.BodyB, e.ShapeA, e.ShapeB);
    }

    // ------------------------------------------------------------------ delivery after a step

    private void DispatchEvents()
    {
        for (int i = 0; i < _publicEventCount; i++)
        {
            var e = _publicEvents[i];
            switch (e.Kind)
            {
                case EventKind.ContactBegin:
                case EventKind.ContactPersist:
                case EventKind.ContactEnd:
                case EventKind.ContactHit:
                    Fire(e, e.BodyA, e.BodyB, e.ShapeA, e.ShapeB, false);
                    Fire(e, e.BodyB, e.BodyA, e.ShapeB, e.ShapeA, true);
                    break;
                case EventKind.SensorEnter:
                case EventKind.SensorExit:
                    FireTrigger(e, e.BodyA, e.BodyB, e.ShapeA, e.ShapeB);
                    FireTrigger(e, e.BodyB, e.BodyA, e.ShapeB, e.ShapeA);
                    break;
                case EventKind.JointBreak:
                    FireJointBreak(e);
                    break;
                case EventKind.Sleep:
                case EventKind.Wake:
                    if (_handlers.TryGetValue(e.BodyA.Index, out var h) && h.Generation == e.BodyA.Generation)
                    {
                        var action = e.Kind == EventKind.Sleep ? h.Sleep : h.Wake;
                        action?.Invoke(new Body(this, e.BodyA));
                    }

                    break;
            }
        }
    }

    private void Fire(in PhysicsEvent e, BodyHandle self, BodyHandle other, ShapeHandle selfShape, ShapeHandle otherShape, bool flip)
    {
        if (!_handlers.TryGetValue(self.Index, out var h) || h.Generation != self.Generation) { return; }
        var action = e.Kind == EventKind.ContactBegin ? h.Enter : e.Kind == EventKind.ContactPersist ? h.Stay : e.Kind == EventKind.ContactHit ? h.Hit : h.Exit;
        if (action == null) { return; }
        action(MakeCollision(e, self, other, selfShape, otherShape, flip));
    }

    private Collision MakeCollision(in PhysicsEvent e, BodyHandle self, BodyHandle other, ShapeHandle selfShape, ShapeHandle otherShape, bool flip)
    {
        Vector3F normal = default;
        int count = 0;
        ContactPoint3D p0 = default, p1 = default, p2 = default, p3 = default;
        var shapes = _core.Shapes.Handles;
        if (e.Kind != EventKind.ContactEnd && _core.Constraints.HighWater > 0 && shapes.IsAlive(e.ShapeA.Index, e.ShapeA.Generation) && shapes.IsAlive(e.ShapeB.Index, e.ShapeB.Generation))
        {
            int c = _engine.ContactOf(e.ShapeA.Index, e.ShapeB.Index);
            var cs = _core.Constraints;
            if (c >= 0 && c < cs.HighWater && cs.IsLive(c) && cs.KindOf(c) == ConstraintKind.Contact && cs.PointCount[c] > 0)
            {
                var pl = cs.Payload(c);
                var posA = _core.Bodies.PositionOf(cs.BodyA[c]);
                count = cs.PointCount[c];
                var n = new Vector3F(pl[ManifoldLayout.Normal], pl[ManifoldLayout.Normal + 1], pl[ManifoldLayout.Normal + 2]);
                normal = flip ? -n : n;
                float scale = _core.Options.Substeps;
                for (int k = 0; k < count; k++)
                {
                    int a = ManifoldLayout.AnchorA(k);
                    float t1 = pl[ManifoldLayout.Tangent1Impulse(k)], t2 = pl[ManifoldLayout.Tangent2Impulse(k)];
                    var cp = new ContactPoint3D(new Vector3D(posA[0] + (double)pl[a], posA[1] + (double)pl[a + 1], posA[2] + (double)pl[a + 2]), pl[ManifoldLayout.Separation(k)],
                        scale * pl[ManifoldLayout.NormalImpulse(k)], scale * float.Sqrt(t1 * t1 + t2 * t2));
                    if (k == 0) { p0 = cp; } else if (k == 1) { p1 = cp; } else if (k == 2) { p2 = cp; } else { p3 = cp; }
                }
            }
        }

        return new Collision(new Body(this, self), new Body(this, other), new Shape(this, selfShape), new Shape(this, otherShape), normal, count, in p0, in p1, in p2, in p3, e.Kind == EventKind.ContactHit ? e.Value : 0f);
    }

    private void FireTrigger(in PhysicsEvent e, BodyHandle self, BodyHandle other, ShapeHandle selfShape, ShapeHandle otherShape)
    {
        if (!_handlers.TryGetValue(self.Index, out var h) || h.Generation != self.Generation) { return; }
        var action = e.Kind == EventKind.SensorEnter ? h.TriggerEnter : h.TriggerExit;
        if (action == null) { return; }
        action(MakeTrigger(self, other, selfShape, otherShape));
    }

    private Trigger MakeTrigger(BodyHandle self, BodyHandle other, ShapeHandle selfShape, ShapeHandle otherShape)
    {
        // The sensor is the shape flagged as such; for a destroyed shape the flag is gone, so the answer is false for it.
        bool selfSensor = _core.Shapes.Handles.IsAlive(selfShape.Index, selfShape.Generation) && _engine.IsSensor(selfShape.Index);
        return new Trigger(new Body(this, self), new Body(this, other), new Shape(this, selfShape), new Shape(this, otherShape), selfSensor);
    }

    // ------------------------------------------------------------------ handler registry

    internal void AddHandler(BodyHandle h, int kind, Delegate handler)
    {
        _core.Resolve(h);
        if (!_handlers.TryGetValue(h.Index, out var set) || set.Generation != h.Generation)
        {
            set = new Handlers { Generation = h.Generation };
            _handlers[h.Index] = set;
        }

        switch (kind)
        {
            case KindEnter: set.Enter += (Action<Collision>)handler; break;
            case KindStay: set.Stay += (Action<Collision>)handler; break;
            case KindExit: set.Exit += (Action<Collision>)handler; break;
            case KindHit: set.Hit += (Action<Collision>)handler; break;
            case KindTriggerEnter: set.TriggerEnter += (Action<Trigger>)handler; break;
            case KindTriggerExit: set.TriggerExit += (Action<Trigger>)handler; break;
            case KindSleep: set.Sleep += (Action<Body>)handler; break;
            default: set.Wake += (Action<Body>)handler; break;
        }
    }

    internal void RemoveHandler(BodyHandle h, int kind, Delegate handler)
    {
        if (!_handlers.TryGetValue(h.Index, out var set) || set.Generation != h.Generation) { return; }
        switch (kind)
        {
            case KindEnter: set.Enter -= (Action<Collision>)handler; break;
            case KindStay: set.Stay -= (Action<Collision>)handler; break;
            case KindExit: set.Exit -= (Action<Collision>)handler; break;
            case KindHit: set.Hit -= (Action<Collision>)handler; break;
            case KindTriggerEnter: set.TriggerEnter -= (Action<Trigger>)handler; break;
            case KindTriggerExit: set.TriggerExit -= (Action<Trigger>)handler; break;
            case KindSleep: set.Sleep -= (Action<Body>)handler; break;
            default: set.Wake -= (Action<Body>)handler; break;
        }

        if (set.IsEmpty) { _handlers.Remove(h.Index); }
    }

    // ------------------------------------------------------------------ filters (see docs/world3d.md, "Layers, filters and sensors")

    /// <summary>
    /// Sets whether shapes on layer <paramref name="layerA"/> collide with shapes on layer <paramref name="layerB"/> (Unity's layer collision matrix; symmetric; default: all collide).
    /// Existing pairs are re-evaluated at the start of the next step: pairs that may no longer collide end (end events), newly allowed ones begin.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">A layer is outside 0..31.</exception>
    public void SetLayerCollision(int layerA, int layerB, bool collide) => _engine.SetLayerCollision(layerA, layerB, collide);

    /// <summary>Whether layer <paramref name="layerA"/> collides with layer <paramref name="layerB"/>.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A layer is outside 0..31.</exception>
    public bool GetLayerCollision(int layerA, int layerB) => _engine.GetLayerCollision(layerA, layerB);

    /// <summary>Unity's name for <c>SetLayerCollision(a, b, !ignore)</c>.</summary>
    public void IgnoreLayerCollision(int layerA, int layerB, bool ignore = true) => _engine.SetLayerCollision(layerA, layerB, !ignore);

    /// <summary>Installs a struct contact filter (middle tier), replacing any earlier one. Pairs are re-evaluated at the next step. The filter is not saved in snapshots: install it again after <see cref="Restore(byte[])"/>.</summary>
    public void SetContactFilter<TFilter>(TFilter filter) where TFilter : struct, IContactFilter
    {
        _engine.ContactFilter = new ContactFilterHolder<TFilter>(filter);
    }

    /// <summary>Removes the struct contact filter; pairs are re-evaluated at the next step.</summary>
    public void ClearContactFilter() => _engine.ContactFilter = null;

    /// <summary>Re-evaluates every pair at the start of the next step; call it when a delegate or filter now answers differently for the same shapes (for example after a restore).</summary>
    public void RefilterPairs() => _engine.FilterDirty = true;
}
