using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D : ICallbackGate
{
    private readonly Dictionary<int, Handlers> _handlers = new();

    internal sealed class Handlers
    {
        public int Generation;
        public Action<Collision>? Enter, Stay, Exit, Hit;
        public Action<Trigger>? TriggerEnter, TriggerExit;
        public Action<Body>? Sleep, Wake;

        public bool IsEmpty => Enter == null && Stay == null && Exit == null && Hit == null && TriggerEnter == null && TriggerExit == null && Sleep == null && Wake == null;
    }

    internal const int KindEnter = 0, KindStay = 1, KindExit = 2, KindHit = 3, KindTriggerEnter = 4, KindTriggerExit = 5, KindSleep = 6, KindWake = 7;

    // ------------------------------------------------------------------ callbacks across restore and resimulation

    // A callback belongs to a handle (index and generation). Destroying the body, joint or character takes its callbacks out of use; once the world has taken a snapshot it also remembers the most recent
    // ones, so a restore that brings the same handle back gives them back. Older ones are forgotten (see HandlerGraveyard).
    private readonly HandlerGraveyard<Handlers> _deadHandlers = new();
    private readonly HandlerGraveyard<JointHandlers> _deadJointHandlers = new();
    private readonly HandlerGraveyard<Action<ControllerColliderHit>> _deadControllerHandlers = new();
    private bool _remembersDestroyed;
    private int _suppressed;

    /// <summary>
    /// Starts a scope in which no per-body, per-joint or character callback runs (collision, trigger, sleep, wake, joint-break and controller-hit handlers). Events are still produced and polled the same way,
    /// so a rollback host does <c>Restore</c> and then resimulates under <c>using (world.SuppressCallbacks())</c> without replaying sounds and particles. Allocates nothing; scopes nest; dispose it once.
    /// </summary>
    public CallbackSuppression SuppressCallbacks()
    {
        _suppressed++;
        return new CallbackSuppression(this);
    }

    void ICallbackGate.EndSuppression()
    {
        if (_suppressed > 0) { _suppressed--; }
    }

    internal void RetireBodyHandlers(BodyHandle h)
    {
        if (_handlers.Remove(h.Index, out var set) && _remembersDestroyed && set.Generation == h.Generation) { _deadHandlers.Add(h.Index, h.Generation, set); }
    }

    private void RetireJointHandlers(int index, int generation)
    {
        if (_jointHandlers.Remove(index, out var set) && _remembersDestroyed && set.Generation == generation) { _deadJointHandlers.Add(index, generation, set); }
    }

    /// <summary>With callbacks suppressed a broken joint's handlers are not run, but they must not stay keyed to the destroyed joint either.</summary>
    private void RetireBrokenJointHandlers()
    {
        foreach (var e in _core.Events.Delivered)
        {
            if (e.Kind == EventKind.JointBreak) { RetireJointHandlers(e.Joint.Index, e.Joint.Generation); }
        }
    }

    /// <summary>After a restore: callbacks of handles that no longer exist are set aside, and set-aside ones whose handle exists again are given back.</summary>
    private void ReconcileHandlersAfterRestore()
    {
        var dead = _deadScratch;
        dead.Clear();
        foreach (var kv in _handlers)
        {
            if (!_core.Bodies.Handles.IsAlive(kv.Key, kv.Value.Generation)) { dead.Add(kv.Key); }
        }

        foreach (int k in dead)
        {
            var set = _handlers[k];
            _handlers.Remove(k);
            if (_remembersDestroyed) { _deadHandlers.Add(k, set.Generation, set); }
        }

        for (int i = _deadHandlers.Count - 1; i >= 0; i--)
        {
            var e = _deadHandlers[i];
            if (!_core.Bodies.Handles.IsAlive(e.Index, e.Generation)) { continue; }
            _deadHandlers.RemoveAt(i);
            if (!_handlers.TryGetValue(e.Index, out var current) || current.Generation != e.Generation) { _handlers[e.Index] = e.Set; }
        }

        dead.Clear();
        foreach (var kv in _jointHandlers)
        {
            if (!_core.Constraints.Handles.IsAlive(kv.Key, kv.Value.Generation)) { dead.Add(kv.Key); }
        }

        foreach (int k in dead)
        {
            var set = _jointHandlers[k];
            _jointHandlers.Remove(k);
            if (_remembersDestroyed) { _deadJointHandlers.Add(k, set.Generation, set); }
        }

        for (int i = _deadJointHandlers.Count - 1; i >= 0; i--)
        {
            var e = _deadJointHandlers[i];
            if (!_core.Constraints.Handles.IsAlive(e.Index, e.Generation)) { continue; }
            _deadJointHandlers.RemoveAt(i);
            if (!_jointHandlers.TryGetValue(e.Index, out var current) || current.Generation != e.Generation) { _jointHandlers[e.Index] = e.Set; }
        }

        RetireDeadControllerHandlers();
        for (int i = _deadControllerHandlers.Count - 1; i >= 0; i--)
        {
            var e = _deadControllerHandlers[i];
            if (!_engine.CharHandles.IsAlive(e.Index, e.Generation)) { continue; }
            _deadControllerHandlers.RemoveAt(i);
            if (!_controllerHandlers.TryGetValue(e.Index, out var current) || current.Generation != e.Generation) { _controllerHandlers[e.Index] = (e.Generation, e.Set); }
        }
    }

    // ------------------------------------------------------------------ polled events (middle tier)

    /// <summary>The delivered events of one kind, a slice of <see cref="Events"/> (events are sorted by kind). Valid until the next <see cref="Step"/>.</summary>
    public ReadOnlySpan<PhysicsEvent> GetEvents(EventKind kind)
    {
        var all = _core.Events.Delivered;
        int lo = 0;
        while (lo < all.Length && all[lo].Kind < kind) { lo++; }
        int hi = lo;
        while (hi < all.Length && all[hi].Kind == kind) { hi++; }
        return all.Slice(lo, hi - lo);
    }

    /// <summary>
    /// Resolves a contact begin, persist, end or hit event into the same <see cref="Collision"/> a callback receives, seen from body A. Points and impulses are those at the end of the step the event was delivered in:
    /// an end event, or an event for a contact that has gone since, has no points.
    /// </summary>
    /// <exception cref="ArgumentException">The event is not a contact event.</exception>
    public Collision GetCollision(in PhysicsEvent e)
    {
        if (e.Kind != EventKind.ContactBegin && e.Kind != EventKind.ContactPersist && e.Kind != EventKind.ContactEnd && e.Kind != EventKind.ContactHit)
        {
            throw new ArgumentException("A " + e.Kind + " event is not a contact event.", nameof(e));
        }

        return MakeCollision(e, e.BodyA, e.BodyB, e.ShapeA, e.ShapeB, false);
    }

    /// <summary>Resolves a sensor enter or exit event into a <see cref="Trigger"/> seen from body A.</summary>
    /// <exception cref="ArgumentException">The event is not a sensor event.</exception>
    public Trigger GetTrigger(in PhysicsEvent e)
    {
        if (e.Kind != EventKind.SensorEnter && e.Kind != EventKind.SensorExit) { throw new ArgumentException("A " + e.Kind + " event is not a sensor event.", nameof(e)); }
        return MakeTrigger(e, e.BodyA, e.BodyB, e.ShapeA, e.ShapeB);
    }

    // ------------------------------------------------------------------ delivery after a step

    private void DispatchEvents()
    {
        foreach (var e in _core.Events.Delivered)
        {
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
        Vector2F normal = default;
        int count = 0;
        ContactPoint2D p0 = default, p1 = default;
        if (e.Kind != EventKind.ContactEnd && _core.Constraints.HighWater > 0 && _core.Shapes.Handles.IsAlive(e.ShapeA.Index, e.ShapeA.Generation) && _core.Shapes.Handles.IsAlive(e.ShapeB.Index, e.ShapeB.Generation))
        {
            int c = _engine.ContactOf(e.ShapeA.Index, e.ShapeB.Index);
            if (c >= 0 && _core.Constraints.KindOf(c) == ConstraintKind.Contact && _core.Constraints.PointCount[c] > 0)
            {
                var cs = _core.Constraints;
                var pl = cs.Payload(c);
                var posA = _core.Bodies.PositionOf(cs.BodyA[c]);
                count = cs.PointCount[c];
                normal = flip ? new Vector2F(-pl[ManifoldLayout.Normal], -pl[ManifoldLayout.Normal + 1]) : new Vector2F(pl[ManifoldLayout.Normal], pl[ManifoldLayout.Normal + 1]);
                float scale = _core.Options.Substeps;
                for (int k = 0; k < count; k++)
                {
                    int a = ManifoldLayout.AnchorA(k);
                    var cp = new ContactPoint2D(new Vector2D(posA[0] + (double)pl[a], posA[1] + (double)pl[a + 1]), pl[ManifoldLayout.Separation(k)],
                        scale * pl[ManifoldLayout.NormalImpulse(k)], scale * pl[ManifoldLayout.Tangent1Impulse(k)]);
                    if (k == 0) { p0 = cp; } else { p1 = cp; }
                }
            }
        }

        return new Collision(new Body(this, self), new Body(this, other), new Shape(this, selfShape), new Shape(this, otherShape), normal, count, in p0, in p1, e.Kind == EventKind.ContactHit ? e.Value : 0f);
    }

    private void FireTrigger(in PhysicsEvent e, BodyHandle self, BodyHandle other, ShapeHandle selfShape, ShapeHandle otherShape)
    {
        if (!_handlers.TryGetValue(self.Index, out var h) || h.Generation != self.Generation) { return; }
        var action = e.Kind == EventKind.SensorEnter ? h.TriggerEnter : h.TriggerExit;
        if (action == null) { return; }
        action(MakeTrigger(e, self, other, selfShape, otherShape));
    }

    private Trigger MakeTrigger(in PhysicsEvent e, BodyHandle self, BodyHandle other, ShapeHandle selfShape, ShapeHandle otherShape)
    {
        // The sensor is the shape flagged as such; for a destroyed shape the flag is gone, so fall back on shape A (the constraint's first shape).
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

    // ------------------------------------------------------------------ filters (see docs/world2d.md, "Layers and filters")

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

    /// <summary>
    /// Re-evaluates every pair at the start of the next step. Changes made through the world, shapes and <see cref="WorldOptions2D.PairFilter"/> are noticed on their own; call this when a delegate or filter
    /// now answers differently for the same shapes (for example after restoring a snapshot into a world whose delegates were installed by the host).
    /// </summary>
    public void RefilterPairs() => _engine.FilterDirty = true;
}
