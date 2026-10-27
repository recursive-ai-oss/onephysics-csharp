using System;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics2D;

public readonly partial struct Body
{
    /// <summary>Raised after a step, in event order, when a contact's approach speed exceeded <see cref="WorldOptions2D.HitSpeedThreshold"/>. Recorded only when <see cref="WorldOptions2D.Events"/> includes <c>EventMask.ContactHit</c>. <see cref="Collision.ApproachSpeed"/> holds the speed.</summary>
    public event Action<Collision> OnCollisionHit
    {
        add { Subscribe(PhysicsWorld2D.KindHit, value); }
        remove { Unsubscribe(PhysicsWorld2D.KindHit, value); }
    }

    /// <summary>Raised after a step when a shape of this body starts overlapping a sensor, or when a sensor of this body starts overlapping another shape. Once per overlap, in event order.</summary>
    public event Action<Trigger> OnTriggerEnter
    {
        add { SubscribeTrigger(PhysicsWorld2D.KindTriggerEnter, value); }
        remove { UnsubscribeTrigger(PhysicsWorld2D.KindTriggerEnter, value); }
    }

    /// <summary>Raised after a step when a sensor overlap of this body ended, including when the other body or shape was destroyed. Once per overlap.</summary>
    public event Action<Trigger> OnTriggerExit
    {
        add { SubscribeTrigger(PhysicsWorld2D.KindTriggerExit, value); }
        remove { UnsubscribeTrigger(PhysicsWorld2D.KindTriggerExit, value); }
    }

    /// <summary>Raised after a step in which this body fell asleep.</summary>
    public event Action<Body> OnSleep
    {
        add { SubscribeBody(PhysicsWorld2D.KindSleep, value); }
        remove { UnsubscribeBody(PhysicsWorld2D.KindSleep, value); }
    }

    /// <summary>Raised after a step in which this body woke up (touched, woken by the host or by the step).</summary>
    public event Action<Body> OnWake
    {
        add { SubscribeBody(PhysicsWorld2D.KindWake, value); }
        remove { UnsubscribeBody(PhysicsWorld2D.KindWake, value); }
    }

    private void SubscribeTrigger(int kind, Action<Trigger> handler) => SubscribeAny(kind, handler);

    private void UnsubscribeTrigger(int kind, Action<Trigger> handler) => UnsubscribeAny(kind, handler);

    private void SubscribeBody(int kind, Action<Body> handler) => SubscribeAny(kind, handler);

    private void UnsubscribeBody(int kind, Action<Body> handler) => UnsubscribeAny(kind, handler);

    private void SubscribeAny(int kind, Delegate handler)
    {
        ArgumentNullException.ThrowIfNull(handler);
        if (_world == null) { throw new InvalidHandleException("The default Body is not attached to a world."); }
        _world.AddHandler(_handle, kind, handler);
    }

    private void UnsubscribeAny(int kind, Delegate handler)
    {
        if (_world == null || handler == null) { return; }
        _world.RemoveHandler(_handle, kind, handler);
    }

    /// <summary>
    /// The layer (0..31) of the body's first shape; setting it moves every shape of the body to that layer (Unity's per-object layer). Layers feed the world's layer collision matrix and query layer masks;
    /// a change takes effect at the next step. Default 0.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">The layer is outside 0..31.</exception>
    public int Layer
    {
        get
        {
            var (w, b) = Resolve();
            int s = w.Engine.ShapeHead[b];
            return s < 0 ? 0 : w.Engine.ShapeLayer[s];
        }
        set
        {
            var (w, b) = Resolve();
            Engine2D.CheckLayer(value, nameof(value));
            for (int s = w.Engine.ShapeHead[b]; s >= 0; s = w.Engine.NextShape[s]) { w.Engine.ShapeLayer[s] = (byte)value; }
            w.Engine.FilterDirty = true;
        }
    }
}

public readonly partial struct Shape
{
    /// <summary>The shape's layer, 0..31. A change takes effect at the next step. Default 0.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The layer is outside 0..31.</exception>
    public int Layer
    {
        get { int s = Resolve(); return _world!.Engine.ShapeLayer[s]; }
        set
        {
            int s = Resolve();
            Engine2D.CheckLayer(value, nameof(value));
            _world!.Engine.ShapeLayer[s] = (byte)value;
            _world.Engine.FilterDirty = true;
        }
    }

    /// <summary>True for a sensor (trigger) shape; fixed at creation (<see cref="ShapeDef.IsSensor"/>).</summary>
    public bool IsSensor { get { int s = Resolve(); return _world!.Engine.IsSensor(s); } }

    /// <summary>The categories this shape belongs to (bit set). A change takes effect at the next step.</summary>
    public uint CategoryBits
    {
        get { int s = Resolve(); return _world!.Engine.Category[s]; }
        set { int s = Resolve(); _world!.Engine.Category[s] = value; _world.Engine.FilterDirty = true; }
    }

    /// <summary>The categories this shape collides with; two shapes collide when each one's mask includes the other's category. A change takes effect at the next step.</summary>
    public uint MaskBits
    {
        get { int s = Resolve(); return _world!.Engine.Mask[s]; }
        set { int s = Resolve(); _world!.Engine.Mask[s] = value; _world.Engine.FilterDirty = true; }
    }

    /// <summary>Group index: shapes sharing a non-zero group always collide when it is positive and never when it is negative, overriding category and mask. A change takes effect at the next step.</summary>
    public int GroupIndex
    {
        get { int s = Resolve(); return _world!.Engine.Group[s]; }
        set { int s = Resolve(); _world!.Engine.Group[s] = value; _world.Engine.FilterDirty = true; }
    }
}
