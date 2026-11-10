using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>
/// A handle to a body: the world, a slot and a generation. It is a small struct that allocates nothing, is cheap to copy and compare, and fails with a
/// <see cref="StaleHandleException"/> naming the body once the body is destroyed (a later body reusing the slot has a different generation). The default value is the null handle: <see cref="IsValid"/> is false and using it throws.
/// </summary>
public readonly partial struct Body : IEquatable<Body>
{
    private readonly PhysicsWorld2D? _world;
    private readonly BodyHandle _handle;

    internal Body(PhysicsWorld2D world, BodyHandle handle)
    {
        _world = world;
        _handle = handle;
    }

    internal BodyHandle Handle => _handle;

    internal PhysicsWorld2D? World => _world;

    /// <summary>The raw core handle (world id, slot, generation) for the raw tier.</summary>
    public BodyHandle RawHandle => _handle;

    private (PhysicsWorld2D World, int Slot) Resolve()
    {
        if (_world == null) { throw new InvalidHandleException("The default Body is not attached to a world."); }
        return (_world, _world.ResolveBody(_handle));
    }

    /// <summary>True while the body exists (not destroyed, not the default value).</summary>
    public bool IsValid => _world != null && _world.Core.Bodies.Handles.IsAlive(_handle.Index, _handle.Generation) && _handle.World == _world.Id;

    /// <summary>Static, kinematic or dynamic.</summary>
    public BodyType Type { get { var (w, b) = Resolve(); return w.Core.Bodies.TypeOf(b); } }

    /// <summary>True for a kinematic body (moves at its velocity, is not pushed by contacts).</summary>
    public bool IsKinematic => Type == BodyType.Kinematic;

    /// <summary>True for a static body.</summary>
    public bool IsStatic => Type == BodyType.Static;

    /// <summary>True while the body is awake. Static bodies are never awake; kinematic bodies always are.</summary>
    public bool IsAwake { get { var (w, b) = Resolve(); return w.Core.Bodies.TypeOf(b) == BodyType.Kinematic || (w.Core.Bodies.TypeOf(b) == BodyType.Dynamic && w.Core.Bodies.IsAwake(b)); } }

    /// <summary>Body origin in metres relative to the world origin. Setting accepts finite components in inclusive ±2^21, otherwise throws ArgumentOutOfRangeException. All Q31.32 values round-trip exactly through double on that closed interval; OneMath FixedBridge accepts only the open interior. Simulation may advance to ±2^22 before Step rejects out-of-bounds state.</summary>
    public Vector2D Position
    {
        get
        {
            var (w, b) = Resolve();
            var bodies = w.Core.Bodies;
            var p = bodies.PositionOf(b);
            var r = bodies.RotationOf(b);
            var e = w.Engine;
            float lx = e.LocalCenter[2 * b], ly = e.LocalCenter[2 * b + 1];
            return new Vector2D(p[0] - (double)(r[0] * lx - r[1] * ly), p[1] - (double)(r[1] * lx + r[0] * ly));
        }
        set { using var replayMutation = _world?.RecordMutation("Body.Position"); SetTransform(value, Angle); }
    }

    /// <summary>The rotation as (cos, sin).</summary>
    public Rot2 Rotation { get { var (w, b) = Resolve(); var r = w.Core.Bodies.RotationOf(b); return new Rot2(r[0], r[1]); } }

    /// <summary>The rotation angle in radians within (-pi, pi].</summary>
    public float Angle { get { var r = Rotation; return (float)RecursiveAI.OneMath.Math64.Atan2(r.S, r.C); } }

    /// <summary>Velocity of the centre of mass, metres per second. Setting it wakes the body; static bodies refuse (<see cref="BodyTypeException"/>).</summary>
    public Vector2F LinearVelocity
    {
        get { var (w, b) = Resolve(); var v = w.Core.Bodies.LinearVelocityOf(b); return new Vector2F(v[0], v[1]); }
        set
        {
            using var replayMutation = _world?.RecordMutation("Body.LinearVelocity");
            var (w, b) = Resolve();
            RequireMovable(w, b);
            if (!float.IsFinite(value.X) || !float.IsFinite(value.Y)) { throw new ArgumentOutOfRangeException(nameof(value), value, "Velocity must be finite."); }
            w.Engine.CancelKinematicDrive(b);
            var v = w.Core.Bodies.LinearVelocityOf(b);
            v[0] = value.X; v[1] = value.Y;
            WakeBody(w, b);
        }
    }

    /// <summary>Angular velocity in radians per second.</summary>
    public float AngularVelocity
    {
        get { var (w, b) = Resolve(); return w.Core.Bodies.AngularVelocityOf(b)[0]; }
        set
        {
            using var replayMutation = _world?.RecordMutation("Body.AngularVelocity");
            var (w, b) = Resolve();
            RequireMovable(w, b);
            if (!float.IsFinite(value)) { throw new ArgumentOutOfRangeException(nameof(value), value, "Angular velocity must be finite."); }
            w.Engine.CancelKinematicDrive(b);
            w.Core.Bodies.AngularVelocityOf(b)[0] = value;
            WakeBody(w, b);
        }
    }

    /// <summary>The same as setting <see cref="LinearVelocity"/>; a method, so it also works on a handle that is not a variable (a list element or a property result).</summary>
    /// <exception cref="BodyTypeException">The body is static.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A component is not finite.</exception>
    public void SetLinearVelocity(Vector2F velocity) { using var replayMutation = _world?.RecordMutation("Body.SetLinearVelocity"); LinearVelocity = velocity; }

    /// <summary>The same as setting <see cref="AngularVelocity"/>; a method, so it also works on a handle that is not a variable.</summary>
    /// <exception cref="BodyTypeException">The body is static.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A component is not finite.</exception>
    public void SetAngularVelocity(float velocity) { using var replayMutation = _world?.RecordMutation("Body.SetAngularVelocity"); AngularVelocity = velocity; }

    /// <summary>The same as setting <see cref="Position"/> (teleports the body, keeping its rotation and velocity, and wakes it); a method, so it also works on a handle that is not a variable.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The position is not finite or is beyond +-2^21.</exception>
    public void SetPosition(Vector2D position) { using var replayMutation = _world?.RecordMutation("Body.SetPosition"); Position = position; }

    /// <summary>Mass in kg (0 for static and kinematic bodies). Setting overrides dynamic mass within 1e-18..1e30 without rescaling inertia; ResetMass restores shape-derived mass. Invalid setters throw ArgumentOutOfRangeException or BodyTypeException.</summary>
    public float Mass { set => SetMass(value); get { var (w, b) = Resolve(); return w.Core.Bodies.TypeOf(b) == BodyType.Dynamic ? w.Engine.Mass[b] : 0f; } }

    /// <summary>Rotational inertia about the centre of mass in kg m² (0 for static and kinematic bodies). Setting overrides dynamic inertia within 1e-18..1e30; ResetInertia restores the shape-derived value. Invalid setters throw ArgumentOutOfRangeException or BodyTypeException.</summary>
    public float Inertia { set => SetInertia(value); get { var (w, b) = Resolve(); return w.Core.Bodies.TypeOf(b) == BodyType.Dynamic ? w.Engine.Inertia[b] : 0f; } }

    /// <summary>The centre of mass in world coordinates.</summary>
    public Vector2D WorldCenterOfMass { get { var (w, b) = Resolve(); var p = w.Core.Bodies.PositionOf(b); return new Vector2D(p[0], p[1]); } }

    private static void RequireMovable(PhysicsWorld2D w, int b, [System.Runtime.CompilerServices.CallerMemberName] string operation = "")
    {
        if (w.Core.Bodies.TypeOf(b) == BodyType.Static) { throw new BodyTypeException(operation, w.Core.HandleOfBody(b), BodyType.Static, false); }
    }

    private static void WakeBody(PhysicsWorld2D w, int b)
    {
        var bodies = w.Core.Bodies;
        if (bodies.IsDynamic(b)) { w.Core.WakeIsland(bodies.Island[b]); bodies.SleepTime[b] = 0f; }
    }

    /// <summary>Teleports the body: its origin goes to <paramref name="position"/> and its rotation to <paramref name="angleRadians"/>, keeping its velocity. Wakes it.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The position is not finite or is beyond +-2^21.</exception>
    public void SetTransform(Vector2D position, float angleRadians)
    {
        using var replayMutation = _world?.RecordMutation("Body.SetTransform");
        var (w, b) = Resolve();
        if (!double.IsFinite(position.X) || !double.IsFinite(position.Y) || Math.Abs(position.X) > 2097152.0 || Math.Abs(position.Y) > 2097152.0)
        {
            throw new ArgumentOutOfRangeException(nameof(position), position, "Position " + position + " is outside the supported range of +-2^21 (2097152) relative to the world origin.");
        }

        if (!float.IsFinite(angleRadians)) { throw new ArgumentOutOfRangeException(nameof(angleRadians), angleRadians, "Angle must be finite."); }
        var e = w.Engine;
        var bodies = w.Core.Bodies;
        var (s, c) = RecursiveAI.OneMath.Math64.SinCos(angleRadians);
        e.CancelKinematicDrive(b);
        var r = bodies.RotationOf(b);
        r[0] = (float)c; r[1] = (float)s;
        float lx = e.LocalCenter[2 * b], ly = e.LocalCenter[2 * b + 1];
        var p = bodies.PositionOf(b);
        p[0] = position.X + (double)(r[0] * lx - r[1] * ly);
        p[1] = position.Y + (double)(r[1] * lx + r[0] * ly);
        e.Dp[2 * b] = 0f; e.Dp[2 * b + 1] = 0f;
        for (int sh = e.ShapeHead[b]; sh >= 0; sh = e.NextShape[sh])
        {
            e.Bp.MoveProxy(e.Proxy[sh], e.TightBox(b, in e.Geom[sh]), 0.0, 0.0);
        }

        WakeBody(w, b);
    }

    /// <summary>Applies a force through the centre of mass for the next step only (cleared after each step). Wakes the body.</summary>
    public void ApplyForce(Vector2F force)
    {
        using var replayMutation = _world?.RecordMutation("Body.ApplyForce");
        var (w, b) = Resolve();
        RequireDynamic(w, b);
        RequireFinite(force, nameof(force));
        w.Engine.Force[2 * b] += force.X; w.Engine.Force[2 * b + 1] += force.Y;
        WakeBody(w, b);
    }

    /// <summary>Applies a force at a world point for the next step only; the offset from the centre of mass adds a torque.</summary>
    public void ApplyForceAt(Vector2F force, Vector2D worldPoint)
    {
        using var replayMutation = _world?.RecordMutation("Body.ApplyForceAt");
        var (w, b) = Resolve();
        RequireDynamic(w, b);
        RequireFinite(force, nameof(force));
        var p = w.Core.Bodies.PositionOf(b);
        float rx = (float)(worldPoint.X - p[0]), ry = (float)(worldPoint.Y - p[1]);
        w.Engine.Force[2 * b] += force.X; w.Engine.Force[2 * b + 1] += force.Y;
        w.Engine.Torque[b] += rx * force.Y - ry * force.X;
        WakeBody(w, b);
    }

    /// <summary>Applies a torque for the next step only.</summary>
    public void ApplyTorque(float torque)
    {
        using var replayMutation = _world?.RecordMutation("Body.ApplyTorque");
        var (w, b) = Resolve();
        RequireDynamic(w, b);
        if (!float.IsFinite(torque)) { throw new ArgumentOutOfRangeException(nameof(torque), torque, "Torque must be finite."); }
        w.Engine.Torque[b] += torque;
        WakeBody(w, b);
    }

    /// <summary>Changes the velocity at once by <c>impulse / mass</c>. Wakes the body.</summary>
    public void ApplyImpulse(Vector2F impulse)
    {
        using var replayMutation = _world?.RecordMutation("Body.ApplyImpulse");
        var (w, b) = Resolve();
        RequireDynamic(w, b);
        RequireFinite(impulse, nameof(impulse));
        var bodies = w.Core.Bodies;
        var v = bodies.LinearVelocityOf(b);
        float m = bodies.InverseMass[b];
        v[0] += m * impulse.X; v[1] += m * impulse.Y;
        if (w.Options.DetailedInspection) { w.Core.BodyReadback.Pending(b, new(impulse.X, impulse.Y, 0), default); }
        WakeBody(w, b);
    }

    /// <summary>Applies an impulse at a world point: changes the linear velocity and, through the offset from the centre of mass, the angular velocity.</summary>
    public void ApplyImpulseAt(Vector2F impulse, Vector2D worldPoint)
    {
        using var replayMutation = _world?.RecordMutation("Body.ApplyImpulseAt");
        var (w, b) = Resolve();
        RequireDynamic(w, b);
        RequireFinite(impulse, nameof(impulse));
        var bodies = w.Core.Bodies;
        var p = bodies.PositionOf(b);
        float rx = (float)(worldPoint.X - p[0]), ry = (float)(worldPoint.Y - p[1]);
        var v = bodies.LinearVelocityOf(b);
        float m = bodies.InverseMass[b];
        v[0] += m * impulse.X; v[1] += m * impulse.Y;
        bodies.AngularVelocityOf(b)[0] += bodies.InverseInertia[b] * (rx * impulse.Y - ry * impulse.X);
        if (w.Options.DetailedInspection) { w.Core.BodyReadback.Pending(b, new(impulse.X, impulse.Y, 0), new(0, 0, rx * impulse.Y - ry * impulse.X)); }
        WakeBody(w, b);
    }

    /// <summary>Changes the angular velocity by <c>impulse / inertia</c> (an angular impulse about the out-of-plane axis, N·m·s; counter-clockwise is positive). Wakes the body.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The impulse is not finite.</exception>
    public void ApplyAngularImpulse(float impulse)
    {
        using var replayMutation = _world?.RecordMutation("Body.ApplyAngularImpulse");
        var (w, b) = Resolve();
        RequireDynamic(w, b);
        if (!float.IsFinite(impulse)) { throw new ArgumentOutOfRangeException(nameof(impulse), impulse, "An angular impulse must be finite."); }
        var bodies = w.Core.Bodies;
        bodies.AngularVelocityOf(b)[0] += bodies.InverseInertia[b] * impulse;
        if (w.Options.DetailedInspection) { w.Core.BodyReadback.Pending(b, default, new(0, 0, impulse)); }
        WakeBody(w, b);
    }

    private static void RequireDynamic(PhysicsWorld2D w, int b, [System.Runtime.CompilerServices.CallerMemberName] string operation = "")
    {
        if (w.Core.Bodies.TypeOf(b) != BodyType.Dynamic) { throw new BodyTypeException(operation, w.Core.HandleOfBody(b), w.Core.Bodies.TypeOf(b), true); }
    }

    private static void RequireFinite(Vector2F v, string name)
    {
        if (!float.IsFinite(v.X) || !float.IsFinite(v.Y)) { throw new ArgumentOutOfRangeException(name, v, "A force or impulse must be finite."); }
    }

    /// <summary>Wakes the body's island.</summary>
    public void Wake() { using var replayMutation = _world?.RecordMutation("Body.Wake");  var (w, b) = Resolve(); WakeBody(w, b); }

    /// <summary>Puts the body's island to sleep now (it wakes when touched).</summary>
    public void Sleep() { using var replayMutation = _world?.RecordMutation("Body.Sleep");  Resolve(); _world!.Core.Sleep(_handle); }

    /// <summary>Destroys the body, its shapes and its contacts. Every copy of this handle becomes stale.</summary>
    public void Destroy() { using var replayMutation = _world?.RecordMutation("Body.Destroy");  Resolve(); _world!.DestroyBody(_handle); }

    /// <summary>The body's shapes in creation order (allocates the array).</summary>
    public Shape[] GetShapes()
    {
        var (w, b) = Resolve();
        var list = new System.Collections.Generic.List<Shape>();
        for (int s = w.Engine.ShapeHead[b]; s >= 0; s = w.Engine.NextShape[s]) { list.Add(new Shape(w, w.Core.HandleOfShape(s))); }
        return list.ToArray();
    }

    /// <summary>Raised after a step for every contact that began touching this body, in event order. Needs no struct callback; the handler is a plain delegate.</summary>
    public event Action<Collision> OnCollisionEnter
    {
        add { Subscribe(0, value); }
        remove { Unsubscribe(0, value); }
    }

    /// <summary>Raised after every step for contacts that persist. Only recorded when <see cref="WorldOptions2D.Events"/> includes <c>EventMask.ContactPersist</c>.</summary>
    public event Action<Collision> OnCollisionStay
    {
        add { Subscribe(1, value); }
        remove { Unsubscribe(1, value); }
    }

    /// <summary>Raised after a step for every contact that stopped touching this body (or whose shape or body was destroyed).</summary>
    public event Action<Collision> OnCollisionExit
    {
        add { Subscribe(2, value); }
        remove { Unsubscribe(2, value); }
    }

    private void Subscribe(int kind, Action<Collision> handler)
    {
        ArgumentNullException.ThrowIfNull(handler);
        if (_world == null) { throw new InvalidHandleException("The default Body is not attached to a world."); }
        _world.AddHandler(_handle, kind, handler);
    }

    private void Unsubscribe(int kind, Action<Collision> handler)
    {
        if (_world == null || handler == null) { return; }
        _world.RemoveHandler(_handle, kind, handler);
    }

    /// <summary>Same world, slot and generation.</summary>
    public bool Equals(Body other) => ReferenceEquals(_world, other._world) && _handle == other._handle;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is Body b && Equals(b);

    /// <inheritdoc/>
    public override int GetHashCode() => _handle.GetHashCode();

    /// <summary>Handle equality.</summary>
    public static bool operator ==(Body a, Body b) => a.Equals(b);

    /// <summary>Handle inequality.</summary>
    public static bool operator !=(Body a, Body b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() => _handle.ToString();
}

/// <summary>A handle to a shape, with the same stale-handle behavior as <see cref="Body"/>.</summary>
public readonly partial struct Shape : IEquatable<Shape>
{
    private readonly PhysicsWorld2D? _world;
    private readonly ShapeHandle _handle;

    internal Shape(PhysicsWorld2D world, ShapeHandle handle)
    {
        _world = world;
        _handle = handle;
    }

    internal ShapeHandle Handle => _handle;

    /// <summary>The raw core handle for the raw tier.</summary>
    public ShapeHandle RawHandle => _handle;

    private int Resolve()
    {
        if (_world == null) { throw new InvalidHandleException("The default Shape is not attached to a world."); }
        return _world.Core.Resolve(_handle);
    }

    /// <summary>True while the shape exists.</summary>
    public bool IsValid => _world != null && _handle.World == _world.Id && _world.Core.Shapes.Handles.IsAlive(_handle.Index, _handle.Generation);

    /// <summary>The body that owns the shape.</summary>
    public Body Body { get { int s = Resolve(); return new Body(_world!, _world!.Core.HandleOfBody(_world.Core.Shapes.Owner[s])); } }

    /// <summary>Friction coefficient; changes apply to contacts updated from the next step on.</summary>
    public float Friction
    {
        get { int s = Resolve(); return _world!.Engine.Friction[s]; }
        set
        {
            using var replayMutation = _world?.RecordMutation("Shape.Friction");
            int s = Resolve();
            if (!(value >= 0f && value <= MaterialLimits.MaxCoefficient)) { throw new ArgumentOutOfRangeException(nameof(value), value, "Friction must be within 0..1e15."); }
            _world!.Engine.Friction[s] = value;
            _world!.Engine.MaterialEpoch++;
        }
    }

    /// <summary>Restitution within 0..1.</summary>
    public float Restitution
    {
        get { int s = Resolve(); return _world!.Engine.Restitution[s]; }
        set
        {
            using var replayMutation = _world?.RecordMutation("Shape.Restitution");
            int s = Resolve();
            if (!(value >= 0f && value <= 1f)) { throw new ArgumentOutOfRangeException(nameof(value), value, "Restitution must be within 0..1."); }
            _world!.Engine.Restitution[s] = value;
            _world!.Engine.MaterialEpoch++;
        }
    }

    /// <summary>Destroys the shape and its contacts and recomputes the owner's mass. Tile-grid boundary shapes cannot be destroyed independently: throws InvalidOperationException; edit the grid or destroy its body instead.</summary>
    public void Destroy() { using var replayMutation = _world?.RecordMutation("Shape.Destroy");  Resolve(); _world!.DestroyShape(_handle); }

    /// <summary>Same world, slot and generation.</summary>
    public bool Equals(Shape other) => ReferenceEquals(_world, other._world) && _handle == other._handle;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is Shape s && Equals(s);

    /// <inheritdoc/>
    public override int GetHashCode() => _handle.GetHashCode();

    /// <summary>Handle equality.</summary>
    public static bool operator ==(Shape a, Shape b) => a.Equals(b);

    /// <summary>Handle inequality.</summary>
    public static bool operator !=(Shape a, Shape b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() => _handle.ToString();
}
