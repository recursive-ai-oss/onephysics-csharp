using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// A rigid body you can hold: a small struct (world, slot, generation) that allocates nothing. Every member fails with a clear <see cref="InvalidHandleException"/> once the body is destroyed
/// or when used on the default value. Positions are doubles in world coordinates; velocities, forces and orientation are float32.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public readonly partial struct Body : IEquatable<Body>
{
    private readonly PhysicsWorld3D? _world;
    private readonly BodyHandle _handle;

    internal Body(PhysicsWorld3D world, BodyHandle handle)
    {
        _world = world;
        _handle = handle;
    }

    internal BodyHandle Handle => _handle;

    internal PhysicsWorld3D? World => _world;

    /// <summary>The raw handle (world id, slot, generation) for the middle and raw tiers.</summary>
    public BodyHandle RawHandle => _handle;

    private (PhysicsWorld3D World, int Slot) Resolve()
    {
        if (_world == null) { throw new InvalidHandleException("The default Body is not attached to a world."); }
        return (_world, _world.ResolveBody(_handle));
    }

    /// <summary>True while the body exists.</summary>
    public bool IsValid => _world != null && _world.Core.Bodies.Handles.IsAlive(_handle.Index, _handle.Generation) && _handle.World == _world.Id;

    /// <summary>Static, kinematic or dynamic.</summary>
    public BodyType Type { get { var (w, b) = Resolve(); return w.Core.Bodies.TypeOf(b); } }

    /// <summary>True for kinematic bodies (moved by their velocity, not pushed).</summary>
    public bool IsKinematic => Type == BodyType.Kinematic;

    /// <summary>True for static bodies.</summary>
    public bool IsStatic => Type == BodyType.Static;

    /// <summary>False for sleeping dynamic bodies; kinematic bodies are always awake.</summary>
    public bool IsAwake { get { var (w, b) = Resolve(); var t = w.Core.Bodies.TypeOf(b); return t == BodyType.Kinematic || (t == BodyType.Dynamic && w.Core.Bodies.IsAwake(b)); } }

    /// <summary>The body origin in world coordinates. Setting it teleports the body (and wakes it).</summary>
    public Vector3D Position
    {
        get
        {
            var (w, b) = Resolve();
            var p = w.Core.Bodies.PositionOf(b);
            var off = w.Engine.RotationOfBody(b).Rotate(w.Engine.LocalCenterOf(b));
            return new Vector3D(p[0] - (double)off.X, p[1] - (double)off.Y, p[2] - (double)off.Z);
        }
        set => SetTransform(value, Rotation);
    }

    /// <summary>The body orientation. Setting it rotates the body about its origin (and wakes it).</summary>
    public QuaternionF Rotation
    {
        get { var (w, b) = Resolve(); return w.Engine.RotationOfBody(b); }
        set => SetTransform(Position, value);
    }

    /// <summary>Linear velocity of the centre of mass, m/s.</summary>
    /// <exception cref="BodyTypeException">Setting it on a static body.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A component is not finite.</exception>
    public Vector3F LinearVelocity
    {
        get { var (w, b) = Resolve(); return Vec(w.Core.Bodies.LinearVelocityOf(b)); }
        set
        {
            var (w, b) = Resolve();
            RequireMovable(w, b);
            RequireFinite(value, nameof(value));
            var v = w.Core.Bodies.LinearVelocityOf(b);
            v[0] = value.X; v[1] = value.Y; v[2] = value.Z;
            WakeBody(w, b);
        }
    }

    /// <summary>Angular velocity in world axes, rad/s.</summary>
    /// <exception cref="BodyTypeException">Setting it on a static body.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A component is not finite.</exception>
    public Vector3F AngularVelocity
    {
        get { var (w, b) = Resolve(); return Vec(w.Core.Bodies.AngularVelocityOf(b)); }
        set
        {
            var (w, b) = Resolve();
            RequireMovable(w, b);
            RequireFinite(value, nameof(value));
            var v = w.Core.Bodies.AngularVelocityOf(b);
            v[0] = value.X; v[1] = value.Y; v[2] = value.Z;
            WakeBody(w, b);
        }
    }

    /// <summary>The same as setting <see cref="LinearVelocity"/>; a method, so it also works on a handle that is not a variable (a list element or a property result).</summary>
    /// <exception cref="BodyTypeException">The body is static.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A component is not finite.</exception>
    public void SetLinearVelocity(Vector3F velocity) => LinearVelocity = velocity;

    /// <summary>The same as setting <see cref="AngularVelocity"/>; a method, so it also works on a handle that is not a variable.</summary>
    /// <exception cref="BodyTypeException">The body is static.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A component is not finite.</exception>
    public void SetAngularVelocity(Vector3F velocity) => AngularVelocity = velocity;

    /// <summary>The same as setting <see cref="Position"/> (teleports the body, keeping its rotation and velocity, and wakes it); a method, so it also works on a handle that is not a variable.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The position is not finite or is beyond +-2^21.</exception>
    public void SetPosition(Vector3D position) => Position = position;

    /// <summary>Mass in kg; 0 for static and kinematic bodies.</summary>
    public float Mass { get { var (w, b) = Resolve(); return w.Core.Bodies.TypeOf(b) == BodyType.Dynamic ? w.Engine.Mass[b] : 0f; } }

    /// <summary>The inertia tensor about the centre of mass in the body's own axes; zero for static and kinematic bodies.</summary>
    public Matrix3x3F Inertia
    {
        get
        {
            var (w, b) = Resolve();
            if (w.Core.Bodies.TypeOf(b) != BodyType.Dynamic) { return default; }
            var i = w.Engine.Inertia;
            int o = 6 * b;
            return new Matrix3x3F(i[o], i[o + 1], i[o + 2], i[o + 1], i[o + 3], i[o + 4], i[o + 2], i[o + 4], i[o + 5]);
        }
    }

    /// <summary>The centre of mass in world coordinates.</summary>
    public Vector3D WorldCenterOfMass { get { var (w, b) = Resolve(); var p = w.Core.Bodies.PositionOf(b); return new Vector3D(p[0], p[1], p[2]); } }

    private static Vector3F Vec(ReadOnlySpan<float> v) => new(v[0], v[1], v[2]);

    private static void RequireMovable(PhysicsWorld3D w, int b, [System.Runtime.CompilerServices.CallerMemberName] string operation = "")
    {
        if (w.Core.Bodies.TypeOf(b) == BodyType.Static) { throw new BodyTypeException(operation, w.Core.HandleOfBody(b), BodyType.Static, false); }
    }

    private static void WakeBody(PhysicsWorld3D w, int b)
    {
        var bodies = w.Core.Bodies;
        if (bodies.IsDynamic(b)) { w.Core.WakeIsland(bodies.Island[b]); bodies.SleepTime[b] = 0f; }
    }

    /// <summary>Teleports the body: the origin goes to <paramref name="position"/>, the orientation to <paramref name="rotation"/>. Velocities are kept; the body wakes.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The position is not finite or beyond ±2^21 from the world origin, or the rotation is zero or not finite.</exception>
    public void SetTransform(Vector3D position, QuaternionF rotation)
    {
        var (w, b) = Resolve();
        const double Bound = 2097152.0;
        if (!double.IsFinite(position.X) || !double.IsFinite(position.Y) || !double.IsFinite(position.Z) || Math.Abs(position.X) > Bound || Math.Abs(position.Y) > Bound || Math.Abs(position.Z) > Bound)
        {
            throw new ArgumentOutOfRangeException(nameof(position), position, "Position " + position + " is outside the supported range of +-2^21 (2097152) relative to the world origin.");
        }

        var q = Engine3D.Normalize(rotation, nameof(rotation));
        var e = w.Engine;
        var bodies = w.Core.Bodies;
        var r = bodies.RotationOf(b);
        r[0] = q.X; r[1] = q.Y; r[2] = q.Z; r[3] = q.W;
        var off = q.Rotate(e.LocalCenterOf(b));
        var p = bodies.PositionOf(b);
        p[0] = position.X + (double)off.X; p[1] = position.Y + (double)off.Y; p[2] = position.Z + (double)off.Z;
        e.Dp[3 * b] = 0f; e.Dp[3 * b + 1] = 0f; e.Dp[3 * b + 2] = 0f;
        for (int sh = e.ShapeHead[b]; sh >= 0; sh = e.NextShape[sh])
        {
            e.Bp.MoveProxy(e.Proxy[sh], e.TightBox(b, sh), 0.0, 0.0, 0.0);
        }

        WakeBody(w, b);
    }

    /// <summary>Adds a force through the centre of mass for the next step (cleared after it), N.</summary>
    /// <exception cref="BodyTypeException">The body is not dynamic.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A component is not finite.</exception>
    public void ApplyForce(Vector3F force)
    {
        var (w, b) = Resolve();
        RequireDynamic(w, b);
        RequireFinite(force, nameof(force));
        var f = w.Engine.Force;
        f[3 * b] += force.X; f[3 * b + 1] += force.Y; f[3 * b + 2] += force.Z;
        WakeBody(w, b);
    }

    /// <summary>Adds a force at a world point for the next step; the offset from the centre of mass makes a torque.</summary>
    public void ApplyForceAt(Vector3F force, Vector3D worldPoint)
    {
        var (w, b) = Resolve();
        RequireDynamic(w, b);
        RequireFinite(force, nameof(force));
        var p = w.Core.Bodies.PositionOf(b);
        var r = new Vector3F((float)(worldPoint.X - p[0]), (float)(worldPoint.Y - p[1]), (float)(worldPoint.Z - p[2]));
        var f = w.Engine.Force;
        f[3 * b] += force.X; f[3 * b + 1] += force.Y; f[3 * b + 2] += force.Z;
        var t = Vector3F.Cross(r, force);
        var tq = w.Engine.Torque;
        tq[3 * b] += t.X; tq[3 * b + 1] += t.Y; tq[3 * b + 2] += t.Z;
        WakeBody(w, b);
    }

    /// <summary>Adds a torque in world axes for the next step, N·m.</summary>
    public void ApplyTorque(Vector3F torque)
    {
        var (w, b) = Resolve();
        RequireDynamic(w, b);
        RequireFinite(torque, nameof(torque));
        var tq = w.Engine.Torque;
        tq[3 * b] += torque.X; tq[3 * b + 1] += torque.Y; tq[3 * b + 2] += torque.Z;
        WakeBody(w, b);
    }

    /// <summary>Changes the velocity by an impulse through the centre of mass, N·s.</summary>
    public void ApplyImpulse(Vector3F impulse)
    {
        var (w, b) = Resolve();
        RequireDynamic(w, b);
        RequireFinite(impulse, nameof(impulse));
        var bodies = w.Core.Bodies;
        var v = bodies.LinearVelocityOf(b);
        float m = bodies.InverseMass[b];
        v[0] += m * impulse.X; v[1] += m * impulse.Y; v[2] += m * impulse.Z;
        WakeBody(w, b);
    }

    /// <summary>Changes the velocities by an impulse at a world point.</summary>
    public void ApplyImpulseAt(Vector3F impulse, Vector3D worldPoint)
    {
        var (w, b) = Resolve();
        RequireDynamic(w, b);
        RequireFinite(impulse, nameof(impulse));
        var bodies = w.Core.Bodies;
        var p = bodies.PositionOf(b);
        var r = new Vector3F((float)(worldPoint.X - p[0]), (float)(worldPoint.Y - p[1]), (float)(worldPoint.Z - p[2]));
        var v = bodies.LinearVelocityOf(b);
        float m = bodies.InverseMass[b];
        v[0] += m * impulse.X; v[1] += m * impulse.Y; v[2] += m * impulse.Z;
        AddAngular(w, b, Vector3F.Cross(r, impulse));
        WakeBody(w, b);
    }

    /// <summary>Changes the angular velocity by an angular impulse in world axes, N·m·s.</summary>
    public void ApplyAngularImpulse(Vector3F impulse)
    {
        var (w, b) = Resolve();
        RequireDynamic(w, b);
        RequireFinite(impulse, nameof(impulse));
        AddAngular(w, b, impulse);
        WakeBody(w, b);
    }

    private static void AddAngular(PhysicsWorld3D w, int b, Vector3F angularImpulse)
    {
        var e = w.Engine;
        e.RefreshWorldInertia(b);
        var dw = e.ApplyWorldInertia(b, angularImpulse);
        var av = w.Core.Bodies.AngularVelocityOf(b);
        av[0] += dw.X; av[1] += dw.Y; av[2] += dw.Z;
    }

    private static void RequireDynamic(PhysicsWorld3D w, int b, [System.Runtime.CompilerServices.CallerMemberName] string operation = "")
    {
        if (w.Core.Bodies.TypeOf(b) != BodyType.Dynamic) { throw new BodyTypeException(operation, w.Core.HandleOfBody(b), w.Core.Bodies.TypeOf(b), true); }
    }

    private static void RequireFinite(Vector3F v, string name)
    {
        if (!float.IsFinite(v.X) || !float.IsFinite(v.Y) || !float.IsFinite(v.Z)) { throw new ArgumentOutOfRangeException(name, v, "A force, impulse or velocity must be finite."); }
    }

    /// <summary>Wakes the body and its island.</summary>
    public void Wake() { var (w, b) = Resolve(); WakeBody(w, b); }

    /// <summary>Puts the body and its island to sleep (dynamic bodies only; velocities are cleared).</summary>
    public void Sleep() { Resolve(); _world!.Core.Sleep(_handle); }

    /// <summary>Destroys the body, its shapes and its contacts. Handles to it go stale.</summary>
    public void Destroy() { Resolve(); _world!.DestroyBody(_handle); }

    /// <summary>The body's shapes, in creation order. Allocates.</summary>
    public Shape[] GetShapes()
    {
        var (w, b) = Resolve();
        var list = new System.Collections.Generic.List<Shape>();
        for (int s = w.Engine.ShapeHead[b]; s >= 0; s = w.Engine.NextShape[s]) { list.Add(new Shape(w, w.Core.HandleOfShape(s))); }
        return list.ToArray();
    }

    /// <inheritdoc />
    public bool Equals(Body other) => ReferenceEquals(_world, other._world) && _handle == other._handle;

    /// <inheritdoc />
    public override bool Equals(object? obj) => obj is Body b && Equals(b);

    /// <inheritdoc />
    public override int GetHashCode() => _handle.GetHashCode();

    /// <summary>Equality.</summary>
    public static bool operator ==(Body a, Body b) => a.Equals(b);

    /// <summary>Inequality.</summary>
    public static bool operator !=(Body a, Body b) => !a.Equals(b);

    /// <inheritdoc />
    public override string ToString() => _handle.ToString();
}

/// <summary>A shape handle: geometry attached to a body, with its material and filter. Fails clearly once destroyed.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public readonly partial struct Shape : IEquatable<Shape>
{
    private readonly PhysicsWorld3D? _world;
    private readonly ShapeHandle _handle;

    internal Shape(PhysicsWorld3D world, ShapeHandle handle)
    {
        _world = world;
        _handle = handle;
    }

    internal ShapeHandle Handle => _handle;

    /// <summary>The raw handle for the middle and raw tiers.</summary>
    public ShapeHandle RawHandle => _handle;

    private int Resolve()
    {
        if (_world == null) { throw new InvalidHandleException("The default Shape is not attached to a world."); }
        return _world.Core.Resolve(_handle);
    }

    /// <summary>True while the shape exists.</summary>
    public bool IsValid => _world != null && _handle.World == _world.Id && _world.Core.Shapes.Handles.IsAlive(_handle.Index, _handle.Generation);

    /// <summary>The body the shape belongs to.</summary>
    public Body Body { get { int s = Resolve(); return new Body(_world!, _world!.Core.HandleOfBody(_world.Core.Shapes.Owner[s])); } }

    /// <summary>Friction coefficient; finite, at least 0.</summary>
    public float Friction
    {
        get { int s = Resolve(); return _world!.Engine.Friction[s]; }
        set
        {
            int s = Resolve();
            if (!(value >= 0f && value <= MaterialLimits.MaxCoefficient)) { throw new ArgumentOutOfRangeException(nameof(value), value, "Friction must be within 0..1e15."); }
            _world!.Engine.Friction[s] = value;
        }
    }

    /// <summary>Restitution in 0..1.</summary>
    public float Restitution
    {
        get { int s = Resolve(); return _world!.Engine.Restitution[s]; }
        set
        {
            int s = Resolve();
            if (!(value >= 0f && value <= 1f)) { throw new ArgumentOutOfRangeException(nameof(value), value, "Restitution must be within 0..1."); }
            _world!.Engine.Restitution[s] = value;
        }
    }

    /// <summary>Destroys the shape; the body's mass is recomputed.</summary>
    public void Destroy() { Resolve(); _world!.DestroyShape(_handle); }

    /// <inheritdoc />
    public bool Equals(Shape other) => ReferenceEquals(_world, other._world) && _handle == other._handle;

    /// <inheritdoc />
    public override bool Equals(object? obj) => obj is Shape s && Equals(s);

    /// <inheritdoc />
    public override int GetHashCode() => _handle.GetHashCode();

    /// <summary>Equality.</summary>
    public static bool operator ==(Shape a, Shape b) => a.Equals(b);

    /// <summary>Inequality.</summary>
    public static bool operator !=(Shape a, Shape b) => !a.Equals(b);

    /// <inheritdoc />
    public override string ToString() => _handle.ToString();
}
