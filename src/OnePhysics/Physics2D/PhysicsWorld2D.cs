using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Broadphase;
using RecursiveAI.OnePhysics.Collision2D;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>
/// A 2D physics world: bodies, shapes, contacts, and a deterministic fixed-step simulation. Everything is single-writer: call it from one thread at a time (steps use worker threads internally
/// if <see cref="WorldOptions2D.ThreadCount"/> is above 1, with identical results). Create one with <c>new PhysicsWorld2D()</c>, add bodies with <c>AddBox</c> and friends or with
/// <see cref="CreateBody(in BodyDef)"/> and the <c>Create*</c> shape methods, and call <see cref="Step"/> with a fixed time step.
/// </summary>
public sealed partial class PhysicsWorld2D
{
    private readonly Engine2D _engine;
    private readonly WorldCore _core;
    /// <summary>Creates a world. The options are copied; later changes to <see cref="Options"/> take effect at the next step (except <see cref="WorldOptions2D.ThreadCount"/>, <see cref="WorldOptions2D.WorldId"/> and <see cref="WorldOptions2D.BroadphaseMargin"/>, fixed at creation).</summary>
    /// <exception cref="ArgumentOutOfRangeException">An option is out of range.</exception>
    public PhysicsWorld2D(WorldOptions2D? options = null)
    {
        var o = (options ?? new WorldOptions2D()).Clone();
        o.Validate();
        _core = new WorldCore(o.ToCore());
        o.WorldId = _core.Id;
        var bp = new Broadphase2D(new BroadphaseOptions { Margin = o.BroadphaseMargin });
        _engine = new Engine2D(_core, bp, o) { World = this };
        _core.AddSection(bp);
        _core.AddSection(_engine);
    }

    /// <summary>The live options. Change a value between steps and the next step uses it.</summary>
    public WorldOptions2D Options => _engine.Opt;

    /// <summary>Number of completed steps.</summary>
    public long StepCount => _core.StepCount;

    /// <summary>The id stamped into this world's handles.</summary>
    public int Id => _core.Id;

    /// <summary>The origin offset accumulated by <see cref="ShiftOrigin"/>; a position in host coordinates is the body's position plus this.</summary>
    public Vector2D Origin => new(_core.Origin[0], _core.Origin[1]);

    /// <summary>The raw tier: the dimension-independent core this world drives (bodies, constraints, islands, events). Read it freely; changing it behind the engine's back is unsupported.</summary>
    public WorldCore Core => _core;

    /// <summary>The events of the last step in their delivered order (contact begin, persist and end, sleep, wake). Valid until the next <see cref="Step"/>.</summary>
    public ReadOnlySpan<PhysicsEvent> Events => _core.Events.Delivered;

    internal Engine2D Engine => _engine;

    // ------------------------------------------------------------------ stepping

    /// <summary>
    /// Advances the world by <paramref name="dt"/> seconds. The world has no clock; give it the same dt every call for the best behavior. After the step the events are delivered:
    /// collision callbacks registered on bodies run in event order (kind, then the bodies' slots) and <see cref="Events"/> holds the same list for polling.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="dt"/> is not finite and positive, or an option is out of range.</exception>
    /// <exception cref="BodyOutOfBoundsException">A moving body's bounds are beyond ±2^22 or not finite; nothing was changed.</exception>
    public void Step(float dt)
    {
        _engine.Opt.Validate();
        _engine.CheckBounds();
        SyncCoreOptions();
        _core.Step(new Stages2D(_engine), dt);
        InvalidateQueryPad();
        _engine.ReconcileJoints();
        if (_handlers.Count > 0 || _jointHandlers.Count > 0) { DispatchEvents(); }
    }

    private void SyncCoreOptions()
    {
        var o = _engine.Opt;
        var c = _core.Options;
        c.Gravity = new Vector3F(o.Gravity.X, o.Gravity.Y, 0f);
        c.Substeps = o.Substeps; c.VelocityIterations = o.VelocityIterations; c.RelaxIterations = o.RelaxIterations;
        c.SleepEnabled = o.SleepEnabled; c.TimeToSleep = o.TimeToSleep; c.SleepThreshold = o.SleepThreshold; c.Events = o.Events;
    }

    private void LoadOptionsFromCore()
    {
        var o = _engine.Opt;
        var c = _core.Options;
        o.Gravity = new Vector2F(c.Gravity.X, c.Gravity.Y);
        o.Substeps = c.Substeps; o.VelocityIterations = c.VelocityIterations; o.RelaxIterations = c.RelaxIterations;
        o.SleepEnabled = c.SleepEnabled; o.TimeToSleep = c.TimeToSleep; o.SleepThreshold = c.SleepThreshold; o.Events = c.Events;
    }

    // ------------------------------------------------------------------ hooks for a deterministic host

    /// <summary>The 64-bit hash of the whole simulation state (bodies, shapes, contacts with their warm-start impulses, islands, broadphase, options and origin; not pending callbacks). Equal states give equal hashes on every platform.</summary>
    public ulong ComputeHash() => _core.ComputeHash();

    /// <summary>Serializes the whole world, byte-exact: restoring the bytes into a world made with the same options and continuing gives identical per-step hashes, events and snapshots. Layout in docs/world2d.md.</summary>
    public byte[] Snapshot() => _core.Snapshot();

    /// <summary>
    /// Replaces the world's state with a snapshot. The snapshot is validated first; on any problem a <see cref="SnapshotFormatException"/> is thrown and the world is unchanged.
    /// Handles taken before the snapshot stay valid in a world with the same <see cref="Id"/> when the same bodies exist in it. Collision callbacks of bodies that no longer exist are dropped.
    /// </summary>
    /// <exception cref="SnapshotFormatException">The bytes are not a valid snapshot of a 2D world.</exception>
    public void Restore(byte[] snapshot)
    {
        ArgumentNullException.ThrowIfNull(snapshot);
        byte[] backup = _core.Snapshot();
        _core.Restore(snapshot);
        string? problem = null;
        if (!_engine.IsConsistentWithCore()) { problem = _engine.ConsistencyProblem.Length > 0 ? _engine.ConsistencyProblem : "the snapshot's 2D engine state does not match its bodies and shapes"; }
        else
        {
            LoadOptionsFromCore();
            try { _engine.Opt.Validate(); }
            catch (ArgumentOutOfRangeException e) { problem = "the snapshot holds an option out of range: " + e.Message; }
            if (problem == null && _engine.FindOutOfBounds(true, out _, out string bounds)) { problem = "the snapshot places a body out of range: " + bounds; }
        }

        if (problem != null)
        {
            _core.Restore(backup);
            LoadOptionsFromCore();
            throw new SnapshotFormatException(problem, 0);
        }

        InvalidateQueryPad();
        var dead = new List<int>();
        foreach (var kv in _handlers)
        {
            if (!_core.Bodies.Handles.IsAlive(kv.Key, kv.Value.Generation)) { dead.Add(kv.Key); }
        }

        foreach (int k in dead) { _handlers.Remove(k); }
        if (_controllerHandlers.Count > 0) { DropDeadControllerHandlers(); }
    }

    /// <summary>
    /// Moves the world origin: every body position decreases by <paramref name="offset"/> and <see cref="Origin"/> grows by it, so a host position (origin + position) does not change.
    /// Call it between steps on every peer at the same step. Components must be multiples of 2^-31 (every 31.32 fixed-point value is) within +-2^22, otherwise an exception names the value.
    /// </summary>
    public void ShiftOrigin(Vector2D offset)
    {
        _core.ShiftOrigin([offset.X, offset.Y]);
        _engine.RebuildProxies();
        InvalidateQueryPad();
    }

    // ------------------------------------------------------------------ middle tier: bodies and shapes

    /// <summary>The <see cref="Body"/> for a raw handle (from an event, an exception or the raw tier).</summary>
    /// <exception cref="InvalidHandleException">The handle is null, belongs to another world or refers to a destroyed body.</exception>
    public Body GetBody(BodyHandle handle)
    {
        _core.Resolve(handle);
        return new Body(this, handle);
    }

    /// <summary>Creates a body from a definition.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite or the position is beyond +-2^21 from the origin (the message names it).</exception>
    public Body CreateBody(in BodyDef def) => new(this, _engine.CreateBody(in def));

    /// <summary>Creates a body from a definition; the one-liner form of <see cref="CreateBody(in BodyDef)"/>.</summary>
    public Body CreateBody(BodyType type, Vector2D position, float angle = 0f)
    {
        var d = new BodyDef { Type = type, Position = position, Angle = angle };
        return CreateBody(in d);
    }

    /// <summary>Adds a circle (centre in the body's frame).</summary>
    /// <exception cref="InvalidShapeException">The radius or centre is not valid.</exception>
    public Shape CreateCircle(Body body, float radius, Vector2F center = default, ShapeDef? def = null)
        => AddShape(body, Shape2D.CreateCircle(new Vec2(center.X, center.Y), radius), def);

    /// <summary>Adds a capsule between two points of the body's frame.</summary>
    public Shape CreateCapsule(Body body, Vector2F p1, Vector2F p2, float radius, ShapeDef? def = null)
        => AddShape(body, Shape2D.CreateCapsule(new Vec2(p1.X, p1.Y), new Vec2(p2.X, p2.Y), radius), def);

    /// <summary>Adds the convex hull of up to 8 points, rounded by <paramref name="radius"/>. Hostile point sets (duplicates, collinear, NaN) throw <see cref="InvalidShapeException"/>.</summary>
    public Shape CreatePolygon(Body body, ReadOnlySpan<Vector2F> points, float radius = 0f, ShapeDef? def = null)
    {
        var v = new Vec2[points.Length];
        for (int i = 0; i < points.Length; i++) { v[i] = new Vec2(points[i].X, points[i].Y); }
        return AddShape(body, Shape2D.CreatePolygon(v, radius), def);
    }

    /// <summary>Adds a box of the given half extents, optionally offset and rotated within the body's frame and rounded by <paramref name="radius"/>.</summary>
    public Shape CreateBox(Body body, float halfWidth, float halfHeight, Vector2F center = default, float angle = 0f, float radius = 0f, ShapeDef? def = null)
    {
        var (s, c) = RecursiveAI.OneMath.Math64.SinCos(angle);
        return AddShape(body, Shape2D.CreateBox(halfWidth, halfHeight, new Vec2(center.X, center.Y), new Rot2((float)c, (float)s), radius), def);
    }

    /// <summary>Adds a two-sided segment with no thickness (it has no area, so it adds no mass).</summary>
    public Shape CreateSegment(Body body, Vector2F p1, Vector2F p2, ShapeDef? def = null)
        => AddShape(body, Shape2D.CreateSegment(new Vec2(p1.X, p1.Y), new Vec2(p2.X, p2.Y)), def);

    /// <summary>
    /// Adds a chain: one chain-segment shape per link, with ghost vertices so bodies slide over the joints without snagging. At least 2 points (3 for a loop). One-sided chains are solid on the right of the direction of travel.
    /// Returns the first segment's shape; the others have the following slots in creation order unless slots were being reused.
    /// </summary>
    public Shape CreateChain(Body body, ReadOnlySpan<Vector2F> points, bool loop = false, bool twoSided = true, ShapeDef? def = null)
    {
        var v = new Vec2[points.Length];
        for (int i = 0; i < points.Length; i++) { v[i] = new Vec2(points[i].X, points[i].Y); }
        var chain = new Chain2D(v, loop, twoSided);
        Shape first = default;
        for (int i = 0; i < chain.SegmentCount; i++)
        {
            var s = AddShape(body, chain.GetSegment(i), def);
            if (i == 0) { first = s; }
        }

        return first;
    }

    private Shape AddShape(Body body, in Shape2D geometry, ShapeDef? def)
    {
        if (body.World != this) { throw new ArgumentException("The body belongs to another world.", nameof(body)); }
        var d = def ?? new ShapeDef();
        return new Shape(this, _engine.AddShape(body.Handle, in geometry, in d));
    }

    // ------------------------------------------------------------------ happy path

    /// <summary>Adds a box of the given full size (width, height) centred at <paramref name="at"/>. Dynamic by default.</summary>
    /// <example><c>var box = world.AddBox(new Vector2F(1, 1), at: new Vector2D(0, 5));</c></example>
    public Body AddBox(Vector2F size, Vector2D at = default, float angle = 0f, BodyType type = BodyType.Dynamic, Material2D? material = null, float density = 1f)
    {
        var body = CreateBody(type, at, angle);
        CreateBox(body, 0.5f * size.X, 0.5f * size.Y, default, 0f, 0f, ShapeFor(material, density));
        return body;
    }

    /// <summary>Adds a circle centred at <paramref name="at"/>. Dynamic by default.</summary>
    public Body AddCircle(float radius, Vector2D at = default, BodyType type = BodyType.Dynamic, Material2D? material = null, float density = 1f)
    {
        var body = CreateBody(type, at);
        CreateCircle(body, radius, default, ShapeFor(material, density));
        return body;
    }

    /// <summary>Adds a capsule whose axis runs from (-halfLength, 0) to (halfLength, 0) in the body's frame. Dynamic by default.</summary>
    public Body AddCapsule(float halfLength, float radius, Vector2D at = default, float angle = 0f, BodyType type = BodyType.Dynamic, Material2D? material = null, float density = 1f)
    {
        var body = CreateBody(type, at, angle);
        CreateCapsule(body, new Vector2F(-halfLength, 0f), new Vector2F(halfLength, 0f), radius, ShapeFor(material, density));
        return body;
    }

    /// <summary>Adds the convex hull of up to 8 points (in the body's frame), rounded by <paramref name="radius"/>. Dynamic by default.</summary>
    public Body AddPolygon(ReadOnlySpan<Vector2F> points, Vector2D at = default, float angle = 0f, float radius = 0f, BodyType type = BodyType.Dynamic, Material2D? material = null, float density = 1f)
    {
        var body = CreateBody(type, at, angle);
        try { CreatePolygon(body, points, radius, ShapeFor(material, density)); }
        catch (ArgumentException) { body.Destroy(); throw; }
        return body;
    }

    /// <summary>Adds a static two-sided segment between two world points.</summary>
    public Body AddSegment(Vector2D a, Vector2D b, Material2D? material = null)
    {
        var body = CreateBody(BodyType.Static, default);
        CreateSegment(body, a.ToSingle(), b.ToSingle(), ShapeFor(material, 0f));
        return body;
    }

    /// <summary>Adds a static chain through world points (two-sided unless <paramref name="twoSided"/> is false, then solid on the right of the direction of travel).</summary>
    public Body AddChain(ReadOnlySpan<Vector2D> points, bool loop = false, bool twoSided = true, Material2D? material = null)
    {
        var p = new Vector2F[points.Length];
        for (int i = 0; i < p.Length; i++) { p[i] = points[i].ToSingle(); }
        var body = CreateBody(BodyType.Static, default);
        try { CreateChain(body, p, loop, twoSided, ShapeFor(material, 0f)); }
        catch (ArgumentException) { body.Destroy(); throw; }
        return body;
    }

    private static ShapeDef ShapeFor(Material2D? material, float density)
    {
        var d = new ShapeDef { Density = density };
        return material.HasValue ? d.WithMaterial(material.Value) : d;
    }

    // ------------------------------------------------------------------ used by handles

    internal int ResolveBody(BodyHandle h) => _core.Resolve(h);

    internal void DestroyBody(BodyHandle h)
    {
        _engine.DestroyBody(h);
        _handlers.Remove(h.Index);
        if (_controllerHandlers.Count > 0) { DropDeadControllerHandlers(); }
    }

    internal void DestroyShape(ShapeHandle h) => _engine.DestroyShape(h);
}
