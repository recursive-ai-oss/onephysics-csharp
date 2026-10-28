using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Broadphase;
using RecursiveAI.OnePhysics.Collision3D;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// A 3D rigid-body world. The happy path is the <c>Add*</c> methods and <see cref="Step"/>; <see cref="CreateBody(in BodyDef)"/> and the <c>Create*</c> shape methods are the middle tier;
/// <see cref="Core"/> exposes the shared storage for the raw tier. Results are bit-identical on every platform, thread count and vector width. Not thread-safe: use a world from one thread at a time.
/// </summary>
public sealed partial class PhysicsWorld3D
{
    private readonly Engine3D _engine;
    private readonly WorldCore _core;
    /// <summary>Creates a world.</summary>
    /// <param name="options">Options; copied. Defaults when null.</param>
    /// <exception cref="ArgumentOutOfRangeException">An option is out of range.</exception>
    public PhysicsWorld3D(WorldOptions3D? options = null)
    {
        var o = (options ?? new WorldOptions3D()).Clone();
        o.Validate();
        _core = new WorldCore(o.ToCore()) { SymmetricSweep = true, SplitWhenAnyBodyRests = true }; // alternating sweep order keeps one-iteration friction from drifting tall stacks sideways
        o.WorldId = _core.Id;
        var bp = new Broadphase3D(new BroadphaseOptions { Margin = o.BroadphaseMargin });
        _engine = new Engine3D(_core, bp, o) { World = this };
        _core.AddSection(bp);
        _core.AddSection(_engine);
    }

    /// <summary>The live options. Changes apply from the next step (not <see cref="WorldOptions3D.BroadphaseMargin"/>, <see cref="WorldOptions3D.ThreadCount"/> or <see cref="WorldOptions3D.WorldId"/>).</summary>
    public WorldOptions3D Options => _engine.Opt;

    /// <summary>Number of steps taken.</summary>
    public long StepCount => _core.StepCount;

    /// <summary>The id used in this world's handles.</summary>
    public int Id => _core.Id;

    /// <summary>The accumulated origin shift; world positions are relative to it.</summary>
    public Vector3D Origin => new(_core.Origin[0], _core.Origin[1], _core.Origin[2]);

    /// <summary>The shared core (raw tier): bodies, shapes, constraints, islands, coloring, events.</summary>
    public WorldCore Core => _core;

    /// <summary>The events of the last step in the core's deterministic order, with contact and hit events collapsed to one per pair of shapes (see docs/world3d.md). Valid until the next <see cref="Step"/>.</summary>
    public ReadOnlySpan<PhysicsEvent> Events => _publicEvents.AsSpan(0, _publicEventCount);

    internal Engine3D Engine => _engine;

    // ------------------------------------------------------------------ stepping

    /// <summary>Advances the world by a fixed time. Collision callbacks run after the step, in event order.</summary>
    /// <param name="dt">Seconds; finite and positive. Use the same value every step for reproducible results.</param>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="dt"/> is invalid or an option is out of range.</exception>
    /// <exception cref="BodyOutOfBoundsException">A moving body is outside the supported range; the world is unchanged. Move, destroy it or call <see cref="ShiftOrigin"/>.</exception>
    public void Step(float dt)
    {
        _engine.Opt.Validate();
        _engine.CheckBounds();
        SyncCoreOptions();
        _core.Step(new Stages3D(_engine), dt);
        _engine.ReconcileJoints();
        InvalidateQueryPad();
        BuildPublicEvents();
        if (_handlers.Count > 0 || _jointHandlers.Count > 0) { DispatchEvents(); }
    }

    private void SyncCoreOptions()
    {
        var o = _engine.Opt;
        var c = _core.Options;
        c.Gravity = o.Gravity;
        c.Substeps = o.Substeps; c.VelocityIterations = o.VelocityIterations; c.RelaxIterations = o.RelaxIterations;
        c.SleepEnabled = o.SleepEnabled; c.TimeToSleep = o.TimeToSleep; c.SleepThreshold = o.SleepThreshold; c.Events = o.Events;
    }

    private void LoadOptionsFromCore()
    {
        var o = _engine.Opt;
        var c = _core.Options;
        o.Gravity = c.Gravity;
        o.Substeps = c.Substeps; o.VelocityIterations = c.VelocityIterations; o.RelaxIterations = c.RelaxIterations;
        o.SleepEnabled = c.SleepEnabled; o.TimeToSleep = c.TimeToSleep; o.SleepThreshold = c.SleepThreshold; o.Events = c.Events;
    }

    // ------------------------------------------------------------------ hooks for a deterministic host

    /// <summary>A 64-bit hash of the whole world state in the documented order; equal worlds give equal hashes on every platform.</summary>
    public ulong ComputeHash() => _core.ComputeHash();

    /// <summary>The whole world as bytes (snapshot format 1 plus the 3D section). Delegates are not saved.</summary>
    public byte[] Snapshot() => _core.Snapshot();

    /// <summary>Replaces the world's state with a snapshot taken from a world with the same layout. Atomic: on failure the world is unchanged.</summary>
    /// <exception cref="SnapshotFormatException">The bytes are not a valid snapshot of a 3D world.</exception>
    public void Restore(byte[] snapshot)
    {
        ArgumentNullException.ThrowIfNull(snapshot);
        byte[] backup = _core.Snapshot();
        _core.Restore(snapshot);
        string? problem = null;
        if (!_engine.IsConsistentWithCore()) { problem = "the snapshot's 3D engine state does not match its bodies and shapes"; }
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

        var dead = new List<int>();
        foreach (var kv in _handlers)
        {
            if (!_core.Bodies.Handles.IsAlive(kv.Key, kv.Value.Generation)) { dead.Add(kv.Key); }
        }

        foreach (int k in dead) { _handlers.Remove(k); }
        DropStaleJointHandlers();
        DropDeadControllerHandlers();
        InvalidateQueryPad();
        BuildPublicEvents();
    }

    /// <summary>Moves the world origin by <paramref name="offset"/> (call between steps, on every peer at the same step): all positions shrink by it. Offsets must be multiples of 2^-31.</summary>
    public void ShiftOrigin(Vector3D offset)
    {
        _core.ShiftOrigin([offset.X, offset.Y, offset.Z]);
        _engine.ShiftJoints([offset.X, offset.Y, offset.Z]);
        _engine.RebuildProxies();
        InvalidateQueryPad();
    }

    // ------------------------------------------------------------------ middle tier: bodies and shapes

    /// <summary>Wraps a raw handle as a <see cref="Body"/>.</summary>
    /// <exception cref="InvalidHandleException">The handle is stale, null or from another world.</exception>
    public Body GetBody(BodyHandle handle)
    {
        _core.Resolve(handle);
        return new Body(this, handle);
    }

    /// <summary>Creates a body.</summary>
    public Body CreateBody(in BodyDef def) => new(this, _engine.CreateBody(in def));

    /// <summary>Creates a body with just a type, position and orientation.</summary>
    public Body CreateBody(BodyType type, Vector3D position, QuaternionF rotation = default)
    {
        var d = new BodyDef { Type = type, Position = position, Rotation = rotation.LengthSquared() == 0f ? QuaternionF.Identity : rotation };
        return CreateBody(in d);
    }

    /// <summary>Attaches a convex shape described by a <see cref="ShapeSpec3D"/>.</summary>
    /// <exception cref="ArgumentException">The shape's numbers are invalid (the message names them).</exception>
    public Shape CreateShape(Body body, ShapeSpec3D spec, ShapeDef? def = null) => AddShape(body, spec.Recipe.ToArray(), def);

    /// <summary>Attaches a sphere.</summary>
    public Shape CreateSphere(Body body, float radius, ShapeDef? def = null) => CreateShape(body, ShapeSpec3D.Sphere(radius), def);

    /// <summary>Attaches a box.</summary>
    public Shape CreateBox(Body body, Vector3F halfExtents, float convexRadius = 0f, ShapeDef? def = null) => CreateShape(body, ShapeSpec3D.Box(halfExtents, convexRadius), def);

    /// <summary>Attaches a capsule along the body's y axis.</summary>
    public Shape CreateCapsule(Body body, float halfHeight, float radius, ShapeDef? def = null) => CreateShape(body, ShapeSpec3D.Capsule(halfHeight, radius), def);

    /// <summary>Attaches a cylinder along the body's y axis.</summary>
    public Shape CreateCylinder(Body body, float halfHeight, float radius, float convexRadius = 0f, ShapeDef? def = null) => CreateShape(body, ShapeSpec3D.Cylinder(halfHeight, radius, convexRadius), def);

    /// <summary>Attaches the convex hull of a point set.</summary>
    public Shape CreateConvexHull(Body body, ReadOnlySpan<Vector3F> points, float convexRadius = 0f, ShapeDef? def = null) => CreateShape(body, ShapeSpec3D.ConvexHull(points, convexRadius), def);

    /// <summary>Attaches a compound of convex parts as one shape (it may be dynamic).</summary>
    public Shape CreateCompound(Body body, ReadOnlySpan<CompoundPart> parts, ShapeDef? def = null) => AddShape(body, Geom3D.CompoundRecipe(parts), def);

    /// <summary>Attaches a triangle mesh; the body must be static.</summary>
    /// <param name="body">A static body.</param>
    /// <param name="vertices">Vertices in the shape's frame.</param>
    /// <param name="indices">Triangle corners, three per triangle, outward winding.</param>
    /// <param name="def">Material and filter; the density is ignored.</param>
    /// <param name="flatEdgeTolerance">See <see cref="MeshBuildOptions.FlatEdgeTolerance"/>; 0 for the default.</param>
    /// <exception cref="MeshBuildException">The mesh is invalid (the message names the input).</exception>
    /// <exception cref="StaticOnlyShapeException">The body is not static.</exception>
    public Shape CreateMesh(Body body, ReadOnlySpan<Vector3F> vertices, ReadOnlySpan<int> indices, ShapeDef? def = null, double flatEdgeTolerance = 0.0)
        => AddShape(body, Geom3D.MeshRecipe(vertices, indices, flatEdgeTolerance), def);

    /// <summary>Attaches a height field; the body must be static. Sample (i, j) is at x = i·scale.X, z = j·scale.Z from the shape's origin, height × scale.Y.</summary>
    /// <exception cref="MeshBuildException">The grid is invalid (the message names the input).</exception>
    /// <exception cref="StaticOnlyShapeException">The body is not static.</exception>
    public Shape CreateHeightField(Body body, ReadOnlySpan<float> heights, int samplesX, int samplesZ, Vector3F scale, ReadOnlySpan<bool> holes = default, ShapeDef? def = null, double flatEdgeTolerance = 0.0)
        => AddShape(body, Geom3D.HeightFieldRecipe(heights, samplesX, samplesZ, scale, holes, flatEdgeTolerance), def);

    private Shape AddShape(Body body, byte[] recipe, ShapeDef? def)
    {
        if (body.World != this) { throw new ArgumentException("The body belongs to another world.", nameof(body)); }
        var d = def ?? new ShapeDef();
        return new Shape(this, _engine.AddShape(body.Handle, recipe, in d));
    }

    // ------------------------------------------------------------------ happy path

    /// <summary>Adds a box. <paramref name="size"/> is the full size.</summary>
    public Body AddBox(Vector3F size, Vector3D at = default, QuaternionF rotation = default, BodyType type = BodyType.Dynamic, Material3D? material = null, float density = 1f)
        => AddOne(type, at, rotation, ShapeSpec3D.Box(new Vector3F(0.5f * size.X, 0.5f * size.Y, 0.5f * size.Z)), material, density);

    /// <summary>Adds a sphere.</summary>
    public Body AddSphere(float radius, Vector3D at = default, BodyType type = BodyType.Dynamic, Material3D? material = null, float density = 1f)
        => AddOne(type, at, default, ShapeSpec3D.Sphere(radius), material, density);

    /// <summary>Adds a capsule along the y axis: <paramref name="halfHeight"/> is half the length of the straight part.</summary>
    public Body AddCapsule(float halfHeight, float radius, Vector3D at = default, QuaternionF rotation = default, BodyType type = BodyType.Dynamic, Material3D? material = null, float density = 1f)
        => AddOne(type, at, rotation, ShapeSpec3D.Capsule(halfHeight, radius), material, density);

    /// <summary>Adds a cylinder along the y axis.</summary>
    public Body AddCylinder(float halfHeight, float radius, Vector3D at = default, QuaternionF rotation = default, BodyType type = BodyType.Dynamic, Material3D? material = null, float density = 1f)
        => AddOne(type, at, rotation, ShapeSpec3D.Cylinder(halfHeight, radius), material, density);

    /// <summary>Adds the convex hull of a point set.</summary>
    public Body AddConvexHull(ReadOnlySpan<Vector3F> points, Vector3D at = default, QuaternionF rotation = default, BodyType type = BodyType.Dynamic, Material3D? material = null, float density = 1f)
        => AddOne(type, at, rotation, ShapeSpec3D.ConvexHull(points), material, density);

    /// <summary>Adds a compound of convex parts as one body.</summary>
    public Body AddCompound(ReadOnlySpan<CompoundPart> parts, Vector3D at = default, QuaternionF rotation = default, BodyType type = BodyType.Dynamic, Material3D? material = null, float density = 1f)
    {
        var body = CreateBody(type, at, rotation);
        try { AddShape(body, Geom3D.CompoundRecipe(parts), ShapeFor(material, density)); }
        catch (Exception) { body.Destroy(); throw; }
        return body;
    }

    /// <summary>Adds a static triangle mesh.</summary>
    public Body AddMesh(ReadOnlySpan<Vector3F> vertices, ReadOnlySpan<int> indices, Vector3D at = default, QuaternionF rotation = default, Material3D? material = null)
    {
        var body = CreateBody(BodyType.Static, at, rotation);
        try { CreateMesh(body, vertices, indices, ShapeFor(material, 0f)); }
        catch (Exception) { body.Destroy(); throw; }
        return body;
    }

    /// <summary>Adds a static height field.</summary>
    public Body AddHeightField(ReadOnlySpan<float> heights, int samplesX, int samplesZ, Vector3F scale, Vector3D at = default, QuaternionF rotation = default, Material3D? material = null, ReadOnlySpan<bool> holes = default)
    {
        var body = CreateBody(BodyType.Static, at, rotation);
        try { CreateHeightField(body, heights, samplesX, samplesZ, scale, holes, ShapeFor(material, 0f)); }
        catch (Exception) { body.Destroy(); throw; }
        return body;
    }

    private Body AddOne(BodyType type, Vector3D at, QuaternionF rotation, ShapeSpec3D spec, Material3D? material, float density)
    {
        var body = CreateBody(type, at, rotation);
        try { CreateShape(body, spec, ShapeFor(material, density)); }
        catch (Exception) { body.Destroy(); throw; }
        return body;
    }

    private static ShapeDef ShapeFor(Material3D? material, float density)
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
    }

    internal void DestroyShape(ShapeHandle h) => _engine.DestroyShape(h);
}
