using System.Diagnostics.CodeAnalysis;
using System;
using System.Globalization;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

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
        if (o.PairCapacity > 0 || o.ContactCapacity > 0 || o.IslandCapacity > 0) { ReserveCapacity(); }
    }

    // Sizes the storage to the capacity options, and to a multiple of the body count where an option is 0 and Warmup asked for a default.
    private void ReserveCapacity(bool defaults = false)
    {
        var o = _engine.Opt;
        int bodies = Math.Max(_core.Bodies.HighWater, 1), shapes = Math.Max(_core.Shapes.HighWater, 1);
        int pairs = o.PairCapacity > 0 ? o.PairCapacity : defaults ? Math.Max(8 * bodies, 64) : 0;
        int contacts = o.ContactCapacity > 0 ? o.ContactCapacity : defaults ? Math.Max(6 * bodies, 64) : 0;
        int islands = o.IslandCapacity > 0 ? o.IslandCapacity : defaults ? Math.Max(bodies, 16) : 0;
        // Pool threads started from here on prepare their narrowphase scratch before their first step.
        WorkerPool.ThreadInit = Engine3D.PrepareThread;
        _core.Reserve(contacts, islands, bodies);
        _core.ReserveRestore(contacts, pairs, islands, bodies);
        _engine.Reserve(pairs, contacts, bodies, shapes);
        // The public contact events are built from the core's every step.
        _pairIndex.EnsureCapacity(contacts + 16);
        if (_aggs.Length < contacts + 16) { Array.Resize(ref _aggs, contacts + 16); }
        if (_publicEvents.Length < 2 * contacts + 16) { Array.Resize(ref _publicEvents, 2 * contacts + 16); }
    }

    /// <summary>
    /// Gets the world ready for allocation-free steps: sizes the storage to <see cref="WorldOptions3D.PairCapacity"/>, <see cref="WorldOptions3D.ContactCapacity"/> and <see cref="WorldOptions3D.IslandCapacity"/>
    /// (eight pairs, six contacts and one island per body where an option is 0, and never less than the world holds now), starts the worker threads, makes the calling thread's query scratch
    /// (<see cref="WorldOptions3D.QueryScratchCapacity"/>), and runs the hash, snapshot and restore paths once. Call it after the bodies exist, from the thread that will run queries.
    /// Steps, queries, <see cref="ComputeHash"/>, <see cref="TrySnapshot"/> and <see cref="Restore(ReadOnlySpan{byte})"/> then allocate nothing while the world stays within those counts and the calling thread does the querying.
    /// The world's state, events and results are unchanged (the snapshot round trip restores exactly what it saved; no callback runs). Allocates.
    /// </summary>
    public void Warmup() => WarmupCore(true);

    private void WarmupCore(bool prime)
    {
        if (prime) { PrimeProcess(_engine.Opt.ThreadCount); }
        QueryScratch.Warm(_engine.Opt.QueryScratchCapacity > 0 ? _engine.Opt.QueryScratchCapacity : 64);
        _core.ComputeHash();
        // Restoring swaps the live storage with a spare. The first round trip makes the spare the live one, so the storage is reserved again before the second.
        for (int i = 0; i < 2; i++)
        {
            ReserveCapacity(true);
            var buffer = new byte[_core.GetSnapshotSize()];
            if (!_core.TrySnapshot(buffer, out int n)) { break; }
            Restore(buffer.AsSpan(0, n));
        }

        InvalidateQueryPad();
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
    [Experimental("OPHYS0001")]
    public WorldCore Core => _core;

    /// <summary>The events of the last step in the core's deterministic order, with contact and hit events collapsed to one per pair of shapes (see docs/world3d.md). Valid until the next <see cref="Step"/>.</summary>
    public ReadOnlySpan<PhysicsEvent> Events => _publicEvents.AsSpan(0, _publicEventCount);

    internal Engine3D Engine => _engine;

    // ------------------------------------------------------------------ stepping

    /// <summary>Advances the world by a fixed time. Collision callbacks run after the step, in event order.</summary>
    /// <param name="dt">Seconds; finite and positive. Use the same value every step for reproducible results.</param>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="dt"/> is invalid or an option is out of range.</exception>
    /// <exception cref="BodyOutOfBoundsException">A moving body is outside the supported range or has a non-finite position, velocity or rotation; the world is unchanged. Move, destroy it or call <see cref="ShiftOrigin"/>.</exception>
    public void Step(float dt)
    {
        _engine.Opt.Validate();
        _engine.CheckBounds();
        SyncCoreOptions();
        _engine.StepVehicles(this, dt);
        _engine.StepFluids(dt);
        _core.Step(new Stages3D(_engine), dt);
        _engine.ReconcileJoints();
        InvalidateQueryPad();
        BuildPublicEvents();
        if (_suppressed == 0)
        {
            if (_handlers.Count > 0 || _jointHandlers.Count > 0) { DispatchEvents(); }
        }
        else if (_jointHandlers.Count > 0) { RetireBrokenJointHandlers(); }
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

    private readonly List<int> _deadScratch = new();

    /// <summary>The 64-bit hash of the whole simulation state (bodies, shapes, contacts with their warm-start impulses, islands, broadphase, options and origin; not pending callbacks). Equal states give equal hashes on every platform. Allocates nothing.</summary>
    public ulong ComputeHash() => _core.ComputeHash();

    /// <summary>The number of bytes <see cref="TrySnapshot"/> writes for the world as it is now. Allocates nothing.</summary>
    public int GetSnapshotSize() => _core.GetSnapshotSize();

    /// <summary>
    /// Writes the snapshot into <paramref name="destination"/>: byte-exact, so restoring the bytes into a world made with the same options and continuing gives identical per-step hashes, events and snapshots. Layout in docs/architecture.md.
    /// When it fits, returns true and <paramref name="written"/> is its length. When it does not, nothing is written, the result is false and <paramref name="written"/> is the length needed (see <see cref="GetSnapshotSize"/>).
    /// Allocates nothing once the world's internal buffer has grown to the snapshot's size.
    /// </summary>
    public bool TrySnapshot(Span<byte> destination, out int written)
    {
        _remembersDestroyed = true;
        return _core.TrySnapshot(destination, out written);
    }

    /// <summary>The same bytes as <see cref="TrySnapshot"/> in a new array. Delegates and collision callbacks are not saved.</summary>
    public byte[] Snapshot()
    {
        _remembersDestroyed = true;
        return _core.Snapshot();
    }

    /// <summary>The same as <see cref="Restore(ReadOnlySpan{byte})"/> for an array.</summary>
    /// <exception cref="ArgumentNullException"><paramref name="snapshot"/> is null.</exception>
    /// <returns>The saved options whose value the restore changed; see <see cref="OptionChanges"/>.</returns>
    /// <exception cref="SnapshotFormatException">The bytes are not a valid snapshot of a 3D world; the world is unchanged.</exception>
    public OptionChanges Restore(byte[] snapshot)
    {
        ArgumentNullException.ThrowIfNull(snapshot);
        return Restore((ReadOnlySpan<byte>)snapshot);
    }

    /// <summary>
    /// Replaces the world's state with a snapshot. The snapshot is validated first; on any problem a <see cref="SnapshotFormatException"/> is thrown and the world is unchanged.
    /// Handles taken before the snapshot stay valid in a world with the same <see cref="Id"/> when the same bodies exist in it. Callbacks belong to handles: a restore that brings a destroyed body, joint or character back (the same handle) gives its callbacks back, and the world forgets the callbacks of handles that stay gone (it remembers the most recent 1024 destroyed ones once it has taken a snapshot). Rollback hosts should resimulate under <see cref="SuppressCallbacks"/> and read polled events.
    /// Allocates nothing once the world's internal buffers have grown to the snapshot's size.
    /// </summary>
    /// <returns>The saved options whose value the restore changed (the snapshot's values replace the ones the world was built with); see <see cref="OptionChanges"/>.</returns>
    /// <exception cref="SnapshotFormatException">The bytes are not a valid snapshot of a 3D world; the world is unchanged.</exception>
    public OptionChanges Restore(ReadOnlySpan<byte> snapshot)
    {
        var before = _engine.Opt;
        var oldGravity = before.Gravity;
        int oldSubsteps = before.Substeps, oldVelocity = before.VelocityIterations, oldRelax = before.RelaxIterations;
        bool oldSleep = before.SleepEnabled;
        float oldTimeToSleep = before.TimeToSleep, oldThreshold = before.SleepThreshold;
        var oldEvents = before.Events;
        _core.BeginRestore(snapshot);
        string? problem = null;
        try
        {
            _core.ApplyRestore();
        }
        catch
        {
            _core.RollbackRestore();
            throw;
        }

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
            _core.RollbackRestore();
            LoadOptionsFromCore();
            throw new SnapshotFormatException(problem, 0);
        }

        var after = _engine.Opt;
        var changed = OptionChanges.None;
        if (after.Gravity != oldGravity) { changed |= OptionChanges.Gravity; }
        if (after.Substeps != oldSubsteps) { changed |= OptionChanges.Substeps; }
        if (after.VelocityIterations != oldVelocity) { changed |= OptionChanges.VelocityIterations; }
        if (after.RelaxIterations != oldRelax) { changed |= OptionChanges.RelaxIterations; }
        if (after.SleepEnabled != oldSleep) { changed |= OptionChanges.SleepEnabled; }
        if (BitConverter.SingleToInt32Bits(after.TimeToSleep) != BitConverter.SingleToInt32Bits(oldTimeToSleep)) { changed |= OptionChanges.TimeToSleep; }
        if (BitConverter.SingleToInt32Bits(after.SleepThreshold) != BitConverter.SingleToInt32Bits(oldThreshold)) { changed |= OptionChanges.SleepThreshold; }
        if (after.Events != oldEvents) { changed |= OptionChanges.Events; }

        ReconcileHandlersAfterRestore();
        InvalidateQueryPad();
        BuildPublicEvents();
        return changed;
    }

    /// <summary>Moves the world origin by <paramref name="offset"/> (call between steps, on every peer at the same step): all positions shrink by it. Offsets must be multiples of 2^-31.</summary>
    public void ShiftOrigin(Vector3D offset)
    {
        double[] d = [offset.X, offset.Y, offset.Z];
        _core.ValidateShift(d);
        _engine.CheckShift(d);
        _core.ShiftOrigin(d);
        _engine.ShiftJoints(d);
        _engine.ShiftFluids(offset.X, offset.Y, offset.Z);
        _engine.RebuildProxies();
        InvalidateQueryPad();
    }

    // ------------------------------------------------------------------ middle tier: bodies and shapes

    /// <summary>True while the body exists in this world. False for null handles, handles of another world and destroyed bodies (never throws). Allocates nothing.</summary>
    public bool IsBodyAlive(BodyHandle handle) => !handle.IsNull && handle.World == Id && _core.Bodies.Handles.IsAlive(handle.Index, handle.Generation);

    /// <summary>True while the shape exists in this world. False for null handles, handles of another world and destroyed shapes (never throws). Allocates nothing.</summary>
    public bool IsShapeAlive(ShapeHandle handle) => !handle.IsNull && handle.World == Id && _core.Shapes.Handles.IsAlive(handle.Index, handle.Generation);

    /// <summary>True while the joint exists in this world. False for null handles, handles of another world, destroyed joints and constraints that are not joints (never throws). Allocates nothing.</summary>
    public bool IsJointAlive(JointHandle handle) => !handle.IsNull && handle.World == Id && _core.Constraints.Handles.IsAlive(handle.Index, handle.Generation) && _core.Constraints.KindOf(handle.Index) == ConstraintKind.Joint;

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
    public Body CreateBody(BodyType type, Vector3D position, QuaternionF rotation = default) => CreateBody(type, position, rotation, nameof(position));

    // positionParam names the caller's parameter in the error: the happy-path methods call this with their own "at".
    private Body CreateBody(BodyType type, Vector3D position, QuaternionF rotation, string positionParam)
    {
        if (!double.IsFinite(position.X) || !double.IsFinite(position.Y) || !double.IsFinite(position.Z)) { throw new ArgumentOutOfRangeException(positionParam, position, "Body position must be finite."); }
        var d = new BodyDef { Type = type, Position = position, Rotation = rotation.LengthSquared() == 0f ? QuaternionF.Identity : rotation };
        try { return CreateBody(in d); }
        catch (ArgumentOutOfRangeException e) when (e.ParamName == "def" && e.Message.Contains("osition", StringComparison.Ordinal))
        {
            throw new ArgumentOutOfRangeException(positionParam, position, e.Message);
        }
    }

    /// <summary>Attaches a convex shape described by a <see cref="ShapeSpec3D"/>.</summary>
    /// <exception cref="ArgumentException">The shape's numbers are invalid (the message names them).</exception>
    public Shape CreateShape(Body body, ShapeSpec3D spec, ShapeDef? def = null) => AddShape(body, spec.Recipe.ToArray(), def);

    /// <summary>Attaches a sphere.</summary>
    public Shape CreateSphere(Body body, float radius, ShapeDef? def = null) => CreateShape(body, ShapeSpec3D.Sphere(radius), def);

    /// <summary>Attaches a box of the given full size (the same as <see cref="AddBox"/>).</summary>
    public Shape CreateBox(Body body, Vector3F size, float convexRadius = 0f, ShapeDef? def = null) => CreateShape(body, ShapeSpec3D.Box(size, convexRadius), def);

    /// <summary>Attaches a capsule along the body's y axis.</summary>
    public Shape CreateCapsule(Body body, float halfLength, float radius, ShapeDef? def = null) => CreateShape(body, ShapeSpec3D.Capsule(halfLength, radius), def);

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
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="at"/> is not finite or beyond +-2^21 from the origin, or a side of <paramref name="size"/> is not finite, not positive or larger than twice <see cref="ConvexShape3D.MaxDimension"/>.</exception>
    public Body AddBox(Vector3F size, Vector3D at = default, QuaternionF rotation = default, BodyType type = BodyType.Dynamic, Material3D? material = null, float density = 1f)
    {
        float max = 2f * ConvexShape3D.MaxDimension;
        if (!(float.IsFinite(size.X) && float.IsFinite(size.Y) && float.IsFinite(size.Z) && size.X > 0f && size.Y > 0f && size.Z > 0f && size.X <= max && size.Y <= max && size.Z <= max))
        {
            throw new ArgumentOutOfRangeException(nameof(size), size, "Each side of the box must be finite and within (0, " + max.ToString("R", CultureInfo.InvariantCulture) + "].");
        }

        return AddOne(type, at, rotation, ShapeSpec3D.Box(size), material, density);
    }

    /// <summary>Adds a sphere.</summary>
    public Body AddSphere(float radius, Vector3D at = default, BodyType type = BodyType.Dynamic, Material3D? material = null, float density = 1f)
        => AddOne(type, at, default, ShapeSpec3D.Sphere(radius), material, density);

    /// <summary>Adds a capsule along the y axis: <paramref name="halfLength"/> is half the length of the straight part.</summary>
    public Body AddCapsule(float halfLength, float radius, Vector3D at = default, QuaternionF rotation = default, BodyType type = BodyType.Dynamic, Material3D? material = null, float density = 1f)
        => AddOne(type, at, rotation, ShapeSpec3D.Capsule(halfLength, radius), material, density);

    /// <summary>Adds a cylinder along the y axis.</summary>
    public Body AddCylinder(float halfHeight, float radius, Vector3D at = default, QuaternionF rotation = default, BodyType type = BodyType.Dynamic, Material3D? material = null, float density = 1f)
        => AddOne(type, at, rotation, ShapeSpec3D.Cylinder(halfHeight, radius), material, density);

    /// <summary>Adds the convex hull of a point set.</summary>
    public Body AddConvexHull(ReadOnlySpan<Vector3F> points, Vector3D at = default, QuaternionF rotation = default, BodyType type = BodyType.Dynamic, Material3D? material = null, float density = 1f)
        => AddOne(type, at, rotation, ShapeSpec3D.ConvexHull(points), material, density);

    /// <summary>Adds a compound of convex parts as one body.</summary>
    public Body AddCompound(ReadOnlySpan<CompoundPart> parts, Vector3D at = default, QuaternionF rotation = default, BodyType type = BodyType.Dynamic, Material3D? material = null, float density = 1f)
    {
        var body = CreateBody(type, at, rotation, nameof(at));
        try { AddShape(body, Geom3D.CompoundRecipe(parts), ShapeFor(material, density)); }
        catch (Exception) { body.Destroy(); throw; }
        return body;
    }

    /// <summary>Adds a static triangle mesh.</summary>
    public Body AddMesh(ReadOnlySpan<Vector3F> vertices, ReadOnlySpan<int> indices, Vector3D at = default, QuaternionF rotation = default, Material3D? material = null)
    {
        var body = CreateBody(BodyType.Static, at, rotation, nameof(at));
        try { CreateMesh(body, vertices, indices, ShapeFor(material, 0f)); }
        catch (Exception) { body.Destroy(); throw; }
        return body;
    }

    /// <summary>Adds a static height field.</summary>
    public Body AddHeightField(ReadOnlySpan<float> heights, int samplesX, int samplesZ, Vector3F scale, Vector3D at = default, QuaternionF rotation = default, Material3D? material = null, ReadOnlySpan<bool> holes = default)
    {
        var body = CreateBody(BodyType.Static, at, rotation, nameof(at));
        try { CreateHeightField(body, heights, samplesX, samplesZ, scale, holes, ShapeFor(material, 0f)); }
        catch (Exception) { body.Destroy(); throw; }
        return body;
    }

    private Body AddOne(BodyType type, Vector3D at, QuaternionF rotation, ShapeSpec3D spec, Material3D? material, float density)
    {
        var body = CreateBody(type, at, rotation, nameof(at));
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
        RetireBodyHandlers(h);
    }

    internal void DestroyShape(ShapeHandle h) => _engine.DestroyShape(h);
}
