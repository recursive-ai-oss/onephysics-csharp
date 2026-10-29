using System.Diagnostics.CodeAnalysis;
using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

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
        var bp = new Broadphase2D(new BroadphaseOptions { Margin = WorldCore.Grid(o.BroadphaseMargin), EnlargedRebuildShift = 1, EnlargedRefitSteps = 7 });
        _engine = new Engine2D(_core, bp, o) { World = this };
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
        _core.Reserve(contacts, islands, bodies);
        _core.ReserveRestore(contacts, pairs, islands, bodies);
        _engine.Reserve(pairs, contacts, bodies, shapes);
        // Every pool thread, including those an earlier world started, prepares its query scratch now instead of at whichever step first gives it a batch query.
        WorkerPool.AddThreadInit(PrepareWorkerThread);
    }

    private static void PrepareWorkerThread() => QueryScratch.Warm(64);

    /// <summary>
    /// Gets the world ready for allocation-free steps: sizes the storage to <see cref="WorldOptions2D.PairCapacity"/>, <see cref="WorldOptions2D.ContactCapacity"/> and <see cref="WorldOptions2D.IslandCapacity"/>
    /// (eight pairs, six contacts and one island per body where an option is 0, and never less than the world holds now), starts the worker threads, makes the calling thread's query scratch
    /// (<see cref="WorldOptions2D.QueryScratchCapacity"/>), and runs the hash, snapshot and restore paths once. Call it after the bodies exist, from the thread that will run queries.
    /// Steps, queries, <see cref="ComputeHash"/>, <see cref="TrySnapshot"/> and <see cref="Restore(ReadOnlySpan{byte})"/> then allocate nothing while the world stays within those counts and the calling thread does the querying.
    /// The world's state, events and results are unchanged (the snapshot round trip restores exactly what it saved; no callback runs). Allocates.
    /// The first call in a process also runs a small trial world that lands, settles and sleeps a pile and runs every kind of query, character move and trigger, so those code paths are loaded before your first step.
    /// A path the trial world does not reach is loaded by the .NET runtime on the calling thread the first time it runs, and the runtime allocates a few kilobytes doing so; that is the runtime's, not memory the world holds.
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

    /// <summary>The live options. Change a value between steps and the next step uses it.</summary>
    public WorldOptions2D Options => _engine.Opt;

    /// <summary>Number of completed steps.</summary>
    public long StepCount => _core.StepCount;

    /// <summary>The id stamped into this world's handles.</summary>
    public int Id => _core.Id;

    /// <summary>The origin offset accumulated by <see cref="ShiftOrigin"/>; a position in host coordinates is the body's position plus this.</summary>
    public Vector2D Origin => new(_core.Origin[0], _core.Origin[1]);

    /// <summary>The raw tier: the dimension-independent core this world drives (bodies, constraints, islands, events). Read it freely; changing it behind the engine's back is unsupported.</summary>
    [Experimental("OPHYS0001")]
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
    /// <exception cref="BodyOutOfBoundsException">A moving body's bounds are beyond ±2^22 or not finite, or its position, velocity or rotation is not finite; nothing was changed.</exception>
    public void Step(float dt)
    {
        _engine.Opt.Validate();
        _engine.CheckBounds();
        SyncCoreOptions();
        _core.Step(new Stages2D(_engine), dt);
        InvalidateQueryPad();
        _engine.ReconcileJoints();
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

    private readonly List<int> _deadScratch = new();

    /// <summary>The 64-bit hash of the whole simulation state (bodies, shapes, contacts with their warm-start impulses, islands, broadphase, options and origin; not pending callbacks). Equal states give equal hashes on every platform. The state is streamed through the hash as it is walked; no snapshot is built or copied. Allocates nothing.</summary>
    public ulong ComputeHash() => _core.ComputeHash();

    /// <summary>The number of bytes <see cref="TrySnapshot"/> writes for the world as it is now. Allocates nothing.</summary>
    public int GetSnapshotSize() => _core.GetSnapshotSize();

    /// <summary>
    /// Writes the snapshot into <paramref name="destination"/>: byte-exact, so restoring the bytes into a world made with the same options and continuing gives identical per-step hashes, events and snapshots. Layout in docs/spec/architecture.md.
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
    /// <exception cref="SnapshotFormatException">The bytes are not a valid snapshot of a 2D world; the world is unchanged.</exception>
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
    /// <exception cref="SnapshotFormatException">The bytes are not a valid snapshot of a 2D world; the world is unchanged.</exception>
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

        InvalidateQueryPad();
        ReconcileHandlersAfterRestore();
        return changed;
    }

    /// <summary>
    /// Moves the world origin: every body position decreases by <paramref name="offset"/> and <see cref="Origin"/> grows by it, so a host position (origin + position) does not change.
    /// Call it between steps on every peer at the same step. Components must be multiples of 2^-31 (every 31.32 fixed-point value is) within +-2^22, otherwise an exception names the value.
    /// </summary>
    public void ShiftOrigin(Vector2D offset)
    {
        double[] d = [offset.X, offset.Y];
        _core.ValidateShift(d);
        _engine.CheckShift(d);
        _core.ShiftOrigin(d);
        _engine.ShiftProxies(d);
        InvalidateQueryPad();
    }

    // ------------------------------------------------------------------ middle tier: bodies and shapes

    /// <summary>True while the body exists in this world. False for null handles, handles of another world and destroyed bodies (never throws). Allocates nothing.</summary>
    public bool IsBodyAlive(BodyHandle handle) => !handle.IsNull && handle.World == Id && _core.Bodies.Handles.IsAlive(handle.Index, handle.Generation);

    /// <summary>True while the shape exists in this world. False for null handles, handles of another world and destroyed shapes (never throws). Allocates nothing.</summary>
    public bool IsShapeAlive(ShapeHandle handle) => !handle.IsNull && handle.World == Id && _core.Shapes.Handles.IsAlive(handle.Index, handle.Generation);

    /// <summary>True while the joint exists in this world. False for null handles, handles of another world, destroyed joints and constraints that are not joints (never throws). Allocates nothing.</summary>
    public bool IsJointAlive(JointHandle handle) => !handle.IsNull && handle.World == Id && _core.Constraints.Handles.IsAlive(handle.Index, handle.Generation) && _core.Constraints.KindOf(handle.Index) == ConstraintKind.Joint;

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
    public Body CreateBody(BodyType type, Vector2D position, float angleRadians = 0f) => CreateBody(type, position, angleRadians, nameof(position));

    // positionParam names the caller's parameter in the error: the happy-path methods call this with their own "at".
    private Body CreateBody(BodyType type, Vector2D position, float angleRadians, string positionParam)
    {
        if (!double.IsFinite(position.X) || !double.IsFinite(position.Y)) { throw new ArgumentOutOfRangeException(positionParam, position, "Body position must be finite."); }
        if (!float.IsFinite(angleRadians)) { throw new ArgumentOutOfRangeException(nameof(angleRadians), angleRadians, "Body angle must be finite."); }
        var d = new BodyDef { Type = type, Position = position, Angle = angleRadians };
        try { return CreateBody(in d); }
        catch (ArgumentOutOfRangeException e) when (e.ParamName == "def" && e.Message.Contains("osition", StringComparison.Ordinal))
        {
            // Only the position range is left to fail here (the engine checks the +-2^21 bound).
            throw new ArgumentOutOfRangeException(positionParam, position, e.Message);
        }
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

    /// <summary>Adds a box of the given full size (width, height, the same as <see cref="AddBox"/>), optionally offset and rotated within the body's frame and rounded by <paramref name="radius"/>.</summary>
    /// <param name="body">The body that gets the shape.</param>
    /// <param name="size">Full width and height in metres.</param>
    /// <param name="center">Centre of the box in the body's frame.</param>
    /// <param name="angleRadians">Rotation of the box in the body's frame, radians.</param>
    /// <param name="radius">Corner rounding radius, metres; the box grows by it on every side.</param>
    /// <param name="def">Shape settings; null means the defaults.</param>
    public Shape CreateBox(Body body, Vector2F size, Vector2F center = default, float angleRadians = 0f, float radius = 0f, ShapeDef? def = null)
    {
        var (s, c) = RecursiveAI.OneMath.Math64.SinCos(angleRadians);
        return AddShape(body, Shape2D.CreateBox(0.5f * size.X, 0.5f * size.Y, new Vec2(center.X, center.Y), new Rot2((float)c, (float)s), radius), def);
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
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="at"/> or <paramref name="angleRadians"/> is not finite, or <paramref name="at"/> is beyond +-2^21 from the origin.</exception>
    /// <exception cref="InvalidShapeException">A side of <paramref name="size"/> is not finite or is smaller than 2E-05 (its <see cref="ArgumentException.ParamName"/> is <c>size</c>).</exception>
    public Body AddBox(Vector2F size, Vector2D at = default, float angleRadians = 0f, BodyType type = BodyType.Dynamic, Material2D? material = null, float density = 1f)
    {
        CheckSize(size, nameof(size));
        var body = CreateBody(type, at, angleRadians, nameof(at));
        try { CreateBox(body, size, default, 0f, 0f, ShapeFor(material, density)); }
        catch (InvalidShapeException e) { body.Destroy(); throw e.ForParameter(nameof(size)); }
        catch (Exception) { body.Destroy(); throw; }
        return body;
    }

    // The box sides are full sizes here; the shape code speaks of half extents, so the happy path states its own limits in the caller's terms.
    private static void CheckSize(Vector2F size, string param)
    {
        float min = 2f * Constants2D.MinShapeExtent, max = 2f * Constants2D.MaxShapeCoordinate;
        if (!float.IsFinite(size.X) || !float.IsFinite(size.Y))
        {
            throw new InvalidShapeException(ShapeError.NotFinite, "size is (" + size.X.ToString("R", CultureInfo.InvariantCulture) + ", " + size.Y.ToString("R", CultureInfo.InvariantCulture) + "); both sides must be finite", param);
        }

        if (!(size.X >= min && size.Y >= min && size.X <= max && size.Y <= max))
        {
            throw new InvalidShapeException(ShapeError.OutOfRange, "size is (" + size.X.ToString("R", CultureInfo.InvariantCulture) + ", " + size.Y.ToString("R", CultureInfo.InvariantCulture) + "); each side must be within " + min.ToString("R", CultureInfo.InvariantCulture) + ".." + max.ToString("R", CultureInfo.InvariantCulture), param);
        }
    }

    /// <summary>Adds a circle centred at <paramref name="at"/>. Dynamic by default.</summary>
    public Body AddCircle(float radius, Vector2D at = default, BodyType type = BodyType.Dynamic, Material2D? material = null, float density = 1f)
    {
        var body = CreateBody(type, at, 0f, nameof(at));
        try { CreateCircle(body, radius, default, ShapeFor(material, density)); }
        catch (InvalidShapeException e) { body.Destroy(); throw e.ForParameter(nameof(radius)); }
        catch (Exception) { body.Destroy(); throw; }
        return body;
    }

    /// <summary>Adds a capsule whose axis runs from (-halfLength, 0) to (halfLength, 0) in the body's frame. Dynamic by default.</summary>
    public Body AddCapsule(float halfLength, float radius, Vector2D at = default, float angleRadians = 0f, BodyType type = BodyType.Dynamic, Material2D? material = null, float density = 1f)
    {
        var body = CreateBody(type, at, angleRadians, nameof(at));
        try { CreateCapsule(body, new Vector2F(-halfLength, 0f), new Vector2F(halfLength, 0f), radius, ShapeFor(material, density)); }
        catch (InvalidShapeException e)
        {
            body.Destroy();
            throw e.ForParameter(e.Message.StartsWith("radius", StringComparison.Ordinal) ? nameof(radius) : nameof(halfLength), e.Message.StartsWith("radius", StringComparison.Ordinal) ? null : "halfLength is " + halfLength.ToString("R", CultureInfo.InvariantCulture) + "; the capsule's two end points must be at least " + Constants2D.MinShapeExtent.ToString("R", CultureInfo.InvariantCulture) + " apart (halfLength at least " + (0.5f * Constants2D.MinShapeExtent).ToString("R", CultureInfo.InvariantCulture) + ")");
        }
        catch (Exception) { body.Destroy(); throw; }
        return body;
    }

    /// <summary>Adds the convex hull of up to 8 points (in the body's frame), rounded by <paramref name="radius"/>. Dynamic by default.</summary>
    public Body AddPolygon(ReadOnlySpan<Vector2F> points, Vector2D at = default, float angleRadians = 0f, float radius = 0f, BodyType type = BodyType.Dynamic, Material2D? material = null, float density = 1f)
    {
        var body = CreateBody(type, at, angleRadians, nameof(at));
        try { CreatePolygon(body, points, radius, ShapeFor(material, density)); }
        catch (InvalidShapeException e) { body.Destroy(); throw e.ForParameter(e.Message.StartsWith("radius", StringComparison.Ordinal) ? nameof(radius) : nameof(points)); }
        catch (ArgumentException) { body.Destroy(); throw; }
        return body;
    }

    /// <summary>Adds a static two-sided segment between two world points.</summary>
    public Body AddSegment(Vector2D a, Vector2D b, Material2D? material = null)
    {
        var body = CreateBody(BodyType.Static, default);
        try { CreateSegment(body, a.ToSingle(), b.ToSingle(), ShapeFor(material, 0f)); }
        catch (InvalidShapeException e) { body.Destroy(); throw e.ForParameter(nameof(a)); }
        catch (Exception) { body.Destroy(); throw; }
        return body;
    }

    /// <summary>Adds a static chain through world points (two-sided unless <paramref name="twoSided"/> is false, then solid on the right of the direction of travel).</summary>
    public Body AddChain(ReadOnlySpan<Vector2D> points, bool loop = false, bool twoSided = true, Material2D? material = null)
    {
        var p = new Vector2F[points.Length];
        for (int i = 0; i < p.Length; i++) { p[i] = points[i].ToSingle(); }
        var body = CreateBody(BodyType.Static, default);
        try { CreateChain(body, p, loop, twoSided, ShapeFor(material, 0f)); }
        catch (InvalidShapeException e) { body.Destroy(); throw e.ForParameter(nameof(points)); }
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
        RetireBodyHandlers(h);
        if (_controllerHandlers.Count > 0) { RetireDeadControllerHandlers(); }
    }

    internal void DestroyShape(ShapeHandle h) => _engine.DestroyShape(h);
}
