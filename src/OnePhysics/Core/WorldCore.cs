using System;
using System.Collections.Generic;
using System.Globalization;

namespace RecursiveAI.OnePhysics.Core;

/// <summary>The phases the solver runs on every batch, in each substep (see <see cref="IWorldStages"/>).</summary>
public enum SolvePhase : byte
{
    /// <summary>Apply the impulses accumulated in the previous step.</summary>
    WarmStart = 0,

    /// <summary>Solve velocity constraints with position bias (soft constraints).</summary>
    Solve = 1,

    /// <summary>Solve again without bias to remove the energy the bias added.</summary>
    Relax = 2,

    /// <summary>Once per step after the last substep: restitution.</summary>
    Restitution = 3,
}

/// <summary>Timing and counts of the running step, passed to every stage call.</summary>
public readonly struct StepContext
{
    /// <summary>The step's time step in seconds.</summary>
    public float Dt { get; }

    /// <summary>Time step of one substep: <c>Dt / Substeps</c>.</summary>
    public float SubstepDt { get; }

    /// <summary>The reciprocal of <see cref="SubstepDt"/>, computed once so every stage uses the same bits.</summary>
    public float InverseSubstepDt { get; }

    /// <summary>Substeps in this step.</summary>
    public int Substeps { get; }

    internal StepContext(float dt, int substeps)
    {
        Dt = dt;
        Substeps = substeps;
        SubstepDt = dt / substeps;
        InverseSubstepDt = 1f / SubstepDt;
    }
}

/// <summary>
/// The dimension-specific part of a step: narrowphase, constraint preparation, solving, integration. The core owns the pipeline and the order
/// (<c>UpdateContacts → Prepare → [IntegrateVelocities → WarmStart → Solve×N → IntegratePositions → Relax×M] × substeps → Restitution → StoreResults → events</c>) and calls these methods; a 2D or 3D engine,
/// or a test engine, supplies the implementation as a type argument so the calls are specialized and inlinable.
/// </summary>
/// <remarks>
/// <para>Every method except <see cref="UpdateContacts"/> and <see cref="BodyMotion"/> may be called concurrently from several threads on disjoint data. Implementations must therefore only write
/// the bodies and constraints they are handed, and keep any mutable state of their own in reference-type fields written serially (an implementing struct is copied).
/// Constraint batches never contain two constraints that share a dynamic body, so solving a batch may update both bodies freely; static and kinematic bodies are read-only for solvers.</para>
/// <para><see cref="UpdateContacts"/> runs serially: it may use <see cref="WorldCore.Executor"/> internally for a parallel narrowphase, but must apply its additions and removals in an order that depends only on the data (ascending keys), because the coloring depends on the order constraints are added.</para>
/// </remarks>
public interface IWorldStages
{
    /// <summary>Broadphase pairs and narrowphase: add, remove and update contact, sensor and joint constraints with <see cref="WorldCore.AddConstraint"/> and <see cref="WorldCore.RemoveConstraint"/>.</summary>
    void UpdateContacts(WorldCore world);

    /// <summary>Prepares a chunk of the active constraints (anchors, effective masses) once per step.</summary>
    void Prepare(WorldCore world, ReadOnlySpan<int> constraints, in StepContext context);

    /// <summary>Applies forces and gravity to the velocities of a chunk of awake bodies (ascending). Kinematic bodies are included and should be left alone.</summary>
    void IntegrateVelocities(WorldCore world, ReadOnlySpan<int> bodies, in StepContext context);

    /// <summary>Solves one batch (at most <see cref="ConstraintColoring.Lanes"/> constraints of one color, ascending, or a slice of the overflow set) in the given phase.</summary>
    void SolveBatch(WorldCore world, ReadOnlySpan<int> constraints, SolvePhase phase, in StepContext context);

    /// <summary>Advances the positions and rotations of a chunk of awake bodies (ascending), kinematic bodies included.</summary>
    void IntegratePositions(WorldCore world, ReadOnlySpan<int> bodies, in StepContext context);

    /// <summary>After the last substep: stores results for the next step's warm start, and may request joint breaks by setting <see cref="ConstraintBits.BreakRequested"/> and writing the break value. <paramref name="eventSlot"/> is this call's own slot for <see cref="EventCollector.Emit"/>.</summary>
    void StoreResults(WorldCore world, ReadOnlySpan<int> constraints, int eventSlot, in StepContext context);

    /// <summary>How much a body moves, in the units of <see cref="WorldOptions.SleepThreshold"/> (typically a speed). Called serially for each awake dynamic body in ascending order.</summary>
    float BodyMotion(WorldCore world, int body);

    /// <summary>
    /// After the restitution pass and before <see cref="StoreResults"/>: per-body work that needs the final pose of the step, such as continuous collision against static geometry or clearing applied forces.
    /// Called over chunks of awake bodies (ascending), possibly concurrently, so it may only write the bodies it is handed. The default does nothing.
    /// </summary>
    void FinalizeBodies(WorldCore world, ReadOnlySpan<int> bodies, in StepContext context) { }

    /// <summary>
    /// Once per step after the constraints are colored and before anything is prepared: <paramref name="batches"/> is the number of batches the solver will see (the overflow set cut into groups of <see cref="ConstraintColoring.Lanes"/>,
    /// then each color cut into groups of that size). The default does nothing.
    /// </summary>
    void BeginSolve(WorldCore world, int batches, in StepContext context) { }

    /// <summary>
    /// Prepares one batch (after every chunk has been prepared with <see cref="Prepare"/>), cut exactly as <see cref="SolveBatch"/> will see it; <paramref name="batch"/> numbers the batches of the step from zero in solve order. Batches of one color may be prepared concurrently,
    /// so an implementation writes only data owned by its batch. The default does nothing.
    /// </summary>
    void PrepareBatch(WorldCore world, ReadOnlySpan<int> constraints, int batch, in StepContext context) { }

    /// <summary>
    /// Once, serially, after every <see cref="FinalizeBodies"/> call of the step has finished and before <see cref="StoreResults"/>: work that reads the final pose of other bodies, such as bullets
    /// sweeping against bodies that moved this step. It may use <see cref="WorldCore.Executor"/> for its own parallel work. The default does nothing.
    /// </summary>
    void AfterFinalize(WorldCore world, in StepContext context) { }
}

/// <summary>
/// The dimension-independent simulation core: body, shape and constraint storage, deterministic coloring and scheduling, islands and sleeping, events, snapshot and restore, and the world hash.
/// An engine (2D or 3D) owns one, adds its shape data and algorithms through <see cref="IWorldStages"/> and <see cref="ISnapshotSection"/>, and drives it with <see cref="Step{TStages}"/>.
/// All mutation happens on one thread between or inside steps as documented; only the phases that call the stages run in parallel.
/// </summary>
public sealed class WorldCore
{
    private const int ChunkSize = 64;
    private const double CoordinateBound = 2097152.0; // 2^21
    private const double OffsetScale = 2147483648.0;  // 2^31: shifts are multiples of 2^-31
    private const double OffsetBound = 4194304.0;     // 2^22
    private const uint Magic = 0x4E53504F; // "OPSN" little-endian
    private const uint FormatVersion = 1;

    private static int s_nextWorldId;
    private static readonly object s_idLock = new();

    private readonly WorldOptions _options;
    private BodyStore _bodies;
    private ShapeStore _shapes;
    private ConstraintStore _constraints;
    private IslandSet _islands;
    private readonly ConstraintColoring _coloring = new();
    private readonly EventCollector _events;
    private readonly List<ISnapshotSection> _sections = new();
    private double[] _origin;
    private long _stepCount;
    private int[] _awake = new int[64];
    private int _awakeCount;
    private readonly List<int> _scratch = new();

    /// <summary>Creates a world. The options are copied and validated (<see cref="WorldOptions.Validate"/>).</summary>
    public WorldCore(WorldOptions? options = null)
    {
        _options = (options ?? new WorldOptions()).Clone();
        _options.Validate();
        if (_options.WorldId == 0)
        {
            lock (s_idLock) { _options.WorldId = ++s_nextWorldId; }
        }

        Executor = new TaskExecutor(_options.ThreadCount);
        Layout = _options.Layout;
        _bodies = new BodyStore(Layout);
        _shapes = new ShapeStore();
        _constraints = new ConstraintStore(Layout);
        _islands = new IslandSet();
        _events = new EventCollector(_options);
        _origin = new double[Layout.PositionLanes];
    }

    /// <summary>The id stamped into this world's handles.</summary>
    public int Id => _options.WorldId;

    /// <summary>The options in force. They may be changed between steps (they are validated at the start of each step); restoring a snapshot overwrites the simulation values with the saved ones. <see cref="WorldOptions.ThreadCount"/> is fixed when the world is created.</summary>
    public WorldOptions Options => _options;

    /// <summary>The storage layout.</summary>
    public DimensionLayout Layout { get; }

    /// <summary>The executor stages may use for their own parallel work.</summary>
    public TaskExecutor Executor { get; }

    /// <summary>Body storage. Fetch again after <see cref="Restore"/>.</summary>
    public BodyStore Bodies => _bodies;

    /// <summary>Shape slots. Fetch again after <see cref="Restore"/>.</summary>
    public ShapeStore Shapes => _shapes;

    /// <summary>Constraint storage. Fetch again after <see cref="Restore"/>.</summary>
    public ConstraintStore Constraints => _constraints;

    /// <summary>The islands. Fetch again after <see cref="Restore"/>.</summary>
    public IslandSet Islands => _islands;

    /// <summary>The coloring and the active constraint lists of the current step.</summary>
    public ConstraintColoring Coloring => _coloring;

    /// <summary>Event collection and the last step's delivered events.</summary>
    public EventCollector Events => _events;

    /// <summary>Steps taken so far (saved in snapshots).</summary>
    public long StepCount => _stepCount;

    /// <summary>Accumulated origin shift: add to a stored position to get the host's coordinate.</summary>
    public ReadOnlySpan<double> Origin => _origin;

    /// <summary>The awake bodies ascending: awake dynamic bodies and all kinematic bodies. During <see cref="IWorldStages.UpdateContacts"/> the list is as of the start of the step; afterwards it includes bodies the new constraints woke.</summary>
    public ReadOnlySpan<int> AwakeBodies => _awake.AsSpan(0, _awakeCount);

    /// <summary>The array behind <see cref="AwakeBodies"/>, for parallel tasks (which cannot hold a span).</summary>
    internal int[] AwakeArray => _awake;

    /// <summary>Registers extra state saved, restored and hashed after the core's own. Sections are written in registration order and must be registered identically in a world that restores the snapshot.</summary>
    public void AddSection(ISnapshotSection section)
    {
        ArgumentNullException.ThrowIfNull(section);
        foreach (var s in _sections)
        {
            if (s.SectionId == section.SectionId) { throw new ArgumentException("A section with id " + section.SectionId.ToString(CultureInfo.InvariantCulture) + " is already registered.", nameof(section)); }
        }

        _sections.Add(section);
    }

    // ------------------------------------------------------------------ handles

    /// <summary>The handle of a live body slot.</summary>
    public BodyHandle HandleOfBody(int body) => new(Id, body, _bodies.Handles.GenerationOf(body));

    /// <summary>The handle of a live shape slot.</summary>
    public ShapeHandle HandleOfShape(int shape) => new(Id, shape, _shapes.Handles.GenerationOf(shape));

    /// <summary>The handle of a live joint (constraint) slot.</summary>
    public JointHandle HandleOfJoint(int constraint) => new(Id, constraint, _constraints.Handles.GenerationOf(constraint));

    /// <summary>Checks a body handle and returns its slot. Throws <see cref="InvalidHandleException"/> (null handle), <see cref="ForeignHandleException"/> or <see cref="StaleHandleException"/>.</summary>
    public int Resolve(BodyHandle h)
    {
        Check(h.IsNull, h.World, h.Index, h.Generation, _bodies.Handles, "body", h.ToString());
        return h.Index;
    }

    /// <summary>Checks a shape handle and returns its slot; throws as <see cref="Resolve(BodyHandle)"/>.</summary>
    public int Resolve(ShapeHandle h)
    {
        Check(h.IsNull, h.World, h.Index, h.Generation, _shapes.Handles, "shape", h.ToString());
        return h.Index;
    }

    /// <summary>Checks a joint handle and returns its constraint slot; throws as <see cref="Resolve(BodyHandle)"/>, and also when the slot holds a constraint that is not a joint.</summary>
    public int Resolve(JointHandle h)
    {
        Check(h.IsNull, h.World, h.Index, h.Generation, _constraints.Handles, "joint", h.ToString());
        if (_constraints.KindOf(h.Index) != ConstraintKind.Joint) { throw new InvalidHandleException(h + " does not refer to a joint."); }
        return h.Index;
    }

    private void Check(bool isNull, int world, int index, int generation, HandleTable table, string what, string text)
    {
        if (isNull) { throw new InvalidHandleException("The null " + what + " handle was used."); }
        if (world != Id) { throw new ForeignHandleException(text + " belongs to world " + world.ToString(CultureInfo.InvariantCulture) + ", not world " + Id.ToString(CultureInfo.InvariantCulture) + "."); }
        if (!table.IsAlive(index, generation)) { throw new StaleHandleException(text + " is stale: the " + what + " was destroyed."); }
    }

    // ------------------------------------------------------------------ bodies and shapes

    /// <summary>
    /// Creates a body. Positions are relative to the world origin and must be finite with magnitude at most 2^21; rotation must be finite and normalized within 1e-3 (defaults to identity);
    /// velocities finite (default zero). A dynamic body needs a finite positive <paramref name="inverseMass"/>; static and kinematic bodies get zero inverse mass and inertia whatever is passed.
    /// <paramref name="inverseInertia"/> defaults to the identity tensor (the diagonal lanes are 1).
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">A position component beyond the bound, a non-finite value, or a non-positive inverse mass for a dynamic body.</exception>
    /// <exception cref="ArgumentException">A span of the wrong length.</exception>
    public BodyHandle CreateBody(BodyType type, ReadOnlySpan<double> position, ReadOnlySpan<float> rotation = default, ReadOnlySpan<float> linearVelocity = default,
        ReadOnlySpan<float> angularVelocity = default, float inverseMass = 1f, ReadOnlySpan<float> inverseInertia = default, BodyBits flags = BodyBits.None)
    {
        var l = Layout;
        if (type != BodyType.Static && type != BodyType.Kinematic && type != BodyType.Dynamic) { throw new ArgumentOutOfRangeException(nameof(type), type, "Unknown body type."); }
        Lanes(position.Length, l.PositionLanes, nameof(position), false);
        Lanes(rotation.Length, l.RotationLanes, nameof(rotation), true);
        Lanes(linearVelocity.Length, l.LinearLanes, nameof(linearVelocity), true);
        Lanes(angularVelocity.Length, l.AngularLanes, nameof(angularVelocity), true);
        Lanes(inverseInertia.Length, l.InertiaLanes, nameof(inverseInertia), true);
        for (int i = 0; i < position.Length; i++)
        {
            double p = position[i];
            if (!double.IsFinite(p) || Math.Abs(p) > CoordinateBound)
            {
                throw new ArgumentOutOfRangeException(nameof(position), p, "Position component " + i.ToString(CultureInfo.InvariantCulture) + " = " + p.ToString("R", CultureInfo.InvariantCulture) + " is outside the supported range of +-2^21 (2097152) relative to the world origin.");
            }
        }

        RequireFinite(rotation, nameof(rotation));
        RequireFinite(linearVelocity, nameof(linearVelocity));
        RequireFinite(angularVelocity, nameof(angularVelocity));
        RequireFinite(inverseInertia, nameof(inverseInertia));
        if (rotation.Length > 0)
        {
            float n2 = 0f;
            foreach (float q in rotation) { n2 += q * q; }
            if (!(Math.Abs(n2 - 1f) <= 1e-3f)) { throw new ArgumentException("rotation must be normalized (squared length within 1e-3 of 1), was " + n2.ToString("R", CultureInfo.InvariantCulture) + ".", nameof(rotation)); }
        }

        if (type == BodyType.Dynamic && !(inverseMass > 0f && float.IsFinite(inverseMass)))
        {
            throw new ArgumentOutOfRangeException(nameof(inverseMass), inverseMass, "A dynamic body needs a finite positive inverse mass.");
        }

        int b = _bodies.AllocateSlot();
        _bodies.ClearSlot(b);
        position.CopyTo(_bodies.PositionOf(b));
        if (rotation.Length > 0) { rotation.CopyTo(_bodies.RotationOf(b)); } else { l.WriteIdentityRotation(_bodies.RotationOf(b)); }
        linearVelocity.CopyTo(_bodies.LinearVelocityOf(b));
        angularVelocity.CopyTo(_bodies.AngularVelocityOf(b));
        var flagBits = (byte)(((byte)flags & (byte)(BodyBits.Bullet | BodyBits.NoSleep)) | (byte)BodyBits.Awake);
        _bodies.Type[b] = (byte)type;
        _bodies.Flags[b] = flagBits;
        _coloring.EnsureBodies(_bodies.HighWater);
        _coloring.ResetBody(b);
        if (type == BodyType.Dynamic)
        {
            _bodies.InverseMass[b] = inverseMass;
            var inv = _bodies.InverseInertiaOf(b);
            if (inverseInertia.Length > 0) { inverseInertia.CopyTo(inv); }
            else if (l.InertiaLanes == 6) { inv[0] = 1f; inv[3] = 1f; inv[5] = 1f; }
            else { inv[0] = 1f; }
            _islands.CreateFor(_bodies, b);
        }

        return HandleOfBody(b);
    }

    private static void Lanes(int actual, int expected, string name, bool optional)
    {
        if (actual != expected && !(optional && actual == 0))
        {
            throw new ArgumentException(name + " needs " + expected.ToString(CultureInfo.InvariantCulture) + " components but has " + actual.ToString(CultureInfo.InvariantCulture) + ".", name);
        }
    }

    private static void RequireFinite(ReadOnlySpan<float> v, string name)
    {
        for (int i = 0; i < v.Length; i++)
        {
            if (!float.IsFinite(v[i])) { throw new ArgumentOutOfRangeException(name, v[i], "Component " + i.ToString(CultureInfo.InvariantCulture) + " of " + name + " is not finite."); }
        }
    }

    /// <summary>Destroys a body, its shapes and every constraint touching it (their end events are queued). Islands touching it wake. Cost: proportional to the body's constraints if it is dynamic, to all constraints and shapes otherwise.</summary>
    public void DestroyBody(BodyHandle handle)
    {
        int b = Resolve(handle);
        _scratch.Clear();
        if (_bodies.IsDynamic(b))
        {
            for (int e = _bodies.FirstEdge(b); e >= 0; e = _constraints.NextEdge(e)) { _scratch.Add(e >> 1); }
        }
        else
        {
            for (int c = 0; c < _constraints.HighWater; c++)
            {
                if (_constraints.IsLive(c) && (_constraints.BodyA[c] == b || _constraints.BodyB[c] == b)) { _scratch.Add(c); }
            }
        }

        _scratch.Sort();
        var doomed = _scratch.ToArray();
        foreach (int c in doomed) { RemoveConstraint(c); }
        for (int s = 0; s < _shapes.HighWater; s++)
        {
            if (_shapes.Handles.IsLive(s) && _shapes.Owner[s] == b) { _shapes.FreeSlot(s); }
        }

        _islands.RemoveBody(_bodies, b);
        _bodies.ClearSlot(b);
        _bodies.Handles.Free(b);
    }

    /// <summary>Attaches a shape slot to a body. The geometry itself lives in the engine's own arrays indexed by the returned slot.</summary>
    public ShapeHandle CreateShape(BodyHandle owner)
    {
        int b = Resolve(owner);
        return HandleOfShape(_shapes.AllocateSlot(b));
    }

    /// <summary>Destroys a shape and the constraints that reference it (cost: all constraints).</summary>
    public void DestroyShape(ShapeHandle handle)
    {
        int s = Resolve(handle);
        _scratch.Clear();
        for (int c = 0; c < _constraints.HighWater; c++)
        {
            if (_constraints.IsLive(c) && (_constraints.ShapeA[c] == s && _constraints.ShapeGenerationA(c) == handle.Generation || _constraints.ShapeB[c] == s && _constraints.ShapeGenerationB(c) == handle.Generation)) { _scratch.Add(c); }
        }

        var doomed = _scratch.ToArray();
        foreach (int c in doomed) { RemoveConstraint(c); }
        _shapes.FreeSlot(s);
    }

    /// <summary>Wakes the island of a body (a no-op for static and kinematic bodies and for bodies already awake).</summary>
    public void Wake(BodyHandle handle)
    {
        int b = Resolve(handle);
        if (_bodies.IsDynamic(b)) { WakeIsland(_bodies.Island[b]); }
    }

    /// <summary>Puts the island of a dynamic body to sleep now (a no-op for static and kinematic bodies and for islands already asleep): velocities are zeroed and one sleep event per body is queued.  The island wakes again as soon as anything touches it.</summary>
    public void Sleep(BodyHandle handle)
    {
        int b = Resolve(handle);
        if (_bodies.IsDynamic(b) && !_islands.IsAsleep(_bodies.Island[b])) { SleepIsland(_bodies.Island[b]); }
    }

    /// <summary>Wakes an island: its bodies become awake with sleep timers reset, and one wake event per body is queued.</summary>
    public void WakeIsland(int island)
    {
        if (!_islands.IsAsleep(island)) { return; }
        _islands.SetAsleep(island, false);
        foreach (int b in _islands.Bodies(island))
        {
            _bodies.Flags[b] |= (byte)BodyBits.Awake;
            _bodies.SleepTime[b] = 0f;
            _events.Emit(0, new PhysicsEvent(EventKind.Wake, HandleOfBody(b), default, default, default, 0, 0, default, 0f));
        }
    }

    private void SleepIsland(int island)
    {
        _islands.SetAsleep(island, true);
        foreach (int b in _islands.Bodies(island))
        {
            _bodies.Flags[b] &= unchecked((byte)~(byte)BodyBits.Awake);
            _bodies.LinearVelocityOf(b).Clear();
            _bodies.AngularVelocityOf(b).Clear();
            _events.Emit(0, new PhysicsEvent(EventKind.Sleep, HandleOfBody(b), default, default, default, 0, 0, default, 0f));
        }
    }

    // ------------------------------------------------------------------ constraints

    /// <summary>
    /// Adds a constraint. Contacts and joints between dynamic bodies merge their islands; adding one wakes the islands of its dynamic bodies; it takes the lowest free color on its dynamic bodies.
    /// Queues a begin event for contacts and an enter event for sensors. Must be called serially and, from stages, in an order that depends only on the data. Returns the constraint index.
    /// </summary>
    public int AddConstraint(in ConstraintDesc desc)
    {
        if (desc.Kind < ConstraintKind.Contact || desc.Kind > ConstraintKind.Joint) { throw new ArgumentOutOfRangeException(nameof(desc), desc.Kind, "Unknown constraint kind."); }
        int a = Resolve(desc.BodyA), b = Resolve(desc.BodyB);
        if (a == b) { throw new ArgumentException("A constraint needs two different bodies.", nameof(desc)); }
        int sa = -1, sb = -1, ga = 0, gb = 0;
        if (!desc.ShapeA.IsNull) { sa = Resolve(desc.ShapeA); ga = desc.ShapeA.Generation; }
        if (!desc.ShapeB.IsNull) { sb = Resolve(desc.ShapeB); gb = desc.ShapeB.Generation; }
        if ((desc.Flags & ~(ConstraintBits.ReportPersist)) != 0) { throw new ArgumentOutOfRangeException(nameof(desc), desc.Flags, "Only ReportPersist may be set when adding a constraint."); }

        int c = _constraints.AllocateSlot();
        _constraints.ClearSlot(c);
        _constraints.Fill(c, desc, a, b, sa, ga, sb, gb);
        bool aDyn = _bodies.IsDynamic(a), bDyn = _bodies.IsDynamic(b);
        if (aDyn) { _constraints.LinkEdge(_bodies, c, 0); }
        if (bDyn) { _constraints.LinkEdge(_bodies, c, 1); }
        if (desc.Kind != ConstraintKind.Sensor)
        {
            if (aDyn) { WakeIsland(_bodies.Island[a]); }
            if (bDyn) { WakeIsland(_bodies.Island[b]); }
            if (aDyn && bDyn) { _islands.Merge(_bodies, _bodies.Island[a], _bodies.Island[b]); }
            _constraints.ColorWritable[c] = _coloring.Assign(a, aDyn, b, bDyn);
        }

        if (desc.Kind != ConstraintKind.Joint) { EmitConstraintEvent(desc.Kind == ConstraintKind.Sensor ? EventKind.SensorEnter : EventKind.ContactBegin, c, true, 0f); }
        return c;
    }

    /// <summary>
    /// Removes a constraint by index. Queues an end event for contacts and an exit event for sensors, counts the removal against the island (so a possibly disconnected island is split before it can sleep)
    /// and wakes the island(s) of its dynamic bodies. Serial, like <see cref="AddConstraint"/>.
    /// </summary>
    public void RemoveConstraint(int constraint)
    {
        if (!_constraints.IsLive(constraint)) { throw new StaleHandleException("Constraint " + constraint.ToString(CultureInfo.InvariantCulture) + " is not live."); }
        RemoveInternal(constraint, 0f, false);
    }

    /// <summary>Removes a joint by handle (no event is queued for an ordinary removal).</summary>
    public void DestroyJoint(JointHandle handle) => RemoveInternal(Resolve(handle), 0f, false);

    private void RemoveInternal(int c, float breakValue, bool broke)
    {
        var kind = _constraints.KindOf(c);
        int a = _constraints.BodyA[c], b = _constraints.BodyB[c];
        bool aDyn = _bodies.IsDynamic(a), bDyn = _bodies.IsDynamic(b);
        if (broke) { EmitConstraintEvent(EventKind.JointBreak, c, false, breakValue); }
        else if (kind == ConstraintKind.Contact) { EmitConstraintEvent(EventKind.ContactEnd, c, true, 0f); }
        else if (kind == ConstraintKind.Sensor) { EmitConstraintEvent(EventKind.SensorExit, c, true, 0f); }

        if (aDyn) { _constraints.UnlinkEdge(_bodies, c, 0); }
        if (bDyn) { _constraints.UnlinkEdge(_bodies, c, 1); }
        if (kind != ConstraintKind.Sensor)
        {
            if (aDyn) { WakeIsland(_bodies.Island[a]); }
            if (bDyn) { WakeIsland(_bodies.Island[b]); }
            if (aDyn && bDyn) { _islands.NoteRemoval(_bodies.Island[a]); }
            _coloring.Release(_constraints.Color[c], a, aDyn, b, bDyn);
        }

        _constraints.ClearSlot(c);
        _constraints.Handles.Free(c);
    }

    private void EmitConstraintEvent(EventKind kind, int c, bool withShapes, float value)
    {
        if (!_events.Wants(kind)) { return; }
        var cons = _constraints;
        int a = cons.BodyA[c], b = cons.BodyB[c];
        ShapeHandle sa = default, sb = default;
        if (withShapes)
        {
            if (cons.ShapeA[c] >= 0) { sa = new ShapeHandle(Id, cons.ShapeA[c], cons.ShapeGenerationA(c)); }
            if (cons.ShapeB[c] >= 0) { sb = new ShapeHandle(Id, cons.ShapeB[c], cons.ShapeGenerationB(c)); }
        }

        var joint = kind == EventKind.JointBreak ? HandleOfJoint(c) : default;
        _events.Emit(0, new PhysicsEvent(kind, HandleOfBody(a), HandleOfBody(b), sa, sb, withShapes ? cons.SubShapeA[c] : 0, withShapes ? cons.SubShapeB[c] : 0, joint, value));
    }

    // ------------------------------------------------------------------ origin

    /// <summary>
    /// Shifts the world origin by <paramref name="offset"/> (one double per position lane): every body position decreases by it and <see cref="Origin"/> increases by it, so host coordinates (origin + stored position) do not change.
    /// Offsets must be finite multiples of 2^-31 (every 31.32 value is) with magnitude at most 2^22; with positions on that grid the subtraction is exact. Call between steps, on every peer at the same step.
    /// Anything an engine keeps in world coordinates (broadphase bounds, cached world-space data) must be shifted by the engine as well.
    /// </summary>
    public void ShiftOrigin(ReadOnlySpan<double> offset)
    {
        if (offset.Length != Layout.PositionLanes) { throw new ArgumentException("offset needs " + Layout.PositionLanes.ToString(CultureInfo.InvariantCulture) + " components.", nameof(offset)); }
        for (int i = 0; i < offset.Length; i++)
        {
            double o = offset[i];
            if (!double.IsFinite(o) || Math.Abs(o) > OffsetBound) { throw new ArgumentOutOfRangeException(nameof(offset), o, "Origin offset component " + i.ToString(CultureInfo.InvariantCulture) + " = " + o.ToString("R", CultureInfo.InvariantCulture) + " is not finite or exceeds 2^22."); }
            double scaled = o * OffsetScale;
            if (Math.Floor(scaled) != scaled) { throw new ArgumentException("Origin offset component " + i.ToString(CultureInfo.InvariantCulture) + " = " + o.ToString("R", CultureInfo.InvariantCulture) + " is not a multiple of 2^-31.", nameof(offset)); }
        }

        int lanes = Layout.PositionLanes;
        var pos = _bodies.Position;
        for (int i = 0; i < pos.Length; i++) { pos[i] -= offset[i % lanes]; }
        for (int i = 0; i < lanes; i++) { _origin[i] += offset[i]; }
    }

    // ------------------------------------------------------------------ the step

    /// <summary>
    /// Advances the world by <paramref name="dt"/> seconds. The step has no clock: given the same state, options and <paramref name="dt"/> it produces the same bits on every platform and thread count.
    /// Events raised during the step and since the previous one are delivered sorted in <see cref="Events"/> when it returns.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="dt"/> is not finite and positive.</exception>
    public void Step<TStages>(TStages stages, float dt) where TStages : IWorldStages
    {
        if (!(dt > 0f) || !float.IsFinite(dt)) { throw new ArgumentOutOfRangeException(nameof(dt), dt, "dt must be finite and positive."); }
        _options.Validate();
        _events.ClearDelivered();
        BuildAwakeBodies();
        stages.UpdateContacts(this);
        BuildAwakeBodies();
        _coloring.EnsureBodies(_bodies.HighWater);
        _coloring.BuildActive(_bodies, _constraints);
        var ctx = new StepContext(dt, _options.Substeps);
        int active = _coloring.ActiveCount;
        int storeChunks = (active + ChunkSize - 1) / ChunkSize;
        _events.EnsureSlots(1 + storeChunks);
        var list = _coloring.ListArray;

        Executor.For(storeChunks, new PrepareTask<TStages>(this, stages, list, 0, active, ctx));
        PrepareBatches(stages, ctx);

        int bodyChunks = (_awakeCount + ChunkSize - 1) / ChunkSize;
        for (int s = 0; s < _options.Substeps; s++)
        {
            _flipSweep = (s & 1) == 1;
            Executor.For(bodyChunks, new BodyTask<TStages>(this, stages, _awake, _awakeCount, ctx, true));
            SolveAll(stages, SolvePhase.WarmStart, ctx);
            for (int i = 0; i < _options.VelocityIterations; i++) { SolveAll(stages, SolvePhase.Solve, ctx); }
            Executor.For(bodyChunks, new BodyTask<TStages>(this, stages, _awake, _awakeCount, ctx, false));
            for (int i = 0; i < _options.RelaxIterations; i++) { SolveAll(stages, SolvePhase.Relax, ctx); }
        }

        SolveAll(stages, SolvePhase.Restitution, ctx);
        Executor.For(bodyChunks, new FinalizeTask<TStages>(this, stages, _awake, _awakeCount, ctx));
        stages.AfterFinalize(this, ctx);
        Executor.For(storeChunks, new StoreTask<TStages>(this, stages, list, active, ctx));

        ProcessBreaks();
        UpdateSleep(stages, dt);
        EmitPersist();
        _events.Deliver();
        _stepCount++;
    }

    private void BuildAwakeBodies()
    {
        int n = _bodies.HighWater;
        if (_awake.Length < n) { _awake = new int[Math.Max(n, _awake.Length * 2)]; }
        int k = 0;
        var type = _bodies.Type;
        for (int b = 0; b < n; b++)
        {
            if (!_bodies.IsLive(b)) { continue; }
            if (type[b] == (byte)BodyType.Kinematic || (type[b] == (byte)BodyType.Dynamic && _bodies.IsAwake(b))) { _awake[k++] = b; }
        }

        _awakeCount = k;
    }

    private void PrepareBatches<TStages>(TStages stages, in StepContext ctx) where TStages : IWorldStages
    {
        var list = _coloring.ListArray;
        int os = _coloring.OverflowStart;
        int on = _coloring.Overflow.Length;
        int batches = (on + ConstraintColoring.Lanes - 1) / ConstraintColoring.Lanes;
        for (int color = 0; color < ConstraintColoring.ColorCount; color++) { batches += (_coloring.Color(color).Length + ConstraintColoring.Lanes - 1) / ConstraintColoring.Lanes; }
        stages.BeginSolve(this, batches, in ctx);
        int batch = 0;
        for (int lo = 0; lo < on; lo += ConstraintColoring.Lanes)
        {
            stages.PrepareBatch(this, list.AsSpan(os + lo, Math.Min(ConstraintColoring.Lanes, on - lo)), batch++, in ctx);
        }

        for (int color = 0; color < ConstraintColoring.ColorCount; color++)
        {
            int n = _coloring.Color(color).Length;
            if (n == 0) { continue; }
            int count = (n + ConstraintColoring.Lanes - 1) / ConstraintColoring.Lanes;
            int tasks = (count + ConstraintColoring.BatchesPerTask - 1) / ConstraintColoring.BatchesPerTask;
            Executor.For(tasks, new PrepareBatchTask<TStages>(this, stages, list, _coloring.ColorStart(color), n, batch, ctx));
            batch += count;
        }
    }

    private readonly struct PrepareBatchTask<TStages> : IParallelBody where TStages : IWorldStages
    {
        private readonly WorldCore _w; private readonly TStages _s; private readonly int[] _list; private readonly int _start, _count, _firstBatch; private readonly StepContext _ctx;
        public PrepareBatchTask(WorldCore w, TStages s, int[] list, int start, int count, int firstBatch, StepContext ctx) { _w = w; _s = s; _list = list; _start = start; _count = count; _firstBatch = firstBatch; _ctx = ctx; }
        public void Run(int task)
        {
            int first = task * ConstraintColoring.BatchesPerTask;
            for (int bt = first; bt < first + ConstraintColoring.BatchesPerTask; bt++)
            {
                int lo = bt * ConstraintColoring.Lanes;
                if (lo >= _count) { break; }
                _s.PrepareBatch(_w, _list.AsSpan(_start + lo, Math.Min(ConstraintColoring.Lanes, _count - lo)), _firstBatch + bt, in _ctx);
            }
        }
    }

    /// <summary>When set, the relax pass sweeps the colors in the opposite order (and the serial overflow last), so the sequential bias of one sweep is cancelled by the next.</summary>
    public bool SymmetricSweep { get; set; }
    private bool _flipSweep;

    private void SolveAll<TStages>(TStages stages, SolvePhase phase, in StepContext ctx) where TStages : IWorldStages
    {
        var list = _coloring.ListArray;
        int os = _coloring.OverflowStart;
        int on = _coloring.Overflow.Length;
        bool reverse = SymmetricSweep && phase != SolvePhase.WarmStart && ((phase == SolvePhase.Relax) != _flipSweep);
        if (!reverse) { SolveOverflow(stages, list, os, on, phase, in ctx); }
        for (int k = 0; k < ConstraintColoring.ColorCount; k++)
        {
            int color = reverse ? ConstraintColoring.ColorCount - 1 - k : k;
            int n = _coloring.Color(color).Length;
            if (n == 0) { continue; }
            int batches = (n + ConstraintColoring.Lanes - 1) / ConstraintColoring.Lanes;
            int tasks = (batches + ConstraintColoring.BatchesPerTask - 1) / ConstraintColoring.BatchesPerTask;
            Executor.For(tasks, new SolveTask<TStages>(this, stages, list, _coloring.ColorStart(color), n, phase, ctx));
        }

        if (reverse) { SolveOverflow(stages, list, os, on, phase, in ctx); }
    }

    private void SolveOverflow<TStages>(TStages stages, int[] list, int os, int on, SolvePhase phase, in StepContext ctx) where TStages : IWorldStages
    {
        for (int lo = 0; lo < on; lo += ConstraintColoring.Lanes)
        {
            stages.SolveBatch(this, list.AsSpan(os + lo, Math.Min(ConstraintColoring.Lanes, on - lo)), phase, in ctx);
        }
    }

    private readonly struct PrepareTask<TStages> : IParallelBody where TStages : IWorldStages
    {
        private readonly WorldCore _w; private readonly TStages _s; private readonly int[] _list; private readonly int _start, _count; private readonly StepContext _ctx;
        public PrepareTask(WorldCore w, TStages s, int[] list, int start, int count, StepContext ctx) { _w = w; _s = s; _list = list; _start = start; _count = count; _ctx = ctx; }
        public void Run(int task)
        {
            int lo = task * ChunkSize;
            _s.Prepare(_w, _list.AsSpan(_start + lo, Math.Min(ChunkSize, _count - lo)), in _ctx);
        }
    }

    private readonly struct BodyTask<TStages> : IParallelBody where TStages : IWorldStages
    {
        private readonly WorldCore _w; private readonly TStages _s; private readonly int[] _list; private readonly int _count; private readonly StepContext _ctx; private readonly bool _velocities;
        public BodyTask(WorldCore w, TStages s, int[] list, int count, StepContext ctx, bool velocities) { _w = w; _s = s; _list = list; _count = count; _ctx = ctx; _velocities = velocities; }
        public void Run(int task)
        {
            int lo = task * ChunkSize;
            var span = _list.AsSpan(lo, Math.Min(ChunkSize, _count - lo));
            if (_velocities) { _s.IntegrateVelocities(_w, span, in _ctx); } else { _s.IntegratePositions(_w, span, in _ctx); }
        }
    }

    private readonly struct FinalizeTask<TStages> : IParallelBody where TStages : IWorldStages
    {
        private readonly WorldCore _w; private readonly TStages _s; private readonly int[] _list; private readonly int _count; private readonly StepContext _ctx;
        public FinalizeTask(WorldCore w, TStages s, int[] list, int count, StepContext ctx) { _w = w; _s = s; _list = list; _count = count; _ctx = ctx; }
        public void Run(int task)
        {
            int lo = task * ChunkSize;
            _s.FinalizeBodies(_w, _list.AsSpan(lo, Math.Min(ChunkSize, _count - lo)), in _ctx);
        }
    }

    private readonly struct SolveTask<TStages> : IParallelBody where TStages : IWorldStages
    {
        private readonly WorldCore _w; private readonly TStages _s; private readonly int[] _list; private readonly int _start, _count; private readonly SolvePhase _phase; private readonly StepContext _ctx;
        public SolveTask(WorldCore w, TStages s, int[] list, int start, int count, SolvePhase phase, StepContext ctx) { _w = w; _s = s; _list = list; _start = start; _count = count; _phase = phase; _ctx = ctx; }
        public void Run(int task)
        {
            int firstBatch = task * ConstraintColoring.BatchesPerTask;
            for (int bt = firstBatch; bt < firstBatch + ConstraintColoring.BatchesPerTask; bt++)
            {
                int lo = bt * ConstraintColoring.Lanes;
                if (lo >= _count) { break; }
                _s.SolveBatch(_w, _list.AsSpan(_start + lo, Math.Min(ConstraintColoring.Lanes, _count - lo)), _phase, in _ctx);
            }
        }
    }

    private readonly struct StoreTask<TStages> : IParallelBody where TStages : IWorldStages
    {
        private readonly WorldCore _w; private readonly TStages _s; private readonly int[] _list; private readonly int _count; private readonly StepContext _ctx;
        public StoreTask(WorldCore w, TStages s, int[] list, int count, StepContext ctx) { _w = w; _s = s; _list = list; _count = count; _ctx = ctx; }
        public void Run(int task)
        {
            int lo = task * ChunkSize;
            _s.StoreResults(_w, _list.AsSpan(lo, Math.Min(ChunkSize, _count - lo)), 1 + task, in _ctx);
        }
    }

    private void ProcessBreaks()
    {
        _scratch.Clear();
        var flags = _constraints.Flags;
        for (int c = 0; c < _constraints.HighWater; c++)
        {
            if ((flags[c] & (byte)ConstraintBits.BreakRequested) == 0) { continue; }
            if (_constraints.IsLive(c) && _constraints.KindOf(c) == ConstraintKind.Joint) { _scratch.Add(c); }
            else { flags[c] &= unchecked((byte)~(byte)ConstraintBits.BreakRequested); }
        }

        var breaking = _scratch.ToArray();
        foreach (int c in breaking) { RemoveInternal(c, _constraints.BreakValue[c], true); }
    }

    private void UpdateSleep<TStages>(TStages stages, float dt) where TStages : IWorldStages
    {
        var sleep = _bodies.SleepTime;
        var flags = _bodies.Flags;
        bool enabled = _options.SleepEnabled;
        for (int i = 0; i < _awakeCount; i++)
        {
            int b = _awake[i];
            if (!_bodies.IsLive(b) || _bodies.TypeOf(b) != BodyType.Dynamic) { continue; }
            if (enabled && (flags[b] & (byte)BodyBits.NoSleep) == 0 && stages.BodyMotion(this, b) <= _options.SleepThreshold) { sleep[b] += dt; }
            else { sleep[b] = 0f; }
        }

        if (!enabled) { return; }
        float limit = _options.TimeToSleep;

        // Split the one island that is about to sleep and has lost constraints, the one that has rested longest (lowest id on ties).
        int candidate = -1;
        float best = 0f;
        for (int i = 0; i < _islands.HighWater; i++)
        {
            if (!_islands.IsLive(i) || _islands.IsAsleep(i) || _islands.RemovedConstraints(i) == 0) { continue; }
            float m = MinSleepTime(i);
            if (m >= limit && (candidate < 0 || m > best)) { candidate = i; best = m; }
        }

        if (candidate >= 0) { _islands.Split(_bodies, _constraints, candidate); }

        for (int i = 0; i < _islands.HighWater; i++)
        {
            if (!_islands.IsLive(i) || _islands.IsAsleep(i) || _islands.RemovedConstraints(i) != 0) { continue; }
            if (MinSleepTime(i) >= limit) { SleepIsland(i); }
        }
    }

    private float MinSleepTime(int island)
    {
        float m = float.MaxValue;
        var sleep = _bodies.SleepTime;
        foreach (int b in _islands.Bodies(island)) { if (sleep[b] < m) { m = sleep[b]; } }
        return m;
    }

    private void EmitPersist()
    {
        if (!_events.Wants(EventKind.ContactPersist)) { return; }
        var cons = _constraints;
        for (int c = 0; c < cons.HighWater; c++)
        {
            if (!cons.IsLive(c) || cons.Kind[c] != (byte)ConstraintKind.Contact || (cons.Flags[c] & (byte)ConstraintBits.ReportPersist) == 0) { continue; }
            int a = cons.BodyA[c], b = cons.BodyB[c];
            if ((_bodies.IsDynamic(a) && _bodies.IsAwake(a)) || (_bodies.IsDynamic(b) && _bodies.IsAwake(b))) { EmitConstraintEvent(EventKind.ContactPersist, c, true, 0f); }
        }
    }

    // ------------------------------------------------------------------ snapshot, restore, hash

    /// <summary>The 64-bit world hash: the hash of the canonical state in snapshot order, without the format header and without events (see <see cref="StateWriter"/> for the function). Equal states give equal hashes on every platform.</summary>
    public ulong ComputeHash()
    {
        var w = StateWriter.ForHash();
        WriteState(w, false);
        return w.Hash;
    }

    /// <summary>Serializes the whole world: options, step counter, origin, bodies, shapes, constraints (with warm-start payloads and colors), islands and sleep state, delivered and pending events, and registered sections. Layout in docs/architecture.md; format version 1.</summary>
    public byte[] Snapshot()
    {
        var w = StateWriter.ToBuffer();
        w.WriteUInt32(Magic);
        w.WriteUInt32(FormatVersion);
        WriteState(w, true);
        return w.ToArray();
    }

    private void WriteState(StateWriter w, bool includeEvents)
    {
        _options.Write(w);
        w.WriteInt64(_stepCount);
        w.WriteDoubles(_origin);
        _bodies.Write(w);
        _shapes.Write(w);
        _constraints.Write(w);
        _islands.Write(w);
        if (includeEvents) { _events.Write(w); }
        w.WriteInt32(_sections.Count);
        foreach (var s in _sections)
        {
            w.WriteInt32(s.SectionId);
            if (w.IsHashing)
            {
                s.Write(w);
            }
            else
            {
                var inner = StateWriter.ToBuffer();
                s.Write(inner);
                w.WriteInt32(inner.Length);
                w.WriteBytes(inner.ToArray());
            }
        }
    }

    /// <summary>
    /// Replaces the world's state with a snapshot taken by <see cref="Snapshot"/> from a world with the same layout and sections. Everything is validated first; on any problem a
    /// <see cref="SnapshotFormatException"/> is thrown and the world is unchanged. Handles issued by the world that took the snapshot are valid only if this world has the same <see cref="Id"/>.
    /// Re-read <see cref="Bodies"/>, <see cref="Constraints"/>, <see cref="Islands"/> afterwards: they are replaced objects.
    /// </summary>
    public void Restore(byte[] snapshot)
    {
        ArgumentNullException.ThrowIfNull(snapshot);
        byte[]? backup = _sections.Count > 0 ? Snapshot() : null;
        try
        {
            RestoreCore(snapshot);
        }
        catch when (backup != null)
        {
            // A section may have been replaced before a later one failed; put the previous state back so a failed restore leaves the world as it was.
            RestoreCore(backup);
            throw;
        }
    }

    private void RestoreCore(byte[] snapshot)
    {
        var r = new StateReader(snapshot);
        if (r.ReadUInt32() != Magic) { throw r.Error("not a physics snapshot (bad magic)"); }
        uint version = r.ReadUInt32();
        if (version != FormatVersion) { throw r.Error("unsupported snapshot version " + version.ToString(CultureInfo.InvariantCulture)); }
        var opts = WorldOptions.Read(r, _options);
        if (opts.Layout != Layout) { throw r.Error("snapshot layout does not match this world's layout"); }
        long steps = r.ReadInt64();
        if (steps < 0) { throw r.Error("negative step count"); }
        var origin = new double[Layout.PositionLanes];
        r.ReadDoubles(origin, true, "origin");

        var bodies = BodyStore.Read(r, Layout);
        var shapes = ShapeStore.Read(r);
        var cons = ConstraintStore.Read(r, Layout);
        ValidateReferences(r, bodies, shapes, cons);
        for (int c = 0; c < cons.HighWater; c++)
        {
            if (!cons.IsLive(c)) { continue; }
            if (bodies.IsDynamic(cons.BodyA[c])) { cons.LinkEdge(bodies, c, 0); }
            if (bodies.IsDynamic(cons.BodyB[c])) { cons.LinkEdge(bodies, c, 1); }
        }

        var coloring = new ConstraintColoring();
        coloring.EnsureBodies(bodies.HighWater);
        for (int c = 0; c < cons.HighWater; c++)
        {
            if (!cons.IsLive(c)) { continue; }
            bool colored = cons.Kind[c] != (byte)ConstraintKind.Sensor && (bodies.IsDynamic(cons.BodyA[c]) || bodies.IsDynamic(cons.BodyB[c]));
            byte col = cons.Color[c];
            if (colored ? (col >= ConstraintColoring.ColorCount && col != ConstraintStore.Overflow) : col != ConstraintStore.Uncolored) { throw r.Error("constraint " + c.ToString(CultureInfo.InvariantCulture) + " has an invalid color"); }
        }

        if (!coloring.RebuildMasks(bodies, cons)) { throw r.Error("two constraints on one body share a color"); }
        var islands = IslandSet.Read(r, bodies, cons);
        var events = new EventCollector(_options);
        events.Read(r);

        int sections = r.ReadInt32();
        if (sections != _sections.Count) { throw r.Error("snapshot has " + sections.ToString(CultureInfo.InvariantCulture) + " sections, this world registers " + _sections.Count.ToString(CultureInfo.InvariantCulture)); }
        var blobs = new List<byte[]>();
        foreach (var s in _sections)
        {
            if (r.ReadInt32() != s.SectionId) { throw r.Error("section id mismatch"); }
            int len = r.ReadCount(1, "section bytes");
            var blob = new byte[len];
            r.ReadBytes(blob);
            blobs.Add(blob);
        }

        if (r.Remaining != 0) { throw r.Error("trailing bytes after the snapshot"); }

        // Everything the core owns validated: now the sections (each all-or-nothing), then swap the core state in.
        for (int i = 0; i < _sections.Count; i++)
        {
            var sub = new StateReader(blobs[i]);
            _sections[i].Read(sub);
            if (sub.Remaining != 0) { throw new SnapshotFormatException("section " + _sections[i].SectionId.ToString(CultureInfo.InvariantCulture) + " left unread bytes", r.Position); }
        }

        _options.CopySimulationValuesFrom(opts);
        _stepCount = steps;
        _origin = origin;
        _bodies = bodies;
        _shapes = shapes;
        _constraints = cons;
        _islands = islands;
        _coloring.CopyFrom(coloring);
        _events.CopyFrom(events);
        _awakeCount = 0;
    }

    private static void ValidateReferences(StateReader r, BodyStore bodies, ShapeStore shapes, ConstraintStore cons)
    {
        for (int s = 0; s < shapes.HighWater; s++)
        {
            int owner = shapes.Owner[s];
            if (shapes.Handles.IsLive(s) ? !bodies.IsLive(owner) : owner != -1) { throw r.Error("shape " + s.ToString(CultureInfo.InvariantCulture) + " has an invalid owner"); }
        }

        for (int c = 0; c < cons.HighWater; c++)
        {
            if (!cons.IsLive(c)) { continue; }
            if (!bodies.IsLive(cons.BodyA[c]) || !bodies.IsLive(cons.BodyB[c])) { throw r.Error("constraint " + c.ToString(CultureInfo.InvariantCulture) + " refers to a dead body"); }
            CheckShape(r, shapes, cons.ShapeA[c], cons.ShapeGenerationA(c), c);
            CheckShape(r, shapes, cons.ShapeB[c], cons.ShapeGenerationB(c), c);
        }
    }

    private static void CheckShape(StateReader r, ShapeStore shapes, int shape, int generation, int c)
    {
        bool ok = shape == -1 ? generation == 0 : shapes.Handles.IsAlive(shape, generation);
        if (!ok) { throw r.Error("constraint " + c.ToString(CultureInfo.InvariantCulture) + " refers to a dead shape"); }
    }
}
