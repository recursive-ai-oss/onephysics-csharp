using System;
using System.Buffers;
using System.Buffers.Binary;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using OneScope.Sdk;
using RecursiveAI.OnePhysics.Debug;
using RecursiveAI.OnePhysics.Replay;

namespace RecursiveAI.OnePhysics.Scope;

/// <summary>A OnePhysics world as the viewer sees it. Step, snapshot, restore and hash are the library's own.</summary>
internal sealed partial class ScopeWorld : IScopeWorld
{
    // Most contact-begin events one step puts on the timeline; the rest are counted by the "contacts" series.
    private const int MaxContactEventsPerStep = 32;
    private const int SnapshotHeaderBytes = 12;

    private readonly WorldHost _host;
    private readonly WorldSetup _setup;
    private readonly List<StepOp> _ops = [];
    private readonly DebugGeometry _geometry = new();
    private float _dt;

    public ScopeWorld(WorldHost host, WorldSetup setup)
    {
        _host = host;
        _setup = setup;
        _dt = setup.DefaultDt;
        Step = host.StepCount;
    }

    public long Step { get; private set; }

    public ulong Hash() => _host.Hash();

    public void Advance(ReadOnlySpan<byte> input, TimelineSink sink)
    {
        ArgumentNullException.ThrowIfNull(sink);
        Run(input, sink, null);
    }

    public void AdvanceAxis(string axis, int position, ReadOnlySpan<byte> input) =>
        throw new ArgumentException($"The axis '{axis}' is not declared; this source has only the time axis.", nameof(axis));

    /// <summary>Runs one step: state changes and pokes in order, then the step, then changes made after it. When <paramref name="emit"/> is given, writes them as replay records.</summary>
    internal void Run(ReadOnlySpan<byte> input, TimelineSink sink, ReplayEmitter? emit)
    {
        StepOps.Parse(input, _ops);
        foreach (StepOp op in _ops)
        {
            switch (op.Kind)
            {
                case OpKind.Dt:
                    _dt = op.Dt;
                    break;
                case OpKind.State:
                    ApplyState(op, input, emit);
                    break;
                case OpKind.PostState:
                    break;
                default:
                    ApplyPoke(op, emit);
                    break;
            }
        }

        emit?.BeginStep(_dt);
        _host.Step(_dt);
        Step++;
        if (sink.IsRecording)
        {
            Record(sink);
        }

        foreach (StepOp op in _ops)
        {
            if (op.Kind == OpKind.PostState)
            {
                ApplyState(op, input, emit);
            }
        }

        emit?.EndStep(_host.StepCount, _host.Hash());
    }

    private void ApplyState(in StepOp op, ReadOnlySpan<byte> input, ReplayEmitter? emit)
    {
        ReadOnlySpan<byte> state = input.Slice(op.Offset, op.Length);
        RestoreState(state);
        emit?.Mutation(op.Name!, _host.StepCount, state);
    }

    private void ApplyPoke(in StepOp op, ReplayEmitter? emit)
    {
        if (!EntityIds.TryBody(op.Target, _host.Id, out BodyHandle body) || !_host.IsBodyAlive(body))
        {
            throw new ScopeFormatException("step input", $"expected a body that exists for the poke on entity {op.Target}.");
        }

        _host.ApplyPoke(body, op);
        emit?.Mutation(StepOps.PokeName(op), _host.StepCount, _host.Snapshot());
    }

    private void RestoreState(ReadOnlySpan<byte> snapshot)
    {
        try
        {
            _host.Restore(snapshot);
        }
        catch (SnapshotFormatException e)
        {
            throw new ScopeFormatException("snapshot", "expected a valid OnePhysics snapshot: " + e.Message);
        }
    }

    /// <summary>The records a replay of this world starts with, as the library's own recorder writes them.</summary>
    internal (byte[] Snapshot, byte[]? Options, long InitialStep) InitialRecords()
    {
        using var ms = new MemoryStream();
        using (_host.StartRecording(ms))
        {
        }

        ms.Position = 0;
        var reader = new ReplayReader(ms, "initial records");
        reader.ReadNext();
        byte[] snapshot = reader.Payload.ToArray();
        byte[]? options = null;
        if (reader.FormatVersion == 2 && reader.ReadNext() && reader.Kind == ReplayRecordKind.LiveOptions)
        {
            options = reader.Payload.ToArray();
        }

        return (snapshot, options, reader.InitialStep);
    }

    public void WriteSnapshot(IBufferWriter<byte> writer)
    {
        ArgumentNullException.ThrowIfNull(writer);
        byte[] snapshot = _host.Snapshot();
        Span<byte> head = writer.GetSpan(SnapshotHeaderBytes);
        BinaryPrimitives.WriteInt64LittleEndian(head, Step);
        BinaryPrimitives.WriteSingleLittleEndian(head[8..], _dt);
        writer.Advance(SnapshotHeaderBytes);
        int offset = 0;
        while (offset < snapshot.Length)
        {
            Span<byte> span = writer.GetSpan(Math.Min(snapshot.Length - offset, 1 << 16));
            int n = Math.Min(span.Length, snapshot.Length - offset);
            snapshot.AsSpan(offset, n).CopyTo(span);
            writer.Advance(n);
            offset += n;
        }
    }

    public void Restore(ReadOnlySpan<byte> snapshot)
    {
        if (snapshot.Length <= SnapshotHeaderBytes)
        {
            throw new ScopeFormatException("snapshot", $"expected more than {SnapshotHeaderBytes} bytes, found {snapshot.Length}.");
        }

        long step = BinaryPrimitives.ReadInt64LittleEndian(snapshot);
        float dt = BinaryPrimitives.ReadSingleLittleEndian(snapshot[8..]);
        if (step < 0 || !float.IsFinite(dt) || dt <= 0f)
        {
            throw new ScopeFormatException("snapshot", "expected a nonnegative step and a positive finite time step in the first 12 bytes.");
        }

        RestoreState(snapshot[SnapshotHeaderBytes..]);
        Step = step;
        _dt = dt;
    }

    public void EncodePoke(in PokeCall call, IBufferWriter<byte> into)
    {
        ArgumentNullException.ThrowIfNull(into);
        ScopeValue[] a = call.Args ?? [];
        OpKind kind = call.Poke switch
        {
            ScopeInfo.PokeImpulse => OpKind.Impulse,
            ScopeInfo.PokeSetVelocity => OpKind.SetVelocity,
            ScopeInfo.PokeSetPosition => OpKind.SetPosition,
            ScopeInfo.PokeWake => OpKind.Wake,
            _ => throw new ArgumentException($"The poke '{call.Poke}' is not one of impulse, set_velocity, set_position or wake.", nameof(call)),
        };
        int expected = kind == OpKind.Wake ? 1 : 2;
        if (a.Length != expected || a[0].Kind != ValueKind.Entity || (expected == 2 && a[1].Kind != ValueKind.Vector))
        {
            throw new ArgumentException($"The poke '{call.Poke}' takes a body entity{(expected == 2 ? " and a vector" : string.Empty)}.", nameof(call));
        }

        EntityId target = a[0].GetEntity();
        if (!EntityIds.TryBody(target.Value, _host.Id, out BodyHandle body) || _host.TypeOf(body) is not BodyType type)
        {
            throw new ArgumentException($"Entity {target} is not a body that exists.", nameof(call));
        }

        if ((kind is OpKind.Impulse or OpKind.Wake && type != BodyType.Dynamic) || (kind == OpKind.SetVelocity && type == BodyType.Static))
        {
            throw new ArgumentException($"The poke '{call.Poke}' cannot move a {type.ToString().ToLowerInvariant()} body.", nameof(call));
        }

        System.Numerics.Vector3 v = expected == 2 ? a[1].GetVector() : default;
        if (!float.IsFinite(v.X) || !float.IsFinite(v.Y) || !float.IsFinite(v.Z))
        {
            throw new ArgumentException("The vector must be finite.", nameof(call));
        }

        // The world accepts positions within +-2^21 (its exact-grid range); refuse others here, so a recorded poke never throws in a step.
        const float Bound = 2097152f;
        if (kind == OpKind.SetPosition && (!Within(v.X, Bound) || !Within(v.Y, Bound) || !Within(v.Z, Bound)))
        {
            throw new ArgumentException($"A position must be within {Bound.ToString(CultureInfo.InvariantCulture)} of the origin on every axis.", nameof(call));
        }

        StepOps.WritePoke(into, new StepOp(kind, target.Value, v.X, v.Y, v.Z));
    }

    private static bool Within(float value, float bound) => value >= -bound && value <= bound;

    public Difference? FirstDifference(IScopeWorld other)
    {
        if (other is not ScopeWorld o)
        {
            throw new ArgumentException("The other world is from a different module.", nameof(other));
        }

        if (_host.Dimension != o._host.Dimension)
        {
            return new Difference(EntityIds.World, "dimension", _host.Dimension.ToString(CultureInfo.InvariantCulture), o._host.Dimension.ToString(CultureInfo.InvariantCulture));
        }

        byte[] a = _host.Snapshot();
        byte[] b = o._host.Snapshot();
        if (a.AsSpan().SequenceEqual(b))
        {
            return null;
        }

        // Bodies and joints first, since a difference there is what a user can look at; then the first of everything else (allocator
        // counts, broadphase nodes, contacts), which are consequences. World-level fields are named "section entity field".
        SnapshotDiffReport report = SnapshotDesync.Compare(a, b);
        SnapshotDifference? d = null;
        foreach (SnapshotDifference candidate in report.Differences)
        {
            if (candidate.Index >= 0 && candidate.Entity is "body" or "joint")
            {
                d = candidate;
                break;
            }
        }

        d ??= report.First;
        if (d is null)
        {
            return null;
        }

        bool entityField = d.Index >= 0 && d.Entity is "body" or "joint";
        EntityId entity = !entityField
            ? EntityIds.World
            : d.Entity == "body"
                ? new EntityId((1UL << 56) | (((ulong)(uint)d.Generation & 0xFFFFFF) << 32) | (uint)d.Index)
                : new EntityId((2UL << 56) | (((ulong)(uint)d.Generation & 0xFFFFFF) << 32) | (uint)d.Index);
        string field = entityField ? d.Field : d.Section + " " + d.Entity + " " + d.Field;
        // The planar engine stores a body's angular velocity in the first lane of a vector; it is a single number.
        if (_host.Dimension == 2 && field == "angular velocity x")
        {
            field = "angular velocity";
        }

        if (d.Occurrence > 0)
        {
            field += "[" + d.Occurrence.ToString(CultureInfo.InvariantCulture) + "]";
        }

        return new Difference(entity, field, d.A?.Value ?? "(absent)", d.B?.Value ?? "(absent)");
    }

    public void CollectEntities(List<EntityInfo> into)
    {
        ArgumentNullException.ThrowIfNull(into);
        into.Clear();
        into.Add(new EntityInfo(EntityIds.World, EntityId.None, "world", "World"));
        _host.CollectDebug(DebugContent.Bodies | DebugContent.Joints, _geometry);
        foreach (DebugBody b in _geometry.Bodies)
        {
            into.Add(new EntityInfo(EntityIds.Of(b.Body), EntityIds.World, "body", $"Body {b.Body.Index.ToString(CultureInfo.InvariantCulture)} ({b.State})"));
        }

        foreach (DebugJoint j in _geometry.Joints)
        {
            into.Add(new EntityInfo(EntityIds.Of(j.Joint), EntityIds.World, "joint", $"Joint {j.Joint.Index.ToString(CultureInfo.InvariantCulture)}"));
        }
    }

    public bool Inspect(EntityId entity, PropertyList into)
    {
        ArgumentNullException.ThrowIfNull(into);
        into.Clear();
        if (entity == EntityIds.World)
        {
            InspectWorld(into);
            return true;
        }

        if (EntityIds.TryBody(entity.Value, _host.Id, out BodyHandle body))
        {
            return _host.InspectBody(body, into);
        }

        return EntityIds.TryJoint(entity.Value, _host.Id, out JointHandle joint) && _host.InspectJoint(joint, into);
    }

    private void InspectWorld(PropertyList into)
    {
        StepCounters c = _host.Counters;
        into.Add("World", "Step", ScopeValue.OfInteger(Step));
        into.Add("World", "Completed steps", ScopeValue.OfInteger(_host.StepCount));
        into.Add("World", "Time step", ScopeValue.OfNumber(_dt));
        into.Add("World", "Hash", ScopeValue.OfText(_host.Hash().ToString("x16", CultureInfo.InvariantCulture)));
        into.Add("Counters", "Awake bodies", ScopeValue.OfInteger(c.AwakeBodies));
        into.Add("Counters", "Awake islands", ScopeValue.OfInteger(c.AwakeIslands));
        into.Add("Counters", "Asleep islands", ScopeValue.OfInteger(c.AsleepIslands));
        into.Add("Counters", "Broadphase pairs", ScopeValue.OfInteger(c.BroadphasePairs));
        into.Add("Counters", "Manifolds", ScopeValue.OfInteger(c.Manifolds));
        into.Add("Counters", "Contact points", ScopeValue.OfInteger(c.ContactPoints));
        into.Add("Counters", "Joints", ScopeValue.OfInteger(c.Joints));
        into.Add("Counters", "Sleep transitions", ScopeValue.OfInteger(c.SleepTransitions));
        into.Add("Counters", "Wake transitions", ScopeValue.OfInteger(c.WakeTransitions));
    }

    private void Record(TimelineSink sink)
    {
        StepCounters c = _host.Counters;
        sink.Sample(ScopeInfo.TrackAwake, c.AwakeBodies);
        sink.Sample(ScopeInfo.TrackContacts, c.ContactPoints);
        int contacts = 0;
        foreach (PhysicsEvent e in _host.Events)
        {
            switch (e.Kind)
            {
                case EventKind.ContactBegin when contacts < MaxContactEventsPerStep:
                    contacts++;
                    sink.Event(ScopeInfo.TrackContact, EntityIds.Of(e.BodyA), $"body {e.BodyA.Index.ToString(CultureInfo.InvariantCulture)} touches body {e.BodyB.Index.ToString(CultureInfo.InvariantCulture)}");
                    break;
                case EventKind.Sleep:
                    sink.Event(ScopeInfo.TrackSleep, EntityIds.Of(e.BodyA), "falls asleep");
                    break;
                case EventKind.Wake:
                    sink.Event(ScopeInfo.TrackSleep, EntityIds.Of(e.BodyA), "wakes");
                    break;
                case EventKind.JointBreak:
                    sink.Event(ScopeInfo.TrackJointBreak, EntityIds.Of(e.Joint), "joint breaks");
                    break;
            }
        }
    }
}
