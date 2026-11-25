using System;
using System.Buffers.Binary;
using System.Collections.Generic;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>Byte sizes of one body's streamed state in a dimension (docs/spec/interest.md).</summary>
internal readonly struct InterestLayout
{
    public readonly int Dimension, PositionLanes, RotationLanes, LinearLanes, AngularLanes, PayloadFloats, StateBytes;

    public InterestLayout(int dimension)
    {
        Dimension = dimension;
        PositionLanes = dimension;
        RotationLanes = dimension == 3 ? 4 : 2;
        LinearLanes = dimension;
        AngularLanes = dimension == 3 ? 3 : 1;
        PayloadFloats = dimension == 3 ? 104 : 40;
        // position doubles, rotation, linear and angular floats, one byte for sleeping.
        StateBytes = 8 * PositionLanes + 4 * (RotationLanes + LinearLanes + AngularLanes) + 1;
    }
}

/// <summary>Who is inside, who is a proxy, and what a packet for the entering bodies carries; also the shared hash and the per-body state record.</summary>
internal sealed class InterestRoles : IExtractFilter
{
    public const byte Absent = 0, Inside = 1, Proxy = 2;

    public readonly byte[] Role;
    private readonly WorldCore _core;
    /// <summary>Joints the client holds: both bodies present, one of them dynamic there, and a kind a packet can carry.</summary>
    public bool[] Desired = Array.Empty<bool>();

    /// <summary>Bodies the roles left out because a packet cannot carry them, with their role had they been carried.</summary>
    public readonly List<InterestSkip> Skipped = new();

    private InterestRoles(WorldCore core, byte[] role) { _core = core; Role = role; }

    /// <summary>The body is a dynamic inside body: the only kind of body the client's solver moves.</summary>
    public bool ClientDynamic(int b) => Role[b] == Inside && _core.Bodies.TypeOf(b) == BodyType.Dynamic;

    public static InterestRoles Compute(ITransferEngine e, InterestRegion region, bool proxies, bool withJoints)
    {
        var core = e.Core;
        var bodies = core.Bodies;
        int high = bodies.HighWater;
        int lanes = core.Layout.PositionLanes;
        var origin = core.Origin;
        var inside = new bool[high];
        var abs = new double[3];
        for (int b = 0; b < high; b++)
        {
            if (!bodies.IsLive(b) || bodies.TypeOf(b) == BodyType.Static) { continue; }
            var p = bodies.PositionOf(b);
            for (int k = 0; k < lanes; k++) { abs[k] = origin[k] + p[k]; }
            if (region.Contains(abs)) { inside[b] = true; }
        }

        foreach (var h in region.Bodies)
        {
            if (h.World == core.Id && (uint)h.Index < (uint)high && bodies.Handles.IsAlive(h.Index, h.Generation)) { inside[h.Index] = true; }
        }

        e.ExpandSet(inside);
        var role = new byte[high];
        for (int b = 0; b < high; b++) { if (inside[b] && bodies.IsLive(b)) { role[b] = Inside; } }
        var result = new InterestRoles(core, role);
        if (!proxies && !withJoints) { result.SkipUncarriable(e); return result; }

        if (proxies)
        {
            var near = new bool[high];
            var cs = core.Constraints;
            for (int c = 0; c < cs.HighWater; c++)
            {
                if (!cs.IsLive(c) || cs.KindOf(c) != ConstraintKind.Joint) { continue; }
                int a = cs.BodyA[c], b = cs.BodyB[c];
                if (result.ClientDynamic(a) && role[b] == Absent) { near[b] = true; }
                if (result.ClientDynamic(b) && role[a] == Absent) { near[a] = true; }
            }

            var pairs = new List<TransferPair>();
            e.CollectPairs(pairs);
            foreach (var pair in pairs)
            {
                int a = core.Shapes.Owner[pair.ShapeA], b = core.Shapes.Owner[pair.ShapeB];
                if (a == b) { continue; }
                bool touching = false;
                for (int c = pair.FirstConstraint; c >= 0; c = e.NextInChain(c)) { if (cs.KindOf(c) == ConstraintKind.Contact) { touching = true; break; } }
                if (!touching) { continue; }
                if (result.ClientDynamic(a) && role[b] == Absent) { near[b] = true; }
                if (result.ClientDynamic(b) && role[a] == Absent) { near[a] = true; }
            }

            e.ExpandSet(near);
            for (int b = 0; b < high; b++) { if (near[b] && role[b] == Absent && bodies.IsLive(b)) { role[b] = Proxy; } }
        }

        result.SkipUncarriable(e);
        if (withJoints) { result.FindJoints(e); }
        return result;
    }

    /// <summary>
    /// Takes out of the roles every body a packet cannot carry (an articulation, a rope attachment, a character holding a body that is not in the stream...). They are never sent and never part of the shared hash; the host
    /// sees them in <see cref="Skipped"/>. Done once the proxies are known, because a hold may be on a proxy.
    /// </summary>
    private void SkipUncarriable(ITransferEngine e)
    {
        var present = new bool[Role.Length];
        bool any = false;
        for (int b = 0; b < present.Length; b++) { if (Role[b] != Absent) { present[b] = true; any = true; } }
        if (!any) { return; }
        var skipped = new List<(int Body, string Reason)>();
        e.FindUncarriable(present, skipped);
        skipped.Sort((x, y) => x.Body.CompareTo(y.Body));
        foreach (var (body, reason) in skipped)
        {
            Skipped.Add(new InterestSkip(_core.HandleOfBody(body), Role[body] == Inside, reason));
            Role[body] = Absent;
        }
    }

    private void FindJoints(ITransferEngine e)
    {
        var cs = _core.Constraints;
        Desired = new bool[cs.HighWater];
        for (int c = 0; c < cs.HighWater; c++)
        {
            if (!cs.IsLive(c) || cs.KindOf(c) != ConstraintKind.Joint) { continue; }
            int a = cs.BodyA[c], b = cs.BodyB[c];
            if (Role[a] == Absent || Role[b] == Absent || !(ClientDynamic(a) || ClientDynamic(b))) { continue; }
            try { e.WriteJoint(c); }
            catch (BodySetTransferException) { continue; }   // a pulley or mouse joint: not carried
            Desired[c] = true;
        }
    }

    // ---- IExtractFilter
    public bool IsProxy(int body) => Role[body] == Proxy;
    public bool KeepJoint(int constraint) => (uint)constraint < (uint)Desired.Length && Desired[constraint];
    public bool KeepContact(int bodyA, int bodyB) => ClientDynamic(bodyA) || ClientDynamic(bodyB);
    public bool[] RagdollSet(bool[] inSet)
    {
        var r = new bool[inSet.Length];
        for (int b = 0; b < r.Length; b++) { r[b] = inSet[b] && Role[b] == Inside; }
        return r;
    }

    // ---- state record

    /// <summary>The streamed state of a body as it is in this world now: absolute position, rotation, velocities, sleeping.</summary>
    public static byte[] EncodeState(WorldCore core, in InterestLayout l, int b)
    {
        var buf = new byte[l.StateBytes];
        var bodies = core.Bodies;
        var origin = core.Origin;
        var p = bodies.PositionOf(b);
        int o = 0;
        for (int k = 0; k < l.PositionLanes; k++) { BinaryPrimitives.WriteInt64LittleEndian(buf.AsSpan(o), BitConverter.DoubleToInt64Bits(origin[k] + p[k])); o += 8; }
        o = PutFloats(buf, o, bodies.RotationOf(b));
        o = PutFloats(buf, o, bodies.LinearVelocityOf(b));
        o = PutFloats(buf, o, bodies.AngularVelocityOf(b));
        buf[o] = (byte)(bodies.TypeOf(b) == BodyType.Dynamic && !bodies.IsAwake(b) ? 1 : 0);
        return buf;
    }

    /// <summary>The state record of a body as a packet holds it.</summary>
    public static byte[] EncodeFromPacket(in InterestLayout l, PacketBody pb)
    {
        var buf = new byte[l.StateBytes];
        int o = 0;
        for (int k = 0; k < l.PositionLanes; k++) { BinaryPrimitives.WriteInt64LittleEndian(buf.AsSpan(o), BitConverter.DoubleToInt64Bits(pb.Position[k])); o += 8; }
        o = PutFloats(buf, o, pb.Rotation);
        o = PutFloats(buf, o, pb.Linear);
        o = PutFloats(buf, o, pb.Angular);
        buf[o] = (byte)(pb.Asleep ? 1 : 0);
        return buf;
    }

    private static int PutFloats(byte[] buf, int o, ReadOnlySpan<float> v)
    {
        for (int k = 0; k < v.Length; k++) { BinaryPrimitives.WriteUInt32LittleEndian(buf.AsSpan(o), BitConverter.SingleToUInt32Bits(v[k])); o += 4; }
        return o;
    }

    /// <summary>Writes a state record into the body (pose and velocities; the sleeping flag is handled by the caller) and moves its broadphase proxies.</summary>
    public static void DecodeState(ITransferEngine e, in InterestLayout l, int b, ReadOnlySpan<byte> s)
    {
        var core = e.Core;
        var bodies = core.Bodies;
        var origin = core.Origin;
        var p = bodies.PositionOf(b);
        int o = 0;
        for (int k = 0; k < l.PositionLanes; k++) { p[k] = BitConverter.Int64BitsToDouble(BinaryPrimitives.ReadInt64LittleEndian(s.Slice(o))) - origin[k]; o += 8; }
        o = GetFloats(s, o, bodies.RotationOf(b));
        o = GetFloats(s, o, bodies.LinearVelocityOf(b));
        GetFloats(s, o, bodies.AngularVelocityOf(b));
        e.RefreshBodyProxies(b);
    }

    private static int GetFloats(ReadOnlySpan<byte> s, int o, Span<float> into)
    {
        for (int k = 0; k < into.Length; k++) { into[k] = BitConverter.UInt32BitsToSingle(BinaryPrimitives.ReadUInt32LittleEndian(s.Slice(o))); o += 4; }
        return o;
    }

    public static bool Asleep(ReadOnlySpan<byte> state) => state[state.Length - 1] != 0;

    /// <summary>The shared-entity hash over inside bodies in ascending authority slot (docs/spec/interest.md); the sleeping byte is not part of it.</summary>
    public static ulong Hash(in InterestLayout l, List<(int Slot, int Generation, byte Type, byte[] State)> items)
    {
        var w = StateWriter.ForHash();
        w.WriteInt32(l.Dimension);
        w.WriteInt32(items.Count);
        foreach (var it in items)
        {
            w.WriteInt32(it.Slot); w.WriteInt32(it.Generation); w.WriteByte(it.Type);
            w.WriteBytes(it.State.AsSpan(0, it.State.Length - 1));
        }

        return w.Hash;
    }
}
