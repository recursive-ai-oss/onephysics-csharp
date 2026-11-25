using System;
using System.Collections.Generic;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>
/// The authority's side of an interest stream: for one client, writes a snapshot and then deltas of what lies inside a region (see <see cref="InterestRegion"/>), with kinematic proxies for the bodies outside that the inside
/// bodies touch or hang from. A writer remembers what it sent, so it serves one client; make one per client and feed each its messages in order. Create it with <c>CreateInterestWriter</c> on a world.
/// </summary>
/// <remarks>Layout: <c>docs/spec/interest.md</c>. A message depends on the one before it: a client that missed one needs a new snapshot (<see cref="Reset"/>).</remarks>
public sealed class InterestWriter
{
    internal const uint Magic = 0x4E49504F;   // "OPIN"
    internal const uint Version = 1;

    private readonly ITransferEngine _e;
    private readonly string _what;
    private readonly InterestOptions _options;
    private readonly InterestLayout _layout;

    private long _next;
    private byte[] _prevRole = Array.Empty<byte>();
    private int[] _prevGen = Array.Empty<int>();
    private byte[]?[] _prevState = Array.Empty<byte[]?>();
    // The joints the client holds (authority joint slot and generation, as slot << 32 | generation) with the slots of their two bodies.
    private SortedDictionary<long, (int A, int B)> _prevJoints = new();

    internal InterestWriter(ITransferEngine engine, string what, InterestOptions? options)
    {
        _e = engine; _what = what; _options = options ?? new InterestOptions();
        _layout = new InterestLayout(engine.Dimension);
    }

    /// <summary>2 or 3.</summary>
    public int Dimension => _layout.Dimension;

    /// <summary>
    /// The bodies the last <see cref="Write"/> left out because a packet cannot carry them: an articulation link, a body with a rope or cloth attached, a rail vehicle, a bicycle, a character holding a body that is not in the stream,
    /// and the like. A skipped body is not an error: it is simply absent from the client. Empty before the first <see cref="Write"/>.
    /// </summary>
    public IReadOnlyList<InterestSkip> LastSkipped => _skipped;

    private IReadOnlyList<InterestSkip> _skipped = Array.Empty<InterestSkip>();

    /// <summary>The sequence number the next message will carry: 0 before the first.</summary>
    public long NextSequence => _next;

    /// <summary>Forgets what was sent: the next message is a snapshot, which makes the client forget everything it held.</summary>
    public void Reset()
    {
        _next = 0;
        _prevRole = Array.Empty<byte>(); _prevGen = Array.Empty<int>(); _prevState = Array.Empty<byte[]?>();
        _prevJoints = new SortedDictionary<long, (int A, int B)>();
    }

    /// <summary>
    /// The hash of the entities this writer's world and a client's world both hold for <paramref name="region"/>: the inside bodies, in ascending body slot, each by slot, generation, type, position, rotation, linear and angular velocity
    /// (the same record <see cref="InterestClient.ComputeSharedHash"/> folds). Proxies, joints, contacts, shapes and sleeping are not part of it. Allocates; no state changes.
    /// </summary>
    /// <exception cref="ArgumentNullException"><paramref name="region"/> is null.</exception>
    /// <exception cref="ArgumentException">The region's dimension differs from the world's.</exception>
    public ulong ComputeSharedHash(InterestRegion region)
    {
        Check(region);
        var core = _e.Core;
        core.FlushPostStep();
        var roles = InterestRoles.Compute(_e, region, _options.Proxies, false);
        var items = new List<(int, int, byte, byte[])>();
        for (int b = 0; b < roles.Role.Length; b++)
        {
            if (roles.Role[b] != InterestRoles.Inside) { continue; }
            items.Add((b, core.Bodies.Handles.GenerationOf(b), (byte)core.Bodies.TypeOf(b), InterestRoles.EncodeState(core, _layout, b)));
        }

        return InterestRoles.Hash(_layout, items);
    }

    private void Check(InterestRegion region)
    {
        ArgumentNullException.ThrowIfNull(region);
        if (region.Dimension != 0 && region.Dimension != _layout.Dimension) { throw new ArgumentException("The region holds " + region.Dimension + "D boxes and the world is " + _layout.Dimension + "D.", nameof(region)); }
    }

    /// <summary>
    /// Writes the next message: a snapshot the first time (and after <see cref="Reset"/>), otherwise a delta against the previous message. Does not change the world's simulation state.
    /// </summary>
    /// <param name="region">What the client sees now; it may differ from message to message (a moving interest).</param>
    /// <returns>The message bytes (<c>docs/spec/interest.md</c>), identical on every platform.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="region"/> is null.</exception>
    /// <exception cref="ArgumentException">The region's dimension differs from the world's.</exception>
    public byte[] Write(InterestRegion region)
    {
        Check(region);
        var core = _e.Core;
        core.FlushPostStep();
        var roles = InterestRoles.Compute(_e, region, _options.Proxies, true);
        var role = roles.Role;
        var bodies = core.Bodies;
        int high = role.Length;

        bool PresentBefore(int b) => b < _prevRole.Length && _prevRole[b] != 0 && bodies.Handles.IsAlive(b, _prevGen[b]);
        // A dynamic body changes how the client simulates it when its role changes (inside is dynamic there, a proxy is kinematic), so it leaves and enters again; so does a body that carries a character or vehicle, because
        // only an inside body comes with that state. Any other kind just takes the new role.
        bool Reenter(int b) => PresentBefore(b) && role[b] != 0 && _prevRole[b] != role[b] && (bodies.TypeOf(b) == BodyType.Dynamic || _e.HasEntity(b));

        var leaves = new List<(int Slot, int Gen)>();
        for (int b = 0; b < _prevRole.Length; b++)
        {
            if (_prevRole[b] == 0) { continue; }
            bool alive = bodies.Handles.IsAlive(b, _prevGen[b]);
            if (!alive || role[b] == 0 || (alive && Reenter(b))) { leaves.Add((b, _prevGen[b])); }
        }

        var leaving = new bool[Math.Max(high, _prevRole.Length)];
        foreach (var (slot, _) in leaves) { leaving[slot] = true; }

        var entrants = new List<int>();
        var roleChanges = new List<int>();
        for (int b = 0; b < high; b++)
        {
            if (role[b] == 0) { continue; }
            if (!PresentBefore(b) || leaving[b]) { entrants.Add(b); }
            else if (_prevRole[b] != role[b]) { roleChanges.Add(b); }
        }

        byte[] packetBytes = Array.Empty<byte>();
        var entering = new bool[high];
        foreach (int b in entrants) { entering[b] = true; }
        if (entrants.Count > 0)
        {
            var handles = new BodyHandle[entrants.Count];
            for (int i = 0; i < handles.Length; i++) { handles[i] = core.HandleOfBody(entrants[i]); }
            var packet = BodySetTransfer.Extract(_e, handles, new BodySetOptions { Move = false, Boundary = BoundaryPolicy.Break }, _what, roles);
            if (packet.BodyCount != entrants.Count || packet.AnchorCount != 0) { throw new BodySetTransferException(_what + ": the entering bodies pulled in others (" + packet.BodyCount + " bodies for " + entrants.Count + "); this is a bug in the interest roles."); }
            packetBytes = packet.ToArray();
        }

        // Joints: what the client holds after its bodies left, what it should hold, and the difference.
        var cs = core.Constraints;
        var desired = new SortedDictionary<long, (int A, int B)>();
        for (int c = 0; c < roles.Desired.Length; c++)
        {
            if (!roles.Desired[c]) { continue; }
            desired[Key(c, cs.Handles.GenerationOf(c))] = (cs.BodyA[c], cs.BodyB[c]);
        }

        var removes = new List<long>();
        var adds = new List<(long Key, int C)>();
        foreach (var kv in _prevJoints)
        {
            bool gone = Leaves(leaving, kv.Value.A) || Leaves(leaving, kv.Value.B);
            if (!gone && !desired.ContainsKey(kv.Key)) { removes.Add(kv.Key); }
        }

        foreach (var kv in desired)
        {
            bool held = _prevJoints.TryGetValue(kv.Key, out var old) && !Leaves(leaving, old.A) && !Leaves(leaving, old.B);
            if (held) { continue; }
            if (entering[kv.Value.A] && entering[kv.Value.B]) { continue; }   // travels in the packet
            adds.Add((kv.Key, (int)(kv.Key >> 32)));
        }

        // Body states: those that differ from what was last sent.
        var states = new List<(int Slot, byte[] State)>();
        var nowState = new byte[high][];
        for (int b = 0; b < high; b++)
        {
            if (role[b] == 0) { continue; }
            nowState[b] = InterestRoles.EncodeState(core, _layout, b);
            if (entering[b] || !PresentBefore(b)) { continue; }
            var before = b < _prevState.Length ? _prevState[b] : null;
            if (before == null || !before.AsSpan().SequenceEqual(nowState[b])) { states.Add((b, nowState[b])); }
        }

        // ---- write
        var w = StateWriter.ToBuffer();
        bool snapshot = _next == 0;
        w.WriteUInt32(Magic); w.WriteUInt32(Version);
        w.WriteInt32(_layout.Dimension);
        w.WriteDouble(_e.MetresPerUnit);
        w.WriteByte(snapshot ? (byte)0 : (byte)1);
        w.WriteInt64(_next);
        w.WriteInt64(snapshot ? -1 : _next - 1);
        w.WriteInt32(leaves.Count);
        foreach (var (slot, gen) in leaves) { w.WriteInt32(slot); w.WriteInt32(gen); }
        w.WriteInt32(entrants.Count);
        foreach (int b in entrants) { w.WriteInt32(b); w.WriteInt32(bodies.Handles.GenerationOf(b)); w.WriteByte(role[b]); }
        w.WriteInt32(roleChanges.Count);
        foreach (int b in roleChanges) { w.WriteInt32(b); w.WriteInt32(bodies.Handles.GenerationOf(b)); w.WriteByte(role[b]); }
        w.WriteInt32(packetBytes.Length);
        w.WriteBytes(packetBytes);
        w.WriteInt32(removes.Count);
        foreach (long k in removes) { w.WriteInt32((int)(k >> 32)); w.WriteInt32((int)(k & 0xFFFFFFFF)); }
        w.WriteInt32(adds.Count);
        foreach (var (key, c) in adds)
        {
            w.WriteInt32((int)(key >> 32)); w.WriteInt32((int)(key & 0xFFFFFFFF));
            int a = cs.BodyA[c], b = cs.BodyB[c];
            w.WriteInt32(a); w.WriteInt32(bodies.Handles.GenerationOf(a));
            w.WriteInt32(b); w.WriteInt32(bodies.Handles.GenerationOf(b));
            w.WriteByte((byte)(cs.Flags[c] & (byte)(ConstraintBits.ReportPersist | ConstraintBits.HasLifetime)));
            w.WriteByte(cs.Color[c]);
            var blob = _e.WriteJoint(c);
            w.WriteInt32(blob.Length); w.WriteBytes(blob);
            var payload = cs.Payload(c);
            for (int k = 0; k < _layout.PayloadFloats; k++) { w.WriteSingle(payload[k]); }
        }

        w.WriteInt32(states.Count);
        foreach (var (slot, st) in states) { w.WriteInt32(slot); w.WriteInt32(bodies.Handles.GenerationOf(slot)); w.WriteBytes(st); }
        var bytes = w.ToArray();

        // ---- remember
        _skipped = roles.Skipped.Count == 0 ? Array.Empty<InterestSkip>() : roles.Skipped.ToArray();
        _prevRole = (byte[])role.Clone();
        _prevGen = new int[high];
        _prevState = new byte[]?[high];
        for (int b = 0; b < high; b++)
        {
            if (role[b] == 0) { continue; }
            _prevGen[b] = bodies.Handles.GenerationOf(b);
            _prevState[b] = nowState[b];
        }

        _prevJoints = desired;
        _next++;
        return bytes;
    }

    private static bool Leaves(bool[] leaving, int slot) => (uint)slot < (uint)leaving.Length && leaving[slot];

    private static long Key(int slot, int generation) => ((long)slot << 32) | (uint)generation;
}
