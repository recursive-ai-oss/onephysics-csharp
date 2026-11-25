using System;
using System.Buffers.Binary;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>
/// The client's side of an interest stream: applies the authority's messages to a world so that it holds the bodies inside the interest region as ordinary bodies and the bodies around them as kinematic proxies, and
/// answers for the hash of what both sides hold. Create it with <c>CreateInterestClient</c> on a world. The world may also hold other bodies; the client only ever touches the ones it created.
/// </summary>
/// <remarks>
/// The client may step its world between messages to predict; the next <see cref="Apply"/> first puts every body it holds back to the state the authority last sent (the body's own pose, velocities and sleeping; solver
/// state such as warm starts is the client's own), then applies the message. Do not destroy bodies the client created; to drop them all call <see cref="Reset"/>. Layout of the messages: <c>docs/spec/interest.md</c>.
/// </remarks>
public sealed class InterestClient
{
    private sealed class Entry
    {
        public int Generation;
        public byte Role;
        public BodyHandle Local;
        public byte[] State = Array.Empty<byte>();
    }

    private sealed class JointAdd
    {
        public long Key;
        public int ASlot, AGen, BSlot, BGen;
        public PacketJoint Joint = new();
    }

    private sealed class Message
    {
        public byte Kind;
        public long Sequence, Base;
        public readonly List<(int Slot, int Gen)> Leaves = new();
        public readonly List<(int Slot, int Gen, byte Role)> Entrants = new();
        public readonly List<(int Slot, int Gen, byte Role)> RoleChanges = new();
        public BodySetPacket? Packet;
        public readonly List<long> JointRemoves = new();
        public readonly List<JointAdd> JointAdds = new();
        public readonly List<(int Slot, int Gen, byte[] State)> States = new();
    }

    private readonly ITransferEngine _e;
    private readonly Func<string, IDisposable?> _record;
    private readonly string _what;
    private readonly InterestLayout _layout;
    private readonly SortedDictionary<int, Entry> _entries = new();
    private readonly SortedDictionary<long, (JointHandle Handle, int A, int B)> _joints = new();
    private readonly Dictionary<int, int> _authorityOfLocal = new();
    private long _sequence = -1;

    internal InterestClient(ITransferEngine engine, string what, Func<string, IDisposable?> record)
    {
        _e = engine; _what = what; _record = record;
        _layout = new InterestLayout(engine.Dimension);
    }

    /// <summary>The sequence number of the last message applied, -1 before the first.</summary>
    public long Sequence => _sequence;

    /// <summary>Bodies held as real bodies (inside the interest region).</summary>
    public int SharedBodyCount { get { int n = 0; foreach (var kv in _entries) { if (kv.Value.Role == InterestRoles.Inside) { n++; } } return n; } }

    /// <summary>Bodies held as kinematic (or static) proxies.</summary>
    public int ProxyCount { get { int n = 0; foreach (var kv in _entries) { if (kv.Value.Role == InterestRoles.Proxy) { n++; } } return n; } }

    /// <summary>Joints the client holds that came from the stream (a joint whose body the client destroyed is not counted).</summary>
    public int JointCount { get { int n = 0; foreach (var kv in _joints) { if (_e.Core.Constraints.Handles.IsAlive(kv.Value.Handle.Index, kv.Value.Handle.Generation)) { n++; } } return n; } }

    /// <summary>The local body that stands for an authority body (by the handle the authority's world gave it), or false when the client does not hold it.</summary>
    public bool TryGetLocal(BodyHandle authorityBody, out BodyHandle local)
    {
        if (_entries.TryGetValue(authorityBody.Index, out var e) && e.Generation == authorityBody.Generation && Alive(e)) { local = e.Local; return true; }
        local = default;
        return false;
    }

    /// <summary>The authority body a local body stands for (world 0: the authority's world id is not known to the client), or false when the body is not one the client created.</summary>
    public bool TryGetAuthority(BodyHandle local, out BodyHandle authorityBody)
    {
        if (local.World == _e.Core.Id && _authorityOfLocal.TryGetValue(local.Index, out int slot) && _entries.TryGetValue(slot, out var e) && e.Local == local)
        {
            authorityBody = new BodyHandle(0, slot, e.Generation);
            return true;
        }

        authorityBody = default;
        return false;
    }

    /// <summary>True when the local body is a proxy: it follows the authority's pose and the client's solver cannot push it.</summary>
    public bool IsProxy(BodyHandle local) => local.World == _e.Core.Id && _authorityOfLocal.TryGetValue(local.Index, out int slot) && _entries.TryGetValue(slot, out var e) && e.Local == local && e.Role == InterestRoles.Proxy;

    /// <summary>
    /// The hash of the bodies the client holds as inside bodies, as they are now: the same fold as <see cref="InterestWriter.ComputeSharedHash"/>, so after applying message n and before stepping, it equals the authority's hash
    /// for the region of message n (taken after the authority's step n). After predicting it differs, which is how a client sees how far it ran from the authority.
    /// </summary>
    public ulong ComputeSharedHash()
    {
        var core = _e.Core;
        core.FlushPostStep();
        var items = new List<(int, int, byte, byte[])>();
        foreach (var kv in _entries)
        {
            var e = kv.Value;
            if (e.Role != InterestRoles.Inside || !Alive(e)) { continue; }
            int b = e.Local.Index;
            items.Add((kv.Key, e.Generation, (byte)core.Bodies.TypeOf(b), InterestRoles.EncodeState(core, _layout, b)));
        }

        return InterestRoles.Hash(_layout, items);
    }

    /// <summary>Destroys every body the client created and forgets the stream: the next message must be a snapshot.</summary>
    public void Reset()
    {
        using var scope = _record(_what + ".Reset");
        Clear();
    }

    private void Clear()
    {
        foreach (var kv in _entries) { if (Alive(kv.Value)) { _e.DestroyBody(kv.Value.Local); } }
        _entries.Clear(); _joints.Clear(); _authorityOfLocal.Clear();
        _sequence = -1;
    }

    private bool Alive(Entry e) => _e.Core.Bodies.Handles.IsAlive(e.Local.Index, e.Local.Generation);

    /// <summary>
    /// Puts every body the client holds back to the state the authority last sent. <see cref="Apply"/> does this first; call it yourself to throw a prediction away without waiting for a message.
    /// </summary>
    public void RevertPrediction()
    {
        using var scope = _record(_what + ".RevertPrediction");
        _e.Core.FlushPostStep();
        Revert();
    }

    private void Revert()
    {
        var core = _e.Core;
        var bodies = core.Bodies;
        foreach (var kv in _entries)
        {
            var e = kv.Value;
            if (!Alive(e)) { continue; }
            if (!InterestRoles.EncodeState(core, _layout, e.Local.Index).AsSpan(0, _layout.StateBytes - 1).SequenceEqual(e.State.AsSpan(0, _layout.StateBytes - 1))) { InterestRoles.DecodeState(_e, _layout, e.Local.Index, e.State); }
        }

        FixSleep(null);
    }

    /// <summary>Makes the sleeping flag of the bodies (all, or the listed authority slots) what the authority said: sleeps first, wakes last, so a body the authority has awake is awake.</summary>
    private void FixSleep(List<int>? slots)
    {
        var core = _e.Core;
        var bodies = core.Bodies;
        IEnumerable<KeyValuePair<int, Entry>> Each()
        {
            if (slots == null) { foreach (var kv in _entries) { yield return kv; } }
            else { foreach (int s in slots) { if (_entries.TryGetValue(s, out var e)) { yield return new KeyValuePair<int, Entry>(s, e); } } }
        }

        foreach (var kv in Each())
        {
            var e = kv.Value;
            if (!Alive(e) || bodies.TypeOf(e.Local.Index) != BodyType.Dynamic) { continue; }
            if (InterestRoles.Asleep(e.State) && bodies.IsAwake(e.Local.Index)) { core.Sleep(e.Local); }
        }

        foreach (var kv in Each())
        {
            var e = kv.Value;
            if (!Alive(e) || bodies.TypeOf(e.Local.Index) != BodyType.Dynamic) { continue; }
            if (!InterestRoles.Asleep(e.State) && !bodies.IsAwake(e.Local.Index)) { core.Wake(e.Local); }
        }
    }

    // ------------------------------------------------------------------ apply

    /// <summary>
    /// Applies the next message of the stream. The bytes are read and checked completely, against the message before it, before the world is touched; a message that fails there changes nothing. Entering bodies are
    /// inserted exactly as <c>InsertBodies</c> does (lowest free slots, in the packet's order), bodies that left are destroyed, then joints and body states are applied.
    /// </summary>
    /// <param name="message">One message written by <see cref="InterestWriter.Write"/>.</param>
    /// <param name="maximumBytes">Largest message accepted; default 256 MiB.</param>
    /// <returns>What changed.</returns>
    /// <exception cref="InterestStreamException">The bytes are not a valid message, it is for another dimension or length unit, it does not follow the message applied before it (a delta needs the previous sequence number), or it names a body or joint the client does not hold.</exception>
    /// <exception cref="BodySetTransferException">A body would land beyond the world's range; the insertion is atomic, but the client may be left with earlier steps of this message applied. Reset and ask for a snapshot.</exception>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="maximumBytes"/> is negative.</exception>
    public InterestApplyResult Apply(ReadOnlySpan<byte> message, int maximumBytes = 256 * 1024 * 1024)
    {
        if (maximumBytes < 0) { throw new ArgumentOutOfRangeException(nameof(maximumBytes), maximumBytes, "maximumBytes must be at least 0."); }
        if (message.Length > maximumBytes) { throw new InterestStreamException("message", 0, "the message is " + message.Length.ToString(CultureInfo.InvariantCulture) + " bytes, above the limit of " + maximumBytes.ToString(CultureInfo.InvariantCulture) + "."); }
        var m = Parse(message.ToArray());
        var core = _e.Core;
        using var scope = _record(_what);
        core.FlushPostStep();
        bool snapshot = m.Kind == 0;
        if (snapshot) { if (m.Sequence != 0) { throw new InterestStreamException("sequence", 0, "a snapshot has sequence 0, found " + m.Sequence.ToString(CultureInfo.InvariantCulture)); } }
        else if (m.Base != _sequence || m.Sequence != _sequence + 1)
        {
            throw new InterestStreamException("baseSequence", 0, "this delta follows message " + m.Base.ToString(CultureInfo.InvariantCulture) + " and the client has applied " + (_sequence < 0 ? "none" : _sequence.ToString(CultureInfo.InvariantCulture)) + "; ask the authority for a snapshot");
        }

        Validate(m, snapshot);

        // ---- mutate
        if (snapshot) { Clear(); } else { Revert(); }
        var leavers = new List<Entry>();
        foreach (var (slot, _) in m.Leaves) { var e = _entries[slot]; leavers.Add(e); _entries.Remove(slot); }

        var entered = new List<int>();
        if (m.Packet != null)
        {
            var map = BodySetTransfer.Insert(_e, m.Packet, _identity ??= MakeIdentity(), _what);
            var oldBodies = map.OldBodies; var newBodies = map.NewBodies;
            var slotOf = new int[oldBodies.Length];
            for (int i = 0; i < oldBodies.Length; i++)
            {
                var (slot, gen, role) = m.Entrants[i];
                slotOf[i] = slot;
                var entry = new Entry { Generation = gen, Role = role, Local = newBodies[i] };
                _entries[slot] = entry;
                _authorityOfLocal[newBodies[i].Index] = slot;
                entered.Add(slot);
            }

            var oldJoints = map.OldJoints; var newJoints = map.NewJoints;
            for (int i = 0; i < oldJoints.Length; i++) { _joints[Key(oldJoints[i].Index, oldJoints[i].Generation)] = (newJoints[i], slotOf[m.Packet.Model.Joints[i].BodyA], slotOf[m.Packet.Model.Joints[i].BodyB]); }
            foreach (int slot in entered)
            {
                var entry = _entries[slot];
                entry.State = InterestRoles.EncodeState(core, _layout, entry.Local.Index);
            }
        }

        foreach (var e in leavers)
        {
            if (Alive(e)) { _e.DestroyBody(e.Local); }
            _authorityOfLocal.Remove(e.Local.Index);
        }

        PruneJoints();
        int removed = 0;
        foreach (long key in m.JointRemoves)
        {
            if (_joints.TryGetValue(key, out var jh)) { _joints.Remove(key); if (core.Constraints.Handles.IsAlive(jh.Handle.Index, jh.Handle.Generation)) { core.DestroyJoint(jh.Handle); removed++; } }
        }

        foreach (var ja in m.JointAdds)
        {
            var a = _entries[ja.ASlot].Local; var b = _entries[ja.BSlot].Local;
            core.ColorHint = ja.Joint.Color;
            try { _joints[ja.Key] = (_e.CreateJoint(a, b, ja.Joint), ja.ASlot, ja.BSlot); }
            finally { core.ColorHint = -1; }
        }

        foreach (var (slot, _, role) in m.RoleChanges) { _entries[slot].Role = role; }
        var written = new List<int>();
        foreach (var (slot, _, state) in m.States)
        {
            var e = _entries[slot];
            e.State = state;
            if (!Alive(e)) { continue; }
            InterestRoles.DecodeState(_e, _layout, e.Local.Index, state);
            written.Add(slot);
        }

        FixSleep(written);
        _sequence = m.Sequence;
        return new InterestApplyResult(snapshot ? InterestMessageKind.Snapshot : InterestMessageKind.Delta, m.Sequence, entered.Count, leavers.Count, written.Count, m.JointAdds.Count, removed);
    }

    private TransferTransform? _identity;

    private TransferTransform MakeIdentity() => _layout.Dimension == 3 ? TransferTransform.Create3D(default, null) : TransferTransform.Create2D(default, 0f);

    private void PruneJoints()
    {
        var dead = new List<long>();
        foreach (var kv in _joints) { if (!_e.Core.Constraints.Handles.IsAlive(kv.Value.Handle.Index, kv.Value.Handle.Generation)) { dead.Add(kv.Key); } }
        foreach (long k in dead) { _joints.Remove(k); }
    }

    private static long Key(int slot, int generation) => ((long)slot << 32) | (uint)generation;

    /// <summary>Checks the message against what the client holds, so that applying it cannot fail for a reason in the bytes.</summary>
    private void Validate(Message m, bool snapshot)
    {
        var held = new SortedDictionary<int, int>();
        if (!snapshot) { foreach (var kv in _entries) { held[kv.Key] = kv.Value.Generation; } }
        foreach (var (slot, gen) in m.Leaves)
        {
            if (!held.TryGetValue(slot, out int g) || g != gen) { throw new InterestStreamException("leaves", 0, "body " + slot.ToString(CultureInfo.InvariantCulture) + " (generation " + gen.ToString(CultureInfo.InvariantCulture) + ") is not held by this client"); }
            held.Remove(slot);
        }

        foreach (var (slot, gen, _) in m.RoleChanges)
        {
            if (!held.TryGetValue(slot, out int g) || g != gen) { throw new InterestStreamException("roleChanges", 0, "body " + slot.ToString(CultureInfo.InvariantCulture) + " is not held by this client"); }
        }

        foreach (var (slot, gen, _) in m.Entrants)
        {
            if (held.ContainsKey(slot)) { throw new InterestStreamException("entrants", 0, "body " + slot.ToString(CultureInfo.InvariantCulture) + " is already held by this client"); }
            held[slot] = gen;
        }

        foreach (var (slot, gen, _) in m.States)
        {
            if (!held.TryGetValue(slot, out int g) || g != gen) { throw new InterestStreamException("states", 0, "body " + slot.ToString(CultureInfo.InvariantCulture) + " is not held by this client"); }
        }

        var entering = new HashSet<int>();
        foreach (var (slot, _, _) in m.Entrants) { entering.Add(slot); }
        foreach (var (slot, _, _) in m.States) { if (entering.Contains(slot)) { throw new InterestStreamException("states", 0, "body " + slot.ToString(CultureInfo.InvariantCulture) + " enters in this message and also has a state record"); } }
        var leaving = new HashSet<int>();
        foreach (var (slot, _) in m.Leaves) { leaving.Add(slot); }
        bool Held(long key) => !snapshot && _joints.TryGetValue(key, out var j) && !leaving.Contains(j.A) && !leaving.Contains(j.B);
        foreach (long key in m.JointRemoves)
        {
            if (!Held(key)) { throw new InterestStreamException("jointRemoves", 0, "joint " + (key >> 32).ToString(CultureInfo.InvariantCulture) + " is not held by this client"); }
        }

        var seen = new HashSet<long>();
        var removing = new HashSet<long>(m.JointRemoves);
        foreach (var ja in m.JointAdds)
        {
            if (!held.TryGetValue(ja.ASlot, out int ga) || ga != ja.AGen || !held.TryGetValue(ja.BSlot, out int gb) || gb != ja.BGen)
            { throw new InterestStreamException("jointAdds", 0, "joint " + (ja.Key >> 32).ToString(CultureInfo.InvariantCulture) + " joins a body the client does not hold"); }
            if (ja.ASlot == ja.BSlot) { throw new InterestStreamException("jointAdds", 0, "joint " + (ja.Key >> 32).ToString(CultureInfo.InvariantCulture) + " joins a body to itself"); }
            if (!seen.Add(ja.Key) || (Held(ja.Key) && !removing.Contains(ja.Key))) { throw new InterestStreamException("jointAdds", 0, "joint " + (ja.Key >> 32).ToString(CultureInfo.InvariantCulture) + " is already held"); }
        }
    }

    // ------------------------------------------------------------------ parse

    private sealed class Reader
    {
        private readonly byte[] _d;
        public int Pos;
        public Reader(byte[] d) { _d = d; }
        public int Remaining => _d.Length - Pos;
        public InterestStreamException Bad(string field, string message) => new(field, Pos, message);

        public ReadOnlySpan<byte> Take(int n, string field)
        {
            if (n < 0 || Remaining < n) { throw Bad(field, "the message ends here; " + n.ToString(CultureInfo.InvariantCulture) + " more bytes were expected, " + Remaining.ToString(CultureInfo.InvariantCulture) + " remain"); }
            var s = _d.AsSpan(Pos, n);
            Pos += n;
            return s;
        }

        public byte Byte(string f) => Take(1, f)[0];
        public int Int(string f) => BinaryPrimitives.ReadInt32LittleEndian(Take(4, f));
        public uint UInt(string f) => BinaryPrimitives.ReadUInt32LittleEndian(Take(4, f));
        public long Long(string f) => BinaryPrimitives.ReadInt64LittleEndian(Take(8, f));
        public double Double(string f) => BitConverter.UInt64BitsToDouble(BinaryPrimitives.ReadUInt64LittleEndian(Take(8, f)));

        public int Count(string f, int minBytesEach)
        {
            int n = Int(f);
            if (n < 0 || (long)n * minBytesEach > Remaining) { Pos -= 4; throw Bad(f, "count " + n.ToString(CultureInfo.InvariantCulture) + " needs at least " + ((long)n * minBytesEach).ToString(CultureInfo.InvariantCulture) + " bytes but " + Remaining.ToString(CultureInfo.InvariantCulture) + " remain"); }
            return n;
        }

        public int Slot(string f)
        {
            int s = Int(f);
            if (s < 0) { Pos -= 4; throw Bad(f, "a body or joint slot is not negative, found " + s.ToString(CultureInfo.InvariantCulture)); }
            return s;
        }

        public int Generation(string f)
        {
            int g = Int(f);
            if (g <= 0) { Pos -= 4; throw Bad(f, "a generation is positive, found " + g.ToString(CultureInfo.InvariantCulture)); }
            return g;
        }
    }

    private static string At(string name, int i, string field) => name + "[" + i.ToString(CultureInfo.InvariantCulture) + "]." + field;

    private Message Parse(byte[] data)
    {
        var r = new Reader(data);
        var m = new Message();
        if (r.UInt("magic") != InterestWriter.Magic) { r.Pos = 0; throw r.Bad("magic", "not an interest message"); }
        uint version = r.UInt("version");
        if (version != InterestWriter.Version) { r.Pos -= 4; throw r.Bad("version", "unsupported message version " + version.ToString(CultureInfo.InvariantCulture) + "; this reader knows version 1"); }
        int dim = r.Int("dimension");
        if (dim != _layout.Dimension) { r.Pos -= 4; throw r.Bad("dimension", "the message is for a " + dim.ToString(CultureInfo.InvariantCulture) + "D world and this client's world is " + _layout.Dimension.ToString(CultureInfo.InvariantCulture) + "D"); }
        double unit = r.Double("metresPerUnit");
        if (BitConverter.DoubleToInt64Bits(unit) != BitConverter.DoubleToInt64Bits(_e.MetresPerUnit)) { r.Pos -= 8; throw r.Bad("metresPerUnit", "the message's length unit is " + unit.ToString("R", CultureInfo.InvariantCulture) + " m per unit and this world's is " + _e.MetresPerUnit.ToString("R", CultureInfo.InvariantCulture) + "; create both worlds with the same LengthUnit"); }
        m.Kind = r.Byte("kind");
        if (m.Kind > 1) { r.Pos--; throw r.Bad("kind", "must be 0 (snapshot) or 1 (delta)"); }
        m.Sequence = r.Long("sequence");
        m.Base = r.Long("baseSequence");
        if (m.Sequence < 0) { throw r.Bad("sequence", "must not be negative"); }
        if (m.Kind == 0 && m.Base != -1) { throw r.Bad("baseSequence", "a snapshot has no base (-1)"); }

        int n = r.Count("leaveCount", 8);
        int prev = -1;
        for (int i = 0; i < n; i++)
        {
            int slot = r.Slot(At("leaves", i, "slot")); int gen = r.Generation(At("leaves", i, "generation"));
            if (slot <= prev) { throw r.Bad(At("leaves", i, "slot"), "slots must ascend"); }
            prev = slot;
            m.Leaves.Add((slot, gen));
        }

        n = r.Count("entrantCount", 9);
        prev = -1;
        for (int i = 0; i < n; i++)
        {
            int slot = r.Slot(At("entrants", i, "slot")); int gen = r.Generation(At("entrants", i, "generation")); byte role = r.Byte(At("entrants", i, "role"));
            if (slot <= prev) { throw r.Bad(At("entrants", i, "slot"), "slots must ascend"); }
            if (role != InterestRoles.Inside && role != InterestRoles.Proxy) { r.Pos--; throw r.Bad(At("entrants", i, "role"), "must be 1 (inside) or 2 (proxy)"); }
            prev = slot;
            m.Entrants.Add((slot, gen, role));
        }

        n = r.Count("roleChangeCount", 9);
        prev = -1;
        for (int i = 0; i < n; i++)
        {
            int slot = r.Slot(At("roleChanges", i, "slot")); int gen = r.Generation(At("roleChanges", i, "generation")); byte role = r.Byte(At("roleChanges", i, "role"));
            if (slot <= prev) { throw r.Bad(At("roleChanges", i, "slot"), "slots must ascend"); }
            if (role != InterestRoles.Inside && role != InterestRoles.Proxy) { r.Pos--; throw r.Bad(At("roleChanges", i, "role"), "must be 1 (inside) or 2 (proxy)"); }
            prev = slot;
            m.RoleChanges.Add((slot, gen, role));
        }

        int packetLength = r.Count("packetLength", 1);
        int packetAt = r.Pos;
        var packetBytes = r.Take(packetLength, "packet");
        if (packetLength == 0)
        {
            if (m.Entrants.Count != 0) { throw r.Bad("packet", "bodies enter but the packet is empty"); }
        }
        else
        {
            try { m.Packet = BodySetPacket.Read(packetBytes); }
            catch (BodySetPacketException e) { throw new InterestStreamException("packet." + e.Field, packetAt + e.Offset, e.Message); }
            var model = m.Packet.Model;
            if (model.Dimension != _layout.Dimension || m.Packet.AnchorCount != 0 || m.Packet.BodyCount != m.Entrants.Count) { throw new InterestStreamException("packet", packetAt, "the packet must hold exactly the " + m.Entrants.Count.ToString(CultureInfo.InvariantCulture) + " entering bodies of this dimension, with no anchors"); }
            for (int i = 0; i < m.Entrants.Count; i++)
            {
                if (model.Bodies[i].Slot != m.Entrants[i].Slot || model.Bodies[i].Generation != m.Entrants[i].Gen) { throw new InterestStreamException("packet", packetAt, "packet body " + i.ToString(CultureInfo.InvariantCulture) + " is not the entering body " + m.Entrants[i].Slot.ToString(CultureInfo.InvariantCulture)); }
            }
        }

        n = r.Count("jointRemoveCount", 8);
        long prevKey = -1;
        for (int i = 0; i < n; i++)
        {
            int slot = r.Slot(At("jointRemoves", i, "slot")); int gen = r.Generation(At("jointRemoves", i, "generation"));
            long key = Key(slot, gen);
            if (key <= prevKey) { throw r.Bad(At("jointRemoves", i, "slot"), "joints must ascend"); }
            prevKey = key;
            m.JointRemoves.Add(key);
        }

        n = r.Count("jointAddCount", 34);
        prevKey = -1;
        for (int i = 0; i < n; i++)
        {
            var ja = new JointAdd();
            int slot = r.Slot(At("jointAdds", i, "slot")); int gen = r.Generation(At("jointAdds", i, "generation"));
            ja.Key = Key(slot, gen);
            if (ja.Key <= prevKey) { throw r.Bad(At("jointAdds", i, "slot"), "joints must ascend"); }
            prevKey = ja.Key;
            ja.ASlot = r.Slot(At("jointAdds", i, "bodyA")); ja.AGen = r.Generation(At("jointAdds", i, "bodyAGeneration"));
            ja.BSlot = r.Slot(At("jointAdds", i, "bodyB")); ja.BGen = r.Generation(At("jointAdds", i, "bodyBGeneration"));
            byte flags = r.Byte(At("jointAdds", i, "flags"));
            if ((flags & ~(byte)(ConstraintBits.ReportPersist | ConstraintBits.HasLifetime)) != 0) { r.Pos--; throw r.Bad(At("jointAdds", i, "flags"), "unknown flag bits"); }
            byte color = r.Byte(At("jointAdds", i, "color"));
            if (color > 23 && color < 254) { r.Pos--; throw r.Bad(At("jointAdds", i, "color"), "a colour is 0..23, 254 or 255"); }
            int blobLength = r.Count(At("jointAdds", i, "blobLength"), 1);
            int blobAt = r.Pos;
            var blob = r.Take(blobLength, At("jointAdds", i, "blob")).ToArray();
            try { PacketBlobs.ValidateJoint(_layout.Dimension, blob); }
            catch (BodySetPacketException e) { throw new InterestStreamException(At("jointAdds", i, "blob") + "." + e.Field, blobAt + e.Offset, e.Message); }
            var payload = new float[_layout.PayloadFloats];
            for (int k = 0; k < payload.Length; k++)
            {
                float v = BitConverter.UInt32BitsToSingle(r.UInt(At("jointAdds", i, "payload")));
                if (!float.IsFinite(v)) { r.Pos -= 4; throw r.Bad(At("jointAdds", i, "payload[" + k.ToString(CultureInfo.InvariantCulture) + "]"), "must be finite"); }
                payload[k] = v;
            }

            ja.Joint = new PacketJoint { Slot = slot, Generation = gen, Flags = (byte)(flags & (byte)ConstraintBits.ReportPersist), Color = color, Blob = blob, Payload = payload };
            m.JointAdds.Add(ja);
        }

        n = r.Count("stateCount", 8 + _layout.StateBytes);
        prev = -1;
        for (int i = 0; i < n; i++)
        {
            int slot = r.Slot(At("states", i, "slot")); int gen = r.Generation(At("states", i, "generation"));
            if (slot <= prev) { throw r.Bad(At("states", i, "slot"), "slots must ascend"); }
            prev = slot;
            int at = r.Pos;
            var state = r.Take(_layout.StateBytes, At("states", i, "state")).ToArray();
            CheckState(state, r, at, At("states", i, "state"));
            m.States.Add((slot, gen, state));
        }

        if (r.Remaining != 0) { throw r.Bad("end", r.Remaining.ToString(CultureInfo.InvariantCulture) + " trailing bytes"); }
        return m;
    }

    /// <summary>A state record holds finite numbers, a position within the world's range, a rotation of about unit length and velocities no larger than a body can have.</summary>
    private void CheckState(byte[] s, Reader r, int at, string field)
    {
        int o = 0;
        for (int k = 0; k < _layout.PositionLanes; k++, o += 8)
        {
            double v = BitConverter.Int64BitsToDouble(BinaryPrimitives.ReadInt64LittleEndian(s.AsSpan(o)));
            if (!(Math.Abs(v) <= 2097152.0)) { r.Pos = at + o; throw r.Bad(field + ".position[" + k.ToString(CultureInfo.InvariantCulture) + "]", "must be finite and within ±2^21"); }
        }

        double len2 = 0;
        for (int k = 0; k < _layout.RotationLanes + _layout.LinearLanes + _layout.AngularLanes; k++, o += 4)
        {
            float v = BitConverter.UInt32BitsToSingle(BinaryPrimitives.ReadUInt32LittleEndian(s.AsSpan(o)));
            bool rotation = k < _layout.RotationLanes;
            if (!float.IsFinite(v) || (!rotation && Math.Abs(v) > 1e6f)) { r.Pos = at + o; throw r.Bad(field + (rotation ? ".rotation" : ".velocity") + "[" + k.ToString(CultureInfo.InvariantCulture) + "]", rotation ? "must be finite" : "must be finite and at most 1e6"); }
            if (rotation) { len2 += (double)v * v; }
        }

        if (!(Math.Abs(len2 - 1.0) <= 0.01)) { r.Pos = at + 8 * _layout.PositionLanes; throw r.Bad(field + ".rotation", "must have unit length"); }
        if (s[o] > 1) { r.Pos = at + o; throw r.Bad(field + ".asleep", "must be 0 or 1"); }
    }
}
