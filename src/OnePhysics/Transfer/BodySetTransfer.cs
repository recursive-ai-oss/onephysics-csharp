using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>One live shape pair with at least one contact constraint, as the engine's pair table holds it.</summary>
internal readonly struct TransferPair
{
    public readonly int ShapeA, ShapeB, FirstConstraint;
    public TransferPair(int a, int b, int first) { ShapeA = a; ShapeB = b; FirstConstraint = first; }
}

/// <summary>What the dimension-neutral transfer code needs from an engine. Engine3D and Engine2D implement it in their own partial files.</summary>
internal interface ITransferEngine
{
    WorldCore Core { get; }
    int Dimension { get; }
    double MetresPerUnit { get; }

    // ---- extraction
    /// <summary>Throws <see cref="BodySetTransferException"/> naming the feature when a body of the set takes part in something a packet does not carry (a character, a vehicle, a rod attachment...).</summary>
    void CheckMembers(int[] members, bool[] inSet);
    byte[] WriteShape(int shape);
    /// <summary>The joint's record, carrying its remaining lifetime; throws <see cref="BodySetTransferException"/> for a joint kind whose state is not body-relative.</summary>
    byte[] WriteJoint(int constraint);
    /// <summary>Fills the engine's per-body floats and flags of a member body.</summary>
    void ReadBodyExtras(int body, PacketBody into);
    /// <summary>Fills pose and velocities of a kinematic anchor standing for the body: its origin frame, and the velocity of its origin.</summary>
    void ReadAnchor(int body, PacketBody into);
    void CollectPairs(List<TransferPair> pairs);
    int NextInChain(int constraint);
    void DestroyBody(BodyHandle handle);
    /// <summary>The slot of a live shape on a static body that is the same shape (geometry, local pose) at exactly the given pose, or -1.</summary>
    int FindSupport(byte[] blob, ReadOnlySpan<double> relPosition, ReadOnlySpan<float> rotation);
    /// <summary>Turns a contact payload round, as if shape A and shape B had traded places: the normal and the arms swap sides; impulses that depend on the tangent basis and the manifold-reuse data restart from zero.</summary>
    void SwapContactPayload(Span<float> payload);
    /// <summary>Adds to the set every body that belongs, through a ragdoll, to a body already in it.</summary>
    void ExpandSet(bool[] inSet);
    /// <summary>The ragdolls whose parts are all in the set, as packet records, in ascending slot order. <paramref name="bodyIndex"/> maps a body slot, <paramref name="jointIndex"/> a constraint slot, to packet indices.</summary>
    void WriteRagdolls(bool[] inSet, int[] bodyIndex, int[] jointIndex, List<PacketRagdoll> into);
    /// <summary>Rebuilds a ragdoll from its record on the new bodies and joints; returns its handle.</summary>
    RagdollHandle CreateRagdoll(PacketRagdoll record, BodyHandle[] bodies, JointHandle[] joints);
    /// <summary>
    /// The characters and then the vehicles whose body is in <paramref name="entitySet"/>, as packet records in ascending slot order; bodies are named by <paramref name="bodyIndex"/> (a body that is in the packet but
    /// outside <paramref name="inSet"/> cannot happen: <see cref="CheckMembers"/> refused it). Fields that name a body outside the packet are cleared or refused as the spec says.
    /// </summary>
    void WriteEntities(bool[] inSet, bool[] entitySet, int[] bodyIndex, List<PacketEntity> into);
    /// <summary>Rebuilds a character or vehicle from its record on the new bodies, with its world-space values taken through the transform; returns its new slot and generation.</summary>
    (int Slot, int Generation) CreateEntity(PacketEntity record, PacketModel model, BodyHandle[] bodies, TransferTransform transform);
    /// <summary>True when the body is the body of a character or the chassis of a vehicle (something that travels with an entity record).</summary>
    bool HasEntity(int body);
    /// <summary>
    /// Removes from <paramref name="set"/> every body a packet could not carry, repeatedly (taking one out can strand another that depends on it), and says why for each: the feature it takes part in that a packet does not carry,
    /// or a hold that names a body outside the set. Used by interest streams, which skip such bodies instead of refusing.
    /// </summary>
    void FindUncarriable(bool[] set, List<(int Body, string Reason)> skipped);

    // ---- insertion
    /// <summary>Creates the body at its final pose (centre of mass for a member, origin for an anchor), awake, with the engine's per-body floats.</summary>
    BodyHandle CreateBody(PacketBody pb, ReadOnlySpan<double> relPosition, ReadOnlySpan<float> rotation, ReadOnlySpan<float> linear, ReadOnlySpan<float> angular);
    ShapeHandle CreateShape(BodyHandle body, byte[] blob);
    /// <summary>After the body's shapes exist: mass overrides, accumulators, then puts the centre of mass back at <paramref name="relPosition"/> and moves the proxies.</summary>
    void FinishBody(BodyHandle body, PacketBody pb, ReadOnlySpan<double> relPosition, TransferTransform transform);
    JointHandle CreateJoint(BodyHandle a, BodyHandle b, PacketJoint joint);
    /// <summary>Starts the pair table entry for two new shapes; false when the receiving world's filters would not let them collide, in which case their contacts are dropped.</summary>
    bool BeginPair(ShapeHandle a, ShapeHandle b, out int pair);
    void LinkContact(int pair, int previous, int constraint);
    /// <summary>Rotates the world-axis vectors of a contact's payload by the portal rotation and clears the impulses that depend on the tangent basis.</summary>
    void TransformContactPayload(Span<float> payload, TransferTransform transform);
    void CheckInsert(PacketModel model);

    // ---- interest streams
    /// <summary>After a body's pose was written straight into the store: moves the broadphase proxies of its shapes to the new bounds.</summary>
    void RefreshBodyProxies(int body);
}

/// <summary>
/// Narrows what an extraction carries, for interest streams: some bodies of the set travel as kinematic proxies, and joints and contacts that the receiving world could not simulate (kinematic against kinematic) are left out.
/// </summary>
internal interface IExtractFilter
{
    /// <summary>The body travels as a proxy: static stays static, anything else becomes kinematic, with no sleep state or controls.</summary>
    bool IsProxy(int body);
    /// <summary>The joint among the set is carried.</summary>
    bool KeepJoint(int constraint);
    /// <summary>A contact between two bodies of the set (or one of them and a static body outside it) is carried.</summary>
    bool KeepContact(int bodyA, int bodyB);
    /// <summary>The set whose ragdolls are written: ragdolls with a part outside it are not.</summary>
    bool[] RagdollSet(bool[] inSet);
}

/// <summary>Extraction and insertion of body sets, shared by both dimensions.</summary>
internal static class BodySetTransfer
{
    private static string Num(int v) => v.ToString(CultureInfo.InvariantCulture);

    // ------------------------------------------------------------------ extract

    public static BodySetPacket Extract(ITransferEngine e, ReadOnlySpan<BodyHandle> handles, BodySetOptions options, string what) => Extract(e, handles, options, what, null);

    public static BodySetPacket Extract(ITransferEngine e, ReadOnlySpan<BodyHandle> handles, BodySetOptions options, string what, IExtractFilter? filter)
    {
        if (options == null) { options = new BodySetOptions(); }
        if (!Enum.IsDefined(options.Boundary)) { throw new ArgumentOutOfRangeException(nameof(options), options.Boundary, "Unknown boundary policy."); }
        var core = e.Core;
        if (handles.Length == 0) { throw new ArgumentException("A body set needs at least one body.", nameof(handles)); }
        var bodies = core.Bodies;
        int high = bodies.HighWater;
        var inSet = new bool[high];
        foreach (var h in handles)
        {
            int b = core.Resolve(h);
            if (inSet[b]) { throw new ArgumentException("The body " + h + " is listed twice.", nameof(handles)); }
            inSet[b] = true;
        }

        // A ragdoll goes whole: naming one part names them all.
        e.ExpandSet(inSet);
        var list = new List<int>();
        for (int b = 0; b < high; b++) { if (inSet[b]) { list.Add(b); } }
        var members = list.ToArray();
        e.CheckMembers(members, inSet);

        // Joints: both ends in the set go along; one end outside is the boundary.
        var cs = core.Constraints;
        var jointSlots = new List<int>();
        var anchorOf = new SortedDictionary<int, int>();   // outside body slot -> packet body index (assigned below)
        for (int c = 0; c < cs.HighWater; c++)
        {
            if (!cs.IsLive(c) || cs.KindOf(c) != ConstraintKind.Joint) { continue; }
            int a = cs.BodyA[c], b = cs.BodyB[c];
            bool ia = inSet[a], ib = inSet[b];
            if (!ia && !ib) { continue; }
            if (ia && ib) { if (filter != null && !filter.KeepJoint(c)) { continue; } e.WriteJoint(c); jointSlots.Add(c); continue; }
            int outside = ia ? b : a;
            switch (options.Boundary)
            {
                case BoundaryPolicy.Refuse:
                    throw new BodySetTransferException(what + ": the joint " + core.HandleOfJoint(c) + " connects " + core.HandleOfBody(ia ? a : b) + " in the set to " + core.HandleOfBody(outside)
                        + " outside it. Add the outside body to the set, or choose BoundaryPolicy.Break (drop the joint) or BoundaryPolicy.HoldAsAnchor (keep it, with the outside body as a kinematic anchor).");
                case BoundaryPolicy.Break:
                    break;
                default:
                    e.WriteJoint(c);
                    jointSlots.Add(c);
                    anchorOf[outside] = -1;
                    break;
            }
        }

        var model = new PacketModel { Dimension = e.Dimension, MetresPerUnit = e.MetresPerUnit, SourceWorld = core.Id };
        int memberCount = members.Length, anchorCount = anchorOf.Count;
        model.MemberCount = memberCount;
        model.Bodies = new PacketBody[memberCount + anchorCount];
        var packetIndexOfBody = new int[high];
        Array.Fill(packetIndexOfBody, -1);
        int lanes = core.Layout.PositionLanes;
        var origin = core.Origin;
        for (int i = 0; i < memberCount; i++)
        {
            int b = members[i];
            packetIndexOfBody[b] = i;
            model.Bodies[i] = ReadMember(e, core, b, lanes, origin);
            if (filter != null && filter.IsProxy(b))
            {
                var pr = model.Bodies[i];
                if (pr.Type != (byte)BodyType.Static) { pr.Type = (byte)BodyType.Kinematic; }
                pr.Asleep = false; pr.SleepTime = 0f; pr.HasControls = false; pr.HasSleep = false;
            }
        }

        int ai = memberCount;
        foreach (int outside in new List<int>(anchorOf.Keys))
        {
            anchorOf[outside] = ai;
            packetIndexOfBody[outside] = ai;
            var pb = new PacketBody { Slot = outside, Generation = bodies.Handles.GenerationOf(outside), Type = (byte)(bodies.TypeOf(outside) == BodyType.Static ? BodyType.Static : BodyType.Kinematic), Anchor = true };
            e.ReadAnchor(outside, pb);
            for (int k = 0; k < lanes; k++) { pb.Position[k] += origin[k]; }
            model.Bodies[ai++] = pb;
        }

        // Shapes, ascending by slot so new slots keep the order of the old ones (contact shape A stays below shape B).
        var shapes = core.Shapes;
        var shapeList = new List<PacketShape>();
        var packetIndexOfShape = new int[shapes.HighWater];
        Array.Fill(packetIndexOfShape, -1);
        for (int s = 0; s < shapes.HighWater; s++)
        {
            if (!shapes.Handles.IsLive(s) || !inSet[shapes.Owner[s]]) { continue; }
            packetIndexOfShape[s] = shapeList.Count;
            shapeList.Add(new PacketShape { Slot = s, Generation = shapes.Handles.GenerationOf(s), Body = packetIndexOfBody[shapes.Owner[s]], Blob = e.WriteShape(s) });
        }

        model.Shapes = shapeList.ToArray();

        model.Joints = new PacketJoint[jointSlots.Count];
        for (int i = 0; i < jointSlots.Count; i++)
        {
            int c = jointSlots[i];
            model.Joints[i] = new PacketJoint
            {
                Slot = c, Generation = cs.Handles.GenerationOf(c),
                BodyA = packetIndexOfBody[cs.BodyA[c]], BodyB = packetIndexOfBody[cs.BodyB[c]],
                Flags = (byte)(cs.Flags[c] & (byte)(ConstraintBits.ReportPersist | ConstraintBits.HasLifetime)), Color = cs.Color[c], BreakValue = 0f,
                Blob = e.WriteJoint(c), Payload = cs.Payload(c).ToArray(),
            };
        }

        // Contacts between two shapes of the set, pair by pair in ascending shape order, each pair's constraints in chain order.
        var pairs = new List<TransferPair>();
        e.CollectPairs(pairs);
        pairs.Sort((x, y) => x.ShapeA != y.ShapeA ? x.ShapeA.CompareTo(y.ShapeA) : x.ShapeB.CompareTo(y.ShapeB));
        var contacts = new List<PacketContact>();
        foreach (var pair in pairs)
        {
            int ia = (uint)pair.ShapeA < (uint)packetIndexOfShape.Length ? packetIndexOfShape[pair.ShapeA] : -1;
            int ib = (uint)pair.ShapeB < (uint)packetIndexOfShape.Length ? packetIndexOfShape[pair.ShapeB] : -1;
            if (ia < 0 || ib < 0) { continue; }
            if (filter != null && !filter.KeepContact(shapes.Owner[pair.ShapeA], shapes.Owner[pair.ShapeB])) { continue; }
            bool first = true;
            for (int c = pair.FirstConstraint; c >= 0; c = e.NextInChain(c))
            {
                if (cs.KindOf(c) != ConstraintKind.Contact) { continue; }
                var pc = new PacketContact
                {
                    Slot = c, Generation = cs.Handles.GenerationOf(c), ShapeA = ia, ShapeB = ib, SubA = cs.SubShapeA[c], SubB = cs.SubShapeB[c],
                    Flags = (byte)(cs.Flags[c] & (byte)ConstraintBits.ReportPersist), Color = cs.Color[c], Points = cs.PointCount[c], Features = cs.FeatureIdsOf(c).ToArray(),
                    BreakValue = 0f, NewPair = first, Payload = cs.Payload(c).ToArray(),
                };
                first = false;
                contacts.Add(pc);
            }
        }

        model.Contacts = contacts.ToArray();
        var jointIndexOfSlot = new int[cs.HighWater];
        Array.Fill(jointIndexOfSlot, -1);
        for (int i = 0; i < jointSlots.Count; i++) { jointIndexOfSlot[jointSlots[i]] = i; }
        var ragdolls = new List<PacketRagdoll>();
        var writeSet = filter != null ? filter.RagdollSet(inSet) : inSet;
        e.WriteRagdolls(writeSet, packetIndexOfBody, jointIndexOfSlot, ragdolls);
        model.Ragdolls = ragdolls.ToArray();
        var entities = new List<PacketEntity>();
        e.WriteEntities(inSet, writeSet, packetIndexOfBody, entities);
        model.Entities = entities.ToArray();

        // Contacts between a member and a static shape outside the set (the ground it rests on): the receiving world may hold the same shape, and then the contact keeps its warm start.
        var raw = new List<(int Member, int Outside, PacketSupportContact Contact)>();
        foreach (var pair in pairs)
        {
            int ia = (uint)pair.ShapeA < (uint)packetIndexOfShape.Length ? packetIndexOfShape[pair.ShapeA] : -1;
            int ib = (uint)pair.ShapeB < (uint)packetIndexOfShape.Length ? packetIndexOfShape[pair.ShapeB] : -1;
            if ((ia < 0) == (ib < 0)) { continue; }
            int outside = ia < 0 ? pair.ShapeA : pair.ShapeB;
            if (bodies.TypeOf(shapes.Owner[outside]) != BodyType.Static) { continue; }
            if (filter != null && !filter.KeepContact(shapes.Owner[ia >= 0 ? pair.ShapeA : pair.ShapeB], shapes.Owner[outside])) { continue; }
            bool first = true;
            for (int c = pair.FirstConstraint; c >= 0; c = e.NextInChain(c))
            {
                if (cs.KindOf(c) != ConstraintKind.Contact) { continue; }
                raw.Add((ia >= 0 ? ia : ib, outside, new PacketSupportContact
                {
                    Member = ia >= 0 ? ia : ib, MemberIsA = ia >= 0, SubA = cs.SubShapeA[c], SubB = cs.SubShapeB[c],
                    Flags = (byte)(cs.Flags[c] & (byte)ConstraintBits.ReportPersist), Color = cs.Color[c], Points = cs.PointCount[c], Features = cs.FeatureIdsOf(c).ToArray(),
                    BreakValue = 0f, NewPair = first, Payload = cs.Payload(c).ToArray(),
                }));
                first = false;
            }
        }

        // Member by member, outside shape by outside shape; the sort is stable, so a pair's constraints keep their chain order.
        var order = new int[raw.Count];
        for (int i = 0; i < order.Length; i++) { order[i] = i; }
        Array.Sort(order, (x, y) => raw[x].Member != raw[y].Member ? raw[x].Member.CompareTo(raw[y].Member) : raw[x].Outside != raw[y].Outside ? raw[x].Outside.CompareTo(raw[y].Outside) : x.CompareTo(y));
        var supportOf = new Dictionary<int, int>();
        var supports = new List<PacketSupport>();
        var supportContacts = new List<PacketSupportContact>();
        foreach (int k in order)
        {
            var (member, outside, contact) = raw[k];
            if (!supportOf.TryGetValue(outside, out int si))
            {
                si = supports.Count; supportOf[outside] = si;
                int ob = shapes.Owner[outside];
                var sp = new PacketSupport { Blob = e.WriteShape(outside), Rotation = bodies.RotationOf(ob).ToArray(), Position = new double[lanes] };
                var opos = bodies.PositionOf(ob);
                for (int d = 0; d < lanes; d++) { sp.Position[d] = origin[d] + opos[d]; }
                supports.Add(sp);
            }

            contact.Support = si;
            supportContacts.Add(contact);
        }

        model.Supports = supports.ToArray();
        model.SupportContacts = supportContacts.ToArray();
        var packet = new BodySetPacket(PacketIO.Write(model), model);

        if (options.Move)
        {
            // Highest slot first, so the lowest slots are the last to be freed and the receiving order never depends on it.
            for (int i = members.Length - 1; i >= 0; i--) { e.DestroyBody(core.HandleOfBody(members[i])); }
        }

        return packet;
    }

    private static PacketBody ReadMember(ITransferEngine e, WorldCore core, int b, int lanes, ReadOnlySpan<double> origin)
    {
        var bodies = core.Bodies;
        var pb = new PacketBody { Slot = b, Generation = bodies.Handles.GenerationOf(b), Type = (byte)bodies.TypeOf(b) };
        var flags = bodies.Flags[b];
        pb.Bits = (byte)(((flags & (byte)BodyBits.Bullet) != 0 ? 1 : 0) | ((flags & (byte)BodyBits.NoSleep) != 0 ? 2 : 0));
        pb.Asleep = bodies.TypeOf(b) == BodyType.Dynamic && !bodies.IsAwake(b);
        pb.SleepTime = bodies.SleepTime[b];
        pb.Position = new double[lanes];
        var pos = bodies.PositionOf(b);
        for (int k = 0; k < lanes; k++) { pb.Position[k] = origin[k] + pos[k]; }
        pb.Rotation = bodies.RotationOf(b).ToArray();
        pb.Linear = bodies.LinearVelocityOf(b).ToArray();
        pb.Angular = bodies.AngularVelocityOf(b).ToArray();
        var controls = core.Controls.Get(b);
        if (controls.Present)
        {
            pb.HasControls = true; pb.Controls = controls;
            if ((controls.Overrides & 8) != 0) { pb.Controls.TargetPosition = new RecursiveAI.OneMath.Vector3D(controls.TargetPosition.X + origin[0], controls.TargetPosition.Y + origin[1], lanes == 3 ? controls.TargetPosition.Z + origin[2] : 0.0); }
        }

        var sleep = core.BodySleepOf(b);
        if (sleep.IsOverride) { pb.HasSleep = true; pb.Sleep = sleep; }
        e.ReadBodyExtras(b, pb);
        return pb;
    }

    // ------------------------------------------------------------------ insert

    public static BodySetHandleMap Insert(ITransferEngine e, BodySetPacket packet, TransferTransform transform, string what)
    {
        ArgumentNullException.ThrowIfNull(packet);
        var m = packet.Model;
        var core = e.Core;
        if (m.Dimension != e.Dimension) { throw new BodySetTransferException(what + ": the packet comes from a " + Num(m.Dimension) + "D world and this world is " + Num(e.Dimension) + "D."); }
        if (BitConverter.DoubleToInt64Bits(m.MetresPerUnit) != BitConverter.DoubleToInt64Bits(e.MetresPerUnit))
        {
            throw new BodySetTransferException(what + ": the packet's length unit is " + m.MetresPerUnit.ToString("R", CultureInfo.InvariantCulture) + " m per unit and this world's is " + e.MetresPerUnit.ToString("R", CultureInfo.InvariantCulture)
                + ". Packets are not rescaled; create both worlds with the same LengthUnit.");
        }

        e.CheckInsert(m);
        int lanes = core.Layout.PositionLanes;
        var origin = core.Origin;

        // Everything that can be checked without touching the world, before touching it.
        int n = m.Bodies.Length;
        var rel = new double[n][];
        var rot = new float[n][];
        var lin = new float[n][];
        var ang = new float[n][];
        for (int i = 0; i < n; i++)
        {
            var b = m.Bodies[i];
            var p = new double[lanes];
            transform.Position(b.Position, p);
            for (int k = 0; k < lanes; k++)
            {
                p[k] -= origin[k];
                if (!(Math.Abs(p[k]) <= 2097152.0)) { throw new BodySetTransferException(what + ": body " + Num(i) + " (source slot " + Num(b.Slot) + ") would land at " + p[k].ToString("R", CultureInfo.InvariantCulture) + " m on axis " + Num(k) + ", beyond the +-2^21 m this world accepts."); }
            }

            if (b.HasControls && (b.Controls.Overrides & 8) != 0 && !transform.IsIdentityRotation) { throw new BodySetTransferException(what + ": body " + Num(i) + " has a kinematic target, which cannot be rotated by a portal. Insert without a rotation."); }
            rel[i] = p;
            rot[i] = new float[b.Rotation.Length]; transform.Orientation(b.Rotation, rot[i]);
            lin[i] = new float[b.Linear.Length]; transform.Vector(b.Linear, lin[i]);
            ang[i] = new float[b.Angular.Length]; transform.Angular(b.Angular, ang[i]);
        }

        // Static shapes outside the set that the receiving world holds too, found before anything changes.
        var supportShape = new int[m.Supports.Length];
        var tmpPos = new double[lanes];
        for (int i = 0; i < supportShape.Length; i++)
        {
            var sp = m.Supports[i];
            transform.Position(sp.Position, tmpPos);
            for (int k = 0; k < lanes; k++) { tmpPos[k] -= origin[k]; }
            var sr = new float[sp.Rotation.Length]; transform.Orientation(sp.Rotation, sr);
            supportShape[i] = e.FindSupport(sp.Blob, tmpPos, sr);
        }

        var newBodies = new BodyHandle[n];
        int created = 0;
        var oldBodies = new BodyHandle[n];
        var newShapes = new ShapeHandle[m.Shapes.Length];
        var oldShapes = new ShapeHandle[m.Shapes.Length];
        var newJoints = new JointHandle[m.Joints.Length];
        var oldJoints = new JointHandle[m.Joints.Length];
        RagdollHandle[] ragdollMap = Array.Empty<RagdollHandle>();
        var oldCharacters = new List<CharacterHandle>(); var newCharacters = new List<CharacterHandle>();
        var oldVehicles = new List<VehicleHandle>(); var newVehicles = new List<VehicleHandle>();
        try
        {
            for (int i = 0; i < n; i++)
            {
                newBodies[i] = e.CreateBody(m.Bodies[i], rel[i], rot[i], lin[i], ang[i]);
                created++;
                oldBodies[i] = new BodyHandle(m.SourceWorld, m.Bodies[i].Slot, m.Bodies[i].Generation);
            }

            for (int i = 0; i < m.Shapes.Length; i++)
            {
                newShapes[i] = e.CreateShape(newBodies[m.Shapes[i].Body], m.Shapes[i].Blob);
                oldShapes[i] = new ShapeHandle(m.SourceWorld, m.Shapes[i].Slot, m.Shapes[i].Generation);
            }

            for (int i = 0; i < m.MemberCount; i++) { e.FinishBody(newBodies[i], m.Bodies[i], rel[i], transform); }
            for (int i = m.MemberCount; i < n; i++) { e.FinishBody(newBodies[i], m.Bodies[i], rel[i], transform); }

            for (int i = 0; i < m.Joints.Length; i++)
            {
                var j = m.Joints[i];
                core.ColorHint = j.Color;
                try { newJoints[i] = e.CreateJoint(newBodies[j.BodyA], newBodies[j.BodyB], j); }
                finally { core.ColorHint = -1; }
                oldJoints[i] = new JointHandle(m.SourceWorld, j.Slot, j.Generation);
            }

            var newRagdolls = new RagdollHandle[m.Ragdolls.Length];
            for (int i = 0; i < m.Ragdolls.Length; i++) { newRagdolls[i] = e.CreateRagdoll(m.Ragdolls[i], newBodies, newJoints); }
            ragdollMap = newRagdolls;
            foreach (var en in m.Entities)
            {
                var (slot, generation) = e.CreateEntity(en, m, newBodies, transform);
                if (en.Kind == PacketEntity.KindCharacter) { oldCharacters.Add(new CharacterHandle(m.SourceWorld, en.Slot, en.Generation)); newCharacters.Add(new CharacterHandle(core.Id, slot, generation)); }
                else { oldVehicles.Add(new VehicleHandle(m.SourceWorld, en.Slot, en.Generation)); newVehicles.Add(new VehicleHandle(core.Id, slot, generation)); }
            }

            AddContacts(e, core, m, newShapes, newBodies, transform);
            AddSupportContacts(e, core, m, newShapes, newBodies, supportShape, transform);
            SleepComponents(core, m, newBodies);
        }
        catch
        {
            for (int i = created - 1; i >= 0; i--)
            {
                if (core.Bodies.Handles.IsAlive(newBodies[i].Index, newBodies[i].Generation)) { e.DestroyBody(newBodies[i]); }
            }

            throw;
        }

        var oldRagdolls = new RagdollHandle[m.Ragdolls.Length];
        for (int i = 0; i < oldRagdolls.Length; i++) { oldRagdolls[i] = new RagdollHandle(m.SourceWorld, m.Ragdolls[i].Slot, m.Ragdolls[i].Generation); }
        return new BodySetHandleMap(oldBodies, newBodies, m.MemberCount, oldShapes, newShapes, oldJoints, newJoints, oldRagdolls, ragdollMap,
            oldCharacters.ToArray(), newCharacters.ToArray(), oldVehicles.ToArray(), newVehicles.ToArray());
    }

    private static void AddContacts(ITransferEngine e, WorldCore core, PacketModel m, ShapeHandle[] shapes, BodyHandle[] bodies, TransferTransform transform)
    {
        var cs = core.Constraints;
        int pair = -1, previous = -1;
        bool allowed = false;
        var payload = new float[m.PayloadFloats];
        foreach (var pc in m.Contacts)
        {
            if (pc.NewPair)
            {
                allowed = e.BeginPair(shapes[pc.ShapeA], shapes[pc.ShapeB], out pair);
                previous = -1;
            }

            if (!allowed) { continue; }
            var sa = m.Shapes[pc.ShapeA]; var sb = m.Shapes[pc.ShapeB];
            var desc = new ConstraintDesc
            {
                Kind = ConstraintKind.Contact, BodyA = bodies[sa.Body], BodyB = bodies[sb.Body], ShapeA = shapes[pc.ShapeA], ShapeB = shapes[pc.ShapeB],
                SubShapeA = pc.SubA, SubShapeB = pc.SubB, Flags = ConstraintBits.ReportPersist,
            };
            core.ColorHint = pc.Color;
            int c = core.AddConstraint(desc);
            cs.Flags[c] = pc.Flags;
            cs.PointCount[c] = pc.Points;
            pc.Features.CopyTo(cs.FeatureIdsOf(c));
            pc.Payload.CopyTo(payload, 0);
            if (!transform.IsIdentityRotation) { e.TransformContactPayload(payload, transform); }
            payload.CopyTo(cs.Payload(c));
            e.LinkContact(pair, previous, c);
            previous = c;
        }
    }

    private static void AddSupportContacts(ITransferEngine e, WorldCore core, PacketModel m, ShapeHandle[] shapes, BodyHandle[] bodies, int[] supportShape, TransferTransform transform)
    {
        var cs = core.Constraints;
        int pair = -1, previous = -1;
        bool allowed = false;
        var payload = new float[m.PayloadFloats];
        foreach (var pc in m.SupportContacts)
        {
            int outsideSlot = supportShape[pc.Support];
            if (pc.NewPair)
            {
                allowed = false; previous = -1;
                if (outsideSlot >= 0)
                {
                    var outsideHandle = core.HandleOfShape(outsideSlot);
                    var memberHandle = shapes[pc.Member];
                    allowed = memberHandle.Index < outsideSlot ? e.BeginPair(memberHandle, outsideHandle, out pair) : e.BeginPair(outsideHandle, memberHandle, out pair);
                }
            }

            if (!allowed) { continue; }
            // The pair table orders its shapes by slot; the contact has to be written in that order, which may not be the source's.
            int memberSlot = shapes[pc.Member].Index;
            bool memberIsA = memberSlot < outsideSlot;
            var memberShape = shapes[pc.Member];
            var outsideShape = core.HandleOfShape(outsideSlot);
            var sh = m.Shapes[pc.Member];
            var desc = new ConstraintDesc
            {
                Kind = ConstraintKind.Contact, BodyA = memberIsA ? bodies[sh.Body] : core.HandleOfBody(core.Shapes.Owner[outsideSlot]), BodyB = memberIsA ? core.HandleOfBody(core.Shapes.Owner[outsideSlot]) : bodies[sh.Body],
                ShapeA = memberIsA ? memberShape : outsideShape, ShapeB = memberIsA ? outsideShape : memberShape,
                SubShapeA = pc.MemberIsA == memberIsA ? pc.SubA : pc.SubB, SubShapeB = pc.MemberIsA == memberIsA ? pc.SubB : pc.SubA, Flags = ConstraintBits.ReportPersist,
            };
            core.ColorHint = pc.Color;
            int c = core.AddConstraint(desc);
            cs.Flags[c] = pc.Flags;
            cs.PointCount[c] = pc.Points;
            var features = cs.FeatureIdsOf(c);
            pc.Features.CopyTo(features);
            pc.Payload.CopyTo(payload, 0);
            if (pc.MemberIsA != memberIsA)
            {
                for (int k = 0; k < features.Length; k++) { features[k] = (features[k] << 16) | (features[k] >> 16); }
                e.SwapContactPayload(payload);
            }

            if (!transform.IsIdentityRotation) { e.TransformContactPayload(payload, transform); }
            payload.CopyTo(cs.Payload(c));
            e.LinkContact(pair, previous, c);
            previous = c;
        }
    }

    /// <summary>Puts to sleep every connected group of dynamic bodies that was asleep as a whole in the source (connected by the packet's joints and contacts, which is how the receiving world will connect them).</summary>
    private static void SleepComponents(WorldCore core, PacketModel m, BodyHandle[] bodies)
    {
        int n = m.MemberCount;
        var parent = new int[n];
        for (int i = 0; i < n; i++) { parent[i] = i; }
        int Find(int x) { while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; } return x; }
        void Union(int a, int b)
        {
            if (a >= n || b >= n || m.Bodies[a].Type != (byte)BodyType.Dynamic || m.Bodies[b].Type != (byte)BodyType.Dynamic) { return; }
            int ra = Find(a), rb = Find(b);
            if (ra != rb) { if (ra < rb) { parent[rb] = ra; } else { parent[ra] = rb; } }
        }

        foreach (var j in m.Joints) { Union(j.BodyA, j.BodyB); }
        foreach (var c in m.Contacts) { Union(m.Shapes[c.ShapeA].Body, m.Shapes[c.ShapeB].Body); }
        var anyAwake = new bool[n];
        for (int i = 0; i < n; i++) { if (m.Bodies[i].Type == (byte)BodyType.Dynamic && !m.Bodies[i].Asleep) { anyAwake[Find(i)] = true; } }
        for (int i = 0; i < n; i++)
        {
            if (m.Bodies[i].Type != (byte)BodyType.Dynamic || !m.Bodies[i].Asleep || anyAwake[Find(i)] || Find(i) != i) { continue; }
            core.Sleep(bodies[i]);
        }
    }
}
