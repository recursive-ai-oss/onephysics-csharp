using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

// Body-set transfer (docs/spec/transfer.md): what the 2D engine writes into a packet and how it builds the same bodies, shapes, joints and contacts in another world.
internal sealed partial class Engine2D : ITransferEngine
{
    WorldCore ITransferEngine.Core => Core;
    int ITransferEngine.Dimension => 2;
    double ITransferEngine.MetresPerUnit => Opt.LengthUnit.MetresPerUnit;

    private static BodySetTransferException Refuse(string what) => new(what);

    // ------------------------------------------------------------------ extraction

    void ITransferEngine.CheckMembers(int[] members, bool[] inSet)
    {
        var s = new RegionSelection(Core.Bodies.HighWater);
        foreach (int b in members) { s.AddBody(b); }
        for (int i = 0; i < Core.Shapes.HighWater; i++) { if (Core.Shapes.Handles.IsLive(i) && inSet[Core.Shapes.Owner[i]]) { s.Add("shape", i); } }
        Rods.CloseRegion(s);
        Cables.CloseRegion(s);
        _articulations?.CloseRegion(s);
        _charExt?.CloseRegion(s);
        _actuators?.SelectRegion(s);
        _destruction?.SelectRegion(s);
        CheckRagdolls(inSet);

        for (int i = 0; i < VehHandles.HighWater; i++) { if (VehHandles.IsLive(i) && inSet[Vehicles[i]!.Body]) { s.Add("vehicle", i); } }
        for (int i = 0; i < BikeHandles.HighWater; i++) { if (BikeHandles.IsLive(i) && inSet[Bikes[i]!.Frame]) { s.Add("bicycle", i); } }
        for (int i = 0; i < CharHandles.HighWater; i++) { if (CharHandles.IsLive(i) && inSet[CharBody[i]]) { s.Add("character", i); } }
        for (int i = 0; i < RailHandles.HighWater; i++) { if (RailHandles.IsLive(i) && inSet[Rails[i]!.Car]) { s.Add("rail vehicle", i); } }
        for (int i = 0; i < TrackHandles.HighWater; i++) { if (TrackHandles.IsLive(i) && Tracks[i]!.Body >= 0 && inSet[Tracks[i]!.Body]) { s.Add("rail track", i); } }
        foreach (var kv in s.Entities)
        {
            if (kv.Key == "body" || kv.Key == "shape" || kv.Value.Count == 0) { continue; }
            int first = -1;
            foreach (int x in kv.Value) { first = x; break; }
            throw Refuse("Body-set transfer: a body of the set belongs to a " + kv.Key + " (#" + first.ToString(CultureInfo.InvariantCulture) + "), which a packet does not carry. Destroy it or leave its bodies out of the set.");
        }

        foreach (int b in members)
        {
            if (Core.BodyTimeScaleOf(b) != 1f) { throw Refuse("Body-set transfer: body " + Core.HandleOfBody(b) + " has a local time scale, which a packet does not carry."); }
            foreach (var tile in Core.ScaleStore.Tiles) { if (Array.IndexOf(tile.Bodies, b) >= 0) { throw Refuse("Body-set transfer: body " + Core.HandleOfBody(b) + " belongs to a streamed tile, which a packet does not carry."); } }
        }

        for (int sh = 0; sh < Core.Shapes.HighWater; sh++)
        {
            if (!Core.Shapes.Handles.IsLive(sh) || !inSet[Core.Shapes.Owner[sh]]) { continue; }
            if (Core.ContactMaterials.MaterialCount(sh) > 0) { throw Refuse("Body-set transfer: shape " + Core.HandleOfShape(sh) + " has per-segment contact materials, which a packet does not carry."); }
        }
    }

    private void CheckRagdolls(bool[] inSet)
    {
        for (int r = 0; r < RagHandles.HighWater; r++)
        {
            if (!RagHandles.IsLive(r)) { continue; }
            bool any = false;
            foreach (int b in Ragdolls[r]!.Body) { if (inSet[b]) { any = true; } }
            if (any && ActiveOf(r) != null) { throw Refuse("Body-set transfer: the ragdoll #" + r.ToString(CultureInfo.InvariantCulture) + " is an active ragdoll (balance and animation blending), which a packet does not carry. Call DisableActive() first or leave it out."); }
        }
    }

    byte[] ITransferEngine.WriteShape(int s)
    {
        var w = StateWriter.ToBuffer();
        WriteShape(w, in Geom[s]);
        w.WriteSingle(Friction[s]); w.WriteSingle(Restitution[s]); w.WriteSingle(Rolling[s]); w.WriteSingle(Density[s]);
        w.WriteUInt32(Category[s]); w.WriteUInt32(Mask[s]); w.WriteInt32(Group[s]);
        w.WriteByte(ShapeFlags[s]); w.WriteByte(ShapeLayer[s]);
        var surface = Core.Surfaces.Get(s);
        bool has = !surface.IsDefault;
        w.WriteBool(has);
        if (has)
        {
            w.WriteSingle(surface.Velocity.X); w.WriteSingle(surface.Velocity.Y); w.WriteSingle(surface.Velocity.Z);
            w.WriteSingle(surface.OneWayDirection.X); w.WriteSingle(surface.OneWayDirection.Y); w.WriteSingle(surface.OneWayDirection.Z);
            w.WriteSingle(surface.FrictionAxis.X); w.WriteSingle(surface.FrictionAxis.Y); w.WriteSingle(surface.FrictionAxis.Z);
            w.WriteSingle(surface.LongitudinalFriction); w.WriteSingle(surface.TransverseFriction); w.WriteSingle(surface.SpinningFriction);
        }

        return w.ToArray();
    }

    private readonly struct ShapeBlob2D
    {
        public readonly Shape2D Geometry;
        public readonly float Friction, Restitution, Rolling, Density;
        public readonly uint Category, Mask;
        public readonly int Group;
        public readonly byte Flags, Layer;
        public readonly SurfaceProperties Surface;

        public ShapeBlob2D(byte[] blob)
        {
            var r = new StateReader(blob);
            try
            {
                Geometry = ReadShape(r);
                Friction = r.ReadSingle(); Restitution = r.ReadSingle(); Rolling = r.ReadSingle(); Density = r.ReadSingle();
                if (!(Friction >= 0f && Friction <= MaterialLimits.MaxCoefficient && Rolling >= 0f && Rolling <= MaterialLimits.MaxCoefficient && Restitution >= 0f && Restitution <= 1f && Density >= 0f && float.IsFinite(Density)))
                { throw r.Error("the shape material is out of range"); }
                Category = r.ReadUInt32(); Mask = r.ReadUInt32(); Group = r.ReadInt32();
                Flags = r.ReadByte(); Layer = r.ReadByte();
                if ((Flags & ~FlagSensor) != 0) { throw r.Error("unknown shape flags"); }
                if (Layer >= LayerCount) { throw r.Error("the layer must be below " + LayerCount.ToString(CultureInfo.InvariantCulture)); }
                Surface = default;
                if (r.ReadBool())
                {
                    Surface = new SurfaceProperties
                    {
                        Velocity = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle()), OneWayDirection = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle()),
                        FrictionAxis = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle()), LongitudinalFriction = r.ReadSingle(), TransverseFriction = r.ReadSingle(), SpinningFriction = r.ReadSingle(),
                    };
                    try { Surface.Validate(2); } catch (ArgumentException e) { throw r.Error(e.Message); }
                    if (Surface.IsDefault) { throw r.Error("a default surface must be absent"); }
                }

                if (r.Remaining != 0) { throw r.Error("trailing bytes"); }
            }
            catch (SnapshotFormatException e) { throw new BodySetPacketException("shape", e.Offset, e.Message); }
        }
    }

    internal static void ValidateShapeBlob(byte[] blob) => _ = new ShapeBlob2D(blob);

    byte[] ITransferEngine.WriteJoint(int c)
    {
        ref JointData2D j = ref Joints[c];
        if (j.Type == 0) { throw Refuse("Body-set transfer: joint " + Core.HandleOfJoint(c) + " has no record."); }
        var type = (JointType2D)j.Type;
        if (!Carried(type)) { throw Refuse("Body-set transfer: joint " + Core.HandleOfJoint(c) + " is a " + JointData2D.Name(type) + " joint, which is not carried by packets (its state is world-anchored or lives outside the joint record)."); }
        var w = StateWriter.ToBuffer();
        j.Write(w);
        w.WriteInt32(JointLifetime.Get(Core.Constraints, c));
        return w.ToArray();
    }

    private static bool Carried(JointType2D t) => t != JointType2D.Mouse && t != JointType2D.Pulley && t != JointType2D.Path && t != JointType2D.Configurable;

    private static JointData2D ReadJointBlob(byte[] blob)
    {
        var r = new StateReader(blob);
        try
        {
            var d = JointData2D.Read(r);
            int life = r.ReadInt32();
            if (life < 0) { throw r.Error("the remaining lifetime is negative"); }
            if (r.Remaining != 0) { throw r.Error("trailing bytes"); }
            if (!Carried((JointType2D)d.Type)) { throw r.Error("mouse, pulley, path and configurable joints are not carried by packets"); }
            d.LifetimeSteps = life;
            return d;
        }
        catch (SnapshotFormatException e) { throw new BodySetPacketException("joint", e.Offset, e.Message); }
    }

    internal static void ValidateJointBlob(byte[] blob) => ReadJointBlob(blob);

    void ITransferEngine.ReadBodyExtras(int b, PacketBody into)
    {
        into.EngineFlags = (byte)(BodyFlags[b] & 1);
        var x = new float[8];
        x[0] = LinearDamping[b]; x[1] = AngularDamping[b]; x[2] = GravityScale[b];
        for (int k = 0; k < 2; k++) { x[3 + k] = Dp[2 * b + k]; x[5 + k] = Force[2 * b + k]; }
        x[7] = Torque[b];
        into.Extra = x;
    }

    void ITransferEngine.ReadAnchor(int b, PacketBody into)
    {
        var bodies = Core.Bodies;
        var q = bodies.RotationOf(b);
        float lx = LocalCenter[2 * b], ly = LocalCenter[2 * b + 1];
        float ax = q[0] * lx - q[1] * ly, ay = q[1] * lx + q[0] * ly;
        var pos = bodies.PositionOf(b);
        into.Position = new[] { pos[0] - (double)ax, pos[1] - (double)ay };
        into.Rotation = q.ToArray();
        var v = bodies.LinearVelocityOf(b); float w = bodies.AngularVelocityOf(b)[0];
        // Velocity of the origin: v - w x arm, with w x arm = (-w ay, w ax).
        into.Linear = new[] { v[0] + w * ay, v[1] - w * ax };
        into.Angular = new[] { w };
        into.Extra = new float[8];
        into.Extra[2] = 1f;
    }

    void ITransferEngine.CollectPairs(List<TransferPair> pairs)
    {
        for (int p = 0; p < PairHigh; p++) { if (Pairs[p].Live && Pairs[p].Cons >= 0) { pairs.Add(new TransferPair(Pairs[p].A, Pairs[p].B, Pairs[p].Cons)); } }
    }

    int ITransferEngine.NextInChain(int c) => -1;

    void ITransferEngine.DestroyBody(BodyHandle h) => DestroyBody(h);

    // ------------------------------------------------------------------ insertion

    void ITransferEngine.CheckInsert(PacketModel m) { }

    void ITransferEngine.RefreshBodyProxies(int b)
    {
        for (int s = ShapeHead[b]; s >= 0; s = NextShape[s]) { Bp.MoveProxy(Proxy[s], TightBox(b, in Geom[s]), 0.0, 0.0); }
    }

    BodyHandle ITransferEngine.CreateBody(PacketBody pb, ReadOnlySpan<double> rel, ReadOnlySpan<float> rot, ReadOnlySpan<float> lin, ReadOnlySpan<float> ang)
    {
        var def = new BodyDef
        {
            Type = (BodyType)pb.Type, Position = new Vector2D(rel[0], rel[1]), LinearVelocity = new Vector2F(lin[0], lin[1]), AngularVelocity = ang[0],
            LinearDamping = pb.Extra[0], AngularDamping = pb.Extra[1], GravityScale = pb.Extra[2],
            FixedRotation = (pb.EngineFlags & 1) != 0, IsBullet = (pb.Bits & 1) != 0, AllowSleep = (pb.Bits & 2) == 0,
        };
        var h = CreateBody(in def);
        // The definition path takes an angle; the packet's (cosine, sine) bits are what the source world had.
        rot.CopyTo(Core.Bodies.RotationOf(h.Index));
        return h;
    }

    ShapeHandle ITransferEngine.CreateShape(BodyHandle body, byte[] blob)
    {
        var b = new ShapeBlob2D(blob);
        var def = new ShapeDef
        {
            Surface = b.Surface, Density = b.Density, Friction = b.Friction, Restitution = b.Restitution, RollingResistance = b.Rolling, CategoryBits = b.Category, MaskBits = b.Mask,
            GroupIndex = b.Group, IsSensor = (b.Flags & FlagSensor) != 0, Layer = b.Layer,
        };
        return AddShapeResolved(body, in b.Geometry, in def);
    }

    void ITransferEngine.FinishBody(BodyHandle h, PacketBody pb, ReadOnlySpan<double> rel, TransferTransform transform)
    {
        int b = h.Index;
        var origin = Core.Origin;
        if (pb.HasControls)
        {
            var state = pb.Controls;
            if ((state.Overrides & 8) != 0)
            {
                var t = new double[2] { state.TargetPosition.X, state.TargetPosition.Y };
                transform.Position(t, t);
                state.TargetPosition = new Vector3D(t[0] - origin[0], t[1] - origin[1], 0.0);
            }

            Core.Controls.Set(b, state);
            if (pb.Type == (byte)BodyType.Dynamic) { UpdateMass(b, true); }
        }

        if (pb.HasSleep) { Core.SetBodySleepRaw(b, pb.Sleep); }
        Core.Bodies.SleepTime[b] = pb.SleepTime;
        var v = new float[2];
        for (int k = 0; k < 2; k++) { v[k] = pb.Extra[3 + k]; }
        transform.Vector(v, v); for (int k = 0; k < 2; k++) { Dp[2 * b + k] = v[k]; }
        for (int k = 0; k < 2; k++) { v[k] = pb.Extra[5 + k]; }
        transform.Vector(v, v); for (int k = 0; k < 2; k++) { Force[2 * b + k] = v[k]; }
        Torque[b] = pb.Extra[7];
        var pos = Core.Bodies.PositionOf(b);
        for (int k = 0; k < 2; k++) { pos[k] = rel[k]; }
        for (int s = ShapeHead[b]; s >= 0; s = NextShape[s]) { Bp.MoveProxy(Proxy[s], TightBox(b, in Geom[s]), 0.0, 0.0); }
    }

    JointHandle ITransferEngine.CreateJoint(BodyHandle a, BodyHandle b, PacketJoint pj)
    {
        var d = ReadJointBlob(pj.Blob);
        var h = CreateJoint(a, b, d);
        var cs = Core.Constraints;
        pj.Payload.CopyTo(cs.Payload(h.Index));
        cs.Flags[h.Index] |= (byte)(pj.Flags & (byte)ConstraintBits.ReportPersist);
        return h;
    }

    bool ITransferEngine.BeginPair(ShapeHandle a, ShapeHandle b, out int pair)
    {
        int sa = Core.Resolve(a), sb = Core.Resolve(b);
        pair = -1;
        if (sa >= sb || PairMap.ContainsKey(PairKey(sa, sb)) || !PairAllowed(sa, sb)) { return false; }
        pair = AllocatePair(sa, sb);
        if (_fresh.Length < PairHigh) { Array.Resize(ref _fresh, Math.Max(PairHigh, _fresh.Length * 2)); }
        _fresh[pair] = false;
        return true;
    }

    int ITransferEngine.FindSupport(byte[] blob, ReadOnlySpan<double> rel, ReadOnlySpan<float> rot)
    {
        var want = new ShapeBlob2D(blob);
        var wanted = StateWriter.ToBuffer(); WriteShape(wanted, in want.Geometry);
        var bodies = Core.Bodies;
        for (int s = 0; s < Core.Shapes.HighWater; s++)
        {
            if (!Core.Shapes.Handles.IsLive(s)) { continue; }
            int b = Core.Shapes.Owner[s];
            if (bodies.TypeOf(b) != BodyType.Static) { continue; }
            var p = bodies.PositionOf(b); var q = bodies.RotationOf(b);
            if (BitConverter.DoubleToInt64Bits(p[0]) != BitConverter.DoubleToInt64Bits(rel[0]) || BitConverter.DoubleToInt64Bits(p[1]) != BitConverter.DoubleToInt64Bits(rel[1])) { continue; }
            if (q[0] != rot[0] || q[1] != rot[1]) { continue; }
            var have = StateWriter.ToBuffer(); WriteShape(have, in Geom[s]);
            if (!have.WrittenSpan.SequenceEqual(wanted.WrittenSpan)) { continue; }
            return s;
        }

        return -1;
    }

    void ITransferEngine.SwapContactPayload(Span<float> payload)
    {
        for (int k = 0; k < 3; k++) { payload[ManifoldLayout.Normal + k] = -payload[ManifoldLayout.Normal + k]; payload[ManifoldLayout.RollingImpulse + k] = 0f; }
        for (int i = 0; i < 2; i++)
        {
            for (int k = 0; k < 3; k++) { (payload[ManifoldLayout.AnchorA(i) + k], payload[ManifoldLayout.AnchorB(i) + k]) = (payload[ManifoldLayout.AnchorB(i) + k], payload[ManifoldLayout.AnchorA(i) + k]); }
            payload[ManifoldLayout.Tangent1Impulse(i)] = 0f; payload[ManifoldLayout.Tangent2Impulse(i)] = 0f;
        }

        // What follows the two points is the pair's manifold-reuse data, which belongs to the old orientation.
        payload.Slice(ManifoldLayout.Header + 2 * ManifoldLayout.PointStride).Clear();
    }

    void ITransferEngine.LinkContact(int pair, int previous, int c)
    {
        // A 2D pair holds one constraint.
        Pairs[pair].Cons = c;
    }

    void ITransferEngine.TransformContactPayload(Span<float> payload, TransferTransform transform)
    {
        // The tangent is the normal turned a quarter, so tangent impulses and the (scalar) rolling impulse stay valid; the normal and the arms turn.
        var v = new float[2];
        Turn(payload, ManifoldLayout.Normal, v, transform);
        for (int i = 0; i < 2; i++) { Turn(payload, ManifoldLayout.AnchorA(i), v, transform); Turn(payload, ManifoldLayout.AnchorB(i), v, transform); }
    }

    private static void Turn(Span<float> payload, int at, float[] v, TransferTransform transform)
    {
        v[0] = payload[at]; v[1] = payload[at + 1];
        transform.Vector(v, v);
        payload[at] = v[0]; payload[at + 1] = v[1];
    }

    // ------------------------------------------------------------------ ragdolls

    void ITransferEngine.ExpandSet(bool[] inSet)
    {
        bool changed;
        do
        {
            changed = false;
            for (int r = 0; r < RagHandles.HighWater; r++)
            {
                if (!RagHandles.IsLive(r)) { continue; }
                var s = Ragdolls[r]!;
                bool any = false;
                foreach (int b in s.Body) { if ((uint)b < (uint)inSet.Length && inSet[b]) { any = true; } }
                if (!any) { continue; }
                foreach (int b in s.Body) { if ((uint)b < (uint)inSet.Length && !inSet[b]) { inSet[b] = true; changed = true; } }
            }
        }
        while (changed);
    }

    void ITransferEngine.WriteRagdolls(bool[] inSet, int[] bodyIndex, int[] jointIndex, List<PacketRagdoll> into)
    {
        for (int r = 0; r < RagHandles.HighWater; r++)
        {
            if (!RagHandles.IsLive(r)) { continue; }
            var s = Ragdolls[r]!;
            if (!inSet[s.Body[0]]) { continue; }
            var c = s.CloneForFork();
            for (int i = 0; i < s.Parts; i++)
            {
                c.Body[i] = bodyIndex[s.Body[i]]; c.BodyGen[i] = 1;
                if (s.Def.Parts[i].Parent < 0) { c.Joint[i] = -1; c.JointGen[i] = 0; continue; }
                int j = RagdollJointAlive(s, i) ? jointIndex[s.Joint[i]] : -1;
                if (j < 0) { throw Refuse("Body-set transfer: the ragdoll #" + r.ToString(CultureInfo.InvariantCulture) + " has lost the joint of part " + i.ToString(CultureInfo.InvariantCulture) + " (it broke or was destroyed), so it cannot be rebuilt from a packet."); }
                c.Joint[i] = j; c.JointGen[i] = 1;
            }

            var w = StateWriter.ToBuffer();
            WriteRagdollRecord(w, c);
            into.Add(new PacketRagdoll { Slot = r, Generation = RagHandles.GenerationOf(r), Blob = w.ToArray() });
        }
    }

    internal static void ValidateRagdollBlob(byte[] blob, PacketModel m, bool[] used)
    {
        var r = new StateReader(blob);
        try
        {
            var s = ReadRagdollRecord(r, m.MemberCount, used, new LengthUnit(m.MetresPerUnit).Scale);
            if (r.Remaining != 0) { throw r.Error("trailing bytes"); }
            for (int i = 0; i < s.Parts; i++)
            {
                if (m.Bodies[s.Body[i]].Type != (byte)BodyType.Dynamic) { throw r.Error("part " + i.ToString(CultureInfo.InvariantCulture) + " is not a dynamic body"); }
                if (s.Joint[i] < 0) { continue; }
                if (s.Joint[i] >= m.Joints.Length) { throw r.Error("part " + i.ToString(CultureInfo.InvariantCulture) + " names the dangling joint " + s.Joint[i].ToString(CultureInfo.InvariantCulture)); }
                var jb = m.Joints[s.Joint[i]];
                if (jb.Blob.Length == 0 || jb.Blob[0] != (byte)JointType2D.Revolute) { throw r.Error("part " + i.ToString(CultureInfo.InvariantCulture) + " is not joined by a revolute joint"); }
                int parent = s.Def.Parts[i].Parent;
                if (jb.BodyA != s.Body[parent] || jb.BodyB != s.Body[i]) { throw r.Error("the joint of part " + i.ToString(CultureInfo.InvariantCulture) + " does not join it to its parent"); }
            }
        }
        catch (SnapshotFormatException e) { throw new BodySetPacketException("ragdoll", e.Offset, e.Message); }
    }

    RagdollHandle ITransferEngine.CreateRagdoll(PacketRagdoll record, BodyHandle[] bodies, JointHandle[] joints)
    {
        var r = new StateReader(record.Blob);
        var s = ReadRagdollRecord(r, bodies.Length, new bool[bodies.Length], Opt.UnitsPerMetre);
        for (int i = 0; i < s.Parts; i++)
        {
            var b = bodies[s.Body[i]];
            s.Body[i] = b.Index; s.BodyGen[i] = b.Generation;
            if (s.Joint[i] >= 0) { var j = joints[s.Joint[i]]; s.Joint[i] = j.Index; s.JointGen[i] = j.Generation; }
        }

        int slot = ReserveRagdoll();
        RegisterRagdoll(slot, s);
        if (s.Def.SelfCollision == RagdollSelfCollision.None)
        {
            // The parts share the negative group -(slot + 1) so that they do not collide with each other; the slot is this world's.
            int oldGroup = -(record.Slot + 1), newGroup = -(slot + 1);
            for (int i = 0; i < s.Parts; i++)
            {
                for (int sh = ShapeHead[s.Body[i]]; sh >= 0; sh = NextShape[sh]) { if (Group[sh] == oldGroup) { Group[sh] = newGroup; } }
            }
        }

        return new RagdollHandle(Core.Id, slot, RagHandles.GenerationOf(slot));
    }
}
