using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

// Body-set transfer (docs/spec/transfer.md): what the 3D engine writes into a packet and how it builds the same bodies, shapes, joints and contacts in another world.
internal sealed partial class Engine3D : ITransferEngine
{
    WorldCore ITransferEngine.Core => Core;
    int ITransferEngine.Dimension => 3;
    double ITransferEngine.MetresPerUnit => Opt.LengthUnit.MetresPerUnit;

    private static BodySetTransferException Refuse(string what) => new(what);

    // ------------------------------------------------------------------ extraction

    void ITransferEngine.CheckMembers(int[] members, bool[] inSet) => CheckMembers(members, inSet, inSet);

    /// <summary>
    /// Refuses a set that takes part in something a packet does not carry. <paramref name="selected"/> marks the bodies whose own features are checked (<paramref name="members"/>, as a mask) and
    /// <paramref name="travelling"/> the bodies that travel together, which a character's hold has to stay inside.
    /// </summary>
    private void CheckMembers(int[] members, bool[] selected, bool[] travelling)
    {
        var s = new RegionSelection(Core.Bodies.HighWater);
        foreach (int b in members) { s.AddBody(b); }
        foreach (int b in members) { for (int sh = ShapeHead[b]; sh >= 0; sh = NextShape[sh]) { s.Add("shape", sh); } }
        Rods.CloseRegion(s);
        Cables.CloseRegion(s);
        _articulations?.CloseRegion(s);
        _actuators?.SelectRegion(s);
        _destruction?.SelectRegion(s);
        CheckRagdolls(selected);

        for (int i = 0; i < BikeHandles.HighWater; i++) { if (BikeHandles.IsLive(i) && (selected[Bikes[i]!.Frame] || selected[Bikes[i]!.Fork])) { s.Add("bicycle", i); } }
        for (int i = 0; i < RailHandles.HighWater; i++)
        {
            if (!RailHandles.IsLive(i)) { continue; }
            var r = Rails[i]!;
            if (r.Car >= 0 && selected[r.Car]) { s.Add("rail vehicle", i); }
            else { foreach (int w in r.Ws) { if (w >= 0 && w < selected.Length && selected[w]) { s.Add("rail vehicle", i); } } }
        }

        for (int i = 0; i < TrackHandles.HighWater; i++) { if (TrackHandles.IsLive(i) && ((Tracks[i]!.Ballast >= 0 && selected[Tracks[i]!.Ballast]) || (Tracks[i]!.Rails >= 0 && selected[Tracks[i]!.Rails]))) { s.Add("rail track", i); } }
        for (int i = 0; i < LinkHandles.HighWater; i++)
        {
            if (!LinkHandles.IsLive(i)) { continue; }
            ref ClothLink l = ref Links[i];
            if (l.Kind == (int)ClothLinkKind.Attachment && (uint)l.Rigid < (uint)selected.Length && selected[l.Rigid]) { s.Add("cloth attachment", i); }
        }

        foreach (var kv in s.Entities)
        {
            if (kv.Key == "body" || kv.Key == "shape" || kv.Value.Count == 0) { continue; }
            int first = -1;
            foreach (int x in kv.Value) { first = x; break; }
            throw Refuse("Body-set transfer: a body of the set belongs to a " + kv.Key + " (#" + first.ToString(CultureInfo.InvariantCulture) + "), which a packet does not carry. Destroy it or leave its bodies out of the set.");
        }

        foreach (int b in members)
        {
            int ch = CharacterOfBody(b);
            if (ch >= 0) { var why = CharacterRefusal(ch, travelling); if (why != null) { throw Refuse(why); } }
            if (Core.BodyTimeScaleOf(b) != 1f) { throw Refuse("Body-set transfer: body " + Core.HandleOfBody(b) + " has a local time scale, which a packet does not carry."); }
            foreach (var tile in Core.ScaleStore.Tiles) { if (Array.IndexOf(tile.Bodies, b) >= 0) { throw Refuse("Body-set transfer: body " + Core.HandleOfBody(b) + " belongs to a streamed tile, which a packet does not carry."); } }
            for (int sh = ShapeHead[b]; sh >= 0; sh = NextShape[sh])
            {
                if (Core.ContactMaterials.MaterialCount(sh) > 0) { throw Refuse("Body-set transfer: shape " + Core.HandleOfShape(sh) + " has per-triangle contact materials, which a packet does not carry."); }
                var g = Geoms[GeomOf[sh]]!;
                if (g.Recipe.Length > 0 && (g.Recipe[0] == ShapeSpec3D.KVoxelGrid))
                { throw Refuse("Body-set transfer: shape " + Core.HandleOfShape(sh) + " is a voxel grid, which is edited in place and has no immutable recipe to carry."); }
            }
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
        var recipe = Geoms[GeomOf[s]]!.Recipe;
        w.WriteInt32(recipe.Length); w.WriteBytes(recipe);
        var l = Local[s];
        w.WriteSingle(l.Rotation.X); w.WriteSingle(l.Rotation.Y); w.WriteSingle(l.Rotation.Z); w.WriteSingle(l.Rotation.W);
        w.WriteSingle(l.Translation.X); w.WriteSingle(l.Translation.Y); w.WriteSingle(l.Translation.Z);
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

    private readonly struct ShapeBlob3D
    {
        public readonly byte[] Recipe;
        public readonly RigidTransformF Local;
        public readonly float Friction, Restitution, Rolling, Density;
        public readonly uint Category, Mask;
        public readonly int Group;
        public readonly byte Flags, Layer;
        public readonly bool HasSurface;
        public readonly SurfaceProperties Surface;

        public ShapeBlob3D(byte[] blob)
        {
            var r = new StateReader(blob);
            try
            {
                int n = r.ReadCount(1, "shape recipe");
                Recipe = new byte[n]; r.ReadBytes(Recipe);
                float qx = r.ReadSingle(), qy = r.ReadSingle(), qz = r.ReadSingle(), qw = r.ReadSingle(), tx = r.ReadSingle(), ty = r.ReadSingle(), tz = r.ReadSingle();
                if (!float.IsFinite(qx + qy + qz + qw + tx + ty + tz)) { throw r.Error("the local pose is not finite"); }
                float l2 = qx * qx + qy * qy + qz * qz + qw * qw;
                if (!(Math.Abs(l2 - 1f) <= 1e-3f)) { throw r.Error("the local rotation is not a unit quaternion"); }
                if (Math.Abs(tx) > PlacedShape3D.MaxLocalOffset || Math.Abs(ty) > PlacedShape3D.MaxLocalOffset || Math.Abs(tz) > PlacedShape3D.MaxLocalOffset) { throw r.Error("the local offset is out of range"); }
                Local = new RigidTransformF(new QuaternionF(qx, qy, qz, qw), new Vector3F(tx, ty, tz));
                Friction = r.ReadSingle(); Restitution = r.ReadSingle(); Rolling = r.ReadSingle(); Density = r.ReadSingle();
                if (!(Friction >= 0f && Friction <= MaterialLimits.MaxCoefficient && Rolling >= 0f && Rolling <= MaterialLimits.MaxCoefficient && Restitution >= 0f && Restitution <= 1f && Density >= 0f && float.IsFinite(Density)))
                { throw r.Error("the shape material is out of range"); }
                Category = r.ReadUInt32(); Mask = r.ReadUInt32(); Group = r.ReadInt32();
                Flags = r.ReadByte(); Layer = r.ReadByte();
                if ((Flags & ~FlagSensor) != 0) { throw r.Error("unknown shape flags"); }
                if (Layer >= LayerCount) { throw r.Error("the layer must be below " + LayerCount.ToString(CultureInfo.InvariantCulture)); }
                HasSurface = r.ReadBool();
                Surface = default;
                if (HasSurface)
                {
                    Surface = new SurfaceProperties
                    {
                        Velocity = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle()), OneWayDirection = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle()),
                        FrictionAxis = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle()), LongitudinalFriction = r.ReadSingle(), TransverseFriction = r.ReadSingle(), SpinningFriction = r.ReadSingle(),
                    };
                    try { Surface.Validate(3); } catch (ArgumentException e) { throw r.Error(e.Message); }
                    if (Surface.IsDefault) { throw r.Error("a default surface must be absent"); }
                }

                if (r.Remaining != 0) { throw r.Error("trailing bytes"); }
            }
            catch (SnapshotFormatException e) { throw new BodySetPacketException("shape", e.Offset, e.Message); }
        }
    }

    internal static void ValidateShapeBlob(byte[] blob, byte ownerType)
    {
        var b = new ShapeBlob3D(blob);
        if (b.Recipe.Length == 0) { throw new BodySetPacketException("shape.recipe", 0, "the geometry recipe is empty"); }
        if (b.Recipe[0] == ShapeSpec3D.KVoxelGrid) { throw new BodySetPacketException("shape.recipe", 0, "voxel grids are not carried by packets"); }
        Geom3D g;
        try { g = Geom3D.Build(b.Recipe); }
        catch (Exception e) when (e is ArgumentException || e is SnapshotFormatException || e is PhysicsException || e is NotSupportedException || e is OverflowException)
        { throw new BodySetPacketException("shape.recipe", 0, "the geometry recipe is invalid: " + e.Message); }
        if ((g.IsStaticSoup || (g.IsConvex && g.Convex.Kind == ConvexShapeKind3D.Plane)) && ownerType != (byte)BodyType.Static)
        { throw new BodySetPacketException("shape.recipe", 0, "triangle meshes, height fields and planes can only belong to a static body"); }
    }

    byte[] ITransferEngine.WriteJoint(int c)
    {
        ref JointData3D j = ref Joints[c];
        if (j.Type == 0) { throw Refuse("Body-set transfer: joint " + Core.HandleOfJoint(c) + " has no record."); }
        var type = (JointType3D)j.Type;
        if (type == JointType3D.Pulley || type == JointType3D.Mouse)
        { throw Refuse("Body-set transfer: joint " + Core.HandleOfJoint(c) + " is a " + JointData3D.Name(type) + " joint, which is anchored in world coordinates and is not carried by packets."); }
        var w = StateWriter.ToBuffer();
        j.Write(w);
        w.WriteInt32(JointLifetime.Get(Core.Constraints, c));
        return w.ToArray();
    }

    private static JointData3D ReadJointBlob(byte[] blob)
    {
        var r = new StateReader(blob);
        try
        {
            var d = JointData3D.Read(r);
            int life = r.ReadInt32();
            if (life < 0) { throw r.Error("the remaining lifetime is negative"); }
            if (r.Remaining != 0) { throw r.Error("trailing bytes"); }
            if (d.Type == (byte)JointType3D.Pulley || d.Type == (byte)JointType3D.Mouse) { throw r.Error("pulley and mouse joints are not carried by packets"); }
            d.LifetimeSteps = life;
            return d;
        }
        catch (SnapshotFormatException e) { throw new BodySetPacketException("joint", e.Offset, e.Message); }
    }

    internal static void ValidateJointBlob(byte[] blob) => ReadJointBlob(blob);

    void ITransferEngine.ReadBodyExtras(int b, PacketBody into)
    {
        into.EngineFlags = (byte)(BodyFlags[b] & 1);
        var x = new float[12];
        x[0] = LinearDamping[b]; x[1] = AngularDamping[b]; x[2] = GravityScale[b];
        for (int k = 0; k < 3; k++) { x[3 + k] = Dp[3 * b + k]; x[6 + k] = Force[3 * b + k]; x[9 + k] = Torque[3 * b + k]; }
        into.Extra = x;
    }

    void ITransferEngine.ReadAnchor(int b, PacketBody into)
    {
        // The anchor stands where the body's origin frame is: the joint anchors are given in that frame, and the anchor body has its centre of mass at its origin.
        var bodies = Core.Bodies;
        var q = RotationOfBody(b);
        var lc = LocalCenterOf(b);
        var arm = q.Rotate(lc);
        var pos = bodies.PositionOf(b);
        into.Position = new[] { pos[0] - (double)arm.X, pos[1] - (double)arm.Y, pos[2] - (double)arm.Z };
        into.Rotation = bodies.RotationOf(b).ToArray();
        var v = bodies.LinearVelocityOf(b); var w = bodies.AngularVelocityOf(b);
        // Velocity of the origin: v + w x (origin - centre of mass) = v - w x arm.
        into.Linear = new[] { v[0] - (w[1] * arm.Z - w[2] * arm.Y), v[1] - (w[2] * arm.X - w[0] * arm.Z), v[2] - (w[0] * arm.Y - w[1] * arm.X) };
        into.Angular = w.ToArray();
        into.Extra = new float[12];
        into.Extra[2] = 1f;
    }

    void ITransferEngine.CollectPairs(List<TransferPair> pairs)
    {
        for (int p = 0; p < PairHigh; p++) { if (PairLive[p] && PairCons[p] >= 0) { pairs.Add(new TransferPair(PairA[p], PairB[p], PairCons[p])); } }
    }

    int ITransferEngine.NextInChain(int c) => ConsNext[c];

    void ITransferEngine.DestroyBody(BodyHandle h) => DestroyBody(h);

    // ------------------------------------------------------------------ insertion

    void ITransferEngine.CheckInsert(PacketModel m) { }

    void ITransferEngine.RefreshBodyProxies(int b)
    {
        for (int s = ShapeHead[b]; s >= 0; s = NextShape[s]) { Bp.MoveProxy(Proxy[s], TightBox(b, s), 0.0, 0.0, 0.0); }
    }

    BodyHandle ITransferEngine.CreateBody(PacketBody pb, ReadOnlySpan<double> rel, ReadOnlySpan<float> rot, ReadOnlySpan<float> lin, ReadOnlySpan<float> ang)
    {
        var def = new BodyDef
        {
            Type = (BodyType)pb.Type, Position = new Vector3D(rel[0], rel[1], rel[2]), Rotation = new QuaternionF(rot[0], rot[1], rot[2], rot[3]),
            LinearVelocity = new Vector3F(lin[0], lin[1], lin[2]), AngularVelocity = new Vector3F(ang[0], ang[1], ang[2]),
            LinearDamping = pb.Extra[0], AngularDamping = pb.Extra[1], GravityScale = pb.Extra[2],
            FixedRotation = (pb.EngineFlags & 1) != 0, IsBullet = (pb.Bits & 1) != 0, AllowSleep = (pb.Bits & 2) == 0,
        };
        var h = CreateBody(in def);
        // The definition path normalises the rotation, which can change its last bits; the packet's own bits are what the source world had.
        rot.CopyTo(Core.Bodies.RotationOf(h.Index));
        return h;
    }

    ShapeHandle ITransferEngine.CreateShape(BodyHandle body, byte[] blob)
    {
        var b = new ShapeBlob3D(blob);
        var def = new ShapeDef
        {
            Surface = b.Surface, Density = b.Density, Friction = b.Friction, Restitution = b.Restitution, RollingResistance = b.Rolling, CategoryBits = b.Category, MaskBits = b.Mask,
            GroupIndex = b.Group, IsSensor = (b.Flags & FlagSensor) != 0, Layer = b.Layer, LocalPosition = b.Local.Translation, LocalRotation = b.Local.Rotation,
        };
        // The values are already resolved for this world's length unit (packets only travel between equal units), so they go in as they are.
        return AddShapeResolved(body, b.Recipe, in def);
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
                var t = new double[3];
                t[0] = state.TargetPosition.X; t[1] = state.TargetPosition.Y; t[2] = state.TargetPosition.Z;
                transform.Position(t, t);
                state.TargetPosition = new Vector3D(t[0] - origin[0], t[1] - origin[1], t[2] - origin[2]);
            }

            Core.Controls.Set(b, state);
            if (pb.Type == (byte)BodyType.Dynamic) { UpdateMass(b, true); }
        }

        if (pb.HasSleep) { Core.SetBodySleepRaw(b, pb.Sleep); }
        Core.Bodies.SleepTime[b] = pb.SleepTime;
        var v = new float[3];
        for (int k = 0; k < 3; k++) { v[k] = pb.Extra[3 + k]; }
        transform.Vector(v, v); for (int k = 0; k < 3; k++) { Dp[3 * b + k] = v[k]; }
        for (int k = 0; k < 3; k++) { v[k] = pb.Extra[6 + k]; }
        transform.Vector(v, v); for (int k = 0; k < 3; k++) { Force[3 * b + k] = v[k]; }
        for (int k = 0; k < 3; k++) { v[k] = pb.Extra[9 + k]; }
        transform.Angular(v, v); for (int k = 0; k < 3; k++) { Torque[3 * b + k] = v[k]; }
        // Adding shapes moved the stored position while the centre of mass settled; the packet's position is the final one.
        var pos = Core.Bodies.PositionOf(b);
        for (int k = 0; k < 3; k++) { pos[k] = rel[k]; }
        for (int s = ShapeHead[b]; s >= 0; s = NextShape[s]) { Bp.MoveProxy(Proxy[s], TightBox(b, s), 0.0, 0.0, 0.0); }
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
        if (_skip.Length < PairHigh) { Array.Resize(ref _skip, Math.Max(PairHigh, _skip.Length * 2)); }
        return true;
    }

    int ITransferEngine.FindSupport(byte[] blob, ReadOnlySpan<double> rel, ReadOnlySpan<float> rot)
    {
        var want = new ShapeBlob3D(blob);
        var bodies = Core.Bodies;
        for (int s = 0; s < Core.Shapes.HighWater; s++)
        {
            if (!Core.Shapes.Handles.IsLive(s)) { continue; }
            int b = Core.Shapes.Owner[s];
            if (bodies.TypeOf(b) != BodyType.Static) { continue; }
            var p = bodies.PositionOf(b); var q = bodies.RotationOf(b);
            if (BitConverter.DoubleToInt64Bits(p[0]) != BitConverter.DoubleToInt64Bits(rel[0]) || BitConverter.DoubleToInt64Bits(p[1]) != BitConverter.DoubleToInt64Bits(rel[1]) || BitConverter.DoubleToInt64Bits(p[2]) != BitConverter.DoubleToInt64Bits(rel[2])) { continue; }
            if (q[0] != rot[0] || q[1] != rot[1] || q[2] != rot[2] || q[3] != rot[3]) { continue; }
            var l = Local[s];
            if (l.Rotation.X != want.Local.Rotation.X || l.Rotation.Y != want.Local.Rotation.Y || l.Rotation.Z != want.Local.Rotation.Z || l.Rotation.W != want.Local.Rotation.W
                || l.Translation.X != want.Local.Translation.X || l.Translation.Y != want.Local.Translation.Y || l.Translation.Z != want.Local.Translation.Z) { continue; }
            if (!Geoms[GeomOf[s]]!.Recipe.AsSpan().SequenceEqual(want.Recipe)) { continue; }
            return s;
        }

        return -1;
    }

    void ITransferEngine.SwapContactPayload(Span<float> payload)
    {
        for (int k = 0; k < 3; k++) { payload[ManifoldLayout.Normal + k] = -payload[ManifoldLayout.Normal + k]; payload[ManifoldLayout.RollingImpulse + k] = 0f; }
        for (int i = 0; i < 4; i++)
        {
            for (int k = 0; k < 3; k++) { (payload[ManifoldLayout.AnchorA(i) + k], payload[ManifoldLayout.AnchorB(i) + k]) = (payload[ManifoldLayout.AnchorB(i) + k], payload[ManifoldLayout.AnchorA(i) + k]); }
            payload[ManifoldLayout.Tangent1Impulse(i)] = 0f; payload[ManifoldLayout.Tangent2Impulse(i)] = 0f;
        }

        payload.Slice(ManifoldLayout.HintOffset).Clear();
    }

    void ITransferEngine.LinkContact(int pair, int previous, int c)
    {
        Grow(ref ConsNext, c + 1);
        ConsNext[c] = -1;
        if (previous < 0) { PairCons[pair] = c; } else { ConsNext[previous] = c; }
    }

    void ITransferEngine.TransformContactPayload(Span<float> payload, TransferTransform transform)
    {
        var v = new float[3];
        Turn(payload, ManifoldLayout.Normal, v, transform);
        // The tangent basis follows from the normal and is not rotation-covariant, so impulses along it start again from zero; the normal impulses stay.
        for (int k = 0; k < 3; k++) { payload[ManifoldLayout.RollingImpulse + k] = 0f; }
        for (int i = 0; i < 4; i++)
        {
            Turn(payload, ManifoldLayout.AnchorA(i), v, transform); Turn(payload, ManifoldLayout.AnchorB(i), v, transform);
            payload[ManifoldLayout.Tangent1Impulse(i)] = 0f; payload[ManifoldLayout.Tangent2Impulse(i)] = 0f;
        }
    }

    private static void Turn(Span<float> payload, int at, float[] v, TransferTransform transform)
    {
        v[0] = payload[at]; v[1] = payload[at + 1]; v[2] = payload[at + 2];
        transform.Vector(v, v);
        payload[at] = v[0]; payload[at + 1] = v[1]; payload[at + 2] = v[2];
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
                if (jb.Blob.Length == 0 || jb.Blob[0] != (byte)JointType3D.SwingTwist) { throw r.Error("part " + i.ToString(CultureInfo.InvariantCulture) + " is not joined by a swing-twist joint"); }
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
