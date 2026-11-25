using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Characters;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

// Characters and vehicles in body-set packets (docs/spec/transfer.md, "Entities"): what the 2D engine writes for them, how it checks the bytes and how it builds them again in another world.
internal sealed partial class Engine2D
{
    // ------------------------------------------------------------------ what can travel

    /// <summary>The reason a character cannot travel with the packet's bodies, or null. <paramref name="inSet"/> is the set of bodies that travel.</summary>
    private string? CharacterRefusal(int c, bool[] inSet)
    {
        var ext = _charExt;
        if (ext == null || c >= ext.States.Length) { return null; }
        ref var st = ref ext.States[c];
        string who = "the character " + Core.HandleOfBody(CharBody[c]);
        switch (st.Mode)
        {
            case CharacterMode.Roped:
                return "Body-set transfer: " + who + " holds a rope, and a packet does not carry rods. Release the grip first.";
            case CharacterMode.Seated:
            case CharacterMode.Hanging:
            case CharacterMode.Climbing:
                if (!Core.Bodies.Handles.IsAlive(st.Body, st.BodyGen) || (uint)st.Body >= (uint)inSet.Length || !inSet[st.Body])
                {
                    return "Body-set transfer: " + who + " is " + (st.Mode == CharacterMode.Seated ? "seated on" : st.Mode == CharacterMode.Hanging ? "hanging from" : "climbing") + " body " + Core.HandleOfBody(st.Body)
                        + ", which is not in the set. Put that body in the set too, or release the character first.";
                }
                return null;
            default:
                return null;
        }
    }

    // ------------------------------------------------------------------ extraction

    void ITransferEngine.WriteEntities(bool[] inSet, bool[] entitySet, int[] bodyIndex, List<PacketEntity> into)
    {
        for (int c = 0; c < CharHandles.HighWater; c++)
        {
            if (!CharHandles.IsLive(c) || !entitySet[CharBody[c]]) { continue; }
            var w = StateWriter.ToBuffer();
            WriteCharacterEntity(w, c, inSet, bodyIndex);
            into.Add(new PacketEntity { Kind = PacketEntity.KindCharacter, Slot = c, Generation = CharHandles.GenerationOf(c), Blob = w.ToArray() });
        }

        for (int v = 0; v < VehHandles.HighWater; v++)
        {
            if (!VehHandles.IsLive(v) || !entitySet[Vehicles[v]!.Body]) { continue; }
            var w = StateWriter.ToBuffer();
            WriteVehicleEntity(w, Vehicles[v]!, inSet, bodyIndex);
            into.Add(new PacketEntity { Kind = PacketEntity.KindVehicle, Slot = v, Generation = VehHandles.GenerationOf(v), Blob = w.ToArray() });
        }
    }

    bool ITransferEngine.HasEntity(int body) => IsCharacterBody(body) || VehicleOfBody(body) >= 0;

    void ITransferEngine.FindUncarriable(bool[] set, List<(int Body, string Reason)> skipped)
    {
        var all = new List<int>();
        for (int b = 0; b < set.Length; b++) { if (set[b]) { all.Add(b); } }
        if (all.Count == 0) { return; }
        try { ((ITransferEngine)this).CheckMembers(all.ToArray(), set); return; }
        catch (BodySetTransferException) { }

        bool changed;
        do
        {
            changed = false;
            for (int b = 0; b < set.Length; b++)
            {
                if (!set[b]) { continue; }
                string? why = null;
                try { CheckMembers(new[] { b }, SingleBody(b, set.Length), set); }
                catch (BodySetTransferException e) { why = e.Message; }
                if (why == null) { continue; }
                set[b] = false; skipped.Add((b, why)); changed = true;
            }

            // A ragdoll goes whole or not at all.
            for (int r = 0; r < RagHandles.HighWater; r++)
            {
                if (!RagHandles.IsLive(r)) { continue; }
                var parts = Ragdolls[r]!.Body;
                bool any = false, missing = false;
                foreach (int b in parts) { if ((uint)b < (uint)set.Length && set[b]) { any = true; } else { missing = true; } }
                if (!any || !missing) { continue; }
                foreach (int b in parts)
                {
                    if ((uint)b < (uint)set.Length && set[b]) { set[b] = false; skipped.Add((b, "Body-set transfer: the ragdoll #" + r.ToString(CultureInfo.InvariantCulture) + " has a part that cannot be carried, so none of it is.")); changed = true; }
                }
            }
        }
        while (changed);
    }

    private static bool[] SingleBody(int b, int length) { var one = new bool[length]; one[b] = true; return one; }

    private void WriteCharacterEntity(StateWriter w, int c, bool[] inSet, int[] bodyIndex)
    {
        var d = CharDef[c];
        w.NamedInt32(bodyIndex[CharBody[c]]);
        w.NamedBool(d.NeedsExactSlope);
        w.NamedSingle(d.Radius); w.NamedSingle(d.Height); w.NamedSingle(d.NeedsExactSlope ? d.SlopeLimitRadians : d.SlopeLimitDegrees); w.NamedSingle(d.StepOffset); w.NamedSingle(d.Skin);
        w.NamedUInt32(d.LayerMask); w.NamedSingle(d.PushStrength); w.NamedInt32(d.Layer); w.NamedUInt32(d.CategoryBits); w.NamedUInt32(d.MaskBits);
        w.NamedSingle(d.Mass); w.NamedSingle(d.MaxPushForce);
        // What the character stood on: kept when that body travels too, otherwise the character is in the air until its next move finds the ground again.
        int gb = CharGroundBody[c];
        bool groundTravels = CharGround[c] == GroundState.OnGround && gb >= 0 && Core.Bodies.Handles.IsAlive(gb, CharGroundGen[c]) && gb < inSet.Length && inSet[gb];
        if (CharGround[c] == GroundState.OnGround && !groundTravels)
        {
            w.NamedByte((byte)GroundState.InAir); w.NamedSingle(0f); w.NamedSingle(0f); w.NamedInt32(-1);
        }
        else
        {
            w.NamedByte((byte)CharGround[c]); w.NamedSingle(CharNx[c]); w.NamedSingle(CharNy[c]);
            w.NamedInt32(groundTravels ? bodyIndex[gb] : -1);
        }

        w.NamedSingle(CharVx[c]); w.NamedSingle(CharVy[c]);
        var st = _charExt != null && c < _charExt.States.Length ? _charExt.States[c] : default;
        bool held = st.Mode == CharacterMode.Seated || st.Mode == CharacterMode.Hanging || st.Mode == CharacterMode.Climbing;
        if (st.Riding && !(Core.Bodies.Handles.IsAlive(st.Body, st.BodyGen) && st.Body < inSet.Length && inSet[st.Body])) { st.Riding = false; }
        bool has = CharacterExtStore.InUse(in st);
        w.NamedBool(has);
        if (has)
        {
            if (held || st.Riding) { st.Body = bodyIndex[st.Body]; st.BodyGen = 1; } else { st.Body = 0; st.BodyGen = 0; }
            st.SavedGeneration = 0;
            CharacterExtStore.WriteState(w, in st, true);
        }
    }

    private sealed class CharacterEntity3D
    {
        public int Body;
        public CharacterDef Def;
        public GroundState Ground;
        public float Nx, Ny, Vx, Vy;
        public int GroundBody;
        public bool HasAbilities;
        public CharacterExtState Ability;
    }

    private static CharacterEntity3D ReadCharacterEntity(StateReader r, int members, float units)
    {
        var e = new CharacterEntity3D();
        e.Body = r.ReadInt32();
        if (e.Body < 0 || e.Body >= members) { throw r.Error("the character's body is not one of the set's bodies"); }
        bool exact = r.ReadBool();
        var d = new CharacterDef
        {
            Radius = r.ReadSingle(), Height = r.ReadSingle(), SlopeLimitDegrees = r.ReadSingle(), StepOffset = r.ReadSingle(), Skin = r.ReadSingle(),
            LayerMask = r.ReadUInt32(), PushStrength = r.ReadSingle(), Layer = r.ReadInt32(), CategoryBits = r.ReadUInt32(), MaskBits = r.ReadUInt32(),
        };
        d.Mass = r.ReadSingle();
        d.MaxPushForce = r.ReadSingle();
        if (exact)
        {
            d.SlopeLimitRadians = d.SlopeLimitDegrees;
            if (!d.NeedsExactSlope) { throw r.Error("redundant exact character slope encoding"); }
        }

        try { d.Validate(units); }
        catch (ArgumentOutOfRangeException ex) { throw r.Error("invalid character definition: " + ex.Message); }
        e.Def = d;
        byte gs = r.ReadByte();
        if (gs > (byte)GroundState.OnGround) { throw r.Error("invalid character ground state"); }
        e.Ground = (GroundState)gs;
        e.Nx = r.ReadSingle(); e.Ny = r.ReadSingle();
        e.GroundBody = r.ReadInt32();
        e.Vx = r.ReadSingle(); e.Vy = r.ReadSingle();
        if (!float.IsFinite(e.Nx + e.Ny + e.Vx + e.Vy)) { throw r.Error("non-finite character state"); }
        if (Math.Abs(e.Vx) > 1e6f || Math.Abs(e.Vy) > 1e6f) { throw r.Error("character velocity beyond 1e6"); }
        if (e.GroundBody < -1 || e.GroundBody >= members) { throw r.Error("the character's ground body is not one of the set's bodies"); }
        if (e.Ground == GroundState.OnGround && e.GroundBody < 0) { throw r.Error("a character on ground names no ground body"); }
        e.HasAbilities = r.ReadBool();
        if (e.HasAbilities)
        {
            e.Ability = CharacterExtStore.ReadState(r, true, true, 0);
            ref var a = ref e.Ability;
            if (a.Mode == CharacterMode.Roped) { throw r.Error("a packet does not carry a rope grip"); }
            if (a.Mode != CharacterMode.Free || a.Riding)
            {
                if (a.Body < 0 || a.Body >= members || a.BodyGen != 1) { throw r.Error("the held body is not one of the set's bodies"); }
            }

            a.SavedGeneration = 0;
        }

        return e;
    }

    // ------------------------------------------------------------------ vehicles

    private void WriteVehicleEntity(StateWriter w, VehicleState s, bool[] inSet, int[] bodyIndex)
    {
        w.NamedInt32(bodyIndex[s.Body]);
        w.NamedByte((byte)s.Kind);
        WriteVehicleDef(w, s.Def);
        if (s.Kind == VehicleKind.Tracked)
        {
            var t = s.Track!;
            w.NamedSingle(t.SprocketRadius); w.NamedSingle(t.MaxTrackSpeed); w.NamedSingle(t.MaxDriveTorque); w.NamedSingle(t.MaxBrakeTorque);
        }
        else if (s.Kind == VehicleKind.Motorcycle) { w.NamedSingle(s.Bike!.LeanTorque); }

        w.NamedSingle(s.Forward); w.NamedSingle(s.Pitch); w.NamedSingle(s.Brake); w.NamedSingle(s.Handbrake);
        w.NamedSingle(s.EngineW); w.NamedInt32(s.Gear); w.NamedSingle(s.ShiftTimer);
        var origin = Core.Origin;
        for (int i = 0; i < s.Wheels; i++)
        {
            // The ground a wheel stands on stays when that body travels; a wheel on anything else is up in the air until its next cast finds the ground again (the wheel state is rebuilt by every step's cast).
            int gb = s.GroundBody[i];
            bool travels = s.Contact[i] != 0 && gb >= 0 && Core.Bodies.Handles.IsAlive(gb, s.GroundGen[i]) && gb < inSet.Length && inSet[gb];
            w.NamedSingle(s.Sprung[i]); w.NamedSingle(s.Omega[i]); w.NamedSingle(s.Spin[i]); w.NamedSingle(s.Length[i]);
            w.NamedSingle(s.SpringForce[i]); w.NamedSingle(s.SlipRatio[i]); w.NamedSingle(s.Fx[i]);
            w.NamedSingle(s.Nx[i]); w.NamedSingle(s.Ny[i]);
            w.NamedDouble(origin[0] + s.Cx[i]); w.NamedDouble(origin[1] + s.Cy[i]);
            w.NamedByte(travels ? (byte)1 : (byte)0); w.NamedInt32(travels ? bodyIndex[gb] : -1);
        }
    }

    private static VehicleState ReadVehicleEntity(StateReader r, int members, float units, out int chassis, out int[] groundIndex)
    {
        chassis = r.ReadInt32();
        if (chassis < 0 || chassis >= members) { throw r.Error("the chassis is not one of the set's bodies"); }
        var s = new VehicleState();
        byte kind = r.ReadByte();
        if (kind > (byte)VehicleKind.Motorcycle) { throw r.Error("unknown vehicle kind " + kind.ToString(CultureInfo.InvariantCulture)); }
        s.Kind = (VehicleKind)kind;
        s.Def = ReadVehicleDef(r, units);
        try
        {
            if (s.Kind == VehicleKind.Tracked)
            {
                var t = new TrackedVehicleDef
                {
                    Wheels = s.Def.Wheels, Grip = s.Def.Wheels[0].Tyre, SprocketRadius = r.ReadSingle(), MaxTrackSpeed = r.ReadSingle(), MaxDriveTorque = r.ReadSingle(), MaxBrakeTorque = r.ReadSingle(),
                    AirPitchTorque = s.Def.AirPitchTorque,
                };
                t.Units = units;
                t.Validate();
                s.Track = t.Clone();
            }
            else if (s.Kind == VehicleKind.Motorcycle)
            {
                if (s.Wheels != 2 || s.Def.Differentials.Length != 1) { throw r.Error("a motorcycle needs two wheels and one differential"); }
                var m = new MotorcycleDef
                {
                    Front = s.Def.Wheels[0], Rear = s.Def.Wheels[1], Engine = s.Def.Engine, Transmission = s.Def.Transmission, FinalDriveRatio = s.Def.Differentials[0].Ratio, LeanTorque = r.ReadSingle(),
                };
                m.Units = units;
                m.Validate();
                s.Bike = m.Clone();
            }
        }
        catch (ArgumentException e) { throw r.Error("invalid vehicle kind parameters: " + e.Message); }

        s.Forward = r.ReadSingle(); s.Pitch = r.ReadSingle(); s.Brake = r.ReadSingle(); s.Handbrake = r.ReadSingle();
        s.EngineW = r.ReadSingle(); s.Gear = r.ReadInt32(); s.ShiftTimer = r.ReadSingle();
        if (!(Math.Abs(s.Forward) <= 1f) || !(Math.Abs(s.Pitch) <= 1f) || !(s.Brake >= 0f && s.Brake <= 1f) || !(s.Handbrake >= 0f && s.Handbrake <= 1f)) { throw r.Error("invalid vehicle inputs"); }
        float engineTop = 1.2f * (s.Def.Engine.MaxRpm * (TwoPi / 60f));
        if (!float.IsFinite(s.EngineW) || s.EngineW < 0f || s.EngineW > engineTop || !float.IsFinite(s.ShiftTimer) || s.ShiftTimer < 0f) { throw r.Error("invalid engine state"); }
        if (s.Gear < -1 || s.Gear > s.Def.Transmission.Ratios.Length) { throw r.Error("invalid gear"); }
        s.Allocate();
        groundIndex = new int[s.Wheels];
        for (int i = 0; i < s.Wheels; i++)
        {
            s.Sprung[i] = r.ReadSingle(); s.Omega[i] = r.ReadSingle(); s.Spin[i] = r.ReadSingle(); s.Length[i] = r.ReadSingle();
            s.SpringForce[i] = r.ReadSingle(); s.SlipRatio[i] = r.ReadSingle(); s.Fx[i] = r.ReadSingle();
            s.Nx[i] = r.ReadSingle(); s.Ny[i] = r.ReadSingle();
            s.Cx[i] = r.ReadDouble(); s.Cy[i] = r.ReadDouble();
            s.Contact[i] = r.ReadByte(); groundIndex[i] = r.ReadInt32();
            float sum = s.Sprung[i] + s.Omega[i] + s.Spin[i] + s.Length[i] + s.SpringForce[i] + s.SlipRatio[i] + s.Fx[i] + s.Nx[i] + s.Ny[i];
            if (!float.IsFinite(sum) || !double.IsFinite(s.Cx[i] + s.Cy[i])) { throw r.Error("non-finite wheel state"); }
            if (!(s.Sprung[i] > 0f) || s.Sprung[i] > 1e30f || s.Contact[i] > 1) { throw r.Error("invalid wheel state"); }
            if (!(Math.Abs(s.Omega[i]) <= MaxWheelSpin)) { throw r.Error("invalid wheel state: angular velocity beyond the limit"); }
            if (groundIndex[i] < -1 || groundIndex[i] >= members || (s.Contact[i] != 0) != (groundIndex[i] >= 0)) { throw r.Error("invalid wheel ground body"); }
            if (Math.Abs(s.Cx[i]) > 8388608.0 || Math.Abs(s.Cy[i]) > 8388608.0) { throw r.Error("wheel contact point beyond +-2^23 m"); }
        }

        return s;
    }

    internal static void ValidateEntityBlob(byte kind, byte[] blob, PacketModel m, bool[] usedCharacter, bool[] usedVehicle)
    {
        var r = new StateReader(blob);
        float units = new LengthUnit(m.MetresPerUnit).Scale;
        try
        {
            if (kind == PacketEntity.KindCharacter)
            {
                var e = ReadCharacterEntity(r, m.MemberCount, units);
                if (r.Remaining != 0) { throw r.Error("trailing bytes"); }
                if (usedCharacter[e.Body]) { throw r.Error("two characters on one body"); }
                usedCharacter[e.Body] = true;
                if (m.Bodies[e.Body].Type != (byte)BodyType.Kinematic) { throw r.Error("a character needs a kinematic body"); }
            }
            else
            {
                ReadVehicleEntity(r, m.MemberCount, units, out int chassis, out _);
                if (r.Remaining != 0) { throw r.Error("trailing bytes"); }
                if (usedVehicle[chassis]) { throw r.Error("two vehicles on one chassis"); }
                usedVehicle[chassis] = true;
                if (m.Bodies[chassis].Type != (byte)BodyType.Dynamic) { throw r.Error("a vehicle's chassis must be a dynamic body"); }
            }
        }
        catch (SnapshotFormatException e) { throw new BodySetPacketException("entity", e.Offset, e.Message); }
    }

    // ------------------------------------------------------------------ insertion

    (int Slot, int Generation) ITransferEngine.CreateEntity(PacketEntity record, PacketModel model, BodyHandle[] bodies, TransferTransform transform)
    {
        var r = new StateReader(record.Blob);
        var origin = Core.Origin;
        if (record.Kind == PacketEntity.KindCharacter)
        {
            var e = ReadCharacterEntity(r, bodies.Length, Opt.UnitsPerMetre);
            var def = e.Def;
            int c = CreateCharacter(bodies[e.Body], in def);
            var n = new[] { e.Nx, e.Ny }; transform.Vector(n, n);
            var v = new[] { e.Vx, e.Vy }; transform.Vector(v, v);
            CharGround[c] = e.Ground; CharNx[c] = n[0]; CharNy[c] = n[1];
            if (e.GroundBody >= 0) { CharGroundBody[c] = bodies[e.GroundBody].Index; CharGroundGen[c] = bodies[e.GroundBody].Generation; } else { CharGroundBody[c] = -1; CharGroundGen[c] = 0; }
            CharVx[c] = v[0]; CharVy[c] = v[1];
            if (e.HasAbilities)
            {
                var a = e.Ability;
                if (a.Mode != CharacterMode.Free || a.Riding) { int held = a.Body; a.Body = bodies[held].Index; a.BodyGen = bodies[held].Generation; }
                var sv = new[] { a.SwimVelocity.X, a.SwimVelocity.Y }; transform.Vector(sv, sv);
                a.SwimVelocity = new Vector3F(sv[0], sv[1], 0f);
                Extensions.Set(c, in a);
            }

            return (c, CharHandles.GenerationOf(c));
        }
        else
        {
            var s = ReadVehicleEntity(r, bodies.Length, Opt.UnitsPerMetre, out int chassis, out int[] ground);
            s.Body = bodies[chassis].Index; s.BodyGen = bodies[chassis].Generation;
            var p = new double[2];
            for (int i = 0; i < s.Wheels; i++)
            {
                p[0] = s.Cx[i]; p[1] = s.Cy[i];
                transform.Position(p, p);
                s.Cx[i] = p[0] - origin[0]; s.Cy[i] = p[1] - origin[1];
                var n = new[] { s.Nx[i], s.Ny[i] }; transform.Vector(n, n);
                s.Nx[i] = n[0]; s.Ny[i] = n[1];
                if (ground[i] >= 0) { s.GroundBody[i] = bodies[ground[i]].Index; s.GroundGen[i] = bodies[ground[i]].Generation; } else { s.GroundBody[i] = -1; s.GroundGen[i] = 0; }
            }

            int slot = RegisterVehicle(s);
            return (slot, VehHandles.GenerationOf(slot));
        }
    }
}
