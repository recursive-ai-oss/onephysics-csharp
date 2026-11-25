using System;
using System.Buffers.Binary;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>Writes and reads the packet byte layout of <c>docs/spec/transfer.md</c>: little-endian, counts first, every record fixed-shape for its dimension.</summary>
internal static class PacketIO
{
    public const uint Magic = 0x5342504F;   // "OPBS"
    public const uint Version = 1;
    internal const int MaxEntities = 1 << 24;

    // ------------------------------------------------------------------ write

    public static byte[] Write(PacketModel m)
    {
        var w = StateWriter.ToBuffer();
        w.WriteUInt32(Magic); w.WriteUInt32(Version);
        w.WriteInt32(m.Dimension); w.WriteDouble(m.MetresPerUnit);
        w.WriteInt32(m.PositionLanes); w.WriteInt32(m.RotationLanes); w.WriteInt32(m.LinearLanes); w.WriteInt32(m.AngularLanes); w.WriteInt32(m.MaxManifoldPoints); w.WriteInt32(m.PayloadFloats);
        w.WriteInt32(m.MemberCount); w.WriteInt32(m.Bodies.Length - m.MemberCount); w.WriteInt32(m.Shapes.Length); w.WriteInt32(m.Joints.Length); w.WriteInt32(m.Contacts.Length); w.WriteInt32(m.Supports.Length); w.WriteInt32(m.SupportContacts.Length); w.WriteInt32(m.Ragdolls.Length);
        foreach (var b in m.Bodies)
        {
            w.WriteInt32(b.Slot); w.WriteInt32(b.Generation);
            w.WriteByte(b.Type); w.WriteByte(b.Bits); w.WriteByte(b.Asleep ? (byte)1 : (byte)0); w.WriteByte(b.EngineFlags);
            w.WriteSingle(b.SleepTime);
            w.WriteDoubles(b.Position); w.WriteSingles(b.Rotation); w.WriteSingles(b.Linear); w.WriteSingles(b.Angular); w.WriteSingles(b.Extra);
            w.WriteBool(b.HasControls);
            if (b.HasControls)
            {
                var c = b.Controls;
                w.WriteInt32((int)c.Settings.Constraints); w.WriteSingle(c.Settings.MaxLinearSpeed); w.WriteSingle(c.Settings.MaxAngularSpeed); w.WriteSingle(c.Settings.MaxDepenetrationSpeed);
                w.WriteByte(c.Overrides); w.WriteSingle(c.Mass);
                w.WriteSingle(c.Center.X); w.WriteSingle(c.Center.Y); w.WriteSingle(c.Center.Z);
                w.WriteSingle(c.Principal.X); w.WriteSingle(c.Principal.Y); w.WriteSingle(c.Principal.Z);
                w.WriteSingle(c.InertiaRotation.X); w.WriteSingle(c.InertiaRotation.Y); w.WriteSingle(c.InertiaRotation.Z); w.WriteSingle(c.InertiaRotation.W);
                w.WriteDouble(c.TargetPosition.X); w.WriteDouble(c.TargetPosition.Y); w.WriteDouble(c.TargetPosition.Z);
                w.WriteSingle(c.TargetRotation.X); w.WriteSingle(c.TargetRotation.Y); w.WriteSingle(c.TargetRotation.Z); w.WriteSingle(c.TargetRotation.W);
            }

            w.WriteBool(b.HasSleep);
            if (b.HasSleep) { w.WriteSingle(b.Sleep.LinearThreshold); w.WriteSingle(b.Sleep.AngularThreshold); w.WriteSingle(b.Sleep.TimeToSleep); }
        }

        foreach (var s in m.Shapes)
        {
            w.WriteInt32(s.Slot); w.WriteInt32(s.Generation); w.WriteInt32(s.Body); w.WriteInt32(s.Blob.Length); w.WriteBytes(s.Blob);
        }

        foreach (var j in m.Joints)
        {
            w.WriteInt32(j.Slot); w.WriteInt32(j.Generation); w.WriteInt32(j.BodyA); w.WriteInt32(j.BodyB);
            w.WriteByte(j.Flags); w.WriteByte(j.Color); w.WriteSingle(j.BreakValue); w.WriteInt32(j.Blob.Length); w.WriteBytes(j.Blob); w.WriteSingles(j.Payload);
        }

        foreach (var c in m.Contacts)
        {
            w.WriteInt32(c.Slot); w.WriteInt32(c.Generation); w.WriteInt32(c.ShapeA); w.WriteInt32(c.ShapeB); w.WriteInt32(c.SubA); w.WriteInt32(c.SubB);
            w.WriteByte(c.Flags); w.WriteByte(c.Color); w.WriteByte(c.Points); w.WriteBool(c.NewPair); w.WriteUInt32s(c.Features); w.WriteSingle(c.BreakValue); w.WriteSingles(c.Payload);
        }

        foreach (var sp in m.Supports)
        {
            w.WriteDoubles(sp.Position); w.WriteSingles(sp.Rotation); w.WriteInt32(sp.Blob.Length); w.WriteBytes(sp.Blob);
        }

        foreach (var c in m.SupportContacts)
        {
            w.WriteInt32(c.Member); w.WriteInt32(c.Support); w.WriteBool(c.MemberIsA); w.WriteInt32(c.SubA); w.WriteInt32(c.SubB);
            w.WriteByte(c.Flags); w.WriteByte(c.Color); w.WriteByte(c.Points); w.WriteBool(c.NewPair); w.WriteUInt32s(c.Features); w.WriteSingle(c.BreakValue); w.WriteSingles(c.Payload);
        }

        foreach (var rd in m.Ragdolls)
        {
            w.WriteInt32(rd.Slot); w.WriteInt32(rd.Generation); w.WriteInt32(rd.Blob.Length); w.WriteBytes(rd.Blob);
        }

        return w.ToArray();
    }

    // ------------------------------------------------------------------ read

    private sealed class Reader
    {
        private readonly byte[] _d;
        public int Pos;
        public Reader(byte[] d) { _d = d; }
        public int Remaining => _d.Length - Pos;

        public BodySetPacketException Bad(string field, string message) => new(field, Pos, message);

        private ReadOnlySpan<byte> Take(int n, string field)
        {
            if (n < 0 || Remaining < n) { throw Bad(field, "the packet ends here; " + n.ToString(CultureInfo.InvariantCulture) + " more bytes were expected, " + Remaining.ToString(CultureInfo.InvariantCulture) + " remain"); }
            var s = _d.AsSpan(Pos, n);
            Pos += n;
            return s;
        }

        public byte Byte(string f) => Take(1, f)[0];
        public bool Bool(string f) { byte v = Byte(f); if (v > 1) { Pos--; throw Bad(f, "a boolean must be 0 or 1, found " + v.ToString(CultureInfo.InvariantCulture)); } return v == 1; }
        public int Int(string f) => BinaryPrimitives.ReadInt32LittleEndian(Take(4, f));
        public uint UInt(string f) => BinaryPrimitives.ReadUInt32LittleEndian(Take(4, f));
        public long Long(string f) => BinaryPrimitives.ReadInt64LittleEndian(Take(8, f));
        public float Float(string f)
        {
            float v = BitConverter.UInt32BitsToSingle(BinaryPrimitives.ReadUInt32LittleEndian(Take(4, f)));
            if (!float.IsFinite(v)) { Pos -= 4; throw Bad(f, "must be finite"); }
            return v;
        }

        public double Double(string f)
        {
            double v = BitConverter.UInt64BitsToDouble(BinaryPrimitives.ReadUInt64LittleEndian(Take(8, f)));
            if (!double.IsFinite(v)) { Pos -= 8; throw Bad(f, "must be finite"); }
            return v;
        }

        public float[] Floats(int n, string f) { var a = new float[n]; for (int i = 0; i < n; i++) { a[i] = Float(f + "[" + i.ToString(CultureInfo.InvariantCulture) + "]"); } return a; }
        public double[] Doubles(int n, string f) { var a = new double[n]; for (int i = 0; i < n; i++) { a[i] = Double(f + "[" + i.ToString(CultureInfo.InvariantCulture) + "]"); } return a; }
        public byte[] Bytes(int n, string f) => Take(n, f).ToArray();

        public int Count(string f, int minBytesEach)
        {
            int n = Int(f);
            if (n < 0 || n > MaxEntities) { Pos -= 4; throw Bad(f, "count " + n.ToString(CultureInfo.InvariantCulture) + " is outside 0.." + MaxEntities.ToString(CultureInfo.InvariantCulture)); }
            if ((long)n * minBytesEach > Remaining) { Pos -= 4; throw Bad(f, "count " + n.ToString(CultureInfo.InvariantCulture) + " needs at least " + ((long)n * minBytesEach).ToString(CultureInfo.InvariantCulture) + " bytes but " + Remaining.ToString(CultureInfo.InvariantCulture) + " remain"); }
            return n;
        }
    }

    private static string At(string name, int i, string field) => name + "[" + i.ToString(CultureInfo.InvariantCulture) + "]." + field;

    public static PacketModel Read(byte[] data)
    {
        var r = new Reader(data);
        if (r.UInt("magic") != Magic) { r.Pos = 0; throw r.Bad("magic", "not a body-set packet"); }
        uint version = r.UInt("version");
        if (version != Version) { r.Pos -= 4; throw r.Bad("version", "unsupported packet version " + version.ToString(CultureInfo.InvariantCulture) + "; this reader knows version 1"); }
        var m = new PacketModel();
        int dim = r.Int("dimension");
        if (dim != 2 && dim != 3) { r.Pos -= 4; throw r.Bad("dimension", "must be 2 or 3, found " + dim.ToString(CultureInfo.InvariantCulture)); }
        m.Dimension = dim;
        m.MetresPerUnit = r.Double("metresPerUnit");
        if (!(m.MetresPerUnit >= 1e-6 && m.MetresPerUnit <= 1e6)) { r.Pos -= 8; throw r.Bad("metresPerUnit", "must be within 1e-6..1e6"); }
        int[] want = { m.PositionLanes, m.RotationLanes, m.LinearLanes, m.AngularLanes, m.MaxManifoldPoints, m.PayloadFloats };
        string[] names = { "positionLanes", "rotationLanes", "linearLanes", "angularLanes", "maxManifoldPoints", "payloadFloats" };
        for (int i = 0; i < want.Length; i++)
        {
            int v = r.Int(names[i]);
            if (v != want[i]) { r.Pos -= 4; throw r.Bad(names[i], "is " + v.ToString(CultureInfo.InvariantCulture) + " but a " + dim.ToString(CultureInfo.InvariantCulture) + "D packet has " + want[i].ToString(CultureInfo.InvariantCulture)); }
        }

        int members = r.Count("memberCount", 30);
        int anchors = r.Count("anchorCount", 30);
        if ((long)members + anchors > MaxEntities) { throw r.Bad("anchorCount", "too many bodies"); }
        int nShapes = r.Count("shapeCount", 16);
        int nJoints = r.Count("jointCount", 20);
        int nContacts = r.Count("contactCount", 20);
        int nSupports = r.Count("supportCount", 20);
        int nSupportContacts = r.Count("supportContactCount", 20);
        int nRagdolls = r.Count("ragdollCount", 12);
        m.MemberCount = members;
        m.Bodies = new PacketBody[members + anchors];
        int extra = m.ExtraFloats;
        for (int i = 0; i < m.Bodies.Length; i++)
        {
            var b = new PacketBody { Anchor = i >= members };
            m.Bodies[i] = b;
            b.Slot = r.Int(At("bodies", i, "slot")); b.Generation = r.Int(At("bodies", i, "generation"));
            if (b.Slot < 0 || b.Generation <= 0) { throw r.Bad(At("bodies", i, "slot"), "slot and generation must be positive handles of the source world"); }
            b.Type = r.Byte(At("bodies", i, "type"));
            if (b.Type > (byte)BodyType.Dynamic) { r.Pos--; throw r.Bad(At("bodies", i, "type"), "unknown body type " + b.Type.ToString(CultureInfo.InvariantCulture)); }
            if (b.Anchor && b.Type == (byte)BodyType.Dynamic) { r.Pos--; throw r.Bad(At("bodies", i, "type"), "an anchor is static or kinematic"); }
            b.Bits = r.Byte(At("bodies", i, "bits"));
            if ((b.Bits & ~3) != 0) { r.Pos--; throw r.Bad(At("bodies", i, "bits"), "unknown bits"); }
            b.Asleep = r.Bool(At("bodies", i, "asleep"));
            if (b.Asleep && b.Type != (byte)BodyType.Dynamic) { r.Pos--; throw r.Bad(At("bodies", i, "asleep"), "only a dynamic body can be asleep"); }
            b.EngineFlags = r.Byte(At("bodies", i, "engineFlags"));
            if ((b.EngineFlags & ~3) != 0) { r.Pos--; throw r.Bad(At("bodies", i, "engineFlags"), "unknown flags"); }
            b.SleepTime = r.Float(At("bodies", i, "sleepTime"));
            if (b.SleepTime < 0f) { throw r.Bad(At("bodies", i, "sleepTime"), "must not be negative"); }
            b.Position = r.Doubles(m.PositionLanes, At("bodies", i, "position"));
            foreach (double p in b.Position) { if (Math.Abs(p) > 8388608.0) { throw r.Bad(At("bodies", i, "position"), "beyond +-2^23 m"); } }
            b.Rotation = r.Floats(m.RotationLanes, At("bodies", i, "rotation"));
            double q2 = 0; foreach (float q in b.Rotation) { q2 += (double)q * q; }
            if (!(Math.Abs(q2 - 1.0) <= 1e-3)) { throw r.Bad(At("bodies", i, "rotation"), "must be a unit rotation"); }
            b.Linear = r.Floats(m.LinearLanes, At("bodies", i, "linearVelocity"));
            b.Angular = r.Floats(m.AngularLanes, At("bodies", i, "angularVelocity"));
            b.Extra = r.Floats(extra, At("bodies", i, "extra"));
            if (b.Extra[0] < 0f || b.Extra[1] < 0f) { throw r.Bad(At("bodies", i, "extra"), "damping must not be negative"); }
            b.HasControls = r.Bool(At("bodies", i, "hasControls"));
            if (b.HasControls) { b.Controls = ReadControls(r, At("bodies", i, "controls"), m, b); }
            b.HasSleep = r.Bool(At("bodies", i, "hasSleep"));
            if (b.HasSleep)
            {
                float lin = r.Float(At("bodies", i, "sleep.linear")), ang = r.Float(At("bodies", i, "sleep.angular")), t = r.Float(At("bodies", i, "sleep.time"));
                try { b.Sleep = new BodySleepSettings(lin, ang, t); }
                catch (ArgumentOutOfRangeException e) { throw r.Bad(At("bodies", i, "sleep"), e.Message); }
                if (b.Type != (byte)BodyType.Dynamic) { throw r.Bad(At("bodies", i, "sleep"), "sleep settings need a dynamic body"); }
            }
        }

        m.Shapes = new PacketShape[nShapes];
        for (int i = 0; i < nShapes; i++)
        {
            var s = new PacketShape();
            m.Shapes[i] = s;
            s.Slot = r.Int(At("shapes", i, "slot")); s.Generation = r.Int(At("shapes", i, "generation"));
            if (s.Slot < 0 || s.Generation <= 0) { throw r.Bad(At("shapes", i, "slot"), "slot and generation must be positive handles of the source world"); }
            if (i > 0 && s.Slot <= m.Shapes[i - 1].Slot) { throw r.Bad(At("shapes", i, "slot"), "shapes must be in ascending slot order"); }
            s.Body = r.Int(At("shapes", i, "body"));
            if (s.Body < 0 || s.Body >= members) { throw r.Bad(At("shapes", i, "body"), "must name one of the " + members.ToString(CultureInfo.InvariantCulture) + " set bodies (an anchor has no shapes)"); }
            int len = r.Int(At("shapes", i, "blobLength"));
            if (len < 0 || len > r.Remaining) { throw r.Bad(At("shapes", i, "blobLength"), "length " + len.ToString(CultureInfo.InvariantCulture) + " exceeds the remaining bytes"); }
            int at = r.Pos;
            s.Blob = r.Bytes(len, At("shapes", i, "blob"));
            try { PacketBlobs.ValidateShape(dim, s.Blob, m.Bodies[s.Body].Type); }
            catch (BodySetPacketException e) { throw new BodySetPacketException(At("shapes", i, "blob"), at, e.Message); }
        }

        m.Joints = new PacketJoint[nJoints];
        for (int i = 0; i < nJoints; i++)
        {
            var j = new PacketJoint();
            m.Joints[i] = j;
            j.Slot = r.Int(At("joints", i, "slot")); j.Generation = r.Int(At("joints", i, "generation"));
            if (j.Slot < 0 || j.Generation <= 0) { throw r.Bad(At("joints", i, "slot"), "slot and generation must be positive handles of the source world"); }
            if (i > 0 && j.Slot <= m.Joints[i - 1].Slot) { throw r.Bad(At("joints", i, "slot"), "joints must be in ascending slot order"); }
            j.BodyA = r.Int(At("joints", i, "bodyA")); j.BodyB = r.Int(At("joints", i, "bodyB"));
            if (j.BodyA < 0 || j.BodyA >= m.Bodies.Length) { throw r.Bad(At("joints", i, "bodyA"), "dangling body index " + j.BodyA.ToString(CultureInfo.InvariantCulture)); }
            if (j.BodyB < 0 || j.BodyB >= m.Bodies.Length) { throw r.Bad(At("joints", i, "bodyB"), "dangling body index " + j.BodyB.ToString(CultureInfo.InvariantCulture)); }
            if (j.BodyA == j.BodyB) { throw r.Bad(At("joints", i, "bodyB"), "a joint needs two different bodies"); }
            if (m.Bodies[j.BodyA].Type != (byte)BodyType.Dynamic && m.Bodies[j.BodyB].Type != (byte)BodyType.Dynamic) { throw r.Bad(At("joints", i, "bodyA"), "a joint needs a dynamic body"); }
            j.Flags = r.Byte(At("joints", i, "flags"));
            if ((j.Flags & ~(byte)(ConstraintBits.ReportPersist | ConstraintBits.HasLifetime)) != 0) { r.Pos--; throw r.Bad(At("joints", i, "flags"), "unknown constraint bits"); }
            j.Color = ReadColor(r, At("joints", i, "color"));
            j.BreakValue = r.Float(At("joints", i, "breakValue"));
            int len = r.Int(At("joints", i, "blobLength"));
            if (len < 0 || len > r.Remaining) { throw r.Bad(At("joints", i, "blobLength"), "length " + len.ToString(CultureInfo.InvariantCulture) + " exceeds the remaining bytes"); }
            int at = r.Pos;
            j.Blob = r.Bytes(len, At("joints", i, "blob"));
            try { PacketBlobs.ValidateJoint(dim, j.Blob); }
            catch (BodySetPacketException e) { throw new BodySetPacketException(At("joints", i, "blob"), at, e.Message); }
            j.Payload = r.Floats(m.PayloadFloats, At("joints", i, "payload"));
        }

        m.Contacts = new PacketContact[nContacts];
        int maxPoints = m.MaxManifoldPoints;
        for (int i = 0; i < nContacts; i++)
        {
            var c = new PacketContact();
            m.Contacts[i] = c;
            c.Slot = r.Int(At("contacts", i, "slot")); c.Generation = r.Int(At("contacts", i, "generation"));
            if (c.Slot < 0 || c.Generation <= 0) { throw r.Bad(At("contacts", i, "slot"), "slot and generation must be positive handles of the source world"); }
            c.ShapeA = r.Int(At("contacts", i, "shapeA")); c.ShapeB = r.Int(At("contacts", i, "shapeB"));
            if (c.ShapeA < 0 || c.ShapeA >= nShapes) { throw r.Bad(At("contacts", i, "shapeA"), "dangling shape index " + c.ShapeA.ToString(CultureInfo.InvariantCulture)); }
            if (c.ShapeB < 0 || c.ShapeB >= nShapes) { throw r.Bad(At("contacts", i, "shapeB"), "dangling shape index " + c.ShapeB.ToString(CultureInfo.InvariantCulture)); }
            if (c.ShapeA >= c.ShapeB) { throw r.Bad(At("contacts", i, "shapeB"), "shape A must come before shape B"); }
            if (m.Shapes[c.ShapeA].Body == m.Shapes[c.ShapeB].Body) { throw r.Bad(At("contacts", i, "shapeB"), "a contact needs two different bodies"); }
            c.SubA = r.Int(At("contacts", i, "subA")); c.SubB = r.Int(At("contacts", i, "subB"));
            if (c.SubA < 0 || c.SubB < 0) { throw r.Bad(At("contacts", i, "subA"), "sub-shape ids must not be negative"); }
            c.Flags = r.Byte(At("contacts", i, "flags"));
            if ((c.Flags & ~(byte)ConstraintBits.ReportPersist) != 0) { r.Pos--; throw r.Bad(At("contacts", i, "flags"), "unknown constraint bits"); }
            c.Color = ReadColor(r, At("contacts", i, "color"));
            c.Points = r.Byte(At("contacts", i, "points"));
            if (c.Points > maxPoints) { r.Pos--; throw r.Bad(At("contacts", i, "points"), "at most " + maxPoints.ToString(CultureInfo.InvariantCulture) + " manifold points"); }
            c.NewPair = r.Bool(At("contacts", i, "newPair"));
            c.Features = new uint[maxPoints];
            for (int k = 0; k < maxPoints; k++) { c.Features[k] = r.UInt(At("contacts", i, "features")); }
            c.BreakValue = r.Float(At("contacts", i, "breakValue"));
            c.Payload = r.Floats(m.PayloadFloats, At("contacts", i, "payload"));
            bool samePair = i > 0 && m.Contacts[i - 1].ShapeA == c.ShapeA && m.Contacts[i - 1].ShapeB == c.ShapeB;
            if (c.NewPair == samePair) { throw r.Bad(At("contacts", i, "newPair"), samePair ? "a contact of the previous pair must not start a pair" : "the first contact of a shape pair must start a pair"); }
            if (c.NewPair)
            {
                // A pair appears once; pairs ascend by shape A then shape B, so a repeat or a reordering is caught by comparing with the previous pair.
                if (i > 0 && (m.Contacts[i - 1].ShapeA > c.ShapeA || (m.Contacts[i - 1].ShapeA == c.ShapeA && m.Contacts[i - 1].ShapeB > c.ShapeB))) { throw r.Bad(At("contacts", i, "shapeA"), "pairs must be in ascending order"); }
            }
            else if (m.Contacts[i - 1].SubA > c.SubA || (m.Contacts[i - 1].SubA == c.SubA && m.Contacts[i - 1].SubB >= c.SubB)) { throw r.Bad(At("contacts", i, "subA"), "the constraints of a pair must be in ascending sub-shape order"); }
        }

        m.Supports = new PacketSupport[nSupports];
        for (int i = 0; i < nSupports; i++)
        {
            var sp = new PacketSupport();
            m.Supports[i] = sp;
            sp.Position = r.Doubles(m.PositionLanes, At("supports", i, "position"));
            foreach (double p in sp.Position) { if (Math.Abs(p) > 8388608.0) { throw r.Bad(At("supports", i, "position"), "beyond +-2^23 m"); } }
            sp.Rotation = r.Floats(m.RotationLanes, At("supports", i, "rotation"));
            double q2 = 0; foreach (float q in sp.Rotation) { q2 += (double)q * q; }
            if (!(Math.Abs(q2 - 1.0) <= 1e-3)) { throw r.Bad(At("supports", i, "rotation"), "must be a unit rotation"); }
            int len = r.Int(At("supports", i, "blobLength"));
            if (len < 0 || len > r.Remaining) { throw r.Bad(At("supports", i, "blobLength"), "length " + len.ToString(CultureInfo.InvariantCulture) + " exceeds the remaining bytes"); }
            int at = r.Pos;
            sp.Blob = r.Bytes(len, At("supports", i, "blob"));
            try { PacketBlobs.ValidateShape(dim, sp.Blob, (byte)BodyType.Static); }
            catch (BodySetPacketException e) { throw new BodySetPacketException(At("supports", i, "blob"), at, e.Message); }
        }

        m.SupportContacts = new PacketSupportContact[nSupportContacts];
        for (int i = 0; i < nSupportContacts; i++)
        {
            var c = new PacketSupportContact();
            m.SupportContacts[i] = c;
            c.Member = r.Int(At("supportContacts", i, "member")); c.Support = r.Int(At("supportContacts", i, "support"));
            if (c.Member < 0 || c.Member >= nShapes) { throw r.Bad(At("supportContacts", i, "member"), "dangling shape index " + c.Member.ToString(CultureInfo.InvariantCulture)); }
            if (c.Support < 0 || c.Support >= nSupports) { throw r.Bad(At("supportContacts", i, "support"), "dangling support index " + c.Support.ToString(CultureInfo.InvariantCulture)); }
            c.MemberIsA = r.Bool(At("supportContacts", i, "memberIsA"));
            c.SubA = r.Int(At("supportContacts", i, "subA")); c.SubB = r.Int(At("supportContacts", i, "subB"));
            if (c.SubA < 0 || c.SubB < 0) { throw r.Bad(At("supportContacts", i, "subA"), "sub-shape ids must not be negative"); }
            c.Flags = r.Byte(At("supportContacts", i, "flags"));
            if ((c.Flags & ~(byte)ConstraintBits.ReportPersist) != 0) { r.Pos--; throw r.Bad(At("supportContacts", i, "flags"), "unknown constraint bits"); }
            c.Color = ReadColor(r, At("supportContacts", i, "color"));
            c.Points = r.Byte(At("supportContacts", i, "points"));
            if (c.Points > maxPoints) { r.Pos--; throw r.Bad(At("supportContacts", i, "points"), "at most " + maxPoints.ToString(CultureInfo.InvariantCulture) + " manifold points"); }
            c.NewPair = r.Bool(At("supportContacts", i, "newPair"));
            c.Features = new uint[maxPoints];
            for (int k = 0; k < maxPoints; k++) { c.Features[k] = r.UInt(At("supportContacts", i, "features")); }
            c.BreakValue = r.Float(At("supportContacts", i, "breakValue"));
            c.Payload = r.Floats(m.PayloadFloats, At("supportContacts", i, "payload"));
            bool same = i > 0 && m.SupportContacts[i - 1].Member == c.Member && m.SupportContacts[i - 1].Support == c.Support;
            if (c.NewPair == same) { throw r.Bad(At("supportContacts", i, "newPair"), same ? "a contact of the previous pair must not start a pair" : "the first contact of a shape pair must start a pair"); }
            if (same && m.SupportContacts[i - 1].MemberIsA != c.MemberIsA) { throw r.Bad(At("supportContacts", i, "memberIsA"), "the contacts of one pair share their orientation"); }
        }

        m.Ragdolls = new PacketRagdoll[nRagdolls];
        var used = new bool[members];
        for (int i = 0; i < nRagdolls; i++)
        {
            var rd = new PacketRagdoll();
            m.Ragdolls[i] = rd;
            rd.Slot = r.Int(At("ragdolls", i, "slot")); rd.Generation = r.Int(At("ragdolls", i, "generation"));
            if (rd.Slot < 0 || rd.Generation <= 0) { throw r.Bad(At("ragdolls", i, "slot"), "slot and generation must be positive handles of the source world"); }
            if (i > 0 && rd.Slot <= m.Ragdolls[i - 1].Slot) { throw r.Bad(At("ragdolls", i, "slot"), "ragdolls must be in ascending slot order"); }
            int len = r.Int(At("ragdolls", i, "blobLength"));
            if (len < 0 || len > r.Remaining) { throw r.Bad(At("ragdolls", i, "blobLength"), "length " + len.ToString(CultureInfo.InvariantCulture) + " exceeds the remaining bytes"); }
            int at = r.Pos;
            rd.Blob = r.Bytes(len, At("ragdolls", i, "blob"));
            try { PacketBlobs.ValidateRagdoll(dim, rd.Blob, m, used); }
            catch (BodySetPacketException e) { throw new BodySetPacketException(At("ragdolls", i, "blob"), at, e.Message); }
        }

        if (r.Remaining != 0) { throw r.Bad("end", r.Remaining.ToString(CultureInfo.InvariantCulture) + " unexpected trailing bytes"); }
        return m;
    }

    private static byte ReadColor(Reader r, string field)
    {
        byte v = r.Byte(field);
        if (v >= ConstraintColoring.ColorCount && v != ConstraintStore.Overflow && v != ConstraintStore.Uncolored) { r.Pos--; throw r.Bad(field, "colour " + v.ToString(CultureInfo.InvariantCulture) + " is not 0..23, 254 or 255"); }
        return v;
    }

    private static BodyControlState ReadControls(Reader r, string name, PacketModel m, PacketBody body)
    {
        int constraints = r.Int(name + ".constraints");
        float lin = r.Float(name + ".maxLinearSpeed"), ang = r.Float(name + ".maxAngularSpeed"), push = r.Float(name + ".maxDepenetrationSpeed");
        BodyControlSettings settings;
        try { settings = new BodyControlSettings((RigidbodyConstraints)constraints, lin, ang, push); BodyControlStore.ValidateAxes(settings, m.Dimension); }
        catch (ArgumentOutOfRangeException e) { throw r.Bad(name, e.Message); }
        var s = new BodyControlState { Settings = settings };
        s.Overrides = r.Byte(name + ".overrides");
        s.Mass = r.Float(name + ".mass");
        s.Center = new Vector3F(r.Float(name + ".center"), r.Float(name + ".center"), r.Float(name + ".center"));
        s.Principal = new Vector3F(r.Float(name + ".principal"), r.Float(name + ".principal"), r.Float(name + ".principal"));
        s.InertiaRotation = new QuaternionF(r.Float(name + ".inertiaRotation"), r.Float(name + ".inertiaRotation"), r.Float(name + ".inertiaRotation"), r.Float(name + ".inertiaRotation"));
        s.TargetPosition = new Vector3D(r.Double(name + ".targetPosition"), r.Double(name + ".targetPosition"), r.Double(name + ".targetPosition"));
        s.TargetRotation = new QuaternionF(r.Float(name + ".targetRotation"), r.Float(name + ".targetRotation"), r.Float(name + ".targetRotation"), r.Float(name + ".targetRotation"));
        if ((s.Overrides & ~31) != 0 || !s.Present) { throw r.Bad(name + ".overrides", "invalid override flags"); }
        if ((s.Overrides & 7) != 0 && body.Type != (byte)BodyType.Dynamic) { throw r.Bad(name + ".overrides", "mass overrides need a dynamic body"); }
        if ((s.Overrides & 1) != 0 && !BodyControlStore.ValidMass(s.Mass)) { throw r.Bad(name + ".mass", "invalid mass override"); }
        if ((s.Overrides & 2) != 0 && (Math.Abs(s.Center.X) > 2097152 || Math.Abs(s.Center.Y) > 2097152 || Math.Abs(s.Center.Z) > 2097152)) { throw r.Bad(name + ".center", "invalid centre override"); }
        if ((s.Overrides & 4) != 0)
        {
            double n = (double)s.InertiaRotation.X * s.InertiaRotation.X + (double)s.InertiaRotation.Y * s.InertiaRotation.Y + (double)s.InertiaRotation.Z * s.InertiaRotation.Z + (double)s.InertiaRotation.W * s.InertiaRotation.W;
            if (!BodyControlStore.ValidMass(s.Principal.X) || (m.Dimension == 3 && (!BodyControlStore.ValidMass(s.Principal.Y) || !BodyControlStore.ValidMass(s.Principal.Z) || Math.Abs(n - 1) >= 1e-5)))
            { throw r.Bad(name + ".principal", "invalid inertia override"); }
        }

        if ((s.Overrides & 24) != 0 && body.Type != (byte)BodyType.Kinematic) { throw r.Bad(name + ".overrides", "a kinematic drive needs a kinematic body"); }
        if ((s.Overrides & 8) != 0)
        {
            double n = (double)s.TargetRotation.X * s.TargetRotation.X + (double)s.TargetRotation.Y * s.TargetRotation.Y + (double)s.TargetRotation.Z * s.TargetRotation.Z + (double)s.TargetRotation.W * s.TargetRotation.W;
            if (Math.Abs(n - 1) >= 1e-5) { throw r.Bad(name + ".targetRotation", "must be a unit rotation"); }
            if (Math.Abs(s.TargetPosition.X) > 8388608.0 || Math.Abs(s.TargetPosition.Y) > 8388608.0 || Math.Abs(s.TargetPosition.Z) > 8388608.0) { throw r.Bad(name + ".targetPosition", "beyond +-2^23 m"); }
        }

        if (m.Dimension == 2 && (s.Center.Z != 0 || s.Principal.Y != 0 || s.Principal.Z != 0 || ((s.Overrides & 8) != 0 && (s.TargetPosition.Z != 0 || s.TargetRotation.X != 0 || s.TargetRotation.Y != 0))))
        { throw r.Bad(name, "a 2D packet holds planar controls"); }
        return s;
    }
}

/// <summary>Per-dimension validation of the engine-specific blobs of a packet, reachable without a world.</summary>
internal static class PacketBlobs
{
    public static void ValidateShape(int dimension, byte[] blob, byte ownerType)
    {
        if (dimension == 3) { Physics3D.Engine3D.ValidateShapeBlob(blob, ownerType); } else { Physics2D.Engine2D.ValidateShapeBlob(blob); }
    }

    public static void ValidateRagdoll(int dimension, byte[] blob, PacketModel m, bool[] used)
    {
        if (dimension == 3) { Physics3D.Engine3D.ValidateRagdollBlob(blob, m, used); } else { Physics2D.Engine2D.ValidateRagdollBlob(blob, m, used); }
    }

    public static void ValidateJoint(int dimension, byte[] blob)
    {
        if (dimension == 3) { Physics3D.Engine3D.ValidateJointBlob(blob); } else { Physics2D.Engine2D.ValidateJointBlob(blob); }
    }
}
