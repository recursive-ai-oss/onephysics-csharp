using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

internal static class FuzzRailBike
{
    private static float F(byte[] input, int at) => at + 4 <= input.Length ? BitConverter.ToSingle(input, at) : 0.1f;

    // A value around a default: the raw float when the input asks for a hostile one (NaN, infinity, negative, huge), otherwise the default scaled by 0.25..4.
    private static float V(byte[] input, int at, float basis, bool hostile) => hostile ? F(input, at) : basis * (0.25f + (at < input.Length ? input[at] : 64) / 64f);

    private static TrackDefHolder Track(byte[] input, bool planar)
    {
        int n = planar ? 1 : 1 + input[1] % 6;
        var elements = new TrackElement[n];
        for (int i = 0; i < n; i++)
        {
            int o = 8 + 10 * i; byte k = o < input.Length ? input[o] : (byte)0;
            float len = 5 + Math.Abs(F(input, o + 1)) % 400;
            if ((k & 0x80) != 0) { len = F(input, o + 1); }
            elements[i] = planar || k % 3 == 0 ? TrackElement.Straight(len)
                : k % 3 == 1 ? TrackElement.Arc(len, (k & 0x40) != 0 ? F(input, o + 5) : 20 + Math.Abs(F(input, o + 5)) % 500)
                : TrackElement.Transition(len, (k & 0x40) != 0 ? F(input, o + 5) : 0f, 1f / (20 + Math.Abs(F(input, o + 5)) % 500));
        }

        var def = new TrackDef { Elements = elements, Gauge = (input[0] & 0x20) != 0 ? F(input, 4) : 1.435f, Friction = (input[0] & 0x40) != 0 ? F(input, 4) : 0.35f, StartHeading = (input[0] & 0x80) != 0 ? F(input, 4) : 0f };
        if ((input[2] & 1) != 0) { def.Elevation = new[] { new Vector2F(0, 0), new Vector2F(F(input, 8) % 100, F(input, 12) % 20), new Vector2F(F(input, 16), F(input, 20)) }; }
        if ((input[2] & 2) != 0) { def.Cant = new[] { new Vector2F(0, 0), new Vector2F(F(input, 12) % 100, F(input, 16) % 0.2f) }; }
        if ((input[2] & 4) != 0) { def.Start = new Vector3D(F(input, 20), F(input, 24), F(input, 28)); }
        return new TrackDefHolder(def);
    }

    private readonly struct TrackDefHolder
    {
        internal readonly TrackDef Def;
        internal TrackDefHolder(TrackDef d) { Def = d; }
    }

    // Track, rail vehicle and bicycle definitions built from hostile bytes: creation either fails with an ArgumentException (ArgumentOutOfRangeException included) or produces objects that step and snapshot
    // to the same bytes after a restore.
    internal static void Definition(byte[] input)
    {
        if (input.Length < 12) return;
        bool planar = (input[0] & 1) == 0, hostile = (input[0] & 2) != 0;
        try
        {
            var track = Track(input, planar).Def;
            if (planar)
            {
                var w = new P2.PhysicsWorld2D(new() { WorldId = 42 });
                w.AddBox(new(400, 1), at: new(0, -0.5), type: BodyType.Static);
                var t = w.AddTrack(track);
                var rd = new P2.RailDef
                {
                    CarMass = V(input, 12, 30000f, hostile), CarSize = new Vector2F(V(input, 16, 12f, hostile), V(input, 20, 3f, hostile)), Axles = hostile ? (sbyte)input[3] : 2 + input[3] % 4, AxleSpan = V(input, 24, 8f, hostile),
                    WheelRadius = V(input, 28, 0.46f, hostile), MaxTractionForce = V(input, 32, 1e5f, hostile), MaxBrakeForce = V(input, 36, 1e5f, hostile), CouplerSlack = V(input, 40, 0.05f, hostile), BufferStiffness = V(input, 44, 4e6f, hostile), DrivenWheelsets = input[4],
                };
                var v = w.AddRailVehicle(t, F(input, 48) % 50, rd, F(input, 52) % 20);
                var v2 = w.AddRailVehicle(t, 5, rd);
                v.Couple(v2, thisFront: (input[5] & 1) != 0, otherFront: (input[5] & 2) != 0);
                var bd = new P2.BicycleDef { WheelBase = V(input, 56, 1.02f, hostile), Mass = V(input, 60, 94f, hostile), CenterX = V(input, 64, 0.34f, hostile), CenterHeight = V(input, 68, 0.86f, hostile), TyreFriction = V(input, 72, 1f, hostile), PitchInertia = V(input, 76, 16f, hostile) };
                var b = w.AddBicycle(new Vector2D(-40, 0), bd, F(input, 80) % 15);
                for (int i = 0; i < 4; i++) { v.Drive(throttle: Math.Abs(F(input, 84)) % 1, brake: Math.Abs(F(input, 88)) % 1); b.Ride(pedal: Math.Abs(F(input, 92)) % 1, frontBrake: Math.Abs(F(input, 96)) % 1, lean: F(input, 100) % 1); w.Step(1f / 60); }
                var bytes = w.Snapshot(); w.Restore(bytes);
                if (!bytes.AsSpan().SequenceEqual(w.Snapshot())) throw new InvalidOperationException("Rail and bicycle round trip changed bytes (2D).");
            }
            else
            {
                var w = new P3.PhysicsWorld3D(new() { WorldId = 42 });
                w.AddBox(new(400, 1, 400), at: new(0, -0.5, 0), type: BodyType.Static);
                var t = w.AddTrack(track);
                var rd = new P3.RailDef
                {
                    CarMass = V(input, 12, 30000f, hostile), CarSize = new Vector3F(V(input, 16, 2.8f, hostile), V(input, 20, 3f, hostile), V(input, 24, 15f, hostile)), Bogies = hostile ? (sbyte)input[3] : input[3] % 3, AxlesPerBogie = hostile ? (sbyte)input[4] : 1 + input[4] % 3,
                    WheelRadius = V(input, 28, 0.46f, hostile), TreadSpan = V(input, 32, 1.5f, hostile), Conicity = V(input, 36, 0.05f, hostile), FlangeClearance = V(input, 40, 0.007f, hostile), WheelsetMass = V(input, 44, 1500f, hostile),
                    MaxTractionForce = V(input, 48, 1e5f, hostile), MaxBrakeForce = V(input, 52, 1e5f, hostile), CouplerSlack = V(input, 56, 0.05f, hostile), DrivenWheelsets = input[5],
                };
                var v = w.AddRailVehicle(t, Math.Abs(F(input, 60)) % 80, rd, F(input, 64) % 20);
                var v2 = w.AddRailVehicle(t, 5, rd);
                v.Couple(v2, thisFront: (input[6] & 1) != 0, otherFront: (input[6] & 2) != 0);
                var bd = P3.BicycleDef.Road;
                bd.WheelBase = V(input, 68, 1.02f, hostile); bd.Trail = V(input, 72, 0.08f, hostile); bd.HeadAngle = V(input, 76, 0.31f, hostile); bd.TyreFriction = V(input, 80, 1f, hostile); bd.TyreHertz = V(input, 84, 15f, hostile);
                bd.RearFrame.Mass = V(input, 88, 87f, hostile); bd.Front.Mass = V(input, 92, 2f, hostile); bd.RearWheel.Radius = V(input, 96, 0.3f, hostile);
                var b = w.AddBicycle(new Vector3D(-40, 0, -40), bd, F(input, 100) % 15);
                for (int i = 0; i < 4; i++) { v.Drive(throttle: Math.Abs(F(input, 104)) % 1, brake: Math.Abs(F(input, 108)) % 1); b.Ride(steer: F(input, 112) % 1, pedal: Math.Abs(F(input, 116)) % 1, frontBrake: Math.Abs(F(input, 120)) % 1, lean: F(input, 124) % 1); w.Step(1f / 60); }
                var bytes = w.Snapshot(); w.Restore(bytes);
                if (!bytes.AsSpan().SequenceEqual(w.Snapshot())) throw new InvalidOperationException("Rail and bicycle round trip changed bytes (3D).");
            }
        }
        catch (ArgumentException) { }
        catch (PhysicsException) { }
    }

    private static byte[]? s_planar, s_spatial;
    private static byte[] Baseline(bool planar)
    {
        if (planar)
        {
            if (s_planar != null) return s_planar;
            var w = new P2.PhysicsWorld2D(new() { WorldId = 42 });
            w.AddBox(new(400, 1), at: new(0, -0.5), type: BodyType.Static);
            var def = TrackDef.Straight(300); def.Elevation = new[] { new Vector2F(0, 0), new Vector2F(50, 0), new Vector2F(100, 3), new Vector2F(300, 3) };
            var t = w.AddTrack(def);
            var a = w.AddRailVehicle(t, 30, P2.RailDef.Locomotive, 5f); var c = w.AddRailVehicle(t, 30 - 9.25 - 0.6 - 0.02 - 5.25, P2.RailDef.Freight, 5f);
            a.Couple(c, thisFront: false, otherFront: true); a.Drive(throttle: 0.4f);
            var b = w.AddBicycle(new Vector2D(-40, 0), P2.BicycleDef.Road, 6f); b.Ride(pedal: 0.3f);
            for (int i = 0; i < 4; i++) { w.Step(1f / 60); }
            return s_planar = w.Snapshot();
        }

        if (s_spatial != null) return s_spatial;
        var world = new P3.PhysicsWorld3D(new() { WorldId = 42 });
        world.AddBox(new(400, 1, 400), at: new(0, -0.5, 0), type: BodyType.Static);
        var main = world.AddTrack(TrackDef.Curve(400, 100, 40, 0.1f, 60));
        main.AddSwitch(40, new TrackDef { Elements = new[] { TrackElement.Transition(40, 0f, 1f / 300), TrackElement.Arc(60, 300), TrackElement.Straight(100) } }, false);
        var loco = world.AddRailVehicle(main, 40, P3.RailDef.Locomotive, 8f); var tram = world.AddRailVehicle(main, 40 - 9.25 - 0.6 - 0.02 - 6.25, P3.RailDef.Tram, 8f);
        loco.Couple(tram, thisFront: false, otherFront: true); loco.Drive(throttle: 0.4f);
        var bike = world.AddBicycle(new Vector3D(20, 0, -40), P3.BicycleDef.Benchmark, 5f); bike.Ride(pedal: 0.3f);
        for (int i = 0; i < 4; i++) { world.Step(1f / 60); }
        return s_spatial = world.Snapshot();
    }

    // Damaged snapshots with track, rail vehicle and bicycle state: restore either rejects them with a SnapshotFormatException leaving the world unchanged, or accepts them and writes the same bytes back.
    internal static void Snapshot(byte[] input)
    {
        bool planar = input.Length == 0 || (input[0] & 1) == 0;
        var bytes = (byte[])Baseline(planar).Clone();
        for (int i = 1; i + 2 < input.Length; i += 3)
        {
            int offset = bytes.Length - 1 - (((input[i] << 8) | input[i + 1]) % Math.Min(2500, bytes.Length)); bytes[offset] ^= input[i + 2];
        }

        if (input.Length > 0 && (input[0] & 15) == 15) Array.Resize(ref bytes, input[^1] * bytes.Length / 256);
        if (planar)
        {
            var world = new P2.PhysicsWorld2D(new() { WorldId = 42 }); var before = world.Snapshot();
            try { world.Restore(bytes); }
            catch (SnapshotFormatException) { if (!before.AsSpan().SequenceEqual(world.Snapshot())) throw new InvalidOperationException("Rejected rail snapshot changed world."); return; }
            if (!bytes.AsSpan().SequenceEqual(world.Snapshot())) throw new InvalidOperationException("Accepted rail snapshot changed bytes.");
            for (int i = 0; i < 2; i++) { try { world.Step(1f / 60); } catch (PhysicsException) { return; } }
        }
        else
        {
            var world = new P3.PhysicsWorld3D(new() { WorldId = 42 }); var before = world.Snapshot();
            try { world.Restore(bytes); }
            catch (SnapshotFormatException) { if (!before.AsSpan().SequenceEqual(world.Snapshot())) throw new InvalidOperationException("Rejected rail snapshot changed world."); return; }
            if (!bytes.AsSpan().SequenceEqual(world.Snapshot())) throw new InvalidOperationException("Accepted rail snapshot changed bytes.");
            for (int i = 0; i < 2; i++) { try { world.Step(1f / 60); } catch (PhysicsException) { return; } }
        }
    }
}
