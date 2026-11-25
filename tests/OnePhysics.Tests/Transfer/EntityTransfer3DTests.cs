using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Rods;
using System.IO;
using RecursiveAI.OnePhysics.Replay;
using Xunit;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests;

/// <summary>Characters and vehicles in body-set packets (3D): they arrive whole, and what they do next equals what they would have done in the world they left.</summary>
public sealed class EntityTransfer3DTests
{
    private const float Dt = 1f / 60f;

    private static P3.PhysicsWorld3D World(int id, bool ramp = false)
    {
        var w = new P3.PhysicsWorld3D(new() { WorldId = id });
        w.AddBox(new Vector3F(400, 1, 400), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        if (ramp) { w.AddBox(new Vector3F(8, 0.4f, 30), at: new Vector3D(0, 0.5, 14), rotation: QuaternionF.CreateFromAxisAngle(new Vector3F(1, 0, 0), -0.12f), type: BodyType.Static); }
        return w;
    }

    private static P3.Body Chassis(P3.PhysicsWorld3D w, Vector3D at) => w.AddBox(new Vector3F(1.8f, 0.6f, 4.2f), at: at, density: (float)(1500.0 / (1.8 * 0.6 * 4.2)));

    private static double Gap(P3.Body a, P3.Body b, QuaternionF? turn = null, Vector3D at = default)
    {
        var p = a.Position;
        if (turn is { } t) { var r = t.Rotate(new Vector3F((float)p.X, (float)p.Y, (float)p.Z)); p = new Vector3D(r.X + at.X, r.Y + at.Y, r.Z + at.Z); } else { p = new Vector3D(p.X + at.X, p.Y + at.Y, p.Z + at.Z); }
        var q = b.Position;
        var d = new Vector3D(p.X - q.X, p.Y - q.Y, p.Z - q.Z);
        double v = 0;
        var va = a.LinearVelocity; var vb = b.LinearVelocity;
        if (turn is { } t2) { va = t2.Rotate(va); }
        v = Math.Sqrt((double)(va.X - vb.X) * (va.X - vb.X) + (double)(va.Y - vb.Y) * (va.Y - vb.Y) + (double)(va.Z - vb.Z) * (va.Z - vb.Z));
        return Math.Max(Math.Sqrt(d.X * d.X + d.Y * d.Y + d.Z * d.Z), v);
    }

    private static P3.Body Body(P3.PhysicsWorld3D w, BodySetHandleMap map, P3.Body old)
    {
        Assert.True(map.TryMap(old.Handle, out var h));
        return w.GetBody(h);
    }

    // ------------------------------------------------------------------ characters

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(37)]
    public void AWalkingCharacterContinuesLikeTheSource(int before)
    {
        var a = World(9301);
        var player = a.AddCharacter(new Vector3D(0, 1, 0));
        void Tick(P3.PhysicsWorld3D w, P3.Character c, int i) { c.Move(new Vector3F(0.03f, c.IsGrounded ? -0.02f : -0.1f, 0.01f * MathF.Sin(i * 0.1f)), Dt); w.Step(Dt); }
        for (int i = 0; i < 30 + before; i++) { Tick(a, player, i); }
        var packet = a.ExtractBodies(new[] { player.Body }, new BodySetOptions { Move = false });
        Assert.Equal(1, packet.CharacterCount);
        var b = World(9302);
        var map = b.InsertBodies(packet);
        Assert.Single(map.NewCharacters.ToArray());
        var copy = b.GetCharacter(map.NewCharacters[0]);
        // The ground it stood on is not part of the set, so the character is in the air until its next move finds the ground again.
        Assert.False(copy.IsGrounded);
        double worst = 0;
        for (int i = 0; i < 120; i++)
        {
            Tick(a, player, 100 + i); Tick(b, copy, 100 + i);
            worst = Math.Max(worst, Gap(player.Body, copy.Body));
            Assert.Equal(player.Velocity, copy.Velocity);
        }

        Assert.Equal(0.0, worst);
        // The source is untouched by a copy; a move takes the character away.
        Assert.True(player.IsValid);
        var moved = a.ExtractBodies(new[] { player.Body });
        Assert.False(player.IsValid);
        Assert.Equal(1, moved.CharacterCount);
    }

    [Fact]
    public void ACharacterThroughAPortalMovesAndTurnsWithIt()
    {
        var a = World(9303);
        var player = a.AddCharacter(new Vector3D(2, 1, 3));
        var turn = QuaternionF.CreateFromAxisAngle(new Vector3F(0, 1, 0), MathF.PI / 2);
        var at = new Vector3D(10, 0, -5);
        for (int i = 0; i < 40; i++) { player.Move(new Vector3F(0.03f, -0.02f, 0.01f), Dt); a.Step(Dt); }
        var packet = a.ExtractBodies(new[] { player.Body }, new BodySetOptions { Move = false });
        var b = World(9304);
        var map = b.InsertBodies(packet, at, turn);
        var copy = b.GetCharacter(map.NewCharacters[0]);
        // The up direction is a turn about itself: it stays exactly as it was.
        Assert.Equal(player.Up, copy.Up);
        double worst = 0;
        for (int i = 0; i < 120; i++)
        {
            player.Move(new Vector3F(0.03f, -0.02f, 0.01f), Dt); a.Step(Dt);
            copy.Move(turn.Rotate(new Vector3F(0.03f, -0.02f, 0.01f)), Dt); b.Step(Dt);
            worst = Math.Max(worst, Gap(player.Body, copy.Body, turn, at));
        }

        Assert.True(worst < 2e-3, "gap " + worst);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ACharacterOnAMovingPlatformKeepsRidingIt(bool exact)
    {
        var a = World(9305);
        var def = P3.CharacterDef.Humanoid; def.ExactPlatformCarry = exact;
        var platform = a.AddBox(new Vector3F(8, 0.5f, 8), at: new Vector3D(0, 0.25, 0), type: BodyType.Kinematic);
        var rider = a.AddCharacter(new Vector3D(1, 1.16, 1), def);
        void Tick(P3.PhysicsWorld3D w, P3.Body p, P3.Character c, int i)
        {
            p.LinearVelocity = new Vector3F(0.5f * MathF.Sin(i * 0.05f), 0, 0.3f); p.AngularVelocity = new Vector3F(0, 0.3f, 0);
            c.Move(new Vector3F(0.01f, c.IsGrounded ? -0.02f : -0.1f, 0), Dt); w.Step(Dt);
        }

        for (int i = 0; i < 60; i++) { Tick(a, platform, rider, i); }
        Assert.True(rider.IsGrounded);
        var packet = a.ExtractBodies(new[] { platform, rider.Body }, new BodySetOptions { Move = false });
        var b = World(9306);
        var map = b.InsertBodies(packet);
        var p2 = Body(b, map, platform); var r2 = b.GetCharacter(map.NewCharacters[0]);
        Assert.True(r2.IsGrounded);
        Assert.Equal(p2.Handle, r2.GroundBody.Handle);
        double worst = 0;
        for (int i = 0; i < 120; i++)
        {
            Tick(a, platform, rider, 60 + i); Tick(b, p2, r2, 60 + i);
            worst = Math.Max(worst, Math.Max(Gap(platform, p2), Gap(rider.Body, r2.Body)));
        }

        Assert.Equal(0.0, worst);
    }

    [Fact]
    public void ARigidCharacterAndItsGroundArrive()
    {
        var a = World(9307);
        var player = a.AddCharacter(new Vector3D(0, 1, 0), P3.CharacterDef.RigidHumanoid);
        for (int i = 0; i < 40; i++) { player.Move(new Vector3F(0.03f, -0.02f, 0), Dt); a.Step(Dt); }
        var packet = a.ExtractBodies(new[] { player.Body }, new BodySetOptions { Move = false });
        var b = World(9308);
        var map = b.InsertBodies(packet);
        var copy = b.GetCharacter(map.NewCharacters[0]);
        double worst = 0;
        for (int i = 0; i < 120; i++)
        {
            player.Move(new Vector3F(0.03f, -0.02f, 0), Dt); a.Step(Dt);
            copy.Move(new Vector3F(0.03f, -0.02f, 0), Dt); b.Step(Dt);
            worst = Math.Max(worst, Gap(player.Body, copy.Body));
        }

        Assert.Equal(0.0, worst);
    }

    [Fact]
    public void ACharacterHoldingWhatIsNotInTheSetIsRefusedAndOneThatIsInItTravelsSeated()
    {
        var a = World(9309);
        var car = Chassis(a, new Vector3D(0, 1.1, 0));
        a.AddVehicle(car, P3.VehicleDef.Car);
        var driver = a.AddCharacter(new Vector3D(0, 2, 0));
        driver.Mount(car, new Vector3F(0.4f, 0.5f, 0.2f));
        a.Step(Dt);
        ulong hash = a.ComputeHash();
        var ex = Assert.Throws<BodySetTransferException>(() => a.ExtractBodies(new[] { driver.Body }));
        Assert.Contains("seated", ex.Message);
        Assert.Equal(hash, a.ComputeHash());
        var packet = a.ExtractBodies(new[] { car, driver.Body }, new BodySetOptions { Move = false });
        Assert.Equal(1, packet.CharacterCount); Assert.Equal(1, packet.VehicleCount);
        var b = World(9310);
        var map = b.InsertBodies(packet);
        var d2 = b.GetCharacter(map.NewCharacters[0]);
        Assert.Equal(CharacterMode.Seated, d2.Mode);
        Assert.Equal(Body(b, map, car).Handle, b.GetVehicle(map.NewVehicles[0]).Chassis.Handle);
    }

    [Fact]
    public void ACharacterHoldingARopeIsRefused()
    {
        var a = World(9311);
        var pts = new Vector3D[31];
        for (int i = 0; i <= 30; i++) { pts[i] = new Vector3D(20, 6, 0) + new Vector3D(Math.Sin(0.2) * 3, -Math.Cos(0.2) * 3, 0) * (i / 30.0); }
        var rope = a.AddRod(new P3.RodDef { Points = pts, Material = new RodMaterial(2e10f, 8e9f, 0.003f, 1000f), PinnedParticles = [0] });
        var def = P3.CharacterDef.Humanoid; def.CanClimb = true;
        var roper = a.AddCharacter(new Vector3D(20, 3, 0), def);
        roper.Grab(rope, 30f);
        a.Step(Dt);
        var ex = Assert.Throws<BodySetTransferException>(() => a.ExtractBodies(new[] { roper.Body }));
        Assert.Contains("rope", ex.Message);
    }

    // ------------------------------------------------------------------ vehicles

    public enum Kind { Car, Tank, Motorcycle }

    private static P3.Vehicle AddVehicle(P3.PhysicsWorld3D w, Kind kind, Vector3D at)
    {
        switch (kind)
        {
            case Kind.Tank: return w.AddTank(at);
            case Kind.Motorcycle: return w.AddMotorcycle(at);
            default: return w.AddVehicle(Chassis(w, at), P3.VehicleDef.Car);
        }
    }

    private static void Drive(P3.Vehicle v, Kind kind, int i)
    {
        switch (kind)
        {
            case Kind.Tank: v.SetInput(throttle: 0.7f, steer: 0.3f * MathF.Sin(i * 0.03f)); break;
            case Kind.Motorcycle: v.SetInput(throttle: 0.5f, steer: 0.1f * MathF.Sin(i * 0.03f)); break;
            default: v.Drive(0.5f, 0.3f * MathF.Sin(i * 0.03f), i % 90 > 80 ? 0.4f : 0f); break;
        }
    }

    [Theory]
    [InlineData(Kind.Car, false, 45)]
    [InlineData(Kind.Car, true, 80)]
    [InlineData(Kind.Tank, false, 40)]
    [InlineData(Kind.Motorcycle, false, 60)]
    public void ADrivingVehicleContinuesLikeTheSource(Kind kind, bool ramp, int before)
    {
        var a = World(9320, ramp);
        var car = AddVehicle(a, kind, new Vector3D(0, kind == Kind.Car ? 1.0 : 1.2, ramp ? 0 : 0));
        for (int i = 0; i < before; i++) { Drive(car, kind, i); a.Step(Dt); }
        var packet = a.ExtractBodies(new[] { car.Chassis }, new BodySetOptions { Move = false });
        Assert.Equal(1, packet.VehicleCount);
        var b = World(9321, ramp);
        var map = b.InsertBodies(packet);
        var copy = b.GetVehicle(map.NewVehicles[0]);
        double worst = 0, wheel = 0;
        for (int i = 0; i < 120; i++)
        {
            Drive(car, kind, before + i); Drive(copy, kind, before + i);
            a.Step(Dt); b.Step(Dt);
            worst = Math.Max(worst, Gap(car.Chassis, copy.Chassis));
            Assert.Equal(car.Chassis.Rotation, copy.Chassis.Rotation);
            Assert.Equal(car.Chassis.AngularVelocity, copy.Chassis.AngularVelocity);
        }

        Assert.Equal(0.0, worst + wheel);
        // It drove, and it is standing on its wheels (a sunken chassis would mean the wheels found no ground).
        var end = copy.Chassis.Position;
        Assert.True(Math.Abs(end.X) + Math.Abs(end.Z) > 2.0 && end.Y > 0.25 && copy.GetWheel(0).IsInContact, "ended at " + end);
    }

    [Fact]
    public void ACarThroughAPortalDrivesOnInTheNewFrame()
    {
        var a = World(9322);
        var car = AddVehicle(a, Kind.Car, new Vector3D(0, 1.0, 0));
        for (int i = 0; i < 60; i++) { Drive(car, Kind.Car, i); a.Step(Dt); }
        var turn = QuaternionF.CreateFromAxisAngle(new Vector3F(0, 1, 0), MathF.PI / 2);
        var at = new Vector3D(20, 0, -8);
        var packet = a.ExtractBodies(new[] { car.Chassis }, new BodySetOptions { Move = false });
        var b = World(9323);
        var map = b.InsertBodies(packet, at, turn);
        var copy = b.GetVehicle(map.NewVehicles[0]);
        double worst = 0;
        for (int i = 0; i < 120; i++)
        {
            Drive(car, Kind.Car, 60 + i); Drive(copy, Kind.Car, 60 + i);
            a.Step(Dt); b.Step(Dt);
            worst = Math.Max(worst, Gap(car.Chassis, copy.Chassis, turn, at));
        }

        Assert.True(worst < 0.05, "gap " + worst);
    }

    [Fact]
    public void ADriverInACarCrossesWithIt()
    {
        var a = World(9324);
        var car = AddVehicle(a, Kind.Car, new Vector3D(0, 1.0, 0));
        var def = P3.CharacterDef.Humanoid; def.ExactPlatformCarry = true;
        var driver = a.AddCharacter(new Vector3D(0, 2, 0), def);
        driver.Mount(car.Chassis, new Vector3F(-0.4f, 0.5f, 0.2f));
        for (int i = 0; i < 50; i++) { Drive(car, Kind.Car, i); a.Step(Dt); }
        var packet = a.ExtractBodies(new[] { car.Chassis, driver.Body }, new BodySetOptions { Move = false });
        var b = World(9325);
        var map = b.InsertBodies(packet);
        var car2 = b.GetVehicle(map.NewVehicles[0]); var d2 = b.GetCharacter(map.NewCharacters[0]);
        double worst = 0;
        for (int i = 0; i < 120; i++)
        {
            Drive(car, Kind.Car, 50 + i); Drive(car2, Kind.Car, 50 + i);
            a.Step(Dt); b.Step(Dt);
            worst = Math.Max(worst, Math.Max(Gap(car.Chassis, car2.Chassis), Gap(driver.Body, d2.Body)));
        }

        Assert.Equal(0.0, worst);
        Assert.Equal(CharacterMode.Seated, d2.Mode);
    }

    // ------------------------------------------------------------------ packets

    [Fact]
    public void PacketsWithEntitiesRoundTripAndTwoPeersStayEqual()
    {
        var a = World(9330);
        var car = AddVehicle(a, Kind.Car, new Vector3D(0, 1.0, 0));
        var tank = AddVehicle(a, Kind.Tank, new Vector3D(10, 1.2, 0));
        var player = a.AddCharacter(new Vector3D(-5, 1, 0));
        for (int i = 0; i < 40; i++) { Drive(car, Kind.Car, i); Drive(tank, Kind.Tank, i); player.Move(new Vector3F(0.02f, -0.02f, 0), Dt); a.Step(Dt); }
        var packet = a.ExtractBodies(new[] { car.Chassis, tank.Chassis, player.Body }, new BodySetOptions { Move = false });
        Assert.Equal(2, packet.VehicleCount); Assert.Equal(1, packet.CharacterCount);
        var bytes = packet.ToArray();
        Assert.Equal(2u, BitConverter.ToUInt32(bytes, 4));
        var read = BodySetPacket.Read(bytes);
        Assert.Equal(bytes, read.ToArray());
        var p1 = World(9331); var p2 = World(9332);
        var m1 = p1.InsertBodies(packet, new Vector3D(3, 0, 3)); var m2 = p2.InsertBodies(read, new Vector3D(3, 0, 3));
        Assert.Equal(p1.ComputeHash(), p2.ComputeHash());
        Assert.Equal(p1.Snapshot(), p2.Snapshot());
        var c1 = p1.GetCharacter(m1.NewCharacters[0]); var c2 = p2.GetCharacter(m2.NewCharacters[0]);
        for (int i = 0; i < 60; i++)
        {
            foreach (var (w, ch, m) in new[] { (p1, c1, m1), (p2, c2, m2) })
            {
                for (int k = 0; k < m.NewVehicles.Length; k++) { Drive(w.GetVehicle(m.NewVehicles[k]), k == 0 ? Kind.Car : Kind.Tank, 40 + i); }
                ch.Move(new Vector3F(0.02f, -0.02f, 0), Dt); w.Step(Dt);
            }

            Assert.Equal(p1.ComputeHash(), p2.ComputeHash());
        }

        // The same world after a snapshot round trip.
        var fresh = new P3.PhysicsWorld3D(new() { WorldId = 9331 });
        fresh.Restore(p1.Snapshot());
        Assert.Equal(p1.ComputeHash(), fresh.ComputeHash());
    }

    [Fact]
    public void WorldsWithoutEntitiesStillWriteVersionOnePackets()
    {
        var a = World(9333);
        var box = a.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 0.5, 0));
        var packet = a.ExtractBodies(new[] { box }, new BodySetOptions { Move = false });
        Assert.Equal(1u, BitConverter.ToUInt32(packet.ToArray(), 4));
    }

    [Fact]
    public void HostileEntityRecordsAreTypedErrors()
    {
        var a = World(9334);
        var car = AddVehicle(a, Kind.Car, new Vector3D(0, 1.0, 0));
        var player = a.AddCharacter(new Vector3D(-5, 1, 0));
        a.Step(Dt);
        var bytes = a.ExtractBodies(new[] { car.Chassis, player.Body }, new BodySetOptions { Move = false }).ToArray();
        int bad = 0;
        // Every truncation and a sweep of single-byte changes in the entity part either read cleanly or fail with the typed error.
        for (int n = 0; n < bytes.Length; n += Math.Max(1, bytes.Length / 400))
        {
            try { BodySetPacket.Read(bytes.AsSpan(0, n)); } catch (BodySetPacketException) { bad++; }
        }

        for (int at = Math.Max(0, bytes.Length - 3000); at < bytes.Length; at++)
        {
            var copy = (byte[])bytes.Clone(); copy[at] ^= 0x7F;
            try { var p = BodySetPacket.Read(copy); var w = World(9335); w.InsertBodies(p); w.Step(Dt); } catch (Exception e) when (e is PhysicsException || e is ArgumentException) { bad++; }
        }

        Assert.True(bad > 100);
    }

    [Fact]
    public void AnInsertionOfEntitiesIsRecordedAndPlaysBack()
    {
        var a = World(9340);
        var car = AddVehicle(a, Kind.Car, new Vector3D(0, 1.0, 0));
        var player = a.AddCharacter(new Vector3D(-5, 1, 0));
        for (int i = 0; i < 40; i++) { Drive(car, Kind.Car, i); player.Move(new Vector3F(0.02f, -0.02f, 0), Dt); a.Step(Dt); }
        var packet = a.ExtractBodies(new[] { car.Chassis, player.Body }, new BodySetOptions { Move = false });
        var b = World(9341);
        using var stream = new MemoryStream();
        using (b.StartRecording(stream)) { b.InsertBodies(packet, new Vector3D(5, 0, 0)); for (int i = 0; i < 30; i++) { b.Step(Dt); } }
        stream.Position = 0;
        var replayer = new ReplayPlayer(stream);
        var c = World(9341);
        while (replayer.ReadNextStep(c)) { }
        Assert.Equal(b.Snapshot(), c.Snapshot());
    }
}
