using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Rods;
using System.IO;
using RecursiveAI.OnePhysics.Replay;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;

namespace RecursiveAI.OnePhysics.Tests;

/// <summary>Characters and vehicles in body-set packets (2D): they arrive whole, and what they do next equals what they would have done in the world they left.</summary>
public sealed class EntityTransfer2DTests
{
    private const float Dt = 1f / 60f;

    private static P2.PhysicsWorld2D World(int id, bool slope = false)
    {
        var w = new P2.PhysicsWorld2D(new() { WorldId = id });
        w.AddBox(new Vector2F(600, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
        if (slope) { w.AddBox(new Vector2F(40, 0.4f), at: new Vector2D(20, 1.5), angleRadians: 0.1f, type: BodyType.Static); }
        return w;
    }

    private static double Gap(P2.Body a, P2.Body b, float turn = 0f, Vector2D at = default)
    {
        var p = a.Position;
        double c = Math.Cos(turn), s = Math.Sin(turn);
        double x = c * p.X - s * p.Y + at.X, y = s * p.X + c * p.Y + at.Y;
        var q = b.Position;
        var va = a.LinearVelocity; var vb = b.LinearVelocity;
        double vx = c * va.X - s * va.Y, vy = s * va.X + c * va.Y;
        double dv = Math.Sqrt((vx - vb.X) * (vx - vb.X) + (vy - vb.Y) * (vy - vb.Y));
        return Math.Max(Math.Sqrt((x - q.X) * (x - q.X) + (y - q.Y) * (y - q.Y)), dv);
    }

    private static P2.Body Body(P2.PhysicsWorld2D w, BodySetHandleMap map, P2.Body old)
    {
        Assert.True(map.TryMap(old.Handle, out var h));
        return w.GetBody(h);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(37)]
    public void AWalkingCharacterContinuesLikeTheSource(int before)
    {
        var a = World(9401);
        var player = a.AddCharacter(new Vector2D(0, 1));
        void Tick(P2.PhysicsWorld2D w, P2.Character c, int i) { c.Move(new Vector2F(0.03f, c.IsGrounded ? -0.02f : -0.1f), Dt); w.Step(Dt); }
        for (int i = 0; i < 30 + before; i++) { Tick(a, player, i); }
        var packet = a.ExtractBodies(new[] { player.Body }, new BodySetOptions { Move = false });
        Assert.Equal(1, packet.CharacterCount);
        var b = World(9402);
        var map = b.InsertBodies(packet);
        var copy = b.GetCharacter(map.NewCharacters[0]);
        double worst = 0;
        for (int i = 0; i < 120; i++)
        {
            Tick(a, player, 100 + i); Tick(b, copy, 100 + i);
            worst = Math.Max(worst, Gap(player.Body, copy.Body));
            Assert.Equal(player.Velocity, copy.Velocity);
        }

        Assert.Equal(0.0, worst);
        a.ExtractBodies(new[] { player.Body });
        Assert.False(player.IsValid);
    }

    [Fact]
    public void ACharacterThroughAPortalMovesAndTurnsWithIt()
    {
        var a = World(9403);
        var player = a.AddCharacter(new Vector2D(2, 1));
        for (int i = 0; i < 40; i++) { player.Move(new Vector2F(0.03f, -0.02f), Dt); a.Step(Dt); }
        var packet = a.ExtractBodies(new[] { player.Body }, new BodySetOptions { Move = false });
        var b = World(9404);
        var map = b.InsertBodies(packet, new Vector2D(10, 0), 0f);
        var copy = b.GetCharacter(map.NewCharacters[0]);
        double worst = 0;
        for (int i = 0; i < 120; i++)
        {
            player.Move(new Vector2F(0.03f, -0.02f), Dt); a.Step(Dt);
            copy.Move(new Vector2F(0.03f, -0.02f), Dt); b.Step(Dt);
            worst = Math.Max(worst, Gap(player.Body, copy.Body, 0f, new Vector2D(10, 0)));
        }

        Assert.True(worst < 1e-6, "gap " + worst);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ACharacterOnAMovingPlatformKeepsRidingIt(bool exact)
    {
        var a = World(9405);
        var def = P2.CharacterDef.Humanoid; def.ExactPlatformCarry = exact;
        var platform = a.AddBox(new Vector2F(8, 0.5f), at: new Vector2D(0, 0.25), type: BodyType.Kinematic);
        var rider = a.AddCharacter(new Vector2D(1, 1.16), def);
        void Tick(P2.PhysicsWorld2D w, P2.Body p, P2.Character c, int i)
        {
            p.LinearVelocity = new Vector2F(0.5f * MathF.Sin(i * 0.05f), 0.2f * MathF.Cos(i * 0.03f));
            c.Move(new Vector2F(0.01f, c.IsGrounded ? -0.02f : -0.1f), Dt); w.Step(Dt);
        }

        for (int i = 0; i < 60; i++) { Tick(a, platform, rider, i); }
        Assert.True(rider.IsGrounded);
        var packet = a.ExtractBodies(new[] { platform, rider.Body }, new BodySetOptions { Move = false });
        var b = World(9406);
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
    public void ACharacterHoldingWhatIsNotInTheSetIsRefusedAndOneThatIsInItTravelsSeated()
    {
        var a = World(9407);
        var car = a.AddCar(new Vector2D(0, 0));
        var driver = a.AddCharacter(new Vector2D(0, 2));
        driver.Mount(car.Chassis, new Vector2F(0.2f, 1.4f));
        a.Step(Dt);
        ulong hash = a.ComputeHash();
        var ex = Assert.Throws<BodySetTransferException>(() => a.ExtractBodies(new[] { driver.Body }));
        Assert.Contains("seated", ex.Message);
        Assert.Equal(hash, a.ComputeHash());
        var packet = a.ExtractBodies(new[] { car.Chassis, driver.Body }, new BodySetOptions { Move = false });
        Assert.Equal(1, packet.CharacterCount); Assert.Equal(1, packet.VehicleCount);
        var b = World(9408);
        var map = b.InsertBodies(packet);
        Assert.Equal(CharacterMode.Seated, b.GetCharacter(map.NewCharacters[0]).Mode);
    }

    [Fact]
    public void ACharacterHoldingARopeIsRefused()
    {
        var a = World(9409);
        var pts = new Vector2D[31];
        for (int i = 0; i <= 30; i++) { pts[i] = new Vector2D(20, 6) + new Vector2D(Math.Sin(0.2) * 3, -Math.Cos(0.2) * 3) * (i / 30.0); }
        var rope = a.AddRod(new P2.RodDef { Points = pts, Material = new RodMaterial(2e10f, 8e9f, 0.003f, 1000f), PinnedParticles = [0] });
        var def = P2.CharacterDef.Humanoid; def.CanClimb = true;
        var roper = a.AddCharacter(new Vector2D(20, 3), def);
        roper.Grab(rope, 30f);
        a.Step(Dt);
        var ex = Assert.Throws<BodySetTransferException>(() => a.ExtractBodies(new[] { roper.Body }));
        Assert.Contains("rope", ex.Message);
    }

    public enum Kind { Car, Tank, Motorcycle }

    private static P2.Vehicle Add(P2.PhysicsWorld2D w, Kind kind) => kind switch { Kind.Tank => w.AddTank(new Vector2D(0, 0)), Kind.Motorcycle => w.AddMotorcycle(new Vector2D(0, 0)), _ => w.AddCar(new Vector2D(0, 0)) };

    private static void Drive(P2.Vehicle v, Kind kind, int i) => v.Drive(kind == Kind.Tank ? 0.8f : 0.5f, kind == Kind.Motorcycle ? 0.2f * MathF.Sin(i * 0.05f) : 0f, i % 90 > 80 ? 0.4f : 0f);

    [Theory]
    [InlineData(Kind.Car, false, 45)]
    [InlineData(Kind.Car, true, 120)]
    [InlineData(Kind.Tank, false, 40)]
    [InlineData(Kind.Motorcycle, false, 60)]
    public void ADrivingVehicleContinuesLikeTheSource(Kind kind, bool slope, int before)
    {
        var a = World(9410, slope);
        var car = Add(a, kind);
        for (int i = 0; i < before; i++) { Drive(car, kind, i); a.Step(Dt); }
        var packet = a.ExtractBodies(new[] { car.Chassis }, new BodySetOptions { Move = false });
        Assert.Equal(1, packet.VehicleCount);
        var b = World(9411, slope);
        var map = b.InsertBodies(packet);
        var copy = b.GetVehicle(map.NewVehicles[0]);
        double worst = 0;
        for (int i = 0; i < 120; i++)
        {
            Drive(car, kind, before + i); Drive(copy, kind, before + i);
            a.Step(Dt); b.Step(Dt);
            worst = Math.Max(worst, Gap(car.Chassis, copy.Chassis));
            Assert.Equal(car.Chassis.Angle, copy.Chassis.Angle);
        }

        Assert.Equal(0.0, worst);
        var end = copy.Chassis.Position;
        Assert.True(end.X > 1.0 && end.Y > -0.2 && copy.GetWheel(0).IsInContact, "ended at " + end);
    }

    [Fact]
    public void ACarThroughAPortalDrivesOnInTheNewFrame()
    {
        var a = World(9412);
        var car = Add(a, Kind.Car);
        for (int i = 0; i < 60; i++) { Drive(car, Kind.Car, i); a.Step(Dt); }
        var at = new Vector2D(20, 0);
        var packet = a.ExtractBodies(new[] { car.Chassis }, new BodySetOptions { Move = false });
        var b = World(9413);
        var map = b.InsertBodies(packet, at, 0f);
        var copy = b.GetVehicle(map.NewVehicles[0]);
        double worst = 0;
        for (int i = 0; i < 120; i++)
        {
            Drive(car, Kind.Car, 60 + i); Drive(copy, Kind.Car, 60 + i);
            a.Step(Dt); b.Step(Dt);
            worst = Math.Max(worst, Gap(car.Chassis, copy.Chassis, 0f, at));
        }

        Assert.True(worst < 1e-3, "gap " + worst);
    }

    [Fact]
    public void ADriverInACarCrossesWithIt()
    {
        var a = World(9414);
        var car = a.AddCar(new Vector2D(0, 0));
        var def = P2.CharacterDef.Humanoid; def.ExactPlatformCarry = true;
        var driver = a.AddCharacter(new Vector2D(0, 2), def);
        driver.Mount(car.Chassis, new Vector2F(0.2f, 1.4f));
        for (int i = 0; i < 50; i++) { Drive(car, Kind.Car, i); a.Step(Dt); }
        var packet = a.ExtractBodies(new[] { car.Chassis, driver.Body }, new BodySetOptions { Move = false });
        var b = World(9415);
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

    [Fact]
    public void PacketsWithEntitiesRoundTripAndTwoPeersStayEqual()
    {
        var a = World(9420);
        var car = a.AddCar(new Vector2D(0, 0));
        var tank = a.AddTank(new Vector2D(30, 0));
        var player = a.AddCharacter(new Vector2D(-5, 1));
        for (int i = 0; i < 40; i++) { Drive(car, Kind.Car, i); Drive(tank, Kind.Tank, i); player.Move(new Vector2F(0.02f, -0.02f), Dt); a.Step(Dt); }
        var packet = a.ExtractBodies(new[] { car.Chassis, tank.Chassis, player.Body }, new BodySetOptions { Move = false });
        Assert.Equal(2, packet.VehicleCount); Assert.Equal(1, packet.CharacterCount);
        var bytes = packet.ToArray();
        Assert.Equal(2u, BitConverter.ToUInt32(bytes, 4));
        var read = BodySetPacket.Read(bytes);
        Assert.Equal(bytes, read.ToArray());
        var p1 = World(9421); var p2 = World(9422);
        var m1 = p1.InsertBodies(packet, new Vector2D(3, 0), 0f); var m2 = p2.InsertBodies(read, new Vector2D(3, 0), 0f);
        Assert.Equal(p1.ComputeHash(), p2.ComputeHash());
        Assert.Equal(p1.Snapshot(), p2.Snapshot());
        var c1 = p1.GetCharacter(m1.NewCharacters[0]); var c2 = p2.GetCharacter(m2.NewCharacters[0]);
        for (int i = 0; i < 60; i++)
        {
            foreach (var (w, ch, m) in new[] { (p1, c1, m1), (p2, c2, m2) })
            {
                for (int k = 0; k < m.NewVehicles.Length; k++) { Drive(w.GetVehicle(m.NewVehicles[k]), k == 0 ? Kind.Car : Kind.Tank, 40 + i); }
                ch.Move(new Vector2F(0.02f, -0.02f), Dt); w.Step(Dt);
            }

            Assert.Equal(p1.ComputeHash(), p2.ComputeHash());
        }

        var fresh = new P2.PhysicsWorld2D(new() { WorldId = 9421 });
        fresh.Restore(p1.Snapshot());
        Assert.Equal(p1.ComputeHash(), fresh.ComputeHash());
    }

    [Fact]
    public void HostileEntityRecordsAreTypedErrors()
    {
        var a = World(9423);
        var car = a.AddCar(new Vector2D(0, 0));
        var player = a.AddCharacter(new Vector2D(-5, 1));
        a.Step(Dt);
        var bytes = a.ExtractBodies(new[] { car.Chassis, player.Body }, new BodySetOptions { Move = false }).ToArray();
        int bad = 0;
        for (int n = 0; n < bytes.Length; n += Math.Max(1, bytes.Length / 400))
        {
            try { BodySetPacket.Read(bytes.AsSpan(0, n)); } catch (BodySetPacketException) { bad++; }
        }

        for (int at = Math.Max(0, bytes.Length - 2000); at < bytes.Length; at++)
        {
            var copy = (byte[])bytes.Clone(); copy[at] ^= 0x7F;
            try { var p = BodySetPacket.Read(copy); var w = World(9424); w.InsertBodies(p); w.Step(Dt); } catch (Exception e) when (e is PhysicsException || e is ArgumentException) { bad++; }
        }

        Assert.True(bad > 100);
    }

    [Fact]
    public void AnInsertionOfEntitiesIsRecordedAndPlaysBack()
    {
        var a = World(9440);
        var car = a.AddCar(new Vector2D(0, 0));
        var player = a.AddCharacter(new Vector2D(-5, 1));
        for (int i = 0; i < 40; i++) { Drive(car, Kind.Car, i); player.Move(new Vector2F(0.02f, -0.02f), Dt); a.Step(Dt); }
        var packet = a.ExtractBodies(new[] { car.Chassis, player.Body }, new BodySetOptions { Move = false });
        var b = World(9441);
        using var stream = new MemoryStream();
        using (b.StartRecording(stream)) { b.InsertBodies(packet, new Vector2D(5, 0), 0f); for (int i = 0; i < 30; i++) { b.Step(Dt); } }
        stream.Position = 0;
        var replayer = new ReplayPlayer(stream);
        var c = World(9441);
        while (replayer.ReadNextStep(c)) { }
        Assert.Equal(b.Snapshot(), c.Snapshot());
    }
}
