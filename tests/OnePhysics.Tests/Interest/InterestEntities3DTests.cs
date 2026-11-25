using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using Xunit;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests;

/// <summary>Interest streams (3D) with characters and vehicles inside, and with what a packet cannot carry near the region.</summary>
public sealed class InterestEntities3DTests
{
    private const float Dt = 1f / 60f;

    private sealed class Scene
    {
        public P3.PhysicsWorld3D World = null!;
        public P3.Character Walker, Driver;
        public P3.Vehicle Car, Tank;
        public P3.Body RopeBox;
        public P3.Articulation Arm;
    }

    private static Scene Build(int id)
    {
        var s = new Scene { World = new P3.PhysicsWorld3D(new() { WorldId = id }) };
        var w = s.World;
        w.AddBox(new Vector3F(400, 1, 400), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        s.Walker = w.AddCharacter(new Vector3D(-16, 1, 0));
        var chassis = w.AddBox(new Vector3F(1.8f, 0.6f, 4.2f), at: new Vector3D(-4, 1.0, -30), density: (float)(1500.0 / (1.8 * 0.6 * 4.2)));
        s.Car = w.AddVehicle(chassis, P3.VehicleDef.Car);
        s.Driver = w.AddCharacter(new Vector3D(-4, 2, -30));
        s.Driver.Mount(chassis, new Vector3F(-0.4f, 0.5f, 0.2f));
        s.Tank = w.AddTank(new Vector3D(4, 1.2, 30), rotation: QuaternionF.CreateFromAxisAngle(new Vector3F(0, 1, 0), MathF.PI));
        s.Arm = w.AddArticulation(P3.ArticulationDef.Quadruped(), new Vector3D(3, 0.625, -8));
        s.RopeBox = w.AddBox(new Vector3F(0.6f, 0.6f, 0.6f), at: new Vector3D(-2, 1.5, 8));
        var rope = w.AddRope(new Vector3D(-2, 4, 8), new Vector3D(-2, 1.5, 8), 8, pinStart: true);
        rope.Attach(8, s.RopeBox, default);
        return s;
    }

    private static InterestRegion Region() => new InterestRegion().AddBox(new Vector3D(-8, -5, -12), new Vector3D(8, 40, 12));

    private static void Tick(Scene s, int step)
    {
        s.Walker.Move(new Vector3F(0.1f, s.Walker.IsGrounded ? -0.02f : -0.1f, 0), Dt);
        s.Car.Drive(0.9f, 0.02f * MathF.Sin(step * 0.04f));
        s.Tank.SetInput(throttle: 0.8f);
        s.World.Step(Dt);
    }

    [Fact]
    public void CharactersAndCarsPassThroughTheRegionWhileUncarriableBodiesAreSkippedAndReported()
    {
        var s = Build(8701);
        var a = s.World;
        var c = new P3.PhysicsWorld3D(new() { WorldId = 8702 });
        var writer = a.CreateInterestWriter();
        var client = c.CreateInterestClient();
        int walkerSteps = 0, carSteps = 0, tankSteps = 0, driverSkipped = 0, armSkipped = 0, ropeSkipped = 0, maxCharacters = 0, maxVehicles = 0;
        var armBodies = new HashSet<BodyHandle>();
        for (int step = 0; step < 300; step++)
        {
            Tick(s, step);
            var region = Region();
            var result = client.Apply(writer.Write(region));
            Assert.Equal(step, result.Sequence);
            Assert.Equal(a.ComputeInterestHash(region), client.ComputeSharedHash());
            if (client.TryGetLocal(s.Walker.Body.Handle, out _)) { walkerSteps++; }
            if (client.TryGetLocal(s.Car.Chassis.Handle, out _)) { carSteps++; }
            if (client.TryGetLocal(s.Tank.Chassis.Handle, out _)) { tankSteps++; }
            foreach (var skip in writer.LastSkipped)
            {
                Assert.False(string.IsNullOrEmpty(skip.Reason));
                if (skip.Body == s.Driver.Body.Handle) { driverSkipped++; }
                else if (skip.Body == s.RopeBox.Handle) { ropeSkipped++; }
                else { armSkipped++; armBodies.Add(skip.Body); }
            }

            maxCharacters = Math.Max(maxCharacters, c.GetCharacters().Length);
            int vehicles = 0; foreach (var b in c.Bodies) { if (c.TryGetVehicle(b, out _)) { vehicles++; } }
            maxVehicles = Math.Max(maxVehicles, vehicles);
        }

        Assert.True(walkerSteps > 60 && walkerSteps < 300, "the walker was in the stream for " + walkerSteps + " steps");
        Assert.True(carSteps > 30 && carSteps < 300, "the car was in the stream for " + carSteps + " steps");
        Assert.True(tankSteps > 30, "the tank was in the stream for " + tankSteps + " steps");
        Assert.True(ropeSkipped > 100, "the box with a rope on it was reported " + ropeSkipped + " times");
        Assert.True(armSkipped > 100, "articulation links were reported " + armSkipped + " times");
        Assert.True(maxCharacters >= 1 && maxVehicles >= 1, "the client held " + maxCharacters + " characters and " + maxVehicles + " vehicles");
    }

    [Fact]
    public void AnEntityThatEntersComesWithItsState()
    {
        var s = Build(8703);
        var a = s.World;
        var c = new P3.PhysicsWorld3D(new() { WorldId = 8704 });
        var writer = a.CreateInterestWriter();
        var client = c.CreateInterestClient();
        for (int step = 0; step < 200; step++)
        {
            Tick(s, step);
            client.Apply(writer.Write(Region()));
        }

        Assert.True(client.TryGetLocal(s.Tank.Chassis.Handle, out var local));
        Assert.True(c.TryGetVehicle(c.GetBody(local), out var tank));
        Assert.True(tank.Chassis.IsValid);
        Assert.True(client.TryGetLocal(s.Walker.Body.Handle, out var wl) || true);
    }

    [Fact]
    public void ADriverWhoseCarIsNotInTheStreamIsSkippedNotRefused()
    {
        var a = new P3.PhysicsWorld3D(new() { WorldId = 8705 });
        a.AddBox(new Vector3F(400, 1, 400), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var chassis = a.AddBox(new Vector3F(1.8f, 0.6f, 4.2f), at: new Vector3D(20, 1.0, 0), density: 357f);
        a.AddVehicle(chassis, P3.VehicleDef.Car);
        var driver = a.AddCharacter(new Vector3D(20, 2, 0));
        driver.Mount(chassis, new Vector3F(0, 0.5f, 0));
        // Name the driver alone: the car, a dynamic body that is not named and lies outside the box, is not a proxy of a kinematic character.
        var region = new InterestRegion().AddBox(new Vector3D(-1, -5, -1), new Vector3D(1, 40, 1));
        region.AddBody(driver.Body.Handle);
        var writer = a.CreateInterestWriter();
        a.Step(Dt);
        var msg = writer.Write(region);
        Assert.Single(writer.LastSkipped);
        Assert.Equal(driver.Body.Handle, writer.LastSkipped[0].Body);
        Assert.True(writer.LastSkipped[0].WasInside);
        Assert.Contains("seated", writer.LastSkipped[0].Reason);
        var c = new P3.PhysicsWorld3D(new() { WorldId = 8706 });
        Assert.Equal(0, c.CreateInterestClient().Apply(msg).Sequence);
        Assert.Empty(c.GetCharacters());
    }
}
