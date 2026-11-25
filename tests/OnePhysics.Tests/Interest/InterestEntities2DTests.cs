using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;

namespace RecursiveAI.OnePhysics.Tests;

/// <summary>Interest streams (2D) with characters and vehicles inside, and with what a packet cannot carry near the region.</summary>
public sealed class InterestEntities2DTests
{
    private const float Dt = 1f / 60f;

    private sealed class Scene
    {
        public P2.PhysicsWorld2D World = null!;
        public P2.Character Walker, Driver;
        public P2.Vehicle Car, Tank;
        public P2.Body RopeBox;
    }

    private static Scene Build(int id)
    {
        var s = new Scene { World = new P2.PhysicsWorld2D(new() { WorldId = id }) };
        var w = s.World;
        w.AddBox(new Vector2F(600, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
        s.Walker = w.AddCharacter(new Vector2D(-30, 1));
        s.Car = w.AddCar(new Vector2D(-20, 0));
        s.Driver = w.AddCharacter(new Vector2D(-20, 2));
        s.Driver.Mount(s.Car.Chassis, new Vector2F(0.2f, 1.4f));
        s.Tank = w.AddTank(new Vector2D(60, 0));
        w.AddArticulation(P2.ArticulationDef.Arm(0.9f, 3, 4f), new Vector2D(3, 0));
        s.RopeBox = w.AddBox(new Vector2F(0.6f, 0.6f), at: new Vector2D(-2, 1.5));
        w.AddRope(new Vector2D(-2, 4), new Vector2D(-2, 1.5), 8, pinStart: true).Attach(8, s.RopeBox, default);
        return s;
    }

    private static InterestRegion Region() => new InterestRegion().AddBox(new Vector2D(-12, -5), new Vector2D(12, 40));

    [Fact]
    public void CharactersAndCarsPassThroughTheRegionWhileUncarriableBodiesAreSkippedAndReported()
    {
        var s = Build(8711);
        var a = s.World;
        var c = new P2.PhysicsWorld2D(new() { WorldId = 8712 });
        var writer = a.CreateInterestWriter();
        var client = c.CreateInterestClient();
        int walkerSteps = 0, carSteps = 0, ropeSkipped = 0, armSkipped = 0, maxCharacters = 0, maxVehicles = 0;
        for (int step = 0; step < 300; step++)
        {
            s.Walker.Move(new Vector2F(0.1f, s.Walker.IsGrounded ? -0.02f : -0.1f), Dt);
            s.Car.Drive(0.9f);
            s.Tank.Drive(-0.8f);
            a.Step(Dt);
            var region = Region();
            var result = client.Apply(writer.Write(region));
            Assert.Equal(step, result.Sequence);
            Assert.Equal(a.ComputeInterestHash(region), client.ComputeSharedHash());
            if (client.TryGetLocal(s.Walker.Body.Handle, out _)) { walkerSteps++; }
            if (client.TryGetLocal(s.Car.Chassis.Handle, out _)) { carSteps++; }
            foreach (var skip in writer.LastSkipped) { if (skip.Body == s.RopeBox.Handle) { ropeSkipped++; } else if (skip.Body != s.Driver.Body.Handle) { armSkipped++; } }
            maxCharacters = Math.Max(maxCharacters, c.GetCharacters().Length);
            int vehicles = 0; foreach (var b in c.Bodies) { if (c.TryGetVehicle(b, out _)) { vehicles++; } }
            maxVehicles = Math.Max(maxVehicles, vehicles);
        }

        Assert.True(walkerSteps > 60 && walkerSteps < 300, "the walker was in the stream for " + walkerSteps + " steps");
        Assert.True(carSteps > 30, "the car was in the stream for " + carSteps + " steps");
        Assert.True(ropeSkipped > 100 && armSkipped > 100, "rope " + ropeSkipped + ", articulation " + armSkipped);
        Assert.True(maxCharacters >= 1 && maxVehicles >= 1);
    }
}
