using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

/// <summary>Sensors against kinematic bodies and characters.</summary>
public class KinematicSensor2DTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld2D NewWorld(int threads = 1) => new(new WorldOptions2D { Gravity = new Vector2F(0, 0), ThreadCount = threads, WorldId = 3 });

    private static Body SensorBody(PhysicsWorld2D w, double x, BodyType type)
    {
        var b = w.CreateBody(type, new Vector2D(x, 0));
        w.CreateBox(b, 1f, 1f, default, 0f, 0f, new ShapeDef { IsSensor = true });
        return b;
    }

    private static Body Kinematic(PhysicsWorld2D w, double x, float vx)
    {
        var b = w.CreateBody(BodyType.Kinematic, new Vector2D(x, 0));
        w.CreateCircle(b, 0.5f, default, new ShapeDef());
        b.LinearVelocity = new Vector2F(vx, 0);
        return b;
    }

    private static List<string> Run(PhysicsWorld2D w, Body a, Body b, int steps)
    {
        var log = new List<string>();
        a.OnTriggerEnter += t => log.Add("a-enter:" + t.SelfIsSensor);
        a.OnTriggerExit += t => log.Add("a-exit:" + t.SelfIsSensor);
        b.OnTriggerEnter += t => log.Add("b-enter:" + t.SelfIsSensor);
        b.OnTriggerExit += t => log.Add("b-exit:" + t.SelfIsSensor);
        for (int i = 0; i < steps; i++) { w.Step(Dt); }
        return log;
    }

    [Theory]
    [InlineData(1)]
    [InlineData(8)]
    public void AKinematicBodyPassingThroughAStaticSensorGivesEnterThenExit(int threads)
    {
        var w = NewWorld(threads);
        var sensor = SensorBody(w, 0, BodyType.Static);
        var k = Kinematic(w, -4, 3f);
        Assert.Equal(["a-enter:True", "b-enter:False", "a-exit:True", "b-exit:False"], Run(w, sensor, k, 240));
    }

    [Fact]
    public void AKinematicSensorMovingOverAStaticBodyGivesEnterThenExit()
    {
        var w = NewWorld();
        var other = w.CreateBody(BodyType.Static, new Vector2D(0, 0));
        w.CreateCircle(other, 0.5f, default, new ShapeDef());
        var sensor = SensorBody(w, -4, BodyType.Kinematic);
        sensor.LinearVelocity = new Vector2F(3f, 0);
        // Events are ordered by body slot: the static body was created first.
        Assert.Equal(["b-enter:False", "a-enter:True", "b-exit:False", "a-exit:True"], Run(w, sensor, other, 240));
    }

    [Fact]
    public void AKinematicSensorAndAKinematicBodyMeet()
    {
        var w = NewWorld();
        var sensor = SensorBody(w, 0, BodyType.Kinematic);
        var k = Kinematic(w, -4, 3f);
        Assert.Equal(["a-enter:True", "b-enter:False", "a-exit:True", "b-exit:False"], Run(w, sensor, k, 240));
    }

    [Fact]
    public void ACharacterWalkingThroughASensorGivesEnterThenExit()
    {
        var w = NewWorld();
        var sensor = SensorBody(w, 0, BodyType.Static);
        var c = w.AddCharacter(new Vector2D(-4, 0), Character.Humanoid);
        var log = new List<string>();
        sensor.OnTriggerEnter += t => log.Add("enter");
        sensor.OnTriggerExit += t => log.Add("exit");
        for (int i = 0; i < 240; i++) { c.Move(new Vector2F(3f * Dt, 0), Dt); w.Step(Dt); }
        Assert.Equal(["enter", "exit"], log);
    }
}
