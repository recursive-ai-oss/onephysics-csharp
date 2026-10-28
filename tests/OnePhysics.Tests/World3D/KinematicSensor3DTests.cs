using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

/// <summary>Sensors against kinematic bodies and characters.</summary>
public class KinematicSensor3DTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld3D NewWorld(int threads = 1) => World3DTests.NewWorld(threads, 5, o => o.Gravity = default);

    private static Body SensorBody(PhysicsWorld3D w, double x, BodyType type)
    {
        var b = w.CreateBody(type, new Vector3D(x, 0, 0));
        w.CreateBox(b, new Vector3F(1, 1, 1), 0f, new ShapeDef { IsSensor = true, Density = 0f });
        return b;
    }

    private static Body Kinematic(PhysicsWorld3D w, double x, float vx)
    {
        var b = w.AddSphere(0.5f, new Vector3D(x, 0, 0), BodyType.Kinematic);
        b.LinearVelocity = new Vector3F(vx, 0, 0);
        return b;
    }

    private static List<string> Run(PhysicsWorld3D w, Body a, Body b, int steps)
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
        var other = w.AddSphere(0.5f, new Vector3D(0, 0, 0), BodyType.Static);
        var sensor = SensorBody(w, -4, BodyType.Kinematic);
        sensor.LinearVelocity = new Vector3F(3f, 0, 0);
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
        var c = w.AddCharacter(new Vector3D(-4, 0, 0), Character.Humanoid);
        var log = new List<string>();
        sensor.OnTriggerEnter += t => log.Add("enter");
        sensor.OnTriggerExit += t => log.Add("exit");
        for (int i = 0; i < 240; i++) { c.Move(new Vector3F(3f * Dt, 0, 0), Dt); w.Step(Dt); }
        Assert.Equal(["enter", "exit"], log);
    }
}
