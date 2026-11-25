using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

/// <summary>A wheel's suspension pushes the ground body back with the same impulse it pushes the chassis, so a loose body far lighter than the vehicle is not ground for it.</summary>
public class VehicleLightGroundTests
{
    private const float Dt = 1f / 60f;
    private const float RestY = 0.5f + 0.34f - 0.1104f;   // wheel radius + max length - static deflection of the default car

    private static PhysicsWorld3D NewWorld() => new(new WorldOptions3D { ThreadCount = 1, WorldId = 92 });

    private static (Body Chassis, Vehicle Car) Car(PhysicsWorld3D w, double y)
    {
        var box = new Vector3F(1.8f, 0.5f, 4.2f);
        var body = w.AddBox(box, at: new Vector3D(0, y, 0), density: 1500f / (box.X * box.Y * box.Z));
        return (body, w.AddVehicle(body, VehicleDef.Car));
    }

    /// <summary>A ball under a wheel of a car driving on it (the streaming scene's cars ran over its spheres) leaves at no more than walking pace.</summary>
    [Fact]
    public void ABallUnderAWheelIsNotThrown()
    {
        var w = NewWorld();
        w.AddBox(new Vector3F(800f, 1f, 800f), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var (_, car) = Car(w, RestY + 0.02);
        var ball = w.AddSphere(0.2f, new Vector3D(0.8, 0.2, 1.3));
        double fastest = 0;
        for (int i = 0; i < 240; i++)
        {
            car.Drive(0.4f, 0f, 0f);
            w.Step(Dt);
            fastest = Math.Max(fastest, ball.LinearVelocity.Length());
        }

        Assert.True(fastest < 6.0, "the ball reached " + fastest + " m/s");
    }
}
