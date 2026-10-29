using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

// A snapshot that parses must either be refused with the world untouched, or give a world that steps without an untyped error.
public class RestoreBounds3DTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld3D NewWorld() => new(new WorldOptions3D { WorldId = 77 });

    private static PhysicsWorld3D CarOnGround()
    {
        var w = NewWorld();
        w.AddBox(new Vector3F(100f, 1f, 100f), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var box = new Vector3F(1.8f, 0.5f, 4.2f);
        var chassis = w.AddBox(box, at: new Vector3D(0, 1.0, 0), density: 1500f / (box.X * box.Y * box.Z));
        w.AddVehicle(chassis, VehicleDef.Car).Drive(0.5f);
        w.AddBox(new Vector3F(1f, 1f, 1f), at: new Vector3D(10, 0.52, 0));   // rests on the ground, so there is a contact
        for (int i = 0; i < 30; i++) { w.Step(Dt); }
        return w;
    }

    [Fact]
    public void AnAbsurdSavedContactImpulseIsRefusedAndTheWorldIsUnchanged()
    {
        var w = CarOnGround();
        var cons = w.Engine.Core.Constraints;
        int found = -1;
        for (int c = 0; c < cons.HighWater && found < 0; c++)
        {
            if (cons.IsLive(c) && cons.KindOf(c) == ConstraintKind.Contact && cons.PointCount[c] > 0) { found = c; }
        }

        Assert.True(found >= 0, "the scene should have a contact");
        float good = cons.Payload(found)[ManifoldLayout.NormalImpulse(0)];
        cons.Payload(found)[ManifoldLayout.NormalImpulse(0)] = 9e32f;
        byte[] bad = w.Snapshot();
        cons.Payload(found)[ManifoldLayout.NormalImpulse(0)] = good;

        var other = CarOnGround();
        byte[] before = other.Snapshot();
        var e = Assert.Throws<SnapshotFormatException>(() => other.Restore(bad));
        Assert.Contains("impulse", e.Message);
        Assert.Equal(before, other.Snapshot());
    }

    [Fact]
    public void ASleepingChassisFarOutOfRangeIsReportedBeforeTheVehicleStageWakesIt()
    {
        var w = NewWorld();
        w.AddBox(new Vector3F(100f, 1f, 100f), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var box = new Vector3F(1.8f, 0.5f, 4.2f);
        var chassis = w.AddBox(box, at: new Vector3D(0, 1.0, 0), density: 1500f / (box.X * box.Y * box.Z));
        var car = w.AddVehicle(chassis, VehicleDef.Car);
        for (int i = 0; i < 60; i++) { w.Step(Dt); }
        chassis.Sleep();

        // Put the sleeping chassis outside the supported range the way a damaged step would leave it, then ask the vehicle to drive.
        var e = w.Engine;
        int slot = chassis.Handle.Index;
        Assert.False(e.Core.Bodies.IsAwake(slot));
        var p = e.Core.Bodies.PositionOf(slot);
        p[0] = -2.0e22; p[1] = 1.0e30; p[2] = 1.4e22;
        car.Drive(1f);
        Assert.Throws<BodyOutOfBoundsException>(() => w.Step(Dt));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AHeavyBodyOnLightOnesRoundTripsThroughSnapshot(bool tallStack)
    {
        var w = NewWorld();
        w.AddBox(new Vector3F(100f, 1f, 100f), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        double y = 0.0;
        int lights = tallStack ? 6 : 1;
        for (int i = 0; i < lights; i++)
        {
            w.AddBox(new Vector3F(1f, 1f, 1f), at: new Vector3D(0, y + 0.5, 0), density: 1f);   // about 1 kg each
            y += 1.0;
        }

        var heavy = new Vector3F(1f, 1f, 1f);
        w.AddBox(heavy, at: new Vector3D(0, y + 0.5, 0), density: 1e6f);                          // about 1e6 kg on top
        for (int i = 0; i < 120; i++) { w.Step(Dt); }

        byte[] snap = w.Snapshot();
        var fresh = NewWorld();
        fresh.Restore(snap);
        Assert.Equal(snap, fresh.Snapshot());
        fresh.Step(Dt);
    }
}
