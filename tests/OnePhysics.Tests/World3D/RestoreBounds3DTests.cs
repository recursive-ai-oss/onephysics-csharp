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

    // A snapshot the library wrote always restores, whatever the impulses in it: the solver can legitimately produce very large ones, so restore checks them only for being finite.
    // What a damaged one may do is throw bodies out of the supported range, and that must end in a typed error, never an argument error from inside a step.
    private static void RestoresAndStepsWithOnlyTypedErrors(PhysicsWorld3D fresh, byte[] snap)
    {
        fresh.Restore(snap);
        Assert.Equal(snap, fresh.Snapshot());
        foreach (var v in fresh.GetVehicles()) { v.Drive(1f); }
        for (int i = 0; i < 3; i++)
        {
            byte[] before = fresh.Snapshot();
            try { fresh.Step(Dt); }
            catch (BodyOutOfBoundsException) { Assert.Equal(before, fresh.Snapshot()); break; }
        }
    }

    [Fact]
    public void AnAbsurdSavedContactImpulseRestoresAndStepsWithOnlyTypedErrors()
    {
        var w = CarOnGround();
        var cons = w.Engine.Core.Constraints;
        int found = -1;
        for (int c = 0; c < cons.HighWater && found < 0; c++)
        {
            if (cons.IsLive(c) && cons.KindOf(c) == ConstraintKind.Contact && cons.PointCount[c] > 0) { found = c; }
        }

        Assert.True(found >= 0, "the scene should have a contact");
        cons.Payload(found)[ManifoldLayout.NormalImpulse(0)] = 9e32f;
        RestoresAndStepsWithOnlyTypedErrors(NewWorld(), w.Snapshot());
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

    // A chain of light boxes (about 1 kg) hung from a static anchor, with a box of about 1e6 kg at the end: every link carries the heavy box's weight.
    private static PhysicsWorld3D HeavyOnLightChain(int links = 4)
    {
        var w = NewWorld();
        var anchor = w.AddBox(new Vector3F(1f, 0.2f, 1f), at: new Vector3D(0, 20, 0), type: BodyType.Static);
        Body prev = anchor;
        for (int i = 0; i < links; i++)
        {
            var link = w.AddBox(new Vector3F(0.2f, 0.8f, 0.2f), at: new Vector3D(0, 19.5 - i, 0), density: 5f);
            w.AddPoint(prev, link, new Vector3D(0, 20 - i, 0));
            prev = link;
        }

        var heavy = w.AddBox(new Vector3F(1f, 1f, 1f), at: new Vector3D(0, 19.5 - links, 0), density: 1e6f);
        w.AddPoint(prev, heavy, new Vector3D(0, 20 - links, 0));
        return w;
    }

    [Fact]
    public void AHeavyBodyHungOnALightChainSettlesRoundTripsAndSteps()
    {
        var w = HeavyOnLightChain();
        for (int i = 0; i < 240; i++) { w.Step(Dt); }
        byte[] snap = w.Snapshot();
        var fresh = NewWorld();
        fresh.Restore(snap);
        Assert.Equal(snap, fresh.Snapshot());
        w.Step(Dt);
        fresh.Step(Dt);
        Assert.Equal(w.Snapshot(), fresh.Snapshot());
    }

    [Fact]
    public void AnAbsurdSavedJointImpulseRestoresAndStepsWithOnlyTypedErrors()
    {
        var w = HeavyOnLightChain(2);
        for (int i = 0; i < 30; i++) { w.Step(Dt); }
        var cs = w.Engine.Core.Constraints;
        int found = -1;
        for (int c = 0; c < cs.HighWater && found < 0; c++) { if (cs.IsLive(c) && cs.KindOf(c) == ConstraintKind.Joint) { found = c; } }
        Assert.True(found >= 0);
        cs.Payload(found)[0] = 7e31f;
        RestoresAndStepsWithOnlyTypedErrors(NewWorld(), w.Snapshot());
    }

    [Theory]
    [InlineData(1000f)]
    [InlineData(0.01f)]
    public void AWrongSavedExtentIsRefusedAndTheWorldIsUnchanged(float wrong)
    {
        var w = CarOnGround();
        var e = w.Engine;
        int slot = -1;
        for (int b = 0; b < e.Core.Bodies.HighWater && slot < 0; b++) { if (e.Core.Bodies.IsDynamic(b)) { slot = b; } }
        float good = e.MaxExtent[slot];
        e.MaxExtent[slot] = wrong;
        byte[] bad = w.Snapshot();
        e.MaxExtent[slot] = good;

        var other = CarOnGround();
        byte[] before = other.Snapshot();
        var ex = Assert.Throws<SnapshotFormatException>(() => other.Restore(bad));
        Assert.Contains("extent", ex.Message);
        Assert.Equal(before, other.Snapshot());
    }
}
