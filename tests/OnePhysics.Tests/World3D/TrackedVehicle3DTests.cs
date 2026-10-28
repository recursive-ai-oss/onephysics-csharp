using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

public class TrackedVehicle3DTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld3D NewWorld(int threads = 1) => new(new WorldOptions3D { ThreadCount = threads, WorldId = 93 });

    private static void Floor(PhysicsWorld3D w, float friction = 0.8f) =>
        w.AddBox(new Vector3F(800f, 1f, 800f), at: new Vector3D(0, -0.5, 0), type: BodyType.Static, material: new Material3D(friction, 0f, 0f));

    private static void Run(PhysicsWorld3D w, int steps) { for (int i = 0; i < steps; i++) { w.Step(Dt); } }

    [Fact]
    public void TankRestsOnItsSuspensionAndSleepsInput()
    {
        var w = NewWorld();
        Floor(w);
        var tank = w.AddTank(new Vector3D(0, 0, 0));
        Assert.Equal(VehicleKind.Tracked, tank.Kind);
        Assert.Equal(10, tank.WheelCount);
        Run(w, 240);
        float sum = 0f;
        for (int i = 0; i < 10; i++) { var s = tank.GetWheel(i); Assert.True(s.IsInContact); sum += s.SuspensionForce; }
        Assert.Equal(20000f * 9.81f, sum, 0.02f * 20000f * 9.81f);
        var rv = tank.Chassis.LinearVelocity;
        Assert.InRange(MathF.Sqrt(rv.X * rv.X + rv.Z * rv.Z), 0f, 0.05f);
    }

    [Fact]
    public void DrivesStraightAtTheBeltSpeed()
    {
        var w = NewWorld();
        Floor(w);
        var tank = w.AddTank(new Vector3D(0, 0, 0));
        Run(w, 60);
        tank.SetInput(1f);
        Run(w, 480);
        Assert.InRange(tank.ForwardSpeed, 10.5f, 12.1f);
        Assert.InRange(tank.GetTrackSpeed(0), 10.5f, 12.1f);
        Assert.InRange(Math.Abs(tank.Chassis.Position.X), 0.0, 1.0);   // under 2 % of the distance
        Assert.InRange(Math.Abs(tank.Chassis.AngularVelocity.Y), 0f, 0.02f);
    }

    [Fact]
    public void OppositeTrackInputsTurnTheTankOnTheSpotAtTheExpectedRate()
    {
        var w = NewWorld();
        Floor(w);
        var def = TrackedVehicleDef.Tank;
        var tank = w.AddTank(new Vector3D(0, 0, 0), def);
        Run(w, 60);
        var start = tank.Chassis.Position;
        tank.SetInput(0f, 0.2f);
        Run(w, 180);
        var yaw = tank.Chassis.AngularVelocity.Y;
        float beltSpeed = 0.2f * def.MaxTrackSpeed;
        float ideal = 2f * beltSpeed / 2.6f;   // (right − left belt speed) over the track gauge; positive steering turns right, which is negative about +Y
        Assert.True(yaw > 0f);
        Assert.InRange(yaw / ideal, 0.55f, 1.05f);
        Assert.True(tank.GetTrackSpeed(0) > 0.5f * beltSpeed && tank.GetTrackSpeed(1) < -0.5f * beltSpeed);
        var now = tank.Chassis.Position;
        Assert.InRange(Math.Sqrt((now.X - start.X) * (now.X - start.X) + (now.Z - start.Z) * (now.Z - start.Z)), 0.0, 0.6);
        // The opposite sign turns the other way.
        tank.SetInput(0f, -0.2f);
        Run(w, 240);
        Assert.True(tank.Chassis.AngularVelocity.Y < 0f);
    }

    [Theory]
    [InlineData(0.30f)]
    [InlineData(0.45f)]
    public void ClimbsGradesUpToTheStatedGrade(float grade)
    {
        var w = NewWorld();
        Floor(w);
        // A slab rising towards +Z from z = 14 m at the given grade.
        float ang = MathF.Atan(grade);
        var rot = QuaternionF.CreateFromAxisSinCos(new Vector3F(1f, 0f, 0f), MathF.Sin(-0.5f * ang), MathF.Cos(-0.5f * ang));
        float len = 60f, th = 1f;
        // Centre of the slab so that its top surface starts at z = 14, y = 0.
        float cz = 14f + 0.5f * len * MathF.Cos(ang) + 0.5f * th * MathF.Sin(ang), cy = 0.5f * len * MathF.Sin(ang) - 0.5f * th * MathF.Cos(ang);
        w.AddBox(new Vector3F(30f, th, len), at: new Vector3D(0, cy, cz), rotation: rot, type: BodyType.Static, material: new Material3D(0.8f, 0f, 0f));
        var tank = w.AddTank(new Vector3D(0, 0, 0));
        tank.SetInput(1f);
        Run(w, 60 * 5);
        var p = tank.Chassis.Position;
        double up = p.Y;
        Assert.True(p.Z > 14 + 10, "z " + p.Z + " y " + p.Y);
        Assert.True(up > 6.0 * grade, "y " + up);
        // It is on the slope, not hovering above it: the road wheels carry the weight.
        int contact = 0;
        for (int i = 0; i < 10; i++) { if (tank.GetWheel(i).IsInContact) { contact++; } }
        Assert.True(contact >= 8);
    }

    [Fact]
    public void HoldsOnASlopeWithNoInputAndStopsWhenBraked()
    {
        var w = NewWorld();
        Floor(w);
        float ang = MathF.Atan(0.3f);
        var rot = QuaternionF.CreateFromAxisSinCos(new Vector3F(1f, 0f, 0f), MathF.Sin(-0.5f * ang), MathF.Cos(-0.5f * ang));
        float len = 60f, th = 1f;
        float cz = 14f + 0.5f * len * MathF.Cos(ang) + 0.5f * th * MathF.Sin(ang), cy = 0.5f * len * MathF.Sin(ang) - 0.5f * th * MathF.Cos(ang);
        w.AddBox(new Vector3F(30f, th, len), at: new Vector3D(0, cy, cz), rotation: rot, type: BodyType.Static, material: new Material3D(0.8f, 0f, 0f));
        var tank = w.AddTank(new Vector3D(0, 0, 0));
        tank.SetInput(1f);
        Run(w, 60 * 3);
        tank.SetInput(0f);
        Run(w, 240);
        var a = tank.Chassis.Position;
        Run(w, 120);
        var b = tank.Chassis.Position;
        Assert.InRange(Math.Abs(b.Z - a.Z), 0.0, 0.3);   // creeps a few cm/s at most: the tyre curve needs a little slip to carry the load
        tank.SetInput(1f, 0f, 1f);
        Run(w, 120);
        Assert.InRange(tank.Chassis.LinearVelocity.Length(), 0f, 0.1f);
    }

    [Fact]
    public void SnapshotMidMotionIsByteExactAndRestoresIntoAFreshWorld()
    {
        PhysicsWorld3D Make()
        {
            var w = NewWorld();
            Floor(w);
            var tank = w.AddTank(new Vector3D(0, 0, 0));
            var tank2 = w.AddTank(new Vector3D(20, 0, 0));
            tank.SetInput(0.8f, 0.3f);
            tank2.SetInput(-0.5f, -0.2f, 0.1f);
            return w;
        }

        var a = Make();
        Run(a, 150);
        var snap = a.Snapshot();
        Run(a, 100);
        var hashA = a.ComputeHash();
        var bytesA = a.Snapshot();

        a.Restore(snap);
        Run(a, 100);
        Assert.Equal(hashA, a.ComputeHash());
        Assert.Equal(bytesA, a.Snapshot());

        var fresh = NewWorld();
        fresh.Restore(snap);
        Run(fresh, 100);
        Assert.Equal(hashA, fresh.ComputeHash());
        Assert.Equal(bytesA, fresh.Snapshot());
        var vs = fresh.GetVehicles();
        Assert.Equal(2, vs.Length);
        Assert.Equal(VehicleKind.Tracked, vs[0].Kind);
        Assert.Equal(-0.2f, vs[1].Steering);
    }

    [Fact]
    public void ThreadCountDoesNotChangeTheResult()
    {
        ulong Run1(int threads)
        {
            var w = NewWorld(threads);
            Floor(w);
            var t = w.AddTank(new Vector3D(0, 0, 0));
            var b = w.AddMotorcycle(new Vector3D(10, 0, 0));
            t.SetInput(0.7f, 0.25f); b.SetInput(0.4f, 0.2f);
            Run(w, 200);
            return w.ComputeHash();
        }

        Assert.Equal(Run1(1), Run1(4));
    }

    [Fact]
    public void TheSharedSceneIsTheSameOnAnyThreadCountAndAfterARestore()
    {
        ulong a = VehicleKinds3DScene.Run(1, false);
        Assert.Equal(a, VehicleKinds3DScene.Run(3, false));
        Assert.Equal(a, VehicleKinds3DScene.Run(1, true));
    }

    [Fact]
    public void BadDefinitionsAreRejectedWithTheNameOfTheField()
    {
        var w = NewWorld();
        Floor(w);
        var d = TrackedVehicleDef.Tank;
        d.SprocketRadius = 0f;
        var e = Assert.Throws<ArgumentOutOfRangeException>(() => w.AddTank(new Vector3D(0, 0, 0), d));
        Assert.Contains("SprocketRadius", e.Message);
        d = TrackedVehicleDef.Tank;
        d.LeftWheels = Array.Empty<WheelDef>();
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddTank(new Vector3D(0, 0, 0), d));
        var m = MotorcycleDef.Road;
        m.HighSpeedSteerAngle = 2f;
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddMotorcycle(new Vector3D(0, 0, 0), m));
        m = MotorcycleDef.Road;
        m.Front.Radius = float.NaN;
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddMotorcycle(new Vector3D(0, 0, 0), m));
        var tank = w.AddTank(new Vector3D(0, 0, 0));
        Assert.Throws<InvalidOperationException>(() => tank.LeanAngle);
        tank.Destroy();
        Assert.Throws<StaleHandleException>(() => tank.Kind);
    }
}
