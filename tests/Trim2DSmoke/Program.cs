using System;
using RecursiveAI.OnePhysics;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Debug;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;

internal static class Program
{
    private static int Main()
    {
        var world = new PhysicsWorld2D(new WorldOptions2D { DetailedInspection = true, SleepEnabled = false });
        var anchor = world.AddCircle(0.1f, new Vector2D(0, 2), BodyType.Static);
        var body = world.AddBox(new Vector2F(1, 1), default);
        var joint = world.AddHinge(anchor, body, anchor.Position);
        long stamp = 0; world.Profile.TimestampSource = () => ++stamp;
        for (int i = 0; i < 120; i++) { world.Step(1f / 60); }
        var view = joint.Inspect();
        var geometry = new DebugGeometry(); world.GetDebugGeometry(DebugContent.Inspection, geometry);
        bool good = view.Rows[0].Flags != JointRowBits.None && view.ForceB.Y > 0 && body.Inspect().GravityImpulse.Y < 0 && world.Counters.Joints == 1;
        good &= geometry.JointErrors.Length == 2;
        var queries = new PhysicsWorld2D(new WorldOptions2D { Gravity = default });
        var rotating = queries.AddBox(new Vector2F(4, 0.1f));
        queries.AddCircle(0.1f, new Vector2D(0, 1.5), BodyType.Static);
        good &= rotating.Sweep(default, 6.2831855f, out _);
        good &= queries.Depenetrate(Shape2D.CreateCircle(default, 0.25f), new Vector2D(0.1, 0.1), 0).Converged;
        var inputs = new[] { QueryInput2D.Ray(new Vector2D(0, 3), new Vector2F(0, -1), 5) };
        good &= queries.QueryBatch(inputs, new QueryHit2D[1]) == 1;
        Console.WriteLine(good ? "OK trimmed 2D inspection" : "FAIL trimmed 2D inspection");
        // Soft bodies: a balloon and a blob on a floor, a rigid box dropped on them, snapshot and restore into a fresh world.
        var soft = new PhysicsWorld2D(new WorldOptions2D { WorldId = 9 });
        soft.AddBox(new Vector2F(20, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        var balloon = soft.AddBalloon(0.5f, new Vector2D(0, 1));
        soft.AddBlob(0.4f, new Vector2D(2, 1));
        soft.AddBox(new Vector2F(0.4f, 0.4f), new Vector2D(0, 3), density: 5f);
        for (int i = 0; i < 90; i++) { soft.Step(1f / 60); }
        var copy = new PhysicsWorld2D(new WorldOptions2D { WorldId = 9 });
        copy.Restore(soft.Snapshot());
        bool softGood = Math.Abs(balloon.Area / balloon.RestArea - 1) < 0.05 && copy.ComputeHash() == soft.ComputeHash() && copy.SoftBodyCount == 2;
        Console.WriteLine(softGood ? "OK trimmed 2D soft bodies" : "FAIL trimmed 2D soft bodies");
        good &= softGood;
        // Rail vehicles and bicycles: a loco on a graded track with a coupled wagon, a rider, snapshot and restore into a fresh world.
        var rail = new PhysicsWorld2D(new WorldOptions2D { WorldId = 10 });
        rail.AddBox(new Vector2F(200, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        var track = rail.AddTrack(TrackDef.Straight(300));
        var loco = rail.AddRailVehicle(track, 20, RailDef.Locomotive, 5f);
        var wagon = rail.AddRailVehicle(track, 20 - 9.25 - 0.6 - 0.02 - 5.25, RailDef.Freight, 5f);
        loco.Couple(wagon, thisFront: false, otherFront: true);
        var rider = rail.AddBicycle(new Vector2D(-60, 0), BicycleDef.Road, 6f);
        for (int i = 0; i < 90; i++) { loco.Drive(throttle: 0.3f); rider.Ride(pedal: 0.2f); rail.Step(1f / 60); }
        var railCopy = new PhysicsWorld2D(new WorldOptions2D { WorldId = 10 });
        railCopy.Restore(rail.Snapshot());
        bool railGood = loco.Speed > 5f && railCopy.ComputeHash() == rail.ComputeHash() && railCopy.GetRailVehicles().Length == 2 && railCopy.GetBicycles().Length == 1;
        Console.WriteLine(railGood ? "OK trimmed 2D rail and bicycles" : "FAIL trimmed 2D rail and bicycles");
        good &= railGood;
        return good ? 0 : 1;
    }
}
