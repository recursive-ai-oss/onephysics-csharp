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
        return good ? 0 : 1;
    }
}
