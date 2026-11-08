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
        return good ? 0 : 1;
    }
}
