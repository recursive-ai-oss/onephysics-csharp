using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;

namespace RecursiveAI.OnePhysics.HashMatrix;

/// <summary>
/// A 2D world with every joint type (a hinge chain with limits and a motor, a slider with a motor and limit, a wheel on a car, a springy distance joint with a length range, a rigid rod, a weld that breaks, a mouse
/// joint and a relative-pose motor), hashed after every step; the snapshot taken at step 20 is restored at step 100 and the replay is hashed too, so the manifest also pins restore with joints.
/// The thread count comes from the runner and must not change a bit.
/// </summary>
internal static class Joints2DScenarios
{
    [System.Runtime.CompilerServices.ModuleInitializer]
    internal static void Init() => Scenarios.Register("world2d/joints", Run);

    private static void Run(ScenarioContext ctx)
    {
        var w = Build(ctx.Threads);
        byte[]? snap = null;
        for (int i = 0; i < 200; i++)
        {
            if (i == 20) { snap = w.Snapshot(); }
            if (i == 100) { w.Restore(snap!); }
            w.Step(1f / 60f);
            ctx.Write((long)w.ComputeHash());
            ctx.EndStep();
        }
    }

    private static PhysicsWorld2D Build(int threads)
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = 31, ThreadCount = threads, SleepEnabled = false });
        var ground = w.AddBox(new Vector2F(60f, 1f), new Vector2D(0, -0.5), type: BodyType.Static);
        var top = w.AddBox(new Vector2F(40f, 0.5f), new Vector2D(0, 12), type: BodyType.Static);

        var prev = top;
        for (int i = 0; i < 8; i++)
        {
            var link = w.AddBox(new Vector2F(1f, 0.2f), new Vector2D(-14 + 0.5 + i, 11.5), density: 1f + i);
            var j = w.AddHinge(prev, link, new Vector2D(-14 + i, 11.5));
            if (i % 3 == 2) { j.SetLimits(-0.4f, 0.4f); j.EnableLimit = true; }
            if (i == 0) { j.EnableMotor = true; j.MotorSpeed = 0.5f; j.MaxMotorTorque = 50f; }
            prev = link;
        }

        var rail = w.AddBox(new Vector2F(0.2f, 0.2f), new Vector2D(4, 8), type: BodyType.Static);
        var slide = w.AddBox(new Vector2F(1f, 1f), new Vector2D(4, 8));
        var s = w.AddSlider(rail, slide, new Vector2D(4, 8), new Vector2F(1, 0));
        s.SetLimits(-3f, 3f); s.EnableLimit = true; s.EnableMotor = true; s.MotorSpeed = 1f; s.MaxMotorForce = 30f;

        var car = w.AddBox(new Vector2F(3f, 0.5f), new Vector2D(-6, 2), density: 2f);
        var wheel = w.AddCircle(0.5f, new Vector2D(-5, 1), density: 1f, material: new Material2D(1.0f));
        var wj = w.AddWheel(car, wheel, new Vector2D(-5, 1), new Vector2F(0, 1));
        wj.SpringHertz = 3f; wj.EnableMotor = true; wj.MotorSpeed = -5f; wj.MaxMotorTorque = 20f;
        wj.SetLimits(-0.3f, 0.3f); wj.EnableLimit = true;

        var bob1 = w.AddCircle(0.4f, new Vector2D(8, 9));
        var bob2 = w.AddCircle(0.4f, new Vector2D(10, 9));
        var sp = w.AddSpring(top, bob1, new Vector2D(8, 11.75), new Vector2D(8, 9), 2f, 0.3f);
        sp.SetLimits(1f, 4f); sp.EnableLimit = true;
        w.AddDistance(bob1, bob2, new Vector2D(8, 9), new Vector2D(10, 9));

        var wall = w.AddBox(new Vector2F(0.4f, 3f), new Vector2D(14, 3), type: BodyType.Static);
        var beam = w.AddBox(new Vector2F(3f, 0.3f), new Vector2D(15.8, 4), density: 3f);
        var weld = w.AddFixed(wall, beam, new Vector2D(14.2, 4));
        weld.BreakForce = 250f;
        var tip = w.AddBox(new Vector2F(1f, 1f), new Vector2D(17, 5), density: 40f);
        w.AddHinge(beam, tip, new Vector2D(17, 4.3));

        var dragged = w.AddBox(new Vector2F(1f, 1f), new Vector2D(-14, 3));
        var mouse = w.AddTarget(ground, dragged, new Vector2D(-14, 3));
        mouse.Target = new Vector2D(-12, 5);
        var follower = w.AddBox(new Vector2F(1f, 0.5f), new Vector2D(-11, 1));
        var rel = w.AddRelative(dragged, follower, 200f, 100f);
        rel.SetOffset(new Vector2F(1.5f, -1f), 0.3f);

        // Some loose bodies so contacts and joints share colors.
        for (int i = 0; i < 12; i++) { w.AddBox(new Vector2F(0.8f, 0.8f), new Vector2D(-3 + 0.9 * (i % 4), 0.4 + 0.85 * (i / 4)), angleRadians: 0.01f * i); }
        return w;
    }
}
