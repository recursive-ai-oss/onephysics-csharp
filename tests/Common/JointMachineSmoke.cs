using System.IO;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

internal static class JointMachineSmoke
{
    internal static bool Run()
    {
        var a = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 471, Gravity = default });
        var ground = a.AddCircle(0.1f, default, BodyType.Static);
        var u = a.AddCircle(0.1f, new(2, 2)); var v = a.AddCircle(0.1f, new(4, 2));
        a.AddPulley(u, v, u.Position, v.Position, new(2, 5), new(4, 5));
        u.LinearVelocity = new(0, 0.2f);
        var follower = a.AddCircle(0.1f, new(0, 2));
        a.CreateJoint(new P2.PathJointDef { BodyA = ground, BodyB = follower, Points = [new(0, 2), new(1, 2), new(2, 3)], EnableMotor = true, MotorSpeed = 0.1f, MaxMotorForce = 10 });
        a.AddFilterJoint(u, follower).LifetimeStepsRemaining = 40;
        var b = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 472, Gravity = default });
        var anchor = b.AddSphere(0.1f, default, BodyType.Static);
        var wheel = b.AddSphere(0.1f, new(1, 2, 0));
        b.AddWheel(anchor, wheel, wheel.Position, new(1, 0, 0));
        var motor = b.AddSphere(0.1f, new(4, 2, 0)); b.AddMotor(anchor, motor, new(4, 3, 0), QuaternionF.Identity, 10, 10);
        var mouse = b.AddSphere(0.1f, new(6, 2, 0)); b.AddMouse(anchor, mouse, new(6, 3, 0), 10);
        for (int i = 0; i < 30; i++) { a.Step(1f / 60f); b.Step(1f / 60f); }
        var a2 = P2.PhysicsWorld2D.LoadScene(new StringReader(a.SaveScene()), threadCount: 8);
        var b2 = P3.PhysicsWorld3D.LoadScene(new StringReader(b.SaveScene()), threadCount: 8);
        var af = a.Fork(); var bf = b.Fork();
        bool ok = true;
        for (int i = 0; i < 30; i++)
        {
            a.Step(1f / 60f); b.Step(1f / 60f); a2.Step(1f / 60f); b2.Step(1f / 60f); af.Step(1f / 60f); bf.Step(1f / 60f);
            ok &= a.ComputeHash() == a2.ComputeHash() && a.ComputeHash() == af.ComputeHash() && b.ComputeHash() == b2.ComputeHash() && b.ComputeHash() == bf.ComputeHash();
        }
        return ok;
    }
}
