using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.HashMatrix;

/// <summary>Members that closed the 2D/3D gaps: 3D fixed-joint springs and the distance-joint motor beside rigid joints and a contact pile, and a 2D world with a changed speculative distance, local shape poses and typed machine joints.</summary>
internal static class ParityGapScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("world3d/joint-springs", ctx =>
        {
            var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 931, ThreadCount = ctx.Threads, SleepEnabled = false });
            var ground = w.AddBox(new Vector3F(40, 1, 40), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
            var anchor = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 6, 0), type: BodyType.Static);
            for (int i = 0; i < 6; i++)
            {
                var b = w.AddBox(new Vector3F(0.5f, 0.5f, 0.5f), at: new Vector3D(-3 + i, 4.5, 0.3 * i));
                w.CreateJoint(new P3.FixedJointDef
                {
                    BodyA = anchor, BodyB = b, LinearHertz = 1.5f + 0.5f * i, LinearDampingRatio = 0.3f, AngularHertz = i % 2 == 0 ? 2f : 0f, AngularDampingRatio = 0.2f,
                    LocalFrameA = new P3.JointFrame3D(new Vector3F(-3 + i, -1.5f, 0.3f * i), QuaternionF.Identity),
                });
            }

            var rod = w.AddSphere(0.25f, at: new Vector3D(6, 4, 0));
            var motor = w.CreateJoint(new P3.DistanceJointDef
            {
                BodyA = anchor, BodyB = rod, Length = 6.5f, MinLength = 1f, MaxLength = 12f, EnableSpring = true, SpringHertz = 0.8f, SpringDampingRatio = 0.5f,
                EnableMotor = true, MotorSpeed = 0.4f, MaxMotorForce = 200f,
            });
            for (int i = 0; i < 20; i++) { w.AddBox(new Vector3F(0.6f, 0.6f, 0.6f), at: new Vector3D(-8 + 0.7 * (i % 5), 0.3 + 0.65 * (i / 5), -6)); }
            _ = ground;
            byte[]? snapshot = null;
            for (int i = 0; i < 300; i++)
            {
                if (i == 100) snapshot = w.Snapshot();
                if (i == 200) { w.Restore(snapshot!); motor.MotorSpeed = -0.4f; }
                w.Step(1f / 60f); ctx.Write((long)w.ComputeHash()); ctx.EndStep();
            }
        });

        Scenarios.Register("world2d/parity-gaps", ctx =>
        {
            var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 932, ThreadCount = ctx.Threads, SpeculativeDistance = 0.04f, BroadphaseMargin = 0.1 });
            w.AddBox(new Vector2F(40, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
            var frame = w.AddCircle(0.2f, at: new Vector2D(0, 3), type: BodyType.Static);
            // One body holding two posed shapes, a gear pair and a pile to touch.
            var body = w.AddCircle(0.3f, at: new Vector2D(-5, 2));
            w.CreateBox(body, new Vector2F(1.2f, 0.2f), def: new P2.ShapeDef { LocalPosition = new Vector2F(0.8f, 0.2f), LocalAngle = 0.6f });
            var wheelA = w.AddCircle(0.5f, at: new Vector2D(2, 1.5)); var wheelB = w.AddCircle(0.5f, at: new Vector2D(4, 1.5));
            w.AddGear(wheelA, wheelB, 2f);
            wheelA.AngularVelocity = 3f;
            for (int i = 0; i < 24; i++) { w.AddBox(new Vector2F(0.5f, 0.5f), at: new Vector2D(8 + 0.55 * (i % 6), 0.3 + 0.55 * (i / 6))); }
            _ = frame;
            byte[]? snapshot = null;
            for (int i = 0; i < 300; i++)
            {
                if (i == 100) snapshot = w.Snapshot();
                if (i == 200) w.Restore(snapshot!);
                w.Step(1f / 60f); ctx.Write((long)w.ComputeHash()); ctx.EndStep();
            }
        });
    }
}
