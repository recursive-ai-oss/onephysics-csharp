using System;
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.HashMatrix;

// Body-set transfer: the packet bytes and the world hash after every step of the receiving world, with and without a portal transform and a boundary anchor.
internal static class TransferScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("transfer/world3d", World3);
        Scenarios.Register("transfer/world2d", World2);
    }

    private static void World3(ScenarioContext ctx)
    {
        var a = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 1101, ThreadCount = ctx.Threads, SleepEnabled = false });
        a.AddBox(new(60, 1, 60), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var stack = new P3.Body[5];
        for (int i = 0; i < stack.Length; i++) stack[i] = a.AddBox(new(1, 1, 1), new Vector3D(0.02 * i, 0.5 + 1.001 * i, 0));
        a.AddRagdoll(P3.RagdollDef.Humanoid, new Vector3D(8, 0.4, 0), QuaternionF.CreateFromAxisAngle(new Vector3F(1, 0, 0), 1.5707964f));
        var pivot = a.AddBox(new(0.4f, 0.4f, 0.4f), new Vector3D(-8, 8, 0), type: BodyType.Static);
        var arm = a.AddBox(new(0.2f, 2f, 0.2f), new Vector3D(-8, 7, 0));
        arm.AngularVelocity = new Vector3F(2, 0, 0);
        a.CreateJoint(new P3.HingeJointDef { BodyA = pivot, BodyB = arm, LocalFrameB = new P3.JointFrame3D(new Vector3F(0, 1, 0), QuaternionF.Identity) });
        for (int i = 0; i < 90; i++) a.Step(1f / 60);

        var all = new System.Collections.Generic.List<P3.Body>();
        foreach (var body in a.Bodies) if (body.Type == BodyType.Dynamic) all.Add(body);
        var packet = a.ExtractBodies(all.ToArray(), new BodySetOptions { Move = false, Boundary = BoundaryPolicy.HoldAsAnchor });
        ctx.Write(packet.AsSpan());
        ctx.Write(packet.BodyCount); ctx.Write(packet.AnchorCount); ctx.Write(packet.ShapeCount); ctx.Write(packet.JointCount); ctx.Write(packet.ContactCount);
        ctx.EndStep();

        // Two receivers: one in place, one through a portal that turns and moves the set.
        var inPlace = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 1102, ThreadCount = ctx.Threads, SleepEnabled = false });
        inPlace.AddBox(new(60, 1, 60), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        inPlace.InsertBodies(packet);
        var portal = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 1103, ThreadCount = ctx.Threads, SleepEnabled = false });
        portal.Options.Gravity = default;
        var map = portal.InsertBodies(BodySetPacket.Read(packet.ToArray()), new Vector3D(100, 5, -30), QuaternionF.CreateFromAxisAngle(new Vector3F(0, 1, 0), 0.6f));
        ctx.Write((long)inPlace.ComputeHash()); ctx.Write((long)portal.ComputeHash()); ctx.Write(map.NewBodies.Length); ctx.Write(map.NewShapes.Length);
        ctx.EndStep();
        for (int i = 0; i < 60; i++)
        {
            inPlace.Step(1f / 60); portal.Step(1f / 60);
            ctx.Write((long)inPlace.ComputeHash()); ctx.Write((long)portal.ComputeHash());
            ctx.EndStep();
        }

        // Moving instead of copying leaves the source world with what it did not give away.
        var moved = a.ExtractBodies(stack);
        ctx.Write(moved.ByteCount); ctx.Write(a.BodyCount); ctx.Write((long)a.ComputeHash());
        ctx.EndStep();
    }

    private static void World2(ScenarioContext ctx)
    {
        var a = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 1111, ThreadCount = ctx.Threads, SleepEnabled = false });
        a.AddBox(new(60, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        var stack = new P2.Body[6];
        for (int i = 0; i < stack.Length; i++) stack[i] = a.AddBox(new(1, 1), new Vector2D(0.02 * i, 0.5 + 1.001 * i));
        a.AddRagdoll(P2.RagdollDef.Humanoid, new Vector2D(8, 1.2), 1.5707964f);
        var pivot = a.AddBox(new(0.4f, 0.4f), new Vector2D(-8, 8), type: BodyType.Static);
        var arm = a.AddBox(new(0.2f, 2f), new Vector2D(-8, 7));
        arm.AngularVelocity = 2f;
        a.CreateJoint(new P2.RevoluteJointDef { BodyA = pivot, BodyB = arm, LocalAnchorB = new Vector2F(0, 1) });
        for (int i = 0; i < 90; i++) a.Step(1f / 60);

        var all = new System.Collections.Generic.List<P2.Body>();
        foreach (var body in a.Bodies) if (body.Type == BodyType.Dynamic) all.Add(body);
        var packet = a.ExtractBodies(all.ToArray(), new BodySetOptions { Move = false, Boundary = BoundaryPolicy.HoldAsAnchor });
        ctx.Write(packet.AsSpan());
        ctx.Write(packet.BodyCount); ctx.Write(packet.AnchorCount); ctx.Write(packet.ShapeCount); ctx.Write(packet.JointCount); ctx.Write(packet.ContactCount);
        ctx.EndStep();

        var inPlace = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 1112, ThreadCount = ctx.Threads, SleepEnabled = false });
        inPlace.AddBox(new(60, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        inPlace.InsertBodies(packet);
        var portal = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 1113, ThreadCount = ctx.Threads, SleepEnabled = false });
        portal.Options.Gravity = default;
        var map = portal.InsertBodies(BodySetPacket.Read(packet.ToArray()), new Vector2D(100, 5), 0.6f);
        ctx.Write((long)inPlace.ComputeHash()); ctx.Write((long)portal.ComputeHash()); ctx.Write(map.NewBodies.Length); ctx.Write(map.NewShapes.Length);
        ctx.EndStep();
        for (int i = 0; i < 60; i++)
        {
            inPlace.Step(1f / 60); portal.Step(1f / 60);
            ctx.Write((long)inPlace.ComputeHash()); ctx.Write((long)portal.ComputeHash());
            ctx.EndStep();
        }

        var moved = a.ExtractBodies(stack);
        ctx.Write(moved.ByteCount); ctx.Write(a.BodyCount); ctx.Write((long)a.ComputeHash());
        ctx.EndStep();
    }
}
