using System;
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.HashMatrix;

// Interest streams: a pile, a ragdoll and a joint chain streamed to a client over a moving region. Per step: the message bytes, the authority's and the client's shared-entity hashes, and the client's own world hash
// after it predicted a few steps ahead and applied the message.
internal static class InterestScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("interest/world3d", World3);
        Scenarios.Register("interest/world2d", World2);
    }

    private static void World3(ScenarioContext ctx)
    {
        var a = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 1201, ThreadCount = ctx.Threads, SleepEnabled = true });
        a.AddBox(new(200, 1, 200), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        for (int i = 0; i < 30; i++) a.AddBox(new(1, 1, 1), new Vector3D(-15 + i, 0.5 + (i % 3) * 1.05, ((i * 7) % 5) - 2.0));
        a.AddRagdoll(P3.RagdollDef.Humanoid, new Vector3D(3, 0.4, 6), QuaternionF.CreateFromAxisAngle(new Vector3F(1, 0, 0), 1.5707964f));
        var prev = a.AddBox(new(0.2f, 0.2f, 0.2f), new Vector3D(-4, 3, -6), type: BodyType.Static);
        for (int i = 0; i < 6; i++)
        {
            var link = a.AddBox(new(1, 0.2f, 0.2f), new Vector3D(-3.5 + i, 3, -6));
            a.CreateJoint(new P3.PointJointDef { BodyA = prev, BodyB = link, LocalFrameA = new P3.JointFrame3D(new Vector3F(i == 0 ? 0f : 0.5f, 0, 0), QuaternionF.Identity), LocalFrameB = new P3.JointFrame3D(new Vector3F(-0.5f, 0, 0), QuaternionF.Identity) });
            prev = link;
        }

        var c = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 1202, ThreadCount = ctx.Threads, SleepEnabled = true });
        var writer = a.CreateInterestWriter();
        var client = c.CreateInterestClient();
        for (int step = 0; step < 120; step++)
        {
            a.Step(1f / 60);
            double centre = -8.0 + 16.0 * step / 120.0;
            var region = new InterestRegion().AddBox(new Vector3D(centre - 6, -5, -12), new Vector3D(centre + 6, 40, 12));
            var msg = writer.Write(region);
            if (step > 0) { for (int k = 0; k < step % 4; k++) c.Step(1f / 60); }
            var result = client.Apply(msg);
            ctx.Write(msg);
            ulong ha = a.ComputeInterestHash(region), hc = client.ComputeSharedHash();
            if (ha != hc) throw new InvalidOperationException("the client's shared hash differs from the authority's at step " + step);
            ctx.Write((long)ha); ctx.Write((long)hc);
            ctx.Write(result.Entered); ctx.Write(result.Left); ctx.Write(result.Updated); ctx.Write(client.ProxyCount);
            ctx.Write((long)c.ComputeHash());
            ctx.EndStep();
        }
    }

    private static void World2(ScenarioContext ctx)
    {
        var a = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 1211, ThreadCount = ctx.Threads, SleepEnabled = true });
        a.AddBox(new(200, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        for (int i = 0; i < 30; i++) a.AddBox(new(1, 1), new Vector2D(-15 + i, 0.5 + (i % 3) * 1.05));
        a.AddRagdoll(P2.RagdollDef.Humanoid, new Vector2D(3, 1.2), 1.5707964f);
        var prev = a.AddBox(new(0.2f, 0.2f), new Vector2D(-4, 3), type: BodyType.Static);
        for (int i = 0; i < 6; i++)
        {
            var link = a.AddBox(new(1, 0.2f), new Vector2D(-3.5 + i, 3));
            a.CreateJoint(new P2.RevoluteJointDef { BodyA = prev, BodyB = link, LocalAnchorA = new Vector2F(i == 0 ? 0f : 0.5f, 0), LocalAnchorB = new Vector2F(-0.5f, 0) });
            prev = link;
        }

        var c = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 1212, ThreadCount = ctx.Threads, SleepEnabled = true });
        var writer = a.CreateInterestWriter();
        var client = c.CreateInterestClient();
        for (int step = 0; step < 120; step++)
        {
            a.Step(1f / 60);
            double centre = -8.0 + 16.0 * step / 120.0;
            var region = new InterestRegion().AddBox(new Vector2D(centre - 6, -5), new Vector2D(centre + 6, 40));
            var msg = writer.Write(region);
            if (step > 0) { for (int k = 0; k < step % 4; k++) c.Step(1f / 60); }
            var result = client.Apply(msg);
            ctx.Write(msg);
            ulong ha = a.ComputeInterestHash(region), hc = client.ComputeSharedHash();
            if (ha != hc) throw new InvalidOperationException("the client's shared hash differs from the authority's at step " + step);
            ctx.Write((long)ha); ctx.Write((long)hc);
            ctx.Write(result.Entered); ctx.Write(result.Left); ctx.Write(result.Updated); ctx.Write(client.ProxyCount);
            ctx.Write((long)c.ComputeHash());
            ctx.EndStep();
        }
    }
}
