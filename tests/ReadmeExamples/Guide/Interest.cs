using System;
using RecursiveAI.OneMath;                       // Vector3D, Vector3F
using RecursiveAI.OnePhysics;                    // BodyType, InterestRegion, InterestWriter, InterestClient
using RecursiveAI.OnePhysics.Physics3D;          // PhysicsWorld3D

namespace GuideExamples;

/// <summary>The code blocks of docs/guide/interest.md.</summary>
public static class Interest
{
    public static (int Inside, int Proxies, bool SameHash, int LastBytes) Run()
    {
        var authority = new PhysicsWorld3D();
        authority.AddBox(new Vector3F(200, 1, 200), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        for (int i = 0; i < 20; i++) authority.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(-10 + i, 0.5 + 1.001 * (i % 2), 0));

        var clientWorld = new PhysicsWorld3D();                       // the client's own world; it may hold other bodies as well
        InterestWriter writer = authority.CreateInterestWriter();     // one writer per client
        InterestClient client = clientWorld.CreateInterestClient();

        byte[] message = Array.Empty<byte>();
        var region = new InterestRegion();
        for (int step = 0; step < 120; step++)
        {
            authority.Step(1f / 60f);
            region = new InterestRegion().AddBox(new Vector3D(-6 + step * 0.1, -5, -20), new Vector3D(6 + step * 0.1, 40, 20));
            message = writer.Write(region);                           // send these bytes to the client
            client.Apply(message);                                    // the client's world now holds what the region holds
            if (step % 10 == 0) clientWorld.Step(1f / 60f);           // predict: the next Apply puts the bodies back where the authority says
        }

        bool same = client.ComputeSharedHash() == authority.ComputeInterestHash(region);
        return (client.SharedBodyCount, client.ProxyCount, same, message.Length);
    }
}
