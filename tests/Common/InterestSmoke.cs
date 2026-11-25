using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

// Interest streams in both worlds: a pile and a joint chain are streamed to a client over a moving region, bytes and hashes equal at one and three threads and the client's shared hash equals the authority's.
internal static class InterestSmoke
{
    internal static bool Run(out ulong digest)
    {
        ulong a = Planar(1), b = Planar(3), c = Spatial(1), d = Spatial(3);
        digest = a ^ (c * 31);
        return a == b && c == d && a != 0 && c != 0;
    }

    private static ulong Planar(int threads)
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 11, ThreadCount = threads });
        w.AddBox(new Vector2F(60, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
        for (int i = 0; i < 8; i++) w.AddBox(new Vector2F(1, 1), at: new Vector2D(-6 + 1.5 * i, 0.5 + 1.001 * (i % 2)));
        var client = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 12, ThreadCount = threads });
        var writer = w.CreateInterestWriter();
        var reader = client.CreateInterestClient();
        ulong fold = 14695981039346656037UL;
        for (int step = 0; step < 40; step++)
        {
            w.Step(1f / 60);
            var region = new InterestRegion().AddBox(new Vector2D(-6 + step * 0.2, -2), new Vector2D(2 + step * 0.2, 20));
            var message = writer.Write(region);
            if (step % 3 == 2) client.Step(1f / 60);
            reader.Apply(message);
            ulong h = reader.ComputeSharedHash();
            if (h != w.ComputeInterestHash(region)) return 0;
            fold = (fold ^ h) * 1099511628211UL;
        }

        return fold;
    }

    private static ulong Spatial(int threads)
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 11, ThreadCount = threads });
        w.AddBox(new Vector3F(60, 1, 60), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        for (int i = 0; i < 8; i++) w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(-6 + 1.5 * i, 0.5 + 1.001 * (i % 2), 0));
        var client = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 12, ThreadCount = threads });
        var writer = w.CreateInterestWriter();
        var reader = client.CreateInterestClient();
        ulong fold = 14695981039346656037UL;
        for (int step = 0; step < 40; step++)
        {
            w.Step(1f / 60);
            var region = new InterestRegion().AddBox(new Vector3D(-6 + step * 0.2, -2, -10), new Vector3D(2 + step * 0.2, 20, 10));
            var message = writer.Write(region);
            if (step % 3 == 2) client.Step(1f / 60);
            reader.Apply(message);
            ulong h = reader.ComputeSharedHash();
            if (h != w.ComputeInterestHash(region)) return 0;
            fold = (fold ^ h) * 1099511628211UL;
        }

        return fold;
    }
}
