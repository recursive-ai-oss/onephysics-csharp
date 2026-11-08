using System.IO;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Physics3D;
using RecursiveAI.OnePhysics.Replay;

internal static class FuzzReplay
{
    public static void Run(byte[] bytes)
    {
        using var stream = new MemoryStream(bytes, false);
        try
        {
            var reader = new ReplayReader(stream, "fuzz", maximumPayloadBytes: 1024 * 1024);
            while (reader.ReadNext()) { }
        }
        catch (ReplayFormatException) { }
    }

    // This generator also produces the small, committed hash-matrix replay fixtures.
    // No randomness, external inputs, callbacks, or optional hardware paths select the recorded inputs.
    public static byte[] Seed(int dimension)
    {
        using var stream = new MemoryStream();
        if (dimension == 2)
        {
            var world = new PhysicsWorld2D(new WorldOptions2D { WorldId = 24, ThreadCount = 1 });
            world.AddBox(new(20, 1), at: new(0, -0.5), type: BodyType.Static);
            world.AddBox(new(1, 1), at: new(0, 1));
            var writer = new ReplayWriter(stream, 2, 0, world.Snapshot());
            for (int i = 0; i < 12; i++)
            {
                if (i == 6) { world.AddCircle(0.25f, at: new(2, 1)); writer.WriteSnapshot(world.Snapshot()); }
                world.Step(1f / 60);
                writer.WriteStep(1f / 60, world.ComputeHash());
            }
            writer.Complete();
        }
        else
        {
            var world = new PhysicsWorld3D(new WorldOptions3D { WorldId = 24, ThreadCount = 1 });
            world.AddBox(new(20, 1, 20), at: new(0, -0.5, 0), type: BodyType.Static);
            world.AddBox(new(1, 1, 1), at: new(0, 1, 0));
            var writer = new ReplayWriter(stream, 3, 0, world.Snapshot());
            for (int i = 0; i < 12; i++)
            {
                if (i == 6) { world.AddSphere(0.25f, at: new(2, 1, 0)); writer.WriteSnapshot(world.Snapshot()); }
                world.Step(1f / 60);
                writer.WriteStep(1f / 60, world.ComputeHash());
            }
            writer.Complete();
        }
        return stream.ToArray();
    }
}
