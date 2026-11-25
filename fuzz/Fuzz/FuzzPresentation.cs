using System;
using System.IO;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Physics3D;
using RecursiveAI.OnePhysics.Presentation;
using RecursiveAI.OnePhysics.Replay;

/// <summary>The presentation recording reader: any bytes are either played or refused with a typed error.</summary>
internal static class FuzzPresentation
{
    public static void Run(byte[] bytes)
    {
        using var stream = new MemoryStream(bytes, false);
        try
        {
            var player = new PresentationPlayer(stream, "fuzz", maximumPayloadBytes: 1024 * 1024);
            if (player.Dimension == 3) player.CreateFollower3D(); else player.CreateFollower2D();
            for (int i = 0; i < 48 && player.ReadNextStep(); i++) { }
        }
        catch (Exception e) when (e is ReplayFormatException or ReplayHashMismatchException or PhysicsException or ArgumentException) { }
    }

    // A short recording with a moving platform, cosmetic bodies, a rope and a mid-run host change, so mutations reach frames, snapshots and the hash.
    public static byte[] Seed(int dimension)
    {
        using var stream = new MemoryStream();
        const float dt = 1f / 60f;
        if (dimension == 2)
        {
            var authority = new PhysicsWorld2D(new WorldOptions2D { WorldId = 25 });
            var floor = authority.AddBox(new(20, 1), at: new(0, -0.5), type: BodyType.Static);
            var platform = authority.AddBox(new(2, 0.2f), at: new(0, 1), type: BodyType.Kinematic);
            var follower = new PresentationWorld2D(new WorldOptions2D { WorldId = 26 });
            follower.Follow(floor); var mirror = follower.Follow(platform);
            follower.World.AddRope(new(0, 0.9), new(0, -0.1), 6, pinStart: false).Attach(0, mirror);
            using var recorder = follower.StartRecording(stream);
            for (int i = 0; i < 12; i++)
            {
                if (i == 5) follower.World.AddBox(new(0.3f, 0.3f), at: new(1, 3));
                platform.MoveKinematic(new Vector2D(0.02 * i, 1), 0f);
                authority.Step(dt); follower.Step(authority, dt);
            }
        }
        else
        {
            var authority = new PhysicsWorld3D(new WorldOptions3D { WorldId = 25 });
            var floor = authority.AddBox(new(20, 1, 20), at: new(0, -0.5, 0), type: BodyType.Static);
            var platform = authority.AddBox(new(2, 0.2f, 2), at: new(0, 1, 0), type: BodyType.Kinematic);
            var follower = new PresentationWorld3D(new WorldOptions3D { WorldId = 26 });
            follower.Follow(floor); var mirror = follower.Follow(platform);
            follower.World.AddRope(new(0, 0.9, 0), new(0, -0.1, 0), 6, pinStart: false).Attach(0, mirror);
            using var recorder = follower.StartRecording(stream);
            for (int i = 0; i < 12; i++)
            {
                if (i == 5) follower.World.AddBox(new(0.3f, 0.3f, 0.3f), at: new(1, 3, 0));
                platform.MoveKinematic(new Vector3D(0.02 * i, 1, 0), QuaternionF.Identity);
                authority.Step(dt); follower.Step(authority, dt);
            }
        }

        return stream.ToArray();
    }
}
