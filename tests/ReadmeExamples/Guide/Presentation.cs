using System;
using System.IO;
using RecursiveAI.OneMath;                    // Vector3D
using RecursiveAI.OnePhysics;                 // BodyType
using RecursiveAI.OnePhysics.Presentation;    // PresentationQuality, PresentationPlayer
using RecursiveAI.OnePhysics.Physics3D;       // PhysicsWorld3D, PresentationWorld3D

namespace GuideExamples;

/// <summary>The code blocks of docs/guide/presentation.md.</summary>
public static class Presentation
{
    public static (ulong Authority, ulong Follower, ulong Replayed, int Cosmetics) Run()
    {
        float dt = 1f / 60f;
        var authority = new PhysicsWorld3D(new WorldOptions3D { WorldId = 1 });
        var floor = authority.AddBox(new Vector3F(20, 1, 20), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var character = authority.AddCapsule(0.5f, 0.3f, at: new Vector3D(0, 1, 0), type: BodyType.Kinematic);
        var platform = authority.AddBox(new Vector3F(2, 0.2f, 2), at: new Vector3D(0, 1, -3), type: BodyType.Kinematic);

        var follower = new PresentationWorld3D(new WorldOptions3D { WorldId = 2 });
        follower.Follow(floor);                                      // a copy of the floor: static
        Body capeMirror = follower.Follow(character);                // a kinematic copy that is moved to the character's pose every step
        Body ropeAnchor = follower.Follow(platform);

        follower.World.AddRope(new Vector3D(0, 1, -3), new Vector3D(0, 0, -3), 10, pinStart: false).Attach(0, ropeAnchor);
        follower.Quality = PresentationQuality.Medium;                // budgets: bodies, particles, cloth and rod effort

        using var file = new MemoryStream();
        using (follower.StartRecording(file))
        {
            for (int i = 0; i < 60; i++)
            {
                character.MoveKinematic(new Vector3D(0.01 * i, 1, 0), QuaternionF.Identity);
                authority.Step(dt);
                // reads the authority's poses and fracture events, then steps the follower
                follower.Step(authority, dt);
            }
        }

        file.Position = 0;
        var player = new PresentationPlayer(file);
        PresentationWorld3D replayed = player.CreateFollower3D();
        while (player.ReadNextStep()) { }                            // checks the follower's hash after every step; the authority is not needed
        _ = capeMirror;
        return (authority.ComputeHash(), follower.ComputeHash(), replayed.ComputeHash(), follower.CosmeticBodyCount);
    }
}
