using System;
using RecursiveAI.OneMath;               // Vector2F, Vector2D
using RecursiveAI.OnePhysics;            // BodyType, WorldMark
using RecursiveAI.OnePhysics.Physics2D;  // PhysicsWorld2D, WorldOptions2D

namespace GuideExamples;

/// <summary>The code blocks of the marks section of docs/guide/rollback.md.</summary>
public static class RollbackMarks
{
    public static (bool Equal, int Frames, int Live) Run()
    {
        const float dt = 1f / 60;
        var world = new PhysicsWorld2D(new WorldOptions2D { WorldId = 1, PairCapacity = 256, ContactCapacity = 256, IslandCapacity = 64 });
        world.AddBox(new Vector2F(20, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
        world.AddCircle(0.5f, at: new Vector2D(0, 4));
        world.Warmup();

        const int Ring = 8;
        var ring = new WorldMark[Ring];
        var hashes = new ulong[Ring];
        int frame = 0;
        void Advance(bool mark = true)
        {
            int slot = frame % Ring;
            if (mark)
            {
                ring[slot] = world.Mark();                      // the state before this frame's step
                world.ReleaseOldMarks(Ring);                    // keep the last 8
            }

            world.Step(dt);
            hashes[slot] = world.ComputeHash();                 // exchange these with peers to catch a desync
            frame++;
        }

        for (int i = 0; i < 100; i++) Advance();
        ulong[] before = (ulong[])hashes.Clone();

        int back = 5;                                           // a late input for frame 95 arrived
        int target = frame - back;
        world.RollbackTo(ring[target % Ring]);                  // marks taken after it are released; the target stays valid
        frame = target;
        using (world.SuppressCallbacks())                       // no sound twice; polled events still work
        {
            for (int i = 0; i < back; i++) Advance(mark: i > 0);   // the target frame still has its mark
        }

        return (before.AsSpan().SequenceEqual(hashes), frame, world.LiveMarkCount);
    }
}
