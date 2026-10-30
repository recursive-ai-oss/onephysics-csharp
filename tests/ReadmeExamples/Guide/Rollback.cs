using System;
using RecursiveAI.OneMath;               // Vector2F, Vector2D
using RecursiveAI.OnePhysics;            // BodyType, OptionChanges
using RecursiveAI.OnePhysics.Physics2D;  // PhysicsWorld2D, WorldOptions2D

namespace GuideExamples;

/// <summary>The code blocks of docs/guide/rollback.md.</summary>
public static class Rollback
{
    public static (bool Equal, int Frames) Run()
    {
        const float dt = 1f / 60;                               // one fixed dt for the whole session
        var world = new PhysicsWorld2D(new WorldOptions2D
        {
            WorldId = 1,                                        // handles stay valid across worlds built with the same id
            PairCapacity = 256, ContactCapacity = 256, IslandCapacity = 64,
        });
        world.AddBox(new Vector2F(20, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
        var ball = world.AddCircle(0.5f, at: new Vector2D(0, 4));
        int sounds = 0;
        ball.OnCollisionEnter += c => sounds++;
        world.Warmup();                                         // after the bodies exist: steps, snapshots and restores no longer allocate

        const int Ring = 8;
        var ring = new byte[Ring][];
        var lengths = new int[Ring];
        var hashes = new ulong[Ring];
        for (int i = 0; i < Ring; i++) ring[i] = new byte[world.GetSnapshotSizeBound()];   // never too small while the capacities hold

        int frame = 0;
        void Advance()
        {
            int slot = frame % Ring;
            if (!world.TrySnapshot(ring[slot], out lengths[slot])) throw new InvalidOperationException("snapshot buffer too small");
            world.Step(dt);
            hashes[slot] = world.ComputeHash();                 // exchange these with peers to catch a desync
            frame++;
        }

        for (int i = 0; i < 100; i++) Advance();
        ulong[] before = (ulong[])hashes.Clone();

        int back = 5;                                           // a late input for frame 95 arrived
        int target = frame - back;
        OptionChanges changed = world.Restore(ring[target % Ring].AsSpan(0, lengths[target % Ring]));
        frame = target;
        using (world.SuppressCallbacks())                       // no sound twice; polled events still work
        {
            for (int i = 0; i < back; i++) Advance();
        }

        return (changed == OptionChanges.None && before.AsSpan().SequenceEqual(hashes), frame);
    }
}
