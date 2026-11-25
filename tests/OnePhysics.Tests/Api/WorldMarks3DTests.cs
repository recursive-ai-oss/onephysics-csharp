using System;
using RecursiveAI.OnePhysics.Replay;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Tests.Core;
using Xunit;
using P = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests.Api;

/// <summary>Mark and RollbackTo in 3D: the world after a rollback is the marked world bit for bit, marks follow their documented lifetime, and a ring of marks allocates nothing once warm.</summary>
[Collection("Allocation")]
public class WorldMarks3DTests
{
    private const float Dt = 1f / 60f;

    private static P.PhysicsWorld3D Build(int threads = 1)
    {
        var w = new P.PhysicsWorld3D(new P.WorldOptions3D { WorldId = 7, ThreadCount = threads, PairCapacity = 512, ContactCapacity = 512, IslandCapacity = 128 });
        w.AddBox(new Vector3F(20, 1, 20), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        for (int i = 0; i < 6; i++) { w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(i * 0.3, 1 + i * 1.2, 0)); }
        w.Warmup();
        return w;
    }

    private static void StepN(P.PhysicsWorld3D w, int n) { for (int i = 0; i < n; i++) { w.Step(Dt); } }

    [Theory]
    [InlineData(1)]
    [InlineData(7)]
    [InlineData(60)]
    public void RollbackRestoresBytesHashAndTheFuture(int k)
    {
        var world = Build();
        var never = Build();
        StepN(world, 20); StepN(never, 20);
        var bytes = world.Snapshot();
        ulong hash = world.ComputeHash();
        var mark = world.Mark();
        Assert.Equal(20, mark.Step);

        world.Options.Gravity = world.Options.Gravity * 0.5f;
        world.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0 * 0.3, 9, 0));
        StepN(world, k);
        var changed = world.RollbackTo(mark);

        Assert.True((changed & OptionChanges.Gravity) != 0);
        Assert.Equal(hash, world.ComputeHash());
        Assert.True(bytes.AsSpan().SequenceEqual(world.Snapshot()));
        StepN(world, 40); StepN(never, 40);
        Assert.Equal(never.ComputeHash(), world.ComputeHash());
        Assert.True(never.Snapshot().AsSpan().SequenceEqual(world.Snapshot()));
    }

    [Fact]
    public void RollbackAfterRestoreAndBodyChurnReturnsToTheMarkedBodies()
    {
        var world = Build();
        StepN(world, 10);
        P.Body first = default;
        foreach (var b in world.Bodies) { if (b.Type == BodyType.Dynamic) { first = b; break; } }
        var mark = world.Mark();
        var bytes = world.Snapshot();
        first.Destroy();
        var added = world.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0 * 0.3, 9, 0));
        StepN(world, 5);
        world.Restore(world.Snapshot());
        world.RollbackTo(mark);
        Assert.True(first.IsValid);
        Assert.False(added.IsValid);
        Assert.True(bytes.AsSpan().SequenceEqual(world.Snapshot()));
    }

    [Fact]
    public void MarksLiveUntilReleasedOrPassedByARollback()
    {
        var world = Build();
        var a = world.Mark();
        StepN(world, 3);
        var b = world.Mark();
        StepN(world, 3);
        var c = world.Mark();
        Assert.Equal(3, world.LiveMarkCount);

        world.RollbackTo(b);
        Assert.True(world.IsMarkValid(a));
        Assert.True(world.IsMarkValid(b));
        Assert.False(world.IsMarkValid(c));
        Assert.Throws<StaleHandleException>(() => world.RollbackTo(c));
        world.RollbackTo(b);                         // the target stays valid
        world.ReleaseMarksBefore(b);
        Assert.False(world.IsMarkValid(a));
        Assert.True(world.ReleaseMark(b));
        Assert.False(world.ReleaseMark(b));
        Assert.Throws<StaleHandleException>(() => world.RollbackTo(b));
        Assert.Equal(0, world.LiveMarkCount);

        var d = world.Mark();                        // a reused slot does not revive an old mark
        Assert.False(world.IsMarkValid(b));
        Assert.True(world.IsMarkValid(d));
        world.ReleaseAllMarks();
        Assert.False(world.IsMarkValid(d));
    }

    [Fact]
    public void ReleaseOldMarksKeepsTheNewest()
    {
        var world = Build();
        var marks = new WorldMark[5];
        for (int i = 0; i < marks.Length; i++) { world.Step(Dt); marks[i] = world.Mark(); }
        Assert.Equal(3, world.ReleaseOldMarks(2));
        Assert.False(world.IsMarkValid(marks[2]));
        Assert.True(world.IsMarkValid(marks[3]) && world.IsMarkValid(marks[4]));
        Assert.Equal(0, world.ReleaseOldMarks(2));
        Assert.Throws<ArgumentOutOfRangeException>(() => world.ReleaseOldMarks(-1));
    }

    [Fact]
    public void ANullMarkAndAMarkOfAnotherWorldAreRefused()
    {
        var world = Build();
        var other = new P.PhysicsWorld3D(new P.WorldOptions3D { WorldId = 8 });
        Assert.Throws<InvalidHandleException>(() => world.RollbackTo(default));
        Assert.Throws<InvalidHandleException>(() => world.RollbackTo(other.Mark()));
        Assert.False(world.ReleaseMark(other.Mark()));
        Assert.False(world.IsMarkValid(default));
    }

    [Fact]
    public void ARecordingSeesARollbackAsOneMutation()
    {
        var world = Build();
        StepN(world, 5);
        var mark = world.Mark();
        using var stream = new System.IO.MemoryStream();
        using (var recording = world.StartRecording(stream))
        {
            StepN(world, 4);
            world.RollbackTo(mark);
            StepN(world, 4);
        }

        stream.Position = 0;
        var player = new ReplayPlayer(stream);
        var played = new P.PhysicsWorld3D(new P.WorldOptions3D { WorldId = 7 });
        int steps = 0;
        while (player.ReadNextStep(played)) { steps++; }
        Assert.Equal(8, steps);
        Assert.Equal(world.ComputeHash(), played.ComputeHash());
    }

    [Fact]
    public void ARingOfMarksAllocatesNothingOnceWarm()
    {
        var world = Build();
        const int Ring = 6;
        var ring = new WorldMark[Ring];
        for (int i = 0; i < 60; i++)
        {
            world.Step(Dt);
            ring[i % Ring] = world.Mark();
            world.ReleaseOldMarks(Ring);
        }

        AllocMeter.Settle();
        long bytes = AllocMeter.Of(() =>
        {
            for (int i = 60; i < 260; i++)
            {
                world.Step(Dt);
                ring[i % Ring] = world.Mark();
                world.ReleaseOldMarks(Ring);
                if (i % 7 == 0)
                {
                    world.RollbackTo(ring[(i - 3) % Ring]);
                    for (int j = 0; j < 3; j++) { world.Step(Dt); ring[(i - 2 + j) % Ring] = world.Mark(); world.ReleaseOldMarks(Ring); }
                }
            }
        });
        Assert.Equal(0, bytes);
    }
}
