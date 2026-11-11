using System;
using System.Buffers;
using System.Collections.Generic;
using System.IO;
using System.Numerics;
using OneScope.Sdk;
using RecursiveAI.OnePhysics.Replay;
using Xunit;
using SdkHistoryOptions = OneScope.Sdk.HistoryOptions;
using SdkHistory = OneScope.Sdk.History;

namespace RecursiveAI.OnePhysics.Scope.Tests;

public class ModuleTests
{
    private const int Steps = 300;

    private static readonly Dictionary<int, Worlds.Run> Runs = [];

    private static Worlds.Run RunOf(int dimension)
    {
        lock (Runs)
        {
            if (!Runs.TryGetValue(dimension, out Worlds.Run? run))
            {
                Runs[dimension] = run = Worlds.Record(dimension, Steps);
            }

            return run;
        }
    }

    public static IEnumerable<object[]> DimensionsAndThreads() => [[2, 1], [2, 8], [3, 1], [3, 8]];

    public static IEnumerable<object[]> Intervals() => [[2, 1], [2, 7], [2, 64], [2, 1000], [3, 1], [3, 7], [3, 64], [3, 1000]];

    private static IScopeSource Open(Worlds.Run run, int threads = 1) => new OnePhysicsModule(threads).Open(run.Replay, "test.oppr");

    [Theory]
    [MemberData(nameof(DimensionsAndThreads))]
    public void HistoryGivesTheLibrarysOwnHashAtEveryStep(int dimension, int threads)
    {
        Worlds.Run run = RunOf(dimension);
        IScopeSource source = Open(run, threads);
        Assert.Equal(Steps, source.Recording.StepCount);
        Assert.Equal(dimension, source.Info.Dimensions);
        var history = new SdkHistory(source);
        Assert.Equal(run.Hashes[0], history.StateHash);
        for (int k = 1; k <= Steps; k++)
        {
            Assert.True(history.Step());
            Assert.Equal(run.Hashes[k], history.StateHash);
        }
    }

    [Theory]
    [MemberData(nameof(Intervals))]
    public void RandomSeeksGiveTheSameHashAsSteppingStraightThere(int dimension, int interval)
    {
        Worlds.Run run = RunOf(dimension);
        var history = new SdkHistory(Open(run, 8), new SdkHistoryOptions { KeyframeInterval = interval });
        uint state = 12345u + (uint)interval;
        for (int i = 0; i < 80; i++)
        {
            state = unchecked(state * 1664525u + 1013904223u);
            long k = (state >> 8) % (Steps + 1);
            history.Seek(k);
            Assert.Equal(k, history.CurrentStep);
            Assert.Equal(run.Hashes[k], history.StateHash);
        }
    }

    [Theory]
    [MemberData(nameof(Intervals))]
    public void AForkWithAPokeDivergesAtTheForkAndMatchesTheLibraryApi(int dimension, int interval)
    {
        Worlds.Run run = RunOf(dimension);
        var history = new SdkHistory(Open(run, 1), new SdkHistoryOptions { KeyframeInterval = interval });
        const long forkStep = 120;
        const int branchSteps = 30;
        Branch branch = history.Fork(forkStep);
        Assert.Null(branch.DivergenceStep);

        var entities = new List<EntityInfo>();
        branch.History.World.CollectEntities(entities);
        EntityInfo target = entities.Find(e => e.Kind == "body" && e.Name.Contains("(Awake)", StringComparison.Ordinal));
        Assert.True(target.Id.IsSome);
        var impulse = dimension == 3 ? new Vector3(2, 9, 1) : new Vector3(2, 9, 0);
        branch.Poke(new PokeCall("impulse", [ScopeValue.OfEntity(target.Id), ScopeValue.OfVector(impulse)]));
        Assert.Equal(1, branch.PendingPokes);
        for (int i = 0; i < branchSteps; i++)
        {
            Assert.True(branch.Step());
        }

        Assert.Equal(forkStep, branch.DivergenceStep);
        Assert.Null(history.Diff(branch.History, forkStep));
        Difference? diff = history.Diff(branch.History, forkStep + 1);
        Assert.NotNull(diff);

        // What the library's own API gives for the same poke, independently of the module.
        ulong[] expected = ReferenceBranch(run, dimension, forkStep, branchSteps, impulse);
        var hashes = new List<ulong>();
        for (long k = forkStep; k <= forkStep + branchSteps; k++)
        {
            branch.History.Seek(k);
            hashes.Add(branch.History.StateHash);
        }

        Assert.Equal(expected, hashes);

        // Seeking back and forth inside the branch, and a written replay of the branch, reproduce it.
        branch.History.Seek(3);
        Assert.Equal(run.Hashes[3], branch.History.StateHash);
        branch.History.Seek(forkStep + 17);
        Assert.Equal(expected[17], branch.History.StateHash);

        var written = new ArrayBufferWriter<byte>();
        branch.History.Source.WriteReplay(written);
        IScopeSource reopened = new OnePhysicsModule().Open(written.WrittenMemory, "branch.oppr");
        Assert.Equal(forkStep + branchSteps, reopened.Recording.StepCount);
        var again = new SdkHistory(reopened, new SdkHistoryOptions { KeyframeInterval = interval });
        for (long k = 0; k <= forkStep + branchSteps; k++)
        {
            again.Seek(k);
            ulong want = k < forkStep + 1 ? run.Hashes[k] : expected[k - forkStep];
            Assert.Equal(want, again.StateHash);
        }

        // The file is a valid OnePhysics replay: the library's own player reproduces every hash in it.
        var player = new ReplayPlayer(new MemoryStream(written.WrittenSpan.ToArray()));
        long played = 0;
        if (dimension == 3)
        {
            var world = player.CreateWorld3D();
            while (player.ReadNextStep(world))
            {
                played++;
            }
        }
        else
        {
            var world = player.CreateWorld2D();
            while (player.ReadNextStep(world))
            {
                played++;
            }
        }

        Assert.Equal(forkStep + branchSteps, played);
    }

    [Theory]
    [InlineData(2)]
    [InlineData(3)]
    public void WritingAReplayReadingItBackAndWritingAgainGivesTheSameBytes(int dimension)
    {
        IScopeSource source = Open(RunOf(dimension));
        var first = new ArrayBufferWriter<byte>();
        source.WriteReplay(first);
        IScopeSource back = new OnePhysicsModule().Open(first.WrittenMemory, "again.oppr");
        var second = new ArrayBufferWriter<byte>();
        back.WriteReplay(second);
        Assert.True(first.WrittenSpan.SequenceEqual(second.WrittenSpan));
        // And it still plays to the library's hashes.
        var history = new SdkHistory(back);
        history.Seek(Steps);
        Assert.Equal(RunOf(dimension).Hashes[Steps], history.StateHash);
    }

    [Theory]
    [InlineData(2)]
    [InlineData(3)]
    public void SnapshotsOpenAtStepZeroAndForksCanStepThem(int dimension)
    {
        Worlds.Run run = RunOf(dimension);
        var player = new ReplayPlayer(new MemoryStream(run.Replay));
        byte[] snapshot;
        ulong hash;
        if (dimension == 3)
        {
            var w = player.CreateWorld3D();
            for (int i = 0; i < 40; i++)
            {
                player.ReadNextStep(w);
            }

            snapshot = w.Snapshot();
            hash = w.ComputeHash();
        }
        else
        {
            var w = player.CreateWorld2D();
            for (int i = 0; i < 40; i++)
            {
                player.ReadNextStep(w);
            }

            snapshot = w.Snapshot();
            hash = w.ComputeHash();
        }

        IScopeSource source = new OnePhysicsModule().Open(snapshot, "state.opsn");
        Assert.Equal(dimension, source.Info.Dimensions);
        Assert.Equal(0, source.Recording.StepCount);
        var history = new SdkHistory(source);
        Assert.Equal(hash, history.StateHash);
        Branch branch = history.Fork(0);
        for (int i = 0; i < 20; i++)
        {
            branch.Step();
        }

        // 40 steps were already taken, so the world's own hash after 60 steps is the recorded one.
        Assert.Equal(run.Hashes[60], branch.History.StateHash);
    }

    [Fact]
    public void ExactStateScenesOpenToo()
    {
        Worlds.Run run = RunOf(3);
        var player = new ReplayPlayer(new MemoryStream(run.Replay));
        var world = player.CreateWorld3D();
        for (int i = 0; i < 25; i++)
        {
            player.ReadNextStep(world);
        }

        var text = new StringWriter();
        Scene.Scene.FromSnapshot(3, 1, world.Snapshot()).Write(text);
        IScopeSource source = new OnePhysicsModule().Open(System.Text.Encoding.UTF8.GetBytes(text.ToString()), "state.scene");
        Assert.Equal(3, source.Info.Dimensions);
        Assert.Equal(world.ComputeHash(), new SdkHistory(source).StateHash);
    }

    [Fact]
    public void EntitiesInspectionAndDrawingWork()
    {
        var history = new SdkHistory(Open(RunOf(3)));
        history.Seek(30);
        var entities = new List<EntityInfo>();
        history.World.CollectEntities(entities);
        Assert.Equal("world", entities[0].Kind);
        Assert.Contains(entities, e => e.Kind == "body");
        Assert.Contains(entities, e => e.Kind == "joint");
        var rows = new PropertyList();
        foreach (EntityInfo e in entities)
        {
            Assert.True(history.World.Inspect(e.Id, rows));
            Assert.NotEmpty(rows.Rows);
        }

        var list = new DrawList();
        history.World.Draw(list, new DrawRequest(0b11111));
        Assert.True(list.Items.Length > 20);
        Assert.Equal(0, list.Dropped);
        Assert.Contains(history.Tracks, t => t.Info.Id == "awake" && t.Samples.Count > 0);
        Assert.False(history.World.Inspect(new EntityId(0xFFFFFFFF), rows));
    }

    [Fact]
    public void BadInputIsRefusedWithATypedError()
    {
        var module = new OnePhysicsModule();
        Assert.Throws<ScopeFormatException>(() => module.Open(new byte[] { 1, 2, 3 }, "junk"));
        Assert.Throws<ScopeFormatException>(() => module.Open(RunOf(3).Replay.AsMemory(0, 40), "truncated.oppr"));
        Assert.Throws<ScopeFormatException>(() => module.Open("OPSN"u8.ToArray(), "empty.opsn"));
        IScopeWorld world = Open(RunOf(2)).CreateWorld();
        Assert.Throws<ScopeFormatException>(() => world.Advance(new byte[] { 0x77 }, TimelineSink.None));
        Assert.Throws<ScopeFormatException>(() => world.Advance(new byte[] { 0x10, 1, 2 }, TimelineSink.None));
        Assert.Throws<ScopeFormatException>(() => world.Advance(new byte[] { 0x02, 0xFF, 0xFF, 0, 0 }, TimelineSink.None));
        Assert.Throws<ScopeFormatException>(() => world.Restore(new byte[4]));
        var bad = new ArrayBufferWriter<byte>();
        Assert.Throws<ArgumentException>(() => world.EncodePoke(new PokeCall("explode", []), bad));
        Assert.Throws<ArgumentException>(() => world.EncodePoke(new PokeCall("impulse", [ScopeValue.OfEntity(new EntityId(12345)), ScopeValue.OfVector(Vector3.One)]), bad));
        Assert.Throws<ArgumentException>(() => world.EncodePoke(new PokeCall("wake", [ScopeValue.OfInteger(3)]), bad));
        Assert.Equal(0, bad.WrittenCount);
    }

    private static ulong[] ReferenceBranch(Worlds.Run run, int dimension, long forkStep, int steps, Vector3 impulse)
    {
        var hashes = new List<ulong>();
        var player = new ReplayPlayer(new MemoryStream(run.Replay));
        if (dimension == 3)
        {
            var world = player.CreateWorld3D();
            for (long i = 0; i < forkStep; i++)
            {
                player.ReadNextStep(world);
            }

            hashes.Add(world.ComputeHash());
            foreach (Physics3D.Body b in world.Bodies)
            {
                if (b.Type == BodyType.Dynamic && b.IsAwake)
                {
                    b.ApplyImpulse(new RecursiveAI.OneMath.Vector3F(impulse.X, impulse.Y, impulse.Z));
                    break;
                }
            }

            for (int i = 0; i < steps; i++)
            {
                world.Step(Worlds.Dt);
                hashes.Add(world.ComputeHash());
            }
        }
        else
        {
            var world = player.CreateWorld2D();
            for (long i = 0; i < forkStep; i++)
            {
                player.ReadNextStep(world);
            }

            hashes.Add(world.ComputeHash());
            foreach (Physics2D.Body b in world.Bodies)
            {
                if (b.Type == BodyType.Dynamic && b.IsAwake)
                {
                    b.ApplyImpulse(new RecursiveAI.OneMath.Vector2F(impulse.X, impulse.Y));
                    break;
                }
            }

            for (int i = 0; i < steps; i++)
            {
                world.Step(Worlds.Dt);
                hashes.Add(world.ComputeHash());
            }
        }

        return [.. hashes];
    }
}
