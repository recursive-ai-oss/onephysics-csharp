using System;
using System.Buffers;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Numerics;
using OneScope.Sdk;
using RecursiveAI.OnePhysics.Replay;
using Xunit;
using SdkHistoryOptions = OneScope.Sdk.ScopeHistoryOptions;
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
    public void SnapshotsOpenAtTheirOwnStepAndForksCanStepThem(int dimension)
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
        Assert.Equal(40, source.Recording.StartStep);
        Assert.Equal(0, source.Recording.StepCount);
        var history = new SdkHistory(source);
        Assert.Equal(40, history.CurrentStep);
        Assert.Equal(hash, history.StateHash);
        Branch branch = history.Fork(40);
        for (int i = 0; i < 20; i++)
        {
            branch.Step();
        }

        Assert.Equal(60, branch.History.CurrentStep);
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

    [Theory]
    [InlineData(2)]
    [InlineData(3)]
    public void RecordedHashesCarryAndAMatchingReplayRaisesNoFinding(int dimension)
    {
        Worlds.Run run = RunOf(dimension);
        IScopeSource source = Open(run);
        Assert.Equal(run.Hashes, source.Recording.ExpectedHashes.ToArray());
        var history = new SdkHistory(source);
        history.Scan();
        Assert.Null(history.FirstHashMismatch);
        Assert.DoesNotContain(history.Findings, f => f.Code == SdkHistory.MismatchCode);
    }

    [Theory]
    [InlineData(2)]
    [InlineData(3)]
    public void ATamperedHashGivesAFindingAtTheFirstWrongStep(int dimension)
    {
        Worlds.Run run = RunOf(dimension);
        byte[] bytes = (byte[])run.Replay.Clone();
        var reader = new ReplayReader(new MemoryStream(bytes, false));
        long target = 100;
        int hashAt = -1;
        long seen = 0;
        while (reader.ReadNext())
        {
            if (reader.Kind == ReplayRecordKind.EndStep && ++seen == target)
            {
                hashAt = (int)reader.RecordOffset + 5 + 8;
            }
        }

        Assert.True(hashAt > 0);
        bytes[hashAt] ^= 0x40;
        var history = new SdkHistory(new OnePhysicsModule().Open(bytes, "tampered.oppr"));
        history.Scan();
        Assert.Equal(target, history.FirstHashMismatch);
        Finding finding = Assert.Single(history.Findings, f => f.Code == SdkHistory.MismatchCode);
        Assert.Equal(target, finding.Step);
        // Playing it is not affected; only the recorded number was changed.
        history.Seek(Steps);
        Assert.Equal(run.Hashes[Steps], history.StateHash);
    }

    [Theory]
    [InlineData(2, 1)]
    [InlineData(2, 7)]
    [InlineData(3, 7)]
    [InlineData(3, 64)]
    public void AReplayThatStartsLateOpensThereSeeksAndForks(int dimension, int interval)
    {
        const int Pre = 40;
        Worlds.Run run = Worlds.Record(dimension, 120, Pre);
        IScopeSource source = Open(run);
        Assert.Equal(Pre, source.Recording.StartStep);
        Assert.Equal(Pre + 120, source.Recording.EndStep);
        var history = new SdkHistory(source, new SdkHistoryOptions { KeyframeInterval = interval });
        Assert.Equal(Pre, history.CurrentStep);
        Assert.Equal(run.Hashes[0], history.StateHash);
        Assert.Throws<ArgumentOutOfRangeException>(() => history.Seek(Pre - 1));
        uint state = 99;
        for (int i = 0; i < 40; i++)
        {
            state = unchecked(state * 1664525u + 1013904223u);
            long k = (state >> 8) % 121;
            history.Seek(Pre + k);
            Assert.Equal(run.Hashes[k], history.StateHash);
        }

        history.Scan();
        Assert.Null(history.FirstHashMismatch);

        Branch branch = history.Fork(Pre + 50);
        branch.Step();
        branch.Step();
        Assert.Equal(Pre + 52, branch.History.CurrentStep);
        Assert.Equal(run.Hashes[52], branch.History.StateHash);

        var written = new ArrayBufferWriter<byte>();
        source.WriteReplay(written);
        IScopeSource back = new OnePhysicsModule().Open(written.WrittenMemory, "late.oppr");
        Assert.Equal(Pre, back.Recording.StartStep);
        var again = new SdkHistory(back, new SdkHistoryOptions { KeyframeInterval = interval });
        again.Seek(Pre + 120);
        Assert.Equal(run.Hashes[120], again.StateHash);
        var second = new ArrayBufferWriter<byte>();
        back.WriteReplay(second);
        Assert.True(written.WrittenSpan.SequenceEqual(second.WrittenSpan));
    }

    [Theory]
    [InlineData(2)]
    [InlineData(3)]
    public void MutationsLiveInsideTheStepInputsAndHostileSizesAreRefused(int dimension)
    {
        // The recording adds a body at step 100: that step's input holds a whole snapshot, and nothing is stored beside the inputs.
        IScopeSource source = Open(RunOf(dimension));
        long big = 0;
        for (long s = 0; s < source.Recording.StepCount; s++)
        {
            big = Math.Max(big, source.Recording.InputAt(s).Length);
        }

        Assert.True(big > 1000);

        // A state change bigger than a step input may be is a typed error, not an allocation.
        var module = new OnePhysicsModule();
        byte[] replay = RunOf(dimension).Replay;
        var reader = new ReplayReader(new MemoryStream(replay, false));
        reader.ReadNext();
        byte[] initial = reader.Payload.ToArray();
        var stream = new MemoryStream();
        var writer = new ReplayWriter(stream, dimension, reader.InitialStep, initial, hashTrack: true, commandTrack: true);
        writer.WriteMutation("Huge", reader.InitialStep, new byte[Recording.MaxInputBytes + 1024]);
        writer.BeginStep(Worlds.Dt);
        writer.EndStep(reader.InitialStep + 1, 0);
        writer.Complete();
        Assert.Throws<ScopeFormatException>(() => module.Open(stream.ToArray(), "huge.oppr"));

        // An input whose state claims more bytes than it holds.
        IScopeWorld world = source.CreateWorld();
        Assert.Throws<ScopeFormatException>(() => world.Advance(new byte[] { 2, 1, 0, (byte)'x', 0xFF, 0xFF, 0xFF, 0x7F, 1, 2 }, TimelineSink.None));
        Assert.Throws<ScopeFormatException>(() => world.Advance(new byte[] { 2, 0, 0, 4, 0, 0, 0, 1, 2, 3, 4 }, TimelineSink.None));
    }

    [Theory]
    [InlineData(2)]
    [InlineData(3)]
    public void DifferenceNamesBodiesFirstAndPlanarAngularVelocityHasNoLane(int dimension)
    {
        IScopeSource source = Open(RunOf(dimension));
        var history = new SdkHistory(source);
        history.Seek(150);
        // A second world that stepped one more: many things differ, and the first one named is a body's field.
        var other = new SdkHistory(source);
        other.Seek(151);
        Difference? diff = history.World.FirstDifference(other.World);
        Assert.NotNull(diff);
        Assert.True(diff.Value.Entity.Value >> 56 == 1, "a body is named first");
        Assert.NotEqual("n", diff.Value.Field);
        if (dimension == 2)
        {
            Assert.DoesNotContain("angular velocity x", diff.Value.Field, StringComparison.Ordinal);
        }
    }

    [Fact]
    public void DifferenceInOnlyWorldLevelStateIsNamedWithItsSectionAndEntity()
    {
        var a = new Physics3D.PhysicsWorld3D(new Physics3D.WorldOptions3D { WorldId = 9 });
        var b = new Physics3D.PhysicsWorld3D(new Physics3D.WorldOptions3D { WorldId = 9, Gravity = new RecursiveAI.OneMath.Vector3F(0, -3, 0) });
        a.AddBox(new RecursiveAI.OneMath.Vector3F(1, 1, 1), new RecursiveAI.OneMath.Vector3D(0, 1, 0));
        b.AddBox(new RecursiveAI.OneMath.Vector3F(1, 1, 1), new RecursiveAI.OneMath.Vector3D(0, 1, 0));
        var module = new OnePhysicsModule();
        IScopeWorld wa = module.Open(a.Snapshot(), "a.opsn").CreateWorld();
        IScopeWorld wb = module.Open(b.Snapshot(), "b.opsn").CreateWorld();
        Difference? diff = wa.FirstDifference(wb);
        Assert.NotNull(diff);
        Assert.Equal(new EntityId(1), diff.Value.Entity);
        Assert.True(diff.Value.Field.Contains(' '));
        Assert.NotEqual("n", diff.Value.Field);

        var c = new Physics2D.PhysicsWorld2D(new Physics2D.WorldOptions2D { WorldId = 9 });
        var d = new Physics2D.PhysicsWorld2D(new Physics2D.WorldOptions2D { WorldId = 9 });
        c.AddCircle(0.5f, new RecursiveAI.OneMath.Vector2D(0, 1));
        d.AddCircle(0.5f, new RecursiveAI.OneMath.Vector2D(0, 1));
        foreach (Physics2D.Body body in d.Bodies)
        {
            body.AngularVelocity = 2f;
        }

        Difference? planar = module.Open(c.Snapshot(), "c.opsn").CreateWorld().FirstDifference(module.Open(d.Snapshot(), "d.opsn").CreateWorld());
        Assert.Equal("angular velocity", planar!.Value.Field);
    }

    [Theory]
    [InlineData(2)]
    [InlineData(3)]
    public void EveryDrawItemNamesItsEntityAndPokesCanBeDragged(int dimension)
    {
        IScopeSource source = Open(RunOf(dimension));
        Assert.All(source.Info.Pokes.Where(p => p.Id is "impulse" or "set_position"), p => Assert.Equal(PokeRole.DragToPoint, p.Role));
        var history = new SdkHistory(source);
        history.Seek(60);
        var list = new DrawList();
        history.World.Draw(list, new DrawRequest(~0UL));
        Assert.True(list.Items.Length > 20);
        foreach (DrawItem item in list.Items)
        {
            Assert.True(item.Entity.IsSome);
        }

        // Shapes carry their body, so a click on one picks a body that can be inspected.
        var rows = new PropertyList();
        Assert.True(history.World.Inspect(list.Items[0].Entity, rows));
    }

    [Fact]
    public void ContactNormalsAndJointLimitsAreDrawn()
    {
        Physics3D.PhysicsWorld3D world = Worlds.LimitScene3D();
        var stream = new MemoryStream();
        using (world.StartRecording(stream))
        {
            for (int i = 0; i < 60; i++)
            {
                world.Step(Worlds.Dt);
            }
        }

        var history = new SdkHistory(new OnePhysicsModule().Open(stream.ToArray(), "limits.oppr"));
        history.Seek(60);
        var contacts = new DrawList();
        history.World.Draw(contacts, new DrawRequest(1UL << ScopeInfo.OverlayContacts));
        Assert.Contains(contacts.Items.ToArray(), i => i.Kind == DrawKind.Arrow && i.Overlay == ScopeInfo.OverlayContacts);
        var limits = new DrawList();
        history.World.Draw(limits, new DrawRequest(1UL << ScopeInfo.OverlayLimits));
        var items = limits.Items.ToArray();
        Assert.All(items, i => Assert.Equal(ScopeInfo.OverlayLimits, i.Overlay));
        // Hinge arc and slider range and cone rays: three joints each leave line items.
        var jointIds = new HashSet<ulong>();
        foreach (DrawItem i in items)
        {
            if (i.Kind == DrawKind.Line)
            {
                jointIds.Add(i.Entity.Value);
            }
        }

        Assert.Equal(3, jointIds.Count);
        Assert.True(items.Length >= 24 + 3 + 12);
    }

    [Theory]
    [InlineData(2, 2)]
    [InlineData(2, 25)]
    [InlineData(3, 2)]
    [InlineData(3, 25)]
    public void ALiveStreamReplaysToTheHostsHashes(int dimension, int attachAfterRecords)
    {
        const int HostSteps = 70;
        var module = new OnePhysicsModule();
        ulong[] hostHash = new ulong[HostSteps + 1];
        long hostStart = 0;
        Func<ulong> hash;
        Func<long> stepCount;
        Action<int> advance;
        LiveFeedStream feed;
        Stream asStream;
        ReplayRecorder recorder;
        if (dimension == 3)
        {
            var world = new Physics3D.PhysicsWorld3D(new Physics3D.WorldOptions3D { WorldId = 5 });
            world.AddBox(new RecursiveAI.OneMath.Vector3F(40, 1, 40), new RecursiveAI.OneMath.Vector3D(0, -0.5, 0), type: BodyType.Static);
            for (int i = 0; i < 4; i++)
            {
                world.AddBox(new RecursiveAI.OneMath.Vector3F(1, 1, 1), new RecursiveAI.OneMath.Vector3D(0, 0.5 + i * 1.02, 0));
            }

            feed = new LiveFeedStream(world);
            recorder = world.StartRecording(feed);
            hash = world.ComputeHash;
            stepCount = () => world.StepCount;
            advance = i =>
            {
                if (i == 20)
                {
                    world.AddSphere(0.5f, new RecursiveAI.OneMath.Vector3D(0.2, 8, 0.1));
                }

                if (i == 30)
                {
                    foreach (Physics3D.Body b in world.Bodies)
                    {
                        if (b.Type == BodyType.Dynamic)
                        {
                            b.ApplyImpulse(new RecursiveAI.OneMath.Vector3F(3, 4, 0));
                            break;
                        }
                    }
                }

                world.Step(Worlds.Dt);
            };
        }
        else
        {
            var world = new Physics2D.PhysicsWorld2D(new Physics2D.WorldOptions2D { WorldId = 5 });
            world.AddBox(new RecursiveAI.OneMath.Vector2F(40, 1), new RecursiveAI.OneMath.Vector2D(0, -0.5), type: BodyType.Static);
            for (int i = 0; i < 4; i++)
            {
                world.AddBox(new RecursiveAI.OneMath.Vector2F(1, 1), new RecursiveAI.OneMath.Vector2D(0, 0.5 + i * 1.02));
            }

            feed = new LiveFeedStream(world);
            recorder = world.StartRecording(feed);
            hash = world.ComputeHash;
            stepCount = () => world.StepCount;
            advance = i =>
            {
                if (i == 20)
                {
                    world.AddCircle(0.5f, new RecursiveAI.OneMath.Vector2D(0.2, 8));
                }

                if (i == 30)
                {
                    foreach (Physics2D.Body b in world.Bodies)
                    {
                        if (b.Type == BodyType.Dynamic)
                        {
                            b.ApplyImpulse(new RecursiveAI.OneMath.Vector2F(3, 4));
                            break;
                        }
                    }
                }

                world.Step(Worlds.Dt);
            };
        }

        asStream = feed;
        Assert.True(asStream.CanWrite);
        hostHash[0] = hash();
        ILiveSource? live = null;
        int records = 0;
        feed.Record = record =>
        {
            records++;
            if (live is null)
            {
                if (records == attachAfterRecords)
                {
                    // The probe takes its snapshot inside the record call; that record is part of it and is not sent.
                    var snapshot = new ArrayBufferWriter<byte>();
                    feed.WriteSnapshot(snapshot);
                    live = module.OpenLive("1.0.0", 2, snapshot.WrittenMemory, "live");
                    hostStart = stepCount();
                }

                return;
            }

            live.AppendRecord(record);
        };

        for (int i = 0; i < HostSteps; i++)
        {
            advance(i);
            hostHash[i + 1] = hash();
        }

        recorder.Dispose();
        Assert.NotNull(live);
        IScopeSource source = live.Source;
        Assert.Equal(hostStart, source.Recording.StartStep);
        Assert.True(source.Recording.IsWritable);
        Assert.Equal(HostSteps, source.Recording.EndStep);
        var history = new SdkHistory(source);
        for (long s = source.Recording.StartStep; s <= HostSteps; s++)
        {
            history.Seek(s);
            Assert.Equal(hostHash[s], history.StateHash);
        }

        // Writing the live source as a replay reopens to the same hashes, and the library's own player agrees.
        var written = new ArrayBufferWriter<byte>();
        source.WriteReplay(written);
        IScopeSource back = module.Open(written.WrittenMemory, "live.oppr");
        Assert.Equal(hostStart, back.Recording.StartStep);
        var again = new SdkHistory(back);
        again.Scan();
        Assert.Null(again.FirstHashMismatch);
        again.Seek(HostSteps);
        Assert.Equal(hostHash[HostSteps], again.StateHash);
        var player = new ReplayPlayer(new MemoryStream(written.WrittenSpan.ToArray()));
        long played = 0;
        if (dimension == 3)
        {
            var w = player.CreateWorld3D();
            while (player.ReadNextStep(w))
            {
                played++;
            }
        }
        else
        {
            var w = player.CreateWorld2D();
            while (player.ReadNextStep(w))
            {
                played++;
            }
        }

        Assert.Equal(HostSteps - hostStart, played);
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
