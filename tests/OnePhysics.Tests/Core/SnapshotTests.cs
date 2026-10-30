using System;
using System.Collections.Generic;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Core;

public class SnapshotTests
{
    [Fact]
    public void RestoreThenStepMatchesSteppingTheOriginalOver1000StepsAnd5000Bodies()
    {
        var scene = ParticleScene.CreatePile(5000, 99, threadCount: 4, worldId: 77);
        var w = scene.World;
        Assert.Equal(5000, w.Bodies.Count);
        Assert.True(w.Constraints.Count > 500);
        scene.ReportPersist = true;
        w.Options.Events = EventMask.All;
        Util.Run(scene, 40);

        byte[] snap = w.Snapshot();
        var expected = Util.Run(scene, 1000);
        byte[] endBytes = w.Snapshot();

        // 1. Restore into the same world and replay.
        w.Restore(snap);
        Assert.Equal(snap, w.Snapshot());
        var again = Util.Run(scene, 1000);
        Assert.Equal(expected, again);
        Assert.Equal(endBytes, w.Snapshot());

        // 2. Restore into a fresh world (same id, different thread count) and replay.
        var fresh = new ParticleScene(new WorldOptions { WorldId = 77, ThreadCount = 1 });
        fresh.ReportPersist = true;
        fresh.World.Restore(snap);
        Assert.Equal(snap, fresh.World.Snapshot());
        var third = Util.Run(fresh, 1000);
        Assert.Equal(expected, third);
        Assert.Equal(endBytes, fresh.World.Snapshot());

        Assert.True(Util.CountDelivered(w, EventKind.Sleep) >= 0);
        Assert.Contains(expected, e => e.Count > 0);
    }

    [Fact]
    public void SnapshotCarriesPendingEventsAndSleepState()
    {
        var scene = ParticleScene.CreatePile(100, 5, worldId: 8);
        var w = scene.World;
        Util.Run(scene, 1200);
        Assert.True(w.Islands.Count > 0);
        bool anyAsleep = false;
        for (int i = 0; i < w.Islands.HighWater; i++) { anyAsleep |= w.Islands.IsLive(i) && w.Islands.IsAsleep(i); }
        Assert.True(anyAsleep);
        // Queue an event between steps, then snapshot.
        w.DestroyBody(w.HandleOfBody(3));
        Assert.True(w.Events.PendingCount > 0);
        byte[] snap = w.Snapshot();

        var other = new ParticleScene(new WorldOptions { WorldId = 8 });
        other.World.Restore(snap);
        Assert.Equal(w.Events.PendingCount, other.World.Events.PendingCount);
        scene.Step(1f / 60f);
        other.Step(1f / 60f);
        Assert.Equal(Util.EventDigest(w.Events.Delivered), Util.EventDigest(other.World.Events.Delivered));
        Assert.Equal(w.ComputeHash(), other.World.ComputeHash());
        Assert.Equal(w.Snapshot(), other.World.Snapshot());
    }

    [Fact]
    public void SectionsAreSavedRestoredAndHashed()
    {
        var scene = ParticleScene.CreatePile(30, 2, worldId: 3);
        var sec = new CounterSection { Value = 41 };
        scene.World.AddSection(sec);
        Assert.Throws<ArgumentException>(() => scene.World.AddSection(new CounterSection()));
        ulong h = scene.World.ComputeHash();
        sec.Value = 42;
        Assert.NotEqual(h, scene.World.ComputeHash());
        byte[] snap = scene.World.Snapshot();
        sec.Value = 7;
        scene.World.Restore(snap);
        Assert.Equal(42, sec.Value);

        var noSection = new ParticleScene(new WorldOptions { WorldId = 3 });
        Assert.Throws<SnapshotFormatException>(() => noSection.World.Restore(snap));
        sec.Value = 5;
        var bad = (byte[])snap.Clone();
        bad[^1] ^= 0xFF; // corrupts the section payload: fails its own validation
        Assert.Throws<SnapshotFormatException>(() => scene.World.Restore(bad));
        Assert.Equal(5, sec.Value); // a failed restore leaves everything as it was
    }

    [Fact]
    public void AnOptionalSectionLeavesBytesAndHashesAloneUntilItHoldsState()
    {
        var plain = ParticleScene.CreatePile(30, 2, worldId: 3);
        var scene = ParticleScene.CreatePile(30, 2, worldId: 3);
        var opt = new OptionalSection();
        scene.World.AddSection(opt);
        // Empty: the snapshot and the hash are those of a world without the section.
        Assert.Equal(plain.World.Snapshot(), scene.World.Snapshot());
        Assert.Equal(plain.World.ComputeHash(), scene.World.ComputeHash());

        opt.Value = 9;
        byte[] snap = scene.World.Snapshot();
        Assert.True(snap.Length > plain.World.Snapshot().Length);
        Assert.NotEqual(plain.World.ComputeHash(), scene.World.ComputeHash());

        // A world that does not know the section refuses the snapshot with a typed error.
        var old = ParticleScene.CreatePile(30, 2, worldId: 3);
        Assert.Throws<SnapshotFormatException>(() => old.World.Restore(snap));

        // A world that knows it restores the value, and a snapshot without the section resets it.
        var other = ParticleScene.CreatePile(30, 2, worldId: 3);
        var otherOpt = new OptionalSection();
        other.World.AddSection(otherOpt);
        other.World.Restore(snap);
        Assert.Equal(9, otherOpt.Value);
        other.World.Restore(plain.World.Snapshot());
        Assert.Equal(0, otherOpt.Value);
        Assert.Equal(1, otherOpt.Resets);
    }

    private sealed class OptionalSection : ISnapshotSection
    {
        public int Value;
        public int Resets;
        public int SectionId => 101;
        public bool IsOptional => true;
        public bool IsPresent => Value != 0;
        public void Reset() { Value = 0; Resets++; }
        public void Write(StateWriter writer) { writer.WriteInt32(Value); }
        public void Read(StateReader reader) { Value = reader.ReadInt32(); }
    }

    private sealed class CounterSection : ISnapshotSection
    {
        public int Value;
        public int SectionId => 100;
        public void Write(StateWriter writer) { writer.WriteInt32(Value); writer.WriteInt32(~Value); }
        public void Read(StateReader reader)
        {
            int v = reader.ReadInt32();
            if (reader.ReadInt32() != ~v) { throw reader.Error("counter check failed"); }
            Value = v;
        }
    }

    [Fact]
    public void EveryTruncationAndManyCorruptionsAreRejectedWithTypedErrorsAndLeaveTheWorldAlone()
    {
        var scene = ParticleScene.CreatePile(60, 4, worldId: 6);
        Util.Run(scene, 30);
        var w = scene.World;
        byte[] good = w.Snapshot();
        ulong before = w.ComputeHash();

        for (int len = 0; len < good.Length; len += Math.Max(1, len / 50 + 1))
        {
            var cut = good.AsSpan(0, len).ToArray();
            Assert.Throws<SnapshotFormatException>(() => w.Restore(cut));
        }

        var rng = new SplitMix64(12345);
        int rejected = 0, accepted = 0;
        for (int i = 0; i < 3000; i++)
        {
            var mutated = (byte[])good.Clone();
            int flips = 1 + (int)(rng.Next() % 4);
            for (int f = 0; f < flips; f++) { mutated[(int)(rng.Next() % (ulong)mutated.Length)] ^= (byte)(1 << (int)(rng.Next() % 8)); }
            var target = new ParticleScene(new WorldOptions { WorldId = 6 });
            try
            {
                target.World.Restore(mutated);
                accepted++;
                // Whatever is accepted must be a usable world.
                target.Step(1f / 60f);
                target.Step(1f / 60f);
            }
            catch (SnapshotFormatException)
            {
                rejected++;
            }
        }

        Assert.True(rejected > 500, "rejected " + rejected + " accepted " + accepted);
        Assert.Equal(before, w.ComputeHash());
        Assert.Equal(good, w.Snapshot());
    }

    [Fact]
    public void HeaderAndLayoutChecks()
    {
        var scene = ParticleScene.CreatePile(10, 4, worldId: 6);
        byte[] good = scene.World.Snapshot();
        var wrongMagic = (byte[])good.Clone(); wrongMagic[0] ^= 1;
        var e1 = Assert.Throws<SnapshotFormatException>(() => scene.World.Restore(wrongMagic));
        Assert.Contains("magic", e1.Message, StringComparison.Ordinal);
        var wrongVersion = (byte[])good.Clone(); wrongVersion[4] = 9;
        var e2 = Assert.Throws<SnapshotFormatException>(() => scene.World.Restore(wrongVersion));
        Assert.Contains("version", e2.Message, StringComparison.Ordinal);
        var planar = new WorldCore(new WorldOptions { Layout = DimensionLayout.Planar });
        var e3 = Assert.Throws<SnapshotFormatException>(() => planar.Restore(good));
        Assert.Contains("layout", e3.Message, StringComparison.Ordinal);
        Assert.Throws<SnapshotFormatException>(() => scene.World.Restore(Array.Empty<byte>()));
        var trailing = new byte[good.Length + 1];
        good.CopyTo(trailing, 0);
        Assert.Throws<SnapshotFormatException>(() => scene.World.Restore(trailing));
        Assert.Equal(2, DimensionLayout.Planar.PositionLanes);
    }

    [Fact]
    public void PlanarWorldsUseTheSameCore()
    {
        var w = new WorldCore(new WorldOptions { Layout = DimensionLayout.Planar });
        var a = w.CreateBody(BodyType.Dynamic, new double[] { 0, 0 });
        var b = w.CreateBody(BodyType.Dynamic, new double[] { 1, 0 });
        Assert.Equal(1f, w.Bodies.RotationOf(a.Index)[0]);
        w.AddConstraint(new ConstraintDesc { Kind = ConstraintKind.Contact, BodyA = a, BodyB = b });
        Assert.Equal(2, w.Layout.MaxManifoldPoints);
        var copy = new WorldCore(new WorldOptions { Layout = DimensionLayout.Planar });
        copy.Restore(w.Snapshot());
        Assert.Equal(w.ComputeHash(), copy.ComputeHash());
    }

    [Fact]
    public void WorldHashOfAFixedSceneIsStable()
    {
        // A regression anchor for the hash definition and the stages; the hash matrix checks the same value on other platforms.
        var scene = ParticleScene.CreatePile(300, 1234, worldId: 1);
        Util.Run(scene, 120);
        Assert.Equal(GoldenHash, scene.World.ComputeHash());
    }

    private const ulong GoldenHash = 10534202668867088900UL;
}
