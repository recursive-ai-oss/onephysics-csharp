using System;
using System.IO;
using System.Buffers.Binary;
using System.Linq;
using RecursiveAI.OnePhysics.Replay;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class BodySleepTests
{
    [Fact]
    public void SeparateThresholdsAndDurationsApplyInBothDimensions()
    {
        var a = new P2.PhysicsWorld2D(new() { Gravity = default, TimeToSleep = 10 });
        var b = new P3.PhysicsWorld3D(new() { Gravity = default, TimeToSleep = 10 });
        var fast2 = a.CreateBody(new() { LinearVelocity = new(0.2f, 0), SleepSettings = new(0.3f, 0.1f, 0.05f) });
        var slow2 = a.CreateBody(new() { AngularVelocity = 0.2f, SleepSettings = new(0.3f, 0.1f, 0.05f) });
        var fast3 = b.CreateBody(new() { LinearVelocity = new(0.2f, 0, 0), SleepSettings = new(0.3f, 0.1f, 0.05f) });
        var slow3 = b.CreateBody(new() { AngularVelocity = new(0, 0.2f, 0), SleepSettings = new(0.3f, 0.1f, 0.05f) });
        for (int i = 0; i < 10; i++) { a.Step(0.01f); b.Step(0.01f); }
        Assert.False(fast2.IsAwake); Assert.False(fast3.IsAwake);
        Assert.True(slow2.IsAwake); Assert.True(slow3.IsAwake);
        slow2.SleepSettings = new(1, 1, 0.5f); slow3.SleepSettings = new(1, 1, 0.5f);
        for (int i = 0; i < 20; i++) { a.Step(0.01f); b.Step(0.01f); }
        Assert.True(slow2.IsAwake); Assert.True(slow3.IsAwake);
        for (int i = 0; i < 40; i++) { a.Step(0.01f); b.Step(0.01f); }
        Assert.False(slow2.IsAwake); Assert.False(slow3.IsAwake);
    }

    [Fact]
    public void NeverSleepAndResetWorkAndDefinitionsCanStartAsleep()
    {
        var a = new P2.PhysicsWorld2D(new() { Gravity = default });
        var b = new P3.PhysicsWorld3D(new() { Gravity = default });
        var x = a.CreateBody(new() { SleepSettings = new(0, 0, 0.01f), StartAsleep = true });
        var y = b.CreateBody(new() { SleepSettings = new(0, 0, 0.01f), StartAsleep = true });
        Assert.False(x.IsAwake); Assert.False(y.IsAwake);
        x.NeverSleep = true; y.NeverSleep = true;
        for (int i = 0; i < 100; i++) { a.Step(0.01f); b.Step(0.01f); }
        Assert.True(x.IsAwake); Assert.True(y.IsAwake);
        x.NeverSleep = false; y.NeverSleep = false;
        for (int i = 0; i < 3; i++) { a.Step(0.01f); b.Step(0.01f); }
        Assert.False(x.IsAwake); Assert.False(y.IsAwake);
        x.SleepSettings = default; y.SleepSettings = default;
        Assert.False(x.SleepSettings.IsOverride); Assert.False(y.SleepSettings.IsOverride);
        Assert.True(x.IsAwake); Assert.True(y.IsAwake);
        x.Destroy(); y.Destroy();
        Assert.Throws<StaleHandleException>(() => x.SleepSettings);
        Assert.Throws<StaleHandleException>(() => y.NeverSleep);
        Assert.False(a.CreateBody(default).SleepSettings.IsOverride);
        Assert.False(b.CreateBody(default).SleepSettings.IsOverride);
    }

    [Fact]
    public void OptionalSectionSurvivesFreshRestoreForkSceneAndReplay()
    {
        var a = new P2.PhysicsWorld2D(new() { WorldId = 791 });
        var x = a.AddBox(new(1, 1), at: new(0, 2));
        var original = a.Snapshot();
        using var stream = new MemoryStream();
        using (a.StartRecording(stream)) { x.SleepSettings = new(0.02f, 0.03f, 0.2f); x.NeverSleep = true; a.Step(0.01f); }
        var snap = a.Snapshot(); var copy = new P2.PhysicsWorld2D(new() { WorldId = 791 }); copy.Restore(snap);
        Assert.Equal(snap, copy.Snapshot()); Assert.Equal(snap, a.Fork().Snapshot());
        Assert.Equal(snap, P2.PhysicsWorld2D.LoadScene(new StringReader(a.SaveScene())).Snapshot());
        stream.Position = 0; var player = new ReplayPlayer(stream); var replay = new P2.PhysicsWorld2D(new() { WorldId = 791 });
        Assert.True(player.ReadNextStep(replay)); Assert.Equal(snap, replay.Snapshot());
        a.Restore(original); Assert.False(x.SleepSettings.IsOverride); Assert.False(x.NeverSleep); Assert.Equal(original, a.Snapshot());

        var b = new P3.PhysicsWorld3D(new() { WorldId = 792 }); var y = b.AddBox(new(1, 1, 1), at: new(0, 2, 0));
        y.SleepSettings = new(0.02f, 0.03f, 0.2f);
        var copy3 = new P3.PhysicsWorld3D(new() { WorldId = 792 }); copy3.Restore(b.Snapshot());
        Assert.Equal(b.Snapshot(), copy3.Snapshot()); Assert.Equal(b.Snapshot(), b.Fork().Snapshot());
        Assert.Equal(b.Snapshot(), P3.PhysicsWorld3D.LoadScene(new StringReader(b.SaveScene())).Snapshot());
    }

    [Fact]
    public void CorruptedPolicySectionIsRejectedAtomicallyAndHasNamedDesyncFields()
    {
        var w = new P2.PhysicsWorld2D(new() { WorldId = 793 }); var b = w.CreateBody(default);
        b.SleepSettings = new(0.02f, 0.03f, 0.25f);
        byte[] snapshot = w.Snapshot();
        var fields = SnapshotDesync.Inspect(w.Core);
        var linear = fields.Single(f => f.Section == "body sleep" && f.Name == "LinearThreshold");
        foreach (float bad in new[] { -1f, float.NaN, float.PositiveInfinity })
        {
            byte[] damaged = (byte[])snapshot.Clone(); BinaryPrimitives.WriteSingleLittleEndian(damaged.AsSpan(linear.Offset), bad);
            Assert.Throws<SnapshotFormatException>(() => w.Restore(damaged)); Assert.Equal(snapshot, w.Snapshot());
        }
        var other = w.Fork(); other.GetBody(b.RawHandle).SleepSettings = new(0.04f, 0.03f, 0.25f);
        Assert.Contains(w.FindDifferences(other).Differences, f => f.Section == "body sleep" && f.Field == "LinearThreshold");
        Assert.Contains(SnapshotDesync.Compare(w.Snapshot(), other.Snapshot()).Differences, f => f.Section == "body sleep");
    }

    [Fact]
    public void HostileParametersAndWrongBodyTypesAreRejected()
    {
        foreach (float bad in new[] { -1f, float.NaN, float.PositiveInfinity })
        {
            Assert.Throws<ArgumentOutOfRangeException>(() => new BodySleepSettings(bad, 0, 1));
            Assert.Throws<ArgumentOutOfRangeException>(() => new BodySleepSettings(0, bad, 1));
            Assert.Throws<ArgumentOutOfRangeException>(() => new BodySleepSettings(0, 0, bad));
        }
        Assert.Throws<ArgumentOutOfRangeException>(() => new BodySleepSettings(0, 0, 0));
        var a = new P2.PhysicsWorld2D(); var x = a.CreateBody(new() { Type = BodyType.Static });
        Assert.Throws<BodyTypeException>(() => x.SleepSettings = new(1, 1, 1));
        Assert.Throws<BodyTypeException>(() => x.NeverSleep = true);
        int before = a.BodyCount;
        Assert.Throws<BodyTypeException>(() => a.CreateBody(new() { Type = BodyType.Kinematic, SleepSettings = new(1, 1, 1) }));
        Assert.Equal(before, a.BodyCount);
    }
}
