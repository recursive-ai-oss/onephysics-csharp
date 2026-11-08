using System;
using System.Collections.Generic;
using System.IO;
using RecursiveAI.OnePhysics.Replay;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class ContactModificationTests
{
    [Theory]
    [InlineData(1)]
    [InlineData(8)]
    public void DisabledContactsLetBodiesFallThroughAndReenable(int threads)
    {
        var a = new P2.PhysicsWorld2D(new() { WorldId = 77, ThreadCount = threads, SleepEnabled = false });
        a.AddBox(new(20, 1), at: new(0, -0.5), type: BodyType.Static);
        var ball = a.AddCircle(0.5f, at: new(0, 0.5));
        int calls = 0;
        a.ContactModifier = (ref ContactModification c) => { calls++; c.Enabled = false; };
        for (int i = 0; i < 35; i++) a.Step(1f / 60);
        Assert.True(calls > 0);
        Assert.True(ball.Position.Y < -1);
        byte[] disabled = a.Snapshot();
        a.Restore(disabled);
        Assert.Equal(disabled, a.Snapshot());

        var b = new P3.PhysicsWorld3D(new() { WorldId = 78, ThreadCount = threads, SleepEnabled = false });
        b.AddBox(new(20, 1, 20), at: new(0, -0.5, 0), type: BodyType.Static);
        var sphere = b.AddSphere(0.5f, at: new(0, 0.5, 0));
        b.ContactModifier = (ref ContactModification c) => c.Enabled = false;
        for (int i = 0; i < 35; i++) b.Step(1f / 60);
        Assert.True(sphere.Position.Y < -1);
    }

    [Fact]
    public void OrderThreadEqualityRestoreAndRecorderEffects()
    {
        var a = Make2(1);
        var b = Make2(8);
        var orderA = new List<long>();
        var orderB = new List<long>();
        a.ContactModifier = (ref ContactModification c) => Edit(ref c, orderA);
        b.SetContactModifier(new Modifier(orderB));
        using var stream = new MemoryStream();
        var hashes = new List<ulong>();
        using (a.StartRecording(stream))
        {
            for (int i = 0; i < 15; i++)
            {
                orderA.Clear(); orderB.Clear();
                a.Step(1f / 60); b.Step(1f / 60);
                Assert.NotEmpty(orderA);
                Assert.Equal(orderA, orderB);
                for (int j = 1; j < orderA.Count; j++) Assert.True(orderA[j - 1] <= orderA[j]);
                Assert.Equal(a.ComputeHash(), b.ComputeHash());
                hashes.Add(a.ComputeHash());
            }
        }
        byte[] saved = a.Snapshot();
        Assert.Empty(a.FindDifferences(saved).Differences);
        string scene = a.SaveScene();
        var sceneCopy = P2.PhysicsWorld2D.LoadScene(new StringReader(scene));
        Assert.Equal(saved, sceneCopy.Snapshot());
        a.Step(1f / 60); ulong expected = a.ComputeHash();
        a.Restore(saved); a.Step(1f / 60);
        Assert.Equal(expected, a.ComputeHash());
        stream.Position = 0;
        var player = new ReplayPlayer(stream);
        var copy = new P2.PhysicsWorld2D(new() { WorldId = 77 });
        int step = 0;
        while (player.ReadNextStep(copy)) Assert.Equal(hashes[step++], copy.ComputeHash());
        Assert.Equal(hashes.Count, step);
    }

    [Fact]
    public void SpatialCallbacksAreWidthAndThreadIndependent()
    {
        var a = Make3(1); var b = Make3(8);
        var oa = new List<long>(); var ob = new List<long>();
        a.ContactModifier = (ref ContactModification c) => Edit(ref c, oa);
        b.ContactModifier = (ref ContactModification c) => Edit(ref c, ob);
        for (int i = 0; i < 20; i++)
        {
            oa.Clear(); ob.Clear(); a.Step(1f / 60); b.Step(1f / 60);
            Assert.NotEmpty(oa); Assert.Equal(oa, ob); Assert.Equal(a.ComputeHash(), b.ComputeHash());
        }
    }

    [Fact]
    public void RemovingModifierRestoresMaterialMixAndGeometry()
    {
        var a = Make2(1);
        a.ContactModifier = (ref ContactModification c) => { c.Friction = 0; c.Restitution = 1; c.Enabled = false; };
        a.Step(1f / 60);
        a.ContactModifier = null;
        int calls = 0;
        a.ContactModifier = (ref ContactModification c) => { calls++; Assert.True(c.Friction > 0); Assert.Equal(0f, c.Restitution); };
        a.Step(1f / 60);
        Assert.True(calls > 0);
    }

    [Fact]
    public void InvalidCoefficientsAndPointIndicesAreRejected()
    {
        var a = Make2(1);
        a.ContactModifier = (ref ContactModification c) =>
        {
            bool rejected = false;
            try { c.Friction = float.NaN; } catch (ArgumentOutOfRangeException) { rejected = true; }
            Assert.True(rejected);
            rejected = false;
            try { c.SetSeparation(c.PointCount, 0); } catch (ArgumentOutOfRangeException) { rejected = true; }
            Assert.True(rejected);
            rejected = false;
            try { c.SetSeparation(0, float.PositiveInfinity); } catch (ArgumentOutOfRangeException) { rejected = true; }
            Assert.True(rejected);
        };
        a.Step(1f / 60);
    }

    [Fact]
    public void RawCallbackIsTheSamePipelineAndSleepingPairsAreVisited()
    {
        var a = Make2(1);
        int calls = 0;
        a.Core.ContactModifier = (ref ContactModification c) => { calls++; Assert.Equal(0f, c.NormalZ); };
        a.Step(1f / 60);
        Assert.Equal(16, calls);
        Assert.Same(a.ContactModifier, a.Core.ContactModifier);
        a.Options.SleepEnabled = true;
        for (int i = 0; i < 100; i++) a.Step(1f / 60);
        calls = 0;
        a.Step(1f / 60);
        Assert.Equal(16, calls);
    }

    [Fact]
    public void EditedSnapshotAndForkRefreshWithoutOriginalCallback()
    {
        var a = Make2(1);
        a.ContactModifier = (ref ContactModification c) => { c.Enabled = false; c.SetSeparation(0, -10f); };
        a.Step(1f / 60);
        var copy = Make2(1);
        copy.Restore(a.Snapshot());
        var fork = a.Fork();
        Assert.Null(fork.ContactModifier);
        Assert.Equal(a.Snapshot(), fork.Snapshot());
        a.ContactModifier = null;
        a.Step(1f / 60); copy.Step(1f / 60); fork.Step(1f / 60);
        Assert.Equal(a.Snapshot(), copy.Snapshot());
        Assert.Equal(a.Snapshot(), fork.Snapshot());
    }

    [Fact]
    public void ConveyorTargetsRelativeTangentSpeedInBothDimensions()
    {
        var a = new P2.PhysicsWorld2D(new() { SleepEnabled = false });
        a.AddBox(new(100, 1), at: new(0, -0.5), type: BodyType.Static);
        var box = a.AddBox(new(1, 1), at: new(0, 0.5));
        a.ContactModifier = (ref ContactModification c) => c.TangentSpeed = 2f;
        for (int i = 0; i < 180; i++) a.Step(1f / 60);
        Assert.InRange(box.LinearVelocity.X, 1.98f, 2.02f);

        var b = new P3.PhysicsWorld3D(new() { SleepEnabled = false });
        b.AddBox(new(100, 1, 100), at: new(0, -0.5, 0), type: BodyType.Static);
        var cube = b.AddBox(new(1, 1, 1), at: new(0, 0.5, 0));
        b.ContactModifier = (ref ContactModification c) => c.TangentSpeed = 2f;
        for (int i = 0; i < 180; i++) b.Step(1f / 60);
        Assert.InRange(cube.LinearVelocity.Z, -2.02f, -1.98f);
    }

    [Fact]
    public void ZeroImpulseLimitPreventsNormalResponse()
    {
        var a = Make2(1); var b = Make3(1);
        a.ContactModifier = (ref ContactModification c) => c.NormalImpulseLimit = 0;
        b.ContactModifier = (ref ContactModification c) => c.NormalImpulseLimit = 0;
        for (int i = 0; i < 10; i++) { a.Step(1f / 60); b.Step(1f / 60); }
        foreach (var core in new[] { a.Core, b.Core })
        {
            var store = core.Constraints;
            int contacts = 0;
            for (int c = 0; c < store.HighWater; c++)
            {
                if (!store.IsLive(c) || store.KindOf(c) != LowLevel.ConstraintKind.Contact) continue;
                contacts++;
                var payload = store.Payload(c);
                for (int pt = 0; pt < store.PointCount[c]; pt++) Assert.Equal(0f, payload[LowLevel.ManifoldLayout.NormalImpulse(pt)]);
            }
            Assert.True(contacts > 0);
            Assert.InRange(core.Bodies.LinearVelocityOf(1)[1], -1.64f, -1.62f);
        }
    }

    private static void Edit(ref ContactModification c, List<long> order)
    {
        order.Add(((long)c.ShapeA.Index << 32) | (uint)c.ShapeB.Index);
        c.Friction = 0.1f; c.Restitution = 0.2f;
        c.TangentSpeed = 0.3f; c.NormalImpulseLimit = 0.25f;
        for (int i = 0; i < c.PointCount; i++) c.SetSeparation(i, c.GetSeparation(i) + 0.001f);
    }
    private readonly struct Modifier(List<long> order) : IContactModifier
    {
        public void Modify(ref ContactModification c) => Edit(ref c, order);
    }
    private static P2.PhysicsWorld2D Make2(int threads)
    {
        var w = new P2.PhysicsWorld2D(new() { WorldId = 77, ThreadCount = threads, SleepEnabled = false });
        w.AddBox(new(100, 1), at: new(0, -0.5), type: BodyType.Static);
        for (int i = 0; i < 16; i++) w.AddBox(new(1, 1), at: new(2 * i - 15, 0.49));
        return w;
    }
    private static P3.PhysicsWorld3D Make3(int threads)
    {
        var w = new P3.PhysicsWorld3D(new() { WorldId = 77, ThreadCount = threads, SleepEnabled = false });
        w.AddBox(new(100, 1, 100), at: new(0, -0.5, 0), type: BodyType.Static);
        for (int i = 0; i < 16; i++) w.AddBox(new(1, 1, 1), at: new(2 * i - 15, 0.49, 0));
        return w;
    }
}
