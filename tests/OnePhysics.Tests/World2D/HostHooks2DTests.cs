using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

/// <summary>The span snapshot API, restore atomicity and the allocation-free hash, step, snapshot and restore of the 2D world.</summary>
public class HostHooks2DTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld2D Build(int threads, bool withCharacter = true)
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { ThreadCount = threads, WorldId = 5, Events = EventMask.All });
        w.AddBox(new Vector2F(60f, 1f), new Vector2D(0, -0.5), type: BodyType.Static);
        Body? prev = null;
        for (int i = 0; i < 40; i++)
        {
            var b = w.AddBox(new Vector2F(0.8f, 0.8f), new Vector2D((i % 8) * 0.9 - 3, 0.5 + (i / 8) * 0.9));
            if (i == 10)
            {
                var j = new RevoluteJointDef { BodyA = prev!.Value, BodyB = b, LocalAnchorA = new Vector2F(0.4f, 0f), LocalAnchorB = new Vector2F(-0.4f, 0f) };
                w.CreateJoint(in j);
            }

            prev = b;
        }

        var sensorBody = w.CreateBody(BodyType.Static, new Vector2D(0, 2));
        w.CreateBox(sensorBody, 1f, 1f, default, 0f, 0f, new ShapeDef { IsSensor = true });
        w.AddCircle(0.3f, new Vector2D(0.2, 6));
        if (withCharacter) { w.AddCharacter(new Vector2D(8, 1)); }
        return w;
    }

    private static byte[] Warm(PhysicsWorld2D w)
    {
        for (int i = 0; i < 1200; i++) { w.Step(Dt); }
        var buffer = new byte[w.GetSnapshotSize() * 2 + 64];
        for (int i = 0; i < 3; i++)
        {
            w.ComputeHash();
            w.TrySnapshot(buffer, out int n);
            w.Restore(buffer.AsSpan(0, n));
            w.Step(Dt);
        }

        return buffer;
    }

    [Theory]
    [InlineData(1)]
    [InlineData(8)]
    public void HashStepSnapshotAndRestoreAllocateNothingOnceWarm(int threads)
    {
        var w = Build(threads);
        var buffer = Warm(w);
        var scratch = new byte[buffer.Length];

        long Measure(Action a)
        {
            a();
            long before = GC.GetAllocatedBytesForCurrentThread();
            for (int i = 0; i < 20; i++) { a(); }
            return GC.GetAllocatedBytesForCurrentThread() - before;
        }

        Assert.Equal(0, Measure(() => w.ComputeHash()));
        Assert.Equal(0, Measure(() => w.GetSnapshotSize()));
        Assert.Equal(0, Measure(() => w.TrySnapshot(scratch, out _)));
        Assert.Equal(0, Measure(() => w.Step(Dt)));
        w.TrySnapshot(scratch, out int len);
        Assert.Equal(0, Measure(() => w.Restore(scratch.AsSpan(0, len))));

        // A whole host tick: hash, snapshot, step, and now and then a rollback.
        long total = Measure(() =>
        {
            w.ComputeHash();
            w.TrySnapshot(scratch, out int n);
            w.Step(Dt);
            w.Restore(scratch.AsSpan(0, n));
            w.Step(Dt);
        });
        Assert.Equal(0, total);
    }

    [Fact]
    public void SpanBytesEqualArrayBytesAndRoundTripContinuesBitIdentically()
    {
        var w = Build(1);
        for (int i = 0; i < 80; i++) { w.Step(Dt); }
        byte[] arr = w.Snapshot();
        int size = w.GetSnapshotSize();
        Assert.Equal(arr.Length, size);
        var span = new byte[size + 10];
        Assert.True(w.TrySnapshot(span, out int n));
        Assert.Equal(size, n);
        Assert.True(span.AsSpan(0, n).SequenceEqual(arr));

        var hashes = new ulong[100];
        for (int i = 0; i < 100; i++) { w.Step(Dt); hashes[i] = w.ComputeHash(); }
        byte[] end = w.Snapshot();

        w.Restore(span.AsSpan(0, n));
        Assert.Equal(arr, w.Snapshot());
        for (int i = 0; i < 100; i++) { w.Step(Dt); Assert.Equal(hashes[i], w.ComputeHash()); }
        Assert.Equal(end, w.Snapshot());

        var fresh = Build(1);
        fresh.Restore(arr);
        for (int i = 0; i < 100; i++) { fresh.Step(Dt); Assert.Equal(hashes[i], fresh.ComputeHash()); }
        Assert.Equal(end, fresh.Snapshot());
    }

    [Fact]
    public void ATooSmallDestinationReturnsFalseAndWritesNothing()
    {
        var w = Build(1);
        for (int i = 0; i < 10; i++) { w.Step(Dt); }
        int size = w.GetSnapshotSize();
        var big = new byte[size + 32];
        Array.Fill(big, (byte)0xAB);
        var window = big.AsSpan(16, size - 1);
        Assert.False(w.TrySnapshot(window, out int needed));
        Assert.Equal(size, needed);
        foreach (byte b in big) { Assert.Equal(0xAB, b); }
        Assert.False(w.TrySnapshot(Span<byte>.Empty, out needed));
        Assert.Equal(size, needed);
        Assert.True(w.TrySnapshot(big.AsSpan(16, size), out needed));
        Assert.Equal(0xAB, big[15]);
        Assert.Equal(0xAB, big[16 + size]);
    }

    [Fact]
    public void CorruptOrTruncatedInputLeavesTheWorldUnchanged()
    {
        var w = Build(1);
        for (int i = 0; i < 60; i++) { w.Step(Dt); }
        byte[] good = w.Snapshot();
        for (int i = 0; i < 20; i++) { w.Step(Dt); }
        byte[] before = w.Snapshot();
        ulong hash = w.ComputeHash();

        var rng = new Random(11);
        int failures = 0;
        for (int trial = 0; trial < 300; trial++)
        {
            var bad = (byte[])good.Clone();
            int len = bad.Length;
            switch (trial % 3)
            {
                case 0: len = rng.Next(0, bad.Length); break;
                case 1: bad[rng.Next(bad.Length)] ^= (byte)(1 << rng.Next(8)); break;
                default: for (int k = 0; k < 8; k++) { bad[rng.Next(bad.Length)] = (byte)rng.Next(256); } break;
            }

            try { w.Restore(bad.AsSpan(0, len)); }
            catch (SnapshotFormatException) { failures++; }

            // Whatever happened, the world is either exactly as before or exactly the (valid) changed one; a failure never leaves a mixture.
            if (w.ComputeHash() != hash) { w.Restore(before); }
            Assert.Equal(hash, w.ComputeHash());
        }

        Assert.True(failures > 100);
        Assert.Equal(before, w.Snapshot());
    }

    [Fact]
    public void AFailedRestoreLeavesTheWorldExactlyAsItWas()
    {
        var w = Build(1);
        for (int i = 0; i < 60; i++) { w.Step(Dt); }
        byte[] good = w.Snapshot();
        for (int i = 0; i < 20; i++) { w.Step(Dt); }
        byte[] before = w.Snapshot();
        for (int cut = 0; cut < good.Length; cut += Math.Max(1, good.Length / 97))
        {
            Assert.Throws<SnapshotFormatException>(() => w.Restore(good.AsSpan(0, cut)));
            Assert.Equal(before, w.Snapshot());
        }

        var trailing = new byte[good.Length + 1];
        good.CopyTo(trailing, 0);
        Assert.Throws<SnapshotFormatException>(() => w.Restore(trailing));
        Assert.Equal(before, w.Snapshot());
        Assert.Throws<ArgumentNullException>(() => w.Restore((byte[])null!));
    }
}
