using System;
using System.Buffers.Binary;
using System.IO;
using RecursiveAI.OnePhysics.Replay;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class ReplayTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void StreamingRecordsRoundTripWithOptionalHashTrack(bool hashes)
    {
        using var output = new MemoryStream();
        var writer = new ReplayWriter(output, 3, 17, new byte[] { 5, 6, 7 }, hashes);
        writer.WriteStep(1f / 60, 0xfedcba9876543210UL);
        writer.WriteSnapshot(new byte[] { 8, 9 });
        writer.WriteStep(0.125f, 23);
        writer.Complete();
        writer.Complete();
        Assert.Equal(19, writer.CompletedSteps);
        output.WriteByte(42); // A following object must not be consumed.
        output.Position = 0;
        using var fragmented = new FragmentedStream(output);
        var reader = new ReplayReader(fragmented, "test");
        Assert.Equal(3, reader.Dimension);
        Assert.Equal(17, reader.InitialStep);
        Assert.Equal(hashes, reader.HasHashTrack);
        Assert.True(reader.ReadNext());
        Assert.Equal(ReplayRecordKind.Snapshot, reader.Kind);
        Assert.Equal(new byte[] { 5, 6, 7 }, reader.Payload.ToArray());
        Assert.Throws<InvalidOperationException>(() => reader.StepTime);
        Assert.True(reader.ReadNext());
        Assert.Equal(18, reader.CompletedSteps);
        Assert.Equal(1f / 60, reader.StepTime);
        Assert.Equal(hashes ? 0xfedcba9876543210UL : (ulong?)null, reader.StepHash);
        Assert.True(reader.ReadNext());
        Assert.Equal(new byte[] { 8, 9 }, reader.Payload.ToArray());
        Assert.True(reader.ReadNext());
        Assert.Equal(0.125f, reader.StepTime);
        Assert.Equal(hashes ? 23UL : (ulong?)null, reader.StepHash);
        Assert.False(reader.ReadNext());
        Assert.False(reader.ReadNext());
        Assert.Equal(42, output.ReadByte());
    }

    [Fact]
    public void EveryTruncatedPrefixIsAFormatErrorWithInputAndOffset()
    {
        byte[] bytes = Example();
        for (int length = 0; length < bytes.Length; length++)
        {
            using var input = new MemoryStream(bytes, 0, length, false);
            ReplayFormatException ex = Assert.Throws<ReplayFormatException>(() => Consume(input));
            Assert.Equal("truncated", ex.InputName);
            Assert.Equal(length, ex.Offset);
        }
    }

    [Fact]
    public void EnvelopeBytesAreLittleEndianAndCanonical()
    {
        using var output = new MemoryStream();
        var writer = new ReplayWriter(output, 2, 0, new byte[] { 0xab });
        writer.WriteStep(0.5f, 0x0807060504030201UL);
        writer.Complete();
        Assert.Equal("4F5050520100020100000000000000000101000000AB020C0000000000003F01020304050607080000000000", Convert.ToHexString(output.ToArray()));
    }

    [Theory]
    [InlineData(0, 0)] // magic
    [InlineData(4, 3)] // version
    [InlineData(6, 4)] // dimension
    [InlineData(7, 2)] // flags
    [InlineData(15, 128)] // negative initial step
    [InlineData(16, 2)] // step before snapshot
    [InlineData(16, 3)] // unknown kind
    [InlineData(17, 0)] // empty initial snapshot
    [InlineData(20, 128)] // negative payload size
    [InlineData(22, 3)] // unknown second kind
    [InlineData(23, 11)] // invalid step length
    [InlineData(30, 255)] // negative step
    [InlineData(40, 1)] // nonempty end
    public void MalformedFieldsAreTypedErrors(int offset, byte value)
    {
        byte[] bytes = Example();
        bytes[offset] = value;
        using var input = new MemoryStream(bytes);
        Assert.Throws<ReplayFormatException>(() => Consume(input));
    }

    [Fact]
    public void ReaderRejectsNonfiniteZeroAndNegativeTimes()
    {
        foreach (float dt in new[] { 0f, -0f, -1f, float.NaN, float.PositiveInfinity, float.NegativeInfinity })
        {
            byte[] bytes = Example();
            BinaryPrimitives.WriteSingleLittleEndian(bytes.AsSpan(27), dt);
            using var input = new MemoryStream(bytes);
            var ex = Assert.Throws<ReplayFormatException>(() => Consume(input));
            Assert.Equal(27, ex.Offset);
        }
    }

    [Fact]
    public void DeclaredSizeIsBoundedBeforePayloadAllocationAndReaderCannotContinue()
    {
        byte[] bytes = Example();
        BinaryPrimitives.WriteInt32LittleEndian(bytes.AsSpan(17), int.MaxValue);
        using var input = new MemoryStream(bytes);
        var reader = new ReplayReader(input, "oversized", 12);
        var ex = Assert.Throws<ReplayFormatException>(() => reader.ReadNext());
        Assert.Equal(17, ex.Offset);
        Assert.Equal(21, input.Position);
        Assert.Throws<InvalidOperationException>(() => reader.ReadNext());
        Assert.True(reader.Payload.IsEmpty);
    }

    [Fact]
    public void WriterRejectsBadArgumentsBeforeWriting()
    {
        using var output = new MemoryStream();
        Assert.Throws<ArgumentOutOfRangeException>(() => new ReplayWriter(output, 4, 0, new byte[] { 1 }));
        Assert.Equal(0, output.Length);
        var writer = new ReplayWriter(output, 2, 0, new byte[] { 1 });
        long before = output.Length;
        foreach (float dt in new[] { 0f, -1f, float.NaN, float.PositiveInfinity })
        {
            Assert.Throws<ArgumentOutOfRangeException>(() => writer.WriteStep(dt));
            Assert.Equal(before, output.Length);
        }
        writer.Complete();
        Assert.Throws<InvalidOperationException>(() => writer.WriteStep(1));
        Assert.Throws<InvalidOperationException>(() => writer.WriteSnapshot(new byte[] { 2 }));
    }

    [Fact]
    public void StepCountOverflowIsRejectedByReaderAndWriter()
    {
        using var output = new MemoryStream();
        var writer = new ReplayWriter(output, 2, long.MaxValue, new byte[] { 1 });
        long size = output.Length;
        Assert.Throws<InvalidOperationException>(() => writer.WriteStep(1));
        Assert.Equal(size, output.Length);
        byte[] bytes = Example();
        BinaryPrimitives.WriteInt64LittleEndian(bytes.AsSpan(8), long.MaxValue);
        using var input = new MemoryStream(bytes);
        Assert.Throws<ReplayFormatException>(() => Consume(input));
    }

    [Fact]
    public void Unchanged2DSessionPlaysBackWithEveryHashEqual()
    {
        var world = new PhysicsWorld2D();
        world.AddBox(new(1, 1), at: new(0, 4));
        using var output = new MemoryStream();
        var writer = new ReplayWriter(output, 2, world.StepCount, world.Snapshot());
        for (int i = 0; i < 40; i++) { world.Step(1f / 60); writer.WriteStep(1f / 60, world.ComputeHash()); }
        writer.Complete();
        output.Position = 0;
        var reader = new ReplayReader(output);
        var playback = new PhysicsWorld2D();
        while (reader.ReadNext())
        {
            if (reader.Kind == ReplayRecordKind.Snapshot) { playback.Restore(reader.Payload); }
            else { playback.Step(reader.StepTime); Assert.Equal(reader.StepHash, playback.ComputeHash()); }
            Assert.Equal(reader.CompletedSteps, playback.StepCount);
        }
        Assert.Equal(world.Snapshot(), playback.Snapshot());
    }

    [Fact]
    public void CheckpointsCarry3DCreationDestructionAndOriginChangesExactly()
    {
        var world = new PhysicsWorld3D();
        var body = world.AddBox(new(1, 1, 1), at: new(0, 4, 0));
        using var output = new MemoryStream();
        var writer = new ReplayWriter(output, 3, world.StepCount, world.Snapshot());
        for (int i = 0; i < 40; i++)
        {
            if (i == 10) { body.Destroy(); world.AddSphere(0.5f, at: new(1, 3, 0)); writer.WriteSnapshot(world.Snapshot()); }
            if (i == 20) { world.ShiftOrigin(new(1, 0, 0)); writer.WriteSnapshot(world.Snapshot()); }
            world.Step(1f / 60);
            writer.WriteStep(1f / 60, world.ComputeHash());
        }
        writer.Complete();
        output.Position = 0;
        var reader = new ReplayReader(output);
        var playback = new PhysicsWorld3D();
        while (reader.ReadNext())
        {
            if (reader.Kind == ReplayRecordKind.Snapshot) { playback.Restore(reader.Payload); }
            else { playback.Step(reader.StepTime); Assert.Equal(reader.StepHash, playback.ComputeHash()); }
            Assert.Equal(reader.CompletedSteps, playback.StepCount);
        }
        Assert.Equal(world.Snapshot(), playback.Snapshot());
    }

    [Theory]
    [InlineData(2)]
    [InlineData(3)]
    public void PlayerChecksCommittedFixtureGeneratorHashes(int dimension)
    {
        using var stream = new MemoryStream(FuzzReplay.Seed(dimension));
        var player = new ReplayPlayer(stream);
        int count = 0;
        if (dimension == 2)
        {
            var world = new PhysicsWorld2D();
            while (player.ReadNextStep(world)) { count++; Assert.Equal(count, player.CompletedSteps); }
        }
        else
        {
            var world = new PhysicsWorld3D();
            while (player.ReadNextStep(world)) { count++; Assert.Equal(count, player.CompletedSteps); }
        }
        Assert.Equal(12, count);
    }

    [Fact]
    public void PlayerNamesFirstHashMismatchAndStops()
    {
        byte[] bytes = FuzzReplay.Seed(2);
        using (var scan = new MemoryStream(bytes))
        {
            var reader = new ReplayReader(scan);
            Assert.True(reader.ReadNext());
            Assert.True(reader.ReadNext());
            bytes[(int)reader.RecordOffset + 9] ^= 1;
        }
        using var input = new MemoryStream(bytes);
        var player = new ReplayPlayer(input);
        var world = new PhysicsWorld2D();
        var ex = Assert.Throws<ReplayHashMismatchException>(() => player.ReadNextStep(world));
        Assert.Equal(1, ex.Step);
        Assert.NotEqual(ex.Expected, ex.Actual);
        Assert.Equal(1, world.StepCount);
        Assert.Throws<InvalidOperationException>(() => player.ReadNextStep(world));
    }

    [Fact]
    public void PlayerRejectsWrongWorldBeforeReadingAndWrapsSnapshotErrors()
    {
        using var input = new MemoryStream(FuzzReplay.Seed(2));
        var player = new ReplayPlayer(input);
        Assert.Throws<ArgumentException>(() => player.ReadNextStep(new PhysicsWorld3D()));
        var world = new PhysicsWorld2D();
        Assert.True(player.ReadNextStep(world));
        Assert.Throws<ArgumentException>(() => player.ReadNextStep(new PhysicsWorld2D()));
        Assert.True(player.ReadNextStep(world));
        using var bad = new MemoryStream(Example());
        var broken = new ReplayPlayer(bad, "bad-snapshot");
        ulong before = world.ComputeHash();
        var ex = Assert.Throws<ReplayFormatException>(() => broken.ReadNextStep(world));
        Assert.Equal("bad-snapshot", ex.InputName);
        Assert.Equal(before, world.ComputeHash());
    }

    [Fact]
    public void PlayerRejectsSnapshotWithInconsistentAbsoluteStep()
    {
        byte[] bytes = FuzzReplay.Seed(2);
        BinaryPrimitives.WriteInt64LittleEndian(bytes.AsSpan(8), 1);
        using var stream = new MemoryStream(bytes);
        var player = new ReplayPlayer(stream);
        Assert.Throws<ReplayFormatException>(() => player.ReadNextStep(new PhysicsWorld2D()));
    }

    [Fact]
    public void TransportDoesNotAllocatePerRecordAfterConstruction()
    {
        using var output = new MemoryStream(65536);
        var writer = new ReplayWriter(output, 2, 0, new byte[] { 1 });
        writer.WriteStep(0.125f, 0);
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 1000; i++) { writer.WriteStep(0.125f, (ulong)i); }
        Assert.Equal(0, GC.GetAllocatedBytesForCurrentThread() - before);
        writer.Complete();
        output.Position = 0;
        var reader = new ReplayReader(output);
        Assert.True(reader.ReadNext());
        Assert.True(reader.ReadNext());
        before = GC.GetAllocatedBytesForCurrentThread();
        while (reader.ReadNext()) { }
        Assert.Equal(0, GC.GetAllocatedBytesForCurrentThread() - before);
    }

    private static byte[] Example()
    {
        using var output = new MemoryStream();
        var writer = new ReplayWriter(output, 2, 0, new byte[] { 1 });
        writer.WriteStep(0.5f, 1);
        writer.Complete();
        return output.ToArray();
    }

    private static void Consume(Stream input)
    {
        var reader = new ReplayReader(input, "truncated");
        while (reader.ReadNext()) { }
    }

    private sealed class FragmentedStream(Stream input) : Stream
    {
        public override bool CanRead => true;
        public override bool CanSeek => false;
        public override bool CanWrite => false;
        public override long Length => throw new NotSupportedException();
        public override long Position { get => throw new NotSupportedException(); set => throw new NotSupportedException(); }
        public override void Flush() => throw new NotSupportedException();
        public override int Read(byte[] buffer, int offset, int count) => input.Read(buffer, offset, Math.Min(count, 1));
        public override int Read(Span<byte> buffer) => input.Read(buffer.Slice(0, Math.Min(buffer.Length, 1)));
        public override long Seek(long offset, SeekOrigin origin) => throw new NotSupportedException();
        public override void SetLength(long value) => throw new NotSupportedException();
        public override void Write(byte[] buffer, int offset, int count) => throw new NotSupportedException();
    }
}
