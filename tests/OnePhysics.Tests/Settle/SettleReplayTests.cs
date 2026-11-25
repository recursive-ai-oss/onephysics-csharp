using System;
using System.Collections.Generic;
using System.IO;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Replay;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests.Settle;

public sealed class SettleReplayTests
{
    private static P3.PhysicsWorld3D Pile3()
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 301 });
        w.AddBox(new(40, 1, 40), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        for (int i = 0; i < 6; i++) w.AddBox(new(1, 1, 1), new Vector3D(0.1 * i, 0.5 + 1.3 * i, 0));
        return w;
    }

    private static P2.PhysicsWorld2D Pile2()
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 302 });
        w.AddBox(new(40, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        for (int i = 0; i < 6; i++) w.AddBox(new(1, 1), new Vector2D(0.1 * i, 0.5 + 1.3 * i));
        return w;
    }

    [Fact]
    public void ASpatialSettleReplaysWithEveryHashAndTheReportEqual()
    {
        var w = Pile3();
        var hashes = new List<ulong>();
        using var stream = new MemoryStream();
        SettleReport report;
        using (var recorder = w.StartRecording(stream))
        {
            for (int i = 0; i < 10; i++) { w.Step(1f / 60); hashes.Add(w.ComputeHash()); }
            report = w.Settle(new P3.SettleOptions3D { ExtraLinearDamping = 0.5f });
            hashes.Add(w.ComputeHash());
            for (int i = 0; i < 10; i++) { w.Step(1f / 60); hashes.Add(w.ComputeHash()); }
        }
        Assert.True(report.Converged);
        stream.Position = 0;
        var reader = new ReplayReader(stream);
        int settles = 0;
        while (reader.ReadNext()) { if (reader.Kind == ReplayRecordKind.Settle) { settles++; Assert.Equal(report.StepsUsed + 10, reader.CompletedSteps); } }
        Assert.Equal(1, settles);
        stream.Position = 0;
        var player = new ReplayPlayer(stream);
        var copy = player.CreateWorld3D();
        int steps = 0;
        while (player.ReadNextStep(copy)) { steps++; }
        Assert.Equal(20, steps);
        Assert.Equal(w.ComputeHash(), copy.ComputeHash());
        Assert.Equal(w.StepCount, copy.StepCount);
    }

    [Fact]
    public void APlanarSettleReplaysWithEveryHashAndTheReportEqual()
    {
        var w = Pile2();
        using var stream = new MemoryStream();
        using (var recorder = w.StartRecording(stream))
        {
            for (int i = 0; i < 5; i++) w.Step(1f / 60);
            Assert.True(w.Settle().Converged);
            w.Step(1f / 60);
        }
        stream.Position = 0;
        var player = new ReplayPlayer(stream);
        var copy = player.CreateWorld2D();
        while (player.ReadNextStep(copy)) { }
        Assert.Equal(w.ComputeHash(), copy.ComputeHash());
    }

    [Fact]
    public void AReplayThatDisagreesWithItsSettleReportIsRejected()
    {
        var w = Pile3();
        using var stream = new MemoryStream();
        using (var recorder = w.StartRecording(stream)) { w.Step(1f / 60); w.Settle(); }
        var bytes = stream.ToArray();
        long at = -1;
        using (var scan = new MemoryStream(bytes))
        {
            var reader = new ReplayReader(scan);
            while (reader.ReadNext()) { if (reader.Kind == ReplayRecordKind.Settle) at = reader.RecordOffset; }
        }
        Assert.True(at > 0);
        // The report block starts after the 5-byte header, the 8-byte count, two length fields and the options block.
        int reportStart = (int)at + 5 + 8 + 4 + 88 + 4;
        bytes[reportStart + 4] ^= 1;
        var player = new ReplayPlayer(new MemoryStream(bytes));
        var copy = player.CreateWorld3D();
        Assert.Throws<ReplayFormatException>(() => { while (player.ReadNextStep(copy)) { } });
    }

    [Fact]
    public void AVersionOneReplayRejectsASettleRecord()
    {
        var header = new byte[16];
        BitConverter.TryWriteBytes(header, 0x5250504fu); header[4] = 1; header[6] = 3;
        var record = new byte[5 + 40]; record[0] = (byte)ReplayRecordKind.Settle; BitConverter.TryWriteBytes(record.AsSpan(1), 40);
        var snapshotRecord = new byte[5 + 12]; snapshotRecord[0] = 1; BitConverter.TryWriteBytes(snapshotRecord.AsSpan(1), 12);
        var all = new byte[16 + snapshotRecord.Length + record.Length];
        header.CopyTo(all, 0); snapshotRecord.CopyTo(all, 16); record.CopyTo(all, 16 + snapshotRecord.Length);
        var reader = new ReplayReader(new MemoryStream(all));
        Assert.True(reader.ReadNext());
        Assert.Throws<ReplayFormatException>(() => reader.ReadNext());
    }

    [Fact]
    public void HistorySeekAcrossASettleReproducesTheRecordedHashes()
    {
        var w = Pile3();
        var hashes = new List<ulong> { w.ComputeHash() };
        using var history = w.StartHistory(new HistoryOptions { KeyframeInterval = 8 });
        for (int i = 0; i < 6; i++) { w.Step(1f / 60); hashes.Add(w.ComputeHash()); }
        w.Settle();
        for (int i = 0; i < 6; i++) { w.Step(1f / 60); hashes.Add(w.ComputeHash()); }
        var copy = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 301 });
        // Frames before the settle replay as recorded; frames after it need the settle applied first.
        history.Seek(6, copy);
        Assert.Equal(hashes[6], copy.ComputeHash());
        history.Seek(7, copy);
        Assert.Equal(hashes[7], copy.ComputeHash());
        history.Seek(10, copy);
        Assert.Equal(hashes[10], copy.ComputeHash());
        history.Seek(history.LastStep, copy);
        Assert.Equal(w.ComputeHash(), copy.ComputeHash());
    }
}
