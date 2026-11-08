using System;
using System.Security.Cryptography;
using RecursiveAI.OneMath;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests.Api;

public class SnapshotEncodingTests
{
    [Fact]
    public void ArbitraryBytesRoundTripIncludingLengthChanges()
    {
        var random = new Random(93);
        for (int n = 0; n < 300; n++)
        {
            var a = new byte[random.Next(4096)];
            random.NextBytes(a);
            var b = new byte[random.Next(4096)];
            a.AsSpan(0, Math.Min(a.Length, b.Length)).CopyTo(b);
            for (int i = 0; i < b.Length; i += random.Next(1, 40)) { b[i] ^= (byte)random.Next(256); }
            Assert.Equal(b, SnapshotEncoding.ApplyDelta(a, SnapshotEncoding.CreateDelta(a, b)));
            Assert.Equal(b, SnapshotEncoding.ApplyDelta(a, SnapshotEncoding.CreateXorDelta(a, b)));
            Assert.Equal(b, SnapshotEncoding.Decompress(SnapshotEncoding.Compress(b, true)));
            Assert.Equal(b, SnapshotEncoding.Decompress(SnapshotEncoding.Compress(b)));
            var delta = SnapshotEncoding.CreateDelta(a, b);
            Assert.Equal(b, SnapshotEncoding.ApplyDelta(a, SnapshotEncoding.Decompress(SnapshotEncoding.Compress(delta))));
        }
    }

    [Fact]
    public void RepeatedAndLiteralPacketBoundaries()
    {
        foreach (int n in new[] { 0, 1, 2, 3, 127, 128, 129, 130, 131, 260, 4096 })
        {
            var bytes = new byte[n];
            Assert.Equal(bytes, SnapshotEncoding.Decompress(SnapshotEncoding.Compress(bytes)));
            for (int i = 0; i < n; i++) { bytes[i] = (byte)i; }
            Assert.Equal(bytes, SnapshotEncoding.Decompress(SnapshotEncoding.Compress(bytes)));
            Assert.Equal(bytes, SnapshotEncoding.ApplyDelta(Array.Empty<byte>(), SnapshotEncoding.CreateDelta(Array.Empty<byte>(), bytes)));
            Assert.Empty(SnapshotEncoding.ApplyDelta(bytes, SnapshotEncoding.CreateDelta(bytes, Array.Empty<byte>())));
        }
    }

    [Fact]
    public void KnownAnswer()
    {
        // Header + SHA-256 of [0,0,0,1,2,3,3,3,3] + run(3,0), literals(1,2), run(4,3).
        byte[] bytes = { 0, 0, 0, 1, 2, 3, 3, 3, 3 };
        Assert.Equal("4F5041430100000009000000" + Convert.ToHexString(SHA256.HashData(bytes)) + "80000101028103",
            Convert.ToHexString(SnapshotEncoding.Compress(bytes)));
        Assert.Equal("0A88E18087CA6EAAC01CA0C8579A4207C0E5A86A2E603E96AC54284EEB917D7E", Convert.ToHexString(SHA256.HashData(SnapshotEncoding.Compress(bytes))));
    }

    [Fact]
    public void WrongBaseCorruptionTruncationAndSizeLimitsAreTyped()
    {
        byte[] a = new byte[1024], b = new byte[2048];
        b[4] = 3;
        var delta = SnapshotEncoding.CreateDelta(a, b);
        a[0] = 1;
        Assert.Throws<SnapshotEncodingException>(() => SnapshotEncoding.ApplyDelta(a, delta));
        a[0] = 0;
        for (int i = 0; i < delta.Length; i++)
        {
            int size = i;
            Assert.Throws<SnapshotEncodingException>(() => SnapshotEncoding.ApplyDelta(a, delta.AsSpan(0, size)));
        }
        var compressed = SnapshotEncoding.Compress(b);
        for (int i = 0; i < compressed.Length; i++)
        {
            int size = i;
            Assert.Throws<SnapshotEncodingException>(() => SnapshotEncoding.Decompress(compressed.AsSpan(0, size)));
        }
        Assert.Throws<SnapshotEncodingException>(() => SnapshotEncoding.Decompress(compressed, 2047));
        Assert.Throws<SnapshotEncodingException>(() => SnapshotEncoding.ApplyDelta(a, delta, 2047));
        compressed[^1] ^= 1;
        Assert.Throws<SnapshotEncodingException>(() => SnapshotEncoding.Decompress(compressed));
    }

    [Fact]
    public void WorldsRoundTripAcrossStepsAndSlotReuse()
    {
        var w2 = new P2.PhysicsWorld2D();
        var w3 = new P3.PhysicsWorld3D();
        var b2 = w2.AddBox(new Vector2F(1, 1), new Vector2D(0, 5));
        var b3 = w3.AddBox(new Vector3F(1, 1, 1), new Vector3D(0, 5, 0));
        w2.AddCharacter(new Vector2D(5, 5));
        w3.AddCharacter(new Vector3D(5, 5, 0));
        var a2 = w2.Snapshot();
        var a3 = w3.Snapshot();
        for (int i = 0; i < 40; i++)
        {
            if (i == 10) { b2.Destroy(); b3.Destroy(); }
            if (i == 20) { w2.AddBox(new Vector2F(2, 1), new Vector2D(0, 9)); w3.AddBox(new Vector3F(2, 1, 1), new Vector3D(0, 9, 0)); }
            w2.Step(1f / 60); w3.Step(1f / 60);
            var full2 = w2.Snapshot(); var full3 = w3.Snapshot();
            Assert.Equal(full2, SnapshotEncoding.ApplyDelta(a2, w2.SnapshotDelta(a2)));
            Assert.Equal(full3, SnapshotEncoding.ApplyDelta(a3, w3.SnapshotDelta(a3)));
            var r2 = new P2.PhysicsWorld2D(); var r3 = new P3.PhysicsWorld3D();
            r2.RestoreCompressed(w2.SnapshotCompressed()); r3.RestoreDelta(a3, w3.SnapshotDelta(a3));
            Assert.Equal(full2, r2.Snapshot()); Assert.Equal(full3, r3.Snapshot());
        }
    }
    [Fact]
    public void OptionalSectionsRoundTripWithoutInterpretation()
    {
        var w = new P3.PhysicsWorld3D();
        w.AddSoftBall(0.4f, new Vector3D(5, 5, 0), segments: 2);
        w.AddCloth(1, 1, 3, new Vector3D(10, 5, 0));
        var chassis = w.AddBox(new Vector3F(1.8f, 0.5f, 4.2f), new Vector3D(15, 5, 0), density: 400);
        w.AddVehicle(chassis, P3.VehicleDef.Car);
        w.AddRagdoll(P3.RagdollDef.Humanoid, new Vector3D(20, 5, 0));
        w.AddCharacter(new Vector3D(25, 5, 0));
        var original = w.Snapshot();
        for (int i = 0; i < 30; i++)
        {
            w.Step(1f / 60);
            var expected = w.Snapshot();
            var decoded = SnapshotEncoding.ApplyDelta(original, SnapshotEncoding.Decompress(SnapshotEncoding.Compress(w.SnapshotDelta(original))));
            Assert.Equal(expected, decoded);
            var restored = new P3.PhysicsWorld3D();
            restored.Restore(decoded);
            Assert.Equal(expected, restored.Snapshot());
        }
    }


    [Fact]
    public void VersionTwoKnownAnswers()
    {
        var a = new byte[256]; for (int i = 0; i < a.Length; i++) { a[i] = (byte)i; }
        var b = (byte[])a.Clone(); b[3] = 200; b[9] = 0;
        var xor = SnapshotEncoding.CreateXorDelta(a, b);
        Assert.Equal("E7603DAF271CE709F25513256218C6B7AA788CE935D2BB6A306547A626A04638", Convert.ToHexString(SHA256.HashData(xor)));
        var compressed = SnapshotEncoding.Compress(a, true);
        Assert.Equal("3FC6B2210AE73007E063BCDF02A845A0D7608AABE4D2707E828DD8C7302A05A8", Convert.ToHexString(SHA256.HashData(compressed)));
        Assert.Equal(b, SnapshotEncoding.ApplyDelta(a, xor)); Assert.Equal(a, SnapshotEncoding.Decompress(compressed));
    }
}
