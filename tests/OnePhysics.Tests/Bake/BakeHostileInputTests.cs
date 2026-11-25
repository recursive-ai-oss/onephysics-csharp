using System;
using System.IO;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Bake;
using RecursiveAI.OnePhysics.Tests.Common;
using Xunit;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests.Bake;

public sealed class BakeHostileInputTests
{
    private static byte[] Sample()
    {
        var w = BakeScenes.Spatial(1);
        using var rec = w.BeginBake(new BakeOptions { LibraryVersion = "test", BlockFrames = 16 });
        for (int i = 0; i < 40; i++) { w.Step(BakeScenes.Dt); }
        return rec.Finish();
    }

    private static void Exercise(byte[] bytes)
    {
        try
        {
            var clip = BakeClip.Parse(bytes, verifyChecksum: false);
            var pts = new Vector3D[Math.Min(65536, 1 << 12)];
            foreach (var track in new[] { clip.Rigid, clip.Soft, clip.Rods, clip.Particles })
            {
                foreach (var e in track)
                {
                    for (double t = 0; t <= clip.Duration + clip.FrameDuration; t += clip.FrameDuration * 3.3)
                    {
                        if (e.FrameCount > 0 && e.PointCount <= pts.Length)
                        {
                            if (track == clip.Rigid) { e.TrySampleRigid(t, out _); } else { e.TrySamplePoints(t, pts); }
                        }
                    }
                }
            }
        }
        catch (BakeFormatException) { }
    }

    [Fact]
    public void EveryTruncationIsATypedError()
    {
        var file = Sample();
        for (int n = 0; n < file.Length; n += Math.Max(1, file.Length / 400))
        {
            var cut = file.AsSpan(0, n).ToArray();
            Assert.Throws<BakeFormatException>(() => BakeClip.Parse(cut));
        }
    }

    [Fact]
    public void FlippedBytesNeverEscapeAsAnotherException()
    {
        var file = Sample();
        var rng = new Random(7);
        for (int i = 0; i < 3000; i++)
        {
            var m = (byte[])file.Clone();
            int flips = 1 + rng.Next(4);
            for (int k = 0; k < flips; k++) { m[rng.Next(m.Length)] ^= (byte)(1 << rng.Next(8)); }
            Exercise(m);
        }
    }

    [Fact]
    public void ABadChecksumAndBadMagicAreNamed()
    {
        var file = Sample();
        var bad = (byte[])file.Clone();
        bad[100] ^= 1;
        var ex = Assert.Throws<BakeFormatException>(() => BakeClip.Parse(bad));
        Assert.Equal("checksum", ex.Field);
        var magic = (byte[])file.Clone(); magic[0] = 0;
        Assert.Equal("magic", Assert.Throws<BakeFormatException>(() => BakeClip.Parse(magic)).Field);
        Assert.Throws<BakeFormatException>(() => BakeClip.Read(new MemoryStream(Array.Empty<byte>())));
    }

    [Fact]
    public void AbsurdCountsAndRangesAreRejectedByName()
    {
        var file = Sample();
        // Frame count (offset 12) of 2^30, a stride of 0 (offset 16), a NaN dt (offset 24).
        var a = (byte[])file.Clone(); BitConverter.GetBytes(1u << 30).CopyTo(a, 12);
        Assert.Equal("frame count", Assert.Throws<BakeFormatException>(() => BakeClip.Parse(a, false)).Field);
        var b = (byte[])file.Clone(); BitConverter.GetBytes(0u).CopyTo(b, 16);
        Assert.Equal("frame stride", Assert.Throws<BakeFormatException>(() => BakeClip.Parse(b, false)).Field);
        var c = (byte[])file.Clone(); BitConverter.GetBytes(float.NaN).CopyTo(c, 24);
        Assert.Equal("dt", Assert.Throws<BakeFormatException>(() => BakeClip.Parse(c, false)).Field);
        var d = (byte[])file.Clone(); d[4] = 9;
        Assert.Equal("format version", Assert.Throws<BakeFormatException>(() => BakeClip.Parse(d, false)).Field);
    }

    [Fact]
    public void AnUnknownChunkFromALaterVersionIsSkipped()
    {
        var file = Sample();
        // Insert a chunk "XTRA" before the end chunk (the last 16 bytes) and fix the checksum.
        int endAt = file.Length - 16;
        var extra = new byte[] { (byte)'X', (byte)'T', (byte)'R', (byte)'A', 3, 0, 0, 0, 9, 9, 9 };
        var body = new byte[endAt + extra.Length];
        Array.Copy(file, body, endAt); extra.CopyTo(body, endAt);
        ulong h = 14695981039346656037UL;
        foreach (byte x in body) { h ^= x; h *= 1099511628211UL; }
        var end = new byte[16];
        BitConverter.GetBytes(0x20444E45u).CopyTo(end, 0); BitConverter.GetBytes(8u).CopyTo(end, 4); BitConverter.GetBytes(h).CopyTo(end, 8);
        var all = new byte[body.Length + 16];
        body.CopyTo(all, 0); end.CopyTo(all, body.Length);
        var clip = BakeClip.Parse(all);
        Assert.NotEmpty(clip.Rigid);
    }
}
