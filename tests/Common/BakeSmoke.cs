using System;
using RecursiveAI.OnePhysics.Bake;
using RecursiveAI.OnePhysics.Tests.Common;

// A bake of each world must be byte-identical at one and three threads and must read back: poses, points, fractures and events.
internal static class BakeSmoke
{
    internal static bool Run(out ulong digest)
    {
        var (a, ca) = Spatial(1);
        var (b, _) = Spatial(3);
        var (c, cc) = Planar(1);
        var (d, _) = Planar(3);
        digest = ChecksumOf(a) ^ (ChecksumOf(c) * 31);
        return a.AsSpan().SequenceEqual(b) && c.AsSpan().SequenceEqual(d) && ca && cc;
    }

    private static ulong ChecksumOf(byte[] file) => BitConverter.ToUInt64(file, file.Length - 8);

    private static (byte[], bool) Spatial(int threads)
    {
        var w = BakeScenes.Spatial(threads);
        using var rec = w.BeginBake(new BakeOptions { LibraryVersion = "smoke" });
        for (int i = 0; i < 90; i++) { w.Step(BakeScenes.Dt); }
        var file = rec.Finish();
        return (file, Reads(BakeClip.Parse(file), 3));
    }

    private static (byte[], bool) Planar(int threads)
    {
        var w = BakeScenes.Planar(threads);
        using var rec = w.BeginBake(new BakeOptions { LibraryVersion = "smoke" });
        for (int i = 0; i < 90; i++) { w.Step(BakeScenes.Dt); }
        var file = rec.Finish();
        return (file, Reads(BakeClip.Parse(file), 2));
    }

    private static bool Reads(BakeClip clip, int dimension)
    {
        if (clip.Dimension != dimension || clip.FrameCount != 91 || clip.Rigid.Count == 0 || clip.Soft.Count == 0 || clip.Rods.Count == 0 || clip.Particles.Count == 0 || clip.Fractures.Count == 0 || clip.Events.Count == 0) { return false; }
        double t = clip.TimeOfFrame(40) + clip.FrameDuration * 0.5;
        foreach (var e in clip.Rigid) { if (e.ExistsAtFrame(40) && !e.TrySampleRigid(t, out _)) { return false; } }
        var pts = new RecursiveAI.OneMath.Vector3D[256];
        foreach (var e in clip.Soft) { if (e.PointCount <= pts.Length && e.ExistsAtFrame(40) && !e.TrySamplePoints(t, pts)) { return false; } }
        return true;
    }
}
