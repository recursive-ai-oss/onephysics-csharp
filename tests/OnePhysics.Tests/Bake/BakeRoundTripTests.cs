using System;
using System.Collections.Generic;
using System.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Bake;
using RecursiveAI.OnePhysics.Fluids;
using RecursiveAI.OnePhysics.Tests.Common;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests.Bake;

public sealed class BakeRoundTripTests
{
    private const float Dt = BakeScenes.Dt;

    // What the world held after each step, read through the public API.
    private sealed class Frame
    {
        public readonly Dictionary<(int, int), (Vector3D P, float[] R)> Bodies = new();
        public readonly Dictionary<(int, int), Vector3D[]> Soft = new(), Rods = new();
        public readonly Dictionary<int, Vector3D> Particles = new();
        public ulong Hash;
    }

    private static Frame Read3(P3.PhysicsWorld3D w)
    {
        var f = new Frame { Hash = w.ComputeHash() };
        foreach (var b in w.Bodies) { var q = b.Rotation; f.Bodies[(b.RawHandle.Index, b.RawHandle.Generation)] = (b.Position, [q.X, q.Y, q.Z, q.W]); }
        for (int i = 0; i < w.Engine.SoftHandles.HighWater; i++)
        {
            if (!w.Engine.SoftHandles.IsLive(i)) { continue; }
            var s = w.GetSoftBody(i); var p = new Vector3D[s.ParticleCount]; s.CopyPositions(p);
            f.Soft[(i, w.Engine.SoftHandles.GenerationOf(i))] = p;
        }

        var store = w.Engine.Rods;
        for (int i = 0; i < store.Data.Length; i++)
        {
            if (store.Data[i] == null) { continue; }
            var r = w.GetRod(i); var p = new Vector3D[r.ParticleCount];
            for (int k = 0; k < p.Length; k++) { p[k] = r.GetPosition(k); }
            f.Rods[(i, store.Generations[i])] = p;
        }

        var all = new FluidParticleInfo[w.FluidParticleCount]; w.GetFluidParticles(all);
        foreach (var p in all) { f.Particles[p.Id] = p.Position; }
        return f;
    }

    private static Frame Read2(P2.PhysicsWorld2D w)
    {
        var f = new Frame { Hash = w.ComputeHash() };
        foreach (var b in w.Bodies) { var q = b.Rotation; var p = b.Position; f.Bodies[(b.RawHandle.Index, b.RawHandle.Generation)] = (new Vector3D(p.X, p.Y, 0), [q.C, q.S]); }
        for (int i = 0; i < w.Engine.SoftHandles.HighWater; i++)
        {
            if (!w.Engine.SoftHandles.IsLive(i)) { continue; }
            var s = w.GetSoftBody(i); var p = new Vector2D[s.ParticleCount]; s.CopyPositions(p);
            f.Soft[(i, w.Engine.SoftHandles.GenerationOf(i))] = p.Select(v => new Vector3D(v.X, v.Y, 0)).ToArray();
        }

        var store = w.Engine.Rods;
        for (int i = 0; i < store.Data.Length; i++)
        {
            if (store.Data[i] == null) { continue; }
            var r = w.GetRod(i); var p = new Vector3D[r.ParticleCount];
            for (int k = 0; k < p.Length; k++) { var v = r.GetPosition(k); p[k] = new Vector3D(v.X, v.Y, 0); }
            f.Rods[(i, store.Generations[i])] = p;
        }

        var all = new FluidParticleInfo[w.FluidParticleCount]; w.GetFluidParticles(all);
        foreach (var p in all) { f.Particles[p.Id] = p.Position; }
        return f;
    }

    private static (byte[] File, List<Frame> Frames) Bake3(BakeOptions o, int steps, int threads = 1)
    {
        var w = BakeScenes.Spatial(threads);
        var frames = new List<Frame> { Read3(w) };
        using var rec = w.BeginBake(o);
        for (int i = 1; i <= steps; i++) { w.Step(Dt); if (i % o.FrameStride == 0) { frames.Add(Read3(w)); } }
        return (rec.Finish(), frames);
    }

    private static (byte[] File, List<Frame> Frames) Bake2(BakeOptions o, int steps, int threads = 1)
    {
        var w = BakeScenes.Planar(threads);
        var frames = new List<Frame> { Read2(w) };
        using var rec = w.BeginBake(o);
        for (int i = 1; i <= steps; i++) { w.Step(Dt); if (i % o.FrameStride == 0) { frames.Add(Read2(w)); } }
        return (rec.Finish(), frames);
    }

    private static BakeOptions Opt(int stride = 1) => new() { FrameStride = stride, LibraryVersion = "test", SceneName = "bake-test" };

    private static void CheckFrames(BakeClip clip, List<Frame> frames, bool rotationIsSigned, double posTol, double vertTol, double rotTol)
    {
        Assert.Equal(frames.Count, clip.FrameCount);
        for (int k = 0; k < frames.Count; k++)
        {
            var f = frames[k];
            double t = clip.TimeOfFrame(k);
            Assert.Equal(f.Hash, clip.GetFrameHash(k));
            // Every body alive at this frame has a sample; none that is dead does.
            int seen = 0;
            foreach (var e in clip.Rigid)
            {
                bool want = f.Bodies.TryGetValue((e.Id.Slot, e.Id.Generation), out var b);
                Assert.Equal(want, e.ExistsAtFrame(k));
                if (!want) { Assert.False(e.TrySampleRigid(t, out _)); continue; }
                seen++;
                Assert.True(e.TrySampleRigid(t, out var pose));
                Assert.InRange(Math.Abs(pose.Position.X - b.P.X), 0, posTol);
                Assert.InRange(Math.Abs(pose.Position.Y - b.P.Y), 0, posTol);
                Assert.InRange(Math.Abs(pose.Position.Z - b.P.Z), 0, posTol);
                float[] r = clip.Dimension == 3 ? [pose.Rotation.X, pose.Rotation.Y, pose.Rotation.Z, pose.Rotation.W] : [pose.Cos, pose.Sin];
                double dot = 0; for (int i = 0; i < r.Length; i++) { dot += r[i] * b.R[i]; }
                double sign = rotationIsSigned && dot < 0 ? -1 : 1;
                for (int i = 0; i < r.Length; i++) { Assert.InRange(Math.Abs(r[i] - sign * b.R[i]), 0, rotTol); }
            }

            Assert.Equal(f.Bodies.Count, seen);
            CheckPoints(clip.Soft, f.Soft, k, t, vertTol);
            CheckPoints(clip.Rods, f.Rods, k, t, vertTol);
            int pseen = 0;
            foreach (var e in clip.Particles)
            {
                bool want = f.Particles.TryGetValue(e.Id.Slot, out var p);
                Assert.Equal(want, e.ExistsAtFrame(k));
                if (!want) { continue; }
                pseen++;
                var one = new Vector3D[1];
                Assert.True(e.TrySamplePoints(t, one));
                Assert.InRange(Math.Abs(one[0].X - p.X), 0, vertTol); Assert.InRange(Math.Abs(one[0].Y - p.Y), 0, vertTol); Assert.InRange(Math.Abs(one[0].Z - p.Z), 0, vertTol);
            }

            Assert.Equal(f.Particles.Count, pseen);
        }
    }

    private static void CheckPoints(IReadOnlyList<BakeEntity> track, Dictionary<(int, int), Vector3D[]> expected, int k, double t, double tol)
    {
        int seen = 0;
        foreach (var e in track)
        {
            bool want = expected.TryGetValue((e.Id.Slot, e.Id.Generation), out var pts);
            Assert.Equal(want, e.ExistsAtFrame(k));
            if (!want) { continue; }
            seen++;
            Assert.Equal(pts!.Length, e.PointCount);
            var got = new Vector3D[e.PointCount];
            Assert.True(e.TrySamplePoints(t, got));
            for (int i = 0; i < got.Length; i++)
            {
                Assert.InRange(Math.Abs(got[i].X - pts[i].X), 0, tol); Assert.InRange(Math.Abs(got[i].Y - pts[i].Y), 0, tol); Assert.InRange(Math.Abs(got[i].Z - pts[i].Z), 0, tol);
            }
        }

        Assert.Equal(expected.Count, seen);
    }

    [Fact]
    public void ThreeDimensionalClipMatchesTheWorldAtEveryFrameWithinTheQuanta()
    {
        var (file, frames) = Bake3(Opt(), 150);
        var clip = BakeClip.Parse(file);
        Assert.Equal(3, clip.Dimension);
        Assert.Equal(BakeTracks.All, clip.Tracks);
        Assert.Equal("bake-test", clip.SceneName);
        Assert.Equal("test", clip.LibraryVersion);
        Assert.NotEmpty(clip.Rigid); Assert.NotEmpty(clip.Soft); Assert.NotEmpty(clip.Rods); Assert.NotEmpty(clip.Particles);
        Assert.Contains(clip.Soft, e => e.IsCloth);
        Assert.Contains(clip.Soft, e => !e.IsCloth && e.SurfaceTriangles.Length > 0);
        CheckFrames(clip, frames, true, Math.Pow(2, -17) + 1e-12, Math.Pow(2, -15) + 1e-12, 2 * Math.Pow(2, -17));
    }

    [Fact]
    public void PlanarClipMatchesTheWorldAtEveryFrameWithinTheQuanta()
    {
        var (file, frames) = Bake2(Opt(), 150);
        var clip = BakeClip.Parse(file);
        Assert.Equal(2, clip.Dimension);
        Assert.Equal(BakeTracks.All, clip.Tracks);
        Assert.NotEmpty(clip.Rigid); Assert.NotEmpty(clip.Soft); Assert.NotEmpty(clip.Rods); Assert.NotEmpty(clip.Particles);
        Assert.Contains(clip.Soft, e => e.IsCloth);
        CheckFrames(clip, frames, false, Math.Pow(2, -17) + 1e-12, Math.Pow(2, -15) + 1e-12, 2 * Math.Pow(2, -17));
    }

    [Fact]
    public void UnquantizedRigidTracksAreExactAtFrameTimes()
    {
        var o = Opt(); o.PositionExponent = -40; o.RotationBits = 0; o.Tracks = BakeTracks.Rigid;
        var (file, frames) = Bake3(o, 100);
        var clip = BakeClip.Parse(file);
        for (int k = 0; k < frames.Count; k++)
        {
            foreach (var e in clip.Rigid)
            {
                if (!frames[k].Bodies.TryGetValue((e.Id.Slot, e.Id.Generation), out var b)) { continue; }
                Assert.True(e.TrySampleRigid(clip.TimeOfFrame(k), out var pose));
                double tol = Math.Pow(2, -41); Assert.InRange(Math.Abs(b.P.X - pose.Position.X), 0, tol); Assert.InRange(Math.Abs(b.P.Y - pose.Position.Y), 0, tol); Assert.InRange(Math.Abs(b.P.Z - pose.Position.Z), 0, tol);
                var q = pose.Rotation;
                float s = q.X * b.R[0] + q.Y * b.R[1] + q.Z * b.R[2] + q.W * b.R[3] < 0 ? -1f : 1f;
                Assert.Equal(b.R[0], s * q.X); Assert.Equal(b.R[1], s * q.Y); Assert.Equal(b.R[2], s * q.Z); Assert.Equal(b.R[3], s * q.W);
            }
        }

        var o2 = Opt(); o2.PositionExponent = -40; o2.RotationBits = 0; o2.Tracks = BakeTracks.Rigid;
        var (file2, frames2) = Bake2(o2, 100);
        var clip2 = BakeClip.Parse(file2);
        for (int k = 0; k < frames2.Count; k++)
        {
            foreach (var e in clip2.Rigid)
            {
                if (!frames2[k].Bodies.TryGetValue((e.Id.Slot, e.Id.Generation), out var b)) { continue; }
                Assert.True(e.TrySampleRigid(clip2.TimeOfFrame(k), out var pose));
                double tol = Math.Pow(2, -41); Assert.InRange(Math.Abs(b.P.X - pose.Position.X), 0, tol); Assert.InRange(Math.Abs(b.P.Y - pose.Position.Y), 0, tol);
                Assert.Equal(b.R[0], pose.Cos); Assert.Equal(b.R[1], pose.Sin);
            }
        }
    }

    [Fact]
    public void AFrameStrideKeepsEveryNthFrameAndAllEvents()
    {
        var (file, frames) = Bake3(Opt(3), 99);
        var clip = BakeClip.Parse(file);
        Assert.Equal(3, clip.FrameStride);
        Assert.Equal(34, clip.FrameCount);
        Assert.Equal(3 * (double)Dt, clip.FrameDuration);
        CheckFrames(clip, frames, true, Math.Pow(2, -17) + 1e-12, Math.Pow(2, -15) + 1e-12, 2 * Math.Pow(2, -17));
        var (full, _) = Bake3(Opt(1), 99);
        var all = BakeClip.Parse(full);
        Assert.Equal(all.Events.Count, clip.Events.Count);
        Assert.Equal(all.Events.Select(e => e.Step), clip.Events.Select(e => e.Step));
    }

    [Fact]
    public void BetweenFramesPositionsAreLinearAndRotationsNormalised()
    {
        var o = Opt(); o.Tracks = BakeTracks.Rigid;
        var (file, _) = Bake3(o, 60);
        var clip = BakeClip.Parse(file);
        foreach (var e in clip.Rigid)
        {
            for (int k = e.FirstFrame; k < e.LastFrame; k += 7)
            {
                Assert.True(e.TrySampleRigid(clip.TimeOfFrame(k), out var a));
                Assert.True(e.TrySampleRigid(clip.TimeOfFrame(k + 1), out var b));
                Assert.True(e.TrySampleRigid((k + 0.25) * clip.FrameDuration, out var m));
                Assert.InRange(Math.Abs(m.Position.Y - (a.Position.Y + 0.25 * (b.Position.Y - a.Position.Y))), 0, 1e-9);
                double len = Math.Sqrt((double)m.Rotation.X * m.Rotation.X + (double)m.Rotation.Y * m.Rotation.Y + (double)m.Rotation.Z * m.Rotation.Z + (double)m.Rotation.W * m.Rotation.W);
                Assert.InRange(len, 1 - 1e-6, 1 + 1e-6);
            }

            // Outside the body's frames there is no sample.
            Assert.False(e.TrySampleRigid(-clip.FrameDuration, out _));
            Assert.False(e.TrySampleRigid(clip.TimeOfFrame(clip.FrameCount), out _));
        }
    }

    [Fact]
    public void FractureTracksNameTheFragmentsTheirActivationFrameAndTheirGeometryOnce()
    {
        var (file3, _) = Bake3(Opt(), 150);
        var c3 = BakeClip.Parse(file3);
        Assert.NotEmpty(c3.Fractures);
        foreach (var f in c3.Fractures)
        {
            Assert.NotEmpty(f.Fragments);
            Assert.False(f.Source.IsNone);
            var src = c3.FindRigid(f.Source.Slot, f.Source.Generation);
            Assert.NotNull(src);
            Assert.True(src!.LastFrame < f.Frame, "the broken body's track ends before the fragments appear");
            foreach (var id in f.Fragments)
            {
                var track = c3.FindRigid(id.Slot, id.Generation);
                Assert.NotNull(track);
                Assert.Equal(f.Frame, track!.FirstFrame);
                var geo = c3.FragmentGeometry.Single(g => g.Body == id);
                Assert.True(geo.Vertices.Length >= 4);
                Assert.True(geo.Triangles.Length >= 12 && geo.Triangles.Length % 3 == 0);
            }
        }

        var (file2, _) = Bake2(Opt(), 150);
        var c2 = BakeClip.Parse(file2);
        Assert.NotEmpty(c2.Fractures);
        foreach (var f in c2.Fractures)
        {
            foreach (var id in f.Fragments)
            {
                var geo = c2.FragmentGeometry.Single(g => g.Body == id);
                Assert.True(geo.Vertices.Length >= 3);
                Assert.Empty(geo.Triangles);
                Assert.Equal(f.Frame, c2.FindRigid(id.Slot, id.Generation)!.FirstFrame);
            }
        }
    }

    [Fact]
    public void EventsAreRecordedInStepOrderWithTheWorldsOwnKinds()
    {
        var w = BakeScenes.Spatial(1);
        using var rec = w.BeginBake(Opt());
        var expected = new List<(int Step, EventKind Kind, int A, int B)>();
        for (int i = 1; i <= 120; i++)
        {
            w.Step(Dt);
            foreach (var e in w.Events) { if ((EventMask.Default & (EventMask)(1 << (int)e.Kind)) != 0) { expected.Add((i, e.Kind, e.BodyA.IsNull ? -1 : e.BodyA.Index, e.BodyB.IsNull ? -1 : e.BodyB.Index)); } }
        }

        var clip = BakeClip.Parse(rec.Finish());
        Assert.NotEmpty(expected);
        Assert.Equal(expected, clip.Events.Select(e => (e.Step, e.Kind, e.BodyA.Slot, e.BodyB.Slot)).ToList());
        Assert.Contains(clip.Events, e => e.Kind == EventKind.ContactBegin);
    }

    [Fact]
    public void ABakeLeavesTheWorldAndItsHashesUntouched()
    {
        var a = BakeScenes.Spatial(1); var b = BakeScenes.Spatial(1);
        using var rec = a.BeginBake(Opt());
        for (int i = 0; i < 90; i++) { a.Step(Dt); b.Step(Dt); Assert.Equal(b.ComputeHash(), a.ComputeHash()); }
        Assert.Equal(b.Snapshot(), a.Snapshot());
    }

    [Fact]
    public void TheBytesDoNotDependOnTheThreadCount()
    {
        var (one, _) = Bake3(Opt(), 90, threads: 1);
        var (four, _) = Bake3(Opt(), 90, threads: 4);
        Assert.Equal(one, four);
        var (p1, _) = Bake2(Opt(), 90, threads: 1);
        var (p4, _) = Bake2(Opt(), 90, threads: 4);
        Assert.Equal(p1, p4);
    }

    [Fact]
    public void ATrackSubsetWritesOnlyThoseTracks()
    {
        var o = Opt(); o.Tracks = BakeTracks.Soft | BakeTracks.Events;
        var (file, _) = Bake3(o, 30);
        var clip = BakeClip.Parse(file);
        Assert.Equal(BakeTracks.Soft | BakeTracks.Events, clip.Tracks);
        Assert.Empty(clip.Rigid); Assert.NotEmpty(clip.Soft);
    }

    [Fact]
    public void OptionsAndMisuseAreRejectedClearly()
    {
        var w = BakeScenes.Spatial(1);
        Assert.Throws<ArgumentOutOfRangeException>(() => w.BeginBake(new BakeOptions { FrameStride = 0 }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.BeginBake(new BakeOptions { PositionExponent = 3 }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.BeginBake(new BakeOptions { ReplayDigest = new byte[5] }));
        using var rec = w.BeginBake(Opt());
        Assert.Throws<InvalidOperationException>(() => w.BeginBake());
        w.Step(Dt);
        w.Step(Dt * 2);
        Assert.Throws<InvalidOperationException>(() => rec.Finish());
        Assert.False(rec.IsActive);
        // A finished or faulted bake detaches, so the world can be baked again.
        using var again = w.BeginBake(Opt());
        Assert.True(again.IsActive);
    }

    [Fact]
    public void TheHeaderNamesTheSceneTheReplayAndTheVersion()
    {
        var w = BakeScenes.Planar(1);
        var replay = new byte[] { 1, 2, 3, 4 };
        var o = Opt(); o.ReplayDigest = BakeRecorder.DigestOf(replay); o.LibraryVersion = null;
        using var rec = w.BeginBake(o);
        w.Step(Dt);
        var clip = BakeClip.Parse(rec.Finish());
        Assert.Equal(BakeRecorder.DigestOf(replay), clip.ReplayDigest.ToArray());
        Assert.Matches(@"^\d+\.\d+\.\d+$", clip.LibraryVersion);
        Assert.Equal(1, clip.FormatVersion);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(5)]
    [InlineData(4096)]
    public void AnyBlockSizeReadsBackTheSameAndSamplesAcrossBlockBoundaries(int block)
    {
        var o = Opt(); o.BlockFrames = block;
        var (file, frames) = Bake3(o, 40);
        var clip = BakeClip.Parse(file);
        CheckFrames(clip, frames, true, Math.Pow(2, -17) + 1e-12, Math.Pow(2, -15) + 1e-12, 2 * Math.Pow(2, -17));
        foreach (var e in clip.Rigid)
        {
            for (int k = e.FirstFrame; k < e.LastFrame; k++)
            {
                Assert.True(e.TrySampleRigid(clip.TimeOfFrame(k) + 0.5 * clip.FrameDuration, out var m));
                Assert.True(e.TrySampleRigid(clip.TimeOfFrame(k), out var a));
                Assert.True(e.TrySampleRigid(clip.TimeOfFrame(k + 1), out var b));
                Assert.InRange(Math.Abs(m.Position.Y - 0.5 * (a.Position.Y + b.Position.Y)), 0, 1e-9);
            }
        }
    }

    [Fact]
    public void FragmentGeometryIsTheFragmentsShapeInItsOwnFrame()
    {
        var w = BakeScenes.Spatial(1);
        var fragments = new Dictionary<(int, int), double>();
        using var rec = w.BeginBake(Opt());
        for (int i = 0; i < 150; i++)
        {
            w.Step(Dt);
            foreach (var f in w.FractureEvents) { for (int k = 0; k < f.FragmentCount; k++) { var b = w.FractureFragments[f.FirstFragment + k]; fragments[(b.RawHandle.Index, b.RawHandle.Generation)] = b.Mass; } }
        }

        var clip = BakeClip.Parse(rec.Finish());
        Assert.NotEmpty(fragments);
        double density = 1000.0 / 1000.0; // AddBox's default density, and fragments keep their parent's
        foreach (var g in clip.FragmentGeometry)
        {
            Assert.True(fragments.TryGetValue((g.Body.Slot, g.Body.Generation), out double mass));
            // The closed surface encloses the volume that the mass came from: the sum of signed tetrahedra about the body origin.
            double vol = 0;
            for (int t = 0; t < g.Triangles.Length; t += 3)
            {
                var a = g.Vertices[g.Triangles[t]]; var b = g.Vertices[g.Triangles[t + 1]]; var c = g.Vertices[g.Triangles[t + 2]];
                vol += ((double)a.X * ((double)b.Y * c.Z - (double)b.Z * c.Y) - (double)a.Y * ((double)b.X * c.Z - (double)b.Z * c.X) + (double)a.Z * ((double)b.X * c.Y - (double)b.Y * c.X)) / 6.0;
            }

            Assert.InRange(Math.Abs(vol) * density / mass, 0.98, 1.02);
        }
    }

    [Fact]
    public void AMillimetreWorldBakesWithItsUnitAndQuantaThatFollowIt()
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 7395, LengthUnit = LengthUnit.Millimetres });
        w.AddBox(new Vector3F(30000, 1000, 30000), new Vector3D(0, -500, 0), type: BodyType.Static);
        var box = w.AddBox(new Vector3F(500, 500, 500), new Vector3D(0, 1500, 0), new QuaternionF(0.1f, 0.2f, 0f, 1f).Normalized());
        w.AddCloth(new P3.ClothDef { Width = 800, Depth = 800, SegmentsX = 6, SegmentsZ = 6, Position = new Vector3D(2000, 1200, 0) });
        var frames = new List<Frame> { Read3(w) };
        using var rec = w.BeginBake(Opt());
        for (int i = 0; i < 90; i++) { w.Step(Dt); frames.Add(Read3(w)); }
        var file = rec.Finish();
        var clip = BakeClip.Parse(file);
        Assert.Equal(0.001, clip.MetresPerUnit);
        Assert.Equal(-7, clip.PositionExponent);
        Assert.Equal(-5, clip.VertexExponent);
        CheckFrames(clip, frames, true, Math.Pow(2, -8) + 1e-12, Math.Pow(2, -6) + 1e-12, 2 * Math.Pow(2, -17));
        var e = clip.FindRigid(box.RawHandle.Index, box.RawHandle.Generation)!;
        Assert.True(e.TrySampleRigid(clip.TimeOfFrame(0), out var p0));
        Assert.InRange(p0.Position.Y, 1499.99, 1500.01);
        // A metre world writes no unit chunk, so its clip reports one metre.
        var (metres, _) = Bake3(Opt(), 3);
        Assert.Equal(1.0, BakeClip.Parse(metres).MetresPerUnit);
        // A host-set exponent is in world units and is used as given.
        var o = Opt(); o.PositionExponent = -3; o.Tracks = BakeTracks.Rigid;
        var w2 = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 7396, LengthUnit = LengthUnit.Millimetres });
        w2.AddBox(new Vector3F(500, 500, 500), new Vector3D(0, 1500, 0));
        using var r2 = w2.BeginBake(o);
        w2.Step(Dt);
        Assert.Equal(-3, BakeClip.Parse(r2.Finish()).PositionExponent);
    }
}
