using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Reflection;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Presentation;
using RecursiveAI.OnePhysics.Replay;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Presentation;

/// <summary>The follower world in both dimensions: the authority is untouched, a recording replays without it, budgets hold, and a rollback never makes cosmetics fast.</summary>
public class PresentationTests
{
    private const float Dt = PresentationScenes.Dt;

    /// <summary>The same checks in either dimension, over the shared scene.</summary>
    private abstract class Rig
    {
        public abstract int Step { get; set; }
        public abstract void Advance();
        public abstract ulong AuthorityHash();
        public abstract byte[] AuthoritySnapshot();
        public abstract void AuthorityRestore(byte[] snapshot);
        public abstract ulong FollowerHash();
        public abstract long FollowerSteps();
        public abstract PresentationRecorder Record(Stream s);
        public abstract int CosmeticBodies();
        public abstract float FastestDynamic();
        public abstract PresentationQuality Quality { get; set; }
        public abstract void AddDynamic(float mass, int n);
        public abstract int Mirrors();
        public abstract int FluidCount();
        public abstract void SetMaxSpeed(float v);
        public static Rig Create(int dim, bool follower, PresentationQuality? q = null) => dim == 3 ? new Rig3(follower, q) : new Rig2(follower, q);
    }

    private sealed class Rig3 : Rig
    {
        private readonly PresentationScenes.Scene3 _s;
        public Rig3(bool follower, PresentationQuality? q) { _s = PresentationScenes.Build3(1, follower, quality: q); }
        public override int Step { get => _s.Step; set => _s.Step = value; }
        public override void Advance() => _s.Advance();
        public override ulong AuthorityHash() => _s.Authority.ComputeHash();
        public override byte[] AuthoritySnapshot() => _s.Authority.Snapshot();
        public override void AuthorityRestore(byte[] snapshot) => _s.Authority.Restore(snapshot);
        public override ulong FollowerHash() => _s.Follower!.ComputeHash();
        public override long FollowerSteps() => _s.Follower!.StepCount;
        public override PresentationRecorder Record(Stream s) => _s.Follower!.StartRecording(s);
        public override int CosmeticBodies() => _s.Follower!.CosmeticBodyCount;
        public override PresentationQuality Quality { get => _s.Follower!.Quality; set => _s.Follower!.Quality = value; }
        public override void SetMaxSpeed(float v) => _s.Follower!.MaxSpeed = v;
        public override int Mirrors() => _s.Follower!.MirrorCount;
        public override int FluidCount() => _s.Follower!.World.FluidParticleCount;
        public override void AddDynamic(float mass, int n)
        {
            for (int i = 0; i < n; i++) _s.Follower!.World.AddBox(new Vector3F(0.2f, 0.2f, 0.2f), new Vector3D(-6 + 0.3 * i, 3 + 0.05 * (i % 7), 4), density: mass / 0.008f);
        }

        public override float FastestDynamic()
        {
            float max = 0;
            var bodies = _s.Follower!.World.Core.Bodies;
            for (int b = 0; b < bodies.HighWater; b++)
            {
                if (!bodies.IsDynamic(b)) continue;
                var v = bodies.LinearVelocityOf(b);
                max = Math.Max(max, MathF.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]));
            }

            return max;
        }
    }

    private sealed class Rig2 : Rig
    {
        private readonly PresentationScenes.Scene2 _s;
        public Rig2(bool follower, PresentationQuality? q) { _s = PresentationScenes.Build2(1, follower, quality: q); }
        public override int Step { get => _s.Step; set => _s.Step = value; }
        public override void Advance() => _s.Advance();
        public override ulong AuthorityHash() => _s.Authority.ComputeHash();
        public override byte[] AuthoritySnapshot() => _s.Authority.Snapshot();
        public override void AuthorityRestore(byte[] snapshot) => _s.Authority.Restore(snapshot);
        public override ulong FollowerHash() => _s.Follower!.ComputeHash();
        public override long FollowerSteps() => _s.Follower!.StepCount;
        public override PresentationRecorder Record(Stream s) => _s.Follower!.StartRecording(s);
        public override int CosmeticBodies() => _s.Follower!.CosmeticBodyCount;
        public override PresentationQuality Quality { get => _s.Follower!.Quality; set => _s.Follower!.Quality = value; }
        public override void SetMaxSpeed(float v) => _s.Follower!.MaxSpeed = v;
        public override int Mirrors() => _s.Follower!.MirrorCount;
        public override int FluidCount() => _s.Follower!.World.FluidParticleCount;
        public override void AddDynamic(float mass, int n)
        {
            for (int i = 0; i < n; i++) _s.Follower!.World.AddBox(new Vector2F(0.2f, 0.2f), new Vector2D(-6 + 0.3 * i, 3 + 0.05 * (i % 7)), density: mass / 0.04f);
        }

        public override float FastestDynamic()
        {
            float max = 0;
            var bodies = _s.Follower!.World.Core.Bodies;
            for (int b = 0; b < bodies.HighWater; b++)
            {
                if (!bodies.IsDynamic(b)) continue;
                var v = bodies.LinearVelocityOf(b);
                max = Math.Max(max, MathF.Sqrt(v[0] * v[0] + v[1] * v[1]));
            }

            return max;
        }
    }

    // ---------------------------------------------------------------- the authority is untouched

    [Theory]
    [InlineData(2)]
    [InlineData(3)]
    public void AuthorityBytesAndHashesAreTheSameWithAndWithoutAFollower(int dim)
    {
        var plain = Rig.Create(dim, false);
        var followed = Rig.Create(dim, true);
        for (int i = 0; i < 200; i++)
        {
            plain.Advance(); followed.Advance();
            Assert.Equal(plain.AuthorityHash(), followed.AuthorityHash());
        }

        Assert.Equal(plain.AuthoritySnapshot(), followed.AuthoritySnapshot());
        Assert.Equal(200, followed.FollowerSteps());
        Assert.True(followed.CosmeticBodies() > 0, "the fracture spawned cosmetic shards");
    }

    [Fact]
    public void AFollowerAddedAndDroppedMidRunLeavesTheAuthorityAlone3D()
    {
        var plain = PresentationScenes.Build3(1, false);
        var late = PresentationScenes.Build3(1, false);
        P3.PresentationWorld3D? f = null;
        for (int i = 0; i < 150; i++)
        {
            plain.Advance(); late.Advance();
            if (i == 30) { f = new P3.PresentationWorld3D(new P3.WorldOptions3D { WorldId = 86 }); f.Follow(late.Floor); f.Follow(late.Platform); f.Follow(late.Block); }
            f?.Step(late.Authority, Dt);
            if (i == 100) { f!.Unfollow(late.Platform); f = null; }
            Assert.Equal(plain.Authority.ComputeHash(), late.Authority.ComputeHash());
        }

        Assert.Equal(plain.Authority.Snapshot(), late.Authority.Snapshot());
    }

    [Fact]
    public void FractureDebrisFollowsTheFragmentsAndSpawnsShards()
    {
        var s = PresentationScenes.Build3(1, true);
        var f = s.Follower!;
        int before = f.MirrorCount;
        for (int i = 0; i <= PresentationScenes.FractureStep; i++) s.Advance();
        string dbg = "after fracture step: mirrors " + f.MirrorCount + " frags " + s.Authority.FractureFragments.Length + " ev " + s.Authority.FractureEvents.Length;
        s.Advance();
        Assert.True(f.MirrorCount > before, dbg + " then " + f.MirrorCount);
        Assert.False(f.TryGetMirror(s.Block, out _) && s.Block.IsValid);
        Assert.Equal(f.Debris.Count, f.CosmeticBodyCount);
        for (int i = 0; i < 120; i++) s.Advance();
        // Every mirror sits where its authoritative body is (within the step that is still to be applied).
        foreach (var frag in s.Authority.FractureFragments)
        {
            Assert.True(f.TryGetMirror(frag, out var m));
            var d = frag.Position - m.Position;
            Assert.True(Math.Sqrt(d.X * d.X + d.Y * d.Y + d.Z * d.Z) < 0.05, "mirror tracks the fragment");
        }
    }

    // ---------------------------------------------------------------- recording and replay

    [Theory]
    [InlineData(2)]
    [InlineData(3)]
    public void ARecordingReplaysEveryStepHashWithoutTheAuthority(int dim)
    {
        var rig = Rig.Create(dim, true);
        var ms = new MemoryStream();
        var hashes = new System.Collections.Generic.List<ulong>();
        using (var rec = rig.Record(ms))
        {
            for (int i = 0; i < 160; i++)
            {
                if (i == 50) rig.AddDynamic(0.3f, 4);                               // the host adds bodies between steps
                if (i == 90) { var q = rig.Quality; q.MaxBodies = 6; rig.Quality = q; }   // and changes the budgets
                rig.Advance();
                hashes.Add(rig.FollowerHash());
            }

            Assert.Equal(160, rec.Frames);
        }

        ms.Position = 0;
        var player = new PresentationPlayer(ms);
        Assert.Equal(dim, player.Dimension);
        ulong last = 0; int n = 0;
        if (dim == 3)
        {
            var f = player.CreateFollower3D();
            while (player.ReadNextStep()) { Assert.Equal(hashes[n++], f.ComputeHash()); last = f.ComputeHash(); }
        }
        else
        {
            var f = player.CreateFollower2D();
            while (player.ReadNextStep()) { Assert.Equal(hashes[n++], f.ComputeHash()); last = f.ComputeHash(); }
        }

        Assert.Equal(160, n);
        Assert.Equal(rig.FollowerHash(), last);
        Assert.Equal(160, player.StepsPlayed);
    }

    [Fact]
    public void ARecordingRunsOnOtherThreadCountsWithTheSameHashes()
    {
        var rig = Rig.Create(3, true);
        var ms = new MemoryStream();
        using (var rec = rig.Record(ms)) { for (int i = 0; i < 90; i++) rig.Advance(); }
        foreach (int threads in new[] { 1, 4 })
        {
            ms.Position = 0;
            var player = new PresentationPlayer(ms);
            var f = player.CreateFollower3D(threads);
            while (player.ReadNextStep()) { }
            Assert.Equal(rig.FollowerHash(), f.ComputeHash());
        }
    }

    [Fact]
    public void ACorruptedHashIsReportedWithItsStep()
    {
        var rig = Rig.Create(3, true);
        var ms = new MemoryStream();
        using (var rec = rig.Record(ms)) { for (int i = 0; i < 20; i++) rig.Advance(); }
        var bytes = ms.ToArray();
        bytes[^6] ^= 0x55;   // inside the last frame's hash (the terminator takes the last 5 bytes)
        var player = new PresentationPlayer(new MemoryStream(bytes));
        player.CreateFollower3D();
        var e = Assert.Throws<ReplayHashMismatchException>(() => { while (player.ReadNextStep()) { } });
        Assert.Equal(20, e.Step);
    }

    [Fact]
    public void MalformedRecordingsFailWithTypedErrors()
    {
        var rig = Rig.Create(3, true);
        var ms = new MemoryStream();
        using (var rec = rig.Record(ms)) { for (int i = 0; i < 12; i++) rig.Advance(); }
        var good = ms.ToArray();
        Assert.Throws<ReplayFormatException>(() => new PresentationPlayer(new MemoryStream(good, 0, 10)));
        var bad = (byte[])good.Clone(); bad[0] ^= 1;
        Assert.Throws<ReplayFormatException>(() => new PresentationPlayer(new MemoryStream(bad)));
        bad = (byte[])good.Clone(); bad[6] = 4;
        Assert.Throws<ReplayFormatException>(() => new PresentationPlayer(new MemoryStream(bad)));
        // Truncation anywhere in the body, and single-byte damage anywhere, only ever raise the two typed errors.
        var rng = new Random(5);
        for (int t = 0; t < 120; t++)
        {
            var data = (byte[])good.Clone();
            int cut = t < 60 ? rng.Next(16, data.Length) : data.Length;
            if (t >= 60) data[rng.Next(16, data.Length)] ^= (byte)(1 << rng.Next(8));
            try
            {
                var p = new PresentationPlayer(new MemoryStream(data, 0, cut), maximumPayloadBytes: 1 << 24);
                p.CreateFollower3D();
                while (p.ReadNextStep()) { }
            }
            catch (Exception e) when (e is ReplayFormatException or ReplayHashMismatchException or SnapshotFormatException) { }
        }

        // A declared length beyond the bound is refused before anything is allocated.
        var huge = (byte[])good.Clone();
        System.Buffers.Binary.BinaryPrimitives.WriteInt32LittleEndian(huge.AsSpan(17), int.MaxValue);
        var hp = new PresentationPlayer(new MemoryStream(huge), maximumPayloadBytes: 1 << 20);
        Assert.Throws<ReplayFormatException>(() => hp.CreateFollower3D());
    }

    // ---------------------------------------------------------------- budgets

    [Theory]
    [InlineData(2)]
    [InlineData(3)]
    public void TheBodyBudgetHoldsAndLoweringCullsLightestThenOldest(int dim)
    {
        var rig = Rig.Create(dim, true, new PresentationQuality { MaxBodies = 20 });
        rig.AddDynamic(0.5f, 12);
        rig.AddDynamic(0.1f, 12);   // lighter, and newer
        rig.Advance();
        Assert.Equal(20, rig.CosmeticBodies());
        for (int i = 0; i < 30; i++) { rig.Advance(); Assert.True(rig.CosmeticBodies() <= 20); }
        // Lowering to 14: the 6 lightest go (all light ones, oldest of them first). Raising again changes nothing that exists.
        var q = rig.Quality; q.MaxBodies = 14; rig.Quality = q;
        var before = rig.CosmeticBodies();
        rig.Advance();
        Assert.Equal(14, rig.CosmeticBodies());
        Assert.True(before > 14);
        var hashBefore = rig.FollowerHash();
        q.MaxBodies = 1000; rig.Quality = q;
        Assert.Equal(14, rig.CosmeticBodies());
        Assert.Equal(hashBefore, rig.FollowerHash());
        rig.Advance();
        Assert.Equal(14, rig.CosmeticBodies());
    }

    [Fact]
    public void CullingKeepsTheHeaviestAndNewestWhenMassesTie3D()
    {
        var f = new P3.PresentationWorld3D(new P3.WorldOptions3D { WorldId = 87 }, new PresentationQuality { MaxBodies = 3 });
        var w = f.World;
        w.AddBox(new Vector3F(50, 1, 50), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var bodies = new P3.Body[6];
        for (int i = 0; i < 6; i++) { bodies[i] = w.AddBox(new Vector3F(0.2f, 0.2f, 0.2f), new Vector3D(i, 5, 0), density: 100f); f.Step(Dt); }
        // Equal masses: the oldest are culled first, so the last three created stay.
        for (int i = 0; i < 3; i++) Assert.False(bodies[i].IsValid);
        for (int i = 3; i < 6; i++) Assert.True(bodies[i].IsValid);
        // A pinned body is neither counted nor culled.
        var pinned = w.AddBox(new Vector3F(0.2f, 0.2f, 0.2f), new Vector3D(9, 5, 0), density: 1f);
        f.Pin(pinned);
        f.Step(Dt);
        Assert.True(pinned.IsValid);
        Assert.Equal(3, f.CosmeticBodyCount);
    }

    [Fact]
    public void TheParticleBudgetCullsTheOldestParticles3D()
    {
        var f = new P3.PresentationWorld3D(new P3.WorldOptions3D { WorldId = 88 }, new PresentationQuality { MaxParticles = 100000 });
        var w = f.World;
        w.AddBox(new Vector3F(20, 1, 20), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var fluid = w.AddParticleFluid(new ParticleFluidDef { Spacing = 0.1f });
        w.FillFluidBox(fluid, new Vector3D(-0.5, 0.2, -0.5), new Vector3D(0.5, 0.6, 0.5), default);
        int n = w.FluidParticleCount;
        Assert.True(n > 200);
        f.Step(Dt);
        Assert.Equal(n, w.FluidParticleCount);
        var q = f.Quality; q.MaxParticles = n / 2; f.Quality = q;
        f.Step(Dt);
        Assert.Equal(n / 2, w.FluidParticleCount);
        Assert.Equal(n / 2, w.FluidSettings.MaxParticles);
        q.MaxParticles = PresentationQuality.Unlimited; f.Quality = q;
        f.Step(Dt);
        Assert.Equal(n / 2, w.FluidParticleCount);
    }

    [Fact]
    public void SoftAndRodSchedulesFollowTheQualityAndRestoreWhenRaised3D()
    {
        var f = new P3.PresentationWorld3D(new P3.WorldOptions3D { WorldId = 89 });
        var w = f.World;
        var rope = w.AddRope(new Vector3D(0, 3, 0), new Vector3D(1, 3, 0), 8);
        f.Quality = PresentationQuality.Low;
        f.Step(Dt);
        Assert.Equal(2, w.Options.SoftSubsteps);
        Assert.Equal(1, w.Options.SoftIterations);
        Assert.Equal(2, rope.Particles.Substeps);
        f.Quality = PresentationQuality.High;
        f.Step(Dt);
        Assert.Equal(4, w.Options.SoftSubsteps);
        Assert.Equal(8, rope.Particles.Substeps);
        Assert.Equal(4, rope.Particles.Iterations);
    }

    // ---------------------------------------------------------------- rollback

    [Theory]
    [InlineData(2)]
    [InlineData(3)]
    public void ARollbackOfOneHundredTwentyStepsNeverMakesCosmeticsFast(int dim)
    {
        var rig = Rig.Create(dim, true);
        for (int i = 0; i < 70; i++) rig.Advance();
        var snap = rig.AuthoritySnapshot(); int stepAtSnap = rig.Step;
        float fastest = 0;
        for (int round = 0; round < 3; round++)
        {
            rig.AddDynamic(0.2f, 6);                       // cosmetics resting around when the poses jump
            for (int i = 0; i < 120; i++) { rig.Advance(); fastest = Math.Max(fastest, rig.FastestDynamic()); }
            rig.AuthorityRestore(snap); rig.Step = stepAtSnap;   // the host rolls back 120 steps
            for (int i = 0; i < 60; i++) { rig.Advance(); fastest = Math.Max(fastest, rig.FastestDynamic()); Assert.True(rig.FastestDynamic() <= 30f); }
        }

        Assert.True(fastest <= 30f, "speed " + fastest);
        if (Environment.GetEnvironmentVariable("PRESENTATION_COST_OUT") is { } o) File.AppendAllText(o, $"dim={dim} rollback 120 steps: fastest cosmetic body {fastest:F3} m/s (bound 30)\n");
    }

    [Fact]
    public void AJumpTeleportsTheMirrorInsteadOfFlingingWhatRestsOnIt3D()
    {
        var a = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 90 });
        var pad = a.AddBox(new Vector3F(2, 0.2f, 2), new Vector3D(0, 1, 0), type: BodyType.Kinematic);
        var f = new P3.PresentationWorld3D(new P3.WorldOptions3D { WorldId = 91 });
        f.Follow(pad);
        var crate = f.World.AddBox(new Vector3F(0.3f, 0.3f, 0.3f), new Vector3D(0, 1.25, 0));
        for (int i = 0; i < 60; i++) { a.Step(Dt); f.Step(a, Dt); }
        Assert.True(crate.Position.Y > 1.1);
        pad.SetTransform(new Vector3D(300, 40, 0), QuaternionF.Identity);   // a pose jump far beyond TeleportSpeed
        a.Step(Dt); f.Step(a, Dt);
        Assert.True(f.TryGetMirror(pad, out var mirror));
        Assert.True(mirror.Position.X > 299);
        Assert.Equal(0f, mirror.LinearVelocity.X);
        Assert.True(crate.LinearVelocity.Y > -5f);   // left behind, falling, not launched
        for (int i = 0; i < 30; i++) { a.Step(Dt); f.Step(a, Dt); }
    }

    // ---------------------------------------------------------------- hostile use

    [Fact]
    public void BadArgumentsAreRefusedWithNamedErrors3D()
    {
        var f = new P3.PresentationWorld3D();
        Assert.Throws<ArgumentOutOfRangeException>(() => f.Step(0f));
        Assert.Throws<ArgumentOutOfRangeException>(() => f.Step(float.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => f.MaxSpeed = -1f);
        Assert.Throws<ArgumentOutOfRangeException>(() => f.TeleportSpeed = 1e9f);
        Assert.Throws<ArgumentOutOfRangeException>(() => f.Quality = new PresentationQuality { MaxBodies = -1 });
        Assert.Throws<ArgumentOutOfRangeException>(() => f.Debris = new PresentationDebris { Count = 1000 });
        Assert.Throws<ArgumentNullException>(() => f.Observe(null!));
        Assert.Throws<ArgumentException>(() => f.Observe(f.World));
        Assert.Throws<InvalidHandleException>(() => f.Follow(default));
        var own = f.World.AddBox(new Vector3F(1, 1, 1));
        Assert.Throws<ArgumentException>(() => f.Follow(own));
        var other = new P3.PhysicsWorld3D();
        var b = other.AddBox(new Vector3F(1, 1, 1), new Vector3D(1, 2, 3));
        var m = f.Follow(b);
        Assert.Same(f.World, m.World);
        Assert.Equal(m, f.Follow(b));
        b.Destroy();
        f.Step(other, Dt);
        Assert.Equal(0, f.MirrorCount);
        Assert.False(m.IsValid);
        using var ms = new MemoryStream();
        using var rec = f.StartRecording(ms);
        Assert.Throws<InvalidOperationException>(() => f.StartRecording(new MemoryStream()));
    }

    // ---------------------------------------------------------------- mirrors copy geometry

    [Fact]
    public void MirrorsCopyEveryShapeKind3D()
    {
        var a = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 92 });
        var f = new P3.PresentationWorld3D(new P3.WorldOptions3D { WorldId = 93 });
        var bodies = new List<P3.Body>
        {
            a.AddSphere(0.5f, new Vector3D(-6, 1, 0)), a.AddBox(new Vector3F(1, 2, 1), new Vector3D(-4, 1, 0)), a.AddCapsule(0.5f, 0.3f, new Vector3D(-2, 1, 0)),
            a.AddCylinder(0.5f, 0.4f, new Vector3D(0, 1, 0)), a.AddConvexHull(new[] { new Vector3F(0, 0, 0), new Vector3F(1, 0, 0), new Vector3F(0, 1, 0), new Vector3F(0, 0, 1), new Vector3F(1, 1, 1) }, new Vector3D(2, 1, 0)),
            a.AddCompound(new[] { new P3.CompoundPart(P3.ShapeSpec3D.Box(new Vector3F(1, 1, 1)), new Vector3F(-0.6f, 0, 0), QuaternionF.Identity), new P3.CompoundPart(P3.ShapeSpec3D.Sphere(0.5f), new Vector3F(0.6f, 0, 0), QuaternionF.Identity) }, new Vector3D(4, 1, 0)),
        };
        var ground = a.CreateBody(BodyType.Static, new Vector3D(0, 0, 0));
        a.CreateHeightField(ground, new float[] { 0, 0.2f, 0.4f, 0, 0.1f, 0.3f, 0, 0, 0.2f }, 3, 3, new Vector3F(2, 1, 2), default, new P3.ShapeDef { Friction = 0.9f, Layer = 3 });
        bodies.Add(ground);
        var wall = a.CreateBody(BodyType.Static, new Vector3D(0, 0, 8));
        a.CreateMesh(wall, new[] { new Vector3F(-3, 0, 0), new Vector3F(3, 0, 0), new Vector3F(0, 4, 0), new Vector3F(0, 4, 1) }, new[] { 0, 1, 2, 0, 2, 3, 1, 3, 2, 0, 3, 1 });
        bodies.Add(wall);
        foreach (var b in bodies) f.Follow(b);
        Assert.Equal(bodies.Count, f.MirrorCount);
        for (int i = 0; i < 5; i++) { a.Step(Dt); f.Step(a, Dt); }
        // The same rays hit the same things at the same distances in both worlds.
        var rng = new Random(11); int hits = 0;
        for (int r = 0; r < 200; r++)
        {
            var from = new Vector3D(rng.NextDouble() * 14 - 7, 2 + rng.NextDouble() * 3, rng.NextDouble() * 12 - 2);
            var dir = new Vector3F((float)(rng.NextDouble() - 0.5), -1f, (float)(rng.NextDouble() - 0.5));
            bool ha = a.Raycast(from, dir, out var x), hb = f.World.Raycast(from, dir, out var y);
            Assert.Equal(ha, hb);
            if (ha) { hits++; Assert.Equal(x.Distance, y.Distance, 4); }
        }

        Assert.True(hits > 20, "rays hit something: " + hits);
        foreach (var b in bodies) { Assert.True(f.TryGetMirror(b, out var m)); Assert.Equal(b.GetShapes().Length, m.GetShapes().Length); Assert.Equal(b.Type == BodyType.Static ? BodyType.Static : BodyType.Kinematic, m.Type); }
    }

    [Fact]
    public void MirrorsCopyEveryShapeKind2D()
    {
        var a = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 94 });
        var f = new P2.PresentationWorld2D(new P2.WorldOptions2D { WorldId = 95 });
        var bodies = new List<P2.Body>
        {
            a.AddCircle(0.5f, new Vector2D(-6, 1)), a.AddBox(new Vector2F(1, 2), new Vector2D(-4, 1)), a.AddCapsule(0.5f, 0.3f, new Vector2D(-2, 1)),
        };
        var ground = a.CreateBody(BodyType.Static, new Vector2D(0, 0));
        a.CreateChain(ground, new[] { new Vector2F(-8, 0), new Vector2F(-2, 0.2f), new Vector2F(2, -0.1f), new Vector2F(8, 0) });
        a.CreateSegment(ground, new Vector2F(-8, 6), new Vector2F(8, 6));
        a.CreatePolygon(ground, new[] { new Vector2F(3, 0), new Vector2F(4, 0), new Vector2F(3.5f, 2) });
        bodies.Add(ground);
        foreach (var b in bodies) f.Follow(b);
        for (int i = 0; i < 5; i++) { a.Step(Dt); f.Step(a, Dt); }
        var rng = new Random(12); int hits = 0;
        for (int r = 0; r < 200; r++)
        {
            var from = new Vector2D(rng.NextDouble() * 14 - 7, 1 + rng.NextDouble() * 4);
            var dir = new Vector2F((float)(rng.NextDouble() - 0.5), -1f);
            bool ha = a.Raycast(from, dir, out var x), hb = f.World.Raycast(from, dir, out var y);
            Assert.Equal(ha, hb);
            if (ha) { hits++; Assert.Equal(x.Distance, y.Distance, 4); }
        }

        Assert.True(hits > 20, "rays hit something: " + hits);
    }

    // ---------------------------------------------------------------- length units

    [Fact]
    public void AFollowerTakesTheAuthoritysMillimetreUnitAndItsDefaultsFollow3D()
    {
        var a = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 96, LengthUnit = LengthUnit.Millimetres });
        var floor = a.AddBox(new Vector3F(40000, 1000, 40000), new Vector3D(0, -500, 0), type: BodyType.Static);
        var block = a.AddBox(new Vector3F(1000, 1000, 1000), new Vector3D(4000, 500, 0), density: 5e-7f);
        var f = new P3.PresentationWorld3D(new P3.WorldOptions3D { WorldId = 97 });
        Assert.True(f.World.LengthUnit.IsMetres);
        f.Follow(floor); f.Follow(block);
        Assert.Equal(LengthUnit.Millimetres, f.World.LengthUnit);
        Assert.Equal(30000f, f.MaxSpeed, 1);          // 30 m/s
        Assert.Equal(30000f, f.TeleportSpeed, 1);
        using var ms = new MemoryStream();
        using (f.StartRecording(ms))
        {
            for (int i = 0; i < 30; i++) { a.Step(Dt); f.Step(a, Dt); }
            a.Fracture(block, new P3.FractureDef { CellCount = 5, Seed = 3 });
            f.Observe(a);   // a manual fracture's events last until the next authoritative step
            for (int i = 0; i < 90; i++) { a.Step(Dt); f.Step(a, Dt); }
        }

        Assert.Equal(f.Debris.Count, f.CosmeticBodyCount);
        var bodies = f.World.Core.Bodies; int shards = 0; float fastest = 0;
        for (int b = 0; b < bodies.HighWater; b++)
        {
            if (!bodies.IsDynamic(b)) continue;
            shards++;
            var body = f.World.GetBody(f.World.Core.HandleOfBody(b));
            Assert.InRange(body.Mass, 0.07f, 0.08f);   // 50 mm cube of 600 kg/m3 = 0.075 kg, as in metres
            var v = body.LinearVelocity; fastest = Math.Max(fastest, MathF.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z));
        }

        Assert.Equal(12, shards);
        Assert.True(fastest <= f.MaxSpeed);
        f.MaxSpeed = 5000f; Assert.Equal(5000f, f.MaxSpeed); f.UseDefaultSpeeds(); Assert.Equal(30000f, f.MaxSpeed, 1);
        // The recording carries the unit (in the live options) and replays to the same hash.
        ms.Position = 0;
        var player = new PresentationPlayer(ms);
        var replayed = player.CreateFollower3D();
        Assert.Equal(LengthUnit.Millimetres, replayed.World.LengthUnit);
        while (player.ReadNextStep()) { }
        Assert.Equal(f.ComputeHash(), replayed.ComputeHash());
        // A different unit in a populated follower is refused with both units in the message.
        var other = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 98 });
        var e = Assert.Throws<ArgumentException>(() => f.Follow(other.AddBox(new Vector3F(1, 1, 1))));
        Assert.Contains("0.001", e.Message);
    }

    [Fact]
    public void AFollowerTakesThePixelUnit2D()
    {
        var a = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 99, LengthUnit = LengthUnit.Pixels(100) });
        var plat = a.AddBox(new Vector2F(200, 20), new Vector2D(0, 100), type: BodyType.Kinematic);
        var f = new P2.PresentationWorld2D(new P2.WorldOptions2D { WorldId = 100 });
        f.Follow(plat);
        Assert.Equal(100.0, 1.0 / f.World.LengthUnit.MetresPerUnit, 6);
        Assert.Equal(3000f, f.MaxSpeed, 1);
        var q = f.Quality; q.MaxBodies = 3; f.Quality = q;
        var shards = f.SpawnShards(new Vector2D(0, 300), default);
        Assert.Equal(12, shards.Length);
        for (int i = 0; i < 5; i++) { plat.MoveKinematic(new Vector2D(i, 100), 0f); a.Step(Dt); f.Step(a, Dt); }
        Assert.Equal(3, f.CosmeticBodyCount);
    }

    // ---------------------------------------------------------------- steady state

    [Theory]
    [InlineData(2)]
    [InlineData(3)]
    public void ObserveAndStepAllocateNothingOnceWarm(int dim)
    {
        Action advance; 
        if (dim == 3) { var s = PresentationScenes.Build3(1, true); for (int i = 0; i < 150; i++) s.Advance(); advance = s.Advance; }
        else { var s = PresentationScenes.Build2(1, true); for (int i = 0; i < 150; i++) s.Advance(); advance = s.Advance; }
        Core.AllocMeter.Settle();
        Core.AllocMeter.AssertAllocatesNothing(() => { for (int i = 0; i < 40; i++) advance(); }, "follower Observe + Step with " + (dim == 3 ? "3D" : "2D") + " hair, cloth, rope and debris");
    }

    // ---------------------------------------------------------------- the two worlds match

    [Fact]
    public void BothDimensionsExposeTheSameMembers()
    {
        static SortedSet<string> Names(Type t) => new(t.GetMembers(BindingFlags.Public | BindingFlags.Instance | BindingFlags.Static | BindingFlags.DeclaredOnly)
            .Where(m => m is not ConstructorInfo && !(m is MethodInfo mi && mi.IsSpecialName && (mi.Name.StartsWith("get_", StringComparison.Ordinal) || mi.Name.StartsWith("set_", StringComparison.Ordinal) || mi.Name.StartsWith("add_", StringComparison.Ordinal) || mi.Name.StartsWith("remove_", StringComparison.Ordinal))))
            .Select(m => m.Name), StringComparer.Ordinal);
        var a = Names(typeof(P2.PresentationWorld2D)); var b = Names(typeof(P3.PresentationWorld3D));
        Assert.Equal(b, a);
        Assert.Contains("Follow", a); Assert.Contains("Observe", a); Assert.Contains("StartRecording", a); Assert.Contains("Quality", a);
        // Nothing on the happy path asks for a struct callback or a generic argument.
        foreach (var t in new[] { typeof(P2.PresentationWorld2D), typeof(P3.PresentationWorld3D) })
            Assert.DoesNotContain(t.GetMethods(BindingFlags.Public | BindingFlags.Instance), m => m.IsGenericMethodDefinition);
    }

    // ---------------------------------------------------------------- cost (indicative; set PRESENTATION_COST_OUT to a file to run it)

    [Theory]
    [InlineData(2, 1)]
    [InlineData(2, 8)]
    [InlineData(3, 1)]
    [InlineData(3, 8)]
    public void FollowerStepCost(int dim, int threads)
    {
        string? output = Environment.GetEnvironmentVariable("PRESENTATION_COST_OUT");
        if (output == null) return;
        var follower = new List<double>(); var authority = new List<double>();
        double ms = 1000.0 / Stopwatch.Frequency;
        if (dim == 3)
        {
            var s = PresentationScenes.Build3(threads, true); s.Timing = true;
            for (int i = 0; i < 400; i++) { s.Advance(); if (i >= 80) { authority.Add(s.AuthorityTicks * ms); follower.Add(s.FollowerTicks * ms); } }
        }
        else
        {
            var s = PresentationScenes.Build2(threads, true); s.Timing = true;
            for (int i = 0; i < 400; i++) { s.Advance(); if (i >= 80) { authority.Add(s.AuthorityTicks * ms); follower.Add(s.FollowerTicks * ms); } }
        }

        follower.Sort(); authority.Sort();
        File.AppendAllText(output, $"dim={dim} threads={threads} follower median={follower[follower.Count / 2]:F3} ms mean={follower.Average():F3} p95={follower[follower.Count * 95 / 100]:F3} max={follower[^1]:F3} | authority median={authority[authority.Count / 2]:F3} ms\n");
    }
}
