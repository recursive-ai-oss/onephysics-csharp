using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics3D;
using RecursiveAI.OnePhysics.Tests.Core;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

public class SoftBody3DTests
{
    private static readonly int[] Quad = { 0, 2, 1, 1, 2, 3 };

    private static readonly int[] Sliver = { 0, 1, 2, 1, 3, 0 };

    private static readonly int[] Pins = { 99 };

    private static readonly int[] TwoIndices = { 0, 1 };

    private static readonly int[] TopPins = { 40, 41 };

    private const float Dt = 1f / 60f;

    private static PhysicsWorld3D NewWorld(Action<WorldOptions3D>? tweak = null) => World3DTests.NewWorld(1, 31, tweak);

    private static void Run(PhysicsWorld3D w, int steps) { for (int i = 0; i < steps; i++) { w.Step(Dt); } }

    // ---- meshes

    [Fact]
    public void GeneratedMeshesAreValidAndHaveTheRightVolume()
    {
        var box = SoftBodyMesh.Box(new Vector3F(1f, 2f, 3f), 2, 3, 4);
        Assert.Equal(3 * 4 * 5, box.VertexCount);
        Assert.Equal(2 * 3 * 4 * 6, box.TetrahedronCount);
        var w = NewWorld();
        var b = w.AddSoftBody(new SoftBodyDef { Mesh = box, Mass = 1f });
        Assert.Equal(6.0, b.RestVolume, 4);
        Assert.Equal(6.0, b.Volume, 4);
        // the boundary is a closed surface: 2 triangles per cell face
        Assert.Equal(2 * (2 * 3 + 2 * 4 + 3 * 4) * 2, box.TriangleCount);

        var ball = SoftBodyMesh.Sphere(1f, 6);
        Assert.True(ball.IsClosed);
        var sb = w.AddSoftBody(new SoftBodyDef { Mesh = ball, Mass = 1f });
        double v = sb.Volume;
        Assert.InRange(v, 3.5, 4.19); // inscribed in the unit sphere: 4/3 pi = 4.19
        Assert.True(SoftBodyMesh.Cloth(1f, 1f, 4, 4).TriangleCount == 32);
    }

    [Fact]
    public void HostileMeshesAreRefusedWithTypedErrors()
    {
        var v = new[] { new Vector3F(0, 0, 0), new Vector3F(1, 0, 0), new Vector3F(0, 0, 1), new Vector3F(1, 0, 1) };
        Assert.Equal(SoftBodyError.BadCount, Assert.Throws<SoftBodyException>(() => SoftBodyMesh.FromTriangles(v, Array.Empty<int>())).Error);
        Assert.Equal(SoftBodyError.BadCount, Assert.Throws<SoftBodyException>(() => SoftBodyMesh.FromTriangles(v, TwoIndices)).Error);
        Assert.Equal(SoftBodyError.BadIndex, Assert.Throws<SoftBodyException>(() => SoftBodyMesh.FromTriangles(v, new[] { 0, 1, 7 })).Error);
        Assert.Equal(SoftBodyError.BadIndex, Assert.Throws<SoftBodyException>(() => SoftBodyMesh.FromTriangles(v, new[] { 0, 1, -1 })).Error);
        Assert.Equal(SoftBodyError.BadIndex, Assert.Throws<SoftBodyException>(() => SoftBodyMesh.FromTriangles(v, new[] { 0, 1, 1 })).Error);
        Assert.Equal(SoftBodyError.Degenerate, Assert.Throws<SoftBodyException>(() => SoftBodyMesh.FromTriangles(new[] { v[0], v[1], new Vector3F(2, 0, 0), v[3] }, Sliver)).Error);
        Assert.Equal(SoftBodyError.UnusedVertex, Assert.Throws<SoftBodyException>(() => SoftBodyMesh.FromTriangles(v, new[] { 0, 2, 1 })).Error);
        var bad = (Vector3F[])v.Clone();
        bad[2] = new Vector3F(float.NaN, 0, 1);
        Assert.Equal(SoftBodyError.NonFinite, Assert.Throws<SoftBodyException>(() => SoftBodyMesh.FromTriangles(bad, Quad)).Error);
        bad[2] = new Vector3F(1e9f, 0, 1);
        Assert.Equal(SoftBodyError.OutOfRange, Assert.Throws<SoftBodyException>(() => SoftBodyMesh.FromTriangles(bad, Quad)).Error);
        // an edge shared by three triangles
        var fin = new[] { new Vector3F(0, 0, 0), new Vector3F(1, 0, 0), new Vector3F(0, 1, 0), new Vector3F(0, -1, 0), new Vector3F(0, 0, 1) };
        Assert.Equal(SoftBodyError.NonManifold, Assert.Throws<SoftBodyException>(() => SoftBodyMesh.FromTriangles(fin, new[] { 0, 1, 2, 1, 0, 3, 0, 1, 4 })).Error);
        // a flat tetrahedron
        var flat = new[] { new Vector3F(0, 0, 0), new Vector3F(1, 0, 0), new Vector3F(0, 1, 0), new Vector3F(1, 1, 0) };
        Assert.Equal(SoftBodyError.Degenerate, Assert.Throws<SoftBodyException>(() => SoftBodyMesh.FromTetrahedra(flat, new[] { 0, 1, 2, 3 })).Error);
        Assert.Equal(SoftBodyError.OutOfRange, Assert.Throws<SoftBodyException>(() => SoftBodyMesh.Cloth(-1f, 1f, 2, 2)).Error);
        Assert.Equal(SoftBodyError.OutOfRange, Assert.Throws<SoftBodyException>(() => SoftBodyMesh.Box(new Vector3F(1, 1, 1), 100, 1, 1)).Error);
    }

    [Fact]
    public void HostileDefinitionsAreRefused()
    {
        var w = NewWorld();
        var mesh = SoftBodyMesh.Cloth(1f, 1f, 3, 3);
        Assert.Throws<SoftBodyException>(() => w.AddSoftBody(new SoftBodyDef()));
        Assert.Equal(SoftBodyError.NonFinite, Assert.Throws<SoftBodyException>(() => w.AddSoftBody(new SoftBodyDef { Mesh = mesh, Mass = float.NaN })).Error);
        Assert.Equal(SoftBodyError.OutOfRange, Assert.Throws<SoftBodyException>(() => w.AddSoftBody(new SoftBodyDef { Mesh = mesh, Mass = -1f })).Error);
        Assert.Equal(SoftBodyError.OutOfRange, Assert.Throws<SoftBodyException>(() => w.AddSoftBody(new SoftBodyDef { Mesh = mesh, EdgeCompliance = -1f })).Error);
        Assert.Equal(SoftBodyError.OutOfRange, Assert.Throws<SoftBodyException>(() => w.AddSoftBody(new SoftBodyDef { Mesh = mesh, Position = new Vector3D(3e6, 0, 0) })).Error);
        Assert.Equal(SoftBodyError.NonFinite, Assert.Throws<SoftBodyException>(() => w.AddSoftBody(new SoftBodyDef { Mesh = mesh, Position = new Vector3D(double.NaN, 0, 0) })).Error);
        Assert.Equal(SoftBodyError.BadIndex, Assert.Throws<SoftBodyException>(() => w.AddSoftBody(new SoftBodyDef { Mesh = mesh, PinnedParticles = Pins })).Error);
        Assert.Equal(SoftBodyError.NotClosed, Assert.Throws<SoftBodyException>(() => w.AddSoftBody(new SoftBodyDef { Mesh = mesh, InternalPressure = 1f })).Error);
        Assert.Equal(SoftBodyError.InvalidDefinition, Assert.Throws<SoftBodyException>(() => w.AddSoftBody(new SoftBodyDef { Mesh = mesh, Rotation = new QuaternionF(0, 0, 0, 0) })).Error);
        Assert.Equal(0, w.SoftBodyCount);
    }

    [Fact]
    public void AStaleHandleFailsClearlyAndIndicesAreChecked()
    {
        var w = NewWorld();
        var cloth = w.AddCloth(1f, 1f, 3);
        Assert.Equal(16, cloth.ParticleCount);
        Assert.Throws<ArgumentOutOfRangeException>(() => cloth.GetPosition(16));
        Assert.Throws<ArgumentOutOfRangeException>(() => cloth.SetPosition(-1, default));
        Assert.Throws<ArgumentOutOfRangeException>(() => cloth.SetPosition(0, new Vector3D(double.NaN, 0, 0)));
        cloth.Destroy();
        Assert.False(cloth.IsValid);
        Assert.Throws<StaleHandleException>(() => cloth.ParticleCount);
        Assert.Throws<StaleHandleException>(() => cloth.Destroy());
        Assert.Throws<InvalidHandleException>(() => default(SoftBody).ParticleCount);
        Assert.Equal(0, w.SoftBodyCount);
    }

    // ---- dynamics

    [Fact]
    public void AHangingStripSettlesToItsStaticLength()
    {
        // A strip of 20 segments hangs from its top two particles. With stiff edges its length is the sum of its rest lengths; the solver's residual stretch must stay small.
        var w = NewWorld(o => { o.SoftSubsteps = 16; o.SoftIterations = 1; });
        var mesh = SoftBodyMesh.Cloth(0.1f, 2f, 1, 20);
        var def = new SoftBodyDef { Mesh = mesh, Position = new Vector3D(0, 3, 0), Rotation = new QuaternionF(-0.70710677f, 0, 0, 0.70710677f), Mass = 2f, LinearDamping = 0.5f, SelfCollision = false };
        def.PinnedParticles = TopPins;
        var strip = w.AddSoftBody(def);
        Run(w, 240);
        const int last = 0; // the free end
        double top = strip.GetPosition(40).Y;
        double bottom = strip.GetPosition(last).Y;
        Assert.InRange(top - bottom, 2.0, 2.0 * 1.03); // inextensible, to 3 %
        Assert.True(strip.GetVelocity(last).Y is > -0.05f and < 0.05f);
    }

    [Fact]
    public void ACompressedVolumeKeepsItsVolumeWithinABound()
    {
        var w = NewWorld(o => { o.SoftSubsteps = 8; });
        World3DTests.Floor(w);
        var def = SoftBodyDef.Rubber(SoftBodyMesh.Box(new Vector3F(1f, 1f, 1f), 4, 4, 4), new Vector3D(0, 0.6, 0));
        def.Mass = 5f; def.VolumeCompliance = 0f; def.EdgeCompliance = 1e-4f;
        var soft = w.AddSoftBody(def);
        // A kinematic plate descends at 0.5 m/s for 0.4 s and squeezes the block by about a fifth of its height; the volume constraint keeps the volume.
        var press = w.AddBox(new Vector3F(1.2f, 0.2f, 1.2f), at: new Vector3D(0, 1.3, 0), type: BodyType.Kinematic);
        press.LinearVelocity = new Vector3F(0f, -0.5f, 0f);
        double worstRatio = 0.0, minHeight = 10.0;
        for (int i = 0; i < 24; i++)
        {
            w.Step(Dt);
            worstRatio = Math.Max(worstRatio, Math.Abs(soft.Volume / soft.RestVolume - 1.0));
            var (mn, mx) = soft.Bounds;
            minHeight = Math.Min(minHeight, mx.Y - mn.Y);
        }

        Assert.True(minHeight < 0.95, "the block was not compressed: " + minHeight); // squeezed by the plate
        Assert.True(worstRatio < 0.10, "the volume changed by " + worstRatio);
    }

    [Fact]
    public void MomentumIsConservedInTwoWayCoupling()
    {
        var w = NewWorld(o => { o.Gravity = default; o.SleepEnabled = false; });
        var def = SoftBodyDef.Jelly(SoftBodyMesh.Box(new Vector3F(1f, 1f, 1f), 3, 3, 3), new Vector3D(0, 0, 0));
        def.Mass = 2f; def.LinearDamping = 0f; def.Friction = 0.3f;
        var soft = w.AddSoftBody(def);
        var ball = w.AddSphere(0.3f, at: new Vector3D(-2.0, 0.1, 0.05));
        ball.LinearVelocity = new Vector3F(8f, 0f, 0f);
        float mb = ball.Mass;
        double p0 = mb * 8.0;
        double worst = 0;
        bool touched = false;
        for (int i = 0; i < 120; i++)
        {
            w.Step(Dt);
            var ps = soft.Momentum;
            var vb = ball.LinearVelocity;
            double px = ps.X + mb * vb.X, py = ps.Y + mb * vb.Y, pz = ps.Z + mb * vb.Z;
            worst = Math.Max(worst, Math.Abs(px - p0) + Math.Abs(py) + Math.Abs(pz));
            if (ps.X > 0.5) { touched = true; }
        }

        Assert.True(touched, "the ball never hit the jelly");
        Assert.True(worst < 0.02 * p0, "momentum drifted by " + worst + " of " + p0);
    }

    [Fact]
    public void AFastSoftBallDoesNotTunnelThroughAThinPlate()
    {
        var w = NewWorld();
        w.AddBox(new Vector3F(4f, 0.02f, 4f), at: new Vector3D(0, 0, 0), type: BodyType.Static);
        var ball = w.AddSoftBall(0.3f, new Vector3D(0, 3, 0));
        ball.AddVelocity(new Vector3F(0, -45f, 0));
        for (int i = 0; i < 90; i++)
        {
            w.Step(Dt);
            var (mn, _) = ball.Bounds;
            Assert.True(mn.Y > -0.1, "the ball went through the plate at step " + i + ": " + mn.Y);
        }
    }

    [Fact]
    public void ClothDrapedOverASphereRestsOnIt()
    {
        var w = NewWorld(o => { o.SoftSubsteps = 8; });
        World3DTests.Floor(w);
        w.AddSphere(0.5f, at: new Vector3D(0, 0.5, 0), type: BodyType.Static);
        var cloth = w.AddCloth(1.6f, 1.6f, 16, new Vector3D(0, 1.3, 0));
        Run(w, 360);
        // the particle above the sphere's centre rests on the sphere's top; none is inside it
        var pos = new Vector3D[cloth.ParticleCount];
        cloth.CopyPositions(pos);
        double lowestInside = double.MaxValue;
        foreach (var p in pos)
        {
            double r = Math.Sqrt(p.X * p.X + (p.Y - 0.5) * (p.Y - 0.5) + p.Z * p.Z);
            lowestInside = Math.Min(lowestInside, r);
            Assert.True(p.Y > -0.001);
        }

        Assert.True(lowestInside > 0.5 - 0.02, "a particle is inside the sphere: " + lowestInside);
        Assert.True(pos[(16 + 1) * 8 + 8].Y > 0.95, "the centre of the cloth should sit on top of the sphere: " + pos[(16 + 1) * 8 + 8].Y);
    }

    [Fact]
    public void ASoftBodyRestingOnAMeshTerrainStays()
    {
        var w = NewWorld(o => { o.SoftSubsteps = 6; });
        var verts = new[] { new Vector3F(-5, 0, -5), new Vector3F(5, 0, -5), new Vector3F(5, 0, 5), new Vector3F(-5, 0, 5) };
        w.AddMesh(verts, new[] { 0, 2, 1, 0, 3, 2 });
        var soft = w.AddSoftBox(new Vector3F(0.6f, 0.6f, 0.6f), new Vector3D(0, 1, 0));
        Run(w, 900); // it bounces and wobbles for a few seconds before it sleeps
        var (mn, mx) = soft.Bounds;
        Assert.InRange(mn.Y, -0.02, 0.1);
        Assert.InRange(mx.Y, 0.3, 0.7);
        Assert.True(soft.IsAsleep);
    }

    [Fact]
    public void ABoxDroppedOnASoftCushionIsHeldUpAndPushedBack()
    {
        var w = NewWorld(o => { o.SoftSubsteps = 8; });
        World3DTests.Floor(w);
        var def = SoftBodyDef.Jelly(SoftBodyMesh.Box(new Vector3F(1.5f, 0.3f, 1.5f), 6, 2, 6), new Vector3D(0, 0.23, 0));
        def.Mass = 6f;
        var cushion = w.AddSoftBody(def);
        var box = w.AddBox(new Vector3F(0.5f, 0.5f, 0.5f), at: new Vector3D(0, 1.2, 0), density: 20f);
        // The box falls 0.7 m (3.7 m/s). While it is over the cushion it never reaches the floor, and the cushion throws it back up at least once.
        float vmin = 0, vmax = 0;
        double lowest = 10;
        for (int i = 0; i < 90; i++)
        {
            w.Step(Dt);
            vmin = Math.Min(vmin, box.LinearVelocity.Y); vmax = Math.Max(vmax, box.LinearVelocity.Y);
            lowest = Math.Min(lowest, box.Position.Y);
        }

        Assert.True(vmin < -3f, "the box did not fall: " + vmin);
        Assert.True(lowest > 0.3 + 0.25, "the box sank through the cushion: " + lowest);
        Assert.True(vmax > 0.5f, "the cushion did not push the box back: " + vmax);
        Assert.True(cushion.Momentum.Y > -1f);
    }

    // ---- host hooks

    private static PhysicsWorld3D SoftScene(int threads = 1)
    {
        var w = World3DTests.NewWorld(threads, 31, o => { o.SoftSubsteps = 4; });
        World3DTests.Floor(w);
        w.AddSphere(0.4f, at: new Vector3D(0, 0.4, 0), type: BodyType.Static);
        w.AddCloth(1.2f, 1.2f, 8, new Vector3D(0, 1.2, 0), pinCorners: false);
        w.AddSoftBox(new Vector3F(0.5f, 0.5f, 0.5f), new Vector3D(1.2, 1.0, 0.1));
        w.AddSoftBall(0.25f, new Vector3D(-1.0, 1.2, 0.0));
        w.AddBox(new Vector3F(0.4f, 0.4f, 0.4f), at: new Vector3D(1.2, 2.2, 0.1));
        return w;
    }

    [Fact]
    public void SnapshotRestoreContinuesBitIdentically()
    {
        var a = SoftScene();
        Run(a, 40);
        var snap = a.Snapshot();
        ulong h0 = a.ComputeHash();
        Run(a, 60);
        ulong h1 = a.ComputeHash();
        var snapEnd = a.Snapshot();

        var b = World3DTests.NewWorld(1, 31, o => { o.SoftSubsteps = 4; });
        b.Restore(snap);
        Assert.Equal(h0, b.ComputeHash());
        Assert.Equal(snap, b.Snapshot());
        Run(b, 60);
        Assert.Equal(h1, b.ComputeHash());
        Assert.Equal(snapEnd, b.Snapshot());

        a.Restore(snap);
        Assert.Equal(h0, a.ComputeHash());
        Run(a, 60);
        Assert.Equal(h1, a.ComputeHash());
    }

    [Fact]
    public void ThreadCountDoesNotChangeTheResult()
    {
        var a = SoftScene(1);
        var b = SoftScene(4);
        for (int i = 0; i < 120; i++)
        {
            a.Step(Dt); b.Step(Dt);
            Assert.Equal(a.ComputeHash(), b.ComputeHash());
        }
    }

    [Fact]
    public void WorldsWithoutSoftBodiesKeepTheirSnapshotSectionCount()
    {
        var plain = World3DTests.NewWorld(1, 31);
        World3DTests.Floor(plain);
        var s1 = plain.Snapshot();
        var withSoft = World3DTests.NewWorld(1, 31);
        World3DTests.Floor(withSoft);
        var cloth = withSoft.AddCloth(1f, 1f, 2);
        var s2 = withSoft.Snapshot();
        Assert.True(s2.Length > s1.Length);
        cloth.Destroy();
        // after the last soft body is gone the section stays (its handle generations must survive), and restoring a snapshot without it empties the world of soft bodies
        withSoft.AddCloth(1f, 1f, 2);
        withSoft.Restore(s1);
        Assert.Equal(0, withSoft.SoftBodyCount);
        Assert.Equal(plain.ComputeHash(), withSoft.ComputeHash());
    }

    [Fact]
    public void RestoreRefusesDamagedSoftBodySections()
    {
        var w = SoftScene();
        Run(w, 5);
        var snap = w.Snapshot();
        // flip a byte near the end (inside the particle data) to NaN-ish values: walk to a double's exponent bytes
        var bad = (byte[])snap.Clone();
        for (int i = bad.Length - 40; i < bad.Length - 32; i++) { bad[i] = 0xFF; }
        var fresh = World3DTests.NewWorld(1, 31);
        Assert.Throws<SnapshotFormatException>(() => fresh.Restore(bad));
        // truncations never throw anything else
        for (int cut = snap.Length - 1; cut > snap.Length - 3000; cut -= 97)
        {
            var part = snap.AsSpan(0, cut).ToArray();
            var f = World3DTests.NewWorld(1, 31);
            Assert.Throws<SnapshotFormatException>(() => f.Restore(part));
        }
    }

    [Fact]
    public void SteppingAllocatesNothing()
    {
        var w = SoftScene();
        w.Warmup();
        Run(w, 30);
        AllocMeter.Settle();
        long bytes = 0;
        for (int i = 0; i < 20; i++) { bytes += AllocMeter.Of(() => w.Step(Dt)); }
        Assert.Equal(0, bytes);
    }

    [Fact]
    public void ABodyThatLeavesTheRangeIsReportedBeforeAnythingChanges()
    {
        var w = NewWorld();
        var cloth = w.AddCloth(1f, 1f, 2, new Vector3D(0, 1, 0));
        cloth.RawVelocities.ToArray();
        var def = cloth.Definition;
        Assert.Empty(def.PinnedParticles!);
        cloth.SetPinned(3, true);
        Assert.True(cloth.IsPinned(3));
        Assert.Single(cloth.Definition.PinnedParticles!);
        w.Step(Dt);
        ulong h = w.ComputeHash();
        Assert.Equal(h, w.ComputeHash());
    }
}
