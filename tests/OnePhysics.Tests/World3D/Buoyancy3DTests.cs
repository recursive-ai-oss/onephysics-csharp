using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

public class Buoyancy3DTests
{
    private const float Dt = 1f / 60f;
    private static readonly Vector3F Up = new(0f, 1f, 0f);

    private static PhysicsWorld3D NewWorld(int id = 71) => new(new WorldOptions3D { ThreadCount = 1, WorldId = id });

    private static void Run(PhysicsWorld3D w, int steps) { for (int i = 0; i < steps; i++) { w.Step(Dt); } }

    // ---- the geometry against brute force --------------------------------------------------------

    private static bool Inside(in ConvexShape3D shape, Vector3F p)
    {
        switch (shape.Kind)
        {
            case ConvexShapeKind3D.Sphere:
                return p.X * p.X + p.Y * p.Y + p.Z * p.Z <= shape.ConvexRadius * shape.ConvexRadius;
            case ConvexShapeKind3D.Box:
                {
                    var h = shape.HalfExtents;
                    return Math.Abs(p.X) <= h.X && Math.Abs(p.Y) <= h.Y && Math.Abs(p.Z) <= h.Z;
                }

            case ConvexShapeKind3D.Cylinder:
                {
                    var h = shape.HalfExtents;
                    return p.X * p.X + p.Z * p.Z <= h.X * h.X && Math.Abs(p.Y) <= h.Y;
                }

            case ConvexShapeKind3D.Capsule:
                {
                    float hh = shape.HalfExtents.Y - shape.ConvexRadius;
                    float y = Math.Clamp(p.Y, -hh, hh);
                    return p.X * p.X + (p.Y - y) * (p.Y - y) + p.Z * p.Z <= shape.ConvexRadius * shape.ConvexRadius;
                }

            default:
                return false;
        }
    }

    private static void BruteForce(in ConvexShape3D shape, in RigidTransformF pose, Vector3F n, float offset, out double volume, out Vector3F centroid)
    {
        var half = shape.HalfExtents;
        float reach = half.X + half.Y + half.Z;
        const int G = 96;
        double cell = Math.Pow(2.0 * reach / G, 3);
        double sx = 0, sy = 0, sz = 0;
        long count = 0;
        var inv = pose.Rotation.Conjugated();
        for (int i = 0; i < G; i++)
        {
            for (int j = 0; j < G; j++)
            {
                for (int k = 0; k < G; k++)
                {
                    var local = new Vector3F(-reach + (i + 0.5f) * 2f * reach / G, -reach + (j + 0.5f) * 2f * reach / G, -reach + (k + 0.5f) * 2f * reach / G);
                    if (!Inside(shape, local)) { continue; }
                    var w = pose.Rotation.Rotate(local) + pose.Translation;
                    if (n.X * w.X + n.Y * w.Y + n.Z * w.Z + offset > 0f) { continue; }
                    count++; sx += w.X; sy += w.Y; sz += w.Z;
                }
            }
        }

        volume = count * cell;
        centroid = count > 0 ? new Vector3F((float)(sx / count), (float)(sy / count), (float)(sz / count)) : default;
    }

    private static QuaternionF About(Vector3F axis, float angle)
    {
        double h = 0.5 * angle;
        float s = (float)Math64.Sin(h), c = (float)Math64.Cos(h);
        return new QuaternionF(axis.X * s, axis.Y * s, axis.Z * s, c);
    }

    public static TheoryData<int> ShapeKinds() => new() { 0, 1, 2, 3 };

    [Theory]
    [MemberData(nameof(ShapeKinds))]
    public void SubmergedVolumeMatchesBruteForce(int kind)
    {
        ConvexShape3D shape = kind switch
        {
            0 => ConvexShape3D.Sphere(0.6f),
            1 => ConvexShape3D.Box(new Vector3F(0.5f, 0.3f, 0.4f)),
            2 => ConvexShape3D.Cylinder(0.5f, 0.35f),
            _ => ConvexShape3D.Capsule(0.4f, 0.25f),
        };
        var rng = new Random(1234 + kind);
        for (int trial = 0; trial < 12; trial++)
        {
            var axis = new Vector3F((float)rng.NextDouble() - 0.5f, (float)rng.NextDouble() - 0.5f, (float)rng.NextDouble() - 0.5f);
            axis = axis * (1f / (float)Math.Sqrt(axis.X * axis.X + axis.Y * axis.Y + axis.Z * axis.Z));
            var pose = new RigidTransformF(About(axis, (float)(rng.NextDouble() * 6.0)), new Vector3F(0.1f, -0.2f, 0.05f));
            var nn = new Vector3F((float)rng.NextDouble() - 0.5f, (float)rng.NextDouble() + 0.2f, (float)rng.NextDouble() - 0.5f);
            nn = nn * (1f / (float)Math.Sqrt(nn.X * nn.X + nn.Y * nn.Y + nn.Z * nn.Z));
            float offset = ((float)rng.NextDouble() - 0.5f) * 1.2f;
            var got = Buoyancy3D.Submerged(shape, pose, nn, offset);
            BruteForce(shape, pose, nn, offset, out double vol, out var cen);
            double full = shape.GetMassProperties().Mass;
            Assert.True(Math.Abs(got.Volume - vol) <= 0.02 * full, $"kind {kind} trial {trial}: volume {got.Volume} vs brute force {vol} (full {full})");
            if (vol > 0.05 * full)
            {
                var d = got.Centroid - cen;
                float err = (float)Math.Sqrt(d.X * d.X + d.Y * d.Y + d.Z * d.Z);
                Assert.True(err <= 0.04f, $"kind {kind} trial {trial}: centroid off by {err}");
            }
        }
    }

    [Fact]
    public void BoxAndSphereAreExact()
    {
        // A 1 x 1 x 1 box with its bottom half below the surface.
        var box = ConvexShape3D.Box(new Vector3F(0.5f, 0.5f, 0.5f));
        var r = Buoyancy3D.Submerged(box, RigidTransformF.Identity, Up, 0f);
        Assert.Equal(0.5f, r.Volume, 6);
        Assert.Equal(-0.25f, r.Centroid.Y, 6);

        // A sphere at the surface: a hemisphere, centroid 3r/8 below the centre.
        var ball = ConvexShape3D.Sphere(2f);
        var h = Buoyancy3D.Submerged(ball, RigidTransformF.Identity, Up, 0f);
        Assert.Equal(2f / 3f * Math.PI * 8.0, h.Volume, 3);
        Assert.Equal(-0.75f, h.Centroid.Y, 5);

        // Clear of the fluid: nothing. Whole: the full volume.
        Assert.Equal(0f, Buoyancy3D.Submerged(box, new RigidTransformF(QuaternionF.Identity, new Vector3F(0, 5, 0)), Up, 0f).Volume);
        Assert.Equal(1f, Buoyancy3D.Submerged(box, new RigidTransformF(QuaternionF.Identity, new Vector3F(0, -5, 0)), Up, 0f).Volume, 6);
    }

    [Fact]
    public void HullMatchesTheBoxItIsMadeFrom()
    {
        var pts = new Vector3F[8];
        for (int i = 0; i < 8; i++) { pts[i] = new Vector3F((i & 1) == 0 ? -0.5f : 0.5f, (i & 2) == 0 ? -0.5f : 0.5f, (i & 4) == 0 ? -0.5f : 0.5f); }
        var hull = ConvexShape3D.FromHull(ConvexHull3D.Build(pts));
        var box = ConvexShape3D.Box(new Vector3F(0.5f, 0.5f, 0.5f));
        var pose = new RigidTransformF(About(new Vector3F(0.6f, 0f, 0.8f), 0.7f), new Vector3F(0f, 0.1f, 0f));
        var a = Buoyancy3D.Submerged(hull, pose, Up, 0.05f, new Vector3F(1, 0, 0));
        var b = Buoyancy3D.Submerged(box, pose, Up, 0.05f, new Vector3F(1, 0, 0));
        Assert.Equal(b.Volume, a.Volume, 5);
        Assert.Equal(b.ProjectedArea, a.ProjectedArea, 5);
        Assert.Equal(b.Centroid.Y, a.Centroid.Y, 5);
    }

    [Fact]
    public void TaperedCapsuleWholeVolumeIsClose()
    {
        var shape = ConvexShape3D.TaperedCapsule(0.5f, 0.2f, 0.35f);
        var got = Buoyancy3D.Submerged(shape, new RigidTransformF(QuaternionF.Identity, new Vector3F(0, -10, 0)), Up, 0f);
        double full = shape.GetMassProperties().Mass;
        Assert.InRange(got.Volume / full, 0.98, 1.02);
        // Half in the water, by symmetry of nothing: the volume is between zero and the whole, and the centroid is below the surface.
        var part = Buoyancy3D.Submerged(shape, RigidTransformF.Identity, Up, 0f);
        Assert.InRange(part.Volume, 0.2 * full, 0.8 * full);
        Assert.True(part.Centroid.Y < 0f);
    }

    [Fact]
    public void ProjectedAreaOfAFullyUnderwaterBoxIsItsFace()
    {
        var box = ConvexShape3D.Box(new Vector3F(0.5f, 1f, 1.5f));
        var pose = new RigidTransformF(QuaternionF.Identity, new Vector3F(0, -9, 0));
        Assert.Equal(6f, Buoyancy3D.Submerged(box, pose, Up, 0f, new Vector3F(1, 0, 0)).ProjectedArea, 4);    // 2 x 3
        Assert.Equal(2f, Buoyancy3D.Submerged(box, pose, Up, 0f, new Vector3F(0, 0, 1)).ProjectedArea, 4);    // 1 x 2
    }

    // ---- in the world ----------------------------------------------------------------------------

    [Fact]
    public void BoxOfHalfTheWatersDensityFloatsAtHalfDepth()
    {
        var w = NewWorld();
        w.AddWater(0.0);
        var box = w.AddBox(new Vector3F(1f, 1f, 1f), at: new Vector3D(0, 0.3, 0), density: 500f);
        Run(w, 900);
        Assert.True(Math.Abs(box.Position.Y) < 0.03, $"y {box.Position.Y} awake {box.IsAwake} v {box.LinearVelocity}");   // half of the cube below the surface puts its centre on the surface
        Assert.True(box.LinearVelocity.Y * box.LinearVelocity.Y < 1e-4f);
    }

    [Fact]
    public void DenserBodySinksAndLighterOnesRiseToTheSurface()
    {
        var w = NewWorld();
        w.AddWater(0.0);
        var stone = w.AddSphere(0.25f, at: new Vector3D(0, -1, 0), density: 2500f);
        var cork = w.AddSphere(0.25f, at: new Vector3D(3, -3, 0), density: 250f);
        Run(w, 60);
        Assert.True(stone.LinearVelocity.Y < 0f);
        Assert.True(cork.LinearVelocity.Y > 0f);
        Run(w, 900);
        Assert.True(cork.Position.Y > -0.25 && cork.Position.Y < 0.25);   // floats with a quarter of its volume under
    }

    [Fact]
    public void CapsizedHullRightsItself()
    {
        var w = NewWorld();
        w.AddWater(0.0);
        var pts = new Vector3F[8];
        for (int i = 0; i < 8; i++) { pts[i] = new Vector3F((i & 1) == 0 ? -1f : 1f, (i & 2) == 0 ? -0.2f : 0.2f, (i & 4) == 0 ? -1f : 1f); }
        var hull = w.AddConvexHull(pts, at: new Vector3D(0, 0, 0), rotation: About(new Vector3F(0, 0, 1), 1.2f), density: 400f);
        Run(w, 1500);
        var upAxis = hull.Rotation.Rotate(new Vector3F(0, 1, 0));
        Assert.True(upAxis.Y > 0.99f, "up axis " + upAxis);
    }

    [Fact]
    public void DragGivesTheAnalyticTerminalVelocity()
    {
        var w = NewWorld();
        const float r = 0.25f, rho = 1000f, cd = 0.5f, density = 2000f;
        w.AddFluid(new FluidDef { SurfaceOffset = 1000.0, Density = rho, LinearDrag = cd, ViscousDamping = 0f });
        var ball = w.AddSphere(r, at: new Vector3D(0, 0, 0), density: density);
        Run(w, 600);
        double vol = 4.0 / 3.0 * Math.PI * r * r * r, area = Math.PI * r * r;
        double expected = Math.Sqrt(2.0 * (density - rho) * vol * 9.81 / (rho * cd * area));
        Assert.Equal(expected, -ball.LinearVelocity.Y, expected * 0.01);
    }

    [Fact]
    public void FlowCarriesANeutralBody()
    {
        var w = NewWorld();
        w.AddWater(100.0, flow: new Vector3F(2f, 0f, 0f));
        var ball = w.AddSphere(0.3f, at: new Vector3D(0, 0, 0), density: 1000f);
        Run(w, 2400);
        Assert.Equal(2f, ball.LinearVelocity.X, 0.05f);
        Assert.InRange(ball.LinearVelocity.Y, -0.02f, 0.02f);
    }

    [Fact]
    public void CompoundSumsItsChildren()
    {
        var w = NewWorld();
        var parts = new[]
        {
            new CompoundPart(ShapeSpec3D.Box(new Vector3F(0.5f, 0.5f, 0.5f)), new Vector3F(-1f, 0f, 0f)),
            new CompoundPart(ShapeSpec3D.Box(new Vector3F(0.25f, 0.25f, 0.25f)), new Vector3F(1f, 0f, 0f)),
        };
        var b = w.AddCompound(parts, at: new Vector3D(0, -20, 0));
        float whole = w.ApplyBuoyancy(b, new FluidDef(), Dt);
        Assert.Equal(1f + 0.125f, whole, 4);
        Assert.Equal(0f, w.ApplyBuoyancy(b, new FluidDef { SurfaceOffset = -100.0 }, Dt));
    }

    [Fact]
    public void MeshesAndSensorsDoNotFloat()
    {
        var w = NewWorld();
        w.AddWater(10.0);
        var verts = new[] { new Vector3F(-5, 0, -5), new Vector3F(5, 0, -5), new Vector3F(0, 0, 5) };
        w.AddMesh(verts, new[] { 0, 1, 2 }, at: new Vector3D(0, -5, 0));
        Run(w, 10);   // nothing to check beyond not throwing: a static mesh is not dynamic
        var ball = w.AddSphere(0.5f, at: new Vector3D(0, 0, 0), density: 1f);
        ball.BuoyancyFactor = 0f;
        float before = (float)ball.Position.Y;
        Run(w, 30);
        Assert.True(ball.Position.Y < before);   // factor 0: no lift, so it falls
    }

    [Fact]
    public void RemovedFluidStopsActing()
    {
        var w = NewWorld();
        var f = w.AddWater(0.0);
        var cork = w.AddSphere(0.25f, at: new Vector3D(0, -2, 0), density: 100f);
        Run(w, 30);
        Assert.True(cork.LinearVelocity.Y > 0f);
        f.Remove();
        Assert.False(f.IsValid);
        Assert.Throws<StaleHandleException>(() => f.Flow);
        Assert.Equal(0, w.FluidCount);
    }

    // ---- hostile input ---------------------------------------------------------------------------

    [Fact]
    public void BadDefinitionsAreRejectedWithTypedErrors()
    {
        var w = NewWorld();
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddFluid(new FluidDef { Density = 0f }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddFluid(new FluidDef { Density = -1f }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddFluid(new FluidDef { Density = float.NaN }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddFluid(new FluidDef { Density = float.PositiveInfinity }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddFluid(new FluidDef { SurfaceOffset = double.NaN }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddFluid(new FluidDef { SurfaceOffset = 1e30 }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddFluid(new FluidDef { SurfaceNormal = default }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddFluid(new FluidDef { SurfaceNormal = new Vector3F(float.NaN, 1, 0) }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddFluid(new FluidDef { Flow = new Vector3F(0, float.NaN, 0) }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddFluid(new FluidDef { Flow = new Vector3F(1e9f, 0, 0) }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddFluid(new FluidDef { LinearDrag = -1f }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddFluid(new FluidDef { AngularDrag = float.NaN }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddFluid(new FluidDef { LimitToRegion = true, RegionMin = new Vector3D(1, 0, 0), RegionMax = new Vector3D(0, 1, 1) }));
        Assert.Throws<ArgumentNullException>(() => w.AddFluid(null!));
        var ex = Assert.Throws<ArgumentOutOfRangeException>(() => w.AddFluid(new FluidDef { Density = 0f }));
        Assert.Contains("Density", ex.Message);
        Assert.Equal(0, w.FluidCount);
        var b = w.AddBox(new Vector3F(1, 1, 1));
        Assert.Throws<ArgumentOutOfRangeException>(() => b.BuoyancyFactor = float.NaN);
        Assert.Throws<ArgumentOutOfRangeException>(() => b.BuoyancyFactor = -1f);
        Assert.Throws<ArgumentOutOfRangeException>(() => w.ApplyBuoyancy(b, new FluidDef(), 0f));
    }

    // ---- snapshots -------------------------------------------------------------------------------

    private static PhysicsWorld3D Pool(int id)
    {
        var w = NewWorld(id);
        w.AddBox(new Vector3F(40f, 1f, 40f), at: new Vector3D(0, -5.5, 0), type: BodyType.Static);
        w.AddFluid(new FluidDef { SurfaceOffset = 0.5, Flow = new Vector3F(0.5f, 0f, 0.25f), LimitToRegion = true, RegionMin = new Vector3D(-20, -5, -20), RegionMax = new Vector3D(20, 3, 20) });
        for (int i = 0; i < 6; i++)
        {
            var at = new Vector3D(-2 + i * 0.9, 1.0 + 0.4 * i, 0.3 * i);
            var b = (i % 3) switch
            {
                0 => w.AddBox(new Vector3F(0.6f, 0.4f, 0.5f), at: at, density: 300f + 150f * i),
                1 => w.AddSphere(0.3f, at: at, density: 700f),
                _ => w.AddCapsule(0.3f, 0.15f, at: at, density: 900f),
            };
            if (i == 2) { b.BuoyancyFactor = 1.5f; }
        }

        return w;
    }

    [Fact]
    public void SnapshotRoundTripIsByteExact()
    {
        var w = Pool(72);
        Run(w, 90);
        var snap = w.Snapshot();
        var hashes = new ulong[120];
        for (int i = 0; i < 120; i++) { w.Step(Dt); hashes[i] = w.ComputeHash(); }
        var end = w.Snapshot();

        w.Restore(snap);
        Assert.Equal(snap, w.Snapshot());
        for (int i = 0; i < 120; i++) { w.Step(Dt); Assert.Equal(hashes[i], w.ComputeHash()); }
        Assert.Equal(end, w.Snapshot());

        var fresh = NewWorld(72);
        fresh.Restore(snap);
        Assert.Equal(snap, fresh.Snapshot());
        Assert.Equal(1, fresh.FluidCount);
        for (int i = 0; i < 120; i++) { fresh.Step(Dt); Assert.Equal(hashes[i], fresh.ComputeHash()); }
        Assert.Equal(end, fresh.Snapshot());
    }

    [Fact]
    public void WorldWithoutFluidsKeepsItsBytes()
    {
        var a = NewWorld(73);
        a.AddBox(new Vector3F(1, 1, 1));
        var before = a.Snapshot();
        var f = a.AddWater(0.0);
        Assert.NotEqual(before, a.Snapshot());
        f.Remove();
        Assert.Equal(before, a.Snapshot());
    }

    [Fact]
    public void DamagedFluidBlocksAreRejectedAndLeaveTheWorldUnchanged()
    {
        var w = Pool(74);
        Run(w, 10);
        var snap = w.Snapshot();
        var rng = new Random(5);
        var other = Pool(74);
        Run(other, 10);
        var keep = other.Snapshot();
        int rejected = 0;
        for (int i = 0; i < 300; i++)
        {
            var bad = (byte[])snap.Clone();
            // Damage the tail, where the fluid block lives.
            int at = bad.Length - 1 - rng.Next(Math.Min(bad.Length - 1, 260));
            bad[at] ^= (byte)(1 << rng.Next(8));
            try { other.Restore(bad); }
            catch (SnapshotFormatException) { rejected++; Assert.Equal(keep, other.Snapshot()); }
            other.Restore(keep);
        }

        Assert.True(rejected > 0);
    }

    [Fact]
    public void ShiftingTheOriginMovesTheWater()
    {
        var w = NewWorld(75);
        var water = w.AddWater(4.0);
        var cork = w.AddSphere(0.25f, at: new Vector3D(0, 6, 0), density: 200f);
        Run(w, 1200);
        double settled = cork.Position.Y;
        Assert.InRange(settled, 4.0 - 0.3, 4.0 + 0.3);
        w.ShiftOrigin(new Vector3D(0, 2, 0));
        Assert.Equal(2.0, water.SurfaceOffset, 9);
        Run(w, 300);
        Assert.InRange(cork.Position.Y, 2.0 - 0.3, 2.0 + 0.3);
    }
}
