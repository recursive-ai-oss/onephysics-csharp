using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Aero;
using RecursiveAI.OnePhysics.Fields;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

// Fabric: anisotropy, strain limit, rest curvature, aerodynamics, tethers, tearing and plasticity on cloth, and the host hooks.
public class Cloth3DTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld3D NewWorld(int threads = 1, Action<WorldOptions3D>? tweak = null) => World3DTests.NewWorld(threads, 31, tweak);

    private static void Run(PhysicsWorld3D w, int steps) { for (int i = 0; i < steps; i++) { w.Step(Dt); } }

    private static double MaxEdgeStrain(SoftBody b)
    {
        var pos = b.RawPositions;
        var mesh = b.Mesh;
        var tris = mesh.Triangles;
        double worst = 0.0;
        for (int t = 0; t < tris.Length / 3; t++)
        {
            for (int k = 0; k < 3; k++)
            {
                int a = tris[3 * t + k], c = tris[3 * t + (k + 1) % 3];
                double rest = Dist(mesh.Vertices[a], mesh.Vertices[c]);
                double dx = pos[3 * a] - pos[3 * c], dy = pos[3 * a + 1] - pos[3 * c + 1], dz = pos[3 * a + 2] - pos[3 * c + 2];
                worst = Math.Max(worst, Math.Sqrt(dx * dx + dy * dy + dz * dz) / rest - 1.0);
            }
        }

        return worst;
    }

    private static double Dist(Vector3F a, Vector3F b)
    {
        double dx = (double)a.X - b.X, dy = (double)a.Y - b.Y, dz = (double)a.Z - b.Z;
        return Math.Sqrt(dx * dx + dy * dy + dz * dz);
    }

    private static ClothMaterial Light() => new() { ArealDensity = 0.3f, LinearDamping = 0f, Damping = 0f, StrainLimit = 0f, Drag = 0f };

    [Fact]
    public void AnisotropicWeaveStretchesMoreAlongTheSofterThread()
    {
        // The same load hangs a sheet by its warp edge and by its weft edge: the softer direction elongates more, and the strain follows the compliance ratio.
        double Sag(bool hangByWeft)
        {
            var w = NewWorld();
            var m = Light(); m.ArealDensity = 2f; m.WarpCompliance = 1e-4f; m.WeftCompliance = 2e-3f; m.ShearCompliance = 1e-3f; m.BendCompliance = 1e-3f;
            var def = new ClothDef { Width = 1f, Depth = 1f, SegmentsX = 8, SegmentsZ = 8, Position = new Vector3D(0, 3, 0), Material = m, SelfCollision = false, SurfaceCollision = false };
            // Rotate the sheet to stand on end: the pinned edge is the top; the long axis of the hang is z (weft) or x (warp).
            def.Rotation = hangByWeft ? new QuaternionF(0.7071068f, 0f, 0f, 0.7071068f) : new QuaternionF(0f, 0f, 0.7071068f, 0.7071068f);
            def.PinnedParticles = hangByWeft ? ClothDef.PinEdge(8, 8, 0) : ClothDef.PinEdge(8, 8, 1);
            var b = w.AddCloth(def);
            Run(w, 240);
            // length of the hanging column through the middle
            var p0 = b.GetPosition(hangByWeft ? 4 : 4 * 9 + 8); var p1 = b.GetPosition(hangByWeft ? 4 + 8 * 9 : 4 * 9);
            double len = (p0 - p1).Length();
            return len / 1.0 - 1.0;
        }

        double byWeft = Sag(true), byWarp = Sag(false);
        Assert.True(byWeft > 4.0 * byWarp, "weft stretch " + byWeft + " warp stretch " + byWarp);
        Assert.True(byWarp >= 0.0);
    }

    [Fact]
    public void StrainLimitHoldsUnderAHeavyHangingLoad()
    {
        double Strain(float limit)
        {
            var w = NewWorld();
            var m = Light(); m.ArealDensity = 40f; m.WarpCompliance = 5e-3f; m.WeftCompliance = 5e-3f; m.ShearCompliance = 5e-3f; m.StrainLimit = limit;
            var def = new ClothDef { Width = 1f, Depth = 1f, SegmentsX = 10, SegmentsZ = 10, Position = new Vector3D(0, 3, 0), Material = m, SelfCollision = false, SurfaceCollision = false };
            def.Rotation = new QuaternionF(0.7071068f, 0f, 0f, 0.7071068f);
            def.PinnedParticles = ClothDef.PinEdge(10, 10, 0);
            var b = w.AddCloth(def);
            Run(w, 300);
            return MaxEdgeStrain(b);
        }

        double free = Strain(0f), limited = Strain(0.05f);
        Assert.True(free > 0.15, "the unlimited cloth stretches " + free);
        Assert.True(limited <= 0.06, "limited strain " + limited);
    }

    // The largest distance between the strip's particles and its rest shape, after turning the rest shape about the pinned edge (the z axis there) to the best angle: the pins fix the edge's position, and nothing holds its tilt.
    private static double RestShapeDeviation(SoftBody b)
    {
        var mesh = b.Mesh;
        var pos = b.RawPositions;
        int n = b.ParticleCount;
        // the pivot: the pinned vertex 0, in the strip's x-y plane
        double px = pos[0], py = pos[1];
        double tipX = pos[3 * (n - 1)] - px, tipY = pos[3 * (n - 1) + 1] - py;
        double rx = mesh.Vertices[n - 1].X - mesh.Vertices[0].X, ry = mesh.Vertices[n - 1].Y - mesh.Vertices[0].Y;
        double phi = Math.Atan2(tipY, tipX) - Math.Atan2(ry, rx), cs = Math.Cos(phi), sn = Math.Sin(phi);
        double worst = 0.0;
        for (int i = 0; i < n; i++)
        {
            double vx = mesh.Vertices[i].X - mesh.Vertices[0].X, vy = mesh.Vertices[i].Y - mesh.Vertices[0].Y, vz = mesh.Vertices[i].Z - mesh.Vertices[0].Z;
            double ex = px + cs * vx - sn * vy, ey = py + sn * vx + cs * vy, ez = pos[2] + vz;
            double dx = pos[3 * i] - ex, dy = pos[3 * i + 1] - ey, dz = pos[3 * i + 2] - ez;
            worst = Math.Max(worst, Math.Sqrt(dx * dx + dy * dy + dz * dz));
        }

        return worst;
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ARestCurvedStripCurlsFromFlatAndHoldsItsRestShape(bool underGravity)
    {
        // A strip 0.6 m long made flat, with a rest curvature of 2/m, pinned along one end, curls to the cylinder of its rest shape (hanging from its pins when there is gravity).
        var w = NewWorld(1, o => { if (!underGravity) { o.Gravity = default; } });
        var m = Light(); m.ArealDensity = underGravity ? 0.05f : 0.3f; m.BendCompliance = 1e-5f; m.Damping = 0.002f; m.LinearDamping = 1f; m.WarpCompliance = 1e-4f; m.WeftCompliance = 1e-4f; m.ShearCompliance = 1e-4f; m.StrainLimit = 0.01f;
        int nx = underGravity ? 8 : 24;
        var def = new ClothDef { Width = 0.6f, Depth = 0.2f, SegmentsX = nx, SegmentsZ = 2, Position = new Vector3D(0, 1, 0), Material = m, RestCurvatureX = 2f, StartCurved = underGravity, SelfCollision = false, SurfaceCollision = false, AllowSleep = false };
        def.PinnedParticles = ClothDef.PinEdge(nx, 2, 3);
        var b = w.AddCloth(def);
        Run(w, 480);
        double worst = RestShapeDeviation(b);
        Assert.True(worst < 0.02, "deviation from the rest shape " + worst + " m of a 0.6 m strip");
    }

    private static double FoldAngle(SoftBody b)
    {
        var p5 = b.GetPosition(5); var p6 = b.GetPosition(6); var p7 = b.GetPosition(7);
        var u = p6 - p5; var v = p7 - p6;
        double dot = (u.X * v.X + u.Y * v.Y + u.Z * v.Z) / (u.Length() * v.Length());
        return Math.Acos(Math.Min(1.0, dot));
    }

    [Fact]
    public void AFoldedStripWithAFlatRestShapeStraightensUnderTheDihedralBend()
    {
        var w = NewWorld(1, o => o.Gravity = default);
        var m = Light(); m.BendCompliance = 1e-4f;
        var def = new ClothDef { Width = 0.6f, Depth = 0.2f, SegmentsX = 12, SegmentsZ = 2, Position = new Vector3D(0, 1, 0), Material = m, SelfCollision = false, SurfaceCollision = false, AllowSleep = false };
        var sd = def.ToSoftBodyDef();
        sd.Fabric!.DihedralBend = true;
        // fold the right half upwards by 40 degrees about the middle line and start there
        var mesh = sd.Mesh!;
        var start = new Vector3F[mesh.VertexCount];
        float c = 0.76604444f, s = 0.64278764f;
        for (int i = 0; i < start.Length; i++)
        {
            var v = mesh.Vertices[i];
            start[i] = v.X > 0f ? new Vector3F(v.X * c, v.X * s, v.Z) : v;
        }

        sd.StartVertices = start;
        sd.LinearDamping = 1f;
        var b = w.AddSoftBody(sd);
        double before = FoldAngle(b);
        Run(w, 240);
        double after = FoldAngle(b);
        Assert.InRange(before, 0.65, 0.75);
        Assert.True(after < 0.03, "returns flat: " + after);
        // a free body keeps its momentum: the bend pushes nothing
        var vsum = new Vector3D();
        for (int i = 0; i < b.ParticleCount; i++) { var v = b.GetVelocity(i); vsum += new Vector3D(v.X, v.Y, v.Z); }
        Assert.True(vsum.Length() / b.ParticleCount < 0.01);
    }

    private static double TerminalSpeed(float massPerArea, float drag, float liftSlope)
    {
        // The equilibrium of weight and the plate force at normal incidence, from the library's own coefficient model.
        var c = AeroCoefficients.Plate(2f);
        c.LiftSlope = liftSlope; c.PlateDrag = drag; c.MinDrag = Math.Min(c.MinDrag, drag);
        double lo = 0.0, hi = 100.0;
        for (int i = 0; i < 80; i++)
        {
            double mid = 0.5 * (lo + hi);
            c.Force(0f, (float)mid, 1f, 1.225f, out _, out float fn);
            if (Math.Abs(fn) > massPerArea * 9.81) { hi = mid; } else { lo = mid; }
        }

        return 0.5 * (lo + hi);
    }

    [Fact]
    public void AFallingSheetReachesTheTerminalSpeedOfThePlateModel()
    {
        var w = NewWorld();
        var m = Light(); m.ArealDensity = 0.3f; m.Drag = 1.2f; m.LiftSlope = 2f;
        var def = new ClothDef { Width = 1f, Depth = 1f, SegmentsX = 6, SegmentsZ = 6, Position = new Vector3D(0, 500, 0), Material = m, SelfCollision = false, SurfaceCollision = false };
        var b = w.AddCloth(def);
        Run(w, 90);
        double vy = 0.0;
        for (int i = 0; i < b.ParticleCount; i++) { vy += b.GetVelocity(i).Y; }
        vy /= b.ParticleCount;
        double expected = TerminalSpeed(0.3f, 1.2f, 2f);
        Assert.InRange(-vy, expected * 0.97, expected * 1.03);
    }

    [Fact]
    public void WindPushesAClothAndItDoesNotExplode()
    {
        var w = NewWorld();
        w.AddField(new FieldDefinition { Kind = FieldKind.Area, Direction = default, Minimum = new Vector3D(-50, -50, -50), Maximum = new Vector3D(50, 50, 50), Flow = new Vector3F(0f, 0f, 8f) });
        var m = Light(); m.ArealDensity = 0.15f; m.Drag = 1.2f; m.LinearDamping = 0.1f; m.StrainLimit = 0.05f;
        // A flag: pinned along its left edge, in the x-y plane, wind along +z hits its face.
        var def = new ClothDef { Width = 1.2f, Depth = 0.8f, SegmentsX = 18, SegmentsZ = 12, Position = new Vector3D(0.6, 3, 0), Material = m, SelfCollision = true, SurfaceCollision = true };
        def.Rotation = new QuaternionF(0.7071068f, 0f, 0f, 0.7071068f);
        def.PinnedParticles = ClothDef.PinEdge(18, 12, 3);
        var b = w.AddCloth(def);
        double fastest = 0.0, farthest = 0.0;
        for (int step = 0; step < 300; step++)
        {
            w.Step(Dt);
            for (int i = 0; i < b.ParticleCount; i += 7)
            {
                var v = b.GetVelocity(i);
                fastest = Math.Max(fastest, Math.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z));
                farthest = Math.Max(farthest, (b.GetPosition(i) - new Vector3D(0, 3, 0)).Length());
            }
        }

        Assert.True(fastest < 30.0, "fastest particle " + fastest);
        Assert.True(farthest < 3.0, "farthest " + farthest);
        // downwind: the free edge has been blown towards +z
        double zsum = 0.0;
        for (int i = 0; i < b.ParticleCount; i++) { zsum += b.GetPosition(i).Z; }
        Assert.True(zsum / b.ParticleCount > 0.05, "mean z " + zsum / b.ParticleCount);
    }

    [Fact]
    public void TethersKeepAGarmentNearItsAnchors()
    {
        double Reach(bool tethers)
        {
            var w = NewWorld();
            var m = Light(); m.ArealDensity = 5f; m.WarpCompliance = 1e-2f; m.WeftCompliance = 1e-2f; m.ShearCompliance = 1e-2f; m.StrainLimit = 0f;
            var def = new ClothDef { Width = 1f, Depth = 1f, SegmentsX = 10, SegmentsZ = 10, Position = new Vector3D(0, 3, 0), Material = m, SelfCollision = false, SurfaceCollision = false };
            def.Rotation = new QuaternionF(0.7071068f, 0f, 0f, 0.7071068f);
            def.PinnedParticles = ClothDef.PinEdge(10, 10, 0);
            def.Tethers = tethers; def.TetherStrain = 0.02f;
            var b = w.AddCloth(def);
            Run(w, 300);
            double low = 10.0;
            for (int i = 0; i < b.ParticleCount; i++) { low = Math.Min(low, b.GetPosition(i).Y); }
            return 3.0 + 0.5 - low;     // hanging length measured from the pinned edge at y = 3.5
        }

        double free = Reach(false), tethered = Reach(true);
        Assert.True(free > 1.1, "untethered hang " + free);
        Assert.True(tethered < 1.06, "tethered hang " + tethered);
    }

    [Fact]
    public void ClothDefaultsToSurfaceCollisionAndEveryMaterialBuilds()
    {
        var w = NewWorld();
        foreach (var m in new[] { ClothMaterial.Cotton, ClothMaterial.Denim, ClothMaterial.Silk, ClothMaterial.Leather, ClothMaterial.RubberSheet })
        {
            var b = w.AddCloth(new ClothDef { Material = m, Position = new Vector3D(0, 2, 0), SegmentsX = 6, SegmentsZ = 6 });
            Assert.True(b.Definition.SurfaceCollision);
            Assert.NotNull(b.Definition.Fabric);
            Assert.InRange(b.Mass, 0.05f, 2f);
        }

        Run(w, 30);
    }

    [Fact]
    public void ClothSnapshotRestoresToTheSameBytesAndTheSameFuture()
    {
        PhysicsWorld3D Make()
        {
            var w = NewWorld();
            w.AddField(new FieldDefinition { Kind = FieldKind.Area, Direction = default, Minimum = new Vector3D(-50, -50, -50), Maximum = new Vector3D(50, 50, 50), Flow = new Vector3F(2f, 0f, 3f) });
            w.AddBox(new Vector3F(10f, 1f, 10f), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
            var m = ClothMaterial.Silk;
            var def = new ClothDef { Width = 1f, Depth = 1f, SegmentsX = 10, SegmentsZ = 10, Position = new Vector3D(0, 1.5, 0), Material = m, RestCurvatureX = 1.5f };
            def.PinnedParticles = ClothDef.PinEdge(10, 10, 0); def.Tethers = true;
            w.AddCloth(def);
            return w;
        }

        var a = Make();
        Run(a, 40);
        var snap = a.Snapshot();
        var b = Make();
        b.Restore(snap);
        Assert.Equal(snap, b.Snapshot());
        Assert.Equal(a.ComputeHash(), b.ComputeHash());
        for (int i = 0; i < 40; i++)
        {
            a.Step(Dt); b.Step(Dt);
            Assert.Equal(a.ComputeHash(), b.ComputeHash());
        }

        // a fork steps like the original
        var f = a.Fork();
        a.Step(Dt); f.Step(Dt);
        Assert.Equal(a.ComputeHash(), f.ComputeHash());
    }

    [Fact]
    public void ClothIsTheSameOnAnyThreadCount()
    {
        ulong Hash(int threads)
        {
            var w = NewWorld(threads);
            w.AddField(new FieldDefinition { Kind = FieldKind.Area, Direction = default, Minimum = new Vector3D(-50, -50, -50), Maximum = new Vector3D(50, 50, 50), Flow = new Vector3F(0f, 0f, 5f) });
            w.AddBox(new Vector3F(10f, 1f, 10f), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
            var def = new ClothDef { Width = 1.5f, Depth = 1.5f, SegmentsX = 20, SegmentsZ = 20, Position = new Vector3D(0, 1.2, 0), Material = ClothMaterial.Cotton };
            def.Rotation = new QuaternionF(0.5f, 0.1f, 0f, 0.86f);
            w.AddCloth(def);
            w.AddCloth(new ClothDef { Width = 1f, Depth = 1f, SegmentsX = 12, SegmentsZ = 12, Position = new Vector3D(0.2, 2.2, 0.1), Material = ClothMaterial.Denim });
            ulong h = 14695981039346656037UL;
            for (int i = 0; i < 120; i++) { w.Step(Dt); h = (h ^ w.ComputeHash()) * 1099511628211UL; }
            return h;
        }

        Assert.Equal(Hash(1), Hash(4));
    }

    [Fact]
    public void ClothParametersAreValidatedWithNamedErrors()
    {
        var w = NewWorld();
        var def = new ClothDef { Material = new ClothMaterial { ArealDensity = 0f } };
        var ex = Assert.Throws<SoftBodyException>(() => w.AddCloth(def));
        Assert.Contains("ArealDensity", ex.Message);
        var sd = new ClothDef().ToSoftBodyDef();
        sd.Fabric!.StrainLimit = float.NaN;
        var ex2 = Assert.Throws<SoftBodyException>(() => w.AddSoftBody(sd));
        Assert.Contains("StrainLimit", ex2.Message);
        sd = new ClothDef().ToSoftBodyDef();
        sd.Fabric!.TetherAnchors = new List<int> { 99999 };
        Assert.Throws<SoftBodyException>(() => w.AddSoftBody(sd));
        sd = new ClothDef().ToSoftBodyDef();
        sd.Fabric!.WeftAxis = sd.Fabric.WarpAxis;
        Assert.Throws<SoftBodyException>(() => w.AddSoftBody(sd));
    }
}
