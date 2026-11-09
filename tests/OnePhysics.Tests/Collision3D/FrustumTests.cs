using System;
using System.IO;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Collision3D;

public sealed class FrustumTests
{
    private static void Near(double expected, double actual, double tolerance = 1e-5) => Assert.InRange(actual, expected - tolerance, expected + tolerance);

    [Fact]
    public void ConeAndCylinderMassMatchAnalyticSolids()
    {
        var cone = ConvexShape3D.TaperedCylinder(2, 0, 3).GetMassProperties();
        double mass = Math.PI * 9 * 4 / 3;
        Near(mass, cone.Mass, 1e-5);
        Near(-1, cone.Center.Y);
        Near(0.3 * mass * 9, cone.Inertia.M22, 2e-5);
        Near(mass * (3.0 * 9 / 20 + 3.0 * 16 / 80), cone.Inertia.M11, 2e-5);
        var a = ConvexShape3D.TaperedCylinder(2, 3, 3);
        var b = ConvexShape3D.Cylinder(2, 3);
        Assert.Equal(b.Kind, a.Kind);
        Assert.Equal(b.GetMassProperties(), a.GetMassProperties());
        var reversed = ConvexShape3D.TaperedCylinder(2, 3, 0).GetMassProperties();
        Near(1, reversed.Center.Y);
        Assert.Equal(cone.Mass, reversed.Mass);
        Assert.Equal(cone.Inertia, reversed.Inertia);
    }

    [Fact]
    public void SupportBoundsScalingAndQueries()
    {
        var shape = ConvexShape3D.TaperedCylinder(1, 1, 2);
        Assert.Equal(new Vector3F(2, -1, 0), shape.Support(Vector3F.UnitX));
        Assert.Equal(new Vector3F(0, 1, 0), shape.Support(Vector3F.UnitY));
        var mass = shape.GetMassProperties();
        Near(Math.PI * 2 * 7 / 3, mass.Mass);
        Near(-3.0 / 14, mass.Center.Y);
        var scaled = ScaledShape3D.Create(shape, new Vector3F(2, 3, 2));
        Near(mass.Mass * 12, scaled.GetMassProperties().Mass, 4e-5);
        ShapeBounds3D.Compute(shape, RigidTransformF.Identity, out var lo, out var hi);
        Assert.True(lo.X <= -2 && hi.X >= 2 && lo.Y <= -1 && hi.Y >= 1);
        Assert.True(Gjk3D.RayCast(shape, RigidTransformF.Identity, new Vector3F(0, 3, 0), new Vector3F(0, -4, 0), 1, out var hit));
        Near(0.5, hit.Fraction, 2e-5);
        var d = Gjk3D.Distance(ConvexShape3D.Point, shape, new RigidTransformF(QuaternionF.Identity, new Vector3F(0, -3, 0)));
        Near(2, d.Distance, 2e-5);
    }

    [Fact]
    public void CapContactPairMatrix()
    {
        var floor = ConvexShape3D.TaperedCylinder(1, 2, 3);
        var hull = ConvexShape3D.FromHull(ConvexHull3D.Build([
            new(-0.3f,-0.3f,-0.3f), new(0.3f,-0.3f,-0.3f), new(-0.3f,0.3f,-0.3f), new(-0.3f,-0.3f,0.3f), new(0.3f,0.3f,0.3f)]));
        ConvexShape3D[] shapes = [ConvexShape3D.Sphere(0.3f), ConvexShape3D.Box(new Vector3F(0.3f,0.3f,0.3f)),
            ConvexShape3D.Capsule(0.2f,0.2f), ConvexShape3D.TaperedCapsule(0.3f,0.2f,0.3f), ConvexShape3D.Cylinder(0.3f,0.3f), hull,
            ConvexShape3D.TaperedCylinder(0.3f,0.2f,0.3f),
            ConvexShape3D.Triangle(new(-0.3f,0,-0.3f), new(0,0,0.3f), new(0.3f,0,-0.3f), 0.02f)];
        var generator = new ManifoldGenerator3D();
        foreach (var shape in shapes)
        {
            float bottom = shape.Support(-Vector3F.UnitY).Y;
            var pose = new RigidTransformF(QuaternionF.Identity, new Vector3F(0, 0.98f - bottom, 0));
            var manifold = generator.Collide(floor, shape, pose);
            Assert.True(manifold.PointCount > 0, shape.Kind.ToString());
            Near(1, manifold.Normal.Y, 2e-3);
            Near(-0.02, manifold[0].Separation, 2e-3);
            var inverse = new RigidTransformF(QuaternionF.Identity, -pose.Translation);
            manifold = generator.Collide(shape, floor, inverse);
            Assert.True(manifold.PointCount > 0, shape.Kind.ToString());
            Near(-1, manifold.Normal.Y, 2e-3);
        }
    }

    [Fact]
    public void InvalidDimensionsAreRejected()
    {
        foreach (float bad in new[] { float.NaN, float.PositiveInfinity, -1f, 2097153f })
        {
            Assert.Throws<ArgumentOutOfRangeException>(() => ConvexShape3D.TaperedCylinder(bad, 1, 2));
            Assert.Throws<ArgumentOutOfRangeException>(() => ConvexShape3D.TaperedCylinder(1, bad, 2));
            Assert.Throws<ArgumentOutOfRangeException>(() => ConvexShape3D.TaperedCylinder(1, 1, bad));
        }
        Assert.Throws<ArgumentOutOfRangeException>(() => ConvexShape3D.TaperedCylinder(0, 1, 1));
        Assert.Throws<ArgumentOutOfRangeException>(() => ConvexShape3D.TaperedCylinder(1, 0, 0));
    }

    [Fact]
    public void WorldRoundTripForkAndThreadCounts()
    {
        var a = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = 1 });
        var b = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = 8 });
        foreach (var w in new[] { a, b })
        {
            w.AddBox(new Vector3F(20, 1, 20), at: new Vector3D(0, -1, 0), type: BodyType.Static);
            for (int i = 0; i < 5; i++) { w.AddTaperedCylinder(0.5f, 0.2f, 0.4f, at: new Vector3D(i, 2, 0)); }
        }
        for (int i = 0; i < 30; i++) { a.Step(1f / 60); b.Step(1f / 60); Assert.Equal(a.ComputeHash(), b.ComputeHash()); }
        byte[] snapshot = a.Snapshot();
        var fork = a.Fork();
        for (int i = 0; i < 20; i++) { a.Step(1f / 60); fork.Step(1f / 60); Assert.Equal(a.ComputeHash(), fork.ComputeHash()); }
        a.Restore(snapshot);
        Assert.Equal(snapshot, a.Snapshot());
    }
}
