using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Collision3D;

public sealed class TriangleShapeTests
{
    private static readonly Vector3F A = new(-2, 0, -2), B = new(0, 0, 2), C = new(2, 0, -2);

    [Fact]
    public void LaminaMassIsAreaAndInertiaIsAboutItsCentroid()
    {
        var shape = ConvexShape3D.Triangle(new(0,0,0), new(2,0,0), new(0,2,0));
        var m = shape.GetMassProperties();
        Assert.Equal(2, m.Mass);
        Assert.Equal(2f / 3, m.Center.X);
        Assert.Equal(2f / 3, m.Center.Y);
        Assert.Equal(4f / 9, m.Inertia.M11, 6);
        Assert.Equal(8f / 9, m.Inertia.M33, 6);
        Assert.Equal(2f / 9, m.Inertia.M12, 6);
    }

    [Fact]
    public void RayAndContactAreDoubleSided()
    {
        var triangle = ConvexShape3D.Triangle(A, B, C);
        var generator = new ManifoldGenerator3D();
        foreach (float side in new[] { -1f, 1f })
        {
            Assert.True(Gjk3D.RayCast(triangle, RigidTransformF.Identity, new(0, side, 0), new(0, -2 * side, 0), 1, out var hit));
            Assert.InRange(hit.Fraction, 0.49999f, 0.50001f);
            var pose = new RigidTransformF(QuaternionF.Identity, new(0, side * 0.4f, 0));
            var m = generator.Collide(triangle, ConvexShape3D.Sphere(0.5f), pose);
            Assert.True(m.PointCount > 0);
            Assert.InRange(m.Normal.Y * side, 0.999f, 1.001f);
            Assert.InRange(m.MinSeparation, -0.10001f, -0.09999f);
        }
    }

    [Fact]
    public void EveryConvexKindContactsBothFaces()
    {
        var triangle = ConvexShape3D.Triangle(A, B, C);
        ConvexShape3D[] shapes = [ConvexShape3D.Sphere(0.3f), ConvexShape3D.Box(new(0.2f,0.3f,0.2f)),
            ConvexShape3D.Capsule(0.2f,0.2f), ConvexShape3D.TaperedCapsule(0.3f,0.2f,0.3f), ConvexShape3D.Cylinder(0.3f,0.3f),
            ConvexShape3D.TaperedCylinder(0.3f,0.2f,0.3f), ConvexShape3D.Triangle(new(-0.2f,0,-0.2f),new(0,0,0.2f),new(0.2f,0,-0.2f),0.1f)];
        var generator = new ManifoldGenerator3D();
        foreach (var s in shapes)
        {
            foreach (float side in new[] { -1f, 1f })
            {
                float support = side > 0 ? -s.Support(-Vector3F.UnitY).Y : s.Support(Vector3F.UnitY).Y;
                var pose = new RigidTransformF(QuaternionF.Identity, new(0, side * (support - 0.02f), 0));
                var m = generator.Collide(triangle, s, pose);
                Assert.True(m.PointCount > 0, s.Kind.ToString());
                Assert.InRange(m.Normal.Y * side, 0.998f, 1.002f);
                Assert.InRange(m.MinSeparation, -0.022f, -0.018f);
            }
        }
    }

    [Fact]
    public void HostileVerticesFailBeforeUse()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => ConvexShape3D.Triangle(A, A, C));
        Assert.Throws<ArgumentOutOfRangeException>(() => ConvexShape3D.Triangle(default, Vector3F.UnitX, 2 * Vector3F.UnitX));
        Assert.Throws<ArgumentOutOfRangeException>(() => ConvexShape3D.Triangle(A, B, new(float.NaN,0,0)));
        Assert.Throws<ArgumentOutOfRangeException>(() => ConvexShape3D.Triangle(A, B, C, float.NaN));
    }

    [Fact]
    public void SnapshotForkAndDebugRetainBothFaces()
    {
        var w = new PhysicsWorld3D();
        w.AddTriangle(A, B, C);
        w.AddSphere(0.3f, at: new Vector3D(0,2,0));
        var snapshot = w.Snapshot();
        var fork = w.Fork();
        for (int i=0;i<60;i++) { w.Step(1f/60); fork.Step(1f/60); Assert.Equal(w.ComputeHash(), fork.ComputeHash()); }
        w.Restore(snapshot);
        Assert.Equal(snapshot, w.Snapshot());
        var debug = new RecursiveAI.OnePhysics.Debug.DebugGeometry();
        w.GetDebugGeometry(RecursiveAI.OnePhysics.Debug.DebugContent.All, debug);
        Assert.NotEmpty(debug.Shapes.ToArray());
    }
}
