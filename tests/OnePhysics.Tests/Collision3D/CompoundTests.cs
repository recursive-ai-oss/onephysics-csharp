using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OneMath.Physics;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Collision3D;

/// <summary>Known answers, hostile input and the API contracts of decorators, placed shapes and compounds.</summary>
public class CompoundTests
{
    private static RigidTransformF At(float x, float y, float z) => new(QuaternionF.Identity, new Vector3F(x, y, z));

    private static QuaternionF AxisAngle(Vector3F axis, double angle)
    {
        axis = axis.Normalized();
        return QuaternionF.CreateFromAxisSinCos(axis, (float)Math.Sin(angle / 2), (float)Math.Cos(angle / 2));
    }

    private static void Near(double expected, double actual, double tol, string what = "") =>
        Assert.True(Math.Abs(expected - actual) <= tol, $"{what} expected {expected}, got {actual} (tolerance {tol})");

    private static void NearRel(double expected, double actual, double rel, string what = "") =>
        Assert.True(Math.Abs(expected - actual) <= rel * Math.Max(Math.Abs(expected), 1e-12), $"{what} expected {expected}, got {actual} (relative tolerance {rel})");

    private static void NearV(Vector3F expected, Vector3F actual, double tol, string what = "")
    {
        Near(expected.X, actual.X, tol, what + ".X");
        Near(expected.Y, actual.Y, tol, what + ".Y");
        Near(expected.Z, actual.Z, tol, what + ".Z");
    }

    private static void SameMass(MassPropertiesF expected, MassPropertiesF actual, double rel)
    {
        NearRel(expected.Mass, actual.Mass, rel, "mass");
        double scale = Math.Max(Math.Abs(expected.Inertia.M11), Math.Max(Math.Abs(expected.Inertia.M22), Math.Abs(expected.Inertia.M33)));
        Near(expected.Center.X, actual.Center.X, rel * 10, "center.X");
        Near(expected.Center.Y, actual.Center.Y, rel * 10, "center.Y");
        Near(expected.Center.Z, actual.Center.Z, rel * 10, "center.Z");
        Near(expected.Inertia.M11, actual.Inertia.M11, rel * scale, "I11");
        Near(expected.Inertia.M22, actual.Inertia.M22, rel * scale, "I22");
        Near(expected.Inertia.M33, actual.Inertia.M33, rel * scale, "I33");
        Near(expected.Inertia.M12, actual.Inertia.M12, rel * scale, "I12");
        Near(expected.Inertia.M13, actual.Inertia.M13, rel * scale, "I13");
        Near(expected.Inertia.M23, actual.Inertia.M23, rel * scale, "I23");
    }

    private static readonly int[] Zero = { 0 };
    private static readonly int[] ZeroOne = { 0, 1 };

    private static T Throws<T>(Action a) where T : Exception => Assert.Throws<T>(a);

    private static ShapeCompositionError ErrorOf(Action a) => Assert.Throws<ShapeCompositionException>(a).Error;

    // ---- mass ------------------------------------------------------------------------------------------------------------------------------------

    [Fact]
    public void CompoundOfBoxesHasTheMassPropertiesOfTheEquivalentBox()
    {
        var kids = new List<CompoundChild3D>();
        for (int x = -1; x <= 1; x += 2)
        {
            for (int y = -1; y <= 1; y += 2)
            {
                for (int z = -1; z <= 1; z += 2)
                {
                    kids.Add(new CompoundChild3D(ConvexShape3D.Box(new Vector3F(0.5f, 0.5f, 0.5f)), At(x * 0.5f, y * 0.5f, z * 0.5f)));
                }
            }
        }

        MassPropertiesF whole = Compound3D.Create(kids.ToArray()).GetMassProperties();
        SameMass(ConvexShape3D.Box(new Vector3F(1, 1, 1)).GetMassProperties(), whole, 1e-5);
    }

    [Fact]
    public void RotatedAndScaledChildrenMakeTheSameBox()
    {
        // A 1 x 2 x 3 box (half extents 0.5, 1, 1.5) from two halves: one as is, one built as a 2-wide box scaled and turned a quarter about Y.
        ConvexShape3D unit = ConvexShape3D.Box(new Vector3F(0.5f, 0.5f, 0.5f));
        ConvexShape3D left = ScaledShape3D.Create(unit, new Vector3F(1f, 2f, 1.5f)); // half 0.5, 1, 0.75
        ConvexShape3D right = ScaledShape3D.Create(unit, new Vector3F(1.5f, 2f, 1f)); // half 0.75, 1, 0.5; a quarter turn about Y makes it 0.5, 1, 0.75
        var q = AxisAngle(Vector3F.UnitY, Math.PI / 2);
        var c = Compound3D.Create(new[]
        {
            new CompoundChild3D(left, At(0, 0, -0.75f)),
            new CompoundChild3D(right, new RigidTransformF(q, new Vector3F(0, 0, 0.75f))),
        });
        SameMass(ConvexShape3D.Box(new Vector3F(0.5f, 1f, 1.5f)).GetMassProperties(), c.GetMassProperties(), 1e-5);
    }

    [Fact]
    public void OffsetSphereFollowsTheParallelAxisTheorem()
    {
        ConvexShape3D s = ConvexShape3D.Sphere(0.5f);
        MassPropertiesF one = s.GetMassProperties();
        PlacedShape3D placed = PlacedShape3D.Create(s, At(3, 0, 0));
        MassPropertiesF m = placed.GetMassProperties();
        NearV(new Vector3F(3, 0, 0), m.Center, 1e-6);
        NearRel(one.Inertia.M11, m.Inertia.M11, 1e-6);

        var pair = Compound3D.Create(new[] { new CompoundChild3D(s, At(3, 0, 0)), new CompoundChild3D(s, At(-3, 0, 0)) });
        MassPropertiesF both = pair.GetMassProperties();
        NearRel(2 * one.Mass, both.Mass, 1e-6);
        NearV(default, both.Center, 1e-6);
        NearRel(2 * one.Inertia.M11, both.Inertia.M11, 1e-5, "Ixx");
        NearRel(2 * (one.Inertia.M22 + (one.Mass * 9f)), both.Inertia.M22, 1e-5, "Iyy");
        NearRel(2 * (one.Inertia.M33 + (one.Mass * 9f)), both.Inertia.M33, 1e-5, "Izz");
    }

    [Fact]
    public void PlacedShapeRotatesItsInertiaTensor()
    {
        ConvexShape3D box = ConvexShape3D.Box(new Vector3F(1, 2, 3));
        MassPropertiesF a = box.GetMassProperties();
        MassPropertiesF b = PlacedShape3D.Create(box, new RigidTransformF(AxisAngle(Vector3F.UnitZ, Math.PI / 2), default)).GetMassProperties();
        NearRel(a.Inertia.M22, b.Inertia.M11, 1e-5);
        NearRel(a.Inertia.M11, b.Inertia.M22, 1e-5);
        NearRel(a.Inertia.M33, b.Inertia.M33, 1e-5);
    }

    [Fact]
    public void HullMassPropertiesCombine()
    {
        var rng = new TestRng(5);
        ConvexHull3D[] hulls = Collision3DHash.RandomHulls(rng, 6);
        var kids = new List<CompoundChild3D>();
        foreach (ConvexHull3D h in hulls)
        {
            kids.Add(new CompoundChild3D(ConvexShape3D.FromHull(h), new RigidTransformF(rng.Rotation(), rng.InBall(3f))));
        }

        MassPropertiesF m = Compound3D.Create(kids.ToArray()).GetMassProperties();
        double sum = 0;
        foreach (ConvexHull3D h in hulls)
        {
            sum += h.MassProperties.Mass;
        }

        NearRel(sum, m.Mass, 1e-5);
    }

    [Fact]
    public void ScaledHullVolumeScalesByTheProductOfTheScales()
    {
        ConvexHull3D hull = Collision3DHash.RandomHulls(new TestRng(9), 2)[1];
        ConvexShape3D scaled = ScaledShape3D.Create(ConvexShape3D.FromHull(hull), new Vector3F(2f, 0.5f, 3f));
        NearRel(hull.MassProperties.Mass * 3f, scaled.GetMassProperties().Mass, 1e-4);
    }

    // ---- scaling rules ---------------------------------------------------------------------------------------------------------------------------

    [Fact]
    public void ScaleSupportPerShapeKind()
    {
        Assert.Equal(ScaleSupport3D.Uniform, ScaledShape3D.GetScaleSupport(ConvexShape3D.Sphere(1)));
        Assert.Equal(ScaleSupport3D.Uniform, ScaledShape3D.GetScaleSupport(ConvexShape3D.Capsule(1, 0.5f)));
        Assert.Equal(ScaleSupport3D.Uniform, ScaledShape3D.GetScaleSupport(ConvexShape3D.TaperedCapsule(1, 0.5f, 0.3f)));
        Assert.Equal(ScaleSupport3D.PerAxis, ScaledShape3D.GetScaleSupport(ConvexShape3D.Box(new Vector3F(1, 2, 3))));
        Assert.Equal(ScaleSupport3D.Uniform, ScaledShape3D.GetScaleSupport(ConvexShape3D.Box(new Vector3F(1, 2, 3), 0.1f)));
        Assert.Equal(ScaleSupport3D.AxisSymmetricY, ScaledShape3D.GetScaleSupport(ConvexShape3D.Cylinder(1, 0.5f)));
        Assert.Equal(ScaleSupport3D.Uniform, ScaledShape3D.GetScaleSupport(ConvexShape3D.Cylinder(1, 0.5f, 0.1f)));
        ConvexHull3D hull = Collision3DHash.RandomHulls(new TestRng(1), 1)[0];
        Assert.Equal(ScaleSupport3D.PerAxis, ScaledShape3D.GetScaleSupport(ConvexShape3D.FromHull(hull)));
        Assert.Equal(ScaleSupport3D.Uniform, ScaledShape3D.GetScaleSupport(ConvexShape3D.FromHull(hull, 0.1f)));
    }

    [Fact]
    public void UniformScaleMatchesTheDirectlyBuiltShape()
    {
        NearRel(0.5 * 2, ScaledShape3D.Create(ConvexShape3D.Sphere(0.5f), 2f).ConvexRadius, 1e-6);
        ConvexShape3D cap = ScaledShape3D.Create(ConvexShape3D.Capsule(1f, 0.25f), 3f);
        NearRel(0.75, cap.ConvexRadius, 1e-6);
        NearRel(3.0, cap.Support(Vector3F.UnitY).Y - cap.ConvexRadius, 1e-6);
        ConvexShape3D cyl = ScaledShape3D.Create(ConvexShape3D.Cylinder(1f, 0.5f), new Vector3F(2f, 3f, 2f));
        NearRel(1.0, cyl.Support(Vector3F.UnitX).X, 1e-6);
        NearRel(3.0, cyl.Support(Vector3F.UnitY).Y, 1e-6);
        ConvexShape3D rounded = ScaledShape3D.Create(ConvexShape3D.Box(new Vector3F(1, 2, 3), 0.25f), 2f);
        NearRel(0.5, rounded.ConvexRadius, 1e-6);
        NearV(new Vector3F(2, 4, 6), rounded.HalfExtents, 1e-5);
    }

    [Fact]
    public void ScaledBoxHasTheManifoldOfTheEquivalentBox()
    {
        var rng = new TestRng(11);
        var g = new ManifoldGenerator3D();
        ConvexShape3D other = ConvexShape3D.Box(new Vector3F(0.6f, 0.4f, 0.8f));
        int contacts = 0;
        for (int i = 0; i < 2000; i++)
        {
            var s = new Vector3F(rng.Range(0.5f, 2f), rng.Range(0.5f, 2f), rng.Range(0.5f, 2f));
            ConvexShape3D scaled = ScaledShape3D.Create(ConvexShape3D.Box(new Vector3F(0.5f, 0.5f, 0.5f)), s);
            ConvexShape3D direct = ConvexShape3D.Box(new Vector3F(0.5f, 0.5f, 0.5f) * s);
            var xf = new RigidTransformF(rng.Rotation(), rng.InBall(1.6f));
            Manifold3D a = g.Collide(scaled, other, xf), b = g.Collide(direct, other, xf);
            Assert.Equal(b.PointCount, a.PointCount);
            Assert.Equal(b.Normal, a.Normal);
            for (int k = 0; k < a.PointCount; k++)
            {
                Assert.Equal(b[k].Point, a[k].Point);
                Assert.Equal(b[k].Separation, a[k].Separation);
                Assert.Equal(b[k].Id, a[k].Id);
            }

            contacts += a.PointCount > 0 ? 1 : 0;
        }

        Assert.True(contacts > 500);
    }

    [Fact]
    public void UnsupportedAndInvalidScalesAreRejected()
    {
        ConvexShape3D box = ConvexShape3D.Box(new Vector3F(1, 1, 1));
        Assert.Equal(ShapeCompositionError.InvalidScale, ErrorOf(() => ScaledShape3D.Create(box, float.NaN)));
        Assert.Equal(ShapeCompositionError.InvalidScale, ErrorOf(() => ScaledShape3D.Create(box, 0f)));
        Assert.Equal(ShapeCompositionError.InvalidScale, ErrorOf(() => ScaledShape3D.Create(box, -1f)));
        Assert.Equal(ShapeCompositionError.InvalidScale, ErrorOf(() => ScaledShape3D.Create(box, float.PositiveInfinity)));
        Assert.Equal(ShapeCompositionError.InvalidScale, ErrorOf(() => ScaledShape3D.Create(box, new Vector3F(1, float.NaN, 1))));
        Assert.Equal(ShapeCompositionError.UnsupportedScale, ErrorOf(() => ScaledShape3D.Create(ConvexShape3D.Sphere(1), new Vector3F(1, 2, 1))));
        Assert.Equal(ShapeCompositionError.UnsupportedScale, ErrorOf(() => ScaledShape3D.Create(ConvexShape3D.Capsule(1, 0.5f), new Vector3F(1, 2, 1))));
        Assert.Equal(ShapeCompositionError.UnsupportedScale, ErrorOf(() => ScaledShape3D.Create(ConvexShape3D.TaperedCapsule(1, 0.5f, 0.3f), new Vector3F(2, 1, 1))));
        Assert.Equal(ShapeCompositionError.UnsupportedScale, ErrorOf(() => ScaledShape3D.Create(ConvexShape3D.Box(new Vector3F(1, 1, 1), 0.2f), new Vector3F(2, 1, 1))));
        Assert.Equal(ShapeCompositionError.UnsupportedScale, ErrorOf(() => ScaledShape3D.Create(ConvexShape3D.Cylinder(1, 1), new Vector3F(2, 1, 1))));
        Assert.Equal(ShapeCompositionError.UnsupportedScale, ErrorOf(() => ScaledShape3D.Create(ConvexShape3D.Cylinder(1, 1, 0.1f), new Vector3F(1, 2, 1))));
        ConvexHull3D hull = Collision3DHash.RandomHulls(new TestRng(2), 1)[0];
        Assert.Equal(ShapeCompositionError.UnsupportedScale, ErrorOf(() => ScaledShape3D.Create(ConvexShape3D.FromHull(hull, 0.1f), new Vector3F(1, 2, 1))));
        Assert.Equal(ShapeCompositionError.InvalidShape, ErrorOf(() => ScaledShape3D.Create(ConvexShape3D.Point, 2f)));
        Assert.Equal(ShapeCompositionError.ShapeBuildFailed, ErrorOf(() => ScaledShape3D.Create(box, 1e30f)));
        Assert.Equal(ShapeCompositionError.ShapeBuildFailed, ErrorOf(() => ScaledShape3D.Create(ConvexShape3D.Box(new Vector3F(0.5f, 0.5f, 0.5f)), 1e-45f)));
        Assert.Equal(ShapeCompositionError.ShapeBuildFailed, ErrorOf(() => ScaledShape3D.Create(ConvexShape3D.FromHull(hull), new Vector3F(1e30f, 1, 1))));

        // Accepted ones keep their kind.
        Assert.Equal(ConvexShapeKind3D.Box, ScaledShape3D.Create(box, new Vector3F(1, 2, 3)).Kind);
        Assert.Equal(ConvexShapeKind3D.Cylinder, ScaledShape3D.Create(ConvexShape3D.Cylinder(1, 1), new Vector3F(2, 3, 2)).Kind);
    }

    // ---- placed shapes ---------------------------------------------------------------------------------------------------------------------------

    [Fact]
    public void PlacedShapesMapQueriesIntoTheHolderFrame()
    {
        var spheres = PlacedShape3D.Create(ConvexShape3D.Sphere(1f), At(1, 0, 0));
        var other = PlacedShape3D.Create(ConvexShape3D.Sphere(0.5f));
        ConvexDistance3D d = spheres.Distance(other, At(4, 0, 0));
        Near(1.5, d.Distance, 1e-5);
        NearV(Vector3F.UnitX, d.Normal, 1e-5);
        NearV(new Vector3F(2, 0, 0), d.PointA, 1e-5);
        NearV(new Vector3F(3.5f, 0, 0), d.PointB, 1e-5);

        ConvexContact3D c = spheres.Collide(new Epa3D(), other, At(2.2f, 0, 0));
        Near(-0.3, c.Separation, 1e-5);
        NearV(Vector3F.UnitX, c.Normal, 1e-5);

        Assert.True(spheres.ShapeCast(other, At(6, 0, 0), new Vector3F(10, 0, 0), out ConvexCastHit3D hit));
        Near(0.35, hit.Fraction, 1e-4); // sphere surfaces meet when the centres are 1.5 apart: 1 + 10t + 1.5 = 6
        NearV(Vector3F.UnitX, hit.Normal, 1e-4);

        Assert.True(spheres.RayCast(new Vector3F(-5, 0, 0), Vector3F.UnitX, 100f, out ConvexCastHit3D ray));
        Near(5.0, ray.Fraction, 1e-4);
        NearV(-Vector3F.UnitX, ray.Normal, 1e-4);

        Vector3F far = spheres.Support(Vector3F.UnitX);
        NearV(new Vector3F(2, 0, 0), far, 1e-5);
        spheres.GetBounds(out Vector3F lo, out Vector3F hi);
        NearV(new Vector3F(0, -1, -1), lo, 1e-3);
        NearV(new Vector3F(2, 1, 1), hi, 1e-3);
    }

    [Fact]
    public void PlacedBoxManifoldIsInTheHolderFrame()
    {
        var g = new ManifoldGenerator3D();
        // A is a slab turned a quarter about Z: its top face (y = 1 in its own frame) now faces -X at x = -1 in the holder frame. B rests against it.
        var a = PlacedShape3D.Create(ConvexShape3D.Box(new Vector3F(5, 1, 5)), new RigidTransformF(AxisAngle(Vector3F.UnitZ, Math.PI / 2), default));
        var b = PlacedShape3D.Create(ConvexShape3D.Box(new Vector3F(0.5f, 0.5f, 0.5f)), At(-0.5f, 0, 0));
        Manifold3D m = g.Collide(a, b, At(-1.0f, 0, 0));
        Assert.Equal(4, m.PointCount);
        NearV(-Vector3F.UnitX, m.Normal, 1e-5);
        for (int i = 0; i < 4; i++)
        {
            Near(0, m[i].Separation, 1e-5);
            Near(-1.0, m[i].Point.X, 1e-5);
            Assert.InRange(m[i].Point.Y, -0.51f, 0.51f);
        }

        // Moving B's holder down by 0.01 keeps the contact; the ids stay.
        Manifold3D n = g.Collide(a, b, At(-1.0f, 0.3f, 0));
        Assert.Equal(4, n.PointCount);
        for (int i = 0; i < 4; i++)
        {
            Assert.Equal(m[i].Id, n[i].Id);
        }
    }

    [Fact]
    public void BoundsOfRotatedShapesAreTightAndPadded()
    {
        ConvexShape3D box = ConvexShape3D.Box(new Vector3F(1, 1, 1));
        ShapeBounds3D.Compute(box, new RigidTransformF(AxisAngle(Vector3F.UnitZ, Math.PI / 4), new Vector3F(10, 0, 0)), out Vector3F lo, out Vector3F hi);
        Near(10 - Math.Sqrt(2), lo.X, 1e-3);
        Near(10 + Math.Sqrt(2), hi.X, 1e-3);
        Assert.True(lo.X < 10 - Math.Sqrt(2) && hi.X > 10 + Math.Sqrt(2), "padding grows the box outward");
        Near(-1, lo.Z, 1e-3);
    }

    [Fact]
    public void DefaultPlacedShapeFailsClearly()
    {
        PlacedShape3D p = default;
        Throws<InvalidOperationException>(() => p.GetMassProperties());
        Throws<InvalidOperationException>(() => p.GetBounds(out _, out _));
    }

    // ---- compound queries with analytic answers --------------------------------------------------------------------------------------------------

    private static Compound3D TwoBoxes() => Compound3D.Create(new[]
    {
        new CompoundChild3D(ScaledShape3D.Create(ConvexShape3D.Box(new Vector3F(0.5f, 0.5f, 0.5f)), 2f), At(5, 0, 0), 7), // half 1 at x = 5
        new CompoundChild3D(ConvexShape3D.Box(new Vector3F(1, 1, 1)), At(10, 0, 0), 9),
    });

    [Fact]
    public void RaysHitDecoratedChildrenAtAnalyticPoints()
    {
        Compound3D c = TwoBoxes();
        Assert.True(c.RayCast(new Vector3F(0, 0.3f, 0.2f), Vector3F.UnitX, 100f, out CompoundRayHit3D hit));
        Assert.Equal(0, hit.Child);
        Assert.Equal(7, hit.MaterialId);
        Near(4.0, hit.Fraction, 1e-4);
        NearV(new Vector3F(4, 0.3f, 0.2f), hit.Point, 1e-4);
        NearV(-Vector3F.UnitX, hit.Normal, 1e-4);

        Assert.True(c.RayCast(new Vector3F(20, 0, 0), -Vector3F.UnitX, 100f, out hit));
        Assert.Equal(1, hit.Child);
        Assert.Equal(9, hit.MaterialId);
        Near(9.0, hit.Fraction, 1e-4);
        NearV(Vector3F.UnitX, hit.Normal, 1e-4);

        Assert.False(c.RayCast(new Vector3F(0, 3, 0), Vector3F.UnitX, 100f, out _));
        Assert.False(c.RayCast(new Vector3F(0, 0, 0), Vector3F.UnitX, 3.5f, out _));
        Assert.True(c.RayCastAny(new Vector3F(0, 0, 0), Vector3F.UnitX, 100f, out hit));
        Assert.InRange(hit.Child, 0, 1);

        Assert.True(c.RayCast(new Vector3F(5, 0, 0), Vector3F.UnitY, 10f, out hit));
        Assert.True(hit.StartsInside);
        Assert.Equal(0f, hit.Fraction);
    }

    [Fact]
    public void RaysHitRotatedChildrenAtAnalyticPoints()
    {
        var rng = new TestRng(21);
        int hits = 0;
        for (int i = 0; i < 500; i++)
        {
            var he = new Vector3F(rng.Range(0.3f, 1.2f), rng.Range(0.3f, 1.2f), rng.Range(0.3f, 1.2f));
            var local = new RigidTransformF(rng.Rotation(), rng.InBall(2f));
            var c = Compound3D.Create(new[] { new CompoundChild3D(ConvexShape3D.Box(he), local) });
            Vector3F o = rng.InBall(6f), target = local.Translation + rng.InBall(1f);
            Vector3F d = target - o;
            bool got = c.RayCast(o, d, 2f, out CompoundRayHit3D h);

            // Slab test in the box frame, in double, for the box itself and for the box grown by a margin.
            Vector3F lo = local.Inverted().TransformPoint(o), ld = local.Inverted().TransformVector(d);
            double[] oo = { lo.X, lo.Y, lo.Z }, dd = { ld.X, ld.Y, ld.Z }, hh = { he.X, he.Y, he.Z };
            (bool hit, double t0, int axis) Slab(double grow)
            {
                double near = 0, far = 2;
                int ax = -1;
                for (int k = 0; k < 3; k++)
                {
                    double h = hh[k] + grow;
                    if (Math.Abs(dd[k]) < 1e-12)
                    {
                        if (Math.Abs(oo[k]) > h)
                        {
                            return (false, 0, -1);
                        }

                        continue;
                    }

                    double a = (-h - oo[k]) / dd[k], b = (h - oo[k]) / dd[k];
                    if (a > b)
                    {
                        (a, b) = (b, a);
                    }

                    if (a > near)
                    {
                        near = a;
                        ax = k;
                    }

                    far = Math.Min(far, b);
                    if (near > far)
                    {
                        return (false, 0, -1);
                    }
                }

                return (true, near, ax);
            }

            (bool hit, double t0, int axis) exact = Slab(0);
            if (exact.hit && exact.t0 > 0.002 && exact.axis >= 0)
            {
                Assert.True(got, $"ray {i} should hit at {exact.t0}");
                Near(exact.t0, h.Fraction, 3e-4, $"ray {i} fraction");
                hits++;
                double[] n = { 0, 0, 0 };
                n[exact.axis] = dd[exact.axis] > 0 ? -1 : 1;
                NearV(local.TransformVector(new Vector3F((float)n[0], (float)n[1], (float)n[2])), h.Normal, 1e-3, $"ray {i} normal");
            }
            else if (!Slab(0.01).hit)
            {
                Assert.False(got, $"ray {i} clearly misses");
            }
        }

        Assert.True(hits > 100, "enough rays hit: " + hits);
    }

    [Fact]
    public void ShapeCastFindsTheFirstChild()
    {
        Compound3D c = TwoBoxes();
        Assert.True(c.ShapeCast(ConvexShape3D.Sphere(0.5f), At(0, 0, 0), new Vector3F(20, 0, 0), out CompoundCastHit3D hit));
        Assert.Equal(0, hit.Child);
        Near((4 - 0.5) / 20.0, hit.Fraction, 1e-4); // the sphere's surface reaches x = 4 when its centre is at 3.5
        NearV(Vector3F.UnitX, hit.Normal, 1e-4);
        NearV(new Vector3F(4, 0, 0), hit.PointOnChild, 1e-4);

        Assert.False(c.ShapeCast(ConvexShape3D.Sphere(0.5f), At(0, 3, 0), new Vector3F(20, 0, 0), out _));
        Assert.True(c.ShapeCast(ConvexShape3D.Sphere(0.5f), At(5, 0.5f, 0), new Vector3F(0, 1, 0), out hit));
        Assert.True(hit.StartsOverlapping);
        Assert.Equal(0f, hit.Fraction);
    }

    [Fact]
    public void OverlapReportsChildrenByBounds()
    {
        Compound3D c = TwoBoxes();
        var ids = new List<int>();
        c.OverlapBounds(new Vector3F(3, -1, -1), new Vector3F(4.5f, 1, 1), ids);
        Assert.Equal(Zero, ids);
        c.OverlapBounds(new Vector3F(0, -1, -1), new Vector3F(20, 1, 1), ids);
        Assert.Equal(ZeroOne, ids);
        c.OverlapBounds(new Vector3F(0, 5, 0), new Vector3F(20, 6, 1), ids);
        Assert.Empty(ids);
        Assert.True(c.TryGetBounds(out Vector3F lo, out Vector3F hi));
        Near(4, lo.X, 1e-3);
        Near(11, hi.X, 1e-3);
    }

    // ---- pairs and manifolds ---------------------------------------------------------------------------------------------------------------------

    [Fact]
    public void ChildContactsCarrySubShapeIdsAndStableFeatureIds()
    {
        var g = new ManifoldGenerator3D();
        // A floor of three slabs; a box resting on the second one.
        var floor = Compound3D.Create(new[]
        {
            new CompoundChild3D(ConvexShape3D.Box(new Vector3F(2, 1, 2)), At(-4, 0, 0)),
            new CompoundChild3D(ConvexShape3D.Box(new Vector3F(2, 1, 2)), At(0, 0, 0)),
            new CompoundChild3D(ConvexShape3D.Box(new Vector3F(2, 1, 2)), At(4, 0, 0)),
        });
        ConvexShape3D box = ConvexShape3D.Box(new Vector3F(0.5f, 0.5f, 0.5f));
        var results = new List<ChildContact3D>();
        Assert.Equal(1, g.CollideCompoundConvex(floor, box, At(0.2f, 1.49f, 0), results));
        ChildContact3D first = results[0];
        Assert.Equal(1, first.SubShapeA);
        Assert.Equal(0, first.SubShapeB);
        Assert.Equal(4, first.Manifold.PointCount);
        NearV(Vector3F.UnitY, first.Manifold.Normal, 1e-5);

        // Straddling two slabs gives two contacts, in ascending child order, and their keys differ.
        Assert.Equal(2, g.CollideCompoundConvex(floor, box, At(2f, 1.49f, 0), results));
        Assert.Equal(1, results[0].SubShapeA);
        Assert.Equal(2, results[1].SubShapeA);
        var keys = new HashSet<ulong>();
        foreach (ChildContact3D cc in results)
        {
            for (int i = 0; i < cc.Manifold.PointCount; i++)
            {
                Assert.True(keys.Add(cc.GetPointKey(i)));
            }
        }

        // The same child pair a frame later: same ids, point for point.
        g.CollideCompoundConvex(floor, box, At(0.2f, 1.49f, 0), results);
        Manifold3D before = results[0].Manifold;
        g.CollideCompoundConvex(floor, box, At(0.25f, 1.485f, 0.01f), results);
        _ = Assert.Single(results);
        for (int i = 0; i < 4; i++)
        {
            Assert.Equal(before[i].Id, results[0].Manifold[i].Id);
        }

        // Convex first, compound second: the child id is on the B side and the manifold is in the convex shape's frame.
        Assert.Equal(1, g.CollideConvexCompound(box, floor, At(-0.2f, -1.49f, 0), results));
        Assert.Equal(0, results[0].SubShapeA);
        Assert.Equal(1, results[0].SubShapeB);
        NearV(-Vector3F.UnitY, results[0].Manifold.Normal, 1e-5);

        // Two compounds.
        var other = Compound3D.Create(new[]
        {
            new CompoundChild3D(ConvexShape3D.Sphere(0.4f), At(-0.5f, 0, 0)),
            new CompoundChild3D(ConvexShape3D.Sphere(0.4f), At(0.5f, 0, 0)),
        });
        Assert.Equal(2, g.CollideCompoundCompound(floor, other, At(0, 1.39f, 0), results));
        Assert.Equal(new[] { (1, 0), (1, 1) }, new[] { (results[0].SubShapeA, results[0].SubShapeB), (results[1].SubShapeA, results[1].SubShapeB) });
    }

    [Fact]
    public void ChildContactPointKeyPacksTheIds()
    {
        var g = new ManifoldGenerator3D();
        var floor = Compound3D.Create(new[] { new CompoundChild3D(ConvexShape3D.Box(new Vector3F(2, 1, 2))), new CompoundChild3D(ConvexShape3D.Box(new Vector3F(2, 1, 2)), At(4, 0, 0)) });
        var results = new List<ChildContact3D>();
        g.CollideConvexCompound(ConvexShape3D.Box(new Vector3F(0.5f, 0.5f, 0.5f)), floor, At(-4, -1.49f, 0), results);
        ChildContact3D c = Assert.Single(results);
        ulong key = c.GetPointKey(0);
        Assert.Equal((uint)1, (uint)((key >> 32) & 0xFFFF));
        Assert.Equal(c.Manifold[0].Id, (uint)(key & 0xFFFFFFFF));
        Assert.Equal(0UL, key >> 48);
    }

    // ---- hostile input ---------------------------------------------------------------------------------------------------------------------------

    [Fact]
    public void BadChildrenAreRejectedWithTypedErrors()
    {
        ConvexShape3D box = ConvexShape3D.Box(new Vector3F(1, 1, 1));
        Assert.Equal(ShapeCompositionError.EmptyCompound, ErrorOf(() => Compound3D.Create(ReadOnlySpan<CompoundChild3D>.Empty)));
        Assert.Equal(ShapeCompositionError.InvalidTransform, ErrorOf(() => _ = new CompoundChild3D(box, At(float.NaN, 0, 0))));
        Assert.Equal(ShapeCompositionError.InvalidTransform, ErrorOf(() => _ = new CompoundChild3D(box, At(0, float.PositiveInfinity, 0))));
        Assert.Equal(ShapeCompositionError.InvalidTransform, ErrorOf(() => _ = new CompoundChild3D(box, At(2e6f, 0, 0))));
        Assert.Equal(ShapeCompositionError.InvalidTransform, ErrorOf(() => _ = new CompoundChild3D(box, new RigidTransformF(new QuaternionF(0, 0, 0, 0), default))));
        Assert.Equal(ShapeCompositionError.InvalidTransform, ErrorOf(() => _ = new CompoundChild3D(box, new RigidTransformF(new QuaternionF(2, 0, 0, 0), default))));
        Assert.Equal(ShapeCompositionError.InvalidTransform, ErrorOf(() => _ = new CompoundChild3D(box, new RigidTransformF(new QuaternionF(float.NaN, 0, 0, 1), default))));
        Assert.Equal(ShapeCompositionError.InvalidShape, ErrorOf(() => _ = new CompoundChild3D(ConvexShape3D.Point)));

        // A default child (no shape) is caught when it is added; the message names the index.
        var withDefault = new[] { new CompoundChild3D(box), default };
        var e = Assert.Throws<ShapeCompositionException>(() => Compound3D.Create(withDefault));
        Assert.Equal(ShapeCompositionError.InvalidShape, e.Error);
        Assert.Contains("children[1]", e.Message, StringComparison.Ordinal);

        // Bounds beyond the tree's range.
        var big = ConvexShape3D.Box(new Vector3F(2097152f, 2097152f, 2097152f));
        var rng = new TestRng(3);
        QuaternionF q = default;
        bool found = false;
        for (int i = 0; i < 1000 && !found; i++)
        {
            q = rng.Rotation();
            ShapeBounds3D.Compute(big, new RigidTransformF(q, new Vector3F(1048576f, 0, 0)), out Vector3F lo, out Vector3F hi);
            found = hi.X > 4194304f;
        }

        Assert.True(found);
        Assert.Equal(ShapeCompositionError.BoundsTooLarge, ErrorOf(() => Compound3D.Create(new[] { new CompoundChild3D(big, new RigidTransformF(q, new Vector3F(1048576f, 0, 0))) })));
    }

    [Fact]
    public void TooManyChildrenAreRejected()
    {
        var kid = new CompoundChild3D(ConvexShape3D.Sphere(0.1f));
        var tooMany = new CompoundChild3D[CompoundShape3D.MaxChildren + 1];
        Array.Fill(tooMany, kid);
        Assert.Equal(ShapeCompositionError.TooManyChildren, ErrorOf(() => Compound3D.Create(tooMany)));
        Assert.Equal(ShapeCompositionError.TooManyChildren, ErrorOf(() => _ = new MutableCompound3D(tooMany)));

        var m = new MutableCompound3D();
        for (int i = 0; i < CompoundShape3D.MaxChildren; i++)
        {
            m.Add(new CompoundChild3D(ConvexShape3D.Sphere(0.1f), At(i % 256, (i / 256) % 256, i / 65536)));
        }

        Assert.Equal(CompoundShape3D.MaxChildren, m.Count);
        Assert.Equal(ShapeCompositionError.TooManyChildren, ErrorOf(() => m.Add(kid)));
        Assert.Equal(CompoundShape3D.MaxChildren, m.Count);
        var h = m.GetHandle(5);
        m.Remove(h);
        Assert.Equal(5, m.Add(kid).Index); // a freed slot can be used again when the compound is full
    }

    [Fact]
    public void BadQueriesAreRejected()
    {
        Compound3D c = TwoBoxes();
        var ids = new List<int>();
        Assert.Equal(ShapeCompositionError.InvalidQuery, ErrorOf(() => c.RayCast(new Vector3F(float.NaN, 0, 0), Vector3F.UnitX, 1f, out _)));
        Assert.Equal(ShapeCompositionError.InvalidQuery, ErrorOf(() => c.RayCast(default, new Vector3F(0, float.PositiveInfinity, 0), 1f, out _)));
        Assert.Equal(ShapeCompositionError.InvalidQuery, ErrorOf(() => c.RayCast(default, Vector3F.UnitX, 0f, out _)));
        Assert.Equal(ShapeCompositionError.InvalidQuery, ErrorOf(() => c.RayCast(default, Vector3F.UnitX, float.NaN, out _)));
        Assert.Equal(ShapeCompositionError.InvalidQuery, ErrorOf(() => c.OverlapBounds(new Vector3F(1, 0, 0), new Vector3F(0, 1, 1), ids)));
        Assert.Equal(ShapeCompositionError.InvalidQuery, ErrorOf(() => c.OverlapBounds(new Vector3F(float.NaN, 0, 0), new Vector3F(0, 1, 1), ids)));
        Assert.Equal(ShapeCompositionError.InvalidQuery, ErrorOf(() => c.ShapeCast(ConvexShape3D.Sphere(1), At(0, 0, 0), new Vector3F(float.NaN, 0, 0), out _)));
        Assert.Equal(ShapeCompositionError.InvalidTransform, ErrorOf(() => c.ShapeCast(ConvexShape3D.Sphere(1), At(float.NaN, 0, 0), default, out _)));
        Assert.Equal(ShapeCompositionError.InvalidShape, ErrorOf(() => c.ShapeCast(ConvexShape3D.Point, At(0, 0, 0), default, out _)));
        Assert.Equal(ShapeCompositionError.InvalidQuery, ErrorOf(() => CompoundPairs3D.FindChildren(c, ConvexShape3D.Sphere(1), At(0, 0, 0), -1f, ids)));
        Assert.Equal(ShapeCompositionError.InvalidTransform, ErrorOf(() => CompoundPairs3D.FindChildren(c, ConvexShape3D.Sphere(1), At(float.NaN, 0, 0), 0f, ids)));
        var g = new ManifoldGenerator3D();
        Throws<ArgumentOutOfRangeException>(() => g.CollideCompoundConvex(c, ConvexShape3D.Sphere(1), At(float.NaN, 0, 0), new List<ChildContact3D>()));
        Assert.Equal(ShapeCompositionError.UnknownChild, ErrorOf(() => c.GetChild(5)));
        Assert.Equal(ShapeCompositionError.UnknownChild, ErrorOf(() => c.GetChild(-1)));
    }

    [Fact]
    public void EmptyMutableCompoundAnswersQueriesAndRefusesMass()
    {
        var m = new MutableCompound3D();
        Assert.False(m.RayCast(default, Vector3F.UnitX, 10f, out _));
        Assert.False(m.ShapeCast(ConvexShape3D.Sphere(1), At(0, 0, 0), Vector3F.UnitX, out _));
        var ids = new List<int>();
        m.OverlapBounds(new Vector3F(-1, -1, -1), new Vector3F(1, 1, 1), ids);
        Assert.Empty(ids);
        Assert.False(m.TryGetBounds(out _, out _));
        Assert.Equal(ShapeCompositionError.EmptyCompound, ErrorOf(() => m.GetMassProperties()));
        m.Validate();
    }

    // ---- mutable compound lifecycle --------------------------------------------------------------------------------------------------------------

    [Fact]
    public void ChildIdsAreStableAndReusedLowestFirst()
    {
        var m = new MutableCompound3D();
        var hs = new CompoundChildHandle3D[5];
        for (int i = 0; i < 5; i++)
        {
            hs[i] = m.Add(new CompoundChild3D(ConvexShape3D.Sphere(0.5f), At(i * 3, 0, 0), i));
            Assert.Equal(i, hs[i].Index);
            Assert.Equal(1, hs[i].Generation);
        }

        m.Remove(hs[3]);
        m.Remove(hs[1]);
        Assert.False(m.IsLive(1));
        Assert.Equal(3, m.Count);
        Assert.Equal(5, m.SlotCount);
        Assert.Equal(2, m.GetMaterialId(2));
        CompoundChildHandle3D a = m.Add(new CompoundChild3D(ConvexShape3D.Sphere(0.5f), At(100, 0, 0), 11));
        CompoundChildHandle3D b = m.Add(new CompoundChild3D(ConvexShape3D.Sphere(0.5f), At(200, 0, 0), 12));
        CompoundChildHandle3D c = m.Add(new CompoundChild3D(ConvexShape3D.Sphere(0.5f), At(300, 0, 0), 13));
        Assert.Equal((1, 2), (a.Index, a.Generation));
        Assert.Equal((3, 2), (b.Index, b.Generation));
        Assert.Equal(5, c.Index);
        m.Validate();

        // Old handles to reused slots are stale; so is a removed one, a default one and a foreign one.
        Assert.Equal(ShapeCompositionError.UnknownChild, ErrorOf(() => m.Remove(hs[1])));
        Assert.Equal(ShapeCompositionError.UnknownChild, ErrorOf(() => m.Move(hs[3], At(0, 0, 0))));
        Assert.Equal(ShapeCompositionError.UnknownChild, ErrorOf(() => m.Remove(default)));
        var other = new MutableCompound3D();
        other.Add(new CompoundChild3D(ConvexShape3D.Sphere(1)));
        Assert.Equal(ShapeCompositionError.UnknownChild, ErrorOf(() => other.Remove(hs[4])));

        // A ray finds the moved child at its new place and reports the changed material.
        m.Move(a, At(0, 10, 0));
        m.SetMaterialId(a, 77);
        Assert.True(m.RayCast(new Vector3F(0, 20, 0), -Vector3F.UnitY, 100f, out CompoundRayHit3D hit));
        Assert.Equal(1, hit.Child);
        Assert.Equal(77, hit.MaterialId);
        Near(9.5, hit.Fraction, 1e-3);
        Assert.Equal(ShapeCompositionError.InvalidTransform, ErrorOf(() => m.Move(a, At(float.NaN, 0, 0))));
        m.Validate();
        Assert.True(m.RayCast(new Vector3F(0, 20, 0), -Vector3F.UnitY, 100f, out hit)); // the failed move changed nothing
        Near(9.5, hit.Fraction, 1e-3);
    }

    [Fact]
    public void SameInputsGiveSameBytes()
    {
        var rng = new TestRng(77);
        ConvexHull3D[] hulls = Collision3DHash.RandomHulls(rng, 4);
        var kids = new List<CompoundChild3D>();
        for (int i = 0; i < 40; i++)
        {
            kids.Add(new CompoundChild3D(Collision3DHash.RandomShape(rng, hulls), new RigidTransformF(rng.Rotation(), rng.InBall(5f)), i));
        }

        byte[] one = Bytes(Compound3D.Create(kids.ToArray()));
        byte[] two = Bytes(Compound3D.Create(kids.ToArray()));
        Assert.Equal(one, two);

        var edit = new MutableCompound3D(kids.ToArray());
        var edit2 = new MutableCompound3D(kids.ToArray());
        edit.Remove(edit.GetHandle(7));
        edit2.Remove(edit2.GetHandle(7));
        edit.Move(edit.GetHandle(3), At(1, 2, 3));
        edit2.Move(edit2.GetHandle(3), At(1, 2, 3));
        Assert.Equal(Bytes(edit), Bytes(edit2));
    }

    private static byte[] Bytes(CompoundShape3D c)
    {
        var w = RecursiveAI.OnePhysics.LowLevel.StateWriter.ToBuffer();
        c.WriteBvh(w);
        return w.ToArray();
    }

    [Fact]
    public void LongRayPassingBesideAFarSphereMissesLikeItsBounds()
    {
        // A ray 1.6 million long passing 0.12 outside a sphere 288 thousand away: the cast's tolerance scales with distance and used to report a hit the tree rightly skipped.
        ConvexShape3D s = ConvexShape3D.Sphere(1.78125f);
        RigidTransformF at = At(-1.59375f, 288184.75f, -2.65625f);
        var origin = new Vector3F(1.1875f, -4.625f, -3.0625f);
        var dir = new Vector3F(-4.9375f, 1619984.2f, 2.875f);
        Assert.False(Gjk3D.RayCast(s, at, origin, dir, 0.6328125f, out _));
        Assert.False(Compound3D.Create(new[] { new CompoundChild3D(s, at) }).RayCast(origin, dir, 0.6328125f, out _));

        // The same ray aimed at the centre still hits, with the entry fraction of the exact answer.
        Vector3F aim = new Vector3F(-1.59375f - 1.1875f, 288184.75f + 4.625f, -2.65625f + 3.0625f) / 0.17792f;
        Assert.True(Gjk3D.RayCast(s, at, origin, aim, 0.6328125f, out ConvexCastHit3D hit));
        Near(0.17792 - 1.78125 / aim.Length(), hit.Fraction, 1e-3);
    }
}
