using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

#pragma warning disable OPHYS0003, OPHYS0002

namespace RecursiveAI.OnePhysics.Tests.Articulations;

public sealed class ArticulationWorldTests3D
{
    private readonly ITestOutputHelper _out;
    public ArticulationWorldTests3D(ITestOutputHelper output) { _out = output; }

    // Writes a line to the file named by ARTI_TRACE (when set), so a passing run can be looked at.
    private static void Trace(string line)
    {
        string? path = Environment.GetEnvironmentVariable("ARTI_TRACE");
        if (path != null) { System.IO.File.AppendAllText(path, line + "\n"); }
    }

    [Fact]
    public void QuadrupedStandsOnAFloor()
    {
        var world = new PhysicsWorld3D();
        world.AddBox(new Vector3F(20f, 1f, 20f), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var q = world.AddArticulation(ArticulationDef.Quadruped(), new Vector3D(0, 0.57, 0));
        var torso = q.GetLink(0);
        double minFoot = 1e9;
        for (int i = 0; i < 300; i++)
        {
            world.Step(1f / 60f);
            if (i % 30 == 0) { Trace(i + ": torso " + torso.Position + " contacts " + q.ContactPointCount); }
            for (int l = 1; l < q.LinkCount; l++)
            {
                var b = q.GetLink(l);
                minFoot = Math.Min(minFoot, b.Position.Y);
            }
        }

        Trace("min link y " + minFoot);
        Assert.True(torso.Position.Y > 0.45, "torso at " + torso.Position);
    }

    private static ArticulationDef Pendulum2()
    {
        var def = new ArticulationDef { FixedBase = true };
        def.AddLink(new ArticulationLinkDef { Mass = 1f, InertiaTensor = new Matrix3x3F(1, 0, 0, 0, 1, 0, 0, 0, 1) });
        for (int i = 0; i < 2; i++)
        {
            var l = new ArticulationLinkDef
            {
                Parent = i, Joint = ArticulationJointKind.Revolute, Axis = new Vector3F(0, 0, 1), Origin = new Vector3F(i == 0 ? 0 : 0.6f, 0, 0), Mass = i == 0 ? 2f : 1f, CenterOfMass = new Vector3F(0.3f, 0, 0),
                InertiaTensor = new Matrix3x3F(0.001f, 0, 0, 0, 0.05f, 0, 0, 0, 0.05f),
            };
            l.AddShape(ShapeSpec3D.Sphere(0.02f), new ShapeDef { CategoryBits = 0, MaskBits = 0 });
            def.AddLink(l);
        }

        return def;
    }

    // The compound double pendulum of Pendulum2 integrated with RK4 from its Lagrangian: the closed-form reference.
    private static (double X, double Y)[] AnalyticDoublePendulum(int steps, double dt)
    {
        const double m1 = 2, m2 = 1, l1 = 0.6, c1 = 0.3, c2 = 0.3, i1 = 0.05, i2 = 0.05, g = 9.81;
        double[] Deriv(double[] y)
        {
            double t1 = y[0], t2 = y[1], w1 = y[2], w2 = y[3];
            double d = t1 - t2, a = m1 * c1 * c1 + i1 + m2 * l1 * l1, b = m2 * l1 * c2 * Math.Cos(d), dd = m2 * c2 * c2 + i2;
            double r1 = -m2 * l1 * c2 * Math.Sin(d) * w2 * w2 - (m1 * c1 + m2 * l1) * g * Math.Cos(t1);
            double r2 = m2 * l1 * c2 * Math.Sin(d) * w1 * w1 - m2 * c2 * g * Math.Cos(t2);
            double det = a * dd - b * b;
            return new[] { w1, w2, (r1 * dd - b * r2) / det, (a * r2 - b * r1) / det };
        }

        var y0 = new[] { 0.5, 0.2, 0.0, 0.0 };
        var result = new (double, double)[steps + 1];
        result[0] = (l1 * Math.Cos(y0[0]) + l1 * Math.Cos(y0[1]), l1 * Math.Sin(y0[0]) + l1 * Math.Sin(y0[1]));
        int sub = 100;
        double h = dt / sub;
        for (int s = 1; s <= steps; s++)
        {
            for (int k = 0; k < sub; k++)
            {
                var k1 = Deriv(y0);
                var k2 = Deriv(Add(y0, k1, h / 2)); var k3 = Deriv(Add(y0, k2, h / 2)); var k4 = Deriv(Add(y0, k3, h));
                for (int j = 0; j < 4; j++) { y0[j] += h / 6 * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]); }
            }

            result[s] = (l1 * Math.Cos(y0[0]) + l1 * Math.Cos(y0[1]), l1 * Math.Sin(y0[0]) + l1 * Math.Sin(y0[1]));
        }

        return result;
    }

    private static double[] Add(double[] y, double[] k, double h) { var r = new double[4]; for (int i = 0; i < 4; i++) { r[i] = y[i] + h * k[i]; } return r; }

    [Fact]
    public void DoublePendulumMatchesTheMaximalCoordinateReference()
    {
        var wa = new PhysicsWorld3D();
        var art = wa.AddArticulation(Pendulum2(), new Vector3D(0, 5, 0));
        art.SetJointPosition(1, 0.5f);
        art.SetJointPosition(2, -0.3f);
        // The same pendulum from ordinary bodies and hinge joints.
        // The reference gets more substeps than the default so that its own joint softness stays small next to what is being compared.
        var wb = new PhysicsWorld3D(new WorldOptions3D { Substeps = 16, VelocityIterations = 4, RelaxIterations = 2 });
        var anchor = wb.CreateBody(BodyType.Static, new Vector3D(0, 5, 0));
        Body Link(Vector3D at, float angle, float mass, float ixx)
        {
            var q = new QuaternionF(0, 0, (float)Math.Sin(angle / 2), (float)Math.Cos(angle / 2));
            var def = new BodyDef { Position = at, Rotation = q, MassOverride = mass, CenterOfMassOverride = new Vector3F(0.3f, 0, 0), InertiaOverride = new Vector3F(0.001f, 0.05f, 0.05f) };
            var b = wb.CreateBody(def);
            wb.CreateShape(b, ShapeSpec3D.Sphere(0.02f), new ShapeDef { CategoryBits = 0, MaskBits = 0 });
            return b;
        }

        double a1 = 0.5, a2 = -0.3;
        var b1 = Link(new Vector3D(0, 5, 0), (float)a1, 2f, 0.001f);
        var p2 = new Vector3D(0.6 * Math.Cos(a1), 5 + 0.6 * Math.Sin(a1), 0);
        var b2 = Link(p2, (float)(a1 + a2), 1f, 0.001f);
        wb.AddHinge(anchor, b1, new Vector3D(0, 5, 0), new Vector3F(0, 0, 1));
        wb.AddHinge(b1, b2, p2, new Vector3F(0, 0, 1));
        double worst = 0, worstAnalytic = 0;
        var exact = AnalyticDoublePendulum(60, 1.0 / 60.0);
        for (int i = 0; i < 60; i++)
        {
            wa.Step(1f / 60f); wb.Step(1f / 60f);
            var ta = art.GetPointPosition(2, new Vector3F(0.6f, 0, 0));
            var qb = b2.Rotation; var tb = b2.Position + new Vector3D(0, 0, 0);
            var local = new Vector3F(0.6f, 0, 0);
            var off = qb.Rotate(local);
            var tip = new Vector3D(b2.Position.X + off.X, b2.Position.Y + off.Y, b2.Position.Z + off.Z);
            // The reference body's position is its origin (the joint end), so the tip is origin + R (0.6, 0, 0).
            worst = Math.Max(worst, Math.Sqrt((ta.X - tip.X) * (ta.X - tip.X) + (ta.Y - tip.Y) * (ta.Y - tip.Y)));
            _ = tb;
            worstAnalytic = Math.Max(worstAnalytic, Math.Sqrt((ta.X - exact[i + 1].X) * (ta.X - exact[i + 1].X) + (ta.Y - 5 - exact[i + 1].Y) * (ta.Y - 5 - exact[i + 1].Y)));
        }

        Trace("double pendulum worst tip difference over 1 s: reference " + worst + ", closed form " + worstAnalytic);
        Assert.True(worstAnalytic < 0.002, "difference from the closed form " + worstAnalytic);
        Assert.True(worst < 0.05, "difference from the maximal-coordinate reference " + worst);
    }

    [Fact]
    public void GravityCompensationHoldsASevenJointArm()
    {
        var world = new PhysicsWorld3D();
        var arm = world.AddArticulation(ArticulationDef.RobotArm(), new Vector3D(0, 0.5, 0));
        // Start off-vertical so the drive has to carry the arm's weight.
        for (int i = 1; i < arm.LinkCount; i++) { if (i % 2 == 0) { arm.SetJointPosition(i, 0.6f); arm.SetJointTarget(i, 0.6f); } }
        var q0 = new double[arm.PositionCount];
        arm.GetPositions(q0);
        for (int i = 0; i < 600; i++) { world.Step(1f / 60f); }
        var q1 = new double[arm.PositionCount];
        arm.GetPositions(q1);
        double worst = 0;
        for (int i = 0; i < q0.Length; i++) { worst = Math.Max(worst, Math.Abs(q1[i] - q0[i])); }
        Trace("arm drift over 10 s: " + worst);
        Assert.True(worst < 1e-4, "drift " + worst);
    }

    [Fact]
    public void ChainOfSixtyFourLinksWithAThousandToOneEndMassStaysConnected()
    {
        var world = new PhysicsWorld3D();
        var chain = world.AddArticulation(ArticulationDef.Chain(64, 0.25f, 1f, 1000f), new Vector3D(0, 30, 0));
        // Start it horizontal (along +X from the base), so it swings down.
        double worstSeparation = 0, worstSpeed = 0;
        for (int i = 0; i < 600; i++)
        {
            world.Step(1f / 60f);
            for (int l = 1; l < chain.LinkCount; l++)
            {
                var parent = chain.GetLink(l - 1); var child = chain.GetLink(l);
                var origin = l == 1 ? new Vector3F(0, 0, 0) : new Vector3F(0.25f, 0, 0);
                var expect = parent.Position + new Vector3D(parent.Rotation.Rotate(origin).X, parent.Rotation.Rotate(origin).Y, parent.Rotation.Rotate(origin).Z);
                worstSeparation = Math.Max(worstSeparation, (child.Position - expect).Length());
                worstSpeed = Math.Max(worstSpeed, child.LinearVelocity.Length());
            }
        }

        Trace("chain: worst separation " + worstSeparation + " worst speed " + worstSpeed);
        Assert.True(worstSeparation < 1e-4, "separation " + worstSeparation);
        Assert.True(worstSpeed < 60.0, "speed " + worstSpeed);
    }

    [Fact]
    public void SnapshotRestoreAndForkReplayTheSameHashes()
    {
        PhysicsWorld3D Make()
        {
            var w = new PhysicsWorld3D();
            w.AddBox(new Vector3F(20f, 1f, 20f), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
            w.AddArticulation(ArticulationDef.Quadruped(), new Vector3D(0, 0.6, 0));
            w.AddBox(new Vector3F(0.3f, 0.3f, 0.3f), at: new Vector3D(0.8, 0.15, 0.1));
            return w;
        }

        var a = Make();
        for (int i = 0; i < 40; i++) { a.Step(1f / 60f); }
        var bytes = a.Snapshot();
        var fork = a.Fork();
        var b = new PhysicsWorld3D();
        b.Restore(bytes);
        Assert.Equal(a.ComputeHash(), b.ComputeHash());
        for (int i = 0; i < 40; i++)
        {
            a.Step(1f / 60f); b.Step(1f / 60f); fork.Step(1f / 60f);
            Assert.Equal(a.ComputeHash(), b.ComputeHash());
            Assert.Equal(a.ComputeHash(), fork.ComputeHash());
        }

        Assert.Equal(a.Snapshot(), b.Snapshot());
    }
}
