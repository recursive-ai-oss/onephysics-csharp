using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

public class Joint3DTests
{
    private const float Dt = 1f / 120f;

    private static PhysicsWorld3D NoGravity(int threads = 1) => World3DTests.NewWorld(threads, 11, o => { o.Gravity = default; });

    private static Body Anchor(PhysicsWorld3D w, double x = 0, double y = 0, double z = 0) => w.AddBox(new Vector3F(0.2f, 0.2f, 0.2f), new Vector3D(x, y, z), type: BodyType.Static);

    private static QuaternionF AxisAngle(Vector3F axis, float angle)
    {
        double s = Math.Sin(angle * 0.5), c = Math.Cos(angle * 0.5);
        return new QuaternionF(axis.X * (float)s, axis.Y * (float)s, axis.Z * (float)s, (float)c);
    }

    private static double Len(Vector3D v) => Math.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z);

    // ------------------------------------------------------------------ pendulum, limits, motors, springs

    [Fact]
    public void HingePendulumPeriodMatchesTheAnalyticPeriod()
    {
        var w = World3DTests.NewWorld();
        const double L = 1.0, y0 = 10.0, r = 0.1, theta0 = 0.15;
        var pivot = Anchor(w, 0, y0, 0);
        var bob = w.AddSphere((float)r, new Vector3D(L * Math.Sin(theta0), y0 - L * Math.Cos(theta0), 0));
        w.AddHinge(pivot, bob, new Vector3D(0, y0, 0), new Vector3F(0, 0, 1));
        var crossings = new List<double>();
        double prev = theta0;
        for (int i = 1; i <= 120 * 8; i++)
        {
            w.Step(Dt);
            var p = bob.Position;
            double a = Math.Atan2(p.X, y0 - p.Y);
            if (prev < 0 && a >= 0) { crossings.Add((i - 1 + (0 - prev) / (a - prev)) * Dt); }
            prev = a;
        }

        Assert.True(crossings.Count >= 4, "crossings " + crossings.Count);
        double period = (crossings[^1] - crossings[0]) / (crossings.Count - 1);
        double expected = 2 * Math.PI * Math.Sqrt((0.4 * r * r + L * L) / (9.81 * L)) * (1 + theta0 * theta0 / 16);
        Assert.InRange(period / expected, 0.995, 1.005);
    }

    [Fact]
    public void PointJointPendulumSwingsInAPlaneAtTheAnalyticPeriod()
    {
        var w = World3DTests.NewWorld();
        const double L = 2.0, y0 = 20.0, theta0 = 0.12;
        var pivot = Anchor(w, 0, y0, 0);
        var bob = w.AddSphere(0.05f, new Vector3D(L * Math.Sin(theta0), y0 - L * Math.Cos(theta0), 0));
        w.AddPoint(pivot, bob, new Vector3D(0, y0, 0));
        var crossings = new List<double>();
        double prev = theta0;
        for (int i = 1; i <= 120 * 12; i++)
        {
            w.Step(Dt);
            var p = bob.Position;
            double a = Math.Atan2(p.X, y0 - p.Y);
            if (prev < 0 && a >= 0) { crossings.Add((i - 1 + (0 - prev) / (a - prev)) * Dt); }
            prev = a;
        }

        double period = (crossings[^1] - crossings[0]) / (crossings.Count - 1);
        double expected = 2 * Math.PI * Math.Sqrt((0.4 * 0.05 * 0.05 + L * L) / (9.81 * L)) * (1 + theta0 * theta0 / 16);
        Assert.InRange(period / expected, 0.995, 1.005);
        Assert.True(Math.Abs(bob.Position.Z) < 0.01);
    }

    [Fact]
    public void HingeLimitHoldsWithinAStatedOvershoot()
    {
        var w = World3DTests.NewWorld();
        var pivot = Anchor(w, 0, 10, 0);
        var arm = w.AddBox(new Vector3F(1f, 0.1f, 0.1f), new Vector3D(0.5, 10, 0));
        var j = w.AddHinge(pivot, arm, new Vector3D(0, 10, 0), new Vector3F(0, 0, 1));
        j.SetLimits(-0.3f, 0.6f);
        j.LimitEnabled = true;
        double worstLow = 0, worstHigh = 0;
        for (int i = 0; i < 600; i++)
        {
            w.Step(Dt);
            var tip = arm.Rotation.Rotate(new Vector3F(1, 0, 0));
            double angle = Math.Atan2(tip.Y, tip.X);
            worstLow = Math.Min(worstLow, angle + 0.3);
            worstHigh = Math.Max(worstHigh, angle - 0.6);
        }

        Assert.True(worstLow > -0.03, "low overshoot " + worstLow);
        Assert.True(worstHigh < 0.03, "high overshoot " + worstHigh);
        var end = arm.Rotation.Rotate(new Vector3F(1, 0, 0));
        Assert.InRange(Math.Atan2(end.Y, end.X), -0.33, -0.2); // rests against the lower limit under gravity
    }

    [Fact]
    public void SliderLimitHoldsAndStaysOnItsAxis()
    {
        var w = NoGravity();
        var rail = Anchor(w);
        var car = w.AddBox(new Vector3F(0.5f, 0.5f, 0.5f), new Vector3D(0, 0, 0));
        var j = w.AddSlider(rail, car, new Vector3D(0, 0, 0), new Vector3F(1, 0, 0));
        j.SetLimits(-1f, 1f);
        j.LimitEnabled = true;
        car.LinearVelocity = new Vector3F(8, 0.5f, 0.3f);
        double worst = 0, off = 0;
        for (int i = 0; i < 360; i++)
        {
            w.Step(Dt);
            worst = Math.Max(worst, car.Position.X - 1);
            off = Math.Max(off, Math.Max(Math.Abs(car.Position.Y), Math.Abs(car.Position.Z)));
        }

        Assert.True(worst < 0.03, "overshoot " + worst);
        Assert.True(off < 0.01, "off axis " + off);
    }

    [Fact]
    public void HingeMotorReachesTargetSpeedAndIsCappedByItsTorque()
    {
        var w = NoGravity();
        var pivot = Anchor(w);
        var wheel = w.AddCylinder(0.1f, 0.5f, new Vector3D(0, 0, 0), AxisAngle(new Vector3F(1, 0, 0), MathF.PI / 2));
        var j = w.AddHinge(pivot, wheel, new Vector3D(0, 0, 0), new Vector3F(0, 0, 1));
        j.MotorSpeed = 3f; j.MaxMotorForce = 1000f; j.MotorEnabled = true;
        for (int i = 0; i < 120; i++) { w.Step(Dt); }
        Assert.InRange(wheel.AngularVelocity.Z, 2.98f, 3.02f);

        var w2 = NoGravity();
        var pivot2 = Anchor(w2);
        var wheel2 = w2.AddCylinder(0.1f, 0.5f, new Vector3D(0, 0, 0), AxisAngle(new Vector3F(1, 0, 0), MathF.PI / 2));
        var j2 = w2.AddHinge(pivot2, wheel2, new Vector3D(0, 0, 0), new Vector3F(0, 0, 1));
        float inertia = wheel2.Inertia.M22; // the cylinder's own axis, turned to world z
        j2.MotorSpeed = 50f; j2.MaxMotorForce = 0.5f; j2.MotorEnabled = true;
        for (int i = 0; i < 120; i++) { w2.Step(Dt); }
        // After one second a torque of 0.5 N m has added 0.5 / I rad/s.
        Assert.InRange(Math.Abs(wheel2.AngularVelocity.Z) * inertia, 0.45f, 0.55f);
    }

    [Fact]
    public void SliderMotorReachesTargetSpeed()
    {
        var w = NoGravity();
        var rail = Anchor(w);
        var car = w.AddBox(new Vector3F(0.5f, 0.5f, 0.5f), new Vector3D(0, 0, 0));
        var j = w.AddSlider(rail, car, new Vector3D(0, 0, 0), new Vector3F(0, 1, 0));
        j.MotorSpeed = 2f; j.MaxMotorForce = 100f; j.MotorEnabled = true;
        for (int i = 0; i < 120; i++) { w.Step(Dt); }
        Assert.InRange(car.LinearVelocity.Y, 1.98f, 2.02f);
    }

    private static (double Period, double Decay) Oscillate(PhysicsWorld3D w, Body b, int steps)
    {
        var ups = new List<double>(); var peaks = new List<double>();
        double prev = b.Position.X, prevV = 0;
        for (int i = 1; i <= steps; i++)
        {
            w.Step(Dt);
            double x = b.Position.X;
            if (prev < 0 && x >= 0) { ups.Add((i - 1 + (0 - prev) / (x - prev)) * Dt); }
            double v = b.LinearVelocity.X;
            if (prevV > 0 && v <= 0) { peaks.Add(x); }
            prev = x; prevV = v;
        }

        Assert.True(ups.Count >= 3 && peaks.Count >= 3, "oscillations " + ups.Count + "/" + peaks.Count);
        double period = (ups[^1] - ups[0]) / (ups.Count - 1);
        double decay = Math.Log(peaks[0] / peaks[2]) / 2; // log decrement per cycle
        return (period, decay);
    }

    [Theory]
    [InlineData(2.0f, 0.05f)]
    [InlineData(3.0f, 0.2f)]
    public void SliderSpringHasTheAnalyticFrequencyAndDamping(float hertz, float zeta)
    {
        var w = World3DTests.NewWorld(1, 11, o => { o.Gravity = default; o.Substeps = 8; });
        var rail = Anchor(w);
        var car = w.AddBox(new Vector3F(0.5f, 0.5f, 0.5f), new Vector3D(0, 0, 0));
        var j = w.AddSlider(rail, car, new Vector3D(0, 0, 0), new Vector3F(1, 0, 0));
        j.SpringHertz = hertz; j.SpringDampingRatio = zeta; j.SpringEnabled = true;
        car.Position = new Vector3D(0.4, 0, 0); // stretch the spring: translation 0 is where the frames met at creation
        var (period, decay) = Oscillate(w, car, 120 * 8);
        double wn = 2 * Math.PI * hertz, wd = wn * Math.Sqrt(1 - zeta * zeta);
        Assert.InRange(period / (2 * Math.PI / wd), 0.98, 1.02);
        double expectedDecay = zeta * wn * (2 * Math.PI / wd); // log decrement per cycle
        Assert.InRange(decay / expectedDecay, 0.98, 1.02);
    }

    [Fact]
    public void DistanceSpringOscillatesAtItsFrequency()
    {
        var w = World3DTests.NewWorld(1, 11, o => { o.Gravity = default; o.Substeps = 8; });
        var a = Anchor(w, -2, 0, 0);
        var b = w.AddBox(new Vector3F(0.5f, 0.5f, 0.5f), new Vector3D(0, 0, 0));
        // Rest length 2 between the anchors; stretch it by 0.3 and let it go: it oscillates about x = 0.
        w.AddSpring(a, b, new Vector3D(-2, 0, 0), new Vector3D(0, 0, 0), 2.5f, 0.1f);
        b.Position = new Vector3D(0.3, 0, 0);
        var (period, _) = Oscillate(w, b, 120 * 6);
        double wn = 2 * Math.PI * 2.5, wd = wn * Math.Sqrt(1 - 0.01);
        Assert.InRange(period / (2 * Math.PI / wd), 0.98, 1.02);
    }

    // ------------------------------------------------------------------ cone, swing-twist, six dof

    private static uint _seed;

    private static float Rand()
    {
        _seed = _seed * 1664525u + 1013904223u;
        return ((_seed >> 8) & 0xFFFF) / 32768f - 1f;
    }

    [Fact]
    public void ConeLimitHoldsOverRandomDrivenMotion()
    {
        _seed = 12345;
        var w = NoGravity();
        var parent = Anchor(w);
        var child = w.AddBox(new Vector3F(1f, 0.4f, 0.4f), new Vector3D(0.5, 0, 0));
        const float half = 0.5f;
        w.AddCone(parent, child, new Vector3D(0, 0, 0), new Vector3F(1, 0, 0), half);
        double worst = 0;
        for (int i = 0; i < 1500; i++)
        {
            child.ApplyTorque(new Vector3F(Rand(), Rand(), Rand()) * 8f);
            w.Step(Dt);
            var axis = child.Rotation.Rotate(new Vector3F(1, 0, 0));
            double angle = Math.Acos(Math.Clamp(axis.X, -1f, 1f));
            worst = Math.Max(worst, angle - half);
        }

        Assert.True(worst < 0.06, "violation " + worst);
        Assert.True(Len(child.Position - new Vector3D(0.5, 0, 0)) < 1.1); // the socket held
    }

    private static void SwingTwistOf(QuaternionF rel, out double twist, out double ey, out double ez, double ly, double lz)
    {
        double x = rel.X, y = rel.Y, z = rel.Z, wq = rel.W;
        if (wq < 0) { x = -x; y = -y; z = -z; wq = -wq; }
        double n = Math.Sqrt(x * x + wq * wq);
        double tx = n > 1e-12 ? x / n : 0, tw = n > 1e-12 ? wq / n : 1;
        twist = 2 * Math.Atan2(tx, tw);
        // swing = q * conj(twist) with twist = (tx, 0, 0, tw): the Hamilton product written out for its y and z parts.
        double sy = y * tw + z * tx, sz = z * tw - y * tx;
        ey = sy / ly; ez = sz / lz;
    }

    [Fact]
    public void SwingTwistLimitsHoldOverRandomDrivenMotion()
    {
        _seed = 777;
        var w = NoGravity();
        var parent = Anchor(w);
        var child = w.AddBox(new Vector3F(1f, 0.4f, 0.4f), new Vector3D(0.5, 0, 0));
        const float tw = 0.4f, sy = 0.6f, sz = 0.3f;
        w.AddCharacterJoint(parent, child, new Vector3D(0, 0, 0), new Vector3F(1, 0, 0), -tw, tw, sy, sz);
        double ly = Math.Sin(sy / 2), lz = Math.Sin(sz / 2);
        double worstTwist = 0, worstEllipse = 0;
        for (int i = 0; i < 1500; i++)
        {
            child.ApplyTorque(new Vector3F(Rand(), Rand(), Rand()) * 6f);
            w.Step(Dt);
            SwingTwistOf(child.Rotation, out double twist, out double ey, out double ez, ly, lz);
            worstTwist = Math.Max(worstTwist, Math.Abs(twist) - tw);
            worstEllipse = Math.Max(worstEllipse, Math.Sqrt(ey * ey + ez * ez) - 1);
        }

        Assert.True(worstTwist < 0.06, "twist violation " + worstTwist);
        Assert.True(worstEllipse < 0.15, "swing violation (relative) " + worstEllipse);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    [InlineData(5)]
    public void SixDofFreesExactlyTheAxisItIsToldTo(int axis)
    {
        var w = NoGravity();
        var parent = Anchor(w);
        var child = w.AddBox(new Vector3F(0.5f, 0.5f, 0.5f), new Vector3D(0, 0, 0));
        var j = w.AddConfigurable(parent, child, new Vector3D(0, 0, 0));
        j.SetAxisMode((JointAxes)(1 << axis), JointAxisMode.Free);
        var e = new Vector3F(axis % 3 == 0 ? 1 : 0, axis % 3 == 1 ? 1 : 0, axis % 3 == 2 ? 1 : 0);
        if (axis < 3) { child.ApplyImpulse(e * child.Mass); } else { child.ApplyAngularImpulse(e * 0.1f); }
        for (int i = 0; i < 120; i++) { w.Step(Dt); }
        var p = child.Position;
        var q = child.Rotation;
        double[] lin = [p.X, p.Y, p.Z];
        double[] rot = [2 * q.X, 2 * q.Y, 2 * q.Z];
        for (int k = 0; k < 3; k++)
        {
            if (axis < 3 && k == axis) { Assert.True(Math.Abs(lin[k]) > 0.5, "free translation moved " + lin[k]); }
            else { Assert.True(Math.Abs(lin[k]) < 0.01, "locked translation " + k + " drifted " + lin[k]); }
            if (axis >= 3 && k == axis - 3) { Assert.True(Math.Abs(rot[k]) > 0.3, "free rotation moved " + rot[k]); }
            else { Assert.True(Math.Abs(rot[k]) < 0.01, "locked rotation " + k + " drifted " + rot[k]); }
        }
    }

    [Fact]
    public void SixDofLockedAxesResistImpulses()
    {
        var w = NoGravity();
        var parent = Anchor(w);
        var child = w.AddBox(new Vector3F(0.5f, 0.5f, 0.5f), new Vector3D(0, 0, 0));
        w.AddConfigurable(parent, child, new Vector3D(0, 0, 0));
        child.ApplyImpulse(new Vector3F(1, 2, 3) * child.Mass);
        child.ApplyAngularImpulse(new Vector3F(0.2f, -0.1f, 0.3f));
        for (int i = 0; i < 240; i++) { w.Step(Dt); }
        Assert.True(Len(child.Position) < 0.02);
        Assert.True(Math.Abs(child.Rotation.X) + Math.Abs(child.Rotation.Y) + Math.Abs(child.Rotation.Z) < 0.02);
    }

    [Fact]
    public void SixDofLimitedAxisStopsAtItsLimits()
    {
        var w = NoGravity();
        var parent = Anchor(w);
        var child = w.AddBox(new Vector3F(0.5f, 0.5f, 0.5f), new Vector3D(0, 0, 0));
        var j = w.AddConfigurable(parent, child, new Vector3D(0, 0, 0));
        j.SetAxisMode(JointAxes.LinearY, JointAxisMode.Limited);
        j.SetAxisLimits(JointAxes.LinearY, -0.5f, 0.75f);
        child.LinearVelocity = new Vector3F(0, 6, 0);
        double top = 0;
        for (int i = 0; i < 120; i++) { w.Step(Dt); top = Math.Max(top, child.Position.Y); }
        Assert.InRange(top, 0.7, 0.8);
    }

    // ------------------------------------------------------------------ gear, rack and pinion, pulley, path

    [Fact]
    public void GearKeepsItsRatio()
    {
        var w = NoGravity();
        var ground = Anchor(w, 0, 0, -5);
        var a = w.AddCylinder(0.1f, 0.5f, new Vector3D(0, 0, 0), AxisAngle(new Vector3F(1, 0, 0), MathF.PI / 2));
        var b = w.AddCylinder(0.1f, 0.25f, new Vector3D(1, 0, 0), AxisAngle(new Vector3F(1, 0, 0), MathF.PI / 2));
        w.AddHinge(ground, a, new Vector3D(0, 0, 0), new Vector3F(0, 0, 1));
        w.AddHinge(ground, b, new Vector3D(1, 0, 0), new Vector3F(0, 0, 1));
        w.AddGear(a, b, new Vector3F(0, 0, 1), new Vector3F(0, 0, 1), 2f);
        a.AngularVelocity = new Vector3F(0, 0, 3f);
        for (int i = 0; i < 240; i++) { w.Step(Dt); }
        Assert.InRange(a.AngularVelocity.Z + 2f * b.AngularVelocity.Z, -0.01f, 0.01f);
        Assert.True(Math.Abs(b.AngularVelocity.Z) > 0.1f);
    }

    [Fact]
    public void RackAndPinionKeepsItsRatio()
    {
        var w = NoGravity();
        var ground = Anchor(w, 0, 0, -5);
        var pinion = w.AddCylinder(0.1f, 0.3f, new Vector3D(0, 0, 0), AxisAngle(new Vector3F(1, 0, 0), MathF.PI / 2));
        var rack = w.AddBox(new Vector3F(2f, 0.1f, 0.1f), new Vector3D(0, 0.4, 0));
        w.AddHinge(ground, pinion, new Vector3D(0, 0, 0), new Vector3F(0, 0, 1));
        w.AddSlider(ground, rack, new Vector3D(0, 0.4, 0), new Vector3F(1, 0, 0));
        w.AddRackAndPinion(pinion, rack, new Vector3F(0, 0, 1), new Vector3F(1, 0, 0), 0.3f);
        pinion.AngularVelocity = new Vector3F(0, 0, 2f);
        for (int i = 0; i < 240; i++) { w.Step(Dt); }
        // Pinion and rack share the motion: momentum is exchanged until the two speeds satisfy the ratio.
        Assert.InRange(rack.LinearVelocity.X - 0.3f * pinion.AngularVelocity.Z, -0.005f, 0.005f);
        Assert.True(rack.LinearVelocity.X > 0.1f);
    }

    [Fact]
    public void PulleyKeepsTheRopeLengthAndTheMassesAccelerateAsAnAtwoodMachine()
    {
        var w = World3DTests.NewWorld();
        var ground = Anchor(w, 0, 20, 0);
        var heavy = w.AddBox(new Vector3F(0.5f, 0.5f, 0.5f), new Vector3D(-1, 10, 0), density: 2f);
        var light = w.AddBox(new Vector3F(0.5f, 0.5f, 0.5f), new Vector3D(1, 10, 0), density: 1f);
        w.AddPulley(heavy, light, heavy.Position, light.Position, new Vector3D(-1, 20, 0), new Vector3D(1, 20, 0));
        for (int i = 0; i < 60; i++) { w.Step(Dt); }
        double la = Len(heavy.Position - new Vector3D(-1, 20, 0)), lb = Len(light.Position - new Vector3D(1, 20, 0));
        Assert.InRange(la + lb, 19.97, 20.03);
        double m1 = heavy.Mass, m2 = light.Mass;
        double expected = (m1 - m2) / (m1 + m2) * 9.81 * 0.5; // speed after half a second
        Assert.InRange(-heavy.LinearVelocity.Y / expected, 0.95, 1.05);
        _ = ground;
    }

    [Fact]
    public void PathJointKeepsTheFollowerOnTheCurveAndTheMotorDrivesIt()
    {
        var w = NoGravity();
        var track = Anchor(w);
        var cart = w.AddBox(new Vector3F(0.3f, 0.3f, 0.3f), new Vector3D(0, 0, 0));
        Vector3D[] pts = [new(0, 0, 0), new(2, 0, 0), new(4, 1, 0), new(6, 1, 2), new(8, 0, 2)];
        var j = w.AddPath(track, cart, pts);
        j.MotorSpeed = 1.5f; j.MaxMotorForce = 500f; j.MotorEnabled = true;
        double worst = 0;
        for (int i = 0; i < 360; i++)
        {
            w.Step(Dt);
            // Distance to the polyline through the control points is at most the spline's deviation from it plus the constraint error.
            double best = double.MaxValue;
            for (int k = 0; k + 1 < pts.Length; k++) { best = Math.Min(best, SegmentDistance(cart.Position, pts[k], pts[k + 1])); }
            worst = Math.Max(worst, best);
        }

        Assert.True(worst < 0.25, "off the curve by " + worst);
        Assert.True(cart.Position.X > 3.5, "x " + cart.Position.X);
        Assert.InRange(cart.LinearVelocity.X * cart.LinearVelocity.X + cart.LinearVelocity.Y * cart.LinearVelocity.Y + cart.LinearVelocity.Z * cart.LinearVelocity.Z, 1.9f, 2.6f);
    }

    private static double SegmentDistance(Vector3D p, Vector3D a, Vector3D b)
    {
        double abx = b.X - a.X, aby = b.Y - a.Y, abz = b.Z - a.Z;
        double t = ((p.X - a.X) * abx + (p.Y - a.Y) * aby + (p.Z - a.Z) * abz) / (abx * abx + aby * aby + abz * abz);
        t = Math.Clamp(t, 0, 1);
        double dx = p.X - (a.X + t * abx), dy = p.Y - (a.Y + t * aby), dz = p.Z - (a.Z + t * abz);
        return Math.Sqrt(dx * dx + dy * dy + dz * dz);
    }

    [Fact]
    public void FixedJointHoldsABodyAndDistanceJointHoldsItsLength()
    {
        var w = World3DTests.NewWorld();
        var anchor = Anchor(w, 0, 10, 0);
        var hang = w.AddBox(new Vector3F(0.5f, 0.5f, 0.5f), new Vector3D(1, 10, 0));
        w.AddFixed(anchor, hang, new Vector3D(0.5, 10, 0));
        var rod = w.AddSphere(0.3f, new Vector3D(-3, 10, 0));
        var rodAnchor = Anchor(w, -3, 12, 0);
        w.AddDistance(rodAnchor, rod, new Vector3D(-3, 12, 0), new Vector3D(-3, 10, 0));
        for (int i = 0; i < 240; i++) { w.Step(Dt); }
        Assert.True(Len(hang.Position - new Vector3D(1, 10, 0)) < 0.03);
        Assert.True(Math.Abs(hang.Rotation.W) > 0.999f);
        Assert.InRange(Len(rod.Position - new Vector3D(-3, 12, 0)), 1.97, 2.03);
    }

    // ------------------------------------------------------------------ breaking, handles, hostile input

    [Fact]
    public void BreakingFiresExactlyOnceAndDestroysTheJoint()
    {
        var w = World3DTests.NewWorld();
        var anchor = Anchor(w, 0, 10, 0);
        var body = w.AddBox(new Vector3F(0.5f, 0.5f, 0.5f), new Vector3D(0, 9, 0), density: 100f);
        var j = w.AddPoint(anchor, body, new Vector3D(0, 10, 0));
        float weight = body.Mass * 9.81f;
        j.BreakForce = weight * 0.5f;
        int calls = 0; float value = 0;
        j.OnJointBreak += e => { calls++; value = e.Value; Assert.False(e.Joint.IsValid); };
        int events = 0;
        for (int i = 0; i < 120; i++)
        {
            w.Step(Dt);
            foreach (var e in w.Events) { if (e.Kind == EventKind.JointBreak) { events++; } }
        }

        Assert.Equal(1, calls);
        Assert.Equal(1, events);
        Assert.True(value > weight * 0.5f);
        Assert.False(j.IsValid);
        Assert.Throws<StaleHandleException>(() => _ = j.Type);
        Assert.Equal(0, w.JointCount);
        Assert.True(body.Position.Y < 8.5); // it fell
    }

    [Fact]
    public void StaleAndForeignHandlesFailClearly()
    {
        var w = World3DTests.NewWorld();
        var a = Anchor(w); var b = w.AddSphere(0.3f, new Vector3D(1, 0, 0));
        var j = w.AddPoint(a, b, new Vector3D(0.5, 0, 0));
        Assert.Equal(JointType3D.Point, j.Type);
        j.Destroy();
        Assert.Throws<StaleHandleException>(() => _ = j.BodyA);
        Assert.Throws<StaleHandleException>(() => j.Destroy());
        var j2 = w.AddPoint(a, b, new Vector3D(0.5, 0, 0));
        b.Destroy();
        Assert.False(j2.IsValid);
        Assert.Throws<StaleHandleException>(() => _ = j2.ReactionForce);
        Assert.Throws<InvalidHandleException>(() => _ = default(Joint).Type);
        var other = World3DTests.NewWorld(1, 99);
        var c = other.AddSphere(0.3f, default);
        Assert.Throws<ForeignHandleException>(() => w.AddPoint(a, c, default));
        var e = w.AddSphere(0.3f, new Vector3D(3, 0, 0));
        var hinge = w.AddHinge(a, e, new Vector3D(1.5, 0, 0), new Vector3F(0, 0, 1));
        Assert.Throws<InvalidJointException>(() => _ = hinge.Length);
        Assert.Throws<InvalidJointException>(() => hinge.SetAxisMode(JointAxes.LinearX, JointAxisMode.Free));
    }

    [Fact]
    public void HostileDefinitionsGiveTypedErrors()
    {
        var w = World3DTests.NewWorld();
        var a = Anchor(w); var b = w.AddSphere(0.3f, new Vector3D(1, 0, 0));
        var s = Anchor(w, 5, 0, 0);
        Assert.Throws<InvalidJointException>(() => w.AddPoint(a, a, default));
        Assert.Throws<InvalidJointException>(() => w.AddPoint(a, s, default));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddPoint(a, b, new Vector3D(double.NaN, 0, 0)));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddPoint(a, b, new Vector3D(3e6, 0, 0)));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddHinge(a, b, default, default));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddHinge(a, b, default, new Vector3F(float.NaN, 0, 0)));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddCone(a, b, default, new Vector3F(1, 0, 0), 0f));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddCone(a, b, default, new Vector3F(1, 0, 0), float.PositiveInfinity));
        var h = new HingeJointDef { BodyA = a, BodyB = b, EnableLimit = true, LowerAngle = 1f, UpperAngle = -1f };
        Assert.Throws<InvalidJointException>(() => w.CreateJoint(in h));
        h.LowerAngle = -4f; h.UpperAngle = 0f;
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateJoint(in h));
        h = new HingeJointDef { BodyA = a, BodyB = b, LocalFrameA = new JointFrame3D(default, new QuaternionF(0, 0, 0, 0)) };
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateJoint(in h));
        h = new HingeJointDef { BodyA = a, BodyB = b, MaxMotorTorque = -1f };
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateJoint(in h));
        var pd = new PathJointDef { BodyA = a, BodyB = b, Points = [new Vector3F(0, 0, 0)] };
        Assert.Throws<InvalidJointException>(() => w.CreateJoint(in pd));
        pd.Points = null;
        Assert.Throws<InvalidJointException>(() => w.CreateJoint(in pd));
        pd.Points = [new Vector3F(0, 0, 0), new Vector3F(float.NaN, 0, 0)];
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateJoint(in pd));
        var gd = new GearJointDef { BodyA = a, BodyB = b, Ratio = 0f };
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateJoint(in gd));
        var dd = new DistanceJointDef { BodyA = a, BodyB = b, Length = 0f };
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateJoint(in dd));
        var pu = new PulleyJointDef { BodyA = a, BodyB = b, FixedPointA = new Vector3D(double.NaN, 0, 0) };
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateJoint(in pu));
        var six = new SixDofJointDef { BodyA = a, BodyB = b, LinearX = (JointAxisMode)9 };
        Assert.Throws<InvalidJointException>(() => w.CreateJoint(in six));
        var fx = new FixedJointDef { BodyA = a, BodyB = b, BreakForce = float.NaN };
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateJoint(in fx));
        Assert.Equal(0, w.JointCount); // nothing half-created
        var j = w.AddHinge(a, b, new Vector3D(0.5, 0, 0), new Vector3F(0, 0, 1));
        Assert.Throws<InvalidJointException>(() => j.SetLimits(2f, 1f));
        Assert.Throws<ArgumentOutOfRangeException>(() => j.SpringHertz = float.NaN);
        Assert.Throws<ArgumentOutOfRangeException>(() => j.MotorSpeed = float.PositiveInfinity);
    }

    // ------------------------------------------------------------------ islands, sleep, collisions

    [Fact]
    public void ABodyHangingOnAChainSleepsAndWakesWithTheWholeChain()
    {
        var w = World3DTests.NewWorld();
        World3DTests.Floor(w);
        var prev = w.AddBox(new Vector3F(0.2f, 0.2f, 0.2f), new Vector3D(0, 0.1, 0), type: BodyType.Static);
        var links = new List<Body>();
        for (int i = 0; i < 4; i++)
        {
            var l = w.AddBox(new Vector3F(0.5f, 0.2f, 0.2f), new Vector3D(0.5 + 0.5 * i, 0.1, 0.0));
            w.AddPoint(prev, l, new Vector3D(0.25 + 0.5 * i, 0.1, 0));
            links.Add(l); prev = l;
        }

        for (int i = 0; i < 600; i++) { w.Step(Dt); }
        Assert.All(links, l => Assert.False(l.IsAwake));
        links[3].Wake();
        Assert.All(links, l => Assert.True(l.IsAwake));
    }

    [Fact]
    public void BodiesJoinedWithoutCollisionIgnoreEachOtherAndCollideWhenTheJointGoes()
    {
        var w = NoGravity();
        var a = w.AddBox(new Vector3F(1f, 1f, 1f), new Vector3D(0, 0, 0));
        var b = w.AddBox(new Vector3F(1f, 1f, 1f), new Vector3D(0.5, 0, 0));
        var j = w.AddPoint(a, b, new Vector3D(0.25, 0, 0));
        var c = w.AddBox(new Vector3F(1f, 1f, 1f), new Vector3D(10, 0, 0));
        var d = w.AddBox(new Vector3F(1f, 1f, 1f), new Vector3D(10.5, 0, 0));
        w.AddPoint(c, d, new Vector3D(10.25, 0, 0), collideConnected: true);
        for (int i = 0; i < 10; i++) { w.Step(Dt); }
        Assert.False(HasContact(w, a, b));
        Assert.True(HasContact(w, c, d));
        j.Destroy();
        for (int i = 0; i < 10; i++) { w.Step(Dt); }
        Assert.True(HasContact(w, a, b));
    }

    private static bool HasContact(PhysicsWorld3D w, Body x, Body y)
    {
        var cs = w.Core.Constraints;
        for (int c = 0; c < cs.HighWater; c++)
        {
            if (!cs.IsLive(c) || cs.KindOf(c) != ConstraintKind.Contact) { continue; }
            var ba = w.Core.HandleOfBody(cs.BodyA[c]); var bb = w.Core.HandleOfBody(cs.BodyB[c]);
            if ((ba == x.RawHandle && bb == y.RawHandle) || (ba == y.RawHandle && bb == x.RawHandle)) { return true; }
        }

        return false;
    }

    // ------------------------------------------------------------------ determinism, snapshots

    /// <summary>A scene with one joint of every kind, hanging and swinging over a floor.</summary>
    internal static PhysicsWorld3D BuildJointScene(int threads, int id = 21)
    {
        var w = World3DTests.NewWorld(threads, id);
        World3DTests.Floor(w);
        var top = w.AddBox(new Vector3F(0.2f, 0.2f, 0.2f), new Vector3D(0, 6, 0), type: BodyType.Static);
        Body Box(double x, double y, double z) => w.AddBox(new Vector3F(0.4f, 0.4f, 0.4f), new Vector3D(x, y, z));
        var b1 = Box(1, 6, 0); var b2 = Box(2, 6, 0); var b3 = Box(3, 6, 0); var b4 = Box(4, 6, 0); var b5 = Box(5, 6, 0); var b6 = Box(6, 6, 0); var b7 = Box(7, 6, 0); var b8 = Box(8, 6, 0);
        w.AddHinge(top, b1, new Vector3D(0.5, 6, 0), new Vector3F(0, 0, 1)).SetLimits(-1f, 1f);
        w.AddCharacterJoint(b1, b2, new Vector3D(1.5, 6, 0), new Vector3F(1, 0, 0));
        w.AddCone(b2, b3, new Vector3D(2.5, 6, 0), new Vector3F(1, 0, 0), 0.6f);
        var six = w.AddConfigurable(b3, b4, new Vector3D(3.5, 6, 0));
        six.SetAxisMode(JointAxes.AngularZ, JointAxisMode.Free); six.SetAxisMode(JointAxes.LinearX, JointAxisMode.Limited); six.SetAxisLimits(JointAxes.LinearX, -0.1f, 0.1f);
        w.AddSlider(b4, b5, new Vector3D(4.5, 6, 0), new Vector3F(1, 0, 0)).SpringEnabled = true;
        w.AddSpring(b5, b6, new Vector3D(5, 6, 0), new Vector3D(6, 6, 0), 3f, 0.3f);
        w.AddDistance(b6, b7, new Vector3D(6.2, 6, 0), new Vector3D(6.8, 6, 0));
        w.AddFixed(b7, b8, new Vector3D(7.5, 6, 0));
        var p = w.AddSphere(0.3f, new Vector3D(-3, 4, 0));
        var q = w.AddSphere(0.3f, new Vector3D(-4, 4, 0));
        w.AddGear(p, q, new Vector3F(0, 0, 1), new Vector3F(0, 0, 1), 1.5f).Ratio = 1.5f;
        p.AngularVelocity = new Vector3F(0, 0, 2);
        var track = w.AddBox(new Vector3F(0.2f, 0.2f, 0.2f), new Vector3D(-8, 3, 0), type: BodyType.Static);
        var cart = w.AddBox(new Vector3F(0.3f, 0.3f, 0.3f), new Vector3D(-8, 3, 0));
        var path = w.AddPath(track, cart, [new Vector3D(-8, 3, 0), new Vector3D(-7, 3, 1), new Vector3D(-6, 3.5, 2), new Vector3D(-5, 3, 3)]);
        path.MotorSpeed = 1f; path.MaxMotorForce = 200f; path.MotorEnabled = true;
        var l = w.AddBox(new Vector3F(0.4f, 0.4f, 0.4f), new Vector3D(-1, 5, 3));
        var r = w.AddBox(new Vector3F(0.4f, 0.4f, 0.4f), new Vector3D(1, 4, 3), density: 3f);
        w.AddPulley(l, r, l.Position, r.Position, new Vector3D(-1, 8, 3), new Vector3D(1, 8, 3));
        return w;
    }

    [Fact]
    public void JointWorldsGiveTheSameHashesOnOneAndEightThreads()
    {
        var a = BuildJointScene(1); var b = BuildJointScene(8);
        for (int i = 0; i < 120; i++)
        {
            a.Step(1f / 60f); b.Step(1f / 60f);
            Assert.Equal(a.ComputeHash(), b.ComputeHash());
        }
    }

    [Fact]
    public void SnapshotAndRestoreWithJointsAreBitIdentical()
    {
        var w = BuildJointScene(1);
        for (int i = 0; i < 40; i++) { w.Step(1f / 60f); }
        byte[] snap = w.Snapshot();
        var hashes = new List<ulong>();
        for (int i = 0; i < 60; i++) { w.Step(1f / 60f); hashes.Add(w.ComputeHash()); }
        byte[] end = w.Snapshot();

        w.Restore(snap);
        Assert.Equal(snap, w.Snapshot());
        for (int i = 0; i < 60; i++) { w.Step(1f / 60f); Assert.Equal(hashes[i], w.ComputeHash()); }
        Assert.Equal(end, w.Snapshot());

        var fresh = new PhysicsWorld3D(new WorldOptions3D { WorldId = 21 });
        fresh.Restore(snap);
        Assert.Equal(snap, fresh.Snapshot());
        for (int i = 0; i < 60; i++) { fresh.Step(1f / 60f); Assert.Equal(hashes[i], fresh.ComputeHash()); }
        Assert.True(fresh.JointCount > 0);
    }

    [Fact]
    public void JointlessWorldsKeepTheirSnapshotVersion()
    {
        var w = World3DTests.BuildScene(1);
        byte[] before = w.Snapshot();
        var a = Anchor(w, 0, 30, 0); var b = w.AddSphere(0.3f, new Vector3D(1, 30, 0));
        var j = w.AddPoint(a, b, new Vector3D(0.5, 30, 0));
        Assert.NotEqual(before.Length, w.Snapshot().Length);
        j.Destroy(); b.Destroy(); a.Destroy();
        // A world that had joints and lost them again writes no joints block.
        var w2 = World3DTests.BuildScene(1);
        Assert.Equal(before, w2.Snapshot());
    }

    [Fact]
    public void RestoringAJointlessSnapshotIntoAWorldWithJointsRemovesThem()
    {
        var plain = World3DTests.NewWorld(1, 21);
        Anchor(plain);
        byte[] snap = plain.Snapshot();
        var w = BuildJointScene(1);
        w.Step(1f / 60f);
        w.Restore(snap);
        Assert.Equal(0, w.JointCount);
        Assert.Equal(snap, w.Snapshot());
    }

    [Fact]
    public void CorruptedJointSnapshotsAreRejectedWithoutChangingTheWorld()
    {
        var w = BuildJointScene(1);
        for (int i = 0; i < 5; i++) { w.Step(1f / 60f); }
        byte[] snap = w.Snapshot();
        ulong h = w.ComputeHash();
        var rng = new Random(5);
        int rejected = 0;
        for (int t = 0; t < 200; t++)
        {
            var bad = (byte[])snap.Clone();
            int at = snap.Length - 1 - rng.Next(Math.Min(snap.Length - 1, 4000));
            bad[at] ^= (byte)(1 << rng.Next(8));
            try { w.Restore(bad); w.Restore(snap); }
            catch (SnapshotFormatException) { rejected++; Assert.Equal(h, w.ComputeHash()); }
        }

        Assert.True(rejected > 0);
        Assert.Throws<SnapshotFormatException>(() => w.Restore(snap.AsSpan(0, snap.Length - 7).ToArray()));
        Assert.Equal(h, w.ComputeHash());
    }

    [Fact]
    public void ShiftingTheOriginKeepsAPulleyWorking()
    {
        var a = BuildJointScene(1); var b = BuildJointScene(1);
        for (int i = 0; i < 20; i++) { a.Step(1f / 60f); b.Step(1f / 60f); }
        b.ShiftOrigin(new Vector3D(64, 0, -32));
        for (int i = 0; i < 40; i++) { a.Step(1f / 60f); b.Step(1f / 60f); }
        // The shifted world is the same world in different coordinates: pulley bodies are at the same place once the origin is added back.
        Assert.Equal(a.JointCount, b.JointCount);
        for (int c = 0; c < a.Core.Constraints.HighWater; c++)
        {
            if (!a.Core.Constraints.IsLive(c) || a.Core.Constraints.KindOf(c) != ConstraintKind.Joint) { continue; }
            var ja = a.GetJoint(a.Core.HandleOfJoint(c)); var jb = b.GetJoint(b.Core.HandleOfJoint(c));
            if (ja.Type != JointType3D.Pulley) { continue; }
            var pa = ja.BodyA.Position; var pb = jb.BodyA.Position;
            Assert.True(Math.Abs(pa.Y - pb.Y) < 0.05 && Math.Abs(pa.X - (pb.X + 64)) < 0.05, "pulley body drifted: " + pa + " vs " + pb);
        }
    }
}
