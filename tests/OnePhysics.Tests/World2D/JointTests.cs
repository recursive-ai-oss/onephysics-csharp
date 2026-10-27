using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

public class JointTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld2D NewWorld(bool gravity = true, int threads = 1, int id = 11)
        => new(new WorldOptions2D { Gravity = gravity ? new Vector2F(0f, -9.81f) : default, SleepEnabled = false, WorldId = id, ThreadCount = threads });

    private static Body Anchor(PhysicsWorld2D w, double x, double y) => w.AddCircle(0.1f, new Vector2D(x, y), BodyType.Static);

    /// <summary>Period and damping ratio of a decaying oscillation from its first peaks (parabolic interpolation between samples).</summary>
    private static (double Period, double Zeta) Oscillation(PhysicsWorld2D w, Func<double> sample, int steps)
    {
        var s = new double[steps];
        for (int i = 0; i < steps; i++) { w.Step(Dt); s[i] = sample(); }
        var t = new List<double>(); var v = new List<double>();
        for (int i = 1; i < steps - 1; i++)
        {
            if (s[i] > 0 && s[i - 1] < s[i] && s[i] >= s[i + 1])
            {
                double den = s[i - 1] - 2 * s[i] + s[i + 1];
                double off = 0.5 * (s[i - 1] - s[i + 1]) / den;
                t.Add((i + off) * Dt); v.Add(s[i] - 0.25 * (s[i - 1] - s[i + 1]) * off);
            }
        }

        Assert.True(t.Count >= 4, "fewer than four peaks: " + t.Count);
        int n = 3;
        double period = (t[n] - t[0]) / n;
        double delta = Math.Log(v[0] / v[n]) / n;
        return (period, delta / Math.Sqrt(4 * Math.PI * Math.PI + delta * delta));
    }

    [Fact]
    public void PendulumPeriodMatchesTheAnalyticAnswer()
    {
        var w = NewWorld();
        var pivot = Anchor(w, 0, 0);
        double L = 2.0, r = 0.05, theta0 = 0.05;
        var bob = w.AddCircle((float)r, new Vector2D(L * Math.Sin(theta0), -L * Math.Cos(theta0)));
        w.AddHinge(pivot, bob, new Vector2D(0, 0));
        var cross = new List<double>();
        double prev = theta0;
        for (int i = 1; i <= 600; i++)
        {
            w.Step(Dt);
            var p = bob.Position;
            double th = Math.Atan2(p.X, -p.Y);
            if (prev > 0 && th <= 0) { cross.Add((i - 1 + prev / (prev - th)) * Dt); }
            prev = th;
        }

        Assert.True(cross.Count >= 4);
        double period = (cross[3] - cross[0]) / 3;
        double expected = 2 * Math.PI * Math.Sqrt((L * L + 0.5 * r * r) / (9.81 * L));
        Assert.InRange(period / expected, 0.995, 1.005);
    }

    [Fact]
    public void RevoluteLimitHoldsUnderLoad()
    {
        var w = NewWorld();
        var pivot = Anchor(w, 0, 0);
        var rod = w.AddBox(new Vector2F(2f, 0.1f), new Vector2D(1, 0), density: 5f);
        var j = w.AddHinge(pivot, rod, new Vector2D(0, 0));
        j.SetLimits(-0.5f, 0.5f);
        j.LimitEnabled = true;
        double min = 0;
        for (int i = 0; i < 480; i++) { w.Step(Dt); min = Math.Min(min, rod.Angle); }
        Assert.InRange(min, -0.5 - 0.02, -0.45); // dropped to the lower limit, overshoot at most 0.02 rad
        Assert.InRange(rod.Angle, -0.5 - 0.005, -0.495);
    }

    [Fact]
    public void PrismaticLimitHoldsUnderLoad()
    {
        var w = NewWorld();
        var rail = Anchor(w, 0, 0);
        var block = w.AddBox(new Vector2F(1f, 1f), new Vector2D(0, 0), density: 10f);
        var j = w.AddSlider(rail, block, new Vector2D(0, 0), new Vector2F(0, 1));
        j.SetLimits(-1f, 1f);
        j.LimitEnabled = true;
        double min = 0;
        for (int i = 0; i < 300; i++) { w.Step(Dt); min = Math.Min(min, block.Position.Y); }
        Assert.InRange(min, -1.01, -0.99);
        Assert.InRange(Math.Abs(block.Position.X), 0, 0.001);
        Assert.InRange(block.Angle, -0.001f, 0.001f);
    }

    [Fact]
    public void RevoluteMotorReachesItsSpeedAndRespectsTheTorqueCap()
    {
        // A disk of radius 1: inertia = m r^2 / 2.
        var w = NewWorld(false);
        var wall = Anchor(w, 0, 0);
        var disk = w.AddCircle(1f, new Vector2D(0, 0));
        var j = w.AddHinge(wall, disk, new Vector2D(0, 0));
        j.MotorEnabled = true; j.MotorSpeed = 3f; j.MaxMotorForce = 1000f;
        for (int i = 0; i < 60; i++) { w.Step(Dt); }
        Assert.InRange(disk.AngularVelocity, 2.97f, 3.03f);

        var w2 = NewWorld(false);
        var wall2 = Anchor(w2, 0, 0);
        var disk2 = w2.AddCircle(1f, new Vector2D(0, 0));
        var j2 = w2.AddHinge(wall2, disk2, new Vector2D(0, 0));
        j2.MotorEnabled = true; j2.MotorSpeed = 100f; j2.MaxMotorForce = 0.5f;
        for (int i = 0; i < 60; i++) { w2.Step(Dt); }
        double expected = 0.5 / disk2.Inertia * 1.0; // torque / inertia after one second
        Assert.InRange(disk2.AngularVelocity / expected, 0.98, 1.02);
    }

    [Fact]
    public void PrismaticMotorReachesItsSpeedAndRespectsTheForceCap()
    {
        var w = NewWorld(false);
        var rail = Anchor(w, 0, 0);
        var block = w.AddBox(new Vector2F(1f, 1f), new Vector2D(0, 0));
        var j = w.AddSlider(rail, block, new Vector2D(0, 0), new Vector2F(1, 0));
        j.MotorEnabled = true; j.MotorSpeed = 2f; j.MaxMotorForce = 1000f;
        for (int i = 0; i < 60; i++) { w.Step(Dt); }
        Assert.InRange(block.LinearVelocity.X, 1.98f, 2.02f);

        var w2 = NewWorld(false);
        var rail2 = Anchor(w2, 0, 0);
        var block2 = w2.AddBox(new Vector2F(1f, 1f), new Vector2D(0, 0));
        var j2 = w2.AddSlider(rail2, block2, new Vector2D(0, 0), new Vector2F(1, 0));
        j2.MotorEnabled = true; j2.MotorSpeed = 100f; j2.MaxMotorForce = 0.5f;
        for (int i = 0; i < 60; i++) { w2.Step(Dt); }
        Assert.InRange(block2.LinearVelocity.X / (0.5 / block2.Mass), 0.98, 1.02);
    }

    [Theory]
    [InlineData(2f, 0.2f)]
    [InlineData(1f, 0.5f)]
    [InlineData(4f, 0.05f)]
    public void DistanceSpringHasTheRequestedFrequencyAndDamping(float hertz, float zeta)
    {
        var w = NewWorld(false);
        var anchor = Anchor(w, 0, 0);
        var mass = w.AddCircle(0.25f, new Vector2D(2, 0));
        var j = w.AddSpring(anchor, mass, new Vector2D(0, 0), new Vector2D(2, 0), hertz, zeta);
        mass.SetTransform(new Vector2D(2.2, 0), 0f);
        var (period, z) = Oscillation(w, () => mass.Position.X - 2.0, 600);
        double expected = 1.0 / (hertz * Math.Sqrt(1 - zeta * zeta));
        Assert.InRange(period / expected, 0.98, 1.02);
        Assert.InRange(z / zeta, 0.98, 1.02);
        Assert.Equal(JointType2D.Distance, j.Type);
    }

    [Theory]
    [InlineData(2f, 0.3f)]
    [InlineData(1f, 0.7f)]
    public void WheelSuspensionHasTheRequestedFrequencyAndDamping(float hertz, float zeta)
    {
        var w = NewWorld(false);
        var chassis = w.AddBox(new Vector2F(2f, 0.5f), new Vector2D(0, 0), type: BodyType.Static);
        var wheel = w.AddCircle(0.4f, new Vector2D(0, 0));
        var j = w.AddWheel(chassis, wheel, new Vector2D(0, 0), new Vector2F(0, 1));
        j.SpringHertz = hertz; j.SpringDampingRatio = zeta;
        wheel.SetTransform(new Vector2D(0, 0.1), 0f);
        var (period, z) = Oscillation(w, () => wheel.Position.Y, 700);
        double expected = 1.0 / (hertz * Math.Sqrt(1 - zeta * zeta));
        Assert.InRange(period / expected, 0.98, 1.02);
        Assert.InRange(z / zeta, 0.98, 1.02);
        Assert.InRange(Math.Abs(wheel.Position.X), 0, 0.001);
    }

    [Fact]
    public void WeldHoldsACantileverWithinAStatedSag()
    {
        var w = NewWorld();
        var wall = w.AddBox(new Vector2F(0.2f, 2f), new Vector2D(0, 0), type: BodyType.Static);
        var beam = w.AddBox(new Vector2F(4f, 0.2f), new Vector2D(2.1, 0), density: 5f); // 4 m long, 0.2 m thick, 4 kg
        w.AddFixed(wall, beam, new Vector2D(0.1, 0));
        for (int i = 0; i < 300; i++) { w.Step(Dt); }
        Assert.InRange(beam.Position.Y, -0.03, 0.0);  // stated sag: at most 3 cm at the beam's centre
        Assert.InRange(beam.Angle, -0.015f, 0.0f);
    }

    [Fact]
    public void ASpringyWeldSagsMoreThanARigidOne()
    {
        double Sag(float hertz)
        {
            var w = NewWorld();
            var wall = w.AddBox(new Vector2F(0.2f, 2f), new Vector2D(0, 0), type: BodyType.Static);
            var beam = w.AddBox(new Vector2F(2f, 0.2f), new Vector2D(1.1, 0));
            var j = w.AddFixed(wall, beam, new Vector2D(0.1, 0));
            if (hertz > 0) { j.SetWeldSprings(0f, 0f, hertz, 0.5f); }
            for (int i = 0; i < 300; i++) { w.Step(Dt); }
            return -beam.Position.Y;
        }

        Assert.True(Sag(2f) > 10 * Sag(0f));
    }

    [Fact]
    public void AMouseJointPullsTheBodyToItsTarget()
    {
        var w = NewWorld(false);
        var ground = Anchor(w, 0, -50);
        var box = w.AddBox(new Vector2F(1f, 1f), new Vector2D(0, 0));
        var j = w.AddTarget(ground, box, new Vector2D(0, 0), hertz: 5f, dampingRatio: 1f);
        j.Target = new Vector2D(3, 2);
        for (int i = 0; i < 240; i++) { w.Step(Dt); }
        var p = box.Position;
        Assert.InRange(p.X, 3 - 0.01, 3 + 0.01);
        Assert.InRange(p.Y, 2 - 0.01, 2 + 0.01);
    }

    [Fact]
    public void AMouseJointRespectsItsForceCap()
    {
        var w = NewWorld(false);
        var ground = Anchor(w, 0, -50);
        var box = w.AddBox(new Vector2F(1f, 1f), new Vector2D(0, 0));
        var j = w.AddTarget(ground, box, new Vector2D(0, 0), 5f, 0.7f, maxForce: 2f);
        j.Target = new Vector2D(100, 0);
        for (int i = 0; i < 60; i++) { w.Step(Dt); }
        Assert.InRange(box.LinearVelocity.X / (2.0 / box.Mass), 0.98, 1.02); // a = F / m for one second
    }

    [Fact]
    public void AMotorJointTracksItsOffset()
    {
        var w = NewWorld(false);
        var a = w.AddCircle(0.2f, new Vector2D(0, 0), BodyType.Static);
        var b = w.AddBox(new Vector2F(1f, 0.5f), new Vector2D(3, 0));
        var j = w.AddRelative(a, b, 1000f, 1000f);
        j.SetOffset(new Vector2F(1f, 0.5f), 0.5f);
        for (int i = 0; i < 180; i++) { w.Step(Dt); }
        var p = b.Position;
        Assert.InRange(p.X, 1 - 0.01, 1 + 0.01);
        Assert.InRange(p.Y, 0.5 - 0.01, 0.5 + 0.01);
        Assert.InRange(b.Angle, 0.5f - 0.01f, 0.5f + 0.01f);
    }

    [Fact]
    public void BreakageFiresExactlyOnceInADeterministicOrder()
    {
        var w = NewWorld();
        var ceiling = w.AddBox(new Vector2F(10f, 0.5f), new Vector2D(0, 10), type: BodyType.Static);
        var weights = new List<Body>(); var joints = new List<Joint>();
        for (int i = 0; i < 3; i++)
        {
            var box = w.AddBox(new Vector2F(1f, 1f), new Vector2D(-4 + 4 * i, 8), density: 20f);
            var j = w.AddHinge(ceiling, box, new Vector2D(-4 + 4 * i, 9));
            if (i != 1) { j.BreakForce = 100f; } // the middle joint holds
            weights.Add(box); joints.Add(j);
        }

        var seen = new List<(int Index, float Value)>();
        foreach (var j in joints) { var jj = j; jj.OnJointBreak += b => seen.Add((b.Joint.RawHandle.Index, b.Value)); }
        var order = new List<int>();
        for (int i = 0; i < 120; i++)
        {
            w.Step(Dt);
            foreach (var e in w.Events) { if (e.Kind == EventKind.JointBreak) { order.Add(e.Joint.Index); } }
        }

        Assert.Equal(2, seen.Count);
        Assert.Equal(new[] { joints[0].RawHandle.Index, joints[2].RawHandle.Index }, order);
        Assert.All(seen, s => Assert.True(s.Value > 100f));
        Assert.False(joints[0].IsValid);
        Assert.True(joints[1].IsValid);
        Assert.Throws<StaleHandleException>(() => joints[0].Type);
        Assert.Equal(1, w.JointCount);
    }

    [Fact]
    public void StaleAndDefaultHandlesFailClearly()
    {
        var w = NewWorld();
        var a = w.AddBox(new Vector2F(1f, 1f), new Vector2D(0, 5));
        var b = w.AddBox(new Vector2F(1f, 1f), new Vector2D(2, 5));
        var j = w.AddHinge(a, b, new Vector2D(1, 5));
        var copy = j;
        j.Destroy();
        Assert.False(copy.IsValid);
        var e = Assert.Throws<StaleHandleException>(() => copy.Type);
        Assert.Contains("Joint", e.Message);
        Assert.Throws<StaleHandleException>(() => copy.Destroy());
        Assert.Throws<InvalidHandleException>(() => default(Joint).Type);

        var j2 = w.AddHinge(a, b, new Vector2D(1, 5));
        b.Destroy();
        Assert.Throws<StaleHandleException>(() => j2.BodyA);
        Assert.Equal(0, w.JointCount);
    }

    [Fact]
    public void WrongKindOfPropertyIsRefused()
    {
        var w = NewWorld();
        var a = w.AddBox(new Vector2F(1f, 1f), new Vector2D(0, 5), type: BodyType.Static);
        var b = w.AddBox(new Vector2F(1f, 1f), new Vector2D(2, 5));
        var weld = w.AddFixed(a, b, new Vector2D(1, 5));
        Assert.Throws<InvalidJointException>(() => weld.LimitEnabled = true);
        Assert.Throws<InvalidJointException>(() => weld.Target);
    }

    [Fact]
    public void HostileDefinitionsGiveTypedErrors()
    {
        var w = NewWorld();
        var a = w.AddBox(new Vector2F(1f, 1f), new Vector2D(0, 5));
        var b = w.AddBox(new Vector2F(1f, 1f), new Vector2D(2, 5));
        var s1 = w.AddBox(new Vector2F(1f, 1f), new Vector2D(0, 0), type: BodyType.Static);
        var s2 = w.AddBox(new Vector2F(1f, 1f), new Vector2D(5, 0), type: BodyType.Static);
        var other = NewWorld(true, 1, 99).AddBox(new Vector2F(1f, 1f), new Vector2D(0, 0));

        var same = Assert.Throws<InvalidJointException>(() => w.AddHinge(a, a, new Vector2D(0, 5)));
        Assert.Contains("two different bodies", same.Message);
        Assert.Throws<InvalidJointException>(() => w.AddHinge(s1, s2, new Vector2D(0, 0)));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddHinge(a, b, new Vector2D(double.NaN, 0)));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddHinge(a, b, new Vector2D(3e6, 0)));
        Assert.Throws<InvalidJointException>(() => w.AddSlider(a, b, new Vector2D(1, 5), new Vector2F(0, 0)));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddSlider(a, b, new Vector2D(1, 5), new Vector2F(float.NaN, 1)));
        Assert.Throws<ForeignHandleException>(() => w.AddHinge(a, other, new Vector2D(0, 0)));
        Assert.Throws<InvalidHandleException>(() => w.AddHinge(a, default, new Vector2D(0, 0)));

        var bad = new RevoluteJointDef { BodyA = a, BodyB = b, EnableLimit = true, LowerAngle = 1f, UpperAngle = -1f };
        Assert.Throws<InvalidJointException>(() => w.CreateJoint(in bad));
        bad = new RevoluteJointDef { BodyA = a, BodyB = b, EnableLimit = true, LowerAngle = 0f, UpperAngle = 7f };
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateJoint(in bad));
        bad = new RevoluteJointDef { BodyA = a, BodyB = b, EnableMotor = true, MotorSpeed = float.PositiveInfinity };
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateJoint(in bad));
        bad = new RevoluteJointDef { BodyA = a, BodyB = b, MaxMotorTorque = -1f };
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateJoint(in bad));
        bad = new RevoluteJointDef { BodyA = a, BodyB = b, BreakForce = float.NaN };
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateJoint(in bad));
        var dist = new DistanceJointDef { BodyA = a, BodyB = b, Length = 0f };
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateJoint(in dist));
        dist = new DistanceJointDef { BodyA = a, BodyB = b, EnableLimit = true, MinLength = 3f, MaxLength = 2f };
        Assert.Throws<InvalidJointException>(() => w.CreateJoint(in dist));
        var motor = new MotorJointDef { BodyA = a, BodyB = b, CorrectionFactor = 2f };
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateJoint(in motor));
        var mouse = new MouseJointDef { BodyA = s1, BodyB = b, Target = new Vector2D(double.PositiveInfinity, 0) };
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateJoint(in mouse));

        // Edits are validated too, and a refused edit changes nothing.
        var j = w.AddHinge(a, b, new Vector2D(1, 5));
        Assert.Throws<InvalidJointException>(() => j.SetLimits(1f, -1f));
        Assert.Throws<ArgumentOutOfRangeException>(() => j.MaxMotorForce = float.NaN);
        Assert.Throws<ArgumentOutOfRangeException>(() => j.SpringHertz = -1f);
        Assert.Equal(1, w.JointCount);
        var hash = w.ComputeHash();
        Assert.Throws<ArgumentOutOfRangeException>(() => j.MotorSpeed = float.NaN);
        Assert.Equal(hash, w.ComputeHash());
    }

    [Fact]
    public void JoinedBodiesDoNotCollideUnlessAsked()
    {
        int Contacts(PhysicsWorld2D w)
        {
            int n = 0;
            var cs = w.Core.Constraints;
            for (int c = 0; c < cs.HighWater; c++) { if (cs.IsLive(c) && cs.KindOf(c) == ConstraintKind.Contact && cs.PointCount[c] > 0) { n++; } }
            return n;
        }

        var w1 = NewWorld(false);
        var a1 = w1.AddBox(new Vector2F(1f, 1f), new Vector2D(0, 0));
        var b1 = w1.AddBox(new Vector2F(1f, 1f), new Vector2D(0.5, 0));
        w1.AddFixed(a1, b1, new Vector2D(0.25, 0));
        for (int i = 0; i < 10; i++) { w1.Step(Dt); }
        Assert.Equal(0, Contacts(w1));

        var w2 = NewWorld(false);
        var a2 = w2.AddBox(new Vector2F(1f, 1f), new Vector2D(0, 0));
        var b2 = w2.AddBox(new Vector2F(1f, 1f), new Vector2D(0.5, 0));
        w2.AddHinge(a2, b2, new Vector2D(0.25, 0), collideConnected: true);
        for (int i = 0; i < 2; i++) { w2.Step(Dt); }
        Assert.Equal(1, Contacts(w2));

        // A contact that exists when the joint is made is dropped, and comes back when the joint goes.
        var w3 = NewWorld(false);
        var a3 = w3.AddBox(new Vector2F(1f, 1f), new Vector2D(0, 0));
        var b3 = w3.AddBox(new Vector2F(1f, 1f), new Vector2D(0.5, 0));
        w3.Step(Dt);
        Assert.Equal(1, Contacts(w3));
        var j3 = w3.AddFixed(a3, b3, new Vector2D(0.25, 0));
        Assert.Equal(0, Contacts(w3));
        for (int i = 0; i < 3; i++) { w3.Step(Dt); }
        Assert.Equal(0, Contacts(w3));
        j3.Destroy();
        for (int i = 0; i < 3; i++) { w3.Step(Dt); }
        Assert.Equal(1, Contacts(w3));
    }

    [Fact]
    public void JointedBodiesSleepTogetherAndWakeTogether()
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = 12 });
        w.AddBox(new Vector2F(20f, 1f), new Vector2D(0, -0.5), type: BodyType.Static);
        var a = w.AddBox(new Vector2F(1f, 1f), new Vector2D(-1, 0.5));
        var b = w.AddBox(new Vector2F(1f, 1f), new Vector2D(1, 0.5));
        w.AddDistance(a, b, new Vector2D(-1, 0.5), new Vector2D(1, 0.5));
        for (int i = 0; i < 300; i++) { w.Step(Dt); }
        Assert.False(a.IsAwake);
        Assert.False(b.IsAwake);
        a.Wake();
        Assert.True(b.IsAwake);
    }

    /// <summary>A scene with every joint type, limits, motors, springs and one joint that breaks.</summary>
    internal static PhysicsWorld2D BuildJointScene(int threads, int id = 21, int kernelWidth = 8)
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = id, ThreadCount = threads, SleepEnabled = false });
        w.Engine.KernelWidth = kernelWidth;
        var ground = w.AddBox(new Vector2F(60f, 1f), new Vector2D(0, -0.5), type: BodyType.Static);
        var top = w.AddBox(new Vector2F(40f, 0.5f), new Vector2D(0, 12), type: BodyType.Static);

        // A chain of hinges with a limit on every third link and a motor on the first.
        var prev = top;
        for (int i = 0; i < 8; i++)
        {
            var link = w.AddBox(new Vector2F(1f, 0.2f), new Vector2D(-14 + 0.5 + i, 11.5), density: 1f + i);
            var j = w.AddHinge(prev, link, new Vector2D(-14 + i, 11.5));
            if (i % 3 == 2) { j.SetLimits(-0.4f, 0.4f); j.LimitEnabled = true; }
            if (i == 0) { j.MotorEnabled = true; j.MotorSpeed = 0.5f; j.MaxMotorForce = 50f; }
            prev = link;
        }

        // A slider with a motor and a limit, a wheel on a body, a springy distance joint, a weld beam, a mouse and a relative motor.
        var rail = w.AddBox(new Vector2F(0.2f, 0.2f), new Vector2D(4, 8), type: BodyType.Static);
        var slide = w.AddBox(new Vector2F(1f, 1f), new Vector2D(4, 8));
        var s = w.AddSlider(rail, slide, new Vector2D(4, 8), new Vector2F(1, 0));
        s.SetLimits(-3f, 3f); s.LimitEnabled = true; s.MotorEnabled = true; s.MotorSpeed = 1f; s.MaxMotorForce = 30f;

        var car = w.AddBox(new Vector2F(3f, 0.5f), new Vector2D(-6, 2), density: 2f);
        var wheel = w.AddCircle(0.5f, new Vector2D(-5, 1), density: 1f, material: new Material2D(1.0f));
        var wj = w.AddWheel(car, wheel, new Vector2D(-5, 1), new Vector2F(0, 1));
        wj.SpringHertz = 3f; wj.MotorEnabled = true; wj.MotorSpeed = -5f; wj.MaxMotorForce = 20f;
        wj.SetLimits(-0.3f, 0.3f); wj.LimitEnabled = true;

        var bob1 = w.AddCircle(0.4f, new Vector2D(8, 9));
        var bob2 = w.AddCircle(0.4f, new Vector2D(10, 9));
        var sp = w.AddSpring(top, bob1, new Vector2D(8, 11.75), new Vector2D(8, 9), 2f, 0.3f);
        sp.SetLimits(1f, 4f); sp.LimitEnabled = true;
        w.AddDistance(bob1, bob2, new Vector2D(8, 9), new Vector2D(10, 9));

        var wall = w.AddBox(new Vector2F(0.4f, 3f), new Vector2D(14, 3), type: BodyType.Static);
        var beam = w.AddBox(new Vector2F(3f, 0.3f), new Vector2D(15.8, 4), density: 3f);
        var weld = w.AddFixed(wall, beam, new Vector2D(14.2, 4));
        weld.BreakForce = 250f;
        var tip = w.AddBox(new Vector2F(1f, 1f), new Vector2D(17, 5), density: 40f);
        w.AddHinge(beam, tip, new Vector2D(17, 4.3));

        var dragged = w.AddBox(new Vector2F(1f, 1f), new Vector2D(-14, 3));
        var mouse = w.AddTarget(ground, dragged, new Vector2D(-14, 3));
        mouse.Target = new Vector2D(-12, 5);
        var follower = w.AddBox(new Vector2F(1f, 0.5f), new Vector2D(-11, 1));
        var rel = w.AddRelative(dragged, follower, 200f, 100f);
        rel.SetOffset(new Vector2F(1.5f, -1f), 0.3f);
        return w;
    }

    [Fact]
    public void JointSceneSnapshotRestoreIsBitIdentical()
    {
        var w = BuildJointScene(1);
        for (int i = 0; i < 1; i++) { w.Step(Dt); } // before the weld breaks, so the break is inside the replayed span
        var snap = w.Snapshot();
        var hashes = new List<ulong>(); var events = new List<string>();
        for (int i = 0; i < 150; i++) { w.Step(Dt); hashes.Add(w.ComputeHash()); foreach (var e in w.Events) { events.Add(e.Kind + ":" + e.BodyA.Index + ":" + e.Joint.Index); } }
        var final = w.Snapshot();
        Assert.True(w.JointCount < 16, "the weld should have broken"); // the scene has 16 joints
        Assert.Contains(events, e => e.StartsWith("JointBreak", StringComparison.Ordinal));

        // Back in the same world.
        w.Restore(snap);
        var again = new List<ulong>(); var events2 = new List<string>();
        for (int i = 0; i < 150; i++) { w.Step(Dt); again.Add(w.ComputeHash()); foreach (var e in w.Events) { events2.Add(e.Kind + ":" + e.BodyA.Index + ":" + e.Joint.Index); } }
        Assert.Equal(hashes, again);
        Assert.Equal(events, events2);
        Assert.True(final.AsSpan().SequenceEqual(w.Snapshot()));

        // Into a fresh world.
        var fresh = BuildJointScene(1);
        fresh.Restore(snap);
        var third = new List<ulong>();
        for (int i = 0; i < 150; i++) { fresh.Step(Dt); third.Add(fresh.ComputeHash()); }
        Assert.Equal(hashes, third);
        Assert.True(final.AsSpan().SequenceEqual(fresh.Snapshot()));
    }

    [Fact]
    public void JointSceneIsIdenticalAcrossThreadsAndKernelWidths()
    {
        var a = BuildJointScene(1);
        var b = BuildJointScene(4);
        var c = BuildJointScene(1, kernelWidth: 1);
        var d = BuildJointScene(3, kernelWidth: 4);
        for (int i = 0; i < 120; i++)
        {
            a.Step(Dt); b.Step(Dt); c.Step(Dt); d.Step(Dt);
            ulong h = a.ComputeHash();
            Assert.Equal(h, b.ComputeHash());
            Assert.Equal(h, c.ComputeHash());
            Assert.Equal(h, d.ComputeHash());
        }
    }

    [Fact]
    public void ARestoredSnapshotWithoutJointsStaysVersionOne()
    {
        var w = World2DTests.BuildScene(1, 5);
        for (int i = 0; i < 5; i++) { w.Step(Dt); }
        var snap = w.Snapshot();
        var w2 = World2DTests.BuildScene(1, 5);
        w2.Restore(snap);
        Assert.Equal(w.ComputeHash(), w2.ComputeHash());
        // Adding a joint then destroying it brings the section back to its jointless form.
        var a = w2.AddBox(new Vector2F(1f, 1f), new Vector2D(30, 30));
        var b = w2.AddBox(new Vector2F(1f, 1f), new Vector2D(32, 30));
        var j = w2.AddHinge(a, b, new Vector2D(31, 30));
        j.Destroy();
        a.Destroy(); b.Destroy();
        Assert.Equal(0, w2.JointCount);
    }
}
