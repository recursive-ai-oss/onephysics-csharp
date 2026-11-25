using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Diagnosis;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Diagnosis;

/// <summary>Every finding has a scene that triggers it and one that does not; the live form never changes the world.</summary>
public class Diagnosis3DTests
{
    private const float Dt = 1f / 60f;
    private static readonly DiagnoseOptions Long = new() { ProbeSteps = 150 };

    private static PhysicsWorld3D New(bool gravity = true, bool ccd = true) => new(new WorldOptions3D { Gravity = gravity ? new Vector3F(0, -9.81f, 0) : default, WorldId = 41, ContinuousEnabled = ccd });
    private static Body Floor(PhysicsWorld3D w) => w.AddBox(new Vector3F(100, 1, 100), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
    private static JointFrame3D At(float x, float y = 0, float z = 0) => new(new Vector3F(x, y, z), QuaternionF.Identity);

    private static bool Has(DiagnosisReport r, DiagnosticCode code) => r.Has(code);
    private static void AssertClean(DiagnosisReport r, DiagnosticCode code) => Assert.False(r.Has(code), "unexpected " + code + ": " + string.Join("\n", r.With(code)));

    // ---------------------------------------------------------------- settings and state

    [Fact]
    public void GravityInCentimetresIsFlagged()
    {
        var w = New(); w.Options.Gravity = new Vector3F(0, -981f, 0);
        w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 1, 0));
        var f = Assert.Single(w.Diagnose(new DiagnoseOptions { ProbeSteps = 0 }).With(DiagnosticCode.ImplausibleGravity));
        Assert.Equal(DiagnosticSeverity.Warning, f.Severity);
        AssertClean(New().Diagnose(), DiagnosticCode.ImplausibleGravity);
    }

    [Fact]
    public void ThinAndHugeBodiesAreFlagged()
    {
        var w = New(); Floor(w);
        var thin = w.AddBox(new Vector3F(1, 0.005f, 1), at: new Vector3D(0, 3, 0));
        var huge = w.AddBox(new Vector3F(1300, 1300, 1300), at: new Vector3D(0, 3000, 0), density: 1e-6f);
        var r = w.Diagnose(new DiagnoseOptions { ProbeSteps = 0 });
        var sizes = r.With(DiagnosticCode.ImplausibleSize).ToList();
        Assert.Contains(sizes, f => f.Severity == DiagnosticSeverity.Warning && f.Bodies[0] == thin.RawHandle);
        Assert.Contains(sizes, f => f.Severity == DiagnosticSeverity.Info && f.Bodies[0] == huge.RawHandle);
        var ok = New(); ok.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 3, 0));
        AssertClean(ok.Diagnose(new DiagnoseOptions { ProbeSteps = 0 }), DiagnosticCode.ImplausibleSize);
    }

    [Fact]
    public void SpeedNearTheLimitIsFlagged()
    {
        var w = New(false);
        var fast = w.AddSphere(0.5f, new Vector3D(0, 0, 0)); fast.LinearVelocity = new Vector3F(250, 0, 0);
        var slow = w.AddSphere(0.5f, new Vector3D(0, 5, 0)); slow.LinearVelocity = new Vector3F(5, 0, 0);
        var r = w.Diagnose(new DiagnoseOptions { ProbeSteps = 0 });
        var f = Assert.Single(r.With(DiagnosticCode.ImplausibleSpeed));
        Assert.Equal(fast.RawHandle, f.Bodies[0]);
        Assert.Equal(DiagnosticSeverity.Warning, f.Severity);
    }

    [Theory]
    [InlineData(2.0, null)]
    [InlineData(80.0, DiagnosticSeverity.Info)]
    [InlineData(800.0, DiagnosticSeverity.Warning)]
    [InlineData(3000.0, DiagnosticSeverity.Error)]
    public void JointMassRatioHasSeverityBands(double ratio, DiagnosticSeverity? expected)
    {
        var w = New(false);
        var a = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 0, 0));
        var b = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(1, 0, 0), density: (float)ratio);
        var j = new PointJointDef { BodyA = a, BodyB = b, LocalFrameA = At(0.5f), LocalFrameB = At(-0.5f) };
        w.CreateJoint(in j);
        var found = w.Diagnose(new DiagnoseOptions { ProbeSteps = 0 }).With(DiagnosticCode.JointMassRatio).ToList();
        if (expected == null) { Assert.Empty(found); return; }
        var f = Assert.Single(found);
        Assert.Equal(expected, f.Severity);
        Assert.Single(f.Joints);
        Assert.Equal(2, f.Bodies.Count);
    }

    [Fact]
    public void HeavyBoxOnLightBoxIsFlaggedByContactMassRatio()
    {
        var heavy = New(); Floor(heavy);
        heavy.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 0.5, 0), density: 1f);
        heavy.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 1.5, 0), density: 400f);
        var f = Assert.Single(heavy.Diagnose(new DiagnoseOptions { ProbeSteps = 3 }).With(DiagnosticCode.ContactMassRatio));
        Assert.Equal(DiagnosticSeverity.Warning, f.Severity);
        var light = New(); Floor(light);
        light.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 0.5, 0), density: 1f);
        light.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 1.5, 0), density: 5f);
        AssertClean(light.Diagnose(new DiagnoseOptions { ProbeSteps = 3 }), DiagnosticCode.ContactMassRatio);
    }

    [Fact]
    public void AnchorFarFromItsBodyIsFlagged()
    {
        var w = New(false);
        var a = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(10, 0, 0), type: BodyType.Static);
        var b = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 0, 0));
        var j = new PointJointDef { BodyA = a, BodyB = b, LocalFrameA = At(0f), LocalFrameB = At(10f) };
        w.CreateJoint(in j);
        var r = w.Diagnose(new DiagnoseOptions { ProbeSteps = 0 });
        var f = Assert.Single(r.With(DiagnosticCode.AnchorFarFromBody));
        Assert.Equal(b.RawHandle, f.Bodies[0]);
        AssertClean(r, DiagnosticCode.AnchorsApart);   // the world anchors do coincide
    }

    [Fact]
    public void AnchorsThatStartApartAreFlagged()
    {
        var w = New(false);
        var a = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 0, 0), type: BodyType.Static);
        var b = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(3, 0, 0));
        var bad = new PointJointDef { BodyA = a, BodyB = b, LocalFrameA = At(0.5f), LocalFrameB = At(-0.5f) };   // 2 m apart
        w.CreateJoint(in bad);
        var r = w.Diagnose(new DiagnoseOptions { ProbeSteps = 0 });
        var f = Assert.Single(r.With(DiagnosticCode.AnchorsApart));
        Assert.Equal(2.0, f.Values[0].Value, 3);
        var ok = New(false);
        var c = ok.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 0, 0), type: BodyType.Static);
        var d = ok.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(1, 0, 0));
        var good = new PointJointDef { BodyA = c, BodyB = d, LocalFrameA = At(0.5f), LocalFrameB = At(-0.5f) };
        ok.CreateJoint(in good);
        AssertClean(ok.Diagnose(new DiagnoseOptions { ProbeSteps = 0 }), DiagnosticCode.AnchorsApart);
    }

    [Fact]
    public void SleepSettingsAreReported()
    {
        var w = New(); Floor(w);
        var a = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 0.5, 0)); a.NeverSleep = true;
        var r = w.Diagnose(new DiagnoseOptions { ProbeSteps = 0 });
        var f = Assert.Single(r.With(DiagnosticCode.NeverSleeps));
        Assert.Equal(a.RawHandle, f.Bodies[0]);
        w.Options.SleepEnabled = false;
        Assert.Equal(2, w.Diagnose(new DiagnoseOptions { ProbeSteps = 0 }).With(DiagnosticCode.NeverSleeps).Count());
    }

    // ---------------------------------------------------------------- observed over time

    [Fact]
    public void BoxesCreatedInsideEachOtherAreFlaggedAndSeparatedBoxesAreNot()
    {
        var w = New(false);
        var a = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 0, 0));
        var b = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0.1, 0, 0));
        var r = w.Diagnose(new DiagnoseOptions { ProbeSteps = 3 });
        var f = Assert.Single(r.With(DiagnosticCode.ShapesOverlapping));
        Assert.Equal(new[] { a.RawHandle, b.RawHandle }, f.Bodies.ToArray());
        var ok = New(false);
        ok.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 0, 0));
        ok.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(1.2, 0, 0));
        AssertClean(ok.Diagnose(new DiagnoseOptions { ProbeSteps = 3 }), DiagnosticCode.ShapesOverlapping);
    }

    [Fact]
    public void RestingStackSleepsAndDriftingBodyDoesNot()
    {
        var w = New(); Floor(w);
        for (int i = 0; i < 3; i++) w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 0.5 + i, 0));
        AssertClean(w.Diagnose(Long), DiagnosticCode.NeverSleeps);

        // A ball bouncing in a closed, frictionless box with no gravity moves at 0.3 m/s for ever: nearly at rest, never settling.
        var z = New(false);
        var mat = new Material3D(0f, 1f, 0f);
        foreach (var (size, at) in new[] { (new Vector3F(10, 1, 10), new Vector3D(0, -5.5, 0)), (new Vector3F(10, 1, 10), new Vector3D(0, 5.5, 0)), (new Vector3F(1, 10, 10), new Vector3D(-5.5, 0, 0)), (new Vector3F(1, 10, 10), new Vector3D(5.5, 0, 0)), (new Vector3F(10, 10, 1), new Vector3D(0, 0, -5.5)), (new Vector3F(10, 10, 1), new Vector3D(0, 0, 5.5)) })
            z.AddBox(size, at: at, type: BodyType.Static, material: mat);
        var ball = z.AddSphere(0.5f, new Vector3D(0, 0, 0), material: mat); ball.LinearVelocity = new Vector3F(0.3f, 0, 0);
        var f = Assert.Single(z.Diagnose(Long).With(DiagnosticCode.NeverSleeps));
        Assert.Equal(DiagnosticSeverity.Warning, f.Severity);
        Assert.Equal(ball.RawHandle, f.Bodies[0]);
    }

    [Fact]
    public void FastBodyWithoutContinuousCollisionIsATunnellingRisk()
    {
        foreach (bool ccd in new[] { false, true })
        {
            var w = New(false, ccd);
            w.AddBox(new Vector3F(0.2f, 10, 10), at: new Vector3D(20, 0, 0), type: BodyType.Static);
            var ball = w.AddSphere(0.25f, new Vector3D(0, 0, 0)); ball.LinearVelocity = new Vector3F(60, 0, 0);   // 1 m per step, thickness 0.5
            var r = w.Diagnose(new DiagnoseOptions { ProbeSteps = 3 });
            if (ccd) { AssertClean(r, DiagnosticCode.TunnellingRisk); continue; }
            var f = Assert.Single(r.With(DiagnosticCode.TunnellingRisk));
            Assert.Equal(DiagnosticSeverity.Warning, f.Severity);
            Assert.Equal(ball.RawHandle, f.Bodies[0]);
        }

        // With continuous collision on, a fast body can still pass a thin movable one unless it is a bullet.
        var m = New(false);
        var plate = m.AddBox(new Vector3F(0.05f, 4, 4), at: new Vector3D(20, 0, 0));
        var shot = m.AddSphere(0.25f, new Vector3D(0, 0, 0)); shot.LinearVelocity = new Vector3F(60, 0, 0);
        var f2 = Assert.Single(m.Diagnose(new DiagnoseOptions { ProbeSteps = 3 }).With(DiagnosticCode.TunnellingRisk));
        Assert.Equal(DiagnosticSeverity.Info, f2.Severity);
        shot.IsBullet = true;
        AssertClean(m.Diagnose(new DiagnoseOptions { ProbeSteps = 3 }), DiagnosticCode.TunnellingRisk);
        _ = plate;
    }

    private static (PhysicsWorld3D World, Body[] Bodies) Chain(double endRatio)
    {
        var c = New();
        Body prev = c.AddBox(new Vector3F(0.2f, 0.2f, 0.2f), at: new Vector3D(0, 10, 0), type: BodyType.Static);
        var list = new List<Body>();
        for (int i = 0; i < 8; i++)
        {
            var b = c.AddBox(new Vector3F(0.4f, 0.4f, 0.4f), at: new Vector3D(0.5 * (i + 1), 10, 0), density: i == 7 ? (float)endRatio : 1f);
            var pj = new PointJointDef { BodyA = prev, BodyB = b, LocalFrameA = At(0.25f), LocalFrameB = At(-0.25f) };
            c.CreateJoint(in pj);
            list.Add(b); prev = b;
        }

        return (c, list.ToArray());
    }

    [Fact]
    public void HeavyEndOfAChainStretchesTheJoints()
    {
        var (bad, _) = Chain(2000);
        var r = bad.Diagnose();
        Assert.True(r.Has(DiagnosticCode.JointNotConverging), r.ToString());
        Assert.True(r.With(DiagnosticCode.JointMassRatio).Any(f => f.Severity == DiagnosticSeverity.Error), r.ToString());
        var (ok, _) = Chain(1);
        AssertClean(ok.Diagnose(), DiagnosticCode.JointNotConverging);
    }

    [Fact]
    public void LoopOfInconsistentJointsIsFighting()
    {
        // A box between two static posts, fixed to both, with the second anchor a metre out of place: six rows too many and nowhere for them to agree.
        var w = New(false);
        var left = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(-2, 0, 0), type: BodyType.Static);
        var right = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(2, 0, 0), type: BodyType.Static);
        var mid = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 0, 0));
        var j1 = new FixedJointDef { BodyA = left, BodyB = mid, LocalFrameA = At(1.5f), LocalFrameB = At(-0.5f) };
        var j2 = new FixedJointDef { BodyA = mid, BodyB = right, LocalFrameA = At(0.5f), LocalFrameB = At(-0.5f) };   // 1 m off
        w.CreateJoint(in j1); w.CreateJoint(in j2);
        var r = w.Diagnose();
        var f = Assert.Single(r.With(DiagnosticCode.JointsFighting));
        Assert.True(f.Severity >= DiagnosticSeverity.Warning, f.ToString());
        Assert.Equal(2, f.Joints.Count);
        AssertClean(r, DiagnosticCode.JointNotConverging);   // reported once, as the loop

        // The same loop with consistent anchors is over-constrained on paper only.
        var ok = New(false);
        var l2 = ok.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(-2, 0, 0), type: BodyType.Static);
        var r2 = ok.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(2, 0, 0), type: BodyType.Static);
        var m2 = ok.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 0, 0));
        var k1 = new FixedJointDef { BodyA = l2, BodyB = m2, LocalFrameA = At(1.5f), LocalFrameB = At(-0.5f) };
        var k2 = new FixedJointDef { BodyA = m2, BodyB = r2, LocalFrameA = At(0.5f), LocalFrameB = At(-1.5f) };
        ok.CreateJoint(in k1); ok.CreateJoint(in k2);
        var f2 = Assert.Single(ok.Diagnose().With(DiagnosticCode.JointsFighting));
        Assert.Equal(DiagnosticSeverity.Info, f2.Severity);

        // A hinge on a post is not over-constrained at all.
        var pend = New(); var post = pend.AddBox(new Vector3F(0.2f, 0.2f, 0.2f), at: new Vector3D(0, 5, 0), type: BodyType.Static);
        var bob = pend.AddSphere(0.3f, new Vector3D(1, 5, 0));
        var h = new HingeJointDef { BodyA = post, BodyB = bob, LocalFrameB = At(-1f) };
        pend.CreateJoint(in h);
        AssertClean(pend.Diagnose(), DiagnosticCode.JointsFighting);
    }

    [Fact]
    public void BodyCrushedByAKinematicBlockKeepsPenetrating()
    {
        var w = New(); Floor(w);
        var box = w.AddBox(new Vector3F(1, 0.5f, 1), at: new Vector3D(0, 0.25, 0));
        var press = w.AddBox(new Vector3F(2, 1, 2), at: new Vector3D(0, 1.05, 0), type: BodyType.Kinematic);
        press.LinearVelocity = new Vector3F(0, -0.4f, 0);
        var r = w.Diagnose(new DiagnoseOptions { ProbeSteps = 120 });
        var f = Assert.Single(r.With(DiagnosticCode.PenetrationNotFalling));
        Assert.Contains(box.RawHandle, f.Bodies);
        var ok = New(); Floor(ok); ok.AddBox(new Vector3F(1, 0.5f, 1), at: new Vector3D(0, 0.25, 0));
        AssertClean(ok.Diagnose(new DiagnoseOptions { ProbeSteps = 120 }), DiagnosticCode.PenetrationNotFalling);
    }

    private static PhysicsWorld3D Motor(float maxTorque, float speed, bool limit, out Joint joint)
    {
        var w = New(false);
        var post = w.AddBox(new Vector3F(0.2f, 0.2f, 0.2f), at: new Vector3D(0, 0, 0), type: BodyType.Static);
        var arm = w.AddBox(new Vector3F(2, 0.5f, 0.5f), at: new Vector3D(1, 0, 0), density: 50f);
        var h = new HingeJointDef
        {
            BodyA = post, BodyB = arm,
            LocalFrameA = new JointFrame3D(new Vector3F(0, 0, 0), new QuaternionF(0, 0, 0.70710678f, 0.70710678f)),   // hinge axis along world y
            LocalFrameB = new JointFrame3D(new Vector3F(-1, 0, 0), new QuaternionF(0, 0, 0.70710678f, 0.70710678f)),
            EnableMotor = true, MotorSpeed = speed, MaxMotorTorque = maxTorque, EnableLimit = limit, LowerAngle = -0.5f, UpperAngle = 0.5f,
        };
        joint = w.CreateJoint(in h);
        return w;
    }

    [Fact]
    public void WeakMotorIsSaturatedAndStrongMotorIsNot()
    {
        var weak = Motor(2f, 10f, false, out _);
        var f = Assert.Single(weak.Diagnose().With(DiagnosticCode.MotorSaturated));
        Assert.Equal(DiagnosticSeverity.Warning, f.Severity);
        var strong = Motor(1e5f, 3f, false, out _);
        AssertClean(strong.Diagnose(), DiagnosticCode.MotorSaturated);
    }

    [Fact]
    public void MotorDrivingIntoItsLimitIsPinned()
    {
        var pinned = Motor(1e5f, 3f, true, out var j);
        var r = pinned.Diagnose();
        var f = Assert.Single(r.With(DiagnosticCode.LimitPinned));
        Assert.Equal(j.RawHandle, f.Joints[0]);
        AssertClean(r, DiagnosticCode.MotorSaturated);
        AssertClean(Motor(1e5f, 3f, false, out _).Diagnose(), DiagnosticCode.LimitPinned);
    }

    [Fact]
    public void ProbeThatCannotStepIsReportedAndTheStaticFindingsStay()
    {
        var w = New(false);
        w.AddGravityField(static (position, tick) => new Vector3F(0, 0, 0));   // a callback is host code: a copy of the world has none
        var thin = w.AddBox(new Vector3F(1, 0.005f, 1), at: new Vector3D(0, 3, 0));
        var r = w.Diagnose();
        var f = Assert.Single(r.With(DiagnosticCode.ProbeStopped));
        Assert.Equal(DiagnosticSeverity.Info, f.Severity);
        Assert.Contains("FieldCallbackException", f.Explanation);
        Assert.Equal(0, r.StepsObserved);
        Assert.Contains(thin.RawHandle, r.With(DiagnosticCode.ImplausibleSize).Single().Bodies);
    }

    [Fact]
    public void IslandHeldAwakeByAMovingKinematicBodyIsExplained()
    {
        var w = New(); Floor(w);
        var box = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 0.5, 0));
        var lift = w.AddBox(new Vector3F(3, 0.2f, 3), at: new Vector3D(0, -0.1, 0), type: BodyType.Kinematic);
        lift.LinearVelocity = new Vector3F(0.5f, 0, 0);
        var f = Assert.Single(w.Diagnose(Long).With(DiagnosticCode.NeverSleeps), x => x.Bodies.Contains(box.RawHandle));
        Assert.Equal(DiagnosticSeverity.Info, f.Severity);
        Assert.Contains("kinematic", f.Explanation);
    }

    // ---------------------------------------------------------------- the report itself

    [Fact]
    public void DiagnoseNeverChangesTheWorld()
    {
        var (w, _) = Chain(2000);
        Floor(w);
        for (int i = 0; i < 5; i++) w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(3, 0.5 + i, 0));
        for (int i = 0; i < 30; i++) w.Step(Dt);
        ulong hash = w.ComputeHash(); var bytes = w.Snapshot(); long steps = w.StepCount;
        var r = w.Diagnose();
        Assert.NotEmpty(r.Findings);
        Assert.Equal(hash, w.ComputeHash());
        Assert.Equal(steps, w.StepCount);
        Assert.True(bytes.AsSpan().SequenceEqual(w.Snapshot()));
        // And stepping on from here is what it would have been.
        var twin = New(); twin.Restore(bytes);
        for (int i = 0; i < 20; i++) { w.Step(Dt); twin.Step(Dt); Assert.Equal(twin.ComputeHash(), w.ComputeHash()); }
    }

    [Fact]
    public void DiagnosingWhileRecordingLeavesTheRecordingValid()
    {
        var (w, _) = Chain(300);
        using var ms = new MemoryStream();
        using (w.StartRecording(ms))
        {
            for (int i = 0; i < 20; i++) w.Step(Dt);
            Assert.NotEmpty(w.Diagnose(new DiagnoseOptions { ProbeSteps = 10 }).Findings);
            for (int i = 0; i < 20; i++) w.Step(Dt);
        }

        ms.Position = 0;
        var player = new RecursiveAI.OnePhysics.Replay.ReplayPlayer(ms);
        var copy = player.CreateWorld3D();
        int steps = 0;
        while (player.ReadNextStep(copy)) { steps++; }   // throws on the first hash that differs
        Assert.Equal(40, steps);
        Assert.Equal(w.ComputeHash(), copy.ComputeHash());
    }

    [Fact]
    public void FindingsAreTheSameAtAnyThreadCount()
    {
        string Run(int threads)
        {
            var w = new PhysicsWorld3D(new WorldOptions3D { Gravity = new Vector3F(0, -9.81f, 0), WorldId = 77, ThreadCount = threads });
            Floor(w);
            for (int i = 0; i < 60; i++) w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D((i % 5) * 1.05, 0.5 + (i / 5) * 1.0, 0), density: i == 3 ? 900f : 1f);
            return w.Diagnose().ToString();
        }

        Assert.Equal(Run(1), Run(8));
    }

    [Fact]
    public void OptionsAreValidated()
    {
        var w = New();
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Diagnose(new DiagnoseOptions { ProbeSteps = -1 }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Diagnose(new DiagnoseOptions { Dt = 0f }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Diagnose(new DiagnoseOptions { Dt = float.NaN }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Diagnose(new DiagnoseOptions { MaxFindingsPerCode = 0 }));
        Assert.Empty(w.Diagnose().Findings);   // an empty world is clean
    }

    [Fact]
    public void FindingsPerCodeAreCappedWorstFirst()
    {
        var w = New(false);
        for (int i = 0; i < 6; i++) w.AddBox(new Vector3F(1, 0.001f * (i + 1), 1), at: new Vector3D(i * 3, 0, 0));
        var r = w.Diagnose(new DiagnoseOptions { ProbeSteps = 0, MaxFindingsPerCode = 2 });
        Assert.Equal(2, r.With(DiagnosticCode.ImplausibleSize).Count());
        Assert.Equal(4, r.Suppressed);
        Assert.True(r.With(DiagnosticCode.ImplausibleSize).First().Values[0].Value < r.With(DiagnosticCode.ImplausibleSize).Last().Values[0].Value);   // thinnest first
    }

    // ---------------------------------------------------------------- a replay window finds what the live form finds

    [Fact]
    public void ReplayWindowFindsTheSameIssueAsTheLiveForm()
    {
        var (w, bodies) = Chain(2000);
        using var ms = new MemoryStream();
        using (w.StartRecording(ms))
        {
            for (int i = 0; i < 100; i++) w.Step(Dt);
        }

        var live = Chain(2000).World.Diagnose(new DiagnoseOptions { ProbeSteps = 100 });
        ms.Position = 0;
        var window = ReplayDiagnostics.Diagnose(ms, 0, 100);
        Assert.Equal(live.ToString(), window.ToString());
        Assert.Equal(0, window.FirstStep);
        Assert.Equal(100, window.LastStep);
        Assert.True(window.Has(DiagnosticCode.JointNotConverging));
        Assert.Equal(bodies[0].RawHandle.World, window.With(DiagnosticCode.JointNotConverging).First().Bodies[0].World);
    }

    [Fact]
    public void ReplayWindowIsValidated()
    {
        var w = New(); Floor(w); w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 3, 0));
        using var ms = new MemoryStream();
        using (w.StartRecording(ms)) { for (int i = 0; i < 20; i++) w.Step(Dt); }
        Assert.Throws<ArgumentOutOfRangeException>(() => { ms.Position = 0; ReplayDiagnostics.Diagnose(ms, 5, 3); });
        Assert.Throws<ArgumentOutOfRangeException>(() => { ms.Position = 0; ReplayDiagnostics.Diagnose(ms, 0, 50); });
        Assert.Throws<ArgumentOutOfRangeException>(() => { ms.Position = 0; ReplayDiagnostics.Diagnose(ms, 30, 40); });
        ms.Position = 0;
        var r = ReplayDiagnostics.Diagnose(ms, 10, 10);   // one state, no steps
        Assert.Equal(0, r.StepsObserved);
    }
}
