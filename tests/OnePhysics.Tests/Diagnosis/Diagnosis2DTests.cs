using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Diagnosis;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Diagnosis;

/// <summary>The 2D twin of <see cref="Diagnosis3DTests"/>: every finding has a scene that triggers it and one that does not.</summary>
public class Diagnosis2DTests
{
    private const float Dt = 1f / 60f;
    private static readonly DiagnoseOptions Long = new() { ProbeSteps = 150 };

    private static PhysicsWorld2D New(bool gravity = true, bool ccd = true) => new(new WorldOptions2D { Gravity = gravity ? new Vector2F(0, -9.81f) : default, WorldId = 42, ContinuousEnabled = ccd });
    private static Body Floor(PhysicsWorld2D w) => w.AddBox(new Vector2F(200, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);

    private static void AssertClean(DiagnosisReport r, DiagnosticCode code) => Assert.False(r.Has(code), "unexpected " + code + ": " + string.Join("\n", r.With(code)));

    [Fact]
    public void GravityInPixelsIsFlagged()
    {
        var w = New(); w.Options.Gravity = new Vector2F(0, -981f);
        w.AddBox(new Vector2F(1, 1), at: new Vector2D(0, 1));
        var f = Assert.Single(w.Diagnose(new DiagnoseOptions { ProbeSteps = 0 }).With(DiagnosticCode.ImplausibleGravity));
        Assert.Equal(DiagnosticSeverity.Warning, f.Severity);
        AssertClean(New().Diagnose(), DiagnosticCode.ImplausibleGravity);
    }

    [Fact]
    public void ThinAndHugeBodiesAreFlagged()
    {
        var w = New(); Floor(w);
        var thin = w.AddBox(new Vector2F(1, 0.005f), at: new Vector2D(0, 3));
        var huge = w.AddBox(new Vector2F(2500, 2500), at: new Vector2D(0, 3000), density: 1e-6f);
        var sizes = w.Diagnose(new DiagnoseOptions { ProbeSteps = 0 }).With(DiagnosticCode.ImplausibleSize).ToList();
        Assert.Contains(sizes, f => f.Severity == DiagnosticSeverity.Warning && f.Bodies[0] == thin.RawHandle);
        Assert.Contains(sizes, f => f.Severity == DiagnosticSeverity.Info && f.Bodies[0] == huge.RawHandle);
        var ok = New(); ok.AddBox(new Vector2F(1, 1), at: new Vector2D(0, 3));
        AssertClean(ok.Diagnose(new DiagnoseOptions { ProbeSteps = 0 }), DiagnosticCode.ImplausibleSize);
    }

    [Fact]
    public void SpeedNearTheLimitIsFlagged()
    {
        var w = New(false);
        var fast = w.AddCircle(0.5f, new Vector2D(0, 0)); fast.LinearVelocity = new Vector2F(250, 0);
        var slow = w.AddCircle(0.5f, new Vector2D(0, 5)); slow.LinearVelocity = new Vector2F(5, 0);
        var f = Assert.Single(w.Diagnose(new DiagnoseOptions { ProbeSteps = 0 }).With(DiagnosticCode.ImplausibleSpeed));
        Assert.Equal(fast.RawHandle, f.Bodies[0]);
    }

    [Theory]
    [InlineData(2.0, null)]
    [InlineData(80.0, DiagnosticSeverity.Info)]
    [InlineData(800.0, DiagnosticSeverity.Warning)]
    [InlineData(3000.0, DiagnosticSeverity.Error)]
    public void JointMassRatioHasSeverityBands(double ratio, DiagnosticSeverity? expected)
    {
        var w = New(false);
        var a = w.AddBox(new Vector2F(1, 1), at: new Vector2D(0, 0));
        var b = w.AddBox(new Vector2F(1, 1), at: new Vector2D(1, 0), density: (float)ratio);
        var j = new RevoluteJointDef { BodyA = a, BodyB = b, LocalAnchorA = new Vector2F(0.5f, 0), LocalAnchorB = new Vector2F(-0.5f, 0) };
        w.CreateJoint(in j);
        var found = w.Diagnose(new DiagnoseOptions { ProbeSteps = 0 }).With(DiagnosticCode.JointMassRatio).ToList();
        if (expected == null) { Assert.Empty(found); return; }
        var f = Assert.Single(found);
        Assert.Equal(expected, f.Severity);
        Assert.Single(f.Joints);
    }

    [Fact]
    public void HeavyBoxOnLightBoxIsFlaggedByContactMassRatio()
    {
        var heavy = New(); Floor(heavy);
        heavy.AddBox(new Vector2F(1, 1), at: new Vector2D(0, 0.5), density: 1f);
        heavy.AddBox(new Vector2F(1, 1), at: new Vector2D(0, 1.5), density: 400f);
        var f = Assert.Single(heavy.Diagnose(new DiagnoseOptions { ProbeSteps = 3 }).With(DiagnosticCode.ContactMassRatio));
        Assert.Equal(DiagnosticSeverity.Warning, f.Severity);
        var light = New(); Floor(light);
        light.AddBox(new Vector2F(1, 1), at: new Vector2D(0, 0.5), density: 1f);
        light.AddBox(new Vector2F(1, 1), at: new Vector2D(0, 1.5), density: 5f);
        AssertClean(light.Diagnose(new DiagnoseOptions { ProbeSteps = 3 }), DiagnosticCode.ContactMassRatio);
    }

    [Fact]
    public void AnchorFarFromItsBodyIsFlagged()
    {
        var w = New(false);
        var a = w.AddBox(new Vector2F(1, 1), at: new Vector2D(10, 0), type: BodyType.Static);
        var b = w.AddBox(new Vector2F(1, 1), at: new Vector2D(0, 0));
        var j = new RevoluteJointDef { BodyA = a, BodyB = b, LocalAnchorA = new Vector2F(0, 0), LocalAnchorB = new Vector2F(10, 0) };
        w.CreateJoint(in j);
        var r = w.Diagnose(new DiagnoseOptions { ProbeSteps = 0 });
        var f = Assert.Single(r.With(DiagnosticCode.AnchorFarFromBody));
        Assert.Equal(b.RawHandle, f.Bodies[0]);
        AssertClean(r, DiagnosticCode.AnchorsApart);
    }

    [Fact]
    public void AnchorsThatStartApartAreFlagged()
    {
        var w = New(false);
        var a = w.AddBox(new Vector2F(1, 1), at: new Vector2D(0, 0), type: BodyType.Static);
        var b = w.AddBox(new Vector2F(1, 1), at: new Vector2D(3, 0));
        var bad = new RevoluteJointDef { BodyA = a, BodyB = b, LocalAnchorA = new Vector2F(0.5f, 0), LocalAnchorB = new Vector2F(-0.5f, 0) };   // 2 m apart
        w.CreateJoint(in bad);
        var f = Assert.Single(w.Diagnose(new DiagnoseOptions { ProbeSteps = 0 }).With(DiagnosticCode.AnchorsApart));
        Assert.Equal(2.0, f.Values[0].Value, 3);
        var ok = New(false);
        var c = ok.AddBox(new Vector2F(1, 1), at: new Vector2D(0, 0), type: BodyType.Static);
        var d = ok.AddBox(new Vector2F(1, 1), at: new Vector2D(1, 0));
        var good = new RevoluteJointDef { BodyA = c, BodyB = d, LocalAnchorA = new Vector2F(0.5f, 0), LocalAnchorB = new Vector2F(-0.5f, 0) };
        ok.CreateJoint(in good);
        AssertClean(ok.Diagnose(new DiagnoseOptions { ProbeSteps = 0 }), DiagnosticCode.AnchorsApart);
    }

    [Fact]
    public void SleepSettingsAreReported()
    {
        var w = New(); Floor(w);
        var a = w.AddBox(new Vector2F(1, 1), at: new Vector2D(0, 0.5)); a.NeverSleep = true;
        var f = Assert.Single(w.Diagnose(new DiagnoseOptions { ProbeSteps = 0 }).With(DiagnosticCode.NeverSleeps));
        Assert.Equal(a.RawHandle, f.Bodies[0]);
        w.Options.SleepEnabled = false;
        Assert.Equal(2, w.Diagnose(new DiagnoseOptions { ProbeSteps = 0 }).With(DiagnosticCode.NeverSleeps).Count());
    }

    [Fact]
    public void BoxesCreatedInsideEachOtherAreFlaggedAndSeparatedBoxesAreNot()
    {
        var w = New(false);
        var a = w.AddBox(new Vector2F(1, 1), at: new Vector2D(0, 0));
        var b = w.AddBox(new Vector2F(1, 1), at: new Vector2D(0.1, 0));
        var f = Assert.Single(w.Diagnose(new DiagnoseOptions { ProbeSteps = 3 }).With(DiagnosticCode.ShapesOverlapping));
        Assert.Equal(new[] { a.RawHandle, b.RawHandle }, f.Bodies.ToArray());
        var ok = New(false);
        ok.AddBox(new Vector2F(1, 1), at: new Vector2D(0, 0));
        ok.AddBox(new Vector2F(1, 1), at: new Vector2D(1.2, 0));
        AssertClean(ok.Diagnose(new DiagnoseOptions { ProbeSteps = 3 }), DiagnosticCode.ShapesOverlapping);
    }

    [Fact]
    public void RestingStackSleepsAndDriftingBodyDoesNot()
    {
        var w = New(); Floor(w);
        for (int i = 0; i < 3; i++) w.AddBox(new Vector2F(1, 1), at: new Vector2D(0, 0.5 + i));
        AssertClean(w.Diagnose(Long), DiagnosticCode.NeverSleeps);

        var z = New(false);
        var mat = new Material2D(0f, 1f);
        foreach (var (size, at) in new[] { (new Vector2F(10, 1), new Vector2D(0, -5.5)), (new Vector2F(10, 1), new Vector2D(0, 5.5)), (new Vector2F(1, 10), new Vector2D(-5.5, 0)), (new Vector2F(1, 10), new Vector2D(5.5, 0)) })
            z.AddBox(size, at: at, type: BodyType.Static, material: mat);
        var ball = z.AddCircle(0.5f, new Vector2D(0, 0), material: mat); ball.LinearVelocity = new Vector2F(0.3f, 0);
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
            w.AddBox(new Vector2F(0.2f, 10), at: new Vector2D(20, 0), type: BodyType.Static);
            var ball = w.AddCircle(0.25f, new Vector2D(0, 0)); ball.LinearVelocity = new Vector2F(60, 0);
            var r = w.Diagnose(new DiagnoseOptions { ProbeSteps = 3 });
            if (ccd) { AssertClean(r, DiagnosticCode.TunnellingRisk); continue; }
            var f = Assert.Single(r.With(DiagnosticCode.TunnellingRisk));
            Assert.Equal(DiagnosticSeverity.Warning, f.Severity);
            Assert.Equal(ball.RawHandle, f.Bodies[0]);
        }

        var m = New(false);
        m.AddBox(new Vector2F(0.05f, 4), at: new Vector2D(20, 0));
        var shot = m.AddCircle(0.25f, new Vector2D(0, 0)); shot.LinearVelocity = new Vector2F(60, 0);
        var f2 = Assert.Single(m.Diagnose(new DiagnoseOptions { ProbeSteps = 3 }).With(DiagnosticCode.TunnellingRisk));
        Assert.Equal(DiagnosticSeverity.Info, f2.Severity);
        shot.IsBullet = true;
        AssertClean(m.Diagnose(new DiagnoseOptions { ProbeSteps = 3 }), DiagnosticCode.TunnellingRisk);
    }

    private static PhysicsWorld2D Chain(double endRatio)
    {
        var c = New();
        Body prev = c.AddBox(new Vector2F(0.2f, 0.2f), at: new Vector2D(0, 10), type: BodyType.Static);
        for (int i = 0; i < 8; i++)
        {
            var b = c.AddBox(new Vector2F(0.4f, 0.4f), at: new Vector2D(0.5 * (i + 1), 10), density: i == 7 ? (float)endRatio : 1f);
            var rj = new RevoluteJointDef { BodyA = prev, BodyB = b, LocalAnchorA = new Vector2F(0.25f, 0), LocalAnchorB = new Vector2F(-0.25f, 0) };
            c.CreateJoint(in rj);
            prev = b;
        }

        return c;
    }

    [Fact]
    public void HeavyEndOfAChainStretchesTheJoints()
    {
        var r = Chain(2000).Diagnose();
        Assert.True(r.Has(DiagnosticCode.JointNotConverging), r.ToString());
        Assert.True(r.With(DiagnosticCode.JointMassRatio).Any(f => f.Severity == DiagnosticSeverity.Error), r.ToString());
        AssertClean(Chain(1).Diagnose(), DiagnosticCode.JointNotConverging);
    }

    [Fact]
    public void LoopOfInconsistentJointsIsFighting()
    {
        PhysicsWorld2D Build(float rightAnchor)
        {
            var w = New(false);
            var left = w.AddBox(new Vector2F(1, 1), at: new Vector2D(-2, 0), type: BodyType.Static);
            var right = w.AddBox(new Vector2F(1, 1), at: new Vector2D(2, 0), type: BodyType.Static);
            var mid = w.AddBox(new Vector2F(1, 1), at: new Vector2D(0, 0));
            var j1 = new WeldJointDef { BodyA = left, BodyB = mid, LocalAnchorA = new Vector2F(1.5f, 0), LocalAnchorB = new Vector2F(-0.5f, 0) };
            var j2 = new WeldJointDef { BodyA = mid, BodyB = right, LocalAnchorA = new Vector2F(0.5f, 0), LocalAnchorB = new Vector2F(rightAnchor, 0) };
            w.CreateJoint(in j1); w.CreateJoint(in j2);
            return w;
        }

        var f = Assert.Single(Build(-0.5f).Diagnose().With(DiagnosticCode.JointsFighting));
        Assert.True(f.Severity >= DiagnosticSeverity.Warning, f.ToString());
        Assert.Equal(2, f.Joints.Count);
        var f2 = Assert.Single(Build(-1.5f).Diagnose().With(DiagnosticCode.JointsFighting));
        Assert.Equal(DiagnosticSeverity.Info, f2.Severity);

        var pend = New(); var post = pend.AddBox(new Vector2F(0.2f, 0.2f), at: new Vector2D(0, 5), type: BodyType.Static);
        var bob = pend.AddCircle(0.3f, new Vector2D(1, 5));
        var h = new RevoluteJointDef { BodyA = post, BodyB = bob, LocalAnchorB = new Vector2F(-1f, 0) };
        pend.CreateJoint(in h);
        AssertClean(pend.Diagnose(), DiagnosticCode.JointsFighting);
    }

    [Fact]
    public void BodyCrushedByAKinematicBlockKeepsPenetrating()
    {
        var w = New(); Floor(w);
        var box = w.AddBox(new Vector2F(1, 0.5f), at: new Vector2D(0, 0.25));
        var press = w.AddBox(new Vector2F(2, 1), at: new Vector2D(0, 1.05), type: BodyType.Kinematic);
        press.LinearVelocity = new Vector2F(0, -0.4f);
        var f = Assert.Single(w.Diagnose(new DiagnoseOptions { ProbeSteps = 120 }).With(DiagnosticCode.PenetrationNotFalling));
        Assert.Contains(box.RawHandle, f.Bodies);
        var ok = New(); Floor(ok); ok.AddBox(new Vector2F(1, 0.5f), at: new Vector2D(0, 0.25));
        AssertClean(ok.Diagnose(new DiagnoseOptions { ProbeSteps = 120 }), DiagnosticCode.PenetrationNotFalling);
    }

    private static PhysicsWorld2D Motor(float maxTorque, float speed, bool limit, out Joint joint)
    {
        var w = New(false);
        var post = w.AddBox(new Vector2F(0.2f, 0.2f), at: new Vector2D(0, 0), type: BodyType.Static);
        var arm = w.AddBox(new Vector2F(2, 0.5f), at: new Vector2D(1, 0), density: 50f);
        var h = new RevoluteJointDef
        {
            BodyA = post, BodyB = arm, LocalAnchorA = new Vector2F(0, 0), LocalAnchorB = new Vector2F(-1, 0),
            EnableMotor = true, MotorSpeed = speed, MaxMotorTorque = maxTorque, EnableLimit = limit, LowerAngle = -0.5f, UpperAngle = 0.5f,
        };
        joint = w.CreateJoint(in h);
        return w;
    }

    [Fact]
    public void WeakMotorIsSaturatedAndStrongMotorIsNot()
    {
        var f = Assert.Single(Motor(2f, 10f, false, out _).Diagnose().With(DiagnosticCode.MotorSaturated));
        Assert.Equal(DiagnosticSeverity.Warning, f.Severity);
        AssertClean(Motor(1e5f, 3f, false, out _).Diagnose(), DiagnosticCode.MotorSaturated);
    }

    [Fact]
    public void MotorDrivingIntoItsLimitIsPinned()
    {
        var r = Motor(1e5f, 3f, true, out var j).Diagnose();
        var f = Assert.Single(r.With(DiagnosticCode.LimitPinned));
        Assert.Equal(j.RawHandle, f.Joints[0]);
        AssertClean(r, DiagnosticCode.MotorSaturated);
        AssertClean(Motor(1e5f, 3f, false, out _).Diagnose(), DiagnosticCode.LimitPinned);
    }

    [Fact]
    public void DiagnoseNeverChangesTheWorld()
    {
        var w = Chain(2000);
        Floor(w);
        for (int i = 0; i < 5; i++) w.AddBox(new Vector2F(1, 1), at: new Vector2D(3, 0.5 + i));
        for (int i = 0; i < 30; i++) w.Step(Dt);
        ulong hash = w.ComputeHash(); var bytes = w.Snapshot(); long steps = w.StepCount;
        Assert.NotEmpty(w.Diagnose().Findings);
        Assert.Equal(hash, w.ComputeHash());
        Assert.Equal(steps, w.StepCount);
        Assert.True(bytes.AsSpan().SequenceEqual(w.Snapshot()));
        var twin = New(); twin.Restore(bytes);
        for (int i = 0; i < 20; i++) { w.Step(Dt); twin.Step(Dt); Assert.Equal(twin.ComputeHash(), w.ComputeHash()); }
    }

    [Fact]
    public void FindingsAreTheSameAtAnyThreadCount()
    {
        string Run(int threads)
        {
            var w = new PhysicsWorld2D(new WorldOptions2D { Gravity = new Vector2F(0, -9.81f), WorldId = 78, ThreadCount = threads });
            Floor(w);
            for (int i = 0; i < 60; i++) w.AddBox(new Vector2F(1, 1), at: new Vector2D((i % 5) * 1.05, 0.5 + (i / 5) * 1.0), density: i == 3 ? 900f : 1f);
            return w.Diagnose().ToString();
        }

        Assert.Equal(Run(1), Run(8));
    }

    [Fact]
    public void OptionsAreValidatedAndFindingsAreCapped()
    {
        var w = New();
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Diagnose(new DiagnoseOptions { ProbeSteps = -1 }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Diagnose(new DiagnoseOptions { Dt = -1f }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Diagnose(new DiagnoseOptions { MaxFindingsPerCode = 1001 }));
        Assert.Empty(w.Diagnose().Findings);
        var t = New(false);
        for (int i = 0; i < 6; i++) t.AddBox(new Vector2F(1, 0.001f * (i + 1)), at: new Vector2D(i * 3, 0));
        var r = t.Diagnose(new DiagnoseOptions { ProbeSteps = 0, MaxFindingsPerCode = 2 });
        Assert.Equal(2, r.With(DiagnosticCode.ImplausibleSize).Count());
        Assert.Equal(4, r.Suppressed);
    }

    [Fact]
    public void ReplayWindowFindsTheSameIssueAsTheLiveForm()
    {
        var w = Chain(2000);
        using var ms = new MemoryStream();
        using (w.StartRecording(ms)) { for (int i = 0; i < 100; i++) w.Step(Dt); }
        var live = Chain(2000).Diagnose(new DiagnoseOptions { ProbeSteps = 100 });
        ms.Position = 0;
        var window = ReplayDiagnostics.Diagnose(ms, 0, 100);
        Assert.Equal(live.ToString(), window.ToString());
        Assert.True(window.Has(DiagnosticCode.JointNotConverging));
        // A window that starts later sees the world at that step.
        ms.Position = 0;
        var late = ReplayDiagnostics.Diagnose(ms, 40, 100);
        Assert.Equal(40, late.FirstStep);
        Assert.Equal(60, late.StepsObserved);
    }
}
