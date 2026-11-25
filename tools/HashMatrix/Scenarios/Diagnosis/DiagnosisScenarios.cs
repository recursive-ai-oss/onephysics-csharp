using System;
using System.IO;
using System.Runtime.CompilerServices;
using System.Text;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Diagnosis;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Physics3D;
using RecursiveAI.OnePhysics.Replay;

namespace RecursiveAI.OnePhysics.HashMatrix;

/// <summary>
/// Findings lists on the same bytes everywhere: a small world with a heavy-ended chain, a motor against its limit, a thin body and two boxes created inside each other is diagnosed after every step
/// (the world hash is written beside it, so a diagnosis that changed the world would show), and a recorded run is diagnosed over a window.
/// </summary>
internal static class DiagnosisScenarios
{
    private const float Dt = 1f / 60f;

    [ModuleInitializer]
    internal static void Register()
    {
        Scenarios.Register("diagnosis/live-2d", ctx => Live2D(ctx));
        Scenarios.Register("diagnosis/live-3d", ctx => Live3D(ctx));
        Scenarios.Register("diagnosis/window-2d", ctx => Window2D(ctx));
        Scenarios.Register("diagnosis/window-3d", ctx => Window3D(ctx));
    }

    private static void Write(ScenarioContext ctx, DiagnosisReport r)
    {
        ctx.Write(r.Findings.Count); ctx.Write(r.StepsObserved); ctx.Write(r.FirstStep); ctx.Write(r.LastStep); ctx.Write(r.Suppressed);
        foreach (var f in r.Findings)
        {
            ctx.Write((int)f.Code); ctx.Write((int)f.Severity);
            ctx.Write(f.Bodies.Count); foreach (var b in f.Bodies) { ctx.Write(b.World); ctx.Write(b.Index); ctx.Write(b.Generation); }
            ctx.Write(f.Joints.Count); foreach (var j in f.Joints) { ctx.Write(j.World); ctx.Write(j.Index); ctx.Write(j.Generation); }
            ctx.Write(f.Values.Count);
            foreach (var v in f.Values) { ctx.Write(Encoding.UTF8.GetBytes(v.Name)); ctx.Write(v.Value); ctx.Write(Encoding.UTF8.GetBytes(v.Unit)); ctx.Write(v.Threshold); }
            ctx.Write(Encoding.UTF8.GetBytes(f.Explanation)); ctx.Write(Encoding.UTF8.GetBytes(f.Suggestion));
        }
    }

    // ---------------------------------------------------------------- 3D

    private static PhysicsWorld3D Build3D(int threads)
    {
        var w = new PhysicsWorld3D(new WorldOptions3D { WorldId = 61, ThreadCount = threads, Gravity = new Vector3F(0, -9.81f, 0) });
        w.AddBox(new Vector3F(60, 1, 60), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        Physics3D.Body prev = w.AddBox(new Vector3F(0.2f, 0.2f, 0.2f), at: new Vector3D(0, 10, 0), type: BodyType.Static);
        for (int i = 0; i < 6; i++)
        {
            var b = w.AddBox(new Vector3F(0.4f, 0.4f, 0.4f), at: new Vector3D(0.5 * (i + 1), 10, 0), density: i == 5 ? 1500f : 1f);
            var j = new PointJointDef { BodyA = prev, BodyB = b, LocalFrameA = new JointFrame3D(new Vector3F(0.25f, 0, 0), QuaternionF.Identity), LocalFrameB = new JointFrame3D(new Vector3F(-0.25f, 0, 0), QuaternionF.Identity) };
            w.CreateJoint(in j);
            prev = b;
        }

        var post = w.AddBox(new Vector3F(0.2f, 0.2f, 0.2f), at: new Vector3D(10, 3, 0), type: BodyType.Static);
        var arm = w.AddBox(new Vector3F(2, 0.5f, 0.5f), at: new Vector3D(11, 3, 0), density: 50f);
        var q = new QuaternionF(0, 0, 0.70710678f, 0.70710678f);
        var h = new HingeJointDef { BodyA = post, BodyB = arm, LocalFrameA = new JointFrame3D(default, q), LocalFrameB = new JointFrame3D(new Vector3F(-1, 0, 0), q), EnableMotor = true, MotorSpeed = 10, MaxMotorTorque = 2f };
        w.CreateJoint(in h);
        w.AddBox(new Vector3F(1, 0.005f, 1), at: new Vector3D(-5, 3, 0));
        w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(-10, 0.5, 0));
        w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(-10.1, 0.5, 0));
        for (int i = 0; i < 4; i++) { w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(20, 0.5 + i, 0), density: i == 3 ? 400f : 1f); }
        return w;
    }

    private static void Live3D(ScenarioContext ctx)
    {
        var w = Build3D(ctx.Threads);
        for (int step = 0; step < 16; step++)
        {
            w.Step(Dt);
            ctx.Write(unchecked((long)w.ComputeHash()));
            Write(ctx, w.Diagnose(new DiagnoseOptions { ProbeSteps = step % 2 == 0 ? 40 : 0 }));
            ctx.Write(unchecked((long)w.ComputeHash()));
            ctx.EndStep();
        }
    }

    private static void Window3D(ScenarioContext ctx)
    {
        var w = Build3D(ctx.Threads);
        using var ms = new MemoryStream();
        using (w.StartRecording(ms)) { for (int i = 0; i < 60; i++) { w.Step(Dt); } }
        ctx.Write(unchecked((long)w.ComputeHash()));
        foreach (var (a, b) in new[] { (0L, 60L), (20L, 50L), (30L, 30L) })
        {
            ms.Position = 0;
            Write(ctx, ReplayDiagnostics.Diagnose(ms, a, b, null, ctx.Threads));
            ctx.EndStep();
        }
    }

    // ---------------------------------------------------------------- 2D

    private static PhysicsWorld2D Build2D(int threads)
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = 62, ThreadCount = threads, Gravity = new Vector2F(0, -9.81f) });
        w.AddBox(new Vector2F(60, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
        Physics2D.Body prev = w.AddBox(new Vector2F(0.2f, 0.2f), at: new Vector2D(0, 10), type: BodyType.Static);
        for (int i = 0; i < 6; i++)
        {
            var b = w.AddBox(new Vector2F(0.4f, 0.4f), at: new Vector2D(0.5 * (i + 1), 10), density: i == 5 ? 1500f : 1f);
            var j = new RevoluteJointDef { BodyA = prev, BodyB = b, LocalAnchorA = new Vector2F(0.25f, 0), LocalAnchorB = new Vector2F(-0.25f, 0) };
            w.CreateJoint(in j);
            prev = b;
        }

        var post = w.AddBox(new Vector2F(0.2f, 0.2f), at: new Vector2D(10, 3), type: BodyType.Static);
        var arm = w.AddBox(new Vector2F(2, 0.5f), at: new Vector2D(11, 3), density: 50f);
        var h = new RevoluteJointDef { BodyA = post, BodyB = arm, LocalAnchorB = new Vector2F(-1, 0), EnableMotor = true, MotorSpeed = 10, MaxMotorTorque = 2f };
        w.CreateJoint(in h);
        w.AddBox(new Vector2F(1, 0.005f), at: new Vector2D(-5, 3));
        w.AddBox(new Vector2F(1, 1), at: new Vector2D(-10, 0.5));
        w.AddBox(new Vector2F(1, 1), at: new Vector2D(-10.1, 0.5));
        for (int i = 0; i < 4; i++) { w.AddBox(new Vector2F(1, 1), at: new Vector2D(20, 0.5 + i), density: i == 3 ? 400f : 1f); }
        return w;
    }

    private static void Live2D(ScenarioContext ctx)
    {
        var w = Build2D(ctx.Threads);
        for (int step = 0; step < 16; step++)
        {
            w.Step(Dt);
            ctx.Write(unchecked((long)w.ComputeHash()));
            Write(ctx, w.Diagnose(new DiagnoseOptions { ProbeSteps = step % 2 == 0 ? 40 : 0 }));
            ctx.Write(unchecked((long)w.ComputeHash()));
            ctx.EndStep();
        }
    }

    private static void Window2D(ScenarioContext ctx)
    {
        var w = Build2D(ctx.Threads);
        using var ms = new MemoryStream();
        using (w.StartRecording(ms)) { for (int i = 0; i < 60; i++) { w.Step(Dt); } }
        ctx.Write(unchecked((long)w.ComputeHash()));
        foreach (var (a, b) in new[] { (0L, 60L), (20L, 50L), (30L, 30L) })
        {
            ms.Position = 0;
            Write(ctx, ReplayDiagnostics.Diagnose(ms, a, b, null, ctx.Threads));
            ctx.EndStep();
        }
    }
}
