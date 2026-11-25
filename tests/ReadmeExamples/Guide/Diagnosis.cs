using System;
using System.IO;
using System.Linq;
using RecursiveAI.OneMath;                // Vector2F, Vector2D
using RecursiveAI.OnePhysics;             // BodyType
using RecursiveAI.OnePhysics.Diagnosis;   // DiagnosisReport, DiagnoseOptions, ReplayDiagnostics
using RecursiveAI.OnePhysics.Physics2D;   // PhysicsWorld2D, RevoluteJointDef

namespace GuideExamples;

/// <summary>The code blocks of docs/guide/diagnosis.md.</summary>
public static class Diagnosis
{
    public static (int Findings, bool SameInReplay, bool WorldUnchanged) Run()
    {
        var world = new PhysicsWorld2D(new WorldOptions2D { WorldId = 1, Gravity = new Vector2F(0, -9.81f) });
        var post = world.AddBox(new Vector2F(0.2f, 0.2f), at: new Vector2D(0, 5), type: BodyType.Static);
        var weight = world.AddBox(new Vector2F(1, 1), at: new Vector2D(1, 5), density: 3000f);   // 3000 times as heavy as the link it hangs from
        var link = world.AddBox(new Vector2F(1, 0.2f), at: new Vector2D(0.5, 5));
        world.CreateJoint(new RevoluteJointDef { BodyA = post, BodyB = link, LocalAnchorB = new Vector2F(-0.5f, 0) });
        world.CreateJoint(new RevoluteJointDef { BodyA = link, BodyB = weight, LocalAnchorA = new Vector2F(0.5f, 0), LocalAnchorB = new Vector2F(-0.5f, 0) });

        ulong before = world.ComputeHash();
        DiagnosisReport report = world.Diagnose();               // examines this world and a copy of it stepped for 1.5 s; this world is not touched
        bool unchanged = before == world.ComputeHash();
        foreach (Finding f in report.Findings)
        {
            Console.WriteLine(f);                                // OPD1002 Error: The revolute joint connects bodies of ... Suggestion: ...
        }

        if (report.With(DiagnosticCode.JointMassRatio).Any(f => f.Severity == DiagnosticSeverity.Error)) { Console.WriteLine("lighten the heavy body or add substeps"); }

        // A recording can be diagnosed over a window of steps, without a world.
        using var recording = new MemoryStream();
        using (world.StartRecording(recording))
        {
            for (int i = 0; i < 60; i++) world.Step(1f / 60);
        }

        recording.Position = 0;
        DiagnosisReport window = ReplayDiagnostics.Diagnose(recording, firstStep: 0, lastStep: 60);
        bool same = window.Has(DiagnosticCode.JointMassRatio) == report.Has(DiagnosticCode.JointMassRatio);
        return (report.Findings.Count, same, unchanged);
    }
}
