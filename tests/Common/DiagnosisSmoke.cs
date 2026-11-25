using System;
using System.IO;
using System.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Diagnosis;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

// World diagnosis in both dimensions: a chain with a 2000:1 end mass is diagnosed live (the world's hash must not move) and over a recorded window (the same findings), and the digest of the findings is returned.
internal static class DiagnosisSmoke
{
    private static ulong Digest(DiagnosisReport r)
    {
        ulong h = 1469598103934665603UL;
        foreach (char c in r.ToString()) { h = (h ^ c) * 1099511628211UL; }
        return h;
    }

    internal static bool Run(out ulong digest)
    {
        digest = 0;
        var w2 = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 71 });
        P2.Body prev2 = w2.AddBox(new Vector2F(0.2f, 0.2f), new Vector2D(0, 10), type: BodyType.Static);
        for (int i = 0; i < 6; i++)
        {
            var b = w2.AddBox(new Vector2F(0.4f, 0.4f), new Vector2D(0.5 * (i + 1), 10), density: i == 5 ? 2000f : 1f);
            w2.CreateJoint(new P2.RevoluteJointDef { BodyA = prev2, BodyB = b, LocalAnchorA = new Vector2F(0.25f, 0), LocalAnchorB = new Vector2F(-0.25f, 0) });
            prev2 = b;
        }

        var w3 = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 72 });
        P3.Body prev3 = w3.AddBox(new Vector3F(0.2f, 0.2f, 0.2f), new Vector3D(0, 10, 0), type: BodyType.Static);
        for (int i = 0; i < 6; i++)
        {
            var b = w3.AddBox(new Vector3F(0.4f, 0.4f, 0.4f), new Vector3D(0.5 * (i + 1), 10, 0), density: i == 5 ? 2000f : 1f);
            w3.CreateJoint(new P3.PointJointDef { BodyA = prev3, BodyB = b, LocalFrameA = new P3.JointFrame3D(new Vector3F(0.25f, 0, 0), QuaternionF.Identity), LocalFrameB = new P3.JointFrame3D(new Vector3F(-0.25f, 0, 0), QuaternionF.Identity) });
            prev3 = b;
        }

        using var rec2 = new MemoryStream(); using var rec3 = new MemoryStream();
        ulong h2 = w2.ComputeHash(), h3 = w3.ComputeHash();
        var live2 = w2.Diagnose(new DiagnoseOptions { ProbeSteps = 40 });
        var live3 = w3.Diagnose(new DiagnoseOptions { ProbeSteps = 40 });
        bool untouched = h2 == w2.ComputeHash() && h3 == w3.ComputeHash() && w2.StepCount == 0 && w3.StepCount == 0;
        using (w2.StartRecording(rec2)) { for (int i = 0; i < 40; i++) w2.Step(1f / 60f); }
        using (w3.StartRecording(rec3)) { for (int i = 0; i < 40; i++) w3.Step(1f / 60f); }
        rec2.Position = 0; rec3.Position = 0;
        var win2 = ReplayDiagnostics.Diagnose(rec2, 0, 40);
        var win3 = ReplayDiagnostics.Diagnose(rec3, 0, 40);
        bool found = live2.Has(DiagnosticCode.JointMassRatio) && live3.Has(DiagnosticCode.JointMassRatio) && live2.Has(DiagnosticCode.JointNotConverging) == win2.Has(DiagnosticCode.JointNotConverging);
        bool same = live2.ToString() == win2.ToString() && live3.ToString() == win3.ToString();
        digest = Digest(live2) * 31UL + Digest(live3);
        return untouched && found && same;
    }

}
