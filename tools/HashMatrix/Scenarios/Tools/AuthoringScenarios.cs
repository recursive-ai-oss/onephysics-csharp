#pragma warning disable OPHYS0002, OPHYS0003
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Tools;
using RecursiveAI.OnePhysics.Tools.Tests;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.HashMatrix;

/// <summary>
/// The tetrahedralizer, the polygon mesher, the ragdoll fitter and the scenes built on them. The tools hash every output bit (at the runner's thread count, which must not matter);
/// the scenes hash the world after each step.
/// </summary>
internal static class AuthoringScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("tools/tetrahedralize", Tetrahedralize);
        Scenarios.Register("tools/triangulate2d", Triangulate2D);
        Scenarios.Register("tools/autoragdoll", AutoRagdoll);
        Scenarios.Register("tools/tet-drop-3d", TetDrop);
        Scenarios.Register("tools/tri-drop-2d", TriDrop);
        Scenarios.Register("tools/autoragdoll-pile-3d", Pile3D);
        Scenarios.Register("tools/autoragdoll-stand-3d", Stand3D);
        Scenarios.Register("tools/autoragdoll-pile-2d", Pile2D);
    }

    private static void WriteTet(ScenarioContext ctx, TetrahedralMesh t)
    {
        ctx.Write(t.Vertices.Length); ctx.Write(t.Tetrahedra.Length); ctx.Write(t.SurfaceTriangles.Length);
        foreach (var v in t.Vertices) { ctx.Write(v.X); ctx.Write(v.Y); ctx.Write(v.Z); }
        foreach (int i in t.Tetrahedra) { ctx.Write(i); }
        foreach (int i in t.SurfaceTriangles) { ctx.Write(i); }
        foreach (var b in t.Bindings) { ctx.Write(b.Tetrahedron); ctx.Write(b.W0); ctx.Write(b.W1); ctx.Write(b.W2); ctx.Write(b.W3); }
        ctx.Write(t.Volume); ctx.Write(t.MeshVolume); ctx.Write(t.WorstRadiusEdge); ctx.Write(t.MinVolumeRatio); ctx.Write(t.SurfaceDeviation); ctx.Write(t.CellSize);
    }

    private static void Tetrahedralize(ScenarioContext ctx)
    {
        var cases = new (TestMesh Mesh, TetrahedralizeOptions Options)[]
        {
            (TestMeshes.Sphere(1, 24, 32), new TetrahedralizeOptions { TargetEdgeLength = 0.2f, MaxThreads = ctx.Threads }),
            (TestMeshes.Torus(1, 0.35f, 32, 16), new TetrahedralizeOptions { TargetEdgeLength = 0.2f, MaxThreads = ctx.Threads }),
            (TetBlob.Build(40, 48), new TetrahedralizeOptions { TargetEdgeLength = 0.3f, MaxThreads = ctx.Threads }),
            (TetBlob.Build(40, 48), new TetrahedralizeOptions { MaxVolume = 0.004f, MaxRadiusEdgeRatio = 1.5f, MaxThreads = ctx.Threads }),
            (TestMeshes.Box(1, 2, 3), new TetrahedralizeOptions { MaxThreads = ctx.Threads, SkipDeviation = true }),
        };
        foreach (var (m, o) in cases) { WriteTet(ctx, Tetrahedralizer.Generate(m.V, m.I, o)); ctx.EndStep(); }
        // A mesh that needs cleaning first (the L-block's caps are not welded to its walls).
        var l = TestMeshes.LBlock();
        var clean = MeshCleaner.Clean(l.V, l.I);
        WriteTet(ctx, Tetrahedralizer.Generate(clean.Vertices, clean.Indices, new TetrahedralizeOptions { TargetEdgeLength = 0.15f, MaxThreads = ctx.Threads })); ctx.EndStep();
    }

    private static void Triangulate2D(ScenarioContext ctx)
    {
        foreach (var (outline, holes) in TriDropScene2D.Shapes())
        {
            foreach (float edge in new[] { 0f, 0.07f })
            {
                var r = TriangleMesher.Generate(outline, holes, new TriangleMeshOptions { TargetEdgeLength = edge });
                ctx.Write(r.Vertices.Length); ctx.Write(r.Triangles.Length);
                foreach (var v in r.Vertices) { ctx.Write(v.X); ctx.Write(v.Y); }
                foreach (int i in r.Triangles) { ctx.Write(i); }
                ctx.Write(r.MinAngleDegrees); ctx.Write(r.MaxAngleDegrees); ctx.Write(r.Area); ctx.Write(r.PolygonArea);
                ctx.EndStep();
            }
        }
    }

    private static void WriteDef3(ScenarioContext ctx, AutoRagdoll3D a)
    {
        ctx.Write(a.IsHumanoid ? 1 : 0); ctx.Write(a.Height);
        foreach (int i in a.PartOfBone) { ctx.Write(i); }
        foreach (var r in a.Roles) { ctx.Write((int)r); }
        if (a.Biped is { } b) { ctx.Write(b.Pelvis); ctx.Write(b.Torso); ctx.Write(b.Head); ctx.Write(b.LeftThigh); ctx.Write(b.LeftShin); ctx.Write(b.LeftFoot); ctx.Write(b.RightThigh); ctx.Write(b.RightShin); ctx.Write(b.RightFoot); }
        foreach (var p in a.Definition.Parts)
        {
            ctx.Write(p.Parent); ctx.Write((int)p.Shape); ctx.Write(p.ShapeHalfSize.X); ctx.Write(p.ShapeHalfSize.Y); ctx.Write(p.ShapeHalfSize.Z);
            ctx.Write(p.Position.X); ctx.Write(p.Position.Y); ctx.Write(p.Position.Z); ctx.Write(p.Rotation.X); ctx.Write(p.Rotation.Y); ctx.Write(p.Rotation.Z); ctx.Write(p.Rotation.W);
            ctx.Write(p.Mass); ctx.Write(p.JointPosition.X); ctx.Write(p.JointPosition.Y); ctx.Write(p.JointPosition.Z);
            ctx.Write(p.JointRotation.X); ctx.Write(p.JointRotation.Y); ctx.Write(p.JointRotation.Z); ctx.Write(p.JointRotation.W);
            ctx.Write(p.LowerTwist); ctx.Write(p.UpperTwist); ctx.Write(p.SwingYHalfAngle); ctx.Write(p.SwingZHalfAngle); ctx.Write(p.DriveStiffness); ctx.Write(p.DriveDamping); ctx.Write(p.MaxTorque);
        }
    }

    private static void AutoRagdoll(ScenarioContext ctx)
    {
        var sk = TestSkeleton.Humanoid();
        foreach (var o in new[] { new AutoRagdollOptions(), new AutoRagdollOptions { IncludeHands = true, SplitTorso = true, TotalMass = 90f }, new AutoRagdollOptions { MassModel = AutoRagdollMassModel.Volume, MinSwingHalfAngle = 0.6f } })
        {
            WriteDef3(ctx, AutoRagdoller.Create(sk.Names, sk.Parents, sk.Positions, sk.Skin, o)); ctx.EndStep();
        }

        WriteDef3(ctx, AutoRagdoller.Create(sk.Names, sk.Parents, sk.Positions)); ctx.EndStep();
        var tall = TestSkeleton.Humanoid(2.1f, rightFirst: true);
        WriteDef3(ctx, AutoRagdoller.Create(tall.Names, tall.Parents, tall.Positions, tall.Skin)); ctx.EndStep();
        // a skeleton that is not a person, with a flat part
        var names = new[] { "hub", "wing", "tail" }; var parents = new[] { -1, 0, 0 }; var pos = new[] { new Vector3F(0, 0, 0), new Vector3F(1, 0, 0), new Vector3F(0, 0, -1) };
        var pts = new List<Vector3F>(); var bi = new List<int>(); var bw = new List<float>();
        for (int i = 0; i < 30; i++) { float x = 1 + i * 0.1f; foreach (float z in new[] { -0.4f, 0.4f }) { pts.Add(new Vector3F(x, 0.01f * (i % 2), z)); bi.Add(1); bw.Add(1f); } }
        WriteDef3(ctx, AutoRagdoller.Create(names, parents, pos, new SkinnedPoints3D(pts.ToArray(), bi.ToArray(), bw.ToArray(), 1))); ctx.EndStep();
        foreach (var o in new[] { new AutoRagdollOptions(), new AutoRagdollOptions { Facing = -1, IncludeHands = true } })
        {
            var a = AutoRagdoller.Create2D(sk.Names, sk.Parents, sk.Positions2D(), sk.Skin2D(), o);
            ctx.Write(a.IsHumanoid ? 1 : 0); ctx.Write(a.Height);
            foreach (var p in a.Definition.Parts)
            {
                ctx.Write(p.Parent); ctx.Write((int)p.Shape); ctx.Write(p.ShapeHalfSize.X); ctx.Write(p.ShapeHalfSize.Y); ctx.Write(p.Position.X); ctx.Write(p.Position.Y); ctx.Write(p.Angle); ctx.Write(p.Mass);
                ctx.Write(p.JointPosition.X); ctx.Write(p.JointPosition.Y); ctx.Write(p.LowerAngle); ctx.Write(p.UpperAngle); ctx.Write(p.MaxTorque);
            }

            ctx.EndStep();
        }
    }

    private static void TetDrop(ScenarioContext ctx)
    {
        var w = TetDropScene.Build(out var bodies, null, ctx.Threads);
        for (int step = 0; step < 150; step++)
        {
            w.Step(1f / 60f);
            ctx.Write(unchecked((long)w.ComputeHash()));
            if (step % 25 == 24) { foreach (var b in bodies) { var c = b.CenterOfMass; ctx.Write(c.X); ctx.Write(c.Y); ctx.Write(c.Z); ctx.Write(b.Volume); } }
            ctx.EndStep();
        }
    }

    private static void TriDrop(ScenarioContext ctx)
    {
        var w = TriDropScene2D.Build(out var bodies);
        for (int step = 0; step < 150; step++)
        {
            w.Step(1f / 60f);
            ctx.Write(unchecked((long)w.ComputeHash()));
            if (step % 25 == 24) { foreach (var b in bodies) { var c = b.CenterOfMass; ctx.Write(c.X); ctx.Write(c.Y); } }
            ctx.EndStep();
        }
    }

    private static void Pile3D(ScenarioContext ctx)
    {
        var w = AutoRagdollScenes.BuildPile3D(out var rags, out _, 8, ctx.Threads);
        for (int step = 0; step < 150; step++)
        {
            w.Step(1f / 60f);
            ctx.Write(unchecked((long)w.ComputeHash()));
            if (step % 25 == 24) { foreach (var r in rags) { var p = r.GetBody(0).Position; ctx.Write(p.X); ctx.Write(p.Y); ctx.Write(p.Z); } }
            ctx.EndStep();
        }
    }

    private static void Stand3D(ScenarioContext ctx)
    {
        var w = AutoRagdollScenes.BuildStand3D(out var rag, out var auto, ctx.Threads);
        for (int step = 0; step < 240; step++)
        {
            if (step == 120) { rag.GetBody(auto.Biped!.Torso).ApplyImpulse(new Vector3F(0, 0, 12f)); }
            w.Step(1f / 60f);
            ctx.Write(unchecked((long)w.ComputeHash()));
            ctx.EndStep();
        }
    }

    private static void Pile2D(ScenarioContext ctx)
    {
        var w = AutoRagdollScenes.BuildPile2D(out var rags, out _, 4, ctx.Threads);
        for (int step = 0; step < 150; step++)
        {
            w.Step(1f / 60f);
            ctx.Write(unchecked((long)w.ComputeHash()));
            if (step % 25 == 24) { foreach (var r in rags) { var p = r.GetBody(0).Position; ctx.Write(p.X); ctx.Write(p.Y); } }
            ctx.EndStep();
        }
    }
}
