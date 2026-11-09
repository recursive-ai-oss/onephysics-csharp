#pragma warning disable OPHYS0002, OPHYS0003
using System.Diagnostics;
using System.Globalization;
using RecursiveAI.OnePhysics.Tools;
using RecursiveAI.OnePhysics.Tools.Tests;

namespace Bar;

// Scenes built on the authoring tools' output (no reference engine runs any of them, so the thresholds are our own; see docs/spec/tools.md):
// tet-drop-3d and tri-drop-2d (soft bodies from tetrahedralized and triangulated meshes), autoragdoll-pile-3d / -stand-3d / -pile-2d (ragdolls fitted from a generated skeleton and skinned mesh).
internal static class AuthoringStability
{
    private static string F(double v) => v.ToString("R", CultureInfo.InvariantCulture);

    internal static int Run(Opts opts)
    {
        int threads = int.Parse(opts.Get("threads", "1"), CultureInfo.InvariantCulture);
        string only = opts.Get("scene", "");
        int fails = 0;
        if (only.Length == 0 || only == "tet-drop-3d")
        {
            var m = TetDropScene.Run(600, threads);
            Console.WriteLine($"tet-drop-3d steps=600 threads={threads} nan={F(m.Nan)} escapes={F(m.Escapes)} maxSpeed={F(m.MaxSpeed)} invertedDuring={m.InvertedTetrahedra} invertedAtEnd={m.FinalInverted} minVolumeRatio={F(m.MinVolumeRatio)} maxVolumeRatio={F(m.MaxVolumeRatio)} lowest={F(m.LowestPoint)} " + (m.Passes ? "PASS" : "FAIL"));
            fails += m.Passes ? 0 : 1;
        }

        if (only.Length == 0 || only == "tri-drop-2d")
        {
            var m = TriDropScene2D.Run();
            Console.WriteLine($"tri-drop-2d steps=600 nan={F(m.Nan)} escapes={F(m.Escapes)} maxSpeed={F(m.MaxSpeed)} invertedDuring={m.InvertedTriangles} invertedAtEnd={m.FinalInverted} minAreaRatio={F(m.MinAreaRatio)} maxAreaRatio={F(m.MaxAreaRatio)} lowest={F(m.LowestPoint)} minAngle={F(m.MinAngle)} " + (m.Passes ? "PASS" : "FAIL"));
            fails += m.Passes ? 0 : 1;
        }

        if (only.Length == 0 || only == "autoragdoll-pile-3d")
        {
            var m = AutoRagdollScenes.RunPile3D(600, threads);
            Console.WriteLine($"autoragdoll-pile-3d steps=600 threads={threads} nan={F(m.Nan)} escapes={F(m.Escapes)} maxSpeed={F(m.MaxSpeed)} finalSpeed95={F(m.FinalSpeed95)} jointError={F(m.JointError)} lowestPelvis={F(m.LowestPelvis)} " + (m.Passes(false) ? "PASS" : "FAIL"));
            fails += m.Passes(false) ? 0 : 1;
        }

        if (only.Length == 0 || only == "autoragdoll-stand-3d")
        {
            var m = AutoRagdollScenes.RunStand3D();
            Console.WriteLine($"autoragdoll-stand-3d steps=600 nan={F(m.Nan)} escapes={F(m.Escapes)} maxSpeed={F(m.MaxSpeed)} jointError={F(m.JointError)} falls={m.Falls} heightError={F(m.MaxHeightError)} " + (m.Passes(true) ? "PASS" : "FAIL"));
            fails += m.Passes(true) ? 0 : 1;
        }

        if (only.Length == 0 || only == "autoragdoll-pile-2d")
        {
            var m = AutoRagdollScenes.RunPile2D(600, threads);
            Console.WriteLine($"autoragdoll-pile-2d steps=600 threads={threads} nan={F(m.Nan)} escapes={F(m.Escapes)} maxSpeed={F(m.MaxSpeed)} finalSpeed95={F(m.FinalSpeed95)} jointError={F(m.JointError)} lowestPelvis={F(m.LowestPelvis)} " + (m.Passes(false) ? "PASS" : "FAIL"));
            fails += m.Passes(false) ? 0 : 1;
        }

        return fails;
    }

    // Wall time of the tools on the generated blob and polygon (best of five, warm), at the thread counts asked for.
    internal static int Timing(Opts opts)
    {
        var blob = TetBlob.Build(80, 96);
        foreach (int threads in new[] { 1, 8 })
        {
            foreach (float edge in new[] { 0.1f, 0.05f })
            {
                double best = double.MaxValue; int tets = 0;
                for (int rep = 0; rep < 6; rep++)
                {
                    var sw = Stopwatch.StartNew();
                    var t = Tetrahedralizer.Generate(blob.V, blob.I, new TetrahedralizeOptions { TargetEdgeLength = edge, MaxThreads = threads });
                    sw.Stop();
                    tets = t.Tetrahedra.Length / 4;
                    if (rep > 0) { best = Math.Min(best, sw.Elapsed.TotalMilliseconds); }
                }

                Console.WriteLine($"tetrahedralize blob triangles={blob.I.Length / 3} edge={edge} tets={tets} threads={threads} bestMs={F(best)}");
            }
        }

        var shapes = TriDropScene2D.Shapes();
        foreach (var (outline, holes) in shapes)
        {
            double best = double.MaxValue; int tris = 0;
            for (int rep = 0; rep < 6; rep++)
            {
                var sw = Stopwatch.StartNew();
                var r = TriangleMesher.Generate(outline, holes, new TriangleMeshOptions { TargetEdgeLength = 0.03f });
                sw.Stop();
                tris = r.Triangles.Length / 3;
                if (rep > 0) { best = Math.Min(best, sw.Elapsed.TotalMilliseconds); }
            }

            Console.WriteLine($"triangulate2d points={outline.Length} triangles={tris} bestMs={F(best)}");
        }

        var sk = TestSkeleton.Humanoid();
        {
            double best = double.MaxValue;
            for (int rep = 0; rep < 20; rep++)
            {
                var sw = Stopwatch.StartNew();
                AutoRagdoller.Create(sk.Names, sk.Parents, sk.Positions, sk.Skin);
                sw.Stop();
                if (rep > 0) { best = Math.Min(best, sw.Elapsed.TotalMilliseconds); }
            }

            Console.WriteLine($"autoragdoll points={sk.Points.Length} bones={sk.Names.Length} bestMs={F(best)}");
        }

        return 0;
    }
}
