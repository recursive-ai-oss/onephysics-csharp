using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Tools;
using RecursiveAI.OnePhysics.Tools.Tests;

namespace RecursiveAI.OnePhysics.HashMatrix;

/// <summary>
/// The authoring tools: each step is one tool run on a generated input, hashing every output bit. Decomposition runs at the runner's thread count, and its bytes must not depend on it.
/// </summary>
internal static class ToolsScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("tools/mesh", MeshTools);
        Scenarios.Register("tools/decompose3d", Decompose3D);
        Scenarios.Register("tools/polygon", Polygon);
        Scenarios.Register("tools/sprite", Sprite);
        Scenarios.Register("tools/decomposed-pile-3d", Pile);
    }

    private static void WriteHull(ScenarioContext ctx, ConvexHull3D h)
    {
        ctx.Write(h.Vertices.Length);
        foreach (var v in h.Vertices) { ctx.Write(v.X); ctx.Write(v.Y); ctx.Write(v.Z); }
        ctx.Write(h.Faces.Length);
        ctx.Write(h.MassProperties.Mass);
    }

    private static void MeshTools(ScenarioContext ctx)
    {
        var meshes = new[] { TestMeshes.Box(1, 2, 3), TestMeshes.Sphere(1.5f, 12, 16), TestMeshes.Torus(1, 0.3f, 24, 8), TestMeshes.Mug(), TestMeshes.LBlock() };
        foreach (var m in meshes)
        {
            var c = MeshCleaner.Clean(m.V, m.I, new MeshCleanOptions { WeldTolerance = 1e-5f });
            ctx.Write(c.Vertices.Length); ctx.Write(c.Indices.Length);
            foreach (var v in c.Vertices) { ctx.Write(v.X); ctx.Write(v.Y); ctx.Write(v.Z); }
            foreach (int i in c.Indices) { ctx.Write(i); }
            ctx.Write(c.Report.Components); ctx.Write(c.Report.FlippedTriangles);
            var mp = MeshMass.ComputeDouble(c.Vertices, c.Indices);
            ctx.Write(mp.Mass); ctx.Write(mp.Center.X); ctx.Write(mp.Center.Y); ctx.Write(mp.Center.Z);
            ctx.Write(mp.Inertia.M11); ctx.Write(mp.Inertia.M22); ctx.Write(mp.Inertia.M33); ctx.Write(mp.Inertia.M12); ctx.Write(mp.Inertia.M13); ctx.Write(mp.Inertia.M23);
            ctx.EndStep();
        }

        var hull = ConvexHull3D.Build(meshes[1].V);
        foreach (var opt in new[] { new HullSimplifyOptions { MaxVertices = 24 }, new HullSimplifyOptions { MaxError = 0.05f }, new HullSimplifyOptions { MaxFaces = 30 } })
        {
            var s = HullSimplifier.Simplify(hull, opt);
            WriteHull(ctx, s.Hull); ctx.Write(s.Error); ctx.Write(s.RemovedVertices);
            ctx.EndStep();
        }
    }

    private static void Decompose3D(ScenarioContext ctx)
    {
        var meshes = new[] { TestMeshes.LBlock(), TestMeshes.UBlock(), TestMeshes.Torus(1, 0.35f, 24, 10), TestMeshes.Mug() };
        foreach (var m in meshes)
        {
            var d = ConvexDecomposer.Decompose(m.V, m.I, new ConvexDecompositionOptions { MaxHulls = 16, Resolution = 24, MaxConcavity = 0.03f, MaxThreads = ctx.Threads });
            ctx.Write(d.Hulls.Count);
            foreach (var h in d.Hulls) { WriteHull(ctx, h); }
            foreach (float c in d.Concavities) { ctx.Write(c); }
            ctx.EndStep();
        }
    }

    private static void Polygon(ScenarioContext ctx)
    {
        void Write(Vector2F[][] pieces)
        {
            ctx.Write(pieces.Length);
            foreach (var p in pieces) { ctx.Write(p.Length); foreach (var v in p) { ctx.Write(v.X); ctx.Write(v.Y); } }
            ctx.EndStep();
        }

        var star = new Vector2F[14];
        for (int i = 0; i < 14; i++) { double a = Math.PI * i / 7; double r = (i & 1) == 0 ? 2.0 : 0.8; star[i] = new Vector2F((float)(r * Math64.Cos(a)), (float)(r * Math64.Sin(a))); }
        Write(PolygonDecomposer.Decompose(star));
        Write(PolygonDecomposer.Decompose(star, new PolygonDecompositionOptions { MaxVerticesPerPiece = 3 }));
        var outer = new Vector2F[40]; var hole = new Vector2F[40];
        for (int i = 0; i < 40; i++) { double a = 2 * Math.PI * i / 40; outer[i] = new Vector2F((float)(5 * Math64.Cos(a)), (float)(5 * Math64.Sin(a))); hole[i] = new Vector2F((float)(3 * Math64.Cos(a)), (float)(3 * Math64.Sin(a))); }
        Write(PolygonDecomposer.Decompose(outer, new[] { hole }));
    }

    private static void Sprite(ScenarioContext ctx)
    {
        const int n = 72;
        var mask = new byte[n * n];
        for (int y = 0; y < n; y++)
        {
            for (int x = 0; x < n; x++)
            {
                double dx = x + 0.5 - 36, dy = y + 0.5 - 36, r2 = dx * dx + dy * dy;
                bool ring = r2 <= 30 * 30 && r2 >= 12 * 12;
                bool dot = (x - 36) * (x - 36) + (y - 36) * (y - 36) <= 16;
                // Soft edge so the contour interpolation is exercised.
                int a = ring || dot ? 255 : 0;
                if ((x * 7 + y * 13) % 11 == 0 && a == 255) { a = 140; }
                mask[y * n + x] = (byte)a;
            }
        }

        foreach (float tol in new[] { 0f, 1f })
        {
            var islands = SpriteOutliner.Trace(mask, n, n, new SpriteOutlineOptions { Tolerance = tol });
            ctx.Write(islands.Length);
            foreach (var isl in islands)
            {
                ctx.Write(isl.Outer.Length); foreach (var v in isl.Outer) { ctx.Write(v.X); ctx.Write(v.Y); }
                ctx.Write(isl.Holes.Length); foreach (var h in isl.Holes) { ctx.Write(h.Length); foreach (var v in h) { ctx.Write(v.X); ctx.Write(v.Y); } }
                var pieces = isl.ToConvexPieces();
                ctx.Write(pieces.Length);
                foreach (var p in pieces) { ctx.Write(p.Length); foreach (var v in p) { ctx.Write(v.X); ctx.Write(v.Y); } }
            }

            ctx.EndStep();
        }
    }

    private static void Pile(ScenarioContext ctx)
    {
        var world = DecomposedPileScene.Build(out var bodies, 12, ctx.Threads);
        for (int step = 0; step < 240; step++)
        {
            world.Step(1f / 60f);
            ctx.Write(unchecked((long)world.ComputeHash()));
            if (step % 20 == 19) { foreach (var b in bodies) { var p = b.Position; ctx.Write(p.X); ctx.Write(p.Y); ctx.Write(p.Z); } }
            ctx.EndStep();
        }
    }
}
