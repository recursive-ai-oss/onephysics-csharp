#pragma warning disable OPHYS0002, OPHYS0003
using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Tools;
using RecursiveAI.OnePhysics.Tools.Tests;

// The authoring tools under NativeAOT and WebAssembly: each runs once and the digest of their outputs is printed, so the smoke apps' output can be compared with the JIT's.
internal static class ToolsSmoke
{
    internal static bool Run(out ulong digest)
    {
        ulong h = 14695981039346656037UL;
        void Mix(float f) { h = (h ^ (uint)BitConverter.SingleToInt32Bits(f)) * 1099511628211UL; }
        void MixI(int i) { h = (h ^ (uint)i) * 1099511628211UL; }
        var l = TestMeshes.LBlock();
        var clean = MeshCleaner.Clean(l.V, l.I);
        MixI(clean.Indices.Length);
        var mass = MeshMass.Compute(clean.Vertices, clean.Indices);
        Mix(mass.Mass); Mix(mass.Center.X);
        var d = ConvexDecomposer.Decompose(l.V, l.I, new ConvexDecompositionOptions { Resolution = 16, MaxThreads = 3 });
        MixI(d.Hulls.Count);
        foreach (var hull in d.Hulls) { foreach (var v in hull.Vertices) { Mix(v.X); Mix(v.Y); Mix(v.Z); } }
        var s = TestMeshes.Sphere(1, 10, 12);
        var simple = HullSimplifier.Simplify(RecursiveAI.OnePhysics.ConvexHull3D.Build(s.V), new HullSimplifyOptions { MaxVertices = 20 });
        MixI(simple.Hull.Vertices.Length); Mix(simple.Error);
        var star = new Vector2F[10];
        for (int i = 0; i < 10; i++) { double a = Math.PI * i / 5; double r = (i & 1) == 0 ? 2.0 : 0.8; star[i] = new Vector2F((float)(r * Math64.Cos(a)), (float)(r * Math64.Sin(a))); }
        var pieces = PolygonDecomposer.Decompose(star);
        MixI(pieces.Length);
        foreach (var p in pieces) { foreach (var v in p) { Mix(v.X); Mix(v.Y); } }
        var mask = new byte[24 * 24];
        for (int y = 0; y < 24; y++) { for (int x = 0; x < 24; x++) { double dx = x - 11.5, dy = y - 11.5, r2 = dx * dx + dy * dy; mask[y * 24 + x] = r2 < 100 && r2 > 16 ? (byte)255 : (byte)0; } }
        var sprite = SpriteOutliner.ToConvexPieces(mask, 24, 24);
        MixI(sprite.Length);
        foreach (var p in sprite) { foreach (var v in p) { Mix(v.X); Mix(v.Y); } }
        // tetrahedralization, quality triangulation and the ragdoll fitter
        var blob = TetBlob.Build(24, 32);
        var tet = Tetrahedralizer.Generate(blob.V, blob.I, new TetrahedralizeOptions { TargetEdgeLength = 0.5f, MaxThreads = 3 });
        MixI(tet.Tetrahedra.Length); foreach (var v in tet.Vertices) { Mix(v.X); Mix(v.Y); Mix(v.Z); } Mix((float)tet.SurfaceDeviation);
        var mesh2 = TriangleMesher.Generate(star, new[] { new[] { new Vector2F(-0.3f, -0.3f), new Vector2F(-0.3f, 0.3f), new Vector2F(0.3f, 0.3f), new Vector2F(0.3f, -0.3f) } }, new TriangleMeshOptions { TargetEdgeLength = 0.4f });
        MixI(mesh2.Triangles.Length); foreach (var v in mesh2.Vertices) { Mix(v.X); Mix(v.Y); }
        var sk = TestSkeleton.Humanoid();
        var rag = AutoRagdoller.Create(sk.Names, sk.Parents, sk.Positions, sk.Skin);
        MixI(rag.Definition.Parts.Length);
        foreach (var part in rag.Definition.Parts) { Mix(part.Mass); Mix(part.Position.X); Mix(part.Position.Y); Mix(part.ShapeHalfSize.X); Mix(part.ShapeHalfSize.Y); Mix(part.JointRotation.W); }
        var rag2 = AutoRagdoller.Create2D(sk.Names, sk.Parents, sk.Positions2D(), sk.Skin2D());
        foreach (var part in rag2.Definition.Parts) { Mix(part.Mass); Mix(part.Angle); Mix(part.ShapeHalfSize.Y); }
        digest = h;
        return d.Hulls.Count == 2 && pieces.Length >= 2 && sprite.Length >= 3 && simple.Hull.Vertices.Length <= 20 && tet.Tetrahedra.Length > 100 && mesh2.Triangles.Length > 30 && rag.Definition.Parts.Length == 13 && rag2.Definition.Parts.Length == 13;
    }
}
