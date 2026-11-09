using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Tools;

// Targets for the authoring tools. Every input either is accepted or fails with a GeometryToolException (or HullBuildException from the engine's hull builder); any other exception is a finding.
internal static class FuzzTools
{
    private sealed class Reader
    {
        private readonly byte[] _b;
        private int _p;
        public Reader(byte[] b) => _b = b;
        public byte Byte() => _p < _b.Length ? _b[_p++] : (byte)0;
        public int Int(int n) => Byte() % n;

        // Mostly modest numbers, sometimes raw bits (NaN, infinity, huge, subnormal).
        public float Float()
        {
            byte mode = Byte();
            if (mode < 24 && _p + 4 <= _b.Length) { float f = BitConverter.ToSingle(_b, _p); _p += 4; return f; }
            return (Byte() - 128) / 16f + Byte() / 4096f;
        }
    }

    private static void Mesh(Reader r, out Vector3F[] v, out int[] idx)
    {
        int nv = 1 + r.Int(24), nt = r.Int(40);
        v = new Vector3F[nv];
        for (int i = 0; i < nv; i++) { v[i] = new Vector3F(r.Float(), r.Float(), r.Float()); }
        idx = new int[3 * nt + (r.Int(8) == 0 ? 1 : 0)];
        for (int i = 0; i < idx.Length; i++) { idx[i] = r.Int(nv + 1) - (r.Int(32) == 0 ? 1 : 0); }
    }

    private static void Guard(Action a)
    {
        try { a(); }
        catch (GeometryToolException) { }
        catch (HullBuildException) { }
    }

    internal static void MeshClean(byte[] input)
    {
        var r = new Reader(input);
        Mesh(r, out var v, out var idx);
        var opt = new MeshCleanOptions { WeldTolerance = r.Byte() < 64 ? r.Float() : 0f, KeepDegenerateTriangles = (r.Byte() & 1) != 0, KeepWinding = (r.Byte() & 1) != 0 };
        Guard(() =>
        {
            var c = MeshCleaner.Clean(v, idx, opt);
            if (opt.WeldTolerance == 0f && !opt.KeepDegenerateTriangles && !opt.KeepWinding)
            {
                var again = MeshCleaner.Clean(c.Vertices, c.Indices, opt);
                if (!again.Indices.AsSpan().SequenceEqual(c.Indices) || again.Vertices.Length != c.Vertices.Length) { throw new InvalidOperationException("Cleaning a cleaned mesh changed it."); }
            }
        });
    }

    internal static void MeshMassTarget(byte[] input)
    {
        var r = new Reader(input);
        Mesh(r, out var v, out var idx);
        Guard(() => { MeshMass.Compute(v, idx); });
        Guard(() => { MeshMass.IsClosed(v, idx); });
    }

    internal static void HullSimplify(byte[] input)
    {
        var r = new Reader(input);
        int n = 4 + r.Int(40);
        var pts = new Vector3F[n];
        for (int i = 0; i < n; i++) { pts[i] = new Vector3F(r.Float(), r.Float(), r.Float()); }
        var opt = new HullSimplifyOptions { MaxVertices = r.Byte() < 128 ? r.Int(30) : 0, MaxFaces = r.Byte() < 64 ? r.Int(30) : 0, MaxError = r.Byte() < 128 ? Math.Abs(r.Float()) : 0f };
        Guard(() =>
        {
            var hull = ConvexHull3D.Build(pts);
            var s = HullSimplifier.Simplify(hull, opt);
            if (s.Hull.Vertices.Length > hull.Vertices.Length) { throw new InvalidOperationException("Simplification added vertices."); }
        });
    }

    internal static void Decompose3D(byte[] input)
    {
        var r = new Reader(input);
        Mesh(r, out var v, out var idx);
        var opt = new ConvexDecompositionOptions { MaxHulls = r.Byte() < 200 ? 1 + r.Int(6) : 0, Resolution = r.Byte() < 200 ? 8 + r.Int(8) : 0, MaxConcavity = r.Byte() < 40 ? r.Float() : 0f, MaxVerticesPerHull = r.Byte() < 40 ? r.Int(10) : 0 };
        Guard(() =>
        {
            var d = ConvexDecomposer.Decompose(v, idx, opt);
            if (d.Hulls.Count == 0) { throw new InvalidOperationException("Decomposition returned no hulls."); }
        });
    }

    internal static void Polygon(byte[] input)
    {
        var r = new Reader(input);
        Vector2F[] Loop(int max)
        {
            int n = r.Int(max);
            var p = new Vector2F[n];
            for (int i = 0; i < n; i++) { p[i] = new Vector2F(r.Float(), r.Float()); }
            return p;
        }

        var outline = Loop(14);
        var holes = new Vector2F[r.Int(3)][];
        for (int i = 0; i < holes.Length; i++) { holes[i] = Loop(6); }
        var opt = new PolygonDecompositionOptions { MaxVerticesPerPiece = r.Byte() < 128 ? r.Int(10) : 0 };
        Guard(() =>
        {
            var pieces = PolygonDecomposer.Decompose(outline, holes, opt);
            foreach (var p in pieces) { if (p.Length < 3) { throw new InvalidOperationException("A piece has fewer than 3 vertices."); } }
        });
    }

    internal static void Sprite(byte[] input)
    {
        var r = new Reader(input);
        int w = 1 + r.Int(24), h = 1 + r.Int(24);
        var alpha = new byte[w * h + (r.Int(16) == 0 ? 1 : 0)];
        for (int i = 0; i < alpha.Length; i++) { alpha[i] = r.Byte() < 128 ? (byte)255 : (byte)0; }
        var opt = new SpriteOutlineOptions { AlphaThreshold = r.Byte() < 200 ? (byte)(1 + r.Int(255)) : (byte?)null, Tolerance = r.Byte() < 128 ? Math.Abs(r.Float()) : null, MinArea = r.Byte() < 64 ? Math.Abs(r.Float()) : null };
        Guard(() =>
        {
            foreach (var island in SpriteOutliner.Trace(alpha, w, h, opt)) { island.ToConvexPieces(); }
        });
    }
}
