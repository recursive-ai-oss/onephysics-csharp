using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using Xunit;

namespace RecursiveAI.OnePhysics.Tools.Tests;

public class TetrahedralizerTests
{
    private static double Det6(Vector3F[] v, int[] t, int k)
    {
        Vector3D P(int i) => new(v[t[4 * k + i]].X, v[t[4 * k + i]].Y, v[t[4 * k + i]].Z);
        var a = P(0); var b = P(1); var c = P(2); var d = P(3);
        double ux = b.X - a.X, uy = b.Y - a.Y, uz = b.Z - a.Z, vx = c.X - a.X, vy = c.Y - a.Y, vz = c.Z - a.Z, wx = d.X - a.X, wy = d.Y - a.Y, wz = d.Z - a.Z;
        return wx * (uy * vz - uz * vy) + wy * (uz * vx - ux * vz) + wz * (ux * vy - uy * vx);
    }

    // Circumradius over shortest edge, from the linear system for the circumcentre solved by Cramer's rule (a different route from the library's).
    private static double RadiusEdge(Vector3F[] v, int[] t, int k)
    {
        var p = new double[4][];
        for (int i = 0; i < 4; i++) { var q = v[t[4 * k + i]]; p[i] = new double[] { q.X, q.Y, q.Z }; }
        var m = new double[3, 3]; var rhs = new double[3];
        for (int r = 0; r < 3; r++)
        {
            double n2 = 0;
            for (int c = 0; c < 3; c++) { m[r, c] = 2 * (p[r + 1][c] - p[0][c]); n2 += p[r + 1][c] * p[r + 1][c] - p[0][c] * p[0][c]; }
            rhs[r] = n2;
        }

        double D(double[,] a) => a[0, 0] * (a[1, 1] * a[2, 2] - a[1, 2] * a[2, 1]) - a[0, 1] * (a[1, 0] * a[2, 2] - a[1, 2] * a[2, 0]) + a[0, 2] * (a[1, 0] * a[2, 1] - a[1, 1] * a[2, 0]);
        double det = D(m);
        var x = new double[3];
        for (int c = 0; c < 3; c++)
        {
            var mc = (double[,])m.Clone();
            for (int r = 0; r < 3; r++) { mc[r, c] = rhs[r]; }
            x[c] = D(mc) / det;
        }

        double R = Math.Sqrt((x[0] - p[0][0]) * (x[0] - p[0][0]) + (x[1] - p[0][1]) * (x[1] - p[0][1]) + (x[2] - p[0][2]) * (x[2] - p[0][2]));
        double lmin = double.MaxValue;
        for (int i = 0; i < 4; i++) { for (int j = i + 1; j < 4; j++) { lmin = Math.Min(lmin, Math.Sqrt(Math.Pow(p[i][0] - p[j][0], 2) + Math.Pow(p[i][1] - p[j][1], 2) + Math.Pow(p[i][2] - p[j][2], 2))); } }
        return R / lmin;
    }

    private static double SegmentDistance(double[] a, double[] b, double[] p)
    {
        double[] ab = { b[0] - a[0], b[1] - a[1], b[2] - a[2] };
        double t = ((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1] + (p[2] - a[2]) * ab[2]) / (ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2]);
        t = Math.Max(0, Math.Min(1, t));
        return Math.Sqrt(Math.Pow(a[0] + t * ab[0] - p[0], 2) + Math.Pow(a[1] + t * ab[1] - p[1], 2) + Math.Pow(a[2] + t * ab[2] - p[2], 2));
    }

    // Distance from a point to the triangle soup by brute force: the plane distance when the projection falls inside a triangle, else the distance to its nearest edge.
    private static double DistanceToSurface(Vector3F[] sv, int[] si, double px, double py, double pz)
    {
        double best = double.MaxValue;
        double[] p = { px, py, pz };
        for (int t = 0; t < si.Length / 3; t++)
        {
            double[] a = { sv[si[3 * t]].X, sv[si[3 * t]].Y, sv[si[3 * t]].Z }, b = { sv[si[3 * t + 1]].X, sv[si[3 * t + 1]].Y, sv[si[3 * t + 1]].Z }, c = { sv[si[3 * t + 2]].X, sv[si[3 * t + 2]].Y, sv[si[3 * t + 2]].Z };
            best = Math.Min(best, Math.Min(SegmentDistance(a, b, p), Math.Min(SegmentDistance(b, c, p), SegmentDistance(c, a, p))));
            double[] u = { b[0] - a[0], b[1] - a[1], b[2] - a[2] }, v = { c[0] - a[0], c[1] - a[1], c[2] - a[2] };
            double[] n = { u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0] };
            double nn = n[0] * n[0] + n[1] * n[1] + n[2] * n[2];
            if (nn == 0) { continue; }
            double dist = ((p[0] - a[0]) * n[0] + (p[1] - a[1]) * n[1] + (p[2] - a[2]) * n[2]) / Math.Sqrt(nn);
            double[] q = { p[0] - dist * n[0] / Math.Sqrt(nn), p[1] - dist * n[1] / Math.Sqrt(nn), p[2] - dist * n[2] / Math.Sqrt(nn) };
            double[] w = { q[0] - a[0], q[1] - a[1], q[2] - a[2] };
            double uu = u[0] * u[0] + u[1] * u[1] + u[2] * u[2], uv = u[0] * v[0] + u[1] * v[1] + u[2] * v[2], vv = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
            double wu = w[0] * u[0] + w[1] * u[1] + w[2] * u[2], wv = w[0] * v[0] + w[1] * v[1] + w[2] * v[2];
            double den = uv * uv - uu * vv, s1 = (uv * wv - vv * wu) / den, t1 = (uv * wu - uu * wv) / den;
            if (s1 >= 0 && t1 >= 0 && s1 + t1 <= 1) { best = Math.Min(best, Math.Abs(dist)); }
        }

        return best;
    }

    public static IEnumerable<object[]> Shapes()
    {
        yield return new object[] { "sphere", 0.2f };
        yield return new object[] { "torus", 0.2f };
        yield return new object[] { "blob", 0.4f };
    }

    private static TestMesh Make(string name) => name switch
    {
        "sphere" => TestMeshes.Sphere(1, 24, 32),
        "torus" => TestMeshes.Torus(1, 0.35f, 32, 16),
        _ => TetBlob.Build(40, 48),
    };

    [Theory]
    [MemberData(nameof(Shapes))]
    public void TetrahedraAreWellShapedAndFillTheSurface(string name, float cell)
    {
        var m = Make(name);
        var t = Tetrahedralizer.Generate(m.V, m.I, new TetrahedralizeOptions { TargetEdgeLength = cell });
        int n = t.Tetrahedra.Length / 4;
        Assert.True(n > 500);
        double vol = 0, worst = 0;
        for (int k = 0; k < n; k++)
        {
            double d6 = Det6(t.Vertices, t.Tetrahedra, k);
            Assert.True(d6 > 0, "tetrahedron " + k + " has volume " + d6 / 6);
            vol += d6 / 6;
            worst = Math.Max(worst, RadiusEdge(t.Vertices, t.Tetrahedra, k));
        }

        Assert.InRange(vol / t.MeshVolume, 0.93, 1.001);
        Assert.Equal(vol, t.Volume, 6);
        Assert.True(worst <= 2.0001, "radius-edge ratio " + worst);
        Assert.True(t.WorstRadiusEdge <= 2.0001 && t.QualityMet);
        Assert.True(t.MinVolumeRatio >= 0.3 - 1e-9, "volume ratio " + t.MinVolumeRatio);

        // The boundary conforms to the input: every output boundary vertex lies within 0.6 cells of the input surface and every input vertex within 0.6 cells of the boundary.
        var bnd = new HashSet<int>(t.SurfaceTriangles);
        double far = 0;
        foreach (int vi in bnd) { far = Math.Max(far, DistanceToSurface(m.V, m.I, t.Vertices[vi].X, t.Vertices[vi].Y, t.Vertices[vi].Z)); }
        Assert.True(far <= 0.6 * cell, "boundary vertex " + far + " from the input surface (cell " + cell + ")");
        Assert.True(t.SurfaceDeviation <= 0.6 * cell, "deviation " + t.SurfaceDeviation);
        Assert.True(t.SurfaceDeviation + 1e-9 >= far);
        double far2 = 0;
        for (int i = 0; i < m.V.Length; i++) { far2 = Math.Max(far2, DistanceToSurface(t.Vertices, t.SurfaceTriangles, m.V[i].X, m.V[i].Y, m.V[i].Z)); }
        Assert.True(far2 <= 0.6 * cell, "input vertex " + far2 + " from the boundary");
    }

    [Fact]
    public void BoundaryIsAClosedOutwardSurfaceOfTheSameVolume()
    {
        var m = TetBlob.Build(40, 48);
        var t = Tetrahedralizer.Generate(m.V, m.I, new TetrahedralizeOptions { TargetEdgeLength = 0.4f });
        var edges = new Dictionary<long, int>();
        var tri = t.SurfaceTriangles;
        for (int f = 0; f < tri.Length / 3; f++)
        {
            for (int k = 0; k < 3; k++)
            {
                long a = tri[3 * f + k], b = tri[3 * f + (k + 1) % 3];
                edges[(a << 32) | b] = edges.GetValueOrDefault((a << 32) | b) + 1;
            }
        }

        foreach (var (key, count) in edges)
        {
            long a = key >> 32, b = key & 0xFFFFFFFF;
            Assert.Equal(1, count);
            Assert.True(edges.ContainsKey((b << 32) | a), "edge " + a + "-" + b + " has no opposite");
        }

        Assert.Equal(t.Volume, Math.Abs(MeshMass.SignedVolume(t.Vertices, tri)), 6);
        Assert.True(MeshMass.SignedVolume(t.Vertices, tri) > 0, "boundary faces outward");
        Assert.True(MeshMass.IsClosed(t.Vertices, tri));
    }

    [Fact]
    public void BindingsPlaceInputVerticesOnTheBoundary()
    {
        var m = TestMeshes.Torus(1, 0.35f, 32, 16);
        var t = Tetrahedralizer.Generate(m.V, m.I, new TetrahedralizeOptions { TargetEdgeLength = 0.2f });
        Assert.Equal(m.V.Length, t.Bindings.Length);
        for (int i = 0; i < m.V.Length; i++)
        {
            var b = t.Bindings[i];
            Assert.InRange(b.W0 + b.W1 + b.W2 + b.W3, 0.9999f, 1.0001f);
            Assert.True(b.W0 >= -1e-6f && b.W1 >= -1e-6f && b.W2 >= -1e-6f && b.W3 >= -1e-6f);
            double x = 0, y = 0, z = 0;
            float[] w = { b.W0, b.W1, b.W2, b.W3 };
            for (int c = 0; c < 4; c++) { var p = t.Vertices[t.Tetrahedra[4 * b.Tetrahedron + c]]; x += w[c] * p.X; y += w[c] * p.Y; z += w[c] * p.Z; }
            double d = Math.Sqrt((x - m.V[i].X) * (x - m.V[i].X) + (y - m.V[i].Y) * (y - m.V[i].Y) + (z - m.V[i].Z) * (z - m.V[i].Z));
            Assert.True(d <= t.SurfaceDeviation + 1e-6, "binding " + i + " is " + d + " from its vertex");
        }
    }

    [Fact]
    public void SizeOptionsRefine()
    {
        var m = TestMeshes.Sphere(1, 24, 32);
        var coarse = Tetrahedralizer.Generate(m.V, m.I, new TetrahedralizeOptions { TargetEdgeLength = 0.4f });
        var fine = Tetrahedralizer.Generate(m.V, m.I, new TetrahedralizeOptions { TargetEdgeLength = 0.2f });
        Assert.InRange((double)(fine.Tetrahedra.Length / 4) / (coarse.Tetrahedra.Length / 4), 6.0, 10.0);
        var byVolume = Tetrahedralizer.Generate(m.V, m.I, new TetrahedralizeOptions { MaxVolume = 0.2f * 0.2f * 0.2f / 12f * 1.0001f });
        Assert.Equal(0.2, byVolume.CellSize, 4);
        var byBoth = Tetrahedralizer.Generate(m.V, m.I, new TetrahedralizeOptions { TargetEdgeLength = 0.4f, MaxVolume = 0.2f * 0.2f * 0.2f / 12f * 1.0001f });
        Assert.Equal(0.2, byBoth.CellSize, 4);
        Assert.Equal(fine.Tetrahedra.Length, byBoth.Tetrahedra.Length);
        var def = Tetrahedralizer.Generate(m.V, m.I);
        Assert.InRange(def.CellSize, 2.0 / 16 - 1e-6, 2.0 / 16 + 1e-6);
    }

    [Fact]
    public void SameBitsAtEveryThreadCountAndRepeat()
    {
        var m = TetBlob.Build(40, 48);
        static ulong Fold(TetrahedralMesh t)
        {
            ulong h = 14695981039346656037UL;
            void Mix(uint v) { h = (h ^ v) * 1099511628211UL; }
            foreach (var v in t.Vertices) { Mix((uint)BitConverter.SingleToInt32Bits(v.X)); Mix((uint)BitConverter.SingleToInt32Bits(v.Y)); Mix((uint)BitConverter.SingleToInt32Bits(v.Z)); }
            foreach (int i in t.Tetrahedra) { Mix((uint)i); }
            foreach (int i in t.SurfaceTriangles) { Mix((uint)i); }
            foreach (var b in t.Bindings) { Mix((uint)b.Tetrahedron); Mix((uint)BitConverter.SingleToInt32Bits(b.W0)); Mix((uint)BitConverter.SingleToInt32Bits(b.W3)); }
            Mix((uint)BitConverter.DoubleToInt64Bits(t.SurfaceDeviation));
            return h;
        }

        ulong one = Fold(Tetrahedralizer.Generate(m.V, m.I, new TetrahedralizeOptions { TargetEdgeLength = 0.3f, MaxThreads = 1 }));
        Assert.Equal(one, Fold(Tetrahedralizer.Generate(m.V, m.I, new TetrahedralizeOptions { TargetEdgeLength = 0.3f, MaxThreads = 8 })));
        Assert.Equal(one, Fold(Tetrahedralizer.Generate(m.V, m.I, new TetrahedralizeOptions { TargetEdgeLength = 0.3f, MaxThreads = 3 })));
        Assert.Equal(one, Fold(Tetrahedralizer.Generate(m.V, m.I, new TetrahedralizeOptions { TargetEdgeLength = 0.3f })));
    }

    [Fact]
    public void ABoxFillsExactlyEvenWithFlatFacesOnTheLattice()
    {
        // Flat faces that coincide with lattice planes (the lattice origin is irregular, but this checks the exact inside tests on a mesh made of axis-aligned quads).
        var m = TestMeshes.Box(2, 2, 2);
        var t = Tetrahedralizer.Generate(m.V, m.I, new TetrahedralizeOptions { TargetEdgeLength = 0.25f });
        Assert.InRange(t.Volume / 8.0, 0.97, 1.0001);
        Assert.True(t.SurfaceDeviation <= 0.6 * 0.25);
    }

    [Fact]
    public void HostileInputsFailWithTypedErrors()
    {
        var box = TestMeshes.Box(1, 1, 1);
        GeometryToolError Err(Vector3F[] v, int[] i, TetrahedralizeOptions o = default)
        {
            var ex = Assert.Throws<GeometryToolException>(() => Tetrahedralizer.Generate(v, i, o));
            Assert.False(string.IsNullOrEmpty(ex.Message));
            return ex.Error;
        }

        Assert.Equal(GeometryToolError.Empty, Err(Array.Empty<Vector3F>(), Array.Empty<int>()));
        Assert.Equal(GeometryToolError.Empty, Err(box.V, Array.Empty<int>()));
        Assert.Equal(GeometryToolError.BadIndex, Err(box.V, new[] { 0, 1 }));
        Assert.Equal(GeometryToolError.BadIndex, Err(box.V, new[] { 0, 1, 99 }));
        var nan = box.V; nan[3] = new Vector3F(float.NaN, 0, 0);
        Assert.Equal(GeometryToolError.NonFinite, Err(nan, box.I));
        var huge = box.V; huge[3] = new Vector3F(1e30f, 0, 0);
        Assert.Equal(GeometryToolError.OutOfRange, Err(huge, box.I));
        var open = box.I.AsSpan(0, box.I.Length - 3).ToArray();
        Assert.Equal(GeometryToolError.OpenMesh, Err(box.V, open));
        var flipped = (int[])box.I.Clone();
        for (int k = 0; k < flipped.Length; k += 3) { (flipped[k + 1], flipped[k + 2]) = (flipped[k + 2], flipped[k + 1]); }
        Assert.Equal(GeometryToolError.InvertedWinding, Err(box.V, flipped));
        Assert.Equal(GeometryToolError.InvalidOptions, Err(box.V, box.I, new TetrahedralizeOptions { TargetEdgeLength = -1f }));
        Assert.Equal(GeometryToolError.InvalidOptions, Err(box.V, box.I, new TetrahedralizeOptions { TargetEdgeLength = float.NaN }));
        Assert.Equal(GeometryToolError.InvalidOptions, Err(box.V, box.I, new TetrahedralizeOptions { MaxVolume = -2f }));
        Assert.Equal(GeometryToolError.InvalidOptions, Err(box.V, box.I, new TetrahedralizeOptions { MaxRadiusEdgeRatio = 0.5f }));
        Assert.Equal(GeometryToolError.InvalidOptions, Err(box.V, box.I, new TetrahedralizeOptions { MaxThreads = 1000 }));
        Assert.Equal(GeometryToolError.InvalidOptions, Err(box.V, box.I, new TetrahedralizeOptions { TargetEdgeLength = 1e-4f }));
        // A slab thinner than the cell: nothing fits.
        var slab = TestMeshes.Box(10, 10, 0.01f);
        Assert.Equal(GeometryToolError.Degenerate, Err(slab.V, slab.I, new TetrahedralizeOptions { TargetEdgeLength = 2f }));
    }

    [Fact]
    public void SoftBodiesMadeFromTheOutputStayStable()
    {
        var m = TetDropScene.Run();
        Assert.True(m.Passes, m.ToString());
    }

    [Fact]
    public void ASoftBodyMeshComesOutOfTheResult()
    {
        var m = TestMeshes.Sphere(1, 16, 24);
        var t = Tetrahedralizer.Generate(m.V, m.I, new TetrahedralizeOptions { TargetEdgeLength = 0.3f });
        var s = t.ToSoftBodyMesh();
        Assert.Equal(t.Tetrahedra.Length / 4, s.TetrahedronCount);
        Assert.Equal(t.Vertices.Length, s.VertexCount);
    }
}
