using System;
using System.Collections.Generic;
using System.Linq;
using RecursiveAI.OneMath;
using Xunit;

namespace RecursiveAI.OnePhysics.Tools.Tests;

public class MeshCleanerTests
{
    private static void AssertSame(CleanedMesh a, CleanedMesh b)
    {
        Assert.Equal(a.Indices, b.Indices);
        Assert.Equal(a.Vertices.Length, b.Vertices.Length);
        for (int i = 0; i < a.Vertices.Length; i++)
        {
            Assert.Equal(BitConverter.SingleToInt32Bits(a.Vertices[i].X), BitConverter.SingleToInt32Bits(b.Vertices[i].X));
            Assert.Equal(BitConverter.SingleToInt32Bits(a.Vertices[i].Y), BitConverter.SingleToInt32Bits(b.Vertices[i].Y));
            Assert.Equal(BitConverter.SingleToInt32Bits(a.Vertices[i].Z), BitConverter.SingleToInt32Bits(b.Vertices[i].Z));
        }
    }

    [Fact]
    public void CleanMeshIsUnchanged()
    {
        var m = TestMeshes.Sphere(1, 12, 16);
        var c = MeshCleaner.Clean(m.V, m.I);
        Assert.Equal(m.I, c.Indices);
        Assert.Equal(0, c.Report.WeldedVertices + c.Report.DegenerateTrianglesRemoved + c.Report.DuplicateTrianglesRemoved + c.Report.FlippedTriangles);
        Assert.True(c.Report.IsClosed);
        Assert.Equal(1, c.Report.Components);
    }

    [Fact]
    public void SeededDefectsAreFixedAndCleaningIsIdempotent()
    {
        var m = TestMeshes.Box(2, 2, 2);
        var verts = new List<Vector3F>(m.Vertices);
        var tris = new List<int>(m.Indices);
        // Split every vertex into two copies a hair apart (weld), flip some triangles, duplicate one, add a degenerate and an unused vertex.
        var rnd = new Random(7);
        int nv = verts.Count;
        for (int i = 0; i < nv; i++) { verts.Add(new Vector3F(verts[i].X + 1e-7f, verts[i].Y, verts[i].Z - 1e-7f)); }
        for (int t = 0; t < tris.Count; t += 3)
        {
            for (int k = 0; k < 3; k++) { if (rnd.Next(2) == 0) { tris[t + k] += nv; } }
            if (t % 6 == 0) { (tris[t + 1], tris[t + 2]) = (tris[t + 2], tris[t + 1]); }
        }

        tris.AddRange(new[] { tris[0], tris[1], tris[2] }); // a duplicate of triangle 0 (identical)
        tris.AddRange(new[] { 0, 0, 1 });                    // repeated vertex
        tris.AddRange(new[] { 0, 1, nv });                   // sliver: nv is within 1e-7 of vertex 0
        verts.Add(new Vector3F(50, 50, 50));                 // unused

        var c = MeshCleaner.Clean(verts.ToArray(), tris.ToArray());
        Assert.Equal(8, c.Vertices.Length);
        Assert.Equal(12, c.Indices.Length / 3);
        Assert.True(c.Report.IsClosed);
        Assert.True(c.Report.DuplicateTrianglesRemoved >= 1);
        Assert.True(c.Report.DegenerateTrianglesRemoved >= 2);
        Assert.True(c.Report.FlippedTriangles > 0);
        Assert.Equal(1, c.Report.UnreferencedVerticesRemoved);
        // Outward and closed: positive volume of the box.
        Assert.Equal(8.0, MeshMass.ComputeDouble(c.Vertices, c.Indices).Mass, 5);
        // Idempotent, bit for bit.
        AssertSame(c, MeshCleaner.Clean(c.Vertices, c.Indices));
    }

    [Fact]
    public void InwardClosedSurfaceIsTurnedOutward()
    {
        var m = TestMeshes.Sphere(1, 10, 12);
        var idx = m.I;
        for (int t = 0; t < idx.Length; t += 3) { (idx[t + 1], idx[t + 2]) = (idx[t + 2], idx[t + 1]); }
        Assert.True(MeshMass.SignedVolume(m.V, idx) < 0);
        var c = MeshCleaner.Clean(m.V, idx);
        Assert.True(MeshMass.SignedVolume(c.Vertices, c.Indices) > 0);
        Assert.Equal(idx.Length / 3, c.Report.FlippedTriangles);
    }

    [Fact]
    public void HolesAndNonManifoldEdgesAreReported()
    {
        var m = TestMeshes.Box(1, 1, 1);
        // Remove the two triangles of the +z face: one hole.
        var idx = m.I.Take(m.I.Length - 6).ToArray();
        var c = MeshCleaner.Clean(m.V, idx);
        Assert.Equal(1, c.Report.HoleCount);
        Assert.Equal(4, c.Report.BoundaryEdgeCount);
        Assert.False(c.Report.IsClosed);

        // A fin: a third triangle on an edge of a closed box.
        var v = m.Vertices.ToList();
        int extra = v.Count;
        v.Add(new Vector3F(3, 3, 3));
        var fin = m.I.Concat(new[] { m.I[0], m.I[1], extra }).ToArray();
        var f = MeshCleaner.Clean(v.ToArray(), fin);
        Assert.Single(f.Report.NonManifoldEdges);
        Assert.Equal(3, f.Report.NonManifoldEdges[0].TriangleCount);
        Assert.False(f.Report.IsClosed);
    }

    [Fact]
    public void TwoSeparatePiecesAreTwoComponents()
    {
        var a = TestMeshes.Box(1, 1, 1);
        a.Append(TestMeshes.Box(1, 1, 1, 5, 0, 0));
        var c = MeshCleaner.Clean(a.V, a.I);
        Assert.Equal(2, c.Report.Components);
        Assert.Equal(16, c.Vertices.Length);
    }

    [Fact]
    public void HostileInputsGetTypedErrors()
    {
        var v = new[] { new Vector3F(0, 0, 0), new Vector3F(1, 0, 0), new Vector3F(0, 1, 0) };
        Assert.Equal(GeometryToolError.InvalidOptions, Assert.Throws<GeometryToolException>(() => MeshCleaner.Clean(v, new[] { 0, 1, 2 }, new MeshCleanOptions { WeldTolerance = float.NaN })).Error);
        Assert.Equal(GeometryToolError.InvalidOptions, Assert.Throws<GeometryToolException>(() => MeshCleaner.Clean(v, new[] { 0, 1, 2 }, new MeshCleanOptions { WeldTolerance = -1 })).Error);
        Assert.Equal(GeometryToolError.BadIndex, Assert.Throws<GeometryToolException>(() => MeshCleaner.Clean(v, new[] { 0, 1, -2 })).Error);
        var inf = new[] { new Vector3F(float.PositiveInfinity, 0, 0), new Vector3F(1, 0, 0), new Vector3F(0, 1, 0) };
        Assert.Equal(GeometryToolError.NonFinite, Assert.Throws<GeometryToolException>(() => MeshCleaner.Clean(inf, new[] { 0, 1, 2 })).Error);
        // Everything degenerate: the result is empty, not an error.
        var flat = MeshCleaner.Clean(new[] { new Vector3F(0, 0, 0), new Vector3F(1, 0, 0), new Vector3F(2, 0, 0) }, new[] { 0, 1, 2 });
        Assert.Empty(flat.Indices);
        // Tiny and huge scales still work.
        var tiny = TestMeshes.Box(1e-4f, 1e-4f, 1e-4f);
        Assert.Equal(36, MeshCleaner.Clean(tiny.V, tiny.I).Indices.Length);
        var huge = TestMeshes.Box(1e6f, 1e6f, 1e6f);
        Assert.Equal(36, MeshCleaner.Clean(huge.V, huge.I).Indices.Length);
    }
}
