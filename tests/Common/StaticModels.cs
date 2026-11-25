using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Tools.Tests;

/// <summary>Structured tetrahedral meshes with exact flat faces (six tetrahedra per cell, all cut along the same diagonal so neighbours conform).</summary>
public static class StaticMeshes
{
    /// <summary>A box [x0, x1] × [y0, y1] × [z0, z1] of nx × ny × nz cells; vertex (i, j, k) is at index (k * (ny + 1) + j) * (nx + 1) + i.</summary>
    public static (Vector3F[] Vertices, int[] Tetrahedra) Grid(float x0, float x1, float y0, float y1, float z0, float z1, int nx, int ny, int nz)
    {
        var v = new Vector3F[(nx + 1) * (ny + 1) * (nz + 1)];
        int Idx(int i, int j, int k) => (k * (ny + 1) + j) * (nx + 1) + i;
        for (int k = 0; k <= nz; k++) { for (int j = 0; j <= ny; j++) { for (int i = 0; i <= nx; i++) { v[Idx(i, j, k)] = new Vector3F(x0 + (x1 - x0) * i / nx, y0 + (y1 - y0) * j / ny, z0 + (z1 - z0) * k / nz); } } }
        var t = new List<int>();
        int[][] perms = { new[] { 0, 1, 2 }, new[] { 0, 2, 1 }, new[] { 1, 0, 2 }, new[] { 1, 2, 0 }, new[] { 2, 0, 1 }, new[] { 2, 1, 0 } };
        for (int k = 0; k < nz; k++)
        {
            for (int j = 0; j < ny; j++)
            {
                for (int i = 0; i < nx; i++)
                {
                    foreach (var pm in perms)
                    {
                        int[] pos = { i, j, k };
                        int a = Idx(i, j, k);
                        pos[pm[0]]++; int b = Idx(pos[0], pos[1], pos[2]);
                        pos[pm[1]]++; int c = Idx(pos[0], pos[1], pos[2]);
                        pos[pm[2]]++; int d = Idx(pos[0], pos[1], pos[2]);
                        // Odd permutations of the axes run left-handed: swap two vertices.
                        bool odd = (pm[0] == 0 && pm[1] == 2) || (pm[0] == 1 && pm[1] == 0) || (pm[0] == 2 && pm[1] == 1);
                        t.Add(a); t.Add(odd ? c : b); t.Add(odd ? b : c); t.Add(d);
                    }
                }
            }
        }

        return (v, t.ToArray());
    }
}

/// <summary>Meshes and problems for the static solve tests and the hash-matrix scenes: a cantilever beam along +x with its root on the plane x = 0, from a structured grid or cut by the tetrahedralizer.</summary>
public sealed class CantileverModel
{
    public Vector3F[] Vertices { get; init; } = Array.Empty<Vector3F>();
    public int[] Tetrahedra { get; init; } = Array.Empty<int>();
    public TetrahedralMesh? Mesh { get; init; }
    public double Length { get; init; }
    public double Width { get; init; }
    public double Height { get; init; }

    /// <summary>Vertices on the root plane x = 0.</summary>
    public List<int> Root { get; } = new();

    /// <summary>Vertices on the tip plane x = Length.</summary>
    public List<int> Tip { get; } = new();

    /// <summary>Boundary triangles on the tip plane.</summary>
    public List<(int A, int B, int C)> TipFaces { get; } = new();

    /// <summary>A beam of nx × ny × nz grid cells (exact flat faces): width along y, height along z, root clamped, the tip faces loadable.</summary>
    public static CantileverModel BuildGrid(float length, float width, float height, int nx, int ny, int nz)
    {
        var (v, t) = StaticMeshes.Grid(0, length, -width / 2, width / 2, -height / 2, height / 2, nx, ny, nz);
        var m = new CantileverModel { Vertices = v, Tetrahedra = t, Length = length, Width = width, Height = height };
        m.Finish(1e-4f * length, true);
        return m;
    }

    /// <summary>The same beam cut from a box by the tetrahedralizer. Its boundary is not flat (it follows the lattice within a fraction of a cell), so the root is the vertices within 0.6 cell of x = 0 and the tip load is spread over the vertices within 0.6 cell of x = Length.</summary>
    public static CantileverModel BuildLattice(float length, float width, float height, float cell)
    {
        var box = TestMeshes.Box(length, width, height, length / 2, 0, 0);
        var mesh = Tetrahedralizer.Generate(box.V, box.I, new TetrahedralizeOptions { TargetEdgeLength = cell, SkipDeviation = true });
        var m = new CantileverModel { Vertices = mesh.Vertices, Tetrahedra = mesh.Tetrahedra, Mesh = mesh, Length = length, Width = width, Height = height };
        m.Finish(0.6f * cell, false);
        return m;
    }

    private void Finish(float eps, bool flat)
    {
        for (int i = 0; i < Vertices.Length; i++)
        {
            if (Vertices[i].X < eps) { Root.Add(i); }
            if (Vertices[i].X > Length - eps) { Tip.Add(i); }
        }

        if (!flat) { return; }
        var tipSet = new HashSet<int>(Tip);
        var t = Tetrahedra;
        int[][] faces = { new[] { 0, 1, 2 }, new[] { 0, 1, 3 }, new[] { 0, 2, 3 }, new[] { 1, 2, 3 } };
        for (int k = 0; k < t.Length / 4; k++)
        {
            foreach (var f in faces)
            {
                int a = t[4 * k + f[0]], b = t[4 * k + f[1]], c = t[4 * k + f[2]];
                if (tipSet.Contains(a) && tipSet.Contains(b) && tipSet.Contains(c)) { TipFaces.Add((a, b, c)); }
            }
        }
    }

    /// <summary>Root clamped; a downward (−z) shear traction totalling <paramref name="force"/> N over the tip faces.</summary>
    public StaticProblem EndLoad(ElasticMaterial material, double force)
    {
        var p = new StaticProblem { Material = material };
        foreach (int v in Root) { p.Supports.Add(new StaticSupport(v)); }
        double traction = -force / (Width * Height);
        foreach (var (a, b, c) in TipFaces) { p.FaceLoads.Add(new FaceLoad(a, b, c, 0, new Vector3D(0, 0, traction))); }
        if (TipFaces.Count == 0) { foreach (int v in Tip) { p.PointLoads.Add(new PointLoad(v, new Vector3D(0, 0, -force / Tip.Count))); } }
        return p;
    }

    /// <summary>Root clamped, gravity along −z.</summary>
    public StaticProblem SelfWeight(ElasticMaterial material, double g = 9.81)
    {
        var p = new StaticProblem { Material = material, Gravity = new Vector3D(0, 0, -g) };
        foreach (int v in Root) { p.Supports.Add(new StaticSupport(v)); }
        return p;
    }
}
