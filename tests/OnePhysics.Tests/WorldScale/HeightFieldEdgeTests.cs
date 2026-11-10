#pragma warning disable OPHYS0001, OPHYS0002, OPHYS0003
using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.WorldScale;

public sealed class HeightFieldEdgeTests
{
    private static float[] Random(Random r, int n, float amplitude, bool flatSome)
    {
        var h = new float[n];
        for (int i = 0; i < n; i++) { h[i] = flatSome && r.Next(3) != 0 ? 0f : (float)((r.NextDouble() - 0.5) * amplitude); }
        return h;
    }

    [Fact]
    public void SharedNormalsGiveTheFlagsOfTheEdgeByEdgeTest()
    {
        var r = new Random(1350);
        for (int round = 0; round < 40; round++)
        {
            int nx = 2 + r.Next(14), nz = 2 + r.Next(14);
            var h = Random(r, nx * nz, round % 3 == 0 ? 0.001f : 3f, round % 2 == 0);
            bool[]? holes = null;
            if (round % 4 == 1 && nx > 2 && nz > 2) { holes = new bool[(nx - 1) * (nz - 1)]; for (int i = 0; i < holes.Length; i += 1 + r.Next(5)) { holes[i] = true; } holes[0] = false; }
            HeightFieldEdges3D? edges = null;
            if (round % 5 == 2) { edges = new HeightFieldEdges3D { West = Random(r, nz, 3f, false), East = Random(r, nz, 3f, true), South = Random(r, nx, 3f, false), North = Random(r, nx, 3f, true) }; }
            var hf = HeightField3D.Create(h, nx, nz, new Vector3F(0.5f + (float)r.NextDouble(), 1f, 0.5f + (float)r.NextDouble()), holes, default, edges);
            Assert.Equal(hf.ReferenceFlags(1e-3), hf.FlagsForTests);
        }
    }

    [Fact]
    public void BorderEdgesBetweenFlatNeighboursAreInactiveOnlyWithTheRing()
    {
        var flat = new float[9];
        var without = HeightField3D.Create(flat, 3, 3, new Vector3F(1, 1, 1));
        var ring = HeightField3D.Create(flat, 3, 3, new Vector3F(1, 1, 1), default, default, new HeightFieldEdges3D { West = new float[3], East = new float[3], South = new float[3], North = new float[3] });
        int Active(HeightField3D f) { int n = 0; foreach (byte b in f.FlagsForTests) { for (int k = 0; k < 6; k++) { n += (b >> k) & 1; } } return n; }
        Assert.True(Active(without) > Active(ring));
        // the field's own diagonals are inactive either way; with the ring no edge of a flat field is active
        Assert.Equal(0, Active(ring));
    }

    [Fact]
    public void RingValidationNamesTheSideAndTheProblem()
    {
        var flat = new float[9];
        var e = Assert.Throws<MeshBuildException>(() => HeightField3D.Create(flat, 3, 3, new Vector3F(1, 1, 1), default, default, new HeightFieldEdges3D { West = new float[2] }));
        Assert.Contains("West", e.Message, StringComparison.Ordinal);
        e = Assert.Throws<MeshBuildException>(() => HeightField3D.Create(flat, 3, 3, new Vector3F(1, 1, 1), default, default, new HeightFieldEdges3D { North = [0f, float.NaN, 0f] }));
        Assert.Contains("North[1]", e.Message, StringComparison.Ordinal);
    }
}
