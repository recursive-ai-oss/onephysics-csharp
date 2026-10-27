using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Collision3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Collision3D;

public class MeshBuildTests
{
    private static readonly Vector3F[] Tri = { new(0, 0, 0), new(1, 0, 0), new(0, 1, 0) };

    private static MeshBuildException Fails(Vector3F[] v, int[] i, MeshBuildError expect, MeshBuildOptions o = default)
    {
        MeshBuildException e = Assert.Throws<MeshBuildException>(() => TriangleMesh3D.Create(v, i, o));
        Assert.Equal(expect, e.Error);
        Assert.False(string.IsNullOrEmpty(e.Message));
        return e;
    }

    [Fact]
    public void HostileMeshInputGetsTypedErrorsNamingTheInput()
    {
        Fails(Array.Empty<Vector3F>(), MeshTestUtil.I(0, 1, 2), MeshBuildError.NoVertices);
        Fails(Tri, Array.Empty<int>(), MeshBuildError.NoTriangles);
        Assert.Contains("multiple of 3", Fails(Tri, MeshTestUtil.I(0, 1), MeshBuildError.InvalidIndexCount).Message);
        Assert.Contains("vertices[1].Y", Fails(MeshTestUtil.V(new(0, 0, 0), new(1, float.NaN, 0), new(0, 1, 0)), MeshTestUtil.I(0, 1, 2), MeshBuildError.NonFinite).Message);
        Assert.Contains("vertices[2].X", Fails(MeshTestUtil.V(new(0, 0, 0), new(1, 0, 0), new(float.NegativeInfinity, 1, 0)), MeshTestUtil.I(0, 1, 2), MeshBuildError.NonFinite).Message);
        Assert.Contains("vertices[0].Z", Fails(MeshTestUtil.V(new(0, 0, 3e6f), new(1, 0, 0), new(0, 1, 0)), MeshTestUtil.I(0, 1, 2), MeshBuildError.OutOfRange).Message);
        Assert.Contains("indices[4]", Fails(Tri, MeshTestUtil.I(0, 1, 2, 0, 3, 2), MeshBuildError.IndexOutOfRange).Message);
        Assert.Contains("triangle 1", Fails(Tri, MeshTestUtil.I(0, 1, 2, 0, 3, 2), MeshBuildError.IndexOutOfRange).Message);
        Fails(Tri, MeshTestUtil.I(0, 1, -1), MeshBuildError.IndexOutOfRange);
        Fails(Tri, MeshTestUtil.I(0, 1, int.MinValue), MeshBuildError.IndexOutOfRange);
        Fails(Tri, MeshTestUtil.I(0, 1, 2), MeshBuildError.InvalidOptions, new MeshBuildOptions { FlatEdgeTolerance = -1 });
        Fails(Tri, MeshTestUtil.I(0, 1, 2), MeshBuildError.InvalidOptions, new MeshBuildOptions { FlatEdgeTolerance = double.NaN });
        Fails(Tri, MeshTestUtil.I(0, 1, 2), MeshBuildError.InvalidOptions, new MeshBuildOptions { FlatEdgeTolerance = 2 });

        // The type derives from ArgumentException, so callers can catch it broadly.
        Assert.IsAssignableFrom<ArgumentException>(Assert.Throws<MeshBuildException>(() => TriangleMesh3D.Create(default, default)));
    }

    [Fact]
    public void ZeroAreaTrianglesAreDroppedAndReported()
    {
        var v = new Vector3F[] { new(0, 0, 0), new(1, 0, 0), new(0, 1, 0), new(2, 0, 0), new(5, 5, 5) };
        // Triangle 0 is fine; 1 repeats an index; 2 is collinear; 3 has two coincident points; 4 is fine; 5 repeats an index again.
        var i = MeshTestUtil.I(0, 1, 2, 0, 1, 1, 0, 1, 3, 0, 0, 4, 1, 3, 2, 0, 3, 3);
        TriangleMesh3D m = TriangleMesh3D.Create(v, i);
        Assert.Equal(2, m.TriangleCount);
        Assert.Equal(MeshTestUtil.I(1, 2, 3, 5), m.DroppedTriangles.ToArray());
        Assert.Equal(0, m.SourceTriangle(0));
        Assert.Equal(4, m.SourceTriangle(1));
        Assert.Throws<ArgumentOutOfRangeException>(() => m.SourceTriangle(2));
        Assert.True(m.TryGetTriangle(1, out SoupTriangle3D t));
        Assert.Equal(new Vector3F(2, 0, 0), t.V1);
        Assert.False(m.TryGetTriangle(2, out _));
        Assert.False(m.TryGetTriangle(-1, out _));

        // Nothing dropped: the identity mapping.
        Assert.Empty(MeshTestUtil.Cube().DroppedTriangles.ToArray());

        Assert.Contains("zero area", Fails(v, MeshTestUtil.I(0, 1, 1, 0, 0, 4), MeshBuildError.NoTriangles).Message);
    }

    [Fact]
    public void HugeAndTinyScalesBuild()
    {
        foreach (float s in new[] { 1e-6f, 1f, 1e6f })
        {
            var v = new Vector3F[] { new(0, 0, 0), new(s, 0, 0), new(0, s, 0), new(s, s, s) };
            TriangleMesh3D m = TriangleMesh3D.Create(v, MeshTestUtil.I(0, 1, 2, 1, 3, 2));
            Assert.Equal(2, m.TriangleCount);
            Assert.True(m.RayCast(new Vector3F(0.2f * s, 0.2f * s, -s), new Vector3F(0, 0, s), 4f, out TriangleRayHit3D h));
            Assert.Equal(1f, h.Fraction, 3);
        }

        // The largest allowed coordinates.
        var big = new Vector3F[] { new(-2097152f, 0, 0), new(2097152f, 0, 0), new(0, 2097152f, 2097152f) };
        Assert.Equal(1, TriangleMesh3D.Create(big, MeshTestUtil.I(0, 1, 2)).TriangleCount);
    }

    [Fact]
    public void HostileHeightFieldInputGetsTypedErrors()
    {
        var ok = new float[9];
        var one = new Vector3F(1, 1, 1);
        void Fail(Action a, MeshBuildError e, string contains)
        {
            MeshBuildException ex = Assert.Throws<MeshBuildException>(a);
            Assert.Equal(e, ex.Error);
            Assert.Contains(contains, ex.Message);
        }

        Fail(() => HeightField3D.Create(ok, 1, 9, one), MeshBuildError.InvalidGrid, "1 x 9");
        Fail(() => HeightField3D.Create(ok, 3, 0, one), MeshBuildError.InvalidGrid, "3 x 0");
        Fail(() => HeightField3D.Create(ok, 20000, 2, one), MeshBuildError.InvalidGrid, "20000");
        Fail(() => HeightField3D.Create(ok, 3, 4, one), MeshBuildError.InvalidGrid, "12 are expected");
        Fail(() => HeightField3D.Create(ok, 3, 3, new Vector3F(0, 1, 1)), MeshBuildError.OutOfRange, "scale.X");
        Fail(() => HeightField3D.Create(ok, 3, 3, new Vector3F(1, -1, 1)), MeshBuildError.OutOfRange, "scale.Y");
        Fail(() => HeightField3D.Create(ok, 3, 3, new Vector3F(1, 1, float.NaN)), MeshBuildError.NonFinite, "scale.Z");
        Fail(() => HeightField3D.Create(ok, 3, 3, new Vector3F(2e6f, 1, 1)), MeshBuildError.OutOfRange, "extent");
        var nan = new float[9];
        nan[7] = float.NaN;
        Fail(() => HeightField3D.Create(nan, 3, 3, one), MeshBuildError.NonFinite, "heights[7] (sample 1, 2)");
        var inf = new float[9];
        inf[0] = float.PositiveInfinity;
        Fail(() => HeightField3D.Create(inf, 3, 3, one), MeshBuildError.NonFinite, "heights[0]");
        var huge = new float[9];
        huge[4] = 1e7f;
        Fail(() => HeightField3D.Create(huge, 3, 3, one), MeshBuildError.OutOfRange, "heights[4]");
        Fail(() => HeightField3D.Create(ok, 3, 3, one, new bool[3]), MeshBuildError.InvalidGrid, "holes has 3");
        Fail(() => HeightField3D.Create(ok, 3, 3, one, new[] { true, true, true, true }), MeshBuildError.NoTriangles, "Every cell is a hole");
        Fail(() => HeightField3D.Create(ok, 3, 3, one, default, new MeshBuildOptions { FlatEdgeTolerance = 5 }), MeshBuildError.InvalidOptions, "FlatEdgeTolerance");
        Assert.Throws<ArgumentOutOfRangeException>(() => HeightField3D.Create(ok, 3, 3, one).IsHoleAt(2, 0));
    }

    [Fact]
    public void BvhBytesAreReproducibleAndCoverEveryTriangleOnce()
    {
        var rng = new TestRng(77);
        TriangleMesh3D a = MeshTestUtil.TerrainMesh(rng, 30, 1f, 2f);
        TriangleMesh3D b = MeshTestUtil.TerrainMesh(new TestRng(77), 30, 1f, 2f);
        Assert.Equal(a.ToBvhBytes(), b.ToBvhBytes());
        Assert.NotEqual(a.ToBvhBytes(), MeshTestUtil.Soup(new TestRng(78), 100, 3f).ToBvhBytes());

        // Every triangle sits in exactly one leaf, and every node's box contains its children and triangles.
        byte[] bytes = a.ToBvhBytes();
        int nodes = BitConverter.ToInt32(bytes, 0);
        Assert.Equal(a.BvhNodeCount, nodes);
        int orderStart = 4 + (nodes * 32) + 4;
        var seen = new int[a.TriangleCount];
        for (int n = 0; n < nodes; n++)
        {
            int count = BitConverter.ToInt32(bytes, 4 + (n * 32) + 28), offset = BitConverter.ToInt32(bytes, 4 + (n * 32) + 24);
            if (count > 0)
            {
                Assert.InRange(count, 1, 4);
                for (int k = offset; k < offset + count; k++)
                {
                    seen[BitConverter.ToInt32(bytes, orderStart + (4 * k))]++;
                }
            }
            else
            {
                Assert.True(offset > n + 1 && offset < nodes);
            }
        }

        Assert.All(seen, s => Assert.Equal(1, s));
    }
}

public class ActiveEdgeTests
{
    // Two triangles sharing the edge from (0,0,0) to (0,0,1), one on each side, bent by `lift` (y of the outer vertices).
    private static TriangleMesh3D Hinge(float leftY, float rightY)
    {
        var v = new Vector3F[] { new(0, 0, 0), new(0, 0, 1), new(-1, leftY, 0), new(1, rightY, 1) };
        // The left triangle (0, 2, 1) and the right triangle (0, 1, 3) are wound to face up (+y) when leftY = rightY = 0.
        return TriangleMesh3D.Create(v, MeshTestUtil.I(0, 2, 1, 0, 1, 3));
    }

    private static bool Active(TriangleMesh3D m, int id, int edge)
    {
        Assert.True(m.TryGetTriangle(id, out SoupTriangle3D t));
        return t.IsEdgeActive(edge);
    }

    [Fact]
    public void FlatAndConcaveSeamsAreInactiveConvexAndBoundaryActive()
    {
        // Flat.
        TriangleMesh3D flat = Hinge(0f, 0f);
        Assert.False(Active(flat, 0, 2)); // the shared edge 2 -> 0 of the first triangle
        Assert.False(Active(flat, 1, 0));
        Assert.True(Active(flat, 0, 0)); // boundary
        Assert.True(Active(flat, 0, 1));
        Assert.True(Active(flat, 1, 1));
        Assert.True(Active(flat, 1, 2));
        Assert.Equal((byte)0b011, MeshQueryHelper.Flags(flat, 0));

        // A valley (outer vertices higher): concave.
        TriangleMesh3D valley = Hinge(1f, 1f);
        Assert.False(Active(valley, 0, 2));
        Assert.False(Active(valley, 1, 0));

        // A ridge (outer vertices lower): convex.
        TriangleMesh3D ridge = Hinge(-1f, -1f);
        Assert.True(Active(ridge, 0, 2));
        Assert.True(Active(ridge, 1, 0));

        // Nearly flat bends within the tolerance count as flat; larger ones do not.
        Assert.False(Active(Hinge(-5e-4f, -5e-4f), 0, 2));
        Assert.True(Active(Hinge(-0.1f, -0.1f), 0, 2));
        Assert.False(Active(Hinge(0.1f, 0.1f), 0, 2));

        // The tolerance is an option.
        var v = new Vector3F[] { new(0, 0, 0), new(0, 0, 1), new(-1, -0.1f, 0), new(1, -0.1f, 1) };
        TriangleMesh3D loose = TriangleMesh3D.Create(v, MeshTestUtil.I(0, 2, 1, 0, 1, 3), new MeshBuildOptions { FlatEdgeTolerance = 0.5 });
        Assert.False(Active(loose, 0, 2));
    }

    [Fact]
    public void CubeEdgesAreActiveAndFaceDiagonalsAreNot()
    {
        TriangleMesh3D cube = MeshTestUtil.Cube();
        int inactive = 0;
        for (int id = 0; id < 12; id++)
        {
            cube.TryGetTriangle(id, out SoupTriangle3D t);
            for (int e = 0; e < 3; e++)
            {
                if (!t.IsEdgeActive(e))
                {
                    inactive++;
                    // The inactive edge is the diagonal of a face: its two end points differ on exactly two axes.
                    Vector3F a = e == 0 ? t.V0 : e == 1 ? t.V1 : t.V2, b = e == 0 ? t.V1 : e == 1 ? t.V2 : t.V0;
                    int differing = (a.X != b.X ? 1 : 0) + (a.Y != b.Y ? 1 : 0) + (a.Z != b.Z ? 1 : 0);
                    Assert.Equal(2, differing);
                }
            }
        }

        Assert.Equal(12, inactive); // six diagonals, two triangles each
    }

    [Fact]
    public void SplitVerticesAtTheSamePositionStillShareTheirSeam()
    {
        // The hinge again, with the shared edge's vertices duplicated for the second triangle.
        var v = new Vector3F[] { new(0, 0, 0), new(0, 0, 1), new(-1, 0, 0), new(0, 0, 0), new(0, 0, 1), new(1, 0, 1) };
        TriangleMesh3D m = TriangleMesh3D.Create(v, MeshTestUtil.I(0, 2, 1, 3, 4, 5));
        Assert.False(Active(m, 0, 2));
        Assert.False(Active(m, 1, 0));
    }

    [Fact]
    public void NonManifoldAndInconsistentlyWoundEdgesStayActive()
    {
        // Three triangles on one edge.
        var v = new Vector3F[] { new(0, 0, 0), new(0, 0, 1), new(-1, 0, 0), new(1, 0, 1), new(0, 1, 0.5f) };
        TriangleMesh3D m = TriangleMesh3D.Create(v, MeshTestUtil.I(0, 1, 2, 1, 0, 3, 1, 0, 4));
        for (int id = 0; id < 3; id++)
        {
            Assert.True(Active(m, id, 0));
        }

        // Neighbours wound the same way across the shared edge.
        TriangleMesh3D same = TriangleMesh3D.Create(MeshTestUtil.V(new(0, 0, 0), new(0, 0, 1), new(-1, 0, 0), new(1, 0, 1)), MeshTestUtil.I(0, 1, 2, 0, 1, 3));
        Assert.True(Active(same, 0, 0));
        Assert.True(Active(same, 1, 0));
    }

    [Fact]
    public void HeightFieldFlagsMatchAnIndependentAngleComputation()
    {
        const double Tol = 1e-3;
        var rng = new TestRng(5);
        HeightField3D hf = MeshTestUtil.RandomField(rng, 14, 11, integerHeights: false, holes: true);
        int checkedEdges = 0, inactiveEdges = 0, convex = 0;
        for (int id = 0; id < hf.TriangleIdCount; id++)
        {
            if (!hf.TryGetTriangle(id, out SoupTriangle3D t))
            {
                continue;
            }

            for (int e = 0; e < 3; e++)
            {
                Vector3F p = e == 0 ? t.V0 : e == 1 ? t.V1 : t.V2, q = e == 0 ? t.V1 : e == 1 ? t.V2 : t.V0;
                // Find the neighbour by brute force: the triangle having the edge q -> p.
                bool hasNeighbour = false, expectActive = true;
                for (int other = 0; other < hf.TriangleIdCount && !hasNeighbour; other++)
                {
                    if (other == id || !hf.TryGetTriangle(other, out SoupTriangle3D o))
                    {
                        continue;
                    }

                    Vector3F[] ov = { o.V0, o.V1, o.V2 };
                    for (int k = 0; k < 3; k++)
                    {
                        if (ov[k] == q && ov[(k + 1) % 3] == p)
                        {
                            hasNeighbour = true;
                            Vector3F opposite = ov[(k + 2) % 3];
                            Vector3D nA = Cross(t.V0, t.V1, t.V2), nB = Cross(o.V0, o.V1, o.V2);
                            double theta = Math.Atan2(Len(CrossV(nA, nB)), Dot(nA, nB));
                            double side = Dot(nA, opposite.ToDouble() - t.V0.ToDouble()); // negative when the neighbour folds away (convex)
                            bool concave = side > 0;
                            expectActive = !(theta <= Math.Asin(Tol) || (concave && Math.Sin(theta) > Tol));
                        }
                    }
                }

                if (!hasNeighbour)
                {
                    expectActive = true;
                }

                Assert.True(expectActive == t.IsEdgeActive(e), $"triangle {id} edge {e}: expected active={expectActive}");
                checkedEdges++;
                if (!expectActive)
                {
                    inactiveEdges++;
                }
                else if (hasNeighbour)
                {
                    convex++;
                }
            }
        }

        Assert.True(inactiveEdges > checkedEdges / 8 && convex > checkedEdges / 8, $"{inactiveEdges} inactive and {convex} convex interior of {checkedEdges}: the field is not exercising both");

        // A plane is entirely inactive inside; only the outer boundary is active.
        var flat = new float[8 * 8];
        HeightField3D plane = HeightField3D.Create(flat, 8, 8, new Vector3F(1, 1, 1));
        int active = 0;
        for (int id = 0; id < plane.TriangleIdCount; id++)
        {
            plane.TryGetTriangle(id, out SoupTriangle3D t);
            active += System.Numerics.BitOperations.PopCount(t.ActiveEdges);
        }

        Assert.Equal(4 * 7, active);
    }

    private static Vector3D Cross(Vector3F a, Vector3F b, Vector3F c) => Vector3D.Cross(b.ToDouble() - a.ToDouble(), c.ToDouble() - a.ToDouble());

    private static Vector3D CrossV(Vector3D a, Vector3D b) => Vector3D.Cross(a, b);

    private static double Dot(Vector3D a, Vector3D b) => Vector3D.Dot(a, b);

    private static double Len(Vector3D a) => Math.Sqrt(a.LengthSquared());
}

internal static class MeshQueryHelper
{
    public static byte Flags(TriangleSoup3D s, int id)
    {
        s.TryGetTriangle(id, out SoupTriangle3D t);
        return t.ActiveEdges;
    }
}
