using System;
using System.Collections.Generic;
using System.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Collision3D;

/// <summary>Generators and brute-force references for the triangle mesh and height field tests.</summary>
internal static class MeshTestUtil
{
    public static readonly Vector3F[] CubeVertices =
    {
        new(0, 0, 0), new(1, 0, 0), new(1, 1, 0), new(0, 1, 0), new(0, 0, 1), new(1, 0, 1), new(1, 1, 1), new(0, 1, 1),
    };

    // Outward winding, two triangles per face; each face's diagonal is its first triangle's last edge.
    public static readonly int[] CubeIndices =
    {
        0, 3, 2, 0, 2, 1, // -z
        4, 5, 6, 4, 6, 7, // +z
        0, 1, 5, 0, 5, 4, // -y
        3, 7, 6, 3, 6, 2, // +y
        0, 4, 7, 0, 7, 3, // -x
        1, 2, 6, 1, 6, 5, // +x
    };

    public static int[] I(params int[] a) => a;

    public static Vector3F[] V(params Vector3F[] a) => a;

    public static TriangleMesh3D Cube() => TriangleMesh3D.Create(CubeVertices, CubeIndices);

    /// <summary>A jittered terrain as an indexed mesh with shared vertices.</summary>
    public static TriangleMesh3D TerrainMesh(TestRng rng, int n, float spacing, float height)
    {
        var vs = new Vector3F[n * n];
        for (int j = 0; j < n; j++)
        {
            for (int i = 0; i < n; i++)
            {
                vs[(j * n) + i] = new Vector3F((i * spacing) + rng.Range(-0.3f, 0.3f) * spacing, rng.Range(0f, height), (j * spacing) + rng.Range(-0.3f, 0.3f) * spacing);
            }
        }

        var idx = new List<int>();
        for (int j = 0; j + 1 < n; j++)
        {
            for (int i = 0; i + 1 < n; i++)
            {
                int a = (j * n) + i, b = a + 1, c = a + n, d = c + 1;
                bool flip = rng.Next(2) == 0;
                if (flip)
                {
                    idx.AddRange(new[] { a, c, d, a, d, b });
                }
                else
                {
                    idx.AddRange(new[] { a, c, b, b, c, d });
                }
            }
        }

        return TriangleMesh3D.Create(vs, idx.ToArray());
    }

    /// <summary>Random triangles scattered in a cube, unwelded, some thin, some overlapping.</summary>
    public static TriangleMesh3D Soup(TestRng rng, int count, float extent)
    {
        var vs = new Vector3F[3 * count];
        var idx = new int[3 * count];
        for (int t = 0; t < count; t++)
        {
            Vector3F c = rng.InCube(extent);
            float size = rng.Range(0.05f, 0.5f) * extent;
            for (int k = 0; k < 3; k++)
            {
                vs[(3 * t) + k] = c + (rng.InBall(size));
                idx[(3 * t) + k] = (3 * t) + k;
            }
        }

        return TriangleMesh3D.Create(vs, idx);
    }

    public static HeightField3D RandomField(TestRng rng, int nx, int nz, bool integerHeights, bool holes)
    {
        var h = new float[nx * nz];
        for (int i = 0; i < h.Length; i++)
        {
            h[i] = integerHeights ? rng.Next(4) : rng.Range(0f, 3f);
        }

        bool[] mask = Array.Empty<bool>();
        if (holes)
        {
            mask = new bool[(nx - 1) * (nz - 1)];
            for (int i = 0; i < mask.Length; i++)
            {
                mask[i] = rng.Next(6) == 0;
            }
        }

        return HeightField3D.Create(h, nx, nz, new Vector3F(1f, 1f, 1f), mask);
    }

    public static bool BruteRay(TriangleSoup3D s, Vector3F o, Vector3F d, float maxFraction, bool cull, out int id, out double t)
    {
        id = -1;
        t = 0;
        if (d == default)
        {
            return false;
        }

        bool found = false;
        for (int k = 0; k < s.TriangleIdCount; k++)
        {
            if (!s.TryGetTriangle(k, out SoupTriangle3D tri))
            {
                continue;
            }

            if (TriangleTests3D.RayTriangle(o, d, tri.V0, tri.V1, tri.V2, cull, out double tt, out _) && tt <= maxFraction && (!found || tt < t))
            {
                found = true;
                id = k;
                t = tt;
            }
        }

        return found;
    }

    public static List<int> BruteBox(TriangleSoup3D s, Bounds3F box)
    {
        var ids = new List<int>();
        for (int k = 0; k < s.TriangleIdCount; k++)
        {
            if (s.TryGetTriangle(k, out SoupTriangle3D tri) && TriangleTests3D.BoxTriangle(box, tri.V0, tri.V1, tri.V2))
            {
                ids.Add(k);
            }
        }

        return ids;
    }

    private static readonly Vector3F[] s_gridDirs =
    {
        new(0, -1, 0), new(0, 1, 0), new(1, 0, 0), new(0, 0, 1), new(1, -1, 0), new(1, 0, 1), new(1, -1, 1), new(-1, -1, 1), new(0, -1, 1), new(1, -2, 1), new(-1, -1, -1),
    };

    /// <summary>A mix of random and grid-aligned rays (quarter-integer origins, small integer directions) so rays pass exactly through edges and vertices.</summary>
    public static void RandomRay(TestRng rng, Bounds3F within, out Vector3F o, out Vector3F d, out float max)
    {
        Vector3F size = within.Size;
        if (rng.Next(3) == 0)
        {
            o = new Vector3F(MathF.Floor(within.Min.X) + (rng.Next((int)(size.X * 4) + 1) * 0.25f), MathF.Floor(within.Max.Y) + 1f - (rng.Next(16) * 0.25f), MathF.Floor(within.Min.Z) + (rng.Next((int)(size.Z * 4) + 1) * 0.25f));
            d = s_gridDirs[rng.Next(s_gridDirs.Length)];
            max = rng.Next(3) == 0 ? 1f : rng.Range(0f, 12f);
        }
        else
        {
            o = new Vector3F(within.Min.X + (rng.Range(-0.1f, 1.1f) * size.X), within.Min.Y + (rng.Range(-0.5f, 2f) * size.Y) + rng.Range(0f, 2f), within.Min.Z + (rng.Range(-0.1f, 1.1f) * size.Z));
            d = rng.OnSphere() * rng.Range(0.2f, 3f);
            max = rng.Range(0f, 12f);
        }
    }

    public static Bounds3F RandomBox(TestRng rng, Bounds3F within)
    {
        Vector3F size = within.Size;
        float extent = Math.Max(size.X, Math.Max(size.Y, size.Z));
        Vector3F c = new(within.Min.X + (rng.Range(-0.05f, 1.05f) * size.X), within.Min.Y + (rng.Range(-0.2f, 1.2f) * size.Y), within.Min.Z + (rng.Range(-0.05f, 1.05f) * size.Z));
        float k = rng.Next(4) == 0 ? 0f : rng.Range(0f, extent * 0.3f);
        Vector3F h = new(rng.Range(0f, 1f) * k, rng.Range(0f, 1f) * k, rng.Range(0f, 1f) * k);
        if (rng.Next(5) == 0)
        {
            c = new Vector3F(MathF.Round(c.X * 4) / 4, MathF.Round(c.Y * 4) / 4, MathF.Round(c.Z * 4) / 4);
            h = new Vector3F(MathF.Round(h.X * 4) / 4, MathF.Round(h.Y * 4) / 4, MathF.Round(h.Z * 4) / 4);
        }

        return new Bounds3F(c - h, c + h);
    }

    public static ConvexShape3D RandomShape(TestRng rng)
    {
        switch (rng.Next(4))
        {
            case 0: return ConvexShape3D.Sphere(rng.Range(0.05f, 0.5f));
            case 1: return ConvexShape3D.Box(new Vector3F(rng.Range(0.05f, 0.4f), rng.Range(0.05f, 0.4f), rng.Range(0.05f, 0.4f)), 0.02f);
            case 2: return ConvexShape3D.Capsule(rng.Range(0.05f, 0.5f), rng.Range(0.05f, 0.3f));
            default: return ConvexShape3D.Cylinder(rng.Range(0.1f, 0.5f), rng.Range(0.1f, 0.4f), 0.02f);
        }
    }
}

public class MeshQueryTests
{
    [Fact]
    public void CubeRaysHitAtAnalyticPoints()
    {
        TriangleMesh3D cube = MeshTestUtil.Cube();
        Assert.Equal(12, cube.TriangleCount);
        Assert.True(cube.RayCast(new Vector3F(0.3f, 0.4f, -1f), new Vector3F(0, 0, 1), 10f, out TriangleRayHit3D hit));
        Assert.Equal(1f, hit.Fraction);
        Assert.Equal(new Vector3F(0.3f, 0.4f, 0f), hit.Point);
        Assert.Equal(new Vector3F(0, 0, -1), hit.Normal);
        Assert.False(hit.BackFace);

        // From inside, the first hit is the back of the +x face at distance 0.5 (a ray length of 2 along +x).
        Assert.True(cube.RayCast(new Vector3F(0.5f, 0.5f, 0.5f), new Vector3F(2, 0, 0), 1f, out hit));
        Assert.Equal(0.25f, hit.Fraction);
        Assert.True(hit.BackFace);
        Assert.False(cube.RayCast(new Vector3F(0.5f, 0.5f, 0.5f), new Vector3F(2, 0, 0), 1f, out _, cullBackFaces: true));

        // Misses: beside the cube, too short, pointing away.
        Assert.False(cube.RayCast(new Vector3F(1.5f, 0.5f, -1f), new Vector3F(0, 0, 1), 10f, out _));
        Assert.False(cube.RayCast(new Vector3F(0.5f, 0.5f, -1f), new Vector3F(0, 0, 1), 0.9f, out _));
        Assert.False(cube.RayCast(new Vector3F(0.5f, 0.5f, -1f), new Vector3F(0, 0, -1), 10f, out _));
        Assert.False(cube.RayCast(new Vector3F(0.5f, 0.5f, -1f), default, 10f, out _));
    }

    [Theory]
    [InlineData(0.5f, 0.5f, -1f, 0f, 0f, 1f)] // through a face diagonal
    [InlineData(-1f, -1f, -1f, 1f, 1f, 1f)] // through two opposite vertices
    [InlineData(0.5f, -1f, -1f, 0f, 1f, 1f)] // through two cube edges
    [InlineData(0.5f, 0.5f, 2f, 0f, 0f, -1f)] // through the diagonals from the other side
    [InlineData(0.25f, 0.75f, -2f, 0f, 0f, 1f)] // across the other face diagonal
    public void RayThroughSharedEdgeOrVertexHitsEachSurfaceOnce(float ox, float oy, float oz, float dx, float dy, float dz)
    {
        TriangleMesh3D cube = MeshTestUtil.Cube();
        var hits = new List<TriangleRayHit3D>();
        int n = cube.RayCastAll(new Vector3F(ox, oy, oz), new Vector3F(dx, dy, dz), 100f, hits);
        // The ray enters and leaves the cube once each; a ray grazing along the surface is excluded from these cases.
        Assert.Equal(2, n);
        Assert.True(hits[0].Fraction < hits[1].Fraction);
        Assert.False(hits[0].BackFace);
        Assert.True(hits[1].BackFace);
    }

    [Fact]
    public void UnitGridHeightFieldHitsAnalyticPointsAndRespectsHoles()
    {
        var flat = new float[25];
        HeightField3D hf = HeightField3D.Create(flat, 5, 5, new Vector3F(1, 1, 1));
        Assert.Equal(32, hf.TriangleCount);
        Assert.True(hf.RayCast(new Vector3F(2.25f, 5f, 1.5f), new Vector3F(0, -1, 0), 10f, out TriangleRayHit3D hit));
        Assert.Equal(5f, hit.Fraction);
        Assert.Equal(new Vector3F(0, 1, 0), hit.Normal);
        Assert.Equal(2 * ((1 * 4) + 2), hit.TriangleId & ~1); // cell (2, 1)

        // Straight down through a sample point shared by four cells, and onto a cell diagonal: once each.
        foreach (var p in new[] { new Vector3F(2, 4, 2), new Vector3F(1.5f, 4, 2.5f), new Vector3F(3, 4, 1), new Vector3F(2.5f, 4, 1.5f), new Vector3F(1, 4, 3) })
        {
            var all = new List<TriangleRayHit3D>();
            Assert.Equal(1, hf.RayCastAll(p, new Vector3F(0, -1, 0), 10f, all));
            Assert.Equal(4f, all[0].Fraction);
        }

        // A slope y = x: the ray from (2.25, 10, 1.5) down meets it at y = 2.25.
        var slope = new float[25];
        for (int j = 0; j < 5; j++)
        {
            for (int i = 0; i < 5; i++)
            {
                slope[(j * 5) + i] = i;
            }
        }

        HeightField3D hs = HeightField3D.Create(slope, 5, 5, new Vector3F(1, 1, 1));
        Assert.True(hs.RayCast(new Vector3F(2.25f, 10f, 1.5f), new Vector3F(0, -1, 0), 20f, out hit));
        Assert.Equal(7.75f, hit.Fraction);
        Assert.Equal(1f / MathF.Sqrt(2f), hit.Normal.Y, 6);

        // Holes: the cell (2, 1) has none, its neighbours do.
        var holes = new bool[16];
        holes[(1 * 4) + 2] = true;
        HeightField3D withHole = HeightField3D.Create(flat, 5, 5, new Vector3F(1, 1, 1), holes);
        Assert.Equal(30, withHole.TriangleCount);
        Assert.False(withHole.RayCast(new Vector3F(2.25f, 5f, 1.5f), new Vector3F(0, -1, 0), 10f, out _));
        Assert.True(withHole.RayCast(new Vector3F(3.25f, 5f, 1.5f), new Vector3F(0, -1, 0), 10f, out _));
        Assert.True(withHole.IsHoleAt(2, 1));
        Assert.False(withHole.TryGetTriangle(2 * ((1 * 4) + 2), out _));
        var none = new List<int>();
        withHole.Overlap(new Bounds3F(new Vector3F(2.1f, -1, 1.1f), new Vector3F(2.9f, 1, 1.9f)), none);
        Assert.Empty(none);
    }

    private static void CheckRays(TriangleSoup3D s, float extent, int count, ulong seed)
    {
        var rng = new TestRng(seed);
        int hits = 0;
        for (int q = 0; q < count; q++)
        {
            MeshTestUtil.RandomRay(rng, s.Bounds, out Vector3F o, out Vector3F d, out float max);
            bool cull = (q & 7) == 0;
            bool want = MeshTestUtil.BruteRay(s, o, d, max, cull, out int wantId, out double wantT);
            bool got = s.RayCast(o, d, max, out TriangleRayHit3D hit, cull);
            Assert.True(want == got, $"query {q}: ray {o} + t {d} (max {max}) brute {want} bvh {got}");
            if (want)
            {
                hits++;
                Assert.True(wantId == hit.TriangleId && BitConverter.SingleToInt32Bits((float)wantT) == BitConverter.SingleToInt32Bits(hit.Fraction), $"query {q}: brute {wantId} at {wantT}, bvh {hit.TriangleId} at {hit.Fraction}");
                Assert.True(s.RayCastAny(o, d, max, out TriangleRayHit3D any, cull));
                Assert.True(any.Fraction >= hit.Fraction);
            }
            else
            {
                Assert.False(s.RayCastAny(o, d, max, out _, cull));
            }

            if (q % 16 == 0)
            {
                var all = new List<TriangleRayHit3D>();
                int n = s.RayCastAll(o, d, max, all, cull);
                int wantN = 0;
                for (int k = 0; k < s.TriangleIdCount; k++)
                {
                    if (s.TryGetTriangle(k, out SoupTriangle3D tri) && TriangleTests3D.RayTriangle(o, d, tri.V0, tri.V1, tri.V2, cull, out double tt, out _) && tt <= max)
                    {
                        wantN++;
                    }
                }

                Assert.Equal(wantN, n);
                for (int k = 1; k < n; k++)
                {
                    Assert.True(all[k - 1].Fraction <= all[k].Fraction);
                }
            }
        }

        Assert.True(hits > count / 20, $"only {hits} of {count} rays hit; the test is not exercising the hit path");
    }

    [Fact]
    public void RaysAgreeWithBruteForceOnMeshes()
    {
        var rng = new TestRng(11);
        CheckRays(MeshTestUtil.TerrainMesh(rng, 20, 1f, 2f), 20f, 50_000, 101);
        CheckRays(MeshTestUtil.Soup(rng, 300, 5f), 5f, 50_000, 102);
        CheckRays(MeshTestUtil.Cube(), 1f, 20_000, 103);
    }

    [Fact]
    public void CoplanarOverlappingTrianglesTieToTheLowestId()
    {
        // Duplicated and rotated copies of triangles on a lattice, all in the plane y = 0, so many rays tie exactly or within rounding.
        var rng = new TestRng(31);
        var vs = new List<Vector3F>();
        var idx = new List<int>();
        for (int t = 0; t < 40; t++)
        {
            Vector3F a = new(rng.Next(5), 0, rng.Next(5)), b = new(rng.Next(5), 0, rng.Next(5)), c = new(rng.Next(5), 0, rng.Next(5));
            int r = rng.Next(3);
            Vector3F[] tri = { a, b, c };
            for (int k = 0; k < 3; k++)
            {
                vs.Add(tri[(k + r) % 3]);
                idx.Add(vs.Count - 1);
            }
        }

        TriangleMesh3D mesh = TriangleMesh3D.Create(vs.ToArray(), idx.ToArray());
        CheckRays(mesh, 5f, 40_000, 501);
    }

    [Fact]
    public void RaysAgreeWithBruteForceOnHeightFields()
    {
        var rng = new TestRng(12);
        CheckRays(MeshTestUtil.RandomField(rng, 40, 33, integerHeights: true, holes: false), 36f, 50_000, 201);
        CheckRays(MeshTestUtil.RandomField(rng, 37, 41, integerHeights: false, holes: true), 36f, 50_000, 202);
        CheckRays(MeshTestUtil.RandomField(rng, 5, 3, integerHeights: true, holes: true), 4f, 20_000, 203);

        // Long and thin: nine levels of the pyramid, the top ones with a single row of blocks.
        CheckRays(MeshTestUtil.RandomField(rng, 1030, 6, integerHeights: false, holes: true), 1000f, 6_000, 204);
    }

    private static void CheckOverlaps(TriangleSoup3D s, float extent, int count, ulong seed)
    {
        var rng = new TestRng(seed);
        var ids = new List<int>();
        int nonEmpty = 0;
        for (int q = 0; q < count; q++)
        {
            Bounds3F box = MeshTestUtil.RandomBox(rng, s.Bounds);
            ids.Clear();
            int n = s.Overlap(box, ids);
            List<int> want = MeshTestUtil.BruteBox(s, box);
            Assert.Equal(want.Count, n);
            Assert.True(want.SequenceEqual(ids), $"query {q}: box {box}");
            if (n > 0)
            {
                nonEmpty++;
            }
        }

        Assert.True(nonEmpty > count / 10, $"only {nonEmpty} of {count} boxes found triangles");
    }

    [Fact]
    public void OverlapAgreesWithBruteForce()
    {
        var rng = new TestRng(13);
        CheckOverlaps(MeshTestUtil.TerrainMesh(rng, 20, 1f, 2f), 20f, 50_000, 301);
        CheckOverlaps(MeshTestUtil.Soup(rng, 300, 5f), 5f, 50_000, 302);
        CheckOverlaps(MeshTestUtil.RandomField(rng, 40, 33, integerHeights: true, holes: false), 36f, 50_000, 303);
        CheckOverlaps(MeshTestUtil.RandomField(rng, 37, 41, integerHeights: false, holes: true), 36f, 50_000, 304);
        CheckOverlaps(MeshTestUtil.RandomField(rng, 1030, 6, integerHeights: false, holes: true), 1000f, 5_000, 305);
    }

    private static void CheckCasts(TriangleSoup3D s, float extent, int count, ulong seed)
    {
        var rng = new TestRng(seed);
        var all = new List<int>();
        for (int k = 0; k < s.TriangleIdCount; k++)
        {
            all.Add(k);
        }

        var scratch = new List<int>();
        int hits = 0;
        for (int q = 0; q < count; q++)
        {
            ConvexShape3D shape = MeshTestUtil.RandomShape(rng);
            var place = new RigidTransformF(rng.Rotation(), rng.InCube(extent * 0.6f) + new Vector3F(extent * 0.5f, extent * 0.4f, extent * 0.5f));
            Vector3F motion = rng.OnSphere() * rng.Range(0.1f, extent * 0.5f);
            bool want = s.CastTriangles(shape, place, motion, all, out SoupCastHit3D w);
            bool got = s.ShapeCast(shape, place, motion, out SoupCastHit3D g, scratch);
            if (want != got)
            {
                // A touch at the very end of the motion may fall either side of the filter's margin.
                Assert.True(want && w.Fraction > 0.999f, $"query {q}: brute {want} (fraction {w.Fraction}) filtered {got}");
                continue;
            }

            if (want)
            {
                hits++;
                Assert.True(Math.Abs(w.Fraction - g.Fraction) <= 2e-4f, $"query {q}: brute fraction {w.Fraction} (triangle {w.TriangleId}), filtered {g.Fraction} (triangle {g.TriangleId})");
            }
        }

        Assert.True(hits > count / 10, $"only {hits} of {count} casts hit");
    }

    [Fact]
    public void ShapeCastAgreesWithBruteForce()
    {
        var rng = new TestRng(14);
        CheckCasts(MeshTestUtil.TerrainMesh(rng, 6, 1f, 1f), 5f, 30_000, 401);
        CheckCasts(MeshTestUtil.Soup(rng, 40, 3f), 3f, 30_000, 402);
        CheckCasts(MeshTestUtil.RandomField(rng, 7, 7, integerHeights: false, holes: true), 6f, 40_000, 403);
    }

    [Fact]
    public void SphereCastLandsOnAPlaneAtTheAnalyticTime()
    {
        var flat = new float[11 * 11];
        HeightField3D hf = HeightField3D.Create(flat, 11, 11, new Vector3F(1, 1, 1));
        ConvexShape3D ball = ConvexShape3D.Sphere(0.5f);
        var at = new RigidTransformF(QuaternionF.Identity, new Vector3F(5.3f, 3f, 4.7f));
        Assert.True(hf.ShapeCast(ball, at, new Vector3F(0, -5, 0), out SoupCastHit3D hit));
        Assert.Equal((3f - 0.5f) / 5f, hit.Fraction, 4);
        Assert.Equal(new Vector3F(0, 1, 0).Y, hit.Normal.Y, 4);
        Assert.Equal(0f, hit.Point.Y, 4);
        Assert.False(hit.StartsOverlapping);
        Assert.False(hf.ShapeCast(ball, at, new Vector3F(0, 1, 0), out _));

        // A ball already resting on it starts overlapping.
        var resting = new RigidTransformF(QuaternionF.Identity, new Vector3F(5.3f, 0.5f, 4.7f));
        Assert.True(hf.ShapeCast(ball, resting, new Vector3F(0, -1, 0), out hit));
        Assert.Equal(0f, hit.Fraction, 4);
        Assert.True(hit.StartsOverlapping);
    }

    [Fact]
    public void CandidatesAreTheTrianglesNearTheShapeInIdOrderWithFlags()
    {
        HeightField3D hf = MeshTestUtil.RandomField(new TestRng(15), 12, 12, integerHeights: false, holes: false);
        ConvexShape3D box = ConvexShape3D.Box(new Vector3F(0.7f, 5f, 0.7f));
        var place = new RigidTransformF(QuaternionF.Identity, new Vector3F(5.5f, 1f, 5.5f));
        var tris = new List<SoupTriangle3D>();
        int n = hf.GetCandidates(box, place, 0.1f, tris);
        Assert.True(n >= 8);
        for (int i = 1; i < n; i++)
        {
            Assert.True(tris[i - 1].Id < tris[i].Id);
        }

        foreach (SoupTriangle3D t in tris)
        {
            Assert.True(hf.TryGetTriangle(t.Id, out SoupTriangle3D again));
            Assert.Equal(again.ActiveEdges, t.ActiveEdges);
            Assert.True(Math.Abs(t.V0.X - 5.5f) < 2f && Math.Abs(t.V0.Z - 5.5f) < 2f);
        }

        // A triangle is a valid convex shape: GJK distance from a sphere above it.
        var buffer = new TriangleShapeBuffer3D();
        SoupTriangle3D first = tris[0];
        ConvexShape3D shape = buffer.Set(first.V0, first.V1, first.V2);
        Vector3F centre = (first.V0 + first.V1 + first.V2) / 3f;
        Vector3F up = first.Normal * 2f;
        ConvexDistance3D dist = Gjk3D.Distance(shape, ConvexShape3D.Sphere(0.5f), new RigidTransformF(QuaternionF.Identity, centre + up));
        Assert.False(dist.Overlapping);
        Assert.Equal(1.5f, dist.Distance, 3);
    }

    [Fact]
    public void MeshesAndHeightFieldsAreStaticOnly()
    {
        Assert.True(MeshTestUtil.Cube().IsStaticOnly);
        Assert.Throws<StaticOnlyShapeException>(() => MeshTestUtil.Cube().GetMassProperties());
        HeightField3D hf = HeightField3D.Create(new float[4], 2, 2, new Vector3F(1, 1, 1));
        Assert.Throws<StaticOnlyShapeException>(() => hf.GetMassProperties());
        Assert.ThrowsAny<NotSupportedException>(() => hf.GetMassProperties());
    }

    [Fact]
    public void BadQueryInputsAreRejected()
    {
        TriangleMesh3D cube = MeshTestUtil.Cube();
        Assert.Throws<ArgumentOutOfRangeException>(() => cube.RayCast(new Vector3F(float.NaN, 0, 0), Vector3F.UnitX, 1f, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => cube.RayCast(default, new Vector3F(float.PositiveInfinity, 0, 0), 1f, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => cube.RayCast(default, Vector3F.UnitX, -1f, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => cube.RayCast(default, Vector3F.UnitX, float.NaN, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => cube.Overlap(new Bounds3F(new Vector3F(1, 1, 1), new Vector3F(0, 0, 0)), new List<int>()));
        Assert.Throws<ArgumentOutOfRangeException>(() => cube.Overlap(new Bounds3F(default, new Vector3F(float.NaN, 0, 0)), new List<int>()));
        Assert.Throws<ArgumentOutOfRangeException>(() => cube.ShapeCast(ConvexShape3D.Sphere(1f), new RigidTransformF(new QuaternionF(0, 0, 0, 2f), default), Vector3F.UnitX, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => cube.ShapeCast(ConvexShape3D.Sphere(1f), RigidTransformF.Identity, new Vector3F(float.NaN, 0, 0), out _));
    }
}
