using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Collision3D;

public class HullTests
{
    /// <summary>Checks every structural and geometric property a hull promises; returns the largest distance of an input point outside a face plane.</summary>
    internal static double Validate(ConvexHull3D hull, ReadOnlySpan<Vector3F> input, bool inputInside = true)
    {
        var vs = hull.Vertices;
        var fs = hull.Faces;
        var es = hull.Edges;
        int V = vs.Length, E = es.Length / 2, F = fs.Length;
        Assert.Equal(0, es.Length % 2);
        Assert.Equal(2, V - E + F);
        // Half-edge consistency.
        for (int e = 0; e < es.Length; e++)
        {
            int twin = e ^ 1;
            int next = es[e].Next;
            Assert.Equal(es[e].Face, es[next].Face);
            Assert.Equal(es[next].Origin, es[twin].Origin); // next starts where twin starts: edge e ends at e's destination
            Assert.NotEqual(es[e].Face, es[twin].Face);
            Assert.NotEqual(es[e].Origin, es[twin].Origin);
        }

        // Faces: planar, convex, outward, polygon rings close.
        int edgeSum = 0;
        for (int f = 0; f < F; f++)
        {
            HullFace3D face = fs[f];
            Assert.True(face.EdgeCount >= 3);
            edgeSum += face.EdgeCount;
            var ring = new List<int>();
            int e = face.FirstEdge;
            for (int k = 0; k < face.EdgeCount; k++)
            {
                Assert.Equal(f, es[e].Face);
                ring.Add(es[e].Origin);
                e = es[e].Next;
            }

            Assert.Equal(face.FirstEdge, e);
            double nl = Math.Sqrt(((double)face.Normal.X * face.Normal.X) + ((double)face.Normal.Y * face.Normal.Y) + ((double)face.Normal.Z * face.Normal.Z));
            Assert.InRange(nl, 1 - 1e-5, 1 + 1e-5);
            double scale = 1;
            foreach (Vector3F v in vs)
            {
                scale = Math.Max(scale, Math.Max(Math.Abs(v.X), Math.Max(Math.Abs(v.Y), Math.Abs(v.Z))));
            }

            double tol = 3e-6 * scale;
            foreach (int r in ring)
            {
                double d = Dot(face.Normal, vs[r]) - face.Offset;
                Assert.True(Math.Abs(d) <= tol * 4, $"face {f} not planar: {d}");
            }

            for (int k = 0; k < ring.Count; k++)
            {
                // Convex corner: the turn at each vertex is counter-clockwise about the outward normal.
                Vector3F a = vs[ring[k]], b = vs[ring[(k + 1) % ring.Count]], c = vs[ring[(k + 2) % ring.Count]];
                Vector3F cr = Vector3F.Cross(b - a, c - b);
                Assert.True(Dot(face.Normal, cr) >= -1e-5 * scale * scale, $"face {f} has a reflex corner");
            }
        }

        Assert.Equal(es.Length, edgeSum);
        // Every vertex lies behind or on every face; every vertex is a corner of at least 3 faces.
        var faceCount = new int[V];
        foreach (HullEdge3D e in es)
        {
            faceCount[e.Origin]++;
        }

        for (int i = 0; i < V; i++)
        {
            Assert.True(faceCount[i] >= 3, $"vertex {i} has {faceCount[i]} edges");
            Assert.Equal(i, es[hull.VertexEdges[i]].Origin);
            for (int f = 0; f < F; f++)
            {
                double out_ = Dot(fs[f].Normal, vs[i]) - fs[f].Offset;
                Assert.True(out_ <= 1e-5 * Math.Max(1, Math.Abs(fs[f].Offset)) + 4e-6, $"vertex {i} is {out_} outside face {f} ({fs[f].EdgeCount} edges, V={V})");
            }
        }

        // No two adjacent faces left coplanar: some vertex of one must sit clearly off the other's plane.
        double ext = 0;
        foreach (Vector3F v in vs)
        {
            ext = Math.Max(ext, Math.Max(Math.Abs(v.X), Math.Max(Math.Abs(v.Y), Math.Abs(v.Z))));
        }

        for (int e = 0; e < es.Length; e += 2)
        {
            double off01 = 0, off10 = 0;
            foreach (int fi in new[] { es[e].Face, es[e + 1].Face })
            {
                int g = fi == es[e].Face ? es[e + 1].Face : es[e].Face;
                int ee = fs[g].FirstEdge;
                for (int k = 0; k < fs[g].EdgeCount; k++)
                {
                    double d = Math.Abs(Dot(fs[fi].Normal, vs[es[ee].Origin]) - fs[fi].Offset);
                    if (fi == es[e].Face)
                    {
                        off01 = Math.Max(off01, d);
                    }
                    else
                    {
                        off10 = Math.Max(off10, d);
                    }

                    ee = es[ee].Next;
                }
            }

            Assert.True(off01 > 3e-7 * ext || off10 > 3e-7 * ext, "adjacent coplanar faces were not merged");
        }

        double worst = 0;
        if (inputInside)
        {
            foreach (Vector3F p in input)
            {
                for (int f = 0; f < F; f++)
                {
                    worst = Math.Max(worst, Dot(fs[f].Normal, p) - fs[f].Offset);
                }
            }
        }

        return worst;
    }

    private static double Dot(Vector3F a, Vector3F b) => ((double)a.X * b.X) + ((double)a.Y * b.Y) + ((double)a.Z * b.Z);

    private static double Extent(ReadOnlySpan<Vector3F> pts)
    {
        double m = 0;
        foreach (Vector3F p in pts)
        {
            m = Math.Max(m, Math.Max(Math.Abs(p.X), Math.Max(Math.Abs(p.Y), Math.Abs(p.Z))));
        }

        return m;
    }

    [Fact]
    public void CubeIsEightVerticesSixQuads()
    {
        var pts = new List<Vector3F>();
        for (int i = 0; i < 8; i++)
        {
            pts.Add(new Vector3F((i & 1) * 2 - 1, ((i >> 1) & 1) * 2 - 1, ((i >> 2) & 1) * 2 - 1));
        }

        pts.Add(Vector3F.Zero); // interior
        pts.Add(new Vector3F(1, 0, 0)); // on a face: dropped
        pts.Add(new Vector3F(1, 1, 0)); // on an edge: dropped
        ConvexHull3D hull = ConvexHull3D.Build(pts.ToArray());
        Validate(hull, pts.ToArray());
        Assert.Equal(8, hull.Vertices.Length);
        Assert.Equal(6, hull.Faces.Length);
        Assert.Equal(12, hull.Edges.Length / 2);
        foreach (HullFace3D f in hull.Faces)
        {
            Assert.Equal(4, f.EdgeCount);
        }

        Assert.Equal(8f, hull.MassProperties.Mass, 4);
        Assert.Equal(0f, hull.MassProperties.Center.X, 5);
        Assert.Equal(8f * 8f / 6f * (1f / 4f) * 2f, hull.MassProperties.Inertia.M11, 3); // m*(2^2+2^2)/12 = 8*8/12
    }

    [Fact]
    public void TetrahedronAndSliverStillBuild()
    {
        var t = new[] { new Vector3F(0, 0, 0), new Vector3F(1, 0, 0), new Vector3F(0, 1, 0), new Vector3F(0, 0, 1) };
        ConvexHull3D hull = ConvexHull3D.Build(t);
        Validate(hull, t);
        Assert.Equal(4, hull.Faces.Length);
        Assert.Equal(1f / 6f, hull.MassProperties.Mass, 5);
        var sliver = new[] { new Vector3F(0, 0, 0), new Vector3F(100, 0, 0), new Vector3F(0, 100, 0), new Vector3F(50, 50, 0.01f) };
        Validate(ConvexHull3D.Build(sliver), sliver);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    [InlineData(5)]
    public void RandomPointSetsAreContainedAndWellFormed(int kind)
    {
        var rng = new TestRng(1000UL + (ulong)kind);
        int worstCount = 0;
        for (int iter = 0; iter < 400; iter++)
        {
            int n = 4 + rng.Next(kind == 5 ? 400 : 120);
            var pts = new Vector3F[n];
            float scale = kind == 4 ? rng.Range(1e-3f, 1e5f) : 1f;
            for (int i = 0; i < n; i++)
            {
                switch (kind)
                {
                    case 0: pts[i] = rng.InCube(1f); break;
                    case 1: pts[i] = rng.OnSphere(); break;
                    case 2: pts[i] = new Vector3F(rng.Next(5) - 2, rng.Next(5) - 2, rng.Next(5) - 2); break; // lattice: coplanar, collinear, duplicates
                    case 3: pts[i] = rng.InBall(1f) * new Vector3F(1f, 0.001f, 1f) + rng.InBall(1e-4f); break; // nearly flat
                    case 4: pts[i] = rng.InBall(scale); break;
                    default: pts[i] = rng.OnSphere() * (1f + (rng.Range(0f, 1e-6f))); break; // nearly all on the hull
                }
            }

            ConvexHull3D hull;
            try
            {
                hull = ConvexHull3D.Build(pts);
            }
            catch (HullBuildException ex)
            {
                Assert.Equal(HullBuildError.Degenerate, ex.Error);
                worstCount++;
                continue;
            }

            double worst = Validate(hull, pts);
            double ext = Extent(pts);
            Assert.True(worst <= 4e-6 * Math.Max(ext, 1e-30) * 4 + 1e-30 || worst <= 3.6e-7 * 3 * ext * 3 + 1e-30, $"kind {kind} iter {iter}: a point is {worst} outside (extent {ext})");
        }

        // Only the lattice and flat sets may be rejected.
        Assert.True(worstCount < 400 / 2);
    }

    [Fact]
    public void LatticeVolumeMatchesBox()
    {
        var pts = new List<Vector3F>();
        for (int x = 0; x < 3; x++)
        {
            for (int y = 0; y < 4; y++)
            {
                for (int z = 0; z < 5; z++)
                {
                    pts.Add(new Vector3F(x, y, z));
                }
            }
        }

        ConvexHull3D hull = ConvexHull3D.Build(pts.ToArray());
        Validate(hull, pts.ToArray());
        Assert.Equal(8, hull.Vertices.Length);
        Assert.Equal(6, hull.Faces.Length);
        Assert.Equal(2f * 3f * 4f, hull.MassProperties.Mass, 3);
        Assert.Equal(1f, hull.MassProperties.Center.X, 4);
        Assert.Equal(1.5f, hull.MassProperties.Center.Y, 4);
        Assert.Equal(2f, hull.MassProperties.Center.Z, 4);
    }

    [Fact]
    public void HostileInputIsRejectedWithTypedErrors()
    {
        static HullBuildException Fail(Vector3F[] pts, HullBuildOptions o = default) => Assert.Throws<HullBuildException>(() => ConvexHull3D.Build(pts, o));
        Assert.Equal(HullBuildError.TooFewPoints, Fail(new[] { Vector3F.Zero, Vector3F.UnitX, Vector3F.UnitY }).Error);
        Assert.Equal(HullBuildError.TooFewPoints, Fail(Array.Empty<Vector3F>()).Error);
        var good = new[] { Vector3F.Zero, Vector3F.UnitX, Vector3F.UnitY, Vector3F.UnitZ };
        var nan = (Vector3F[])good.Clone();
        nan[2] = new Vector3F(0, float.NaN, 0);
        HullBuildException ex = Fail(nan);
        Assert.Equal(HullBuildError.NonFinite, ex.Error);
        Assert.Contains("points[2]", ex.Message, StringComparison.Ordinal);
        nan[2] = new Vector3F(0, float.PositiveInfinity, 0);
        Assert.Equal(HullBuildError.NonFinite, Fail(nan).Error);
        nan[2] = new Vector3F(3e6f, 0, 0);
        Assert.Equal(HullBuildError.OutOfRange, Fail(nan).Error);
        Assert.Equal(HullBuildError.Degenerate, Fail(new[] { Vector3F.One, Vector3F.One, Vector3F.One, Vector3F.One, Vector3F.One }).Error);
        Assert.Equal(HullBuildError.Degenerate, Fail(new[] { Vector3F.Zero, Vector3F.UnitX, new Vector3F(2, 0, 0), new Vector3F(3, 0, 0) }).Error);
        Assert.Equal(HullBuildError.Degenerate, Fail(new[] { Vector3F.Zero, Vector3F.UnitX, Vector3F.UnitY, new Vector3F(1, 1, 0), new Vector3F(0.5f, 0.5f, 1e-9f) }).Error);
        Assert.Equal(HullBuildError.InvalidOptions, Fail(good, new HullBuildOptions { MaxVertices = 3 }).Error);
        Assert.Equal(HullBuildError.InvalidOptions, Fail(good, new HullBuildOptions { Tolerance = -1f }).Error);
        Assert.Equal(HullBuildError.InvalidOptions, Fail(good, new HullBuildOptions { Tolerance = float.NaN }).Error);
    }

    [Fact]
    public void NearDuplicatePointsAndTinyAndHugeScales()
    {
        var rng = new TestRng(77);
        foreach (float scale in new[] { 1e-8f, 1e-4f, 1f, 1e3f, 1e6f })
        {
            var pts = new List<Vector3F>();
            for (int i = 0; i < 40; i++)
            {
                Vector3F p = rng.OnSphere() * scale;
                pts.Add(p);
                pts.Add(p + (rng.OnSphere() * (scale * 1e-9f))); // near duplicate
                pts.Add(p);                                      // exact duplicate
            }

            ConvexHull3D hull = ConvexHull3D.Build(pts.ToArray());
            Validate(hull, pts.ToArray());
        }
    }

    [Fact]
    public void VertexAndFaceLimitsAreHonoured()
    {
        var rng = new TestRng(5);
        var pts = new Vector3F[500];
        for (int i = 0; i < pts.Length; i++)
        {
            pts[i] = rng.OnSphere();
        }

        ConvexHull3D full = ConvexHull3D.Build(pts);
        Assert.True(full.Vertices.Length > 100);
        ConvexHull3D limited = ConvexHull3D.Build(pts, new HullBuildOptions { MaxVertices = 20 });
        Assert.InRange(limited.Vertices.Length, 4, 20);
        Validate(limited, pts, inputInside: false);
        ConvexHull3D fl = ConvexHull3D.Build(pts, new HullBuildOptions { MaxFaces = 30 });
        Assert.InRange(fl.Faces.Length, 4, 30);
        Validate(fl, pts, inputInside: false);
        ConvexHull3D minimal = ConvexHull3D.Build(pts, new HullBuildOptions { MaxVertices = 4 });
        Assert.Equal(4, minimal.Vertices.Length);
    }

    [Fact]
    public void BuildIsDeterministicAndIndependentOfThreads()
    {
        var rng = new TestRng(31);
        var pts = new Vector3F[300];
        for (int i = 0; i < pts.Length; i++)
        {
            pts[i] = rng.InBall(2f);
        }

        ConvexHull3D first = ConvexHull3D.Build(pts);
        var results = new ConvexHull3D[4];
        var threads = new System.Threading.Thread[4];
        for (int t = 0; t < 4; t++)
        {
            int ti = t;
            threads[t] = new System.Threading.Thread(() => results[ti] = ConvexHull3D.Build(pts));
            threads[t].Start();
        }

        foreach (var th in threads)
        {
            th.Join();
        }

        foreach (ConvexHull3D r in results)
        {
            Assert.True(first.Vertices.SequenceEqual(r.Vertices));
            Assert.Equal(first.Edges.Length, r.Edges.Length);
            for (int e = 0; e < first.Edges.Length; e++)
            {
                Assert.Equal(first.Edges[e].Origin, r.Edges[e].Origin);
                Assert.Equal(first.Edges[e].Next, r.Edges[e].Next);
            }
        }
    }
}
