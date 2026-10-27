using System.Collections.Generic;
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Collision3D;

namespace RecursiveAI.OnePhysics.HashMatrix;

/// <summary>
/// Triangle mesh and height field construction and queries from fixed seeds: the BVH's canonical bytes, every triangle's active-edge flags,
/// then ray casts (closest, any, all), box overlaps, convex shape casts and candidate lists, each hashed in its own step.
/// </summary>
internal static class Mesh3DScenarios
{
    private sealed class Rng
    {
        private ulong _s;

        public Rng(ulong seed) => _s = seed;

        public ulong Next()
        {
            ulong z = _s += 0x9E3779B97F4A7C15UL;
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
            z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
            return z ^ (z >> 31);
        }

        public float F() => (Next() >> 40) * (1f / 16777216f);

        public float R(float lo, float hi) => lo + ((hi - lo) * F());

        public int I(int n) => (int)(Next() % (ulong)n);
    }

    [ModuleInitializer]
    internal static void Init() => Scenarios.Register("mesh3d/queries", Run);

    private static void Run(ScenarioContext ctx)
    {
        var rng = new Rng(20261027);

        // A jittered terrain mesh, a scattered soup with degenerate triangles mixed in, and a height field with holes.
        const int N = 48;
        var tv = new Vector3F[N * N];
        for (int j = 0; j < N; j++)
        {
            for (int i = 0; i < N; i++)
            {
                tv[(j * N) + i] = new Vector3F((i + rng.R(-0.3f, 0.3f)) * 0.5f, rng.R(0f, 2f), (j + rng.R(-0.3f, 0.3f)) * 0.5f);
            }
        }

        var ti = new List<int>();
        for (int j = 0; j + 1 < N; j++)
        {
            for (int i = 0; i + 1 < N; i++)
            {
                int a = (j * N) + i, b = a + 1, c = a + N, d = c + 1;
                if (rng.I(2) == 0)
                {
                    ti.AddRange(new[] { a, c, d, a, d, b });
                }
                else
                {
                    ti.AddRange(new[] { a, c, b, b, c, d });
                }
            }
        }

        TriangleMesh3D terrain = TriangleMesh3D.Create(tv, ti.ToArray());

        var sv = new Vector3F[3 * 600];
        var si = new int[sv.Length];
        for (int t = 0; t < 600; t++)
        {
            var c = new Vector3F(rng.R(0f, 12f), rng.R(0f, 4f), rng.R(0f, 12f));
            for (int k = 0; k < 3; k++)
            {
                sv[(3 * t) + k] = t % 17 == 0 && k == 2 ? sv[3 * t] : c + new Vector3F(rng.R(-0.6f, 0.6f), rng.R(-0.6f, 0.6f), rng.R(-0.6f, 0.6f));
                si[(3 * t) + k] = (3 * t) + k;
            }
        }

        TriangleMesh3D soup = TriangleMesh3D.Create(sv, si);

        const int HX = 70, HZ = 53;
        var heights = new float[HX * HZ];
        for (int i = 0; i < heights.Length; i++)
        {
            heights[i] = rng.I(3) == 0 ? rng.I(4) : rng.R(0f, 3f);
        }

        var holes = new bool[(HX - 1) * (HZ - 1)];
        for (int i = 0; i < holes.Length; i++)
        {
            holes[i] = rng.I(9) == 0;
        }

        HeightField3D field = HeightField3D.Create(heights, HX, HZ, new Vector3F(0.5f, 1.25f, 0.5f), holes);

        // Step 0: construction results.
        ctx.Write(terrain.ToBvhBytes());
        ctx.Write(soup.ToBvhBytes());
        ctx.Write(terrain.DroppedTriangles.Length);
        ctx.Write(soup.DroppedTriangles.Length);
        TriangleSoup3D[] shapes = { terrain, soup, field };
        foreach (TriangleSoup3D s in shapes)
        {
            ctx.Write(s.TriangleCount);
            WriteBounds(ctx, s.Bounds);
            for (int id = 0; id < s.TriangleIdCount; id++)
            {
                ctx.Write(s.TryGetTriangle(id, out SoupTriangle3D t) ? 1 + t.ActiveEdges : 0);
            }
        }

        ctx.EndStep();

        // Steps 1..3: rays, overlaps and casts, per shape.
        var hits = new List<TriangleRayHit3D>();
        var ids = new List<int>();
        var cands = new List<SoupTriangle3D>();
        foreach (TriangleSoup3D s in shapes)
        {
            Bounds3F b = s.Bounds;
            Vector3F size = b.Size;
            for (int q = 0; q < 1500; q++)
            {
                var o = new Vector3F(b.Min.X + (rng.R(-0.1f, 1.1f) * size.X), b.Max.Y + rng.R(0f, 2f), b.Min.Z + (rng.R(-0.1f, 1.1f) * size.Z));
                Vector3F d = q % 3 == 0 ? new Vector3F(0, -1, 0) : new Vector3F(rng.R(-1f, 1f), rng.R(-3f, -0.2f), rng.R(-1f, 1f));
                float max = rng.R(0.5f, 12f);
                if (s.RayCast(o, d, max, out TriangleRayHit3D h, q % 5 == 0))
                {
                    ctx.Write(h.TriangleId);
                    ctx.Write(h.Fraction);
                    ctx.Write(h.Point.X);
                    ctx.Write(h.Point.Y);
                    ctx.Write(h.Point.Z);
                    ctx.Write(h.Normal.Y);
                    ctx.Write(h.BackFace ? 1 : 0);
                }
                else
                {
                    ctx.Write(-1);
                }

                ctx.Write(s.RayCastAny(o, d, max, out TriangleRayHit3D any) ? any.TriangleId : -1);
                if (q % 10 == 0)
                {
                    hits.Clear();
                    ctx.Write(s.RayCastAll(o, d, max, hits));
                    foreach (TriangleRayHit3D x in hits)
                    {
                        ctx.Write(x.TriangleId);
                        ctx.Write(x.Fraction);
                    }
                }
            }

            ctx.EndStep();

            for (int q = 0; q < 800; q++)
            {
                Vector3F c = new(b.Min.X + (rng.R(0f, 1f) * size.X), b.Min.Y + (rng.R(-0.2f, 1.2f) * size.Y), b.Min.Z + (rng.R(0f, 1f) * size.Z));
                Vector3F h = new(rng.R(0f, 1f), rng.R(0f, 1f), rng.R(0f, 1f));
                ids.Clear();
                ctx.Write(s.Overlap(new Bounds3F(c - h, c + h), ids));
                foreach (int id in ids)
                {
                    ctx.Write(id);
                }
            }

            for (int q = 0; q < 200; q++)
            {
                ConvexShape3D shape = (q % 3) switch
                {
                    0 => ConvexShape3D.Sphere(rng.R(0.1f, 0.6f)),
                    1 => ConvexShape3D.Capsule(rng.R(0.1f, 0.5f), rng.R(0.1f, 0.3f)),
                    _ => ConvexShape3D.Box(new Vector3F(rng.R(0.1f, 0.5f), rng.R(0.1f, 0.5f), rng.R(0.1f, 0.5f)), 0.02f),
                };
                var place = new RigidTransformF(QuaternionF.Identity, new Vector3F(b.Min.X + (rng.R(0f, 1f) * size.X), b.Max.Y + rng.R(0f, 1.5f), b.Min.Z + (rng.R(0f, 1f) * size.Z)));
                if (s.ShapeCast(shape, place, new Vector3F(rng.R(-1f, 1f), rng.R(-6f, -1f), rng.R(-1f, 1f)), out SoupCastHit3D hit))
                {
                    ctx.Write(hit.TriangleId);
                    ctx.Write(hit.Fraction);
                    ctx.Write(hit.Normal.X);
                    ctx.Write(hit.Normal.Y);
                    ctx.Write(hit.Normal.Z);
                    ctx.Write(hit.Point.X);
                    ctx.Write(hit.Point.Y);
                    ctx.Write(hit.Point.Z);
                    ctx.Write(hit.StartsOverlapping ? 1 : 0);
                }
                else
                {
                    ctx.Write(-1);
                }

                cands.Clear();
                ctx.Write(s.GetCandidates(shape, place, 0.05f, cands));
                foreach (SoupTriangle3D t in cands)
                {
                    ctx.Write(t.Id);
                    ctx.Write((int)t.ActiveEdges);
                }
            }

            ctx.EndStep();
        }
    }

    private static void WriteBounds(ScenarioContext ctx, Bounds3F b)
    {
        ctx.Write(b.Min.X);
        ctx.Write(b.Min.Y);
        ctx.Write(b.Min.Z);
        ctx.Write(b.Max.X);
        ctx.Write(b.Max.Y);
        ctx.Write(b.Max.Z);
    }
}
