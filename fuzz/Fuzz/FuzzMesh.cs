using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Collision3D;

/// <summary>Fuzz targets for triangle mesh and height field construction from hostile arrays, and for their queries on random shapes.</summary>
internal static class FuzzMesh
{
    private static readonly float[] Scales = { 1f, 1f, 0.5f, 2f, 0.25f, 1f, 3f, 0.125f, 1f, 1f, 2f, 0f, -1f, float.NaN, 1e6f, float.PositiveInfinity };

    private static float Lattice(byte b) => (sbyte)b / 8f;

    private static Vector3F[] DecodeVertices(byte[] data, ref int pos, int maxCount)
    {
        int mode = data[0] & 3;
        int count = Math.Min(maxCount, (data.Length - pos) / 2 / (mode == 0 ? 12 : 3));
        var vs = new Vector3F[count];
        double scale = (data[1] & 7) switch { 0 => 1e-6, 1 => 1e-3, 6 => 1e3, 7 => 1e6, _ => 1.0 };
        for (int i = 0; i < count; i++)
        {
            if (mode == 0)
            {
                vs[i] = new Vector3F(BitConverter.ToSingle(data, pos), BitConverter.ToSingle(data, pos + 4), BitConverter.ToSingle(data, pos + 8));
                pos += 12;
            }
            else
            {
                vs[i] = new Vector3F((float)(Lattice(data[pos]) * scale), (float)(Lattice(data[pos + 1]) * scale), (float)(Lattice(data[pos + 2]) * scale));
                pos += 3;
            }
        }

        return vs;
    }

    // Raw float bits, a coarse lattice (many coincident, collinear and coplanar triangles) and lattices at tiny and huge scales; indices mostly valid, sometimes wild.
    private static void CheckSoup(TriangleSoup3D s, ulong seed)
    {
        var rng = new Rng(seed);
        Bounds3F b = s.Bounds;
        Vector3F size = b.Size;
        float extent = Math.Max(1e-30f, Math.Max(size.X, Math.Max(size.Y, size.Z)));
        var ids = new List<int>();
        for (int q = 0; q < 8; q++)
        {
            Vector3F o = new(b.Min.X + (rng.Next() * size.X * 1.2f), b.Min.Y + (rng.Next() * size.Y * 1.2f), b.Min.Z + (rng.Next() * size.Z * 1.2f));
            Vector3F d = new((rng.Next() - 0.5f) * 2f, (rng.Next() - 0.5f) * 2f, (rng.Next() - 0.5f) * 2f);
            float max = rng.Next() * extent * 3f;
            if (!float.IsFinite(o.X + o.Y + o.Z) || !float.IsFinite(d.X + d.Y + d.Z) || !float.IsFinite(max))
            {
                continue;
            }

            bool got = s.RayCast(o, d, max, out TriangleRayHit3D hit);
            bool want = false;
            int wantId = -1;
            double wantT = 0;
            for (int k = 0; k < s.TriangleIdCount; k++)
            {
                if (s.TryGetTriangle(k, out SoupTriangle3D t) && d != default && TriangleTests3D.RayTriangle(o, d, t.V0, t.V1, t.V2, false, out double tt, out _) && tt <= max && (!want || tt < wantT))
                {
                    want = true;
                    wantId = k;
                    wantT = tt;
                }
            }

            if (want != got || (got && (hit.TriangleId != wantId || hit.Fraction != (float)wantT)))
            {
                throw new InvalidOperationException($"ray {o} + t*{d} (max {max}): brute {want} id {wantId} t {wantT}, query {got} id {hit.TriangleId} t {hit.Fraction}");
            }

            Vector3F c = new(b.Min.X + (rng.Next() * size.X), b.Min.Y + (rng.Next() * size.Y), b.Min.Z + (rng.Next() * size.Z));
            Vector3F h = new(rng.Next() * extent * 0.3f, rng.Next() * extent * 0.3f, rng.Next() * extent * 0.3f);
            var box = new Bounds3F(c - h, c + h);
            if (!float.IsFinite(box.Min.X + box.Min.Y + box.Min.Z + box.Max.X + box.Max.Y + box.Max.Z))
            {
                continue;
            }

            ids.Clear();
            s.Overlap(box, ids);
            int n = 0;
            for (int k = 0; k < s.TriangleIdCount; k++)
            {
                if (s.TryGetTriangle(k, out SoupTriangle3D t) && TriangleTests3D.BoxTriangle(box, t.V0, t.V1, t.V2))
                {
                    if (n >= ids.Count || ids[n] != k)
                    {
                        throw new InvalidOperationException($"overlap of {box}: triangle {k} missing or out of order");
                    }

                    n++;
                }
            }

            if (n != ids.Count)
            {
                throw new InvalidOperationException($"overlap of {box}: {ids.Count} ids, brute force {n}");
            }
        }

        if (s.TriangleCount > 0 && s.TriangleCount < 64)
        {
            var place = new RigidTransformF(QuaternionF.Identity, b.Center + new Vector3F(0, extent, 0));
            s.ShapeCast(ConvexShape3D.Sphere(Math.Max(extent * 0.1f, 1e-6f)), place, new Vector3F(0, -2f * extent, 0), out _);
        }
    }

    public static void MeshTarget(byte[] data)
    {
        if (data.Length < 6)
        {
            return;
        }

        int pos = 4;
        Vector3F[] vs = DecodeVertices(data, ref pos, 40);
        bool wild = (data[2] & 1) != 0;
        int width = wild ? 2 : 1;
        var idx = new int[Math.Max(0, (data.Length - pos) / width) / 3 * 3];
        for (int i = 0; i < idx.Length; i++)
        {
            if (wild)
            {
                idx[i] = (data[pos] | (data[pos + 1] << 8)) - 4;
                pos += 2;
            }
            else
            {
                idx[i] = vs.Length == 0 ? data[pos] : data[pos] % vs.Length;
                pos++;
            }
        }

        TriangleMesh3D m;
        try
        {
            m = TriangleMesh3D.Create(vs, idx, new MeshBuildOptions { FlatEdgeTolerance = (data[3] & 1) != 0 ? 0.3 : 0 });
        }
        catch (MeshBuildException)
        {
            return;
        }

        if (m.TriangleCount + m.DroppedTriangles.Length != idx.Length / 3 || m.TriangleCount < 1)
        {
            throw new InvalidOperationException("kept and dropped triangles do not add up");
        }

        CheckSoup(m, BitConverter.ToUInt32(data, 0) | ((ulong)data.Length << 40));
    }

    public static void HeightFieldTarget(byte[] data)
    {
        if (data.Length < 8)
        {
            return;
        }

        int nx = (data[4] % 14) - 1, nz = (data[5] % 11) - 1;
        int pos = 8;
        int cells = Math.Max(0, (nx - 1) * (nz - 1));
        int count = Math.Max(0, nx * nz);
        bool rawBits = (data[0] & 3) == 0;
        var h = new float[(data[6] & 1) != 0 ? count : Math.Max(0, count - 1)];
        for (int i = 0; i < h.Length; i++)
        {
            if (rawBits && pos + 4 <= data.Length)
            {
                h[i] = BitConverter.ToSingle(data, pos);
                pos += 4;
            }
            else
            {
                h[i] = pos < data.Length ? Lattice(data[pos++]) : 0f;
            }
        }

        var mask = new bool[(data[6] & 2) != 0 ? cells : 0];
        for (int i = 0; i < mask.Length; i++)
        {
            mask[i] = pos < data.Length && (data[pos++] & 3) == 0;
        }

        var scale = new Vector3F(Scales[data[7] & 15], Scales[(data[7] >> 4) & 15], Scales[(data[6] >> 4) & 15]);
        HeightField3D hf;
        try
        {
            hf = HeightField3D.Create(h, nx, nz, scale, mask);
        }
        catch (MeshBuildException)
        {
            return;
        }

        CheckSoup(hf, BitConverter.ToUInt32(data, 0) | ((ulong)data.Length << 40));
    }

    // Valid random meshes (jittered and degenerate-prone lattices) from a seed, with a long query sequence against brute force.
    public static void QueryTarget(byte[] data)
    {
        if (data.Length < 8)
        {
            return;
        }

        var rng = new Rng(BitConverter.ToUInt64(data, 0));
        int triCount = 1 + (int)(rng.Next() * 60);
        var vs = new Vector3F[3 * triCount];
        var idx = new int[3 * triCount];
        bool lattice = (data[0] & 1) != 0;
        for (int i = 0; i < vs.Length; i++)
        {
            vs[i] = lattice ? new Vector3F((int)(rng.Next() * 5), (int)(rng.Next() * 3), (int)(rng.Next() * 5)) : new Vector3F(rng.Next() * 6f, rng.Next() * 2f, rng.Next() * 6f);
            idx[i] = i;
        }

        TriangleMesh3D m;
        try
        {
            m = TriangleMesh3D.Create(vs, idx);
        }
        catch (MeshBuildException)
        {
            return;
        }

        for (int rep = 0; rep < 4; rep++)
        {
            CheckSoup(m, rng.NextU64());
        }
    }

    private sealed class Rng
    {
        private ulong _s;

        public Rng(ulong seed) => _s = seed;

        public ulong NextU64()
        {
            ulong z = _s += 0x9E3779B97F4A7C15UL;
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
            z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
            return z ^ (z >> 31);
        }

        /// <summary>Uniform in [0, 1).</summary>
        public float Next() => (NextU64() >> 40) * (1f / 16777216f);
    }
}
