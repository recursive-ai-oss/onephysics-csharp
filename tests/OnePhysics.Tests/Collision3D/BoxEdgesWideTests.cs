using System;
using System.Runtime.Intrinsics;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Collision3D;

/// <summary>The eight-pair edge-axis kernel gives, lane by lane, the bits of the scalar box edge search.</summary>
public class BoxEdgesWideTests
{
    private static void Set(ref Vector256<float> v, int lane, float x) => v = v.WithElement(lane, x);

    private static QuaternionF PoseRotation(TestRng rng)
    {
        switch (rng.Next(6))
        {
            case 0: return QuaternionF.Identity;
            case 1: return new QuaternionF(0.70710678f, 0f, 0f, 0.70710678f); // a quarter turn about x
            case 2: { var q = rng.Rotation(); return new QuaternionF(q.X * 1e-3f, q.Y * 1e-3f, q.Z * 1e-3f, 1f).Normalized(); } // nearly axis-aligned
            default: return rng.Rotation();
        }
    }

    [Fact]
    public void LanesEqualTheScalarSearchBitForBit()
    {
        var rng = new TestRng(77);
        int withEdge = 0;
        for (int round = 0; round < 4000; round++)
        {
            var p = new BoxPairLanes3D();
            var ha = new Vector3F[8]; var hb = new Vector3F[8]; var rr = new Matrix3x3F[8]; var tt = new Vector3F[8];
            for (int l = 0; l < 8; l++)
            {
                ha[l] = new Vector3F(rng.Range(0.05f, 2f), rng.Range(0.05f, 2f), rng.Range(0.05f, 2f));
                hb[l] = new Vector3F(rng.Range(0.05f, 2f), rng.Range(0.05f, 2f), rng.Range(0.05f, 2f));
                rr[l] = PoseRotation(rng).ToMatrix();
                tt[l] = rng.InBall(rng.Range(0.1f, 4f));
                if (round % 97 == 0 && l == 3) { tt[l] = new Vector3F(float.NaN, 0f, 1f); }
                if (round % 89 == 0 && l == 5) { rr[l] = default; }
                Set(ref p.HAx, l, ha[l].X); Set(ref p.HAy, l, ha[l].Y); Set(ref p.HAz, l, ha[l].Z);
                Set(ref p.HBx, l, hb[l].X); Set(ref p.HBy, l, hb[l].Y); Set(ref p.HBz, l, hb[l].Z);
                Set(ref p.R11, l, rr[l].M11); Set(ref p.R12, l, rr[l].M12); Set(ref p.R13, l, rr[l].M13);
                Set(ref p.R21, l, rr[l].M21); Set(ref p.R22, l, rr[l].M22); Set(ref p.R23, l, rr[l].M23);
                Set(ref p.R31, l, rr[l].M31); Set(ref p.R32, l, rr[l].M32); Set(ref p.R33, l, rr[l].M33);
                Set(ref p.Tx, l, tt[l].X); Set(ref p.Ty, l, tt[l].Y); Set(ref p.Tz, l, tt[l].Z);
            }

            var res = BoxEdgesWide3D.Run(in p);
            for (int l = 0; l < 8; l++)
            {
                float best = ManifoldGenerator3D.BoxEdges(ha[l], hb[l], rr[l], tt[l], out int ea, out int eb, out Vector3F axis);
                Assert.Equal(BitConverter.SingleToInt32Bits(best), BitConverter.SingleToInt32Bits(res.Best[l]));
                int index = (int)res.Index[l];
                if (ea < 0) { Assert.Equal(-1, index); continue; }
                withEdge++;
                Assert.Equal(BitConverter.SingleToInt32Bits(axis.X), BitConverter.SingleToInt32Bits(res.Nx[l]));
                Assert.Equal(BitConverter.SingleToInt32Bits(axis.Y), BitConverter.SingleToInt32Bits(res.Ny[l]));
                Assert.Equal(BitConverter.SingleToInt32Bits(axis.Z), BitConverter.SingleToInt32Bits(res.Nz[l]));
                Assert.Equal(ea, ManifoldGenerator3D.BoxEdgeHalfEdge(index / 3, (int)res.Ka[l]));
                Assert.Equal(eb, ManifoldGenerator3D.BoxEdgeHalfEdge(index % 3, (int)res.Kb[l]));
            }
        }

        Assert.True(withEdge > 4000, $"only {withEdge} lanes found an edge axis");
    }
}
