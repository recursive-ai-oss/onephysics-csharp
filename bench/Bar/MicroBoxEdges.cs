using System;
using System.Diagnostics;
using System.Globalization;
using System.Runtime.Intrinsics;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace Bar;

/// <summary>Scalar box edge-axis search against the eight-lane kernel on the same pile-like poses (boxes nearly axis aligned, a few tilted); ns per pair, best of several repetitions.</summary>
static class MicroBoxEdges
{
    public static int Run(Opts o)
    {
        int n = int.Parse(o.Get("pairs", "4096"), CultureInfo.InvariantCulture), reps = int.Parse(o.Get("reps", "200"), CultureInfo.InvariantCulture);
        double tilt = double.Parse(o.Get("tilt", "0.05"), CultureInfo.InvariantCulture);
        ulong s = 12345; double R() { s = s * 6364136223846793005UL + 1442695040888963407UL; return (s >> 11) * (1.0 / 9007199254740992.0); }
        var ha = new Vector3F[n]; var hb = new Vector3F[n]; var rr = new Matrix3x3F[n]; var tt = new Vector3F[n];
        for (int i = 0; i < n; i++)
        {
            ha[i] = new Vector3F((float)(0.3 + R()), (float)(0.3 + R()), (float)(0.3 + R()));
            hb[i] = new Vector3F((float)(0.3 + R()), (float)(0.3 + R()), (float)(0.3 + R()));
            var q = new QuaternionF((float)((R() - .5) * tilt), (float)((R() - .5) * tilt), (float)((R() - .5) * tilt), 1f).Normalized();
            rr[i] = q.ToMatrix();
            tt[i] = new Vector3F((float)((R() - .5) * 3), (float)((R() - .5) * 3), (float)((R() - .5) * 3));
        }

        var lanes = new BoxPairLanes3D[n / 8];
        for (int b = 0; b < n / 8; b++)
        {
            ref var p = ref lanes[b];
            float[] f = new float[18 * 8];
            for (int l = 0; l < 8; l++)
            {
                int i = b * 8 + l;
                float[] v = [ha[i].X, ha[i].Y, ha[i].Z, hb[i].X, hb[i].Y, hb[i].Z, rr[i].M11, rr[i].M12, rr[i].M13, rr[i].M21, rr[i].M22, rr[i].M23, rr[i].M31, rr[i].M32, rr[i].M33, tt[i].X, tt[i].Y, tt[i].Z];
                for (int k = 0; k < 18; k++) f[k * 8 + l] = v[k];
            }
            Vector256<float> L(int k) => Vector256.Create(f, k * 8);
            p.HAx = L(0); p.HAy = L(1); p.HAz = L(2); p.HBx = L(3); p.HBy = L(4); p.HBz = L(5);
            p.R11 = L(6); p.R12 = L(7); p.R13 = L(8); p.R21 = L(9); p.R22 = L(10); p.R23 = L(11); p.R31 = L(12); p.R32 = L(13); p.R33 = L(14); p.Tx = L(15); p.Ty = L(16); p.Tz = L(17);
        }

        double bestS = double.MaxValue, bestW = double.MaxValue; float sink = 0;
        for (int r = 0; r < reps; r++)
        {
            long t0 = Stopwatch.GetTimestamp();
            for (int i = 0; i < n; i++) { sink += ManifoldGenerator3D.BoxEdges(ha[i], hb[i], rr[i], tt[i], out _, out _, out _); }
            double ts = (Stopwatch.GetTimestamp() - t0) * 1e9 / Stopwatch.Frequency / n;
            t0 = Stopwatch.GetTimestamp();
            for (int b = 0; b < lanes.Length; b++) { var res = BoxEdgesWide3D.Run(in lanes[b]); sink += res.Best[0]; }
            double tw = (Stopwatch.GetTimestamp() - t0) * 1e9 / Stopwatch.Frequency / n;
            bestS = Math.Min(bestS, ts); bestW = Math.Min(bestW, tw);
        }

        Console.WriteLine($"pairs {n} tilt {tilt}: scalar {bestS:F1} ns/pair, wide {bestW:F1} ns/pair ({sink > 1e30f})");
        return 0;
    }
}
