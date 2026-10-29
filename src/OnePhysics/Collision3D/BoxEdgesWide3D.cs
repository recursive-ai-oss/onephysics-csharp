using System.Runtime.Intrinsics;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>Eight box-against-box pairs side by side, one pair per lane: half extents of A and B, the rotation of B in A (row-major, entries M11..M33) and B's centre in A's frame.</summary>
internal struct BoxPairLanes3D
{
    public Vector256<float> HAx, HAy, HAz, HBx, HBy, HBz;
    public Vector256<float> R11, R12, R13, R21, R22, R23, R31, R32, R33;
    public Vector256<float> Tx, Ty, Tz;
}

/// <summary>Result of the nine edge-edge axes of eight box pairs. <c>Index</c> is 3 * (axis of A's edge) + (axis of B's edge), or -1 when no pair of edges has an axis; <c>Ka</c> and <c>Kb</c> are the
/// corner-sign codes of the supporting edges (see the scalar routine), all small integers held in float lanes so every lane holds the same type.</summary>
internal struct BoxEdgesResult3D
{
    public Vector256<float> Best, Index, Nx, Ny, Nz, Ka, Kb;
}

/// <summary>
/// The edge-edge part of the box separating axis test (Ericson, Real-Time Collision Detection, 4.4.1; Gregorius, GDC 2013) for eight pairs at once. Every lane performs the operations of
/// <c>ManifoldGenerator3D.BoxEdges</c> in the same order, with compare-and-select where the scalar code branches, so a lane's answer equals the scalar answer bit for bit and the lane count reaches no result.
/// </summary>
internal static class BoxEdgesWide3D
{
    private const float ConeMargin = 1e-4f;

    public static BoxEdgesResult3D Run(in BoxPairLanes3D p)
    {
        var best = Vector256.Create(float.NegativeInfinity);
        var index = Vector256.Create(-1f);
        var bnx = Vector256<float>.Zero; var bny = Vector256<float>.Zero; var bnz = Vector256<float>.Zero;
        var bka = Vector256<float>.Zero; var bkb = Vector256<float>.Zero;
        var zero = Vector256<float>.Zero;
        var one = Vector256.Create(1f); var two = Vector256.Create(2f);
        var tiny = Vector256.Create(1e-10f);
        var cone = Vector256.Create(ConeMargin);
        var cutScale = Vector256.Create(ConeMargin * ConeMargin * 0.998f);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                Vector256<float> cx, cy, cz;
                if (j == 0) { cx = p.R11; cy = p.R21; cz = p.R31; } else if (j == 1) { cx = p.R12; cy = p.R22; cz = p.R32; } else { cx = p.R13; cy = p.R23; cz = p.R33; }
                // e_i x c
                Vector256<float> lx, ly, lz;
                if (i == 0) { lx = zero; ly = -cz; lz = cy; } else if (i == 1) { lx = cz; ly = zero; lz = -cx; } else { lx = -cy; ly = cx; lz = zero; }
                var l2 = lx * lx + ly * ly + lz * lz;
                var cut = cutScale * l2;
                Vector256<float> failCone = i == 0 ? Vector256.LessThan(ly * ly, cut) | Vector256.LessThan(lz * lz, cut)
                    : i == 1 ? Vector256.LessThan(lx * lx, cut) | Vector256.LessThan(lz * lz, cut)
                    : Vector256.LessThan(lx * lx, cut) | Vector256.LessThan(ly * ly, cut);
                var ok = Vector256.GreaterThan(l2, tiny) & ~failCone;
                // Lanes that are not ok compute garbage (possibly NaN) that the mask discards.
                var s = Vector256.Sqrt(l2);
                var nx = lx / s; var ny = ly / s; var nz = lz / s;
                var d = nx * p.Tx + ny * p.Ty + nz * p.Tz;
                var neg = Vector256.LessThan(d, zero);
                nx = Vector256.ConditionalSelect(neg, -nx, nx);
                ny = Vector256.ConditionalSelect(neg, -ny, ny);
                nz = Vector256.ConditionalSelect(neg, -nz, nz);
                d = Vector256.ConditionalSelect(neg, -d, d);
                var m0 = p.R11 * nx + p.R21 * ny + p.R31 * nz;
                var m1 = p.R12 * nx + p.R22 * ny + p.R32 * nz;
                var m2 = p.R13 * nx + p.R23 * ny + p.R33 * nz;
                var ax = Vector256.Abs(nx); var ay = Vector256.Abs(ny); var az = Vector256.Abs(nz);
                var bx = Vector256.Abs(m0); var by = Vector256.Abs(m1); var bz = Vector256.Abs(m2);
                var insideA = i == 0 ? Vector256.GreaterThan(ay, cone) & Vector256.GreaterThan(az, cone)
                    : i == 1 ? Vector256.GreaterThan(ax, cone) & Vector256.GreaterThan(az, cone)
                    : Vector256.GreaterThan(ax, cone) & Vector256.GreaterThan(ay, cone);
                var insideB = j == 0 ? Vector256.GreaterThan(by, cone) & Vector256.GreaterThan(bz, cone)
                    : j == 1 ? Vector256.GreaterThan(bx, cone) & Vector256.GreaterThan(bz, cone)
                    : Vector256.GreaterThan(bx, cone) & Vector256.GreaterThan(by, cone);
                var reachA = ax * p.HAx + ay * p.HAy + az * p.HAz;
                var reachB = bx * p.HBx + by * p.HBy + bz * p.HBz;
                var sep = d - reachA - reachB;
                var take = ok & insideA & insideB & Vector256.GreaterThan(sep, best);
                best = Vector256.ConditionalSelect(take, sep, best);
                index = Vector256.ConditionalSelect(take, Vector256.Create((float)(3 * i + j)), index);
                bnx = Vector256.ConditionalSelect(take, nx, bnx);
                bny = Vector256.ConditionalSelect(take, ny, bny);
                bnz = Vector256.ConditionalSelect(take, nz, bnz);
                // Corner signs of the supporting edges on the two axes each edge does not run along.
                var ka = i == 0 ? Bit(Vector256.GreaterThan(ny, zero), one, zero) + Bit(Vector256.GreaterThan(nz, zero), two, zero)
                    : i == 1 ? Bit(Vector256.GreaterThan(nx, zero), one, zero) + Bit(Vector256.GreaterThan(nz, zero), two, zero)
                    : Bit(Vector256.GreaterThan(nx, zero), one, zero) + Bit(Vector256.GreaterThan(ny, zero), two, zero);
                var kb = j == 0 ? Bit(Vector256.LessThan(m1, zero), one, zero) + Bit(Vector256.LessThan(m2, zero), two, zero)
                    : j == 1 ? Bit(Vector256.LessThan(m0, zero), one, zero) + Bit(Vector256.LessThan(m2, zero), two, zero)
                    : Bit(Vector256.LessThan(m0, zero), one, zero) + Bit(Vector256.LessThan(m1, zero), two, zero);
                bka = Vector256.ConditionalSelect(take, ka, bka);
                bkb = Vector256.ConditionalSelect(take, kb, bkb);
            }
        }

        return new BoxEdgesResult3D { Best = best, Index = index, Nx = bnx, Ny = bny, Nz = bnz, Ka = bka, Kb = bkb };
    }

    private static Vector256<float> Bit(Vector256<float> mask, Vector256<float> yes, Vector256<float> no) => Vector256.ConditionalSelect(mask, yes, no);
}
