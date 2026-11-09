using System;
using RecursiveAI.OnePhysics.Rods;

namespace RecursiveAI.OnePhysics.Fluids;

/// <summary>Rods, ropes and hair as capsules for the fluid step: one per segment, in rod slot then segment order. Both engines share it (a planar rod has z = 0 throughout).</summary>
internal static class FluidRodCoupling
{
    public static void Gather(RodStore rods, double minX, double minY, double minZ, double maxX, double maxY, double maxZ, FluidDeformables into)
    {
        for (int slot = 0; slot < rods.Data.Length; slot++)
        {
            var r = rods.Data[slot];
            if (r == null) { continue; }
            double rad = r.Material.Radius;
            var p = r.PositionData; var v = r.VelocityData; var w = r.InverseMass;
            for (int i = 0; i + 1 < p.Length; i++)
            {
                var a = p[i]; var b = p[i + 1];
                if (Math.Min(a.X, b.X) - rad > maxX || Math.Max(a.X, b.X) + rad < minX || Math.Min(a.Y, b.Y) - rad > maxY || Math.Max(a.Y, b.Y) + rad < minY
                    || Math.Min(a.Z, b.Z) - rad > maxZ || Math.Max(a.Z, b.Z) + rad < minZ) { continue; }
                into.Add(a.X, a.Y, a.Z, b.X, b.Y, b.Z, (float)rad, r.Pinned[i] ? 0f : w[i], r.Pinned[i + 1] ? 0f : w[i + 1],
                    v[i].X, v[i].Y, v[i].Z, v[i + 1].X, v[i + 1].Y, v[i + 1].Z, (1 << 24) | slot, i, i + 1);
            }
        }
    }

    public static void Apply(RodStore rods, FluidDeformables d)
    {
        for (int e = 0; e < d.Count; e++)
        {
            if ((d.Owner[e] >> 24) != 1) { continue; }
            var r = rods.Data[d.Owner[e] & 0xFFFFFF];
            if (r == null) { continue; }
            int pa = d.PA[e], pb = d.PB[e];
            if (d.InvA[e] > 0f) { r.VelocityData[pa] += new RecursiveAI.OneMath.Vector3F(d.JAx[e] * d.InvA[e], d.JAy[e] * d.InvA[e], d.JAz[e] * d.InvA[e]); }
            if (d.InvB[e] > 0f) { r.VelocityData[pb] += new RecursiveAI.OneMath.Vector3F(d.JBx[e] * d.InvB[e], d.JBy[e] * d.InvB[e], d.JBz[e] * d.InvB[e]); }
        }
    }
}
