using System;
using RecursiveAI.OnePhysics.Fluids;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D
{
    /// <summary>Soft-body particles (by body slot, then particle) and rod segments (by rod slot, then segment) near the box.</summary>
    void IFluidHost.GatherDeformables(double minX, double minY, double minZ, double maxX, double maxY, double maxZ, FluidDeformables into)
    {
        for (int i = 0; i < SoftHandles.HighWater; i++)
        {
            if (!SoftHandles.IsLive(i)) { continue; }
            var d = SoftData[i]!;
            double pad = d.Radius;
            for (int p = 0; p < d.N; p++)
            {
                double x = d.Pos[2 * p], y = d.Pos[2 * p + 1];
                if (x + pad < minX || x - pad > maxX || y + pad < minY || y - pad > maxY) { continue; }
                float vx = d.Vel[2 * p], vy = d.Vel[2 * p + 1], im = d.InvMass[p];
                into.Add(x, y, 0.0, x, y, 0.0, d.Radius, im, im, vx, vy, 0f, vx, vy, 0f, i, p, p);
            }
        }

        if (_rods != null && _rods.Count != 0) { FluidRodCoupling.Gather(_rods, minX, minY, minZ, maxX, maxY, maxZ, into); }
    }

    /// <summary>Gives the soft-body and rod particles the velocity change the fluid's contacts made; a sleeping soft body wakes when the largest change exceeds the world's wake speed and otherwise keeps its sleep.</summary>
    void IFluidHost.ApplyDeformables(FluidDeformables f)
    {
        float wake = ParticleFluids.Settings.WakeBodySpeed;
        for (int e = 0; e < f.Count;)
        {
            if ((f.Owner[e] >> 24) != 0) { e++; continue; }
            int slot = f.Owner[e] & 0xFFFFFF;
            int end = e;
            float maxDv = 0f;
            while (end < f.Count && (f.Owner[end] >> 24) == 0 && (f.Owner[end] & 0xFFFFFF) == slot)
            {
                float sc = f.InvA[end] > 0f ? f.DvScaleA(end) : 0f; float dx = f.JAx[end] * sc, dy = f.JAy[end] * sc;
                float m = float.Sqrt(dx * dx + dy * dy);
                if (m > maxDv) { maxDv = m; }
                end++;
            }

            var d = SoftData[slot]!;
            bool apply = true;
            if (d.Asleep)
            {
                if (maxDv > wake) { d.Asleep = false; d.SleepTime = 0f; } else { apply = false; }
            }

            if (apply)
            {
                for (int k = e; k < end; k++)
                {
                    int p = f.PA[k];
                    if (f.InvA[k] <= 0f) { continue; }
                    float sc = f.DvScaleA(k);
                    d.Vel[2 * p] += f.JAx[k] * sc; d.Vel[2 * p + 1] += f.JAy[k] * sc;
                }
            }

            e = end;
        }

        if (_rods != null && _rods.Count != 0) { FluidRodCoupling.Apply(_rods, f); }
    }
}
