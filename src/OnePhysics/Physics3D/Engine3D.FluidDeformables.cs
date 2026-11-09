using System;
using RecursiveAI.OnePhysics.Fluids;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D
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
                double x = d.Pos[3 * p], y = d.Pos[3 * p + 1], z = d.Pos[3 * p + 2];
                if (x + pad < minX || x - pad > maxX || y + pad < minY || y - pad > maxY || z + pad < minZ || z - pad > maxZ) { continue; }
                float vx = d.Vel[3 * p], vy = d.Vel[3 * p + 1], vz = d.Vel[3 * p + 2], im = d.InvMass[p];
                into.Add(x, y, z, x, y, z, d.Radius, im, im, vx, vy, vz, vx, vy, vz, i, p, p);
            }
        }

        if (_rods != null && _rods.Count != 0) { FluidRodCoupling.Gather(_rods, minX, minY, minZ, maxX, maxY, maxZ, into); }
    }

    /// <summary>Gives the soft-body particles and rod particles the velocity change the fluid's contacts made. A sleeping soft body wakes when the largest change exceeds the world's wake speed and otherwise keeps its sleep (the push is dropped).</summary>
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
                float dx = f.JAx[end] * f.InvA[end], dy = f.JAy[end] * f.InvA[end], dz = f.JAz[end] * f.InvA[end];
                float m = float.Sqrt(dx * dx + dy * dy + dz * dz);
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
                    d.Vel[3 * p] += f.JAx[k] * f.InvA[k]; d.Vel[3 * p + 1] += f.JAy[k] * f.InvA[k]; d.Vel[3 * p + 2] += f.JAz[k] * f.InvA[k];
                }
            }

            e = end;
        }

        if (_rods != null && _rods.Count != 0) { FluidRodCoupling.Apply(_rods, f); }
    }
}
