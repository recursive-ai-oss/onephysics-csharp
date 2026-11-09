using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Rods;

namespace RecursiveAI.OnePhysics.Water;

/// <summary>
/// Soft bodies, cloth and rods in shallow water. Each particle is treated as a small blob of the volume it stands for: it gets buoyancy from the part of that volume under the surface (a smooth sphere-cap
/// fraction over the particle's size) and the same quadratic and viscous drag against the water's velocity as a rigid body, as velocity changes before the soft and rod steps; and it displaces water in the cell
/// under it, by the grid's displacement factor, like a rigid body's displacement. The volume put in is remembered per cell (it is state) and taken out when the next step puts in the new amounts.
/// </summary>
internal sealed partial class WaterStore
{
    private long[] _depositKeys = new long[64];
    private float[] _depositVolumes = new float[64];
    private int _depositCount;

    /// <summary>Records the water a particle displaces at a world point of grid <paramref name="g"/> for this step (summed per cell by <see cref="SettleDeposits"/>).</summary>
    internal void Deposit(WaterGrid g, double x, double z, float volume)
    {
        if (!(volume > 0f)) { return; }
        int i = (int)Math.Floor((x - g.OriginX) / g.Dx), k = g.Planar ? 0 : (int)Math.Floor((z - g.OriginZ) / g.Dx);
        if (i < 0) { i = 0; } else if (i >= g.Nx) { i = g.Nx - 1; }
        if (k < 0) { k = 0; } else if (k >= g.Nz) { k = g.Nz - 1; }
        if (_depositCount == _depositKeys.Length) { Array.Resize(ref _depositKeys, _depositCount * 2); Array.Resize(ref _depositVolumes, _depositCount * 2); }
        _depositKeys[_depositCount] = ((long)(k * g.Nx + i) << 32) | (uint)_depositCount;
        _depositVolumes[_depositCount++] = volume;
    }

    /// <summary>
    /// Buoyancy and drag of the water on one particle, as a change of its velocity; returns the volume of the particle under the surface. <paramref name="surface"/> is the water level at the particle,
    /// <paramref name="radius"/> the half-height over which the particle goes from dry to wet, <paramref name="area"/> its area facing the flow. The drag is capped at stopping the particle relative to the water.
    /// </summary>
    internal static float ParticleWater(WaterGrid g, double y, double surface, Vector3F flow, float volume, float radius, float area, float invMass, float gravityDown, float dt, bool planar, ref Vector3F velocity)
    {
        double t = (surface - (y - radius)) / (2.0 * radius);
        if (!(t > 0.0)) { return 0f; }
        if (t > 1.0) { t = 1.0; }
        float f = (float)(t * t * (3.0 - 2.0 * t));
        float submerged = f * volume;
        if (!(invMass > 0f)) { return submerged; }
        float vy = velocity.Y + g.Density * gravityDown * submerged * invMass * dt;
        float vx = velocity.X, vz = velocity.Z;
        float rx = vx - flow.X, ry = vy - flow.Y, rz = planar ? 0f : vz - flow.Z;
        float speed = float.Sqrt(rx * rx + ry * ry + rz * rz);
        if (speed > 0f)
        {
            float mag = 0.5f * g.Density * g.LinearDrag * area * f * speed * speed + g.ViscousDamping * g.Density * submerged * speed;
            float dv = mag * invMass * dt;
            if (dv > speed) { dv = speed; }
            float k = dv / speed;
            vx -= rx * k; vy -= ry * k; if (!planar) { vz -= rz * k; }
        }

        velocity = new Vector3F(vx, vy, vz);
        return submerged;
    }

    private void StepDeformables(int gi, WaterGrid g, float dt)
    {
        _depositCount = 0;
        var rods = _host.WaterRods;
        bool hasRods = rods != null && rods.Count != 0;
        if (g.HoldCount == 0 && !hasRods && !_host.HasSoftBodies) { return; }
        _host.ApplyWaterToSoftBodies(this, g, dt);
        if (hasRods) { ApplyWaterToRods(rods!, g, dt); }
        SettleDeposits(gi, g);
    }

    private void ApplyWaterToRods(RodStore rods, WaterGrid g, float dt)
    {
        double w = g.Width, d = g.Depth;
        float gravityDown = _host.GravityDown;
        bool planar = g.Planar;
        for (int slot = 0; slot < rods.Data.Length; slot++)
        {
            var r = rods.Data[slot];
            if (r == null) { continue; }
            float rad = r.Material.Radius;
            int n = r.PositionData.Length;
            for (int i = 0; i < n; i++)
            {
                var p = r.PositionData[i];
                double lx = p.X - g.OriginX, lz = planar ? 0.5 : p.Z - g.OriginZ;
                if (lx < 0 || lx > w || lz < 0 || lz > d) { continue; }
                // Half a segment on each side of the particle.
                float len = 0f; int segs = 0;
                if (i > 0) { len += r.Length[i - 1]; segs++; }
                if (i < n - 1) { len += r.Length[i]; segs++; }
                float seg = segs > 0 ? len / segs : 2f * rad;
                float volume = (planar ? 2f * rad : (float)Math.PI * rad * rad) * 0.5f * len;
                float size = Math.Max(rad, 0.5f * seg);
                if (!Reference(g, p.X - size, p.Z - size, p.X + size, p.Z + size, out double surface, out _, out _, out float u, out float v)) { continue; }
                var vel = r.VelocityData[i];
                float inv = r.Pinned[i] ? 0f : r.InverseMass[i];
                float area = planar ? seg : 2f * rad * seg;
                float sub = ParticleWater(g, p.Y, surface, new Vector3F(u, 0f, planar ? 0f : v), volume, size, area, inv, gravityDown, dt, planar, ref vel);
                if (inv > 0f) { r.VelocityData[i] = vel; }
                if (sub > 0f) { Deposit(g, p.X, p.Z, sub * g.Displacement); }
            }
        }
    }

    /// <summary>Sums this step's deposits per cell, queues the change from what was held (removing the old, adding the new) and keeps the new amounts.</summary>
    private void SettleDeposits(int gi, WaterGrid g)
    {
        int n = _depositCount;
        if (n > 1) { Array.Sort(_depositKeys, _depositVolumes, 0, n); }
        int[] newCells = n == 0 ? Array.Empty<int>() : new int[n];
        float[] newVolumes = n == 0 ? Array.Empty<float>() : new float[n];
        int m = 0;
        for (int a = 0; a < n; a++)
        {
            int cell = (int)(_depositKeys[a] >> 32);
            if (m > 0 && newCells[m - 1] == cell) { newVolumes[m - 1] += _depositVolumes[a]; }
            else { newCells[m] = cell; newVolumes[m++] = _depositVolumes[a]; }
        }

        int oi = 0, ni = 0;
        while (oi < g.HoldCount || ni < m)
        {
            int oc = oi < g.HoldCount ? g.HoldCells[oi] : int.MaxValue, nc = ni < m ? newCells[ni] : int.MaxValue;
            int cell = Math.Min(oc, nc);
            float delta = (nc == cell ? newVolumes[ni] : 0f) - (oc == cell ? g.HoldVolume[oi] : 0f);
            if (delta != 0f) { QueueCell(gi, cell, delta); }
            if (oc == cell) { oi++; }
            if (nc == cell) { ni++; }
        }

        g.HoldCells = newCells; g.HoldVolume = newVolumes; g.HoldCount = m;
    }

    private void QueueCell(int grid, int cell, float volume)
    {
        if (_pendingCount == _pending.Length) { Array.Resize(ref _pending, _pending.Length * 2); }
        _pending[_pendingCount++] = new Pending { Grid = grid, Volume = volume, Cell = cell };
    }
}
