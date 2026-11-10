using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Water;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D : IWaterHost
{
    private WaterStore? _water;
    private FluidDef? _waterFluid;
    private Func<Shape, bool>? _staticOnly;

    /// <summary>Shallow water; created on first use and registered with the world.</summary>
    internal WaterStore Water => _water ??= new WaterStore(this);

    WorldCore IWaterHost.Core => Core;
    bool IWaterHost.Planar => false;
    int IWaterHost.EngineSectionId => Id;
    WaterStore IWaterHost.Water => Water;
    float IWaterHost.GravityDown => -Opt.Gravity.Y;
    float IWaterHost.ReachOf(int body) => MaxExtent[body];

    bool IWaterHost.BodyBounds(int body, out double minX, out double minY, out double minZ, out double maxX, out double maxY, out double maxZ)
    {
        minX = minY = minZ = double.MaxValue; maxX = maxY = maxZ = -double.MaxValue;
        bool any = false;
        for (int s = ShapeHead[body]; s >= 0; s = NextShape[s])
        {
            if (IsSensor(s)) { continue; }
            TightBoxCore(body, s, out double x0, out double y0, out double z0, out double x1, out double y1, out double z1);
            if (x0 < minX) { minX = x0; } if (y0 < minY) { minY = y0; } if (z0 < minZ) { minZ = z0; }
            if (x1 > maxX) { maxX = x1; } if (y1 > maxY) { maxY = y1; } if (z1 > maxZ) { maxZ = z1; }
            any = true;
        }

        return any;
    }

    float IWaterHost.ApplyWater(int body, Vector3F normal, double offset, Vector3F flow, WaterGrid water, float dt)
    {
        var f = _waterFluid ??= new FluidDef();
        f.SurfaceNormal = normal; f.SurfaceOffset = offset; f.Density = water.Density; f.Flow = flow;
        f.LinearDrag = water.LinearDrag; f.AngularDrag = water.AngularDrag; f.ViscousDamping = water.ViscousDamping;
        return ApplyFluidToBody(body, f, dt);
    }

    float IWaterHost.SubmergedVolume(int body, Vector3F normal, double offset, Vector3F flow, WaterGrid water, float dt)
    {
        var f0 = new Vector3F(Force[3 * body], Force[3 * body + 1], Force[3 * body + 2]);
        var t0 = new Vector3F(Torque[3 * body], Torque[3 * body + 1], Torque[3 * body + 2]);
        float v = ((IWaterHost)this).ApplyWater(body, normal, offset, flow, water, dt);
        Force[3 * body] = f0.X; Force[3 * body + 1] = f0.Y; Force[3 * body + 2] = f0.Z;
        Torque[3 * body] = t0.X; Torque[3 * body + 1] = t0.Y; Torque[3 * body + 2] = t0.Z;
        return v;
    }

    Rods.RodStore? IWaterHost.WaterRods => _rods;

    bool IWaterHost.HasSoftBodies => SoftHandles.Count > 0;

    long IWaterHost.DeformableParticleCount
    {
        get
        {
            long n = 0;
            for (int i = 0; i < SoftHandles.HighWater; i++) { if (SoftHandles.IsLive(i)) { n += SoftData[i]!.N; } }
            if (_rods != null) { foreach (var r in _rods.Data) { if (r != null) { n += r.PositionData.Length; } } }
            return n;
        }
    }

    void IWaterHost.ApplyWaterToSoftBodies(WaterStore store, WaterGrid g, float dt)
    {
        if (SoftHandles.Count == 0) { return; }
        float gravityDown = -Opt.Gravity.Y;
        double w = g.Width, dep = g.Depth;
        for (int slot = 0; slot < SoftHandles.HighWater; slot++)
        {
            if (!SoftHandles.IsLive(slot)) { continue; }
            var d = SoftData[slot]!;
            int n = d.N;
            if (n == 0) { continue; }
            double minX = double.MaxValue, minZ = double.MaxValue, maxX = -double.MaxValue, maxZ = -double.MaxValue;
            for (int p = 0; p < n; p++)
            {
                double x = d.Pos[3 * p], z = d.Pos[3 * p + 2];
                if (x < minX) { minX = x; } if (x > maxX) { maxX = x; } if (z < minZ) { minZ = z; } if (z > maxZ) { maxZ = z; }
            }

            double lx0 = minX - g.OriginX, lx1 = maxX - g.OriginX, lz0 = minZ - g.OriginZ, lz1 = maxZ - g.OriginZ;
            if (lx1 < 0 || lx0 > w || lz1 < 0 || lz0 > dep) { continue; }
            if (!WaterStore.Reference(g, minX, minZ, maxX, maxZ, out double surface, out float sx, out float sz, out float u, out float v)) { continue; }
            double xc = 0.5 * (minX + maxX), zc = 0.5 * (minZ + maxZ);
            var flow = new Vector3F(u, 0f, v);
            if (d.Asleep)
            {
                // Water that moves, or a surface that has tilted under it, wakes a sleeping body; a still one leaves it asleep.
                if (Math.Abs(u) > 0.05f || Math.Abs(v) > 0.05f || Math.Abs(sx) > 0.01f || Math.Abs(sz) > 0.01f) { d.Asleep = false; d.SleepTime = 0f; } else { continue; }
            }

            float bodyVolume = SoftBodyVolume(d);
            for (int p = 0; p < n; p++)
            {
                float im = d.InvMass0[p];
                float mass = im > 0f ? 1f / im : d.TotalMass / n;
                float volume = bodyVolume > 0f ? bodyVolume * (mass / d.TotalMass) : (float)(4.0 / 3.0 * Math.PI) * d.Radius * d.Radius * d.Radius;
                float size = Math.Max(d.Radius, Math32.Cbrt(volume));
                double x = d.Pos[3 * p], y = d.Pos[3 * p + 1], z = d.Pos[3 * p + 2];
                double level = surface + sx * (x - xc) + sz * (z - zc);
                var vel = new Vector3F(d.Vel[3 * p], d.Vel[3 * p + 1], d.Vel[3 * p + 2]);
                float area = (float)Math.PI * size * size;
                float sub = WaterStore.ParticleWater(g, y, level, flow, volume, size, area, d.InvMass[p], gravityDown, dt, false, ref vel);
                if (d.InvMass[p] > 0f) { d.Vel[3 * p] = vel.X; d.Vel[3 * p + 1] = vel.Y; d.Vel[3 * p + 2] = vel.Z; }
                if (sub > 0f) { store.Deposit(g, x, z, sub * g.Displacement); }
            }
        }
    }

    /// <summary>The volume a soft body encloses at rest (tetrahedra or the closed surface), 0 for cloth and open surfaces.</summary>
    private static float SoftBodyVolume(SoftBodyData d)
    {
        double sum = 0.0;
        for (int t = 0; t < d.TetRest.Length; t++) { sum += d.TetRest[t]; }
        float v = (float)(sum / 6.0);
        return v > 0f ? v : d.PressureRest;
    }

    bool IWaterHost.CastDownStatic(double x, double z, double fromY, double maxDistance, out double hitY)
    {
        hitY = 0;
        double lim = BoxLimits.MaxCoordinate;
        if (!(Math.Abs(x) <= lim && Math.Abs(fromY) <= lim && Math.Abs(z) <= lim) || !(maxDistance > 0)) { return false; }
        _staticOnly ??= sh => Core.Bodies.TypeOf(Core.Shapes.Owner[sh.Handle.Index]) == BodyType.Static;
        var filter = new QueryFilter3D { Predicate = _staticOnly };
        if (!World!.Raycast(new Vector3D(x, fromY, z), new Vector3F(0f, -1f, 0f), out var h, (float)Math.Min(maxDistance, 3.0e7), in filter)) { return false; }
        hitY = fromY - h.Distance;
        return true;
    }

    void IWaterHost.WakeBody(int body)
    {
        Core.WakeIsland(Core.Bodies.Island[body]);
        Core.Bodies.SleepTime[body] = 0f;
    }

    internal bool WaterConsistent() => _water == null || _water.Consistent();
}
