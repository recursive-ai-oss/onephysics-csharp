using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Fields;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Water;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D : IWaterHost
{
    private WaterStore? _water;
    private FieldDefinition? _waterField;
    private Func<Shape, bool>? _staticOnly;

    /// <summary>Shallow water; created on first use and registered with the world.</summary>
    internal WaterStore Water => _water ??= new WaterStore(this);

    WorldCore IWaterHost.Core => Core;
    bool IWaterHost.Planar => true;
    int IWaterHost.EngineSectionId => Id;
    WaterStore IWaterHost.Water => Water;
    float IWaterHost.GravityDown => -Opt.Gravity.Y;
    float IWaterHost.ReachOf(int body) => MaxExtent[body];

    bool IWaterHost.BodyBounds(int body, out double minX, out double minY, out double minZ, out double maxX, out double maxY, out double maxZ)
    {
        minX = minY = double.MaxValue; maxX = maxY = -double.MaxValue; minZ = maxZ = 0.0;
        bool any = false;
        for (int s = ShapeHead[body]; s >= 0; s = NextShape[s])
        {
            if (IsSensor(s)) { continue; }
            var box = TightBox(body, in Geom[s]);
            if (box.MinX < minX) { minX = box.MinX; } if (box.MinY < minY) { minY = box.MinY; }
            if (box.MaxX > maxX) { maxX = box.MaxX; } if (box.MaxY > maxY) { maxY = box.MaxY; }
            any = true;
        }

        return any;
    }

    float IWaterHost.ApplyWater(int body, Vector3F normal, double offset, Vector3F flow, WaterGrid water, float dt)
    {
        var d = _waterField ??= new FieldDefinition { Kind = FieldKind.Buoyancy };
        d.Axis = new Vector3F(normal.X, normal.Y, 0f); d.SurfaceHeight = offset; d.Density = water.Density; d.Flow = new Vector3F(flow.X, flow.Y, 0f);
        d.Drag = water.ViscousDamping; d.QuadraticDrag = water.LinearDrag; d.AngularDrag = water.AngularDrag;
        return ApplyFieldBuoyancy(body, d, dt);
    }

    float IWaterHost.SubmergedVolume(int body, Vector3F normal, double offset, Vector3F flow, WaterGrid water, float dt)
    {
        float fx = Force[2 * body], fy = Force[2 * body + 1], t0 = Torque[body];
        float v = ((IWaterHost)this).ApplyWater(body, normal, offset, flow, water, dt);
        Force[2 * body] = fx; Force[2 * body + 1] = fy; Torque[body] = t0;
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
                double x = d.Pos[2 * p], z = 0.0;
                if (x < minX) { minX = x; } if (x > maxX) { maxX = x; } if (z < minZ) { minZ = z; } if (z > maxZ) { maxZ = z; }
            }

            double lx0 = minX - g.OriginX, lx1 = maxX - g.OriginX, lz0 = 0.5, lz1 = 0.5;
            if (lx1 < 0 || lx0 > w || lz1 < 0 || lz0 > dep) { continue; }
            if (!WaterStore.Reference(g, minX, minZ, maxX, maxZ, out double surface, out float sx, out float sz, out float u, out float v)) { continue; }
            double xc = 0.5 * (minX + maxX), zc = 0.5 * (minZ + maxZ);
            var flow = new Vector3F(u, 0f, 0f);
            if (d.Asleep)
            {
                // Water that moves, or a surface that has tilted under it, wakes a sleeping body; a still one leaves it asleep.
                if (Math.Abs(u) > 0.05f || Math.Abs(v) > 0.05f || Math.Abs(sx) > 0.01f || Math.Abs(sz) > 0.01f) { d.Asleep = false; d.SleepTime = 0f; } else { continue; }
            }

            float bodyVolume = SoftBodyArea(d);
            for (int p = 0; p < n; p++)
            {
                float im = d.InvMass0[p];
                float mass = im > 0f ? 1f / im : d.TotalMass / n;
                float volume = bodyVolume > 0f ? bodyVolume * (mass / d.TotalMass) : (float)Math.PI * d.Radius * d.Radius;
                float size = Math.Max(d.Radius, float.Sqrt(volume));
                double x = d.Pos[2 * p], y = d.Pos[2 * p + 1], z = 0.0;
                double level = surface + sx * (x - xc) + sz * (z - zc);
                var vel = new Vector3F(d.Vel[2 * p], d.Vel[2 * p + 1], 0f);
                float area = 2f * size;
                float sub = WaterStore.ParticleWater(g, y, level, flow, volume, size, area, d.InvMass[p], gravityDown, dt, true, ref vel);
                if (d.InvMass[p] > 0f) { d.Vel[2 * p] = vel.X; d.Vel[2 * p + 1] = vel.Y; }
                if (sub > 0f) { store.Deposit(g, x, z, sub * g.Displacement); }
            }
        }
    }

    /// <summary>The area a 2D soft body encloses at rest (its triangles or its outline), 0 for strips.</summary>
    private static float SoftBodyArea(SoftBodyData d)
    {
        double sum = 0.0;
        for (int t = 0; t < d.TriRest2.Length; t++) { sum += d.TriRest2[t]; }
        float a = (float)(0.5 * sum);
        return a > 0f ? a : d.PressureRest;
    }

    bool IWaterHost.CastDownStatic(double x, double z, double fromY, double maxDistance, out double hitY)
    {
        hitY = 0;
        double lim = BoxLimits.MaxCoordinate;
        if (!(Math.Abs(x) <= lim && Math.Abs(fromY) <= lim) || !(maxDistance > 0)) { return false; }
        _staticOnly ??= sh => Core.Bodies.TypeOf(Core.Shapes.Owner[sh.Handle.Index]) == BodyType.Static;
        var filter = new QueryFilter2D { Predicate = _staticOnly };
        if (!World!.Raycast(new Vector2D(x, fromY), new Vector2F(0f, -1f), out var h, (float)Math.Min(maxDistance, 3.0e7), in filter)) { return false; }
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
