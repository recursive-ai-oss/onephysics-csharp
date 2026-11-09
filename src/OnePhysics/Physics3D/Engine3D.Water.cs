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
