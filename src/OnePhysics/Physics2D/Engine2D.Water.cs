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
