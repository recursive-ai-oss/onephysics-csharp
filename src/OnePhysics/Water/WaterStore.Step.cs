using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Water;

internal sealed partial class WaterStore
{
    // Water faster than this, or a surface steeper than this, wakes a sleeping body floating in it.
    private const float WakeSpeed = 0.05f, WakeSlope = 0.01f;

    private struct Pending
    {
        public int Grid;
        public float X0, X1, Z0, Z1, Volume;
    }

    private Pending[] _pending = new Pending[16];
    private int _pendingCount;

    /// <summary>Substeps the last step of each grid took (not saved).</summary>
    public int LastSubsteps;

    private void Queue(int grid, float x0, float x1, float z0, float z1, float volume)
    {
        if (_pendingCount == _pending.Length) { Array.Resize(ref _pending, _pending.Length * 2); }
        _pending[_pendingCount++] = new Pending { Grid = grid, X0 = x0, X1 = x1, Z0 = z0, Z1 = z1, Volume = volume };
    }

    /// <summary>
    /// The first half of a step, before the rigid step: each awake dynamic body gets the buoyancy and drag of the surface it floats in (the tangent plane of the water under it, moving with the water),
    /// the volume it displaces is recorded, and enter and leave events are collected. Grids go in slot order and bodies in slot order inside each. A sleeping body that the water is moving wakes.
    /// </summary>
    public void BeginStep(float dt)
    {
        Events.Clear();
        _pendingCount = 0;
        if (Handles.Count == 0) { return; }
        var core = _host.Core;
        var bodies = core.Bodies;
        for (int gi = 0; gi < Handles.HighWater; gi++)
        {
            if (!Handles.IsLive(gi)) { continue; }
            var g = Grids[gi]!;
            Prune(g);
            double w = g.Width, d = g.Depth;
            for (int b = 0; b < bodies.HighWater; b++)
            {
                if (!bodies.IsLive(b) || !bodies.IsDynamic(b)) { continue; }
                int r = FindRecord(g, b);
                float vOld = r >= 0 ? g.Records[r].Volume : 0f;
                bool awake = bodies.IsAwake(b);
                if (!_host.BodyBounds(b, out double minX, out double minY, out double minZ, out double maxX, out double maxY, out double maxZ)) { continue; }
                double lx0 = minX - g.OriginX, lx1 = maxX - g.OriginX, lz0 = g.Planar ? 0.0 : minZ - g.OriginZ, lz1 = g.Planar ? 1.0 : maxZ - g.OriginZ;
                bool over = lx1 >= 0 && lx0 <= w && lz1 >= 0 && lz0 <= d;
                if (!awake)
                {
                    if (over) { WakeCheck(g, b, minX, minY, minZ, maxX, maxY, maxZ); }
                    if (!bodies.IsAwake(b)) { continue; }
                }

                float v = 0f;
                if (over) { v = Displace(g, b, minX, minY, minZ, maxX, maxY, maxZ, dt); }
                if (v <= 0f && r < 0) { continue; }
                if (r >= 0)
                {
                    ref readonly var old = ref g.Records[r];
                    Queue(gi, old.X0, old.X1, old.Z0, old.Z1, -old.Volume);
                }

                if (v > 0f)
                {
                    var rec = new WaterBodyRecord { Slot = b, Generation = bodies.Handles.GenerationOf(b), Volume = v, X0 = (float)lx0, X1 = (float)lx1, Z0 = (float)lz0, Z1 = (float)lz1 };
                    if (r >= 0) { g.Records[r] = rec; } else { InsertRecord(g, in rec); }
                    Queue(gi, rec.X0, rec.X1, rec.Z0, rec.Z1, v);
                }
                else { RemoveRecord(g, r); }

                if (v > 0f != vOld > 0f)
                {
                    Events.Add(new WaterEventRecord { Entered = v > 0f, Body = b, BodyGeneration = bodies.Handles.GenerationOf(b), Water = gi, WaterGeneration = Handles.GenerationOf(gi), Volume = v > 0f ? v : vOld });
                }
            }
        }
    }

    /// <summary>Takes out the volume of bodies that are gone or no longer dynamic.</summary>
    private void Prune(WaterGrid g)
    {
        var bodies = _host.Core.Bodies;
        for (int r = g.RecordCount - 1; r >= 0; r--)
        {
            var rec = g.Records[r];
            if (bodies.Handles.IsAlive(rec.Slot, rec.Generation) && bodies.TypeOf(rec.Slot) == BodyType.Dynamic) { continue; }
            g.AddVolume(rec.X0, rec.X1, rec.Z0, rec.Z1, -rec.Volume);
            RemoveRecord(g, r);
        }
    }

    private void WakeCheck(WaterGrid g, int b, double minX, double minY, double minZ, double maxX, double maxY, double maxZ)
    {
        double cx = 0.5 * (minX + maxX), cz = g.Planar ? 0.0 : 0.5 * (minZ + maxZ);
        SampleAt(g, cx, cz, out double surface, out float depth, out float u, out float v);
        if (!(depth > WaterGrid.DryDepth) || !(minY < surface)) { return; }
        float reach = _host.ReachOf(b);
        SlopeAt(g, cx, cz, Math.Min(Math.Max(0.5f * reach, g.Dx), 4f * g.Dx), out float sx, out float sz);
        if (Math.Abs(u) > WakeSpeed || Math.Abs(v) > WakeSpeed || Math.Abs(sx) > WakeSlope || Math.Abs(sz) > WakeSlope) { _host.WakeBody(b); }
    }

    private float Displace(WaterGrid g, int b, double minX, double minY, double minZ, double maxX, double maxY, double maxZ, float dt)
    {
        double cx = 0.5 * (minX + maxX), cz = g.Planar ? 0.0 : 0.5 * (minZ + maxZ);
        SampleAt(g, cx, cz, out double surface, out float depth, out float u, out float v);
        if (!(depth > WaterGrid.DryDepth)) { return 0f; }
        float reach = _host.ReachOf(b);
        float span = Math.Min(Math.Max(0.5f * reach, g.Dx), 4f * g.Dx);
        SlopeAt(g, cx, cz, span, out float sx, out float sz);
        // A body wholly above the tilted surface is clear of the water.
        if (minY > surface + (Math.Abs(sx) + Math.Abs(sz)) * reach + 1e-3) { return 0f; }
        float inv = 1f / float.Sqrt(1f + sx * sx + sz * sz);
        var n = new Vector3F(-sx * inv, inv, -sz * inv);
        double offset = n.X * cx + n.Y * surface + n.Z * cz;
        return _host.ApplyWater(b, n, offset, new Vector3F(u, 0f, g.Planar ? 0f : v), g, dt);
    }

    /// <summary>
    /// The second half of a step, after the rigid step: the volumes bodies displaced are added to the water (a body that moved takes its old displacement out where it was and puts the new one where it is),
    /// then every grid advances by <paramref name="dt"/>.
    /// </summary>
    public void EndStep(float dt)
    {
        if (Handles.Count == 0) { return; }
        for (int p = 0; p < _pendingCount; p++)
        {
            ref readonly var q = ref _pending[p];
            Grids[q.Grid]!.AddVolume(q.X0, q.X1, q.Z0, q.Z1, q.Volume);
        }

        _pendingCount = 0;
        float g0 = _host.GravityDown;
        var executor = _host.Core.Executor;
        int substeps = 0;
        for (int gi = 0; gi < Handles.HighWater; gi++)
        {
            if (Handles.IsLive(gi)) { substeps = Math.Max(substeps, Grids[gi]!.Advance(dt, g0, executor)); }
        }

        LastSubsteps = substeps;
    }
}
