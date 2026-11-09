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
                bool awake = bodies.IsAwake(b);
                if (!_host.BodyBounds(b, out double minX, out double minY, out double minZ, out double maxX, out double maxY, out double maxZ)) { continue; }
                double lx0 = minX - g.OriginX, lx1 = maxX - g.OriginX, lz0 = g.Planar ? 0.0 : minZ - g.OriginZ, lz1 = g.Planar ? 1.0 : maxZ - g.OriginZ;
                bool over = lx1 >= 0 && lx0 <= w && lz1 >= 0 && lz0 <= d;
                if (!awake)
                {
                    if (over) { WakeCheck(g, b, minX, minY, minZ, maxX, maxY, maxZ); }
                    if (!bodies.IsAwake(b)) { continue; }
                }

                float vg = 0f;
                if (over) { vg = Displace(g, b, minX, minY, minZ, maxX, maxY, maxZ, dt); }
                bool inWater = vg > 0f, wasIn = r >= 0;
                if (!inWater && !wasIn) { continue; }
                float oldSubmerged = wasIn ? g.Records[r].Submerged : 0f;
                if (wasIn)
                {
                    ref readonly var old = ref g.Records[r];
                    if (old.Volume != 0f) { Queue(gi, old.X0, old.X1, old.Z0, old.Z1, -old.Volume); }
                }

                if (inWater)
                {
                    float v = vg * g.Displacement;
                    var rec = new WaterBodyRecord { Slot = b, Generation = bodies.Handles.GenerationOf(b), Volume = v, Submerged = vg, X0 = (float)lx0, X1 = (float)lx1, Z0 = (float)lz0, Z1 = (float)lz1 };
                    if (wasIn) { g.Records[r] = rec; } else { InsertRecord(g, in rec); }
                    if (v != 0f) { Queue(gi, rec.X0, rec.X1, rec.Z0, rec.Z1, v); }
                }
                else { RemoveRecord(g, r); }

                if (inWater != wasIn)
                {
                    Events.Add(new WaterEventRecord { Entered = inWater, Body = b, BodyGeneration = bodies.Handles.GenerationOf(b), Water = gi, WaterGeneration = Handles.GenerationOf(gi), Volume = inWater ? vg : oldSubmerged });
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

    /// <summary>
    /// The surface a body floats on: the water level around it, read a cell and a half outside its box on each side, as a plane (height at the box's centre, slope along x and z) with the water's velocity there.
    /// The water under the body is left out on purpose: it holds the volume the body displaced and so reads higher than the level the body floats on, and feeding that back into the buoyancy
    /// makes the pair an undamped oscillator (the body pushes water up, the raised water pushes the body up). Read around the body, the level answers to the body only as waves that have travelled out and back.
    /// Returns false when there is no water around the body.
    /// </summary>
    private static bool Reference(WaterGrid g, double minX, double minZ, double maxX, double maxZ, out double surface, out float sx, out float sz, out float u, out float v)
    {
        double d = 0.5 * g.Dx;
        double xc = 0.5 * (minX + maxX), zc = g.Planar ? 0.0 : 0.5 * (minZ + maxZ);
        double sum = 0; float us = 0f, vs = 0f; int n = 0;
        bool hasW = false, hasE = false, hasS = false, hasN = false;
        double sw = 0, se = 0, ss = 0, sn = 0;
        if (Ring(g, minX - d, zc, ref sum, ref us, ref vs, ref n, out sw)) { hasW = true; }
        if (Ring(g, maxX + d, zc, ref sum, ref us, ref vs, ref n, out se)) { hasE = true; }
        if (!g.Planar)
        {
            if (Ring(g, xc, minZ - d, ref sum, ref us, ref vs, ref n, out ss)) { hasS = true; }
            if (Ring(g, xc, maxZ + d, ref sum, ref us, ref vs, ref n, out sn)) { hasN = true; }
        }

        surface = 0; sx = sz = 0f; u = v = 0f;
        if (n == 0) { return false; }
        surface = sum / n; u = us / n; v = vs / n;
        if (hasW && hasE) { sx = (float)((se - sw) / (maxX - minX + 2 * d)); }
        if (hasS && hasN) { sz = (float)((sn - ss) / (maxZ - minZ + 2 * d)); }
        if (sx > 1f) { sx = 1f; } else if (sx < -1f) { sx = -1f; }
        if (sz > 1f) { sz = 1f; } else if (sz < -1f) { sz = -1f; }
        return true;
    }

    private static bool Ring(WaterGrid g, double x, double z, ref double sum, ref float us, ref float vs, ref int n, out double surface)
    {
        SampleAt(g, x, z, out surface, out float depth, out float u, out float v);
        if (!(depth > WaterGrid.DryDepth)) { return false; }
        sum += surface; us += u; vs += v; n++;
        return true;
    }

    private void WakeCheck(WaterGrid g, int b, double minX, double minY, double minZ, double maxX, double maxY, double maxZ)
    {
        if (!Reference(g, minX, minZ, maxX, maxZ, out double surface, out float sx, out float sz, out float u, out float v) || !(minY < surface)) { return; }
        if (Math.Abs(u) > WakeSpeed || Math.Abs(v) > WakeSpeed || Math.Abs(sx) > WakeSlope || Math.Abs(sz) > WakeSlope) { _host.WakeBody(b); }
    }

    private float Displace(WaterGrid g, int b, double minX, double minY, double minZ, double maxX, double maxY, double maxZ, float dt)
    {
        if (!Reference(g, minX, minZ, maxX, maxZ, out double surface, out float sx, out float sz, out float u, out float v)) { return 0f; }
        float reach = _host.ReachOf(b);
        // A body wholly above the tilted surface is clear of the water.
        if (minY > surface + (Math.Abs(sx) + Math.Abs(sz)) * reach + 1e-3) { return 0f; }
        double cx = 0.5 * (minX + maxX), cz = g.Planar ? 0.0 : 0.5 * (minZ + maxZ);
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
