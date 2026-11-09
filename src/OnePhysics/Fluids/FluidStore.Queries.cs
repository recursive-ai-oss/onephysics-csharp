using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Fluids;

/// <summary>A particle a query found.</summary>
public readonly struct FluidParticleInfo
{
    /// <summary>Creates the value.</summary>
    public FluidParticleInfo(int id, int fluid, Vector3D position, Vector3F velocity)
    {
        Id = id; Fluid = fluid; Position = position; Velocity = velocity;
    }

    /// <summary>Id of the particle: unique in the world, assigned in creation order, never reused.</summary>
    public int Id { get; }

    /// <summary>Slot of the particle's fluid.</summary>
    public int Fluid { get; }

    /// <summary>Position in world coordinates (a planar world: z = 0).</summary>
    public Vector3D Position { get; }

    /// <summary>Velocity, m/s.</summary>
    public Vector3F Velocity { get; }
}

/// <summary>A ray against fluid particles, taken as spheres of half the spacing.</summary>
public readonly struct FluidRayHit
{
    /// <summary>Creates the value.</summary>
    public FluidRayHit(FluidParticleInfo particle, float distance, Vector3D point, Vector3F normal)
    {
        Particle = particle; Distance = distance; Point = point; Normal = normal;
    }

    /// <summary>The particle hit.</summary>
    public FluidParticleInfo Particle { get; }

    /// <summary>Distance along the ray to the sphere's surface, metres.</summary>
    public float Distance { get; }

    /// <summary>The point on the sphere.</summary>
    public Vector3D Point { get; }

    /// <summary>Outward normal of the sphere there.</summary>
    public Vector3F Normal { get; }
}

internal struct BodyFluidResult
{
    public int Body, BodyGen, Fluid;
    public float Fraction, WetArea;
    public float Fx, Fy, Fz;
}

internal sealed partial class FluidStore
{
    internal readonly List<BodyFluidResult> Results = new();
    private const float EnterFraction = 0.05f, ExitFraction = 0.02f;

    // ------------------------------------------------------------------ query grid (rebuilt on demand from the double positions)

    private bool _qValid;
    private int _qBuckets, _qBucketMask;
    private int[] _qStart = new int[16], _qList = new int[16], _qCell = new int[16];
    private double _qMinX, _qMinY, _qMinZ;
    private float _qCellSize;
    private FluidKernels _kq;
    private readonly int[] _wetCounts = new int[MaxFluids];

    internal void InvalidateQueryGrid() { _qValid = false; }

    /// <summary>Builds the query grid if the particles moved since it was built.</summary>
    internal void PrepareQueries() { EnsureQueryGrid(); }

    private static int QBucket(long ix, long iy, long iz, int mask)
    {
        unchecked
        {
            uint h = (uint)ix * 73856093u ^ (uint)iy * 19349663u ^ (uint)iz * 83492791u;
            h ^= h >> 15; h *= 2246822519u; h ^= h >> 13;
            return (int)(h & (uint)mask);
        }
    }

    private long QCoord(double v, double min) => (long)Math.Floor((v - min) / _qCellSize);

    private void EnsureQueryGrid()
    {
        if (_qValid) { return; }
        _qValid = true;
        if (Count == 0 || FluidHandles.Count == 0) { _qBuckets = 0; return; }
        _qCellSize = Spacing * KernelScale;
        _kq = new FluidKernels(Dim, _qCellSize);
        double mnx = double.MaxValue, mny = double.MaxValue, mnz = double.MaxValue;
        for (int i = 0; i < Count; i++)
        {
            if (PX[i] < mnx) { mnx = PX[i]; } if (PY[i] < mny) { mny = PY[i]; } if (PZ[i] < mnz) { mnz = PZ[i]; }
        }

        _qMinX = mnx; _qMinY = mny; _qMinZ = mnz;
        int b = 1024;
        while (b < 2 * Count && b < (1 << 22)) { b <<= 1; }
        _qBuckets = b; _qBucketMask = b - 1;
        if (_qStart.Length < b + 1) { _qStart = new int[b + 1]; }
        Array.Clear(_qStart, 0, b + 1);
        if (_qList.Length < Count) { _qList = new int[Math.Max(Count, _qList.Length * 2)]; _qCell = new int[_qList.Length]; }
        for (int i = 0; i < Count; i++)
        {
            int bk = QBucket(QCoord(PX[i], mnx), QCoord(PY[i], mny), QCoord(PZ[i], mnz), _qBucketMask);
            _qCell[i] = bk; _qStart[bk + 1]++;
        }

        for (int k = 0; k < b; k++) { _qStart[k + 1] += _qStart[k]; }
        if (_cursor.Length < b) { _cursor = new int[Math.Max(b, _cursor.Length * 2)]; }
        Array.Copy(_qStart, _cursor, b);
        for (int i = 0; i < Count; i++) { _qList[_cursor[_qCell[i]]++] = i; }
    }

    internal FluidParticleInfo Info(int p) => new(PId[p], PFluid[p], new Vector3D(PX[p], PY[p], PZ[p]), new Vector3F(PVx[p], PVy[p], PVz[p]));

    /// <summary>The particles whose centres lie within <paramref name="radius"/> of the point, in ascending storage order. <paramref name="total"/> counts all of them even when <paramref name="results"/> is shorter.</summary>
    internal int OverlapSphere(Vector3D centre, double radius, int fluid, Span<FluidParticleInfo> results, out int total)
    {
        total = 0;
        EnsureQueryGrid();
        if (_qBuckets == 0) { return 0; }
        double cx = centre.X, cy = centre.Y, cz = Dim == 3 ? centre.Z : 0.0;
        long x0 = QCoord(cx - radius, _qMinX), x1 = QCoord(cx + radius, _qMinX);
        long y0 = QCoord(cy - radius, _qMinY), y1 = QCoord(cy + radius, _qMinY);
        long z0 = Dim == 3 ? QCoord(cz - radius, _qMinZ) : 0, z1 = Dim == 3 ? QCoord(cz + radius, _qMinZ) : 0;
        if (x1 - x0 > 4096 || y1 - y0 > 4096 || z1 - z0 > 4096)
        {
            // A query box larger than the grid's useful range: scan every particle in storage order.
            return ScanAll(cx, cy, cz, radius, fluid, results, ref total);
        }

        // Candidates from several cells may share a bucket; collect and order by particle index so the result is the same however the buckets collide.
        _qHits.Clear();
        double r2 = radius * radius;
        for (long iz = z0; iz <= z1; iz++)
        {
            for (long iy = y0; iy <= y1; iy++)
            {
                for (long ix = x0; ix <= x1; ix++)
                {
                    int bk = QBucket(ix, iy, iz, _qBucketMask);
                    for (int a = _qStart[bk]; a < _qStart[bk + 1]; a++)
                    {
                        int p = _qList[a];
                        if (fluid >= 0 && PFluid[p] != fluid) { continue; }
                        double dx = PX[p] - cx, dy = PY[p] - cy, dz = Dim == 3 ? PZ[p] - cz : 0.0;
                        if (dx * dx + dy * dy + dz * dz <= r2) { _qHits.Add(p); }
                    }
                }
            }
        }

        _qHits.Sort();
        int last = -1, n = 0;
        foreach (int p in _qHits)
        {
            if (p == last) { continue; }
            last = p;
            total++;
            if (n < results.Length) { results[n++] = Info(p); }
        }

        return n;
    }

    private readonly List<int> _qHits = new();

    private int ScanAll(double cx, double cy, double cz, double radius, int fluid, Span<FluidParticleInfo> results, ref int total)
    {
        double r2 = radius * radius;
        int n = 0;
        for (int p = 0; p < Count; p++)
        {
            if (fluid >= 0 && PFluid[p] != fluid) { continue; }
            double dx = PX[p] - cx, dy = PY[p] - cy, dz = Dim == 3 ? PZ[p] - cz : 0.0;
            if (dx * dx + dy * dy + dz * dz > r2) { continue; }
            total++;
            if (n < results.Length) { results[n++] = Info(p); }
        }

        return n;
    }

    /// <summary>The particles inside the box, in storage order.</summary>
    internal int OverlapBox(Vector3D min, Vector3D max, int fluid, Span<FluidParticleInfo> results, out int total)
    {
        total = 0;
        int n = 0;
        for (int p = 0; p < Count; p++)
        {
            if (fluid >= 0 && PFluid[p] != fluid) { continue; }
            if (PX[p] < min.X || PX[p] > max.X || PY[p] < min.Y || PY[p] > max.Y) { continue; }
            if (Dim == 3 && (PZ[p] < min.Z || PZ[p] > max.Z)) { continue; }
            total++;
            if (n < results.Length) { results[n++] = Info(p); }
        }

        return n;
    }

    /// <summary>The nearest particle sphere along the ray. Particles are spheres of half the spacing. Brute force over the cells the ray crosses (a ray is walked cell by cell, Amanatides and Woo 1987).</summary>
    internal bool RayCast(Vector3D origin, Vector3F direction, float maxDistance, int fluid, out FluidRayHit hit)
    {
        hit = default;
        EnsureQueryGrid();
        if (_qBuckets == 0) { return false; }
        double dx = direction.X, dy = direction.Y, dz = Dim == 3 ? direction.Z : 0.0;
        double len = Math.Sqrt(dx * dx + dy * dy + dz * dz);
        if (!(len > 0.0)) { return false; }
        dx /= len; dy /= len; dz /= len;
        double ox = origin.X, oy = origin.Y, oz = Dim == 3 ? origin.Z : 0.0;
        double rad = 0.5 * Spacing;
        double best = double.MaxValue; int bestP = -1;
        // Walk the cells along the ray, a margin of one cell each side covers the sphere radius (radius < cell).
        double step = _qCellSize * 0.5;
        long steps = (long)Math.Min(Math.Ceiling(maxDistance / step) + 1, 2_000_000);
        long lastX = long.MinValue, lastY = long.MinValue, lastZ = long.MinValue;
        for (long s = 0; s <= steps; s++)
        {
            double t = Math.Min(s * step, (double)maxDistance);
            double px = ox + dx * t, py = oy + dy * t, pz = oz + dz * t;
            long ix = QCoord(px, _qMinX), iy = QCoord(py, _qMinY), iz = Dim == 3 ? QCoord(pz, _qMinZ) : 0;
            if (ix == lastX && iy == lastY && iz == lastZ) { if (t >= maxDistance) { break; } continue; }
            lastX = ix; lastY = iy; lastZ = iz;
            for (long cz = Dim == 3 ? iz - 1 : 0; cz <= (Dim == 3 ? iz + 1 : 0); cz++)
            {
                for (long cy = iy - 1; cy <= iy + 1; cy++)
                {
                    for (long cx = ix - 1; cx <= ix + 1; cx++)
                    {
                        int bk = QBucket(cx, cy, cz, _qBucketMask);
                        for (int a = _qStart[bk]; a < _qStart[bk + 1]; a++)
                        {
                            int p = _qList[a];
                            if (fluid >= 0 && PFluid[p] != fluid) { continue; }
                            double mx = ox - PX[p], my = oy - PY[p], mz = Dim == 3 ? oz - PZ[p] : 0.0;
                            double bb = mx * dx + my * dy + mz * dz;
                            double cc = mx * mx + my * my + mz * mz - rad * rad;
                            if (cc > 0.0 && bb > 0.0) { continue; }
                            double disc = bb * bb - cc;
                            if (disc < 0.0) { continue; }
                            double tt = -bb - Math.Sqrt(disc);
                            if (tt < 0.0) { tt = 0.0; }
                            if (tt > maxDistance) { continue; }
                            if (tt < best || (tt == best && p < bestP)) { best = tt; bestP = p; }
                        }
                    }
                }
            }

            // Nothing nearer than the end of the cells read so far can still be found once the best is behind the walked distance by more than the margin.
            if (bestP >= 0 && best <= t - _qCellSize * 2.0) { break; }
            if (t >= maxDistance) { break; }
        }

        if (bestP < 0) { return false; }
        double hx = ox + dx * best, hy = oy + dy * best, hz = oz + dz * best;
        double nx = hx - PX[bestP], ny = hy - PY[bestP], nz = Dim == 3 ? hz - PZ[bestP] : 0.0;
        double nl = Math.Sqrt(nx * nx + ny * ny + nz * nz);
        var normal = nl > 0.0 ? new Vector3F((float)(nx / nl), (float)(ny / nl), (float)(nz / nl)) : new Vector3F(-(float)dx, -(float)dy, -(float)dz);
        hit = new FluidRayHit(Info(bestP), (float)best, new Vector3D(hx, hy, Dim == 3 ? hz : 0.0), normal);
        return true;
    }

    /// <summary>Whether any particle of the fluid lies within the distance of the point.</summary>
    private bool AnyWithin(double x, double y, double z, double r, int fluid)
    {
        if (_qBuckets == 0) { return false; }
        long x0 = QCoord(x - r, _qMinX), x1 = QCoord(x + r, _qMinX), y0 = QCoord(y - r, _qMinY), y1 = QCoord(y + r, _qMinY);
        long z0 = Dim == 3 ? QCoord(z - r, _qMinZ) : 0, z1 = Dim == 3 ? QCoord(z + r, _qMinZ) : 0;
        double r2 = r * r;
        if (x1 - x0 > 64 || y1 - y0 > 64 || z1 - z0 > 64)
        {
            for (int p = 0; p < Count; p++)
            {
                if (PFluid[p] != fluid) { continue; }
                double dx = PX[p] - x, dy = PY[p] - y, dz = Dim == 3 ? PZ[p] - z : 0.0;
                if (dx * dx + dy * dy + dz * dz <= r2) { return true; }
            }

            return false;
        }

        for (long iz = z0; iz <= z1; iz++)
        {
            for (long iy = y0; iy <= y1; iy++)
            {
                for (long ix = x0; ix <= x1; ix++)
                {
                    int bk = QBucket(ix, iy, iz, _qBucketMask);
                    for (int a = _qStart[bk]; a < _qStart[bk + 1]; a++)
                    {
                        int p = _qList[a];
                        if (PFluid[p] != fluid) { continue; }
                        double dx = PX[p] - x, dy = PY[p] - y, dz = Dim == 3 ? PZ[p] - z : 0.0;
                        if (dx * dx + dy * dy + dz * dz <= r2) { return true; }
                    }
                }
            }
        }

        return false;
    }

    /// <summary>The SPH density (kg/m³; kg/m² planar) of the fluid at the world point, from particles of that fluid (all when negative).</summary>
    internal float DensityAt(double x, double y, double z, int fluid)
    {
        if (_qBuckets == 0) { return 0f; }
        var k = _kq;
        double sum = 0.0;
        long ix = QCoord(x, _qMinX), iy = QCoord(y, _qMinY), iz = Dim == 3 ? QCoord(z, _qMinZ) : 0;
        Span<int> seen = stackalloc int[27];
        int ns = 0;
        for (long cz = Dim == 3 ? iz - 1 : 0; cz <= (Dim == 3 ? iz + 1 : 0); cz++)
        {
            for (long cy = iy - 1; cy <= iy + 1; cy++)
            {
                for (long cx = ix - 1; cx <= ix + 1; cx++)
                {
                    int bk = QBucket(cx, cy, cz, _qBucketMask);
                    bool dup = false;
                    for (int q = 0; q < ns; q++) { if (seen[q] == bk) { dup = true; break; } }
                    if (dup) { continue; }
                    seen[ns++] = bk;
                    for (int a = _qStart[bk]; a < _qStart[bk + 1]; a++)
                    {
                        int p = _qList[a];
                        if (fluid >= 0 && PFluid[p] != fluid) { continue; }
                        double dx = PX[p] - x, dy = PY[p] - y, dz = Dim == 3 ? PZ[p] - z : 0.0;
                        float r2 = (float)(dx * dx + dy * dy + dz * dz);
                        sum += Fluids[PFluid[p]]!.Mass * (double)k.W(r2);
                    }
                }
            }
        }

        return (float)sum;
    }

    // ------------------------------------------------------------------ submerged fraction, events

    private readonly Vector3F[] _samples = new Vector3F[64];
    private readonly float[] _depths = new float[64];
    private readonly List<WetRecord> _newWet = new();
    private int[] _bodyOrder = new int[8];

    /// <summary>
    /// How much of each body in or near the fluid is under it: sample points inside the body's shapes (a lattice, at most 64), each wet when a particle of the fluid lies within the point's depth plus three quarters of a spacing of it (a particle touching the nearest surface point).
    /// The fraction is wet points over points. Enter and exit events fire when a body's fraction passes 0.05 upward or 0.02 downward, ordered by body slot then fluid slot.
    /// </summary>
    private void EvaluateWetness(float dt)
    {
        Results.Clear();
        _newWet.Clear();
        EnsureQueryGrid();
        int nb = _bodyCount;
        if (_bodyOrder.Length < nb) { _bodyOrder = new int[Math.Max(nb, _bodyOrder.Length * 2)]; }
        for (int i = 0; i < nb; i++) { _bodyOrder[i] = i; }
        // Insertion sort by body slot (the table is in shape order; a body's slot decides the event order).
        for (int i = 1; i < nb; i++)
        {
            int v = _bodyOrder[i], j = i - 1;
            while (j >= 0 && _bodies[_bodyOrder[j]].Body > _bodies[v].Body) { _bodyOrder[j + 1] = _bodyOrder[j]; j--; }
            _bodyOrder[j + 1] = v;
        }

        var core = Host.Core;
        var wetCounts = _wetCounts;
        int oldPos = 0;
        for (int q = 0; q < nb; q++)
        {
            ref var b = ref _bodies[_bodyOrder[q]];
            // Old records of lower body slots that this step did not see end now.
            while (oldPos < WetCount && Wet[oldPos].Body < b.Body) { EmitExit(Wet[oldPos], 0f); oldPos++; }
            if (!core.Bodies.IsLive(b.Body)) { continue; }
            int gen = core.Bodies.Handles.GenerationOf(b.Body);
            int ns = _qBuckets == 0 ? 0 : Host.SamplePoints(b.Body, _samples, _depths);
            Array.Clear(wetCounts, 0, wetCounts.Length);
            for (int s = 0; s < ns; s++)
            {
                var lp = _samples[s];
                // Sample in the body's frame at its pose at the start of the step: the body is where the fluid found it.
                double wx, wy, wz;
                RotateToWorld(in b, lp, out wx, out wy, out wz);
                // Fluid touches the body near this point's nearest surface: a particle within the depth plus three quarters of a spacing (a contact radius and a quarter) of it.
                double reach = _depths[s] + 0.75 * Spacing;
                for (int f = 0; f < FluidHandles.HighWater; f++)
                {
                    if (!FluidHandles.IsLive(f)) { continue; }
                    if (AnyWithin(wx, wy, wz, reach, f)) { wetCounts[f]++; }
                }
            }

            // Records of this slot with an older generation belong to a body that is gone.
            for (int k = oldPos; k < WetCount && Wet[k].Body <= b.Body; k++) { if (Wet[k].Body == b.Body && Wet[k].BodyGen != gen) { EmitExit(Wet[k], 0f); } }
            for (int f = 0; f < FluidHandles.HighWater; f++)
            {
                if (!FluidHandles.IsLive(f)) { continue; }
                float frac = ns > 0 ? (float)wetCounts[f] / ns : 0f;
                bool wasWet = false;
                for (int w = 0; w < WetCount; w++)
                {
                    if (Wet[w].Body == b.Body && Wet[w].BodyGen == gen && Wet[w].Fluid == f) { wasWet = true; break; }
                }

                bool wet = wasWet ? frac >= ExitFraction : frac >= EnterFraction;
                if (wet && !wasWet) { Events.Add(new FluidEvent(FluidEventKind.Enter, core.HandleOfBody(b.Body), f, frac)); }
                if (!wet && wasWet) { Events.Add(new FluidEvent(FluidEventKind.Exit, core.HandleOfBody(b.Body), f, frac)); }
                if (wet) { _newWet.Add(new WetRecord { Body = b.Body, BodyGen = gen, Fluid = f, Wet = true }); }
                if (frac > 0f || b.WetArea > 0f)
                {
                    float inv = dt > 0f ? 1f / dt : 0f;
                    Results.Add(new BodyFluidResult { Body = b.Body, BodyGen = gen, Fluid = f, Fraction = frac, WetArea = b.WetArea, Fx = b.Tx * inv, Fy = b.Ty * inv, Fz = b.Tz * inv });
                }
            }

            while (oldPos < WetCount && Wet[oldPos].Body <= b.Body) { oldPos++; }
        }

        while (oldPos < WetCount) { EmitExit(Wet[oldPos], 0f); oldPos++; }
        if (Wet.Length < _newWet.Count) { Wet = new WetRecord[Math.Max(_newWet.Count, Wet.Length * 2)]; }
        for (int i = 0; i < _newWet.Count; i++) { Wet[i] = _newWet[i]; }
        WetCount = _newWet.Count;
    }

    private void EmitExit(in WetRecord r, float frac)
    {
        // A record of a body that no longer exists (or whose slot was reused) still ends; the handle is the one the record held.
        var core = Host.Core;
        var handle = new BodyHandle(core.Id, r.Body, r.BodyGen);
        Events.Add(new FluidEvent(FluidEventKind.Exit, handle, r.Fluid, frac));
    }

    private static void RotateToWorld(in FluidBody b, Vector3F p, out double wx, out double wy, out double wz)
    {
        // v' = v + 2 q.w (q.xyz x v) + 2 q.xyz x (q.xyz x v)
        float qx = b.StartQx, qy = b.StartQy, qz = b.StartQz, qw = b.StartQw;
        float tx = 2f * (qy * p.Z - qz * p.Y), ty = 2f * (qz * p.X - qx * p.Z), tz = 2f * (qx * p.Y - qy * p.X);
        float rx = p.X + qw * tx + (qy * tz - qz * ty), ry = p.Y + qw * ty + (qz * tx - qx * tz), rz = p.Z + qw * tz + (qx * ty - qy * tx);
        wx = b.StartX + rx; wy = b.StartY + ry; wz = b.StartZ + rz;
    }

    internal bool TryGetResult(int body, int gen, int fluid, out BodyFluidResult result)
    {
        foreach (var r in Results)
        {
            if (r.Body == body && r.BodyGen == gen && r.Fluid == fluid) { result = r; return true; }
        }

        result = default;
        return false;
    }
}
