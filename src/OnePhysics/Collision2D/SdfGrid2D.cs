using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics;

/// <summary>
/// An immutable 2D signed distance field sampled on a regular grid (negative inside, positive outside, x-fastest), bilinear between samples; the gradient is the interpolant's.
/// Outside the grid box the field is the distance to the box plus the clamped value floored at zero. A static body collides with the field's zero contour,
/// extracted by marching squares (Lorensen and Cline 1987) with linear interpolation along cell edges and simplified within a tolerance
/// (see <see cref="ExtractContours"/>); <see cref="Trace"/> sphere-traces the field itself.
/// </summary>
public sealed class SdfGrid2D
{
    /// <summary>Largest sample count, 2^20; each dimension is 2..2048.</summary>
    public const int MaxSamples = 1 << 20;
    private readonly float[] _values;
    private readonly float _lipschitz;

    /// <summary>Copies samples. Dimensions 2..2048 with at most 2^20 samples, a finite cell size of at least 1e-4, a finite origin, extent within 2^20 metres, finite samples. Throws ArgumentOutOfRangeException otherwise.</summary>
    public SdfGrid2D(int sizeX, int sizeY, ReadOnlySpan<float> samples, float cellSize, Vec2 origin = default)
    {
        if (sizeX < 2 || sizeX > 2048 || sizeY < 2 || sizeY > 2048 || (long)sizeX * sizeY > MaxSamples) throw new ArgumentOutOfRangeException(nameof(sizeX), sizeX, "Grid dimensions must each be 2..2048 with at most 2^20 samples in total.");
        if (!float.IsFinite(cellSize) || cellSize < 1e-4f || (double)Math.Max(sizeX, sizeY) * cellSize > 1048576) throw new ArgumentOutOfRangeException(nameof(cellSize), cellSize, "Cell size must be finite, at least 1e-4, with total extent at most 2^20 metres.");
        if (!float.IsFinite(origin.X) || !float.IsFinite(origin.Y) || Math.Abs(origin.X) > 1048576 || Math.Abs(origin.Y) > 1048576) throw new ArgumentOutOfRangeException(nameof(origin), origin, "Origin must be finite and within 2^20 metres.");
        if (samples.Length != sizeX * sizeY) throw new ArgumentOutOfRangeException(nameof(samples), samples.Length, "Sample count must equal sizeX * sizeY.");
        for (int i = 0; i < samples.Length; i++) if (!float.IsFinite(samples[i])) throw new ArgumentOutOfRangeException(nameof(samples), i, "Sample " + i + " is not finite.");
        SizeX = sizeX; SizeY = sizeY; CellSize = cellSize; Origin = origin; _values = samples.ToArray();
        double best = 1.0, inv = 1.0 / cellSize;
        for (int y = 0; y < sizeY; y++) for (int x = 0; x < sizeX; x++)
        {
            double dx = x + 1 < sizeX ? (double)At(x + 1, y) - At(x, y) : (x > 0 ? (double)At(x, y) - At(x - 1, y) : 0);
            double dy = y + 1 < sizeY ? (double)At(x, y + 1) - At(x, y) : (y > 0 ? (double)At(x, y) - At(x, y - 1) : 0);
            double g = Math.Sqrt(dx * dx + dy * dy) * inv; if (g > best) best = g;
        }
        _lipschitz = (float)(best * 1.4142135623730951);
    }

    /// <summary>Samples along X.</summary>
    public int SizeX { get; }
    /// <summary>Samples along Y.</summary>
    public int SizeY { get; }
    /// <summary>Distance between neighbouring samples.</summary>
    public float CellSize { get; }
    /// <summary>Position of sample (0,0).</summary>
    public Vec2 Origin { get; }
    /// <summary>The copied samples, x-fastest.</summary>
    public ReadOnlySpan<float> Samples => _values;
    /// <summary>The largest gradient magnitude of the interpolant (never below 1); sphere tracing divides each step by it.</summary>
    public float Lipschitz => _lipschitz;
    private float At(int x, int y) => _values[y * SizeX + x];
    private float MaxX => Origin.X + (SizeX - 1) * CellSize;
    private float MaxY => Origin.Y + (SizeY - 1) * CellSize;

    private float Interior(float px, float py)
    {
        float inv = 1f / CellSize, ux = (px - Origin.X) * inv, uy = (py - Origin.Y) * inv;
        int ix = Math.Clamp((int)MathF.Floor(ux), 0, SizeX - 2), iy = Math.Clamp((int)MathF.Floor(uy), 0, SizeY - 2);
        float tx = Math.Clamp(ux - ix, 0f, 1f), ty = Math.Clamp(uy - iy, 0f, 1f);
        float a = At(ix, iy) + (At(ix + 1, iy) - At(ix, iy)) * tx, b = At(ix, iy + 1) + (At(ix + 1, iy + 1) - At(ix, iy + 1)) * tx;
        return a + (b - a) * ty;
    }

    /// <summary>The field at a point; see the type remarks for the value outside the box. Throws ArgumentOutOfRangeException for a non-finite point.</summary>
    public float Value(Vec2 p)
    {
        Check(p);
        float qx = Math.Clamp(p.X, Origin.X, MaxX), qy = Math.Clamp(p.Y, Origin.Y, MaxY);
        float inside = Interior(qx, qy), dx = p.X - qx, dy = p.Y - qy, outside = MathF.Sqrt(dx * dx + dy * dy);
        return outside == 0f ? inside : (inside > 0f ? inside : 0f) + outside;
    }

    /// <summary>The unit gradient of the interpolant (away from the solid); +Y where it vanishes. Throws ArgumentOutOfRangeException for a non-finite point.</summary>
    public Vec2 Gradient(Vec2 p)
    {
        Check(p);
        float qx = Math.Clamp(p.X, Origin.X, MaxX), qy = Math.Clamp(p.Y, Origin.Y, MaxY), dx = p.X - qx, dy = p.Y - qy, outside = MathF.Sqrt(dx * dx + dy * dy);
        if (outside > 0f) return new Vec2(dx / outside, dy / outside);
        float inv = 1f / CellSize, ux = (qx - Origin.X) * inv, uy = (qy - Origin.Y) * inv;
        int ix = Math.Clamp((int)MathF.Floor(ux), 0, SizeX - 2), iy = Math.Clamp((int)MathF.Floor(uy), 0, SizeY - 2);
        float tx = Math.Clamp(ux - ix, 0f, 1f), ty = Math.Clamp(uy - iy, 0f, 1f);
        float v00 = At(ix, iy), v10 = At(ix + 1, iy), v01 = At(ix, iy + 1), v11 = At(ix + 1, iy + 1);
        float gx = (v10 - v00) * (1 - ty) + (v11 - v01) * ty, gy = (v01 - v00) * (1 - tx) + (v11 - v10) * tx, len = MathF.Sqrt(gx * gx + gy * gy);
        return len > 1e-12f ? new Vec2(gx / len, gy / len) : new Vec2(0, 1);
    }

    private static void Check(Vec2 p) { if (!float.IsFinite(p.X) || !float.IsFinite(p.Y)) throw new ArgumentOutOfRangeException(nameof(p), p, "Point must be finite."); }

    /// <summary>Sphere tracing (Hart 1996) of a ray against the zero contour: the direction is normalised, t is a distance in [0, maxT], each step is value / <see cref="Lipschitz"/> and cannot cross the contour. Returns false for a miss; a start inside hits at t = 0. Throws ArgumentOutOfRangeException for a non-finite or zero direction, non-finite origin or invalid maxT.</summary>
    public bool Trace(Vec2 origin, Vec2 direction, float maxT, out float t, out Vec2 normal, out bool startedInside)
    {
        Check(origin); Check(direction);
        if (!(maxT >= 0f) || !float.IsFinite(maxT)) throw new ArgumentOutOfRangeException(nameof(maxT), maxT, "maxT must be finite and not negative.");
        float len = MathF.Sqrt(direction.X * direction.X + direction.Y * direction.Y);
        if (!(len > 0f)) throw new ArgumentOutOfRangeException(nameof(direction), direction, "Direction must be nonzero.");
        float dx = direction.X / len, dy = direction.Y / len; t = 0; normal = default; startedInside = false;
        if (Value(origin) <= 0f) { startedInside = true; normal = Gradient(origin); return true; }
        float tol = 1e-4f * CellSize, k = 1f / Math.Max(1f, _lipschitz), at = 0f;
        for (int i = 0; i < 2048; i++)
        {
            var p = new Vec2(origin.X + dx * at, origin.Y + dy * at); float v = Value(p);
            if (v <= tol) { t = at; normal = Gradient(p); return true; }
            at += Math.Max(v * k, tol * 0.25f);
            if (at > maxT) return false;
        }
        return false;
    }

    /// <summary>Samples a function over a box: ceil(extent / cellSize) + 1 samples per axis from <paramref name="min"/>, in x-fastest order. Throws ArgumentOutOfRangeException for an empty, inverted or oversized box or a bad cell size.</summary>
    public static SdfGrid2D FromFunction(Vec2 min, Vec2 max, float cellSize, Func<Vec2, float> field)
    {
        ArgumentNullException.ThrowIfNull(field);
        if (!float.IsFinite(cellSize) || cellSize < 1e-4f) throw new ArgumentOutOfRangeException(nameof(cellSize), cellSize, "Cell size must be finite and at least 1e-4.");
        if (!float.IsFinite(min.X + min.Y + max.X + max.Y) || !(max.X > min.X) || !(max.Y > min.Y)) throw new ArgumentOutOfRangeException(nameof(max), max, "Box must be finite with max above min.");
        int nx = (int)Math.Min(5000, Math.Ceiling((double)(max.X - min.X) / cellSize) + 1), ny = (int)Math.Min(5000, Math.Ceiling((double)(max.Y - min.Y) / cellSize) + 1);
        if (nx > 2048 || ny > 2048 || (long)nx * ny > MaxSamples) throw new ArgumentOutOfRangeException(nameof(cellSize), cellSize, "The box needs " + nx + " x " + ny + " samples; the limit is 2048 per axis and 2^20 in total.");
        var samples = new float[nx * ny]; int i = 0;
        for (int y = 0; y < ny; y++) for (int x = 0; x < nx; x++) samples[i++] = field(new Vec2(min.X + x * cellSize, min.Y + y * cellSize));
        return new SdfGrid2D(nx, ny, samples, cellSize, min);
    }

    /// <summary>The exact distance field of a simple closed polygon (3 to 4096 points, either winding; the inside is where the winding number is not zero). The grid covers the polygon's bounds plus <paramref name="padding"/> cells. Throws ArgumentOutOfRangeException for bad input.</summary>
    public static SdfGrid2D FromPolygon(ReadOnlySpan<Vec2> points, float cellSize, int padding = 2)
    {
        if (points.Length < 3 || points.Length > 4096) throw new ArgumentOutOfRangeException(nameof(points), points.Length, "A polygon needs 3 to 4096 points.");
        if (padding < 1 || padding > 64) throw new ArgumentOutOfRangeException(nameof(padding), padding, "Padding must be 1..64 cells.");
        double x0 = double.MaxValue, y0 = double.MaxValue, x1 = double.MinValue, y1 = double.MinValue;
        for (int i = 0; i < points.Length; i++)
        {
            if (!float.IsFinite(points[i].X) || !float.IsFinite(points[i].Y) || Math.Abs(points[i].X) > 1048576 || Math.Abs(points[i].Y) > 1048576) throw new ArgumentOutOfRangeException(nameof(points), i, "Point " + i + " is not finite or is beyond 2^20 metres.");
            x0 = Math.Min(x0, points[i].X); x1 = Math.Max(x1, points[i].X); y0 = Math.Min(y0, points[i].Y); y1 = Math.Max(y1, points[i].Y);
        }
        var poly = points.ToArray(); double pad = padding * (double)cellSize;
        return FromFunction(new Vec2((float)(x0 - pad), (float)(y0 - pad)), new Vec2((float)(x1 + pad), (float)(y1 + pad)), cellSize, p =>
        {
            double best = double.MaxValue; int winding = 0;
            for (int a = 0, b = poly.Length - 1; a < poly.Length; b = a++)
            {
                double ax = poly[b].X, ay = poly[b].Y, bx = poly[a].X, by = poly[a].Y, ex = bx - ax, ey = by - ay, px = p.X - ax, py = p.Y - ay;
                double len2 = ex * ex + ey * ey, t = len2 > 0 ? Math.Clamp((px * ex + py * ey) / len2, 0, 1) : 0, dx = px - t * ex, dy = py - t * ey, d2 = dx * dx + dy * dy;
                if (d2 < best) best = d2;
                // Winding number (Sunday 2001): crossings of a ray towards +x, signed by direction.
                if (ay <= p.Y) { if (by > p.Y && ex * (p.Y - ay) - ey * (p.X - ax) > 0) winding++; }
                else if (by <= p.Y && ex * (p.Y - ay) - ey * (p.X - ax) < 0) winding--;
            }
            float d = (float)Math.Sqrt(best); return winding != 0 ? -d : d;
        });
    }

    /// <summary>The exact distance field of a circle.</summary>
    public static Func<Vec2, float> Circle(Vec2 center, float radius) => p => MathF.Sqrt((p.X - center.X) * (p.X - center.X) + (p.Y - center.Y) * (p.Y - center.Y)) - radius;
    /// <summary>The exact distance field of an axis-aligned box.</summary>
    public static Func<Vec2, float> Box(Vec2 center, Vec2 halfExtents) => p =>
    {
        float qx = MathF.Abs(p.X - center.X) - halfExtents.X, qy = MathF.Abs(p.Y - center.Y) - halfExtents.Y, ox = Math.Max(qx, 0f), oy = Math.Max(qy, 0f);
        return MathF.Sqrt(ox * ox + oy * oy) + Math.Min(Math.Max(qx, qy), 0f);
    };
    /// <summary>The half plane behind a line through <paramref name="point"/> with the given unit normal.</summary>
    public static Func<Vec2, float> HalfPlane(Vec2 point, Vec2 normal) => p => (p.X - point.X) * normal.X + (p.Y - point.Y) * normal.Y;
    /// <summary>The union of two fields (the smaller value).</summary>
    public static Func<Vec2, float> Union(Func<Vec2, float> a, Func<Vec2, float> b) => p => Math.Min(a(p), b(p));
    /// <summary>The intersection of two fields (the larger value).</summary>
    public static Func<Vec2, float> Intersection(Func<Vec2, float> a, Func<Vec2, float> b) => p => Math.Max(a(p), b(p));
    /// <summary>The first field with the second cut away.</summary>
    public static Func<Vec2, float> Difference(Func<Vec2, float> a, Func<Vec2, float> b) => p => Math.Max(a(p), -b(p));

    /// <summary>
    /// Extracts the zero contour as closed loops of points, counter-clockwise with the inside on the left (the orientation one-sided chains and tile boundaries use). Samples outside the grid count as empty, so a solid
    /// that reaches the grid edge is closed just outside it. Crossings lie on cell edges at the linear zero; each loop is then simplified by dropping points that stay within
    /// <paramref name="tolerance"/> metres of the chord (Douglas and Peucker 1973), keeping at least 3 points. Loops come in order of their lowest edge index, starting at that edge, so the result is deterministic.
    /// Throws ArgumentOutOfRangeException for a tolerance that is negative or not finite.
    /// </summary>
    public List<Vec2[]> ExtractContours(float tolerance = 0f)
    {
        if (!(tolerance >= 0f) || !float.IsFinite(tolerance)) throw new ArgumentOutOfRangeException(nameof(tolerance), tolerance, "Tolerance must be finite and not negative.");
        int sx = SizeX + 2;  // corner columns -1..SizeX, so the padded edge ids never collide
        float V(int x, int y) => (uint)x < (uint)SizeX && (uint)y < (uint)SizeY ? At(x, y) : CellSize;
        // Edge ids: horizontal edge from corner (x,y) to (x+1,y) is 2*(y+1)*(SizeX+1)+2*(x+1); vertical edge from (x,y) to (x,y+1) is that +1.
        int H(int x, int y) => 2 * ((y + 1) * sx + (x + 1)); int Vv(int x, int y) => H(x, y) + 1;
        Vec2 Cross(int id)
        {
            bool vertical = (id & 1) != 0; int e = id >> 1, x = e % sx - 1, y = e / sx - 1;
            float a = V(x, y), b = vertical ? V(x, y + 1) : V(x + 1, y), t = a / (a - b);
            float px = Origin.X + (x + (vertical ? 0 : t)) * CellSize, py = Origin.Y + (y + (vertical ? t : 0)) * CellSize;
            return new Vec2(px, py);
        }
        var next = new SortedDictionary<int, int>();
        for (int cy = -1; cy < SizeY; cy++) for (int cx = -1; cx < SizeX; cx++)
        {
            float c0 = V(cx, cy), c1 = V(cx + 1, cy), c2 = V(cx + 1, cy + 1), c3 = V(cx, cy + 1);
            int mask = (c0 < 0 ? 1 : 0) | (c1 < 0 ? 2 : 0) | (c2 < 0 ? 4 : 0) | (c3 < 0 ? 8 : 0);
            if (mask == 0 || mask == 15) continue;
            int e0 = H(cx, cy), e1 = Vv(cx + 1, cy), e2 = H(cx, cy + 1), e3 = Vv(cx, cy);
            bool centerInside = (c0 + c1 + c2 + c3) < 0;
            // Directed crossings with the inside on the left, from the combinatorics of the cell alone (so samples that are exactly zero cannot flip a direction).
            // A lone inside corner c_i is walked e_i -> e_(i-1); a lone outside corner c_o is walked e_(o-1) -> e_o; two adjacent inside corners c_i, c_(i+1) are walked e_(i+1) -> e_(i-1).
            switch (mask)
            {
                case 1: next[e0] = e3; break;
                case 2: next[e1] = e0; break;
                case 4: next[e2] = e1; break;
                case 8: next[e3] = e2; break;
                case 14: next[e3] = e0; break;
                case 13: next[e0] = e1; break;
                case 11: next[e1] = e2; break;
                case 7: next[e2] = e3; break;
                case 3: next[e1] = e3; break;
                case 6: next[e2] = e0; break;
                case 12: next[e3] = e1; break;
                case 9: next[e0] = e2; break;
                case 5: if (centerInside) { next[e0] = e1; next[e2] = e3; } else { next[e0] = e3; next[e2] = e1; } break;
                case 10: if (centerInside) { next[e3] = e0; next[e1] = e2; } else { next[e1] = e0; next[e3] = e2; } break;
            }
        }
        var loops = new List<Vec2[]>(); var done = new HashSet<int>();
        foreach (var start in next.Keys)
        {
            if (done.Contains(start)) continue;
            var pts = new List<Vec2>(); int cur = start; int guard = 0;
            while (done.Add(cur))
            {
                pts.Add(Cross(cur));
                if (!next.TryGetValue(cur, out int n)) { pts.Clear(); break; }
                cur = n; if (++guard > next.Count + 2) { pts.Clear(); break; }
            }
            // Crossings that coincide (a sample exactly on the contour) leave repeated points.
            for (int i = pts.Count - 1; i >= 0; i--) { var a = pts[i]; var b = pts[(i + 1) % pts.Count]; if (pts.Count > 3 && MathF.Abs(a.X - b.X) <= 1e-7f * CellSize && MathF.Abs(a.Y - b.Y) <= 1e-7f * CellSize) pts.RemoveAt(i); }
            if (pts.Count >= 3) loops.Add(Simplify(pts, tolerance));
        }
        return loops;
    }

    private static Vec2[] Simplify(List<Vec2> pts, float tol)
    {
        if (tol <= 0f || pts.Count <= 3) return pts.ToArray();
        // Split the closed loop at its two farthest-apart points (the first point and the one farthest from it), then simplify each half.
        int far = 0; float best = -1f;
        for (int i = 1; i < pts.Count; i++) { float dx = pts[i].X - pts[0].X, dy = pts[i].Y - pts[0].Y, d = dx * dx + dy * dy; if (d > best) { best = d; far = i; } }
        var keep = new bool[pts.Count]; keep[0] = true; keep[far] = true;
        Mark(pts, 0, far, tol, keep); MarkWrap(pts, far, tol, keep);
        var result = new List<Vec2>(); for (int i = 0; i < pts.Count; i++) if (keep[i]) result.Add(pts[i]);
        return result.Count >= 3 ? result.ToArray() : pts.ToArray();
    }
    private static void Mark(List<Vec2> p, int a, int b, float tol, bool[] keep)
    {
        var stack = new Stack<(int, int)>(); stack.Push((a, b));
        while (stack.Count != 0)
        {
            var (lo, hi) = stack.Pop(); if (hi - lo < 2) continue;
            float ax = p[lo].X, ay = p[lo].Y, ex = p[hi].X - ax, ey = p[hi].Y - ay, len = MathF.Sqrt(ex * ex + ey * ey), worst = -1f; int at = -1;
            for (int i = lo + 1; i < hi; i++)
            {
                float d = len > 0 ? MathF.Abs(ex * (p[i].Y - ay) - ey * (p[i].X - ax)) / len : MathF.Sqrt((p[i].X - ax) * (p[i].X - ax) + (p[i].Y - ay) * (p[i].Y - ay));
                if (d > worst) { worst = d; at = i; }
            }
            if (worst > tol) { keep[at] = true; stack.Push((lo, at)); stack.Push((at, hi)); }
        }
    }
    private static void MarkWrap(List<Vec2> p, int from, float tol, bool[] keep)
    {
        // The second half runs from `from` to the end and wraps to point 0: simplify it as its own open polyline.
        var tail = new List<Vec2>(); for (int i = from; i < p.Count; i++) tail.Add(p[i]); tail.Add(p[0]);
        var k2 = new bool[tail.Count]; k2[0] = true; k2[^1] = true; Mark(tail, 0, tail.Count - 1, tol, k2);
        for (int i = 0; i < tail.Count - 1; i++) if (k2[i]) keep[from + i] = true;
    }
}
