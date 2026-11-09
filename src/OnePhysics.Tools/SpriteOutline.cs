using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Tools;

/// <summary>Options for <see cref="SpriteOutliner"/>. The default traces at alpha 128, simplifies by one pixel, and gives coordinates in pixels with y pointing up.</summary>
public readonly struct SpriteOutlineOptions
{
    /// <summary>A pixel is solid when its alpha is at least this, from 1 to 255; null chooses 128.</summary>
    public byte? AlphaThreshold { get; init; }

    /// <summary>The most a simplified outline may stray from the traced contour, in pixels, finite and not negative; null chooses 1. 0 keeps every traced vertex.</summary>
    public float? Tolerance { get; init; }

    /// <summary>Islands and holes smaller than this many square pixels are dropped; null chooses 0 (keep all).</summary>
    public float? MinArea { get; init; }

    /// <summary>The size of a pixel in output units, finite and positive; null chooses 1.</summary>
    public float? PixelSize { get; init; }

    /// <summary>Where the image's lower-left corner (or, with <see cref="KeepImageRows"/>, upper-left corner) is placed in output units. Default (0, 0).</summary>
    public Vector2F Origin { get; init; }

    /// <summary>Keep the image's row order, y growing downwards, instead of flipping so that y points up (the physics convention). Default false.</summary>
    public bool KeepImageRows { get; init; }
}

/// <summary>One island of a sprite: an outer boundary and the holes inside it.</summary>
public sealed class SpriteIsland
{
    internal SpriteIsland(Vector2F[] outer, Vector2F[][] holes)
    {
        Outer = outer;
        Holes = holes;
    }

    /// <summary>The outer boundary, counter-clockwise in output coordinates (as the numbers read, whichever way y points).</summary>
    public Vector2F[] Outer { get; }

    /// <summary>The holes, each clockwise.</summary>
    public Vector2F[][] Holes { get; }

    /// <summary>Splits the island into convex pieces; see <see cref="PolygonDecomposer"/>.</summary>
    /// <param name="options">Piece vertex limit.</param>
    /// <returns>Convex pieces.</returns>
    /// <exception cref="GeometryToolException">The island cannot be decomposed (see <see cref="PolygonDecomposer"/>).</exception>
    public Vector2F[][] ToConvexPieces(PolygonDecompositionOptions options = default) => PolygonDecomposer.Decompose(Outer, Holes, options);
}

/// <summary>
/// Turns an alpha mask into outlines: marching squares on the alpha channel (Lorensen and Cline 1987, in 2D) places each contour point where the interpolated alpha crosses the threshold,
/// ambiguous cells keep diagonal solid pixels apart, loops are simplified with Douglas and Peucker's algorithm (1973) within a pixel tolerance, and holes are assigned to the island that contains them.
/// If simplification would make loops cross or touch, the tolerance is halved until they do not, so the result can always be decomposed.
/// </summary>
public static class SpriteOutliner
{
    /// <summary>Traces the outlines of an alpha mask.</summary>
    /// <param name="alpha">Alpha values, row-major, <paramref name="width"/> × <paramref name="height"/> bytes, the first row at the top of the image.</param>
    /// <param name="width">Image width in pixels, from 1 to 16384.</param>
    /// <param name="height">Image height in pixels, from 1 to 16384; width × height at most 2^26.</param>
    /// <param name="options">Threshold, tolerance and placement.</param>
    /// <returns>The islands, ordered by the position of their first contour edge (top row first, then leftwards in image order). A mask with nothing above the threshold gives none.</returns>
    /// <exception cref="GeometryToolException">The size is invalid or does not match the data, or an option is invalid.</exception>
    public static SpriteIsland[] Trace(ReadOnlySpan<byte> alpha, int width, int height, SpriteOutlineOptions options = default)
    {
        if (width < 1 || width > 16384 || height < 1 || height > 16384 || (long)width * height > (1L << 26))
        {
            throw new GeometryToolException(GeometryToolError.OutOfRange, "image size " + MeshInput.Inv(width) + " x " + MeshInput.Inv(height) + " must be 1 to 16384 on each side and at most 2^26 pixels", nameof(width));
        }

        if (alpha.Length != width * height)
        {
            throw new GeometryToolException(GeometryToolError.BadIndex, "alpha has " + MeshInput.Inv(alpha.Length) + " bytes, " + MeshInput.Inv(width) + " x " + MeshInput.Inv(height) + " = " + MeshInput.Inv(width * height) + " expected", nameof(alpha));
        }

        int thr = options.AlphaThreshold ?? 128;
        float tol = options.Tolerance ?? 1f, minArea = options.MinArea ?? 0f, px = options.PixelSize ?? 1f;
        if (thr < 1) { throw Bad("AlphaThreshold", MeshInput.Inv(thr), "from 1 to 255", nameof(options)); }
        if (!float.IsFinite(tol) || tol < 0f) { throw Bad("Tolerance", tol.ToString(System.Globalization.CultureInfo.InvariantCulture), "finite and not negative", nameof(options)); }
        if (!float.IsFinite(minArea) || minArea < 0f) { throw Bad("MinArea", minArea.ToString(System.Globalization.CultureInfo.InvariantCulture), "finite and not negative", nameof(options)); }
        if (!float.IsFinite(px) || !(px > 0f)) { throw Bad("PixelSize", px.ToString(System.Globalization.CultureInfo.InvariantCulture), "finite and positive", nameof(options)); }
        if (!float.IsFinite(options.Origin.X) || !float.IsFinite(options.Origin.Y)) { throw Bad("Origin", options.Origin.ToString(), "finite", nameof(options)); }

        var raw = TraceLoops(alpha, width, height, thr);
        // Drop tiny loops before simplifying.
        var loops = new List<(double X, double Y)[]>();
        foreach (var l in raw)
        {
            if (Math.Abs(PolygonInput.SignedArea(l)) >= minArea) { loops.Add(l); }
        }

        if (loops.Count == 0) { return Array.Empty<SpriteIsland>(); }

        // Simplify; halve the tolerance until the loops are simple and disjoint (the raw contour always is).
        List<(double X, double Y)[]> simple = loops;
        double t = tol;
        for (int attempt = 0; attempt < 12; attempt++)
        {
            var cand = new List<(double X, double Y)[]>(loops.Count);
            foreach (var l in loops) { cand.Add(t > 0 ? Simplify(l, t) : l); }
            cand.RemoveAll(c => c.Length < 3 || PolygonInput.SignedArea(c) == 0);
            if (t == 0 || PolygonInput.FindIntersection(cand) is null) { simple = cand; break; }
            t = attempt == 10 ? 0 : t * 0.5;
            if (attempt == 11) { simple = loops; }
        }

        // Outer boundaries run counter-clockwise on screen, which is negative area in image coordinates.
        var outers = new List<(double X, double Y)[]>();
        var holes = new List<(double X, double Y)[]>();
        foreach (var l in simple)
        {
            if (PolygonInput.SignedArea(l) < 0) { outers.Add(l); } else { holes.Add(l); }
        }

        var holeOf = new List<(double X, double Y)[]>[outers.Count];
        for (int i = 0; i < holeOf.Length; i++) { holeOf[i] = new List<(double X, double Y)[]>(); }
        foreach (var h in holes)
        {
            int best = -1;
            double bestArea = double.PositiveInfinity;
            for (int i = 0; i < outers.Count; i++)
            {
                double a = Math.Abs(PolygonInput.SignedArea(outers[i]));
                if (a < bestArea && PolygonInput.Inside(h[0], outers[i])) { best = i; bestArea = a; }
            }

            if (best >= 0) { holeOf[best].Add(h); }
        }

        var result = new SpriteIsland[outers.Count];
        for (int i = 0; i < result.Length; i++)
        {
            var hs = new Vector2F[holeOf[i].Count][];
            for (int k = 0; k < hs.Length; k++) { hs[k] = ToOutput(holeOf[i][k], height, px, options, clockwise: true); }
            result[i] = new SpriteIsland(ToOutput(outers[i], height, px, options, clockwise: false), hs);
        }

        return result;
    }

    /// <summary>Traces an alpha mask and splits every island into convex pieces.</summary>
    /// <param name="alpha">Alpha values, as for <see cref="Trace"/>.</param>
    /// <param name="width">Image width in pixels.</param>
    /// <param name="height">Image height in pixels.</param>
    /// <param name="outline">Tracing options.</param>
    /// <param name="decomposition">Piece vertex limit.</param>
    /// <returns>All convex pieces of all islands, counter-clockwise, each ready for a polygon shape.</returns>
    /// <exception cref="GeometryToolException">As for <see cref="Trace"/> and <see cref="PolygonDecomposer"/>.</exception>
    public static Vector2F[][] ToConvexPieces(ReadOnlySpan<byte> alpha, int width, int height, SpriteOutlineOptions outline = default, PolygonDecompositionOptions decomposition = default)
    {
        var pieces = new List<Vector2F[]>();
        foreach (var island in Trace(alpha, width, height, outline)) { pieces.AddRange(island.ToConvexPieces(decomposition)); }
        return pieces.ToArray();
    }

    private static GeometryToolException Bad(string name, string value, string expected, string paramName) =>
        new(GeometryToolError.InvalidOptions, name + " = " + value + " must be " + expected, paramName);

    private static Vector2F[] ToOutput((double X, double Y)[] loop, int height, float px, SpriteOutlineOptions o, bool clockwise)
    {
        var pts = new Vector2F[loop.Length];
        for (int i = 0; i < loop.Length; i++)
        {
            double x = loop[i].X * px + o.Origin.X;
            double y = (o.KeepImageRows ? loop[i].Y : height - loop[i].Y) * px + o.Origin.Y;
            pts[i] = new Vector2F((float)x, (float)y);
        }

        // Fix the winding as the numbers read: outer boundaries counter-clockwise, holes clockwise.
        double area = 0;
        for (int i = 0; i < pts.Length; i++)
        {
            var a = pts[i];
            var b = pts[(i + 1) % pts.Length];
            area += (double)a.X * b.Y - (double)b.X * a.Y;
        }

        if ((area > 0) == clockwise) { Array.Reverse(pts); }
        return pts;
    }

    // Marching squares over the padded sample grid; samples sit at pixel centres and the outside counts as zero.
    private static List<(double X, double Y)[]> TraceLoops(ReadOnlySpan<byte> alphaSpan, int w, int h, int thr)
    {
        byte[] alpha = alphaSpan.ToArray();
        int sw = w + 2; // padded sample columns -1..w
        int sh = h + 2;
        int S(int x, int y) => (uint)x < (uint)w && (uint)y < (uint)h ? alpha[y * w + x] : 0;

        // Crossing ids: 2 * (cellX + sw * cellY) + type, where the crossing lies on the horizontal (0) or vertical (1) grid edge that starts at sample (cellX - 1, cellY - 1).
        int Id(int x, int y, int type) => 2 * ((x + 1) + sw * (y + 1)) + type;
        var next = new int[2 * sw * sh];
        Array.Fill(next, -1);
        for (int j = 0; j <= h; j++)
        {
            for (int i = 0; i <= w; i++)
            {
                // Cell (i, j) has corner samples (i-1, j-1) TL, (i, j-1) TR, (i, j) BR, (i-1, j) BL.
                bool tl = S(i - 1, j - 1) >= thr, tr = S(i, j - 1) >= thr, br = S(i, j) >= thr, bl = S(i - 1, j) >= thr;
                int c = (tl ? 1 : 0) | (tr ? 2 : 0) | (br ? 4 : 0) | (bl ? 8 : 0);
                if (c == 0 || c == 15) { continue; }
                int T = Id(i - 1, j - 1, 0), B = Id(i - 1, j, 0), L = Id(i - 1, j - 1, 1), R = Id(i, j - 1, 1);
                switch (c)
                {
                    case 1: next[L] = T; break;
                    case 2: next[T] = R; break;
                    case 4: next[R] = B; break;
                    case 8: next[B] = L; break;
                    case 3: next[L] = R; break;
                    case 6: next[T] = B; break;
                    case 12: next[R] = L; break;
                    case 9: next[B] = T; break;
                    case 7: next[L] = B; break;
                    case 11: next[B] = R; break;
                    case 13: next[R] = T; break;
                    case 14: next[T] = L; break;
                    case 5: next[L] = T; next[R] = B; break;
                    case 10: next[T] = R; next[B] = L; break;
                }
            }
        }

        double Cross(int id)
        {
            int type = id & 1;
            int cell = id >> 1;
            int x = cell % sw - 1, y = cell / sw - 1;
            if (type == 0)
            {
                double a = S(x, y), b = S(x + 1, y);
                return x + 0.5 + Frac(a, b, thr);
            }

            double c0 = S(x, y), c1 = S(x, y + 1);
            return y + 0.5 + Frac(c0, c1, thr);
        }

        var loops = new List<(double X, double Y)[]>();
        var visited = new bool[next.Length];
        var pts = new List<(double X, double Y)>();
        for (int start = 0; start < next.Length; start++)
        {
            if (next[start] < 0 || visited[start]) { continue; }
            pts.Clear();
            int id = start;
            while (!visited[id])
            {
                visited[id] = true;
                int type = id & 1, cell = id >> 1;
                int x = cell % sw - 1, y = cell / sw - 1;
                pts.Add(type == 0 ? (Cross(id), y + 0.5) : (x + 0.5, Cross(id)));
                id = next[id];
                if (id < 0) { break; }
            }

            if (id == start && pts.Count >= 3) { loops.Add(pts.ToArray()); }
        }

        return loops;
    }

    // Where along a grid edge the interpolated alpha equals the threshold (0 at the first sample, 1 at the second).
    private static double Frac(double a, double b, int thr)
    {
        if (a == b) { return 0.5; }
        double t = (thr - a) / (b - a);
        return t < 0 ? 0 : (t > 1 ? 1 : t);
    }

    // Douglas and Peucker (1973) on a closed loop: split at the first point and the point furthest from it, then simplify both chains.
    private static (double X, double Y)[] Simplify((double X, double Y)[] loop, double tol)
    {
        int n = loop.Length;
        if (n <= 3) { return loop; }
        int far = 0;
        double best = -1;
        for (int i = 1; i < n; i++)
        {
            double d = (loop[i].X - loop[0].X) * (loop[i].X - loop[0].X) + (loop[i].Y - loop[0].Y) * (loop[i].Y - loop[0].Y);
            if (d > best) { best = d; far = i; }
        }

        var keep = new bool[n];
        keep[0] = true;
        keep[far] = true;
        var stack = new Stack<(int, int)>();
        stack.Push((0, far));
        stack.Push((far, n)); // the second chain ends back at point 0 (index n wraps)
        while (stack.Count > 0)
        {
            var (a, b) = stack.Pop();
            if (b - a < 2) { continue; }
            var pa = loop[a];
            var pb = loop[b % n];
            int worst = -1;
            double wd = tol;
            for (int i = a + 1; i < b; i++)
            {
                double d = SegmentDistance(loop[i], pa, pb);
                if (d > wd) { wd = d; worst = i; }
            }

            if (worst >= 0)
            {
                keep[worst] = true;
                stack.Push((a, worst));
                stack.Push((worst, b));
            }
        }

        var res = new List<(double X, double Y)>();
        for (int i = 0; i < n; i++) { if (keep[i]) { res.Add(loop[i]); } }
        return res.ToArray();
    }

    private static double SegmentDistance((double X, double Y) p, (double X, double Y) a, (double X, double Y) b)
    {
        double dx = b.X - a.X, dy = b.Y - a.Y;
        double len2 = dx * dx + dy * dy;
        double t = len2 == 0 ? 0 : ((p.X - a.X) * dx + (p.Y - a.Y) * dy) / len2;
        t = t < 0 ? 0 : (t > 1 ? 1 : t);
        double qx = a.X + t * dx - p.X, qy = a.Y + t * dy - p.Y;
        return Math.Sqrt(qx * qx + qy * qy);
    }
}
