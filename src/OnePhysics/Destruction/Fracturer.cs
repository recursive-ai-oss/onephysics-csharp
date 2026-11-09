using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Destruction;

/// <summary>How fracture seeds are placed.</summary>
public enum FracturePattern
{
    /// <summary>Seeds spread uniformly through the piece (rejection sampling inside its bounds).</summary>
    Uniform = 0,

    /// <summary>Seeds crowd around the impact point: the distance from it is the impact radius times a uniform number raised to the cluster power, so more cells are small and near the impact.</summary>
    Clustered = 1,

    /// <summary>The caller supplies the seeds.</summary>
    Points = 2,
}

/// <summary>A seeded generator (SplitMix64, Steele, Lea and Flood 2014). The library never uses a platform generator; equal seeds give equal sequences everywhere.</summary>
internal struct SeedRng
{
    private ulong _state;

    public SeedRng(ulong seed) { _state = seed; }

    public ulong Next()
    {
        ulong z = _state += 0x9E3779B97F4A7C15UL;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
        return z ^ (z >> 31);
    }

    /// <summary>A double in [0, 1): the top 53 bits scaled by 2^-53, exact.</summary>
    public double NextDouble() => (Next() >> 11) * (1.0 / 9007199254740992.0);
}

/// <summary>Voronoi fracture of a convex piece: each cell is the piece cut by the bisector planes between its seed and the others, so the cells tile the piece exactly.</summary>
internal static class Fracturer
{
    /// <summary>
    /// Places <paramref name="count"/> seeds in the piece. <paramref name="impact"/> is in the piece's frame; <paramref name="radius"/> and <paramref name="power"/> shape the clustered pattern.
    /// User seeds are used as given. Deterministic in (piece, parameters, seed).
    /// </summary>
    public static Vector3D[] MakeSeeds(IReadOnlyList<Piece> pieces, int count, FracturePattern pattern, ulong seed, Vector3D impact, double radius, double power, ReadOnlySpan<Vector3D> user)
    {
        if (pattern == FracturePattern.Points) return user.ToArray();
        var rng = new SeedRng(seed);
        var lo = new Vector3D(double.MaxValue, double.MaxValue, double.MaxValue); var hi = -lo;
        foreach (var piece in pieces)
        {
            piece.Bounds(out var l, out var h);
            lo = new Vector3D(Math.Min(lo.X, l.X), Math.Min(lo.Y, l.Y), Math.Min(lo.Z, l.Z));
            hi = new Vector3D(Math.Max(hi.X, h.X), Math.Max(hi.Y, h.Y), Math.Max(hi.Z, h.Z));
        }
        bool planar = pieces[0].Planar;
        var seeds = new List<Vector3D>(count);
        int attempts = 0, limit = 200 * count + 200;
        while (seeds.Count < count && attempts++ < limit)
        {
            Vector3D p;
            if (pattern == FracturePattern.Clustered)
            {
                // A direction from rejection in the unit ball (no trigonometry), scaled by radius * u^power.
                Vector3D d;
                int guard = 0;
                do
                {
                    d = new Vector3D(rng.NextDouble() * 2 - 1, rng.NextDouble() * 2 - 1, planar ? 0 : rng.NextDouble() * 2 - 1);
                } while ((d.LengthSquared() > 1 || d.LengthSquared() < 1e-12) && ++guard < 64);
                if (d.LengthSquared() > 1 || d.LengthSquared() < 1e-12) d = new Vector3D(1, 0, 0);
                d /= d.Length();
                double u = rng.NextDouble();
                p = impact + d * (radius * Math64.Pow(u, power));
            }
            else
            {
                p = new Vector3D(lo.X + (hi.X - lo.X) * rng.NextDouble(), lo.Y + (hi.Y - lo.Y) * rng.NextDouble(), planar ? 0 : lo.Z + (hi.Z - lo.Z) * rng.NextDouble());
            }
            bool inside = false;
            foreach (var piece in pieces) if (piece.Contains(p, 0)) { inside = true; break; }
            if (inside) seeds.Add(p);
        }
        // A piece too thin for rejection sampling still gets its cells: fill with the centroid plus tiny offsets.
        pieces[0].Measure(out _, out var c);
        for (int i = 0; seeds.Count < count; i++)
            seeds.Add(c + new Vector3D(1e-6 * (i + 1), 1e-6 * ((i * 7) % 5), planar ? 0 : 1e-6 * ((i * 3) % 7)));
        return seeds.ToArray();
    }

    /// <summary>
    /// The cell of every seed: the piece clipped by the bisector plane against each other seed, nearest seeds first, stopping once every farther seed lies beyond twice the cell's reach.
    /// A seed within 1e-9 of an earlier seed gets a null cell. A cell that clips away is null.
    /// </summary>
    public static Piece?[] Cells(Piece source, Vector3D[] seeds)
    {
        int n = seeds.Length;
        var cells = new Piece?[n];
        var order = new int[n];
        var dist = new double[n];
        for (int i = 0; i < n; i++)
        {
            bool dup = false;
            for (int j = 0; j < i && !dup; j++) if ((seeds[i] - seeds[j]).LengthSquared() < 1e-18) dup = true;
            if (dup) continue;
            int m = 0;
            for (int j = 0; j < n; j++) if (j != i) { order[m] = j; dist[j] = (seeds[j] - seeds[i]).LengthSquared(); m++; }
            Array.Sort(order, 0, m, Comparer<int>.Create((a, b) => { int c = dist[a].CompareTo(dist[b]); return c != 0 ? c : a.CompareTo(b); }));
            var cell = source;
            double reach = cell.Radius(seeds[i]);
            for (int k = 0; k < m && cell != null; k++)
            {
                int j = order[k];
                double d2 = dist[j];
                if (d2 < 1e-18) continue; // duplicate seed: the earlier index owns the region
                if (Math.Sqrt(d2) * 0.5 > reach) break;
                var normal = (seeds[j] - seeds[i]) / Math.Sqrt(d2);
                double offset = Vector3D.Dot(normal, (seeds[i] + seeds[j]) * 0.5);
                var clipped = cell.Clip(normal, offset);
                if (!ReferenceEquals(clipped, cell)) { cell = clipped; if (cell != null) reach = cell.Radius(seeds[i]); }
            }
            cells[i] = cell;
        }
        return cells;
    }

    /// <summary>A face shared by two pieces: its measure (area in 3D, length in 2D), centroid and unit normal pointing from the first piece into the second.</summary>
    internal readonly record struct SharedFace(double Measure, Vector3D Centroid, Vector3D Normal);

    /// <summary>The contact between two pieces of one body: coplanar opposite faces overlapping with area (3D) or collinear opposite edges overlapping with length (2D). Null when they do not touch.</summary>
    public static SharedFace? Shared(Piece a, Piece b)
    {
        // Cells rounded to float independently share faces only to float precision, so the tests are relative to the pieces' size.
        a.Bounds(out var alo, out var ahi);
        double scale = Math.Max(Math.Max(ahi.X - alo.X, ahi.Y - alo.Y), Math.Max(ahi.Z - alo.Z, 1e-30));
        double tol = 1e-5;
        if (a.Planar)
        {
            var pa = a.Faces[0]; var pb = b.Faces[0];
            double best = 0; Vector3D bc = default, bn = default;
            for (int i = 0; i < pa.Length; i++)
            {
                var a0 = pa[i]; var a1 = pa[(i + 1) % pa.Length];
                var ea = a1 - a0; double la = ea.Length(); if (la == 0) continue;
                var ta = ea / la;
                for (int j = 0; j < pb.Length; j++)
                {
                    var b0 = pb[j]; var b1 = pb[(j + 1) % pb.Length];
                    var eb = b1 - b0; double lb = eb.Length(); if (lb == 0) continue;
                    var tb = eb / lb;
                    if (Vector3D.Dot(ta, tb) > -1 + 1e-9) continue; // must be antiparallel
                    double off = Math.Abs(ta.X * (b0.Y - a0.Y) - ta.Y * (b0.X - a0.X));
                    if (off > tol * Math.Max(la, 1e-12) + 1e-9 * scale) continue;
                    double t0 = Vector3D.Dot(ta, b1 - a0), t1 = Vector3D.Dot(ta, b0 - a0); // b runs against a
                    double lo = Math.Max(0, Math.Min(t0, t1)), hi = Math.Min(la, Math.Max(t0, t1));
                    if (hi - lo > best)
                    {
                        best = hi - lo;
                        bc = a0 + ta * ((lo + hi) * 0.5);
                        bn = new Vector3D(ta.Y, -ta.X, 0); // outward normal of a's edge (counter-clockwise polygon)
                    }
                }
            }
            return best > 1e-5 * scale ? new SharedFace(best, bc, bn) : null;
        }
        double total = 0; var csum = default(Vector3D); var nrm = default(Vector3D);
        foreach (var fa in a.Faces)
        {
            var na = Piece.FaceNormal(fa);
            foreach (var fb in b.Faces)
            {
                var nb = Piece.FaceNormal(fb);
                if (Vector3D.Dot(na, nb) > -1 + 1e-9) continue;
                if (Math.Abs(Vector3D.Dot(na, fb[0] - fa[0])) > tol * scale) continue;
                // Clip polygon fa by the side planes of fb, inside the common plane.
                var poly = new List<Vector3D>(fa);
                for (int i = 0; i < fb.Length && poly.Count >= 3; i++)
                {
                    var p0 = fb[i]; var p1 = fb[(i + 1) % fb.Length];
                    // fb is counter-clockwise about nb, so its interior is to the left of each edge about nb.
                    var inward = Vector3D.Cross(nb, p1 - p0);
                    double len = inward.Length(); if (len == 0) continue;
                    inward /= len;
                    poly = ClipPolygon(poly, inward, Vector3D.Dot(inward, p0));
                }
                if (poly.Count < 3) continue;
                double ar = 0; var cc = default(Vector3D);
                var o = poly[0];
                for (int i = 1; i + 1 < poly.Count; i++)
                {
                    var u = poly[i] - o; var v = poly[i + 1] - o;
                    double t = Vector3D.Dot(Vector3D.Cross(u, v), na) * 0.5;
                    ar += t;
                    cc += (o + poly[i] + poly[i + 1]) * (t / 3);
                }
                if (ar > 0) { total += ar; csum += cc; nrm = na; }
            }
        }
        return total > 1e-8 * scale * scale ? new SharedFace(total, csum / total, nrm) : null;
    }

    // Keeps Dot(inward, x) >= d.
    private static List<Vector3D> ClipPolygon(List<Vector3D> poly, Vector3D inward, double d)
    {
        var res = new List<Vector3D>(poly.Count + 1);
        for (int i = 0; i < poly.Count; i++)
        {
            var a = poly[i]; var b = poly[(i + 1) % poly.Count];
            double da = Vector3D.Dot(inward, a) - d, db = Vector3D.Dot(inward, b) - d;
            if (da >= 0) res.Add(a);
            if ((da > 0 && db < 0) || (da < 0 && db > 0)) { double t = da / (da - db); res.Add(a + (b - a) * t); }
        }
        return res;
    }
}
