using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Tools;

/// <summary>Options for <see cref="PolygonDecomposer.Decompose(ReadOnlySpan{Vector2F}, PolygonDecompositionOptions)"/>.</summary>
public readonly struct PolygonDecompositionOptions
{
    /// <summary>The most vertices a convex piece may have, from 3 to 64; 0 chooses 8, the 2D engine's polygon limit.</summary>
    public int MaxVerticesPerPiece { get; init; }
}

/// <summary>
/// Splits a simple polygon, with or without holes, into convex pieces that tile it exactly: holes are joined to the outline by bridges and the result is triangulated by ear clipping
/// (Eberly, "Triangulation by Ear Clipping"), then triangles are merged across diagonals while the piece stays convex and within the vertex limit (Hertel and Mehlhorn, 1983,
/// "Fast triangulation of the plane with respect to simple polygons"), which gives at most four times the minimum number of pieces. Every orientation and intersection test is exact
/// (Shewchuk-style predicates from onemath), so the pieces share their vertices exactly and neither overlap nor leave gaps.
/// </summary>
public static class PolygonDecomposer
{
    /// <summary>Decomposes a simple polygon.</summary>
    /// <param name="outline">The polygon's corners in either winding, 3 to 16384 finite points within ±2^20; a repeated closing point is ignored.</param>
    /// <param name="options">Piece vertex limit; the default is 8.</param>
    /// <returns>Convex pieces, counter-clockwise, without collinear vertices. Identical input gives identical output on every platform.</returns>
    /// <exception cref="GeometryToolException">The outline is too small or too large, has a non-finite or out-of-range point, has no area, crosses or touches itself, or an option is invalid.</exception>
    public static Vector2F[][] Decompose(ReadOnlySpan<Vector2F> outline, PolygonDecompositionOptions options = default) => Decompose(outline, null, options);

    /// <summary>Decomposes a simple polygon with holes.</summary>
    /// <param name="outline">The outer boundary, as for the other overload.</param>
    /// <param name="holes">Hole boundaries in either winding, or null; each lies strictly inside the outline and inside no other hole, and no two boundaries touch or cross. Islands inside holes are separate polygons. Together with the outline at most 16384 points.</param>
    /// <param name="options">Piece vertex limit; the default is 8.</param>
    /// <returns>Convex pieces, counter-clockwise, without collinear vertices.</returns>
    /// <exception cref="GeometryToolException">As for the other overload, and a hole lies outside the outline or inside another hole, or two boundaries meet.</exception>
    public static Vector2F[][] Decompose(ReadOnlySpan<Vector2F> outline, IReadOnlyList<Vector2F[]>? holes, PolygonDecompositionOptions options = default)
    {
        int maxVerts = options.MaxVerticesPerPiece == 0 ? 8 : options.MaxVerticesPerPiece;
        if (maxVerts < 3 || maxVerts > 64)
        {
            throw new GeometryToolException(GeometryToolError.InvalidOptions, "MaxVerticesPerPiece = " + MeshInput.Inv(maxVerts) + " must be from 3 to 64", nameof(options));
        }

        var loops = new List<(double X, double Y)[]>();
        var outer = PolygonInput.Prepare(outline, nameof(outline), nameof(outline));
        double area = PolygonInput.SignedArea(outer);
        if (area == 0) { throw new GeometryToolException(GeometryToolError.Degenerate, "outline has zero area", nameof(outline)); }
        if (area < 0) { Array.Reverse(outer); }
        loops.Add(outer);
        int total = outer.Length;
        if (holes is not null)
        {
            for (int h = 0; h < holes.Count; h++)
            {
                if (holes[h] is null) { throw new GeometryToolException(GeometryToolError.Empty, "holes[" + MeshInput.Inv(h) + "] is null", nameof(holes)); }
                var loop = PolygonInput.Prepare(holes[h], "holes[" + MeshInput.Inv(h) + "]", nameof(holes));
                double ha = PolygonInput.SignedArea(loop);
                if (ha == 0) { throw new GeometryToolException(GeometryToolError.Degenerate, "holes[" + MeshInput.Inv(h) + "] has zero area", nameof(holes)); }
                if (ha > 0) { Array.Reverse(loop); }
                total += loop.Length;
                if (total > PolygonInput.MaxVertices)
                {
                    throw new GeometryToolException(GeometryToolError.OutOfRange, "the outline and holes have more than " + MeshInput.Inv(PolygonInput.MaxVertices) + " points together", nameof(holes));
                }

                loops.Add(loop);
            }
        }

        string? problem = PolygonInput.FindIntersection(loops);
        if (problem is not null)
        {
            throw new GeometryToolException(GeometryToolError.SelfIntersecting, "the polygon is not simple: " + problem + " (loop 0 is the outline, loop k is holes[k - 1])", nameof(outline));
        }

        for (int h = 1; h < loops.Count; h++)
        {
            if (!PolygonInput.Inside(loops[h][0], outer))
            {
                throw new GeometryToolException(GeometryToolError.SelfIntersecting, "holes[" + MeshInput.Inv(h - 1) + "] lies outside the outline", nameof(holes));
            }

            for (int g = 1; g < loops.Count; g++)
            {
                if (g != h && PolygonInput.Inside(loops[h][0], loops[g]))
                {
                    throw new GeometryToolException(GeometryToolError.SelfIntersecting, "holes[" + MeshInput.Inv(h - 1) + "] lies inside holes[" + MeshInput.Inv(g - 1) + "]; an island inside a hole is a separate polygon", nameof(holes));
                }
            }
        }

        var pos = new List<(double X, double Y)>();
        var ring = BuildRing(loops, pos, nameof(holes));
        var tris = EarClip(ring, pos, nameof(outline));
        return MergeConvex(tris, pos, maxVerts);
    }

    // Joins every hole to the ring by a bridge: P, M, hole..., M', P' (primes are duplicate vertices at the same place).
    private static List<int> BuildRing(List<(double X, double Y)[]> loops, List<(double X, double Y)> pos, string paramName)
    {
        var ring = new List<int>();
        foreach (var p in loops[0]) { ring.Add(pos.Count); pos.Add(p); }
        var holeVerts = new List<int[]>();
        for (int h = 1; h < loops.Count; h++)
        {
            var idx = new int[loops[h].Length];
            for (int i = 0; i < idx.Length; i++) { idx[i] = pos.Count; pos.Add(loops[h][i]); }
            holeVerts.Add(idx);
        }

        // Rightmost holes first: their bridges are the shortest and see the outline most easily.
        var order = new List<int>();
        for (int h = 0; h < holeVerts.Count; h++) { order.Add(h); }
        double MaxX(int h) { double m = double.NegativeInfinity; foreach (int v in holeVerts[h]) { m = Math.Max(m, pos[v].X); } return m; }
        order.Sort((a, b) => { int c = MaxX(b).CompareTo(MaxX(a)); return c != 0 ? c : a.CompareTo(b); });

        var merged = new bool[holeVerts.Count];
        foreach (int h in order)
        {
            int[] hv = holeVerts[h];
            int mi = 0;
            for (int i = 1; i < hv.Length; i++)
            {
                if (pos[hv[i]].X > pos[hv[mi]].X) { mi = i; }
            }

            var m = pos[hv[mi]];
            var cand = new List<int>();
            for (int r = 0; r < ring.Count; r++) { cand.Add(r); }
            cand.Sort((a, b) =>
            {
                int c = Dist2(pos[ring[a]], m).CompareTo(Dist2(pos[ring[b]], m));
                return c != 0 ? c : a.CompareTo(b);
            });

            int chosen = -1;
            foreach (int r in cand)
            {
                if (InWedge(m, r, ring, pos) && Visible(m, pos[ring[r]], ring, pos, holeVerts, merged)) { chosen = r; break; }
            }

            if (chosen < 0)
            {
                throw new GeometryToolException(GeometryToolError.SelfIntersecting, "holes[" + MeshInput.Inv(h) + "] cannot be joined to the outline: no vertex sees it", paramName);
            }

            int p = ring[chosen];
            var next = new List<int>(ring.Count + hv.Length + 2);
            for (int r = 0; r <= chosen; r++) { next.Add(ring[r]); }
            for (int k = 0; k < hv.Length; k++) { next.Add(hv[(mi + k) % hv.Length]); }
            next.Add(pos.Count); pos.Add(m);
            next.Add(pos.Count); pos.Add(pos[p]);
            for (int r = chosen + 1; r < ring.Count; r++) { next.Add(ring[r]); }
            ring = next;
            merged[h] = true;
        }

        return ring;
    }

    // A bridge may only leave a ring vertex into the polygon's interior, which is on the left of the walk prev -> vertex -> next. Duplicates of one position differ in exactly this.
    private static bool InWedge((double X, double Y) m, int r, List<int> ring, List<(double X, double Y)> pos)
    {
        var prev = pos[ring[(r + ring.Count - 1) % ring.Count]];
        var cur = pos[ring[r]];
        var next = pos[ring[(r + 1) % ring.Count]];
        int turn = PolygonInput.Orient(prev, cur, next);
        int toPrev = PolygonInput.Orient(prev, cur, m), toNext = PolygonInput.Orient(cur, next, m);
        if (turn > 0) { return toPrev > 0 && toNext > 0; }
        if (turn < 0) { return toPrev > 0 || toNext > 0; }
        return toPrev > 0;
    }

    private static double Dist2((double X, double Y) a, (double X, double Y) b) => (a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y);

    private static bool Visible((double X, double Y) m, (double X, double Y) p, List<int> ring, List<(double X, double Y)> pos, List<int[]> holeVerts, bool[] merged)
    {
        for (int k = 0; k < ring.Count; k++)
        {
            if (EdgeBlocks(m, p, pos[ring[k]], pos[ring[(k + 1) % ring.Count]])) { return false; }
        }

        for (int h = 0; h < holeVerts.Count; h++)
        {
            if (merged[h]) { continue; }
            int[] hv = holeVerts[h];
            for (int k = 0; k < hv.Length; k++)
            {
                if (EdgeBlocks(m, p, pos[hv[k]], pos[hv[(k + 1) % hv.Length]])) { return false; }
            }
        }

        var mid = ((m.X + p.X) * 0.5, (m.Y + p.Y) * 0.5);
        var ringPts = new (double X, double Y)[ring.Count];
        for (int k = 0; k < ringPts.Length; k++) { ringPts[k] = pos[ring[k]]; }
        bool inside = PolygonInput.Inside(mid, ringPts);
        for (int h = 0; h < holeVerts.Count; h++)
        {
            if (merged[h]) { continue; }
            var pts = new (double X, double Y)[holeVerts[h].Length];
            for (int k = 0; k < pts.Length; k++) { pts[k] = pos[holeVerts[h][k]]; }
            if (PolygonInput.Inside(mid, pts)) { inside = !inside; }
        }

        return inside;
    }

    // An edge blocks the bridge m-p when they share more than an endpoint of the bridge itself.
    private static bool EdgeBlocks((double X, double Y) m, (double X, double Y) p, (double X, double Y) a, (double X, double Y) b)
    {
        var rel = RecursiveAI.OneMath.Predicates.SegmentPredicates.Classify(m.X, m.Y, p.X, p.Y, a.X, a.Y, b.X, b.Y);
        if (rel == RecursiveAI.OneMath.Predicates.SegmentRelation.Disjoint) { return false; }
        if (rel == RecursiveAI.OneMath.Predicates.SegmentRelation.Touching)
        {
            // Touching only at a bridge endpoint is how the bridge meets the boundary.
            bool aEnd = (a.X == m.X && a.Y == m.Y) || (a.X == p.X && a.Y == p.Y);
            bool bEnd = (b.X == m.X && b.Y == m.Y) || (b.X == p.X && b.Y == p.Y);
            if (aEnd || bEnd)
            {
                // The edge touches the bridge at its own endpoint; any other contact (the bridge passing through a vertex) still blocks.
                var other = aEnd ? b : a;
                int o = PolygonInput.Orient(m, p, other);
                return o == 0; // an edge leaving along the bridge line overlaps or continues it
            }
        }

        return true;
    }

    private static List<(int A, int B, int C)> EarClip(List<int> ring, List<(double X, double Y)> pos, string paramName)
    {
        int n = ring.Count;
        var prev = new int[n];
        var next = new int[n];
        for (int i = 0; i < n; i++) { prev[i] = (i + n - 1) % n; next[i] = (i + 1) % n; }
        var tris = new List<(int, int, int)>();
        int remaining = n;
        int cursor = 0;
        bool strict = true;
        int sinceProgress = 0;
        while (remaining > 3)
        {
            int i = cursor;
            int p = prev[i], q = next[i];
            int o = PolygonInput.Orient(pos[ring[p]], pos[ring[i]], pos[ring[q]]);
            bool clip = false, emit = false;
            if (o == 0) { clip = true; }
            else if (o > 0 && !Blocked(i, p, q, ring, pos, prev, next, remaining, strict)) { clip = true; emit = true; }
            if (clip)
            {
                if (emit) { tris.Add((ring[p], ring[i], ring[q])); }
                next[p] = q;
                prev[q] = p;
                remaining--;
                cursor = q;
                sinceProgress = 0;
                strict = true;
                continue;
            }

            cursor = next[i];
            if (++sinceProgress > remaining)
            {
                if (strict) { strict = false; sinceProgress = 0; continue; }
                throw new GeometryToolException(GeometryToolError.SelfIntersecting, "the polygon could not be triangulated; it is probably not simple", paramName);
            }
        }

        int a = cursor, b = next[a], c = next[b];
        if (PolygonInput.Orient(pos[ring[a]], pos[ring[b]], pos[ring[c]]) > 0) { tris.Add((ring[a], ring[b], ring[c])); }
        return tris;
    }

    private static bool Blocked(int i, int p, int q, List<int> ring, List<(double X, double Y)> pos, int[] prev, int[] next, int remaining, bool strict)
    {
        var a = pos[ring[p]];
        var b = pos[ring[i]];
        var c = pos[ring[q]];
        int w = next[q];
        for (int k = 0; k < remaining - 3; k++, w = next[w])
        {
            var v = pos[ring[w]];
            if ((v.X == a.X && v.Y == a.Y) || (v.X == b.X && v.Y == b.Y) || (v.X == c.X && v.Y == c.Y)) { continue; }
            // Only vertices that are not strictly convex can poke into an ear.
            if (PolygonInput.Orient(pos[ring[prev[w]]], v, pos[ring[next[w]]]) > 0) { continue; }
            int o1 = PolygonInput.Orient(a, b, v), o2 = PolygonInput.Orient(b, c, v), o3 = PolygonInput.Orient(c, a, v);
            bool inside = strict ? (o1 >= 0 && o2 >= 0 && o3 >= 0) : (o1 > 0 && o2 > 0 && o3 > 0);
            if (inside) { return true; }
        }

        return false;
    }

    private static Vector2F[][] MergeConvex(List<(int A, int B, int C)> tris, List<(double X, double Y)> pos, int maxVerts)
    {
        var pieces = new List<List<int>?>(tris.Count);
        var owner = new Dictionary<long, int>();
        static long Key(int a, int b) => ((long)a << 32) | (uint)b;
        for (int t = 0; t < tris.Count; t++)
        {
            var (a, b, c) = tris[t];
            pieces.Add(new List<int> { a, b, c });
            owner[Key(a, b)] = t; owner[Key(b, c)] = t; owner[Key(c, a)] = t;
        }

        for (int t = 0; t < tris.Count; t++)
        {
            var (ta, tb, tc) = tris[t];
            for (int e = 0; e < 3; e++)
            {
                int a = e == 0 ? ta : e == 1 ? tb : tc;
                int b = e == 0 ? tb : e == 1 ? tc : ta;
                if (!owner.TryGetValue(Key(a, b), out int pa) || !owner.TryGetValue(Key(b, a), out int pb) || pa == pb) { continue; }
                var P = pieces[pa]!;
                var Q = pieces[pb]!;
                int i = IndexOfEdge(P, a, b), j = IndexOfEdge(Q, b, a);
                var pr = Rotate(P, (i + 1) % P.Count);   // starts at b, ends at a
                var qr = Rotate(Q, (j + 1) % Q.Count);   // starts at a, ends at b
                var merged = new List<int>(pr);
                for (int k = 1; k < qr.Count - 1; k++) { merged.Add(qr[k]); }
                int m = merged.Count;
                // Only the two junction vertices can have become reflex.
                int ja = pr.Count - 1, jb = 0;
                if (PolygonInput.Orient(pos[merged[ja - 1]], pos[merged[ja]], pos[merged[(ja + 1) % m]]) < 0) { continue; }
                if (PolygonInput.Orient(pos[merged[(jb + m - 1) % m]], pos[merged[jb]], pos[merged[jb + 1]]) < 0) { continue; }
                if (CountCorners(merged, pos) > maxVerts) { continue; }
                owner.Remove(Key(a, b));
                owner.Remove(Key(b, a));
                pieces[pa] = merged;
                pieces[pb] = null;
                for (int k = 0; k < m; k++) { owner[Key(merged[k], merged[(k + 1) % m])] = pa; }
            }
        }

        var result = new List<Vector2F[]>();
        foreach (var piece in pieces)
        {
            if (piece is null) { continue; }
            var pts = new List<Vector2F>();
            for (int k = 0; k < piece.Count; k++)
            {
                var a = pos[piece[(k + piece.Count - 1) % piece.Count]];
                var b = pos[piece[k]];
                var c = pos[piece[(k + 1) % piece.Count]];
                if (PolygonInput.Orient(a, b, c) == 0) { continue; }
                pts.Add(new Vector2F((float)b.X, (float)b.Y));
            }

            if (pts.Count >= 3) { result.Add(pts.ToArray()); }
        }

        return result.ToArray();
    }

    private static int CountCorners(List<int> piece, List<(double X, double Y)> pos)
    {
        int n = 0;
        for (int k = 0; k < piece.Count; k++)
        {
            if (PolygonInput.Orient(pos[piece[(k + piece.Count - 1) % piece.Count]], pos[piece[k]], pos[piece[(k + 1) % piece.Count]]) != 0) { n++; }
        }

        return n;
    }

    private static int IndexOfEdge(List<int> piece, int a, int b)
    {
        for (int k = 0; k < piece.Count; k++)
        {
            if (piece[k] == a && piece[(k + 1) % piece.Count] == b) { return k; }
        }

        throw new InvalidOperationException("edge not in piece");
    }

    private static List<int> Rotate(List<int> l, int start)
    {
        var r = new List<int>(l.Count);
        for (int k = 0; k < l.Count; k++) { r.Add(l[(start + k) % l.Count]); }
        return r;
    }
}
