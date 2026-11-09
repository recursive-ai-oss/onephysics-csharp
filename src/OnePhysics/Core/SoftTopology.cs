using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePhysics;

/// <summary>
/// The topology of tearing, shared by the 2D and 3D soft bodies. A tear is a set of broken edges. Where broken edges separate the elements around a vertex (triangles joined across edges, tetrahedra joined across faces),
/// the vertex is split: the elements around it are grouped into connected components (two elements are joined when they share an unbroken edge, or a face with at least one unbroken edge), the component that holds
/// the lowest element keeps the vertex and every other component gets a new vertex, numbered in the order of the vertices they split. Everything is a function of the element list and the broken set, in index order.
/// </summary>
internal static class SoftTopology
{
    /// <summary>Key of an edge: smaller vertex shifted up 20 bits, then the larger.</summary>
    public static long EdgeKey(int a, int b) => a < b ? ((long)a << 20) | (long)b : ((long)b << 20) | (long)a;

    /// <summary>The keys of every edge of every element (3 per triangle, 6 per tetrahedron), sorted ascending, repeats kept.</summary>
    public static long[] ElementEdgeKeys(int[] elems, int per)
    {
        int ne = elems.Length / per;
        int k = per == 3 ? 3 : 6;
        var keys = new long[ne * k];
        int n = 0;
        for (int e = 0; e < ne; e++)
        {
            if (per == 3)
            {
                for (int i = 0; i < 3; i++) { keys[n++] = EdgeKey(elems[3 * e + i], elems[3 * e + (i + 1) % 3]); }
            }
            else
            {
                for (int a = 0; a < 3; a++) { for (int b = a + 1; b < 4; b++) { keys[n++] = EdgeKey(elems[4 * e + a], elems[4 * e + b]); } }
            }
        }

        Array.Sort(keys);
        return keys;
    }

    /// <summary>How many times <paramref name="key"/> occurs in a sorted array.</summary>
    public static int Count(long[] sorted, long key)
    {
        int lo = LowerBound(sorted, key);
        int hi = lo;
        while (hi < sorted.Length && sorted[hi] == key) { hi++; }
        return hi - lo;
    }

    private static int LowerBound(long[] a, long key)
    {
        int lo = 0, hi = a.Length;
        while (lo < hi)
        {
            int mid = (lo + hi) >> 1;
            if (a[mid] < key) { lo = mid + 1; } else { hi = mid; }
        }

        return lo;
    }

    /// <summary>True when the sorted array holds the key.</summary>
    public static bool Has(long[] sorted, long key) => sorted.Length != 0 && LowerBound(sorted, key) < sorted.Length && sorted[LowerBound(sorted, key)] == key;

    /// <summary>Sorted union of two sorted unique key arrays.</summary>
    public static long[] Union(long[] a, long[] b)
    {
        var res = new List<long>(a.Length + b.Length);
        int i = 0, j = 0;
        while (i < a.Length || j < b.Length)
        {
            long v;
            if (j >= b.Length || (i < a.Length && a[i] <= b[j])) { v = a[i++]; } else { v = b[j++]; }
            if (res.Count == 0 || res[res.Count - 1] != v) { res.Add(v); }
        }

        return res.ToArray();
    }

    /// <summary>
    /// Splits the vertices that broken edges have cut loose. <paramref name="elems"/> (triangles when <paramref name="per"/> is 3, tetrahedra when 4) is rewritten to use the new vertices;
    /// <paramref name="source"/> receives, for each new vertex in order (number <paramref name="vertexCount"/> + i), the vertex it was split from. Returns the number of new vertices.
    /// </summary>
    public static int Split(int vertexCount, int[] elems, int per, long[] broken, List<int> source)
    {
        if (broken.Length == 0) { return 0; }
        int ne = elems.Length / per;
        var touched = new bool[vertexCount];
        foreach (long k in broken) { touched[(int)(k >> 20)] = true; touched[(int)(k & 0xFFFFF)] = true; }
        var start = new int[vertexCount + 1];
        for (int i = 0; i < elems.Length; i++) { start[elems[i] + 1]++; }
        for (int v = 0; v < vertexCount; v++) { start[v + 1] += start[v]; }
        var list = new int[elems.Length];
        var fill = new int[vertexCount];
        for (int e = 0; e < ne; e++) { for (int k = 0; k < per; k++) { int v = elems[per * e + k]; list[start[v] + fill[v]++] = e; } }
        int added = 0;
        var parent = new int[16];
        var newId = new int[16];
        for (int v = 0; v < vertexCount; v++)
        {
            if (!touched[v]) { continue; }
            int cnt = start[v + 1] - start[v];
            if (cnt < 2) { continue; }
            if (parent.Length < cnt) { parent = new int[2 * cnt]; newId = new int[2 * cnt]; }
            for (int i = 0; i < cnt; i++) { parent[i] = i; }
            for (int i = 0; i < cnt; i++)
            {
                for (int j = i + 1; j < cnt; j++)
                {
                    if (Joined(elems, per, list[start[v] + i], list[start[v] + j], v, broken))
                    {
                        int ri = Find(parent, i), rj = Find(parent, j);
                        if (ri != rj) { if (ri < rj) { parent[rj] = ri; } else { parent[ri] = rj; } }
                    }
                }
            }

            for (int i = 0; i < cnt; i++)
            {
                if (Find(parent, i) != i) { continue; }
                if (i == 0) { newId[i] = v; continue; }
                newId[i] = vertexCount + added++;
                source.Add(v);
            }

            for (int i = 1; i < cnt; i++)
            {
                int r = Find(parent, i);
                if (r == 0) { continue; }
                int e = list[start[v] + i];
                for (int k = 0; k < per; k++) { if (elems[per * e + k] == v) { elems[per * e + k] = newId[r]; } }
            }
        }

        return added;
    }

    private static int Find(int[] parent, int i)
    {
        while (parent[i] != i) { parent[i] = parent[parent[i]]; i = parent[i]; }
        return i;
    }

    // Whether elements e1 and e2, both around vertex v, are joined: they share an edge through v that is not broken (triangles), or a face through v with an unbroken edge (tetrahedra).
    private static bool Joined(int[] elems, int per, int e1, int e2, int v, long[] broken)
    {
        if (per == 3)
        {
            for (int k = 0; k < 3; k++)
            {
                int u = elems[3 * e1 + k];
                if (u == v) { continue; }
                for (int m = 0; m < 3; m++)
                {
                    if (elems[3 * e2 + m] == u) { return !Has(broken, EdgeKey(v, u)); }
                }
            }

            return false;
        }

        Span<int> common = stackalloc int[4];
        int n = 0;
        for (int k = 0; k < 4; k++)
        {
            int u = elems[4 * e1 + k];
            for (int m = 0; m < 4; m++) { if (elems[4 * e2 + m] == u) { common[n++] = u; break; } }
        }

        if (n != 3) { return false; }
        // A face is cut when all three of its edges are broken.
        return !(Has(broken, EdgeKey(common[0], common[1])) && Has(broken, EdgeKey(common[0], common[2])) && Has(broken, EdgeKey(common[1], common[2])));
    }

    /// <summary>
    /// The broken edges that are still broken after a split: an edge of the new element list is kept when the edge it came from was broken and it has as many elements as that edge had (nothing was separated
    /// along it) or the old edge had one element. An edge whose elements were pulled apart is a free edge again, with its own constraint. <paramref name="src"/> maps every new vertex to its old vertex.
    /// </summary>
    public static long[] RemapBroken(long[] oldBroken, int[] oldElems, int[] newElems, int per, int[] src)
    {
        var oldKeys = ElementEdgeKeys(oldElems, per);
        var newKeys = ElementEdgeKeys(newElems, per);
        var res = new List<long>();
        for (int i = 0; i < newKeys.Length;)
        {
            long k = newKeys[i];
            int j = i;
            while (j < newKeys.Length && newKeys[j] == k) { j++; }
            int a = (int)(k >> 20), b = (int)(k & 0xFFFFF);
            long parent = EdgeKey(src[a], src[b]);
            if (Has(oldBroken, parent))
            {
                int oldCount = Count(oldKeys, parent);
                if (j - i == oldCount || oldCount <= 1) { res.Add(k); }
            }

            i = j;
        }

        return res.ToArray();
    }
}
