using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>
/// Blocks, regions and shape-matching clusters of a 2D soft body. The particles are cut once into compact blocks (recursive bisection of the rest shape); a constraint whose particles all lie in one block is
/// <i>interior</i> to it, any other is a <i>boundary</i> constraint. A block's interior constraints touch no particle of another block, so every block can be solved by one task with no wait between
/// its constraints, and only the boundary constraints need a phase of their own after the blocks (domain decomposition for parallel Gauss–Seidel and position-based solvers: Fratarcangeli, Tibaldo and
/// Pellacini 2016, Vivace; Macklin et al. 2014). Blocks, regions and order come from the mesh alone, so they are the same on any machine, thread count or vector width.
/// </summary>
internal sealed partial class SoftBodyData
{
    // Solver copies of the constraints, grouped by region (a block, or the boundary set) in the order of the public arrays within a region.
    public int[] SEdgeA = Array.Empty<int>(), SEdgeB = Array.Empty<int>();
    public float[] SEdgeRest = Array.Empty<float>();
    public int[] SBendA = Array.Empty<int>(), SBendB = Array.Empty<int>();
    public float[] SBendRest = Array.Empty<float>();
    public int[] STris = Array.Empty<int>();          // 3 per triangle
    public float[] STriRest2 = Array.Empty<float>();

    /// <summary>Number of blocks. Region <c>BlockCount</c> is the boundary set.</summary>
    public int BlockCount = 1;
    /// <summary>Block of each particle.</summary>
    public int[] BlockOf = Array.Empty<int>();

    /// <summary>Stored edges of region r are <c>EdgeStart[r]..EdgeStart[r+1]-1</c>; likewise bends and triangles.</summary>
    public int[] EdgeStart = new int[3], BendStart = new int[3], TriStart = new int[3];

    /// <summary>Whether the pressure constraint is part of a pass: an outline with a positive internal pressure (the pressure can be changed through the handle between steps).</summary>
    public bool HasPressure => PressureRest > 0f && Def.InternalPressure > 0f;

    /// <summary>Whether the body has network constraints at all.</summary>
    public bool UsesNetwork => Def.Model != SoftBodyModel.ShapeMatching;

    /// <summary>Whether the body has shape-matching clusters.</summary>
    public bool UsesMatching => ClusterCount > 0;

    /// <summary>Whether the boundary phase of a pass has anything to do (boundary constraints, pressure, clusters).</summary>
    public bool HasBoundaryWork => (UsesNetwork && (EdgeStart[BlockCount + 1] > EdgeStart[BlockCount] || BendStart[BlockCount + 1] > BendStart[BlockCount] || TriStart[BlockCount + 1] > TriStart[BlockCount])) || (UsesNetwork && HasPressure) || UsesMatching;

    /// <summary>Whether block r has interior network constraints.</summary>
    public bool BlockHasWork(int r) => UsesNetwork && (EdgeStart[r + 1] > EdgeStart[r] || BendStart[r + 1] > BendStart[r] || TriStart[r + 1] > TriStart[r]);

    /// <summary>Particles that share an edge or a bend hinge, per particle (compressed rows: <c>NbrList[NbrStart[p]..NbrStart[p+1]-1]</c>), for the pair filter.</summary>
    public int[] NbrStart = new int[1], NbrList = Array.Empty<int>();

    private void BuildNeighbours()
    {
        var start = new int[N + 1];
        foreach (long k in EdgeKey) { start[(int)(k >> 20) + 1]++; start[(int)(k & 0xFFFFF) + 1]++; }
        foreach (long k in HingeKey) { start[(int)(k >> 20) + 1]++; start[(int)(k & 0xFFFFF) + 1]++; }
        for (int i = 0; i < N; i++) { start[i + 1] += start[i]; }
        var list = new int[start[N]];
        var fill = new int[N];
        foreach (long k in EdgeKey) { Link(list, start, fill, (int)(k >> 20), (int)(k & 0xFFFFF)); }
        foreach (long k in HingeKey) { Link(list, start, fill, (int)(k >> 20), (int)(k & 0xFFFFF)); }
        NbrStart = start; NbrList = list;
    }

    private static void Link(int[] list, int[] start, int[] fill, int a, int b)
    {
        list[start[a] + fill[a]++] = b;
        list[start[b] + fill[b]++] = a;
    }

    private void ColourConstraints()
    {
        BuildNeighbours();
        PartitionParticles();
        int ne = EdgeA.Length, nb = BendA.Length, nt = Tris.Length / 3;

        var idx = new int[2 * ne];
        for (int i = 0; i < ne; i++) { idx[2 * i] = EdgeA[i]; idx[2 * i + 1] = EdgeB[i]; }
        var eo = Order(ne, 2, idx, out EdgeStart);
        SEdgeA = new int[ne]; SEdgeB = new int[ne]; SEdgeRest = new float[ne];
        for (int i = 0; i < ne; i++) { int s = eo[i]; SEdgeA[i] = EdgeA[s]; SEdgeB[i] = EdgeB[s]; SEdgeRest[i] = EdgeRest[s]; }

        idx = new int[2 * nb];
        for (int i = 0; i < nb; i++) { idx[2 * i] = BendA[i]; idx[2 * i + 1] = BendB[i]; }
        var bo = Order(nb, 2, idx, out BendStart);
        SBendA = new int[nb]; SBendB = new int[nb]; SBendRest = new float[nb];
        for (int i = 0; i < nb; i++) { int s = bo[i]; SBendA[i] = BendA[s]; SBendB[i] = BendB[s]; SBendRest[i] = BendRest[s]; }

        var to = Order(nt, 3, Tris, out TriStart);
        STris = new int[3 * nt]; STriRest2 = new float[nt];
        for (int i = 0; i < nt; i++)
        {
            int s = to[i];
            for (int k = 0; k < 3; k++) { STris[3 * i + k] = Tris[3 * s + k]; }
            STriRest2[i] = TriRest2[s];
        }
    }

    // Orders the constraints of one kind by region: the block they are interior to, or the boundary set last; within a region the original order is kept. Returns the stored order and the start of each region.
    private int[] Order(int n, int m, int[] idx, out int[] regionStart)
    {
        int nr = BlockCount + 1;
        var region = new int[n];
        regionStart = new int[nr + 1];
        for (int i = 0; i < n; i++)
        {
            int b = BlockOf[idx[m * i]];
            bool interior = true;
            for (int k = 1; k < m; k++) { if (BlockOf[idx[m * i + k]] != b) { interior = false; break; } }
            region[i] = interior ? b : BlockCount;
            regionStart[region[i] + 1]++;
        }

        for (int r = 0; r < nr; r++) { regionStart[r + 1] += regionStart[r]; }
        var order = new int[n];
        var fill = new int[nr];
        for (int i = 0; i < n; i++) { order[regionStart[region[i]] + fill[region[i]]++] = i; }
        return order;
    }

    private void PartitionParticles()
    {
        int size = Math.Max(Math.Max(1, BlockSize), (N + MaxBlocks - 1) / MaxBlocks);
        BlockOf = new int[N];
        Bisect(N, Rest, size, BlockOf, out BlockCount);
        if (BlockCount == 0) { BlockCount = 1; }
    }

    // Cuts the particles into cells of at most `size` by recursive bisection along the longer axis of the rest positions (median split; ties broken by particle number, so the cut is a pure function of the mesh).
    // Cells are numbered in the order the recursion meets them, the lower half first. An explicit stack stands in for recursion.
    internal static void Bisect(int n, float[] rest, int size, int[] cellOf, out int cells)
    {
        var items = new int[n];
        for (int i = 0; i < n; i++) { items[i] = i; }
        cells = 0;
        var stack = new Stack<(int start, int count)>();
        if (n > 0) { stack.Push((0, n)); }
        while (stack.Count > 0)
        {
            var (start, count) = stack.Pop();
            if (count <= size)
            {
                for (int i = 0; i < count; i++) { cellOf[items[start + i]] = cells; }
                cells++;
                continue;
            }

            float minX = float.PositiveInfinity, minY = float.PositiveInfinity, maxX = float.NegativeInfinity, maxY = float.NegativeInfinity;
            for (int i = 0; i < count; i++)
            {
                int p = items[start + i];
                float x = rest[2 * p], y = rest[2 * p + 1];
                if (x < minX) { minX = x; }
                if (x > maxX) { maxX = x; }
                if (y < minY) { minY = y; }
                if (y > maxY) { maxY = y; }
            }

            int axis = maxY - minY > maxX - minX ? 1 : 0;
            Array.Sort(items, start, count, new RestOrder(rest, axis));
            int half = (count + 1) / 2;
            stack.Push((start + half, count - half));
            stack.Push((start, half));
        }
    }

    // Total order on particles: by one rest coordinate, then by number.
    private readonly struct RestOrder : IComparer<int>
    {
        private readonly float[] _rest;
        private readonly int _axis;
        public RestOrder(float[] rest, int axis) { _rest = rest; _axis = axis; }
        public int Compare(int a, int b)
        {
            int c = _rest[2 * a + _axis].CompareTo(_rest[2 * b + _axis]);
            return c != 0 ? c : a.CompareTo(b);
        }
    }

    // ------------------------------------------------------------------ shape-matching clusters

    /// <summary>Number of clusters (0 when the body does not use shape matching).</summary>
    public int ClusterCount;
    /// <summary>Members of cluster c are <c>ClusterMember[ClusterStart[c]..ClusterStart[c+1]-1]</c>, in ascending particle order; per member: rest offset from the cluster's rest centroid (<c>ClusterQ</c>, 2 floats) and mass.</summary>
    public int[] ClusterStart = new int[1], ClusterMember = Array.Empty<int>();
    public float[] ClusterQ = Array.Empty<float>(), ClusterMass = Array.Empty<float>();
    public float[] ClusterTotalMass = Array.Empty<float>();
    /// <summary>Per cluster: the length that scales rest offsets in the quadratic fit, and the inverse of the 5 x 5 rest moment matrix (25 floats), or a zero length when the cluster has no quadratic fit.</summary>
    public float[] ClusterScale = Array.Empty<float>(), ClusterAinv = Array.Empty<float>();
    /// <summary>Scratch of the current substep: the accumulated position-like multiplier of every membership, 2 floats each.</summary>
    public float[] MatchMu = Array.Empty<float>();

    private void BuildClusters()
    {
        ClusterCount = 0;
        if (Def.Model == SoftBodyModel.Network) { return; }
        int size = Def.ClusterSize > 0 ? Def.ClusterSize : (N <= 64 ? N : 32);
        size = Math.Max(size, (N + MaxClusters - 1) / MaxClusters);
        var cellOf = new int[N];
        Bisect(N, Rest, size, cellOf, out int cells);
        var counts = new int[cells];
        var minX = new float[cells]; var minY = new float[cells]; var maxX = new float[cells]; var maxY = new float[cells];
        for (int c = 0; c < cells; c++) { minX[c] = float.PositiveInfinity; minY[c] = float.PositiveInfinity; maxX[c] = float.NegativeInfinity; maxY[c] = float.NegativeInfinity; }
        for (int i = 0; i < N; i++)
        {
            int c = cellOf[i];
            minX[c] = Math.Min(minX[c], Rest[2 * i]); maxX[c] = Math.Max(maxX[c], Rest[2 * i]);
            minY[c] = Math.Min(minY[c], Rest[2 * i + 1]); maxY[c] = Math.Max(maxY[c], Rest[2 * i + 1]);
        }

        // Each cell grows by a margin of a quarter of its larger side, so that neighbouring clusters share particles and the body does not break into independent rigid pieces.
        var members = new List<int>(N + N / 2);
        var start = new List<int> { 0 };
        for (int c = 0; c < cells; c++)
        {
            float margin = cells == 1 ? 0f : 0.25f * Math.Max(maxX[c] - minX[c], maxY[c] - minY[c]);
            for (int i = 0; i < N; i++)
            {
                bool inside = cells == 1 || cellOf[i] == c
                    || (Rest[2 * i] >= minX[c] - margin && Rest[2 * i] <= maxX[c] + margin && Rest[2 * i + 1] >= minY[c] - margin && Rest[2 * i + 1] <= maxY[c] + margin);
                if (inside) { members.Add(i); }
            }

            start.Add(members.Count);
        }

        ClusterCount = cells;
        ClusterStart = start.ToArray(); ClusterMember = members.ToArray();
        int total = ClusterMember.Length;
        ClusterQ = new float[2 * total]; ClusterMass = new float[total];
        ClusterTotalMass = new float[cells]; ClusterScale = new float[cells]; ClusterAinv = new float[25 * cells];
        MatchMu = new float[2 * total];
        var inv = new double[25];
        var qt = new double[5];
        for (int c = 0; c < cells; c++)
        {
            int m0 = ClusterStart[c], m1 = ClusterStart[c + 1];
            double mass = 0.0, cx = 0.0, cy = 0.0;
            for (int m = m0; m < m1; m++)
            {
                int i = ClusterMember[m];
                double mi = 1.0 / InvMass0[i];
                ClusterMass[m] = (float)mi;
                mass += mi; cx += mi * Rest[2 * i]; cy += mi * Rest[2 * i + 1];
            }

            cx /= mass; cy /= mass;
            ClusterTotalMass[c] = (float)mass;
            double reach = 0.0;
            for (int m = m0; m < m1; m++)
            {
                int i = ClusterMember[m];
                float qx = (float)(Rest[2 * i] - cx), qy = (float)(Rest[2 * i + 1] - cy);
                ClusterQ[2 * m] = qx; ClusterQ[2 * m + 1] = qy;
                reach = Math.Max(reach, Math.Max(Math.Abs(qx), Math.Abs(qy)));
            }

            if (!Def.MatchQuadratic || !(reach > 0.0)) { continue; }
            // The 5 x 5 moment matrix of the quadratic terms (qx, qy, qx^2, qy^2, qx qy), offsets scaled by the cluster's reach so it is well conditioned.
            var a = new double[25];
            for (int m = m0; m < m1; m++)
            {
                double x = ClusterQ[2 * m] / reach, y = ClusterQ[2 * m + 1] / reach, mi = ClusterMass[m];
                qt[0] = x; qt[1] = y; qt[2] = x * x; qt[3] = y * y; qt[4] = x * y;
                for (int r = 0; r < 5; r++) { for (int k = 0; k < 5; k++) { a[5 * r + k] += mi * qt[r] * qt[k]; } }
            }

            if (!Invert5(a, inv)) { continue; }
            ClusterScale[c] = (float)reach;
            for (int k = 0; k < 25; k++) { ClusterAinv[25 * c + k] = (float)inv[k]; }
        }
    }

    // Gauss-Jordan inverse of a 5 x 5 matrix with partial pivoting, in a fixed order. False when a pivot is too small relative to the matrix's largest entry (the cluster's rest shape has no quadratic fit).
    private static bool Invert5(double[] a, double[] inv)
    {
        var m = new double[50];
        double big = 0.0;
        for (int r = 0; r < 5; r++)
        {
            for (int k = 0; k < 5; k++) { m[10 * r + k] = a[5 * r + k]; big = Math.Max(big, Math.Abs(a[5 * r + k])); }
            m[10 * r + 5 + r] = 1.0;
        }

        for (int col = 0; col < 5; col++)
        {
            int piv = col;
            for (int r = col + 1; r < 5; r++) { if (Math.Abs(m[10 * r + col]) > Math.Abs(m[10 * piv + col])) { piv = r; } }
            if (!(Math.Abs(m[10 * piv + col]) > 1e-9 * big)) { return false; }
            if (piv != col) { for (int k = 0; k < 10; k++) { (m[10 * col + k], m[10 * piv + k]) = (m[10 * piv + k], m[10 * col + k]); } }
            double d = 1.0 / m[10 * col + col];
            for (int k = 0; k < 10; k++) { m[10 * col + k] *= d; }
            for (int r = 0; r < 5; r++)
            {
                if (r == col) { continue; }
                double f = m[10 * r + col];
                if (f == 0.0) { continue; }
                for (int k = 0; k < 10; k++) { m[10 * r + k] -= f * m[10 * col + k]; }
            }
        }

        for (int r = 0; r < 5; r++) { for (int k = 0; k < 5; k++) { inv[5 * r + k] = m[10 * r + 5 + k]; } }
        return true;
    }
}
