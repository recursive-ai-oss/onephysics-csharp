using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// Blocks and lane groups of a soft body's constraints. The particles are cut once into compact blocks (recursive bisection of the rest shape); a constraint whose particles all lie in one block is
/// <i>interior</i> to it, any other is a <i>boundary</i> constraint. A block's interior constraints touch no particle of another block, so every block can be solved by one task with no wait between
/// its constraints, and only the boundary constraints need a phase of their own after the blocks (domain decomposition for parallel Gauss–Seidel and position-based solvers: Fratarcangeli, Tibaldo and
/// Pellacini 2016, Vivace; Macklin et al. 2014). Within a block, and within the boundary set, the constraints of a kind are packed greedily, in index order, into groups of four that share no particle,
/// which are solved together in vector lanes with the scalar twin's arithmetic (the order of the groups is the Gauss–Seidel order); constraints that find no partners within the window are solved one by one at the end.
/// Blocks, groups and order come from the mesh alone, so they are the same on any machine, thread count or vector width.
/// </summary>
internal sealed partial class SoftBodyData
{
    /// <summary>Most particles in a block. The blocks of a body are what the solver can run in parallel, so the block size trades parallelism against boundary constraints. A change moves results, so it is a constant.</summary>
    internal const int DefaultBlockSize = 80;

    /// <summary>The block size this body was built with: always <see cref="DefaultBlockSize"/> except in tests that ask their own world for smaller blocks (see <c>Engine3D.SoftBlockSizeForTests</c>).</summary>
    internal int BlockSize = DefaultBlockSize;

    /// <summary>Most blocks of a body: a bigger body gets bigger blocks, because the boundary between blocks grows with their number and the solve is only as parallel as the threads there are.</summary>
    internal const int MaxBlocks = 32;

    /// <summary>How many not yet taken constraints the packer looks at when it fills a group of four.</summary>
    private const int PackWindow = 48;

    // Solver copies of the constraints, grouped by region (a block, or the boundary set) and packed (see Pack). The public raw arrays keep their documented order.
    public int[] SEdgeA = Array.Empty<int>(), SEdgeB = Array.Empty<int>();
    public float[] SEdgeRest = Array.Empty<float>();
    public int[] SBendIdx = Array.Empty<int>();       // 4 per constraint
    public float[] SBendK = Array.Empty<float>();      // 4 per constraint
    public float[] SBendS = Array.Empty<float>(), SBendM0 = Array.Empty<float>();
    public int[] STets = Array.Empty<int>();          // 4 per tetrahedron
    public float[] STetRest = Array.Empty<float>();

    /// <summary>Number of blocks. Region <c>BlockCount</c> is the boundary set.</summary>
    public int BlockCount = 1;
    /// <summary>Block of each particle.</summary>
    public int[] BlockOf = Array.Empty<int>();

    /// <summary>The solver steps of region r (a block, or the boundary set for r = BlockCount) are steps <c>StepStart[r]..StepStart[r+1]-1</c>: the edges, then the bends, then the tetrahedra of the region.
    /// Step s covers stored constraints <c>StepFirst[s]..StepEnd[s]-1</c> of its kind; those before <c>StepQuadEnd[s]</c> are groups of four that share no particle, the rest are solved one by one.</summary>
    public int[] StepStart = new int[3];
    public byte[] StepKind = Array.Empty<byte>();
    public int[] StepFirst = Array.Empty<int>(), StepQuadEnd = Array.Empty<int>(), StepEnd = Array.Empty<int>();

    /// <summary>Whether the pressure constraint is part of a pass: a closed surface with a positive internal pressure (the pressure can be changed through the handle between steps).</summary>
    public bool HasPressure => PressureRest > 0f && Def.InternalPressure > 0f;

    /// <summary>Steps of the boundary set, not counting the pressure constraint.</summary>
    public int BoundarySteps => StepStart[BlockCount + 1] - StepStart[BlockCount];

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
        int ne = EdgeA.Length, nb = BendS.Length, nt = Tets.Length / 4;
        int nr = BlockCount + 1;
        var stamp = new int[N];
        int stampId = 0;

        var idx = new int[2 * ne];
        for (int i = 0; i < ne; i++) { idx[2 * i] = EdgeA[i]; idx[2 * i + 1] = EdgeB[i]; }
        var eo = Pack(ne, 2, idx, stamp, ref stampId, out var eStart, out var eQuad);
        SEdgeA = new int[ne]; SEdgeB = new int[ne]; SEdgeRest = new float[ne];
        for (int i = 0; i < ne; i++) { int s = eo[i]; SEdgeA[i] = EdgeA[s]; SEdgeB[i] = EdgeB[s]; SEdgeRest[i] = EdgeRest[s]; }

        var bo = Pack(nb, 4, BendIdx, stamp, ref stampId, out var bStart, out var bQuad);
        SBendIdx = new int[4 * nb]; SBendK = new float[4 * nb]; SBendS = new float[nb]; SBendM0 = new float[nb];
        for (int i = 0; i < nb; i++)
        {
            int s = bo[i];
            for (int k = 0; k < 4; k++) { SBendIdx[4 * i + k] = BendIdx[4 * s + k]; SBendK[4 * i + k] = BendK[4 * s + k]; }
            SBendS[i] = BendS[s]; SBendM0[i] = BendM0[s];
        }

        // Tetrahedra have no lane kernel; the packing only gives them the block order.
        var to = Pack(nt, 4, Tets, stamp, ref stampId, out var tStart, out var tQuad);
        STets = new int[4 * nt]; STetRest = new float[nt];
        for (int i = 0; i < nt; i++)
        {
            int s = to[i];
            for (int k = 0; k < 4; k++) { STets[4 * i + k] = Tets[4 * s + k]; }
            STetRest[i] = TetRest[s];
        }

        var kindArr = new List<byte>(); var first = new List<int>(); var quad = new List<int>(); var end = new List<int>();
        StepStart = new int[nr + 1];
        for (int r = 0; r < nr; r++)
        {
            StepStart[r] = kindArr.Count;
            Add(0, eStart, eQuad, r, kindArr, first, quad, end);
            Add(1, bStart, bQuad, r, kindArr, first, quad, end);
            Add(2, tStart, tQuad, r, kindArr, first, quad, end);
        }

        StepStart[nr] = kindArr.Count;
        StepKind = kindArr.ToArray(); StepFirst = first.ToArray(); StepQuadEnd = quad.ToArray(); StepEnd = end.ToArray();
    }

    private static void Add(int kind, int[] start, int[] quadEnd, int r, List<byte> kindArr, List<int> first, List<int> quad, List<int> end)
    {
        if (start[r + 1] == start[r]) { return; }
        kindArr.Add((byte)kind); first.Add(start[r]); quad.Add(quadEnd[r]); end.Add(start[r + 1]);
    }

    // Orders the constraints of one kind: by region (the block they are interior to, or the boundary set last), and within a region packed into groups of four that share no particle. The packer takes the
    // first constraint not yet taken, then the next ones in index order (looking at most PackWindow ahead) that share no particle with those already in the group; a group that does not fill up sends its first
    // constraint to the singles, which follow the groups. Returns the stored order (original constraint numbers), the start of each region and the end of its groups.
    private int[] Pack(int n, int m, int[] idx, int[] stamp, ref int stampId, out int[] regionStart, out int[] quadEnd)
    {
        int nr = BlockCount + 1;
        var region = new int[n];
        regionStart = new int[nr + 1];
        quadEnd = new int[nr];
        for (int i = 0; i < n; i++)
        {
            int b = BlockOf[idx[m * i]];
            bool interior = true;
            for (int k = 1; k < m; k++) { if (BlockOf[idx[m * i + k]] != b) { interior = false; break; } }
            region[i] = interior ? b : BlockCount;
            regionStart[region[i] + 1]++;
        }

        for (int r = 0; r < nr; r++) { regionStart[r + 1] += regionStart[r]; }
        var ids = new int[n];
        var fill = new int[nr];
        for (int i = 0; i < n; i++) { ids[regionStart[region[i]] + fill[region[i]]++] = i; }

        var order = new int[n];
        var picked = new bool[n];
        var singles = new List<int>();
        var sel = new int[4];
        for (int r = 0; r < nr; r++)
        {
            int lo = regionStart[r], hi = regionStart[r + 1], head = lo, outPos = lo;
            singles.Clear();
            while (true)
            {
                while (head < hi && picked[head]) { head++; }
                if (head >= hi) { break; }
                stampId++;
                int got = 0, seen = 0;
                for (int i = head; i < hi && seen < PackWindow; i++)
                {
                    if (picked[i]) { continue; }
                    seen++;
                    int c = ids[i];
                    bool free = true;
                    for (int k = 0; k < m; k++) { if (stamp[idx[m * c + k]] == stampId) { free = false; break; } }
                    if (!free) { continue; }
                    for (int k = 0; k < m; k++) { stamp[idx[m * c + k]] = stampId; }
                    sel[got++] = i;
                    if (got == 4) { break; }
                }

                if (got == 4)
                {
                    for (int s = 0; s < 4; s++) { picked[sel[s]] = true; order[outPos++] = ids[sel[s]]; }
                }
                else
                {
                    picked[head] = true; singles.Add(ids[head]);
                }
            }

            quadEnd[r] = outPos;
            foreach (int c in singles) { order[outPos++] = c; }
        }

        return order;
    }

    // Cuts the particles into blocks of at most BlockSize by recursive bisection along the longest axis of the rest positions (median split; ties broken by particle number, so the cut is a pure
    // function of the mesh). Blocks are numbered in the order the recursion meets them.
    private void PartitionParticles()
    {
        int size = Math.Max(Math.Max(1, BlockSize), (N + MaxBlocks - 1) / MaxBlocks);
        BlockOf = new int[N];
        var items = new int[N];
        for (int i = 0; i < N; i++) { items[i] = i; }
        BlockCount = 0;
        // An explicit stack instead of recursion: (start, count).
        var stack = new Stack<(int start, int count)>();
        if (N > 0) { stack.Push((0, N)); }
        // Depth-first with the lower half visited first, so block numbers ascend with the cut order.
        while (stack.Count > 0)
        {
            var (start, count) = stack.Pop();
            if (count <= size)
            {
                for (int i = 0; i < count; i++) { BlockOf[items[start + i]] = BlockCount; }
                BlockCount++;
                continue;
            }

            float minX = float.PositiveInfinity, minY = float.PositiveInfinity, minZ = float.PositiveInfinity;
            float maxX = float.NegativeInfinity, maxY = float.NegativeInfinity, maxZ = float.NegativeInfinity;
            for (int i = 0; i < count; i++)
            {
                int p = items[start + i];
                float x = Rest[3 * p], y = Rest[3 * p + 1], z = Rest[3 * p + 2];
                if (x < minX) { minX = x; }
                if (x > maxX) { maxX = x; }
                if (y < minY) { minY = y; }
                if (y > maxY) { maxY = y; }
                if (z < minZ) { minZ = z; }
                if (z > maxZ) { maxZ = z; }
            }

            float ex = maxX - minX, ey = maxY - minY, ez = maxZ - minZ;
            int axis = ey > ex ? 1 : 0;
            if (ez > (axis == 0 ? ex : ey)) { axis = 2; }
            Array.Sort(items, start, count, new RestOrder(Rest, axis));
            int half = (count + 1) / 2;
            stack.Push((start + half, count - half));
            stack.Push((start, half));
        }

        if (BlockCount == 0) { BlockCount = 1; }
    }

    // Total order on particles: by one rest coordinate, then by number.
    private readonly struct RestOrder : IComparer<int>
    {
        private readonly float[] _rest;
        private readonly int _axis;
        public RestOrder(float[] rest, int axis) { _rest = rest; _axis = axis; }
        public int Compare(int a, int b)
        {
            int c = _rest[3 * a + _axis].CompareTo(_rest[3 * b + _axis]);
            return c != 0 ? c : a.CompareTo(b);
        }
    }
}
