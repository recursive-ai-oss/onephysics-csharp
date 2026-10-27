using System;
using System.Globalization;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Broadphase;

/// <summary>Receives the proxies whose box overlaps a query box.</summary>
public interface IOverlapCallback
{
    /// <summary>Called once per overlapping proxy, in a fixed traversal order. Return false to stop the query.</summary>
    /// <param name="proxy">The proxy id (tree) or key (broadphase).</param>
    /// <param name="userId">The user id stored with the proxy.</param>
    bool Report(int proxy, int userId);
}

/// <summary>Receives the proxies a ray or box sweep reaches, and decides how far the traversal goes (Catto, GDC 2019: the callback clips the ray).</summary>
public interface IRayCallback
{
    /// <summary>
    /// Called for each proxy whose box the ray reaches within <paramref name="maxFraction"/>, nearest subtrees first.
    /// Return a negative value to ignore the proxy, 0 to stop the traversal, a value in (0, maxFraction) to clip the ray to that fraction, or any value at or above <paramref name="maxFraction"/> to continue unchanged.
    /// </summary>
    double Report(int proxy, int userId, double maxFraction);
}

/// <summary>A ray, or a box swept along a translation, prepared for repeated tests against boxes.</summary>
/// <typeparam name="TBox">The box type it tests against.</typeparam>
public interface IRay<TBox> where TBox : struct, IBox<TBox>
{
    /// <summary>True when the ray reaches <paramref name="box"/> at a fraction in [0, <paramref name="maxFraction"/>]; <paramref name="enter"/> is the entry fraction (0 when the ray starts inside).</summary>
    bool Hit(in TBox box, double maxFraction, out double enter);
}

/// <summary>A traversal stack that starts in caller-provided (stack) memory and moves to the heap only for very deep trees.</summary>
internal ref struct TraversalStack
{
    private Span<int> _items;
    private int _count;

    public TraversalStack(Span<int> storage)
    {
        _items = storage;
        _count = 0;
    }

    public void Push(int v)
    {
        if (_count == _items.Length)
        {
            var bigger = new int[_items.Length * 2];
            _items.CopyTo(bigger);
            _items = bigger;
        }

        _items[_count++] = v;
    }

    public bool TryPop(out int v)
    {
        if (_count == 0)
        {
            v = 0;
            return false;
        }

        v = _items[--_count];
        return true;
    }
}

/// <summary>
/// A dynamic bounding-volume hierarchy of axis-aligned boxes: a binary tree whose leaves are proxies and whose internal nodes bound their children.
/// Insertion picks the sibling by the surface area heuristic and rotates grandchildren to shrink internal boxes (Catto, GDC 2019 "Dynamic Bounding Volume Hierarchies";
/// Bittner, Hapala and Havran, "Fast insertion-based optimization of bounding volume hierarchies"; Goldsmith and Salmon 1987; Ericson, Real-Time Collision Detection, ch. 6).
/// <see cref="Rebuild"/> rebuilds top-down with binned SAH.
/// </summary>
/// <remarks>
/// The tree stores the boxes it is given; fattening is the broadphase's job. A proxy id is a node index and stays valid until the proxy is destroyed.
/// Node indices come from a free list (most recently freed first, then fresh indices), so the structure is a function of the operation sequence alone,
/// and the whole structure is saved and restored by <see cref="Write"/> and <see cref="Read"/>.
/// Not thread-safe for writes; any number of threads may query while nobody writes.
/// </remarks>
/// <typeparam name="TBox">The box type: <see cref="Aabb2D"/> or <see cref="Aabb3D"/>.</typeparam>
public class DynamicTree<TBox> where TBox : struct, IBox<TBox>
{
    private const int NullNode = -1;
    private const int MaxNodes = 1 << 26;
    internal const int MovedFlag = 1;
    private const int BinCount = 16;

    private struct Node
    {
        public TBox Box;
        public int Parent;   // free node: next free index
        public int Child1;   // -1 for a leaf
        public int Child2;
        public int Height;   // 0 leaf, > 0 internal, -1 free
        public int UserId;
        public int Flags;
    }

    private Node[] _nodes;
    private int _used;      // nodes [0, _used) have been handed out at least once
    private int _free = NullNode;
    private int _root = NullNode;
    private int _leafCount;

    /// <summary>Creates an empty tree with room for <paramref name="initialCapacity"/> nodes (it grows as needed).</summary>
    public DynamicTree(int initialCapacity = 64)
    {
        if (initialCapacity < 1) { initialCapacity = 1; }
        _nodes = new Node[Math.Min(initialCapacity, MaxNodes)];
    }

    /// <summary>Node index of the root, or -1 when the tree is empty. Nodes are inspected with <see cref="IsLeaf"/>, <see cref="GetChildren"/> and <see cref="GetBounds"/>.</summary>
    public int Root => _root;

    /// <summary>Number of proxies (leaves).</summary>
    public int ProxyCount => _leafCount;

    /// <summary>Number of node slots ever handed out; node indices are below this.</summary>
    public int NodeSlots => _used;

    /// <summary>Height of the tree: 0 for a single leaf, -1 when empty.</summary>
    public int Height => _root == NullNode ? -1 : _nodes[_root].Height;

    /// <summary>The sum of the areas of all internal nodes: the quantity SAH insertion and rebuilding try to keep small. Lower means faster queries.</summary>
    public double TotalInternalArea
    {
        get
        {
            double sum = 0;
            for (int i = 0; i < _used; i++)
            {
                if (_nodes[i].Height > 0) { sum += _nodes[i].Box.Area; }
            }

            return sum;
        }
    }

    // ---- node access ----------------------------------------------------------------------------------------------

    /// <summary>True when <paramref name="proxy"/> names a live proxy (a leaf).</summary>
    public bool IsProxy(int proxy) => (uint)proxy < (uint)_used && _nodes[proxy].Height == 0;

    /// <summary>The stored (fat) bounds of a live node. Throws <see cref="ArgumentException"/> for a free or out-of-range index.</summary>
    public TBox GetBounds(int node) => _nodes[CheckNode(node)].Box;

    /// <summary>The user id stored with a proxy. Throws <see cref="ArgumentException"/> unless <paramref name="proxy"/> is a live proxy.</summary>
    public int GetUserId(int proxy) => _nodes[CheckProxy(proxy)].UserId;

    /// <summary>True when the live node is a leaf.</summary>
    public bool IsLeaf(int node) => _nodes[CheckNode(node)].Height == 0;

    /// <summary>The height of a live node: 0 for leaves.</summary>
    public int GetHeight(int node) => _nodes[CheckNode(node)].Height;

    /// <summary>The two children of an internal node, or (-1, -1) for a leaf.</summary>
    public void GetChildren(int node, out int child1, out int child2)
    {
        ref Node n = ref _nodes[CheckNode(node)];
        child1 = n.Child1;
        child2 = n.Child2;
    }

    internal int GetFlags(int proxy) => _nodes[proxy].Flags;

    internal void SetFlags(int proxy, int flags) => _nodes[proxy].Flags = flags;

    private int CheckNode(int node)
    {
        if ((uint)node >= (uint)_used || _nodes[node].Height < 0)
        {
            throw new ArgumentException("Node " + node.ToString(CultureInfo.InvariantCulture) + " is not a live node of this tree.", nameof(node));
        }

        return node;
    }

    private int CheckProxy(int proxy)
    {
        if (!IsProxy(proxy))
        {
            throw new ArgumentException("Proxy " + proxy.ToString(CultureInfo.InvariantCulture) + " is not a live proxy of this tree (never created, destroyed, or an internal node).", nameof(proxy));
        }

        return proxy;
    }

    private static void CheckBox(in TBox box)
    {
        if (!box.IsValid(out string reason))
        {
            throw new ArgumentOutOfRangeException(nameof(box), reason);
        }
    }

    // ---- structure changes ----------------------------------------------------------------------------------------

    /// <summary>Adds a proxy with the given bounds and returns its id. Throws <see cref="ArgumentOutOfRangeException"/> for a box that is not finite, inverted or beyond <see cref="BoxLimits.MaxCoordinate"/>.</summary>
    public int CreateProxy(in TBox box, int userId)
    {
        CheckBox(box);
        int leaf = Alloc();
        ref Node n = ref _nodes[leaf];
        n.Box = box;
        n.UserId = userId;
        n.Height = 0;
        _leafCount++;
        InsertLeaf(leaf);
        return leaf;
    }

    /// <summary>Removes a proxy. Its id may be handed out again by a later <see cref="CreateProxy"/>.</summary>
    public void DestroyProxy(int proxy)
    {
        CheckProxy(proxy);
        RemoveLeaf(proxy);
        Free(proxy);
        _leafCount--;
    }

    /// <summary>Gives a proxy new bounds by removing and reinserting it (the proxy id is unchanged). The caller decides whether the move is worth it; see the broadphase for fattening.</summary>
    public void MoveProxy(int proxy, in TBox box)
    {
        CheckProxy(proxy);
        CheckBox(box);
        RemoveLeaf(proxy);
        _nodes[proxy].Box = box;
        InsertLeaf(proxy);
    }

    private int Alloc()
    {
        int n;
        if (_free != NullNode)
        {
            n = _free;
            _free = _nodes[n].Parent;
        }
        else
        {
            if (_used == _nodes.Length)
            {
                if (_used >= MaxNodes) { throw new InvalidOperationException("The tree is full (" + MaxNodes.ToString(CultureInfo.InvariantCulture) + " nodes)."); }
                Array.Resize(ref _nodes, Math.Min(_nodes.Length * 2, MaxNodes));
            }

            n = _used++;
        }

        _nodes[n] = default;
        _nodes[n].Parent = NullNode;
        _nodes[n].Child1 = NullNode;
        _nodes[n].Child2 = NullNode;
        return n;
    }

    private void Free(int n)
    {
        _nodes[n] = default;
        _nodes[n].Height = -1;
        _nodes[n].Parent = _free;
        _free = n;
    }

    private void Refit(int a)
    {
        ref Node na = ref _nodes[a];
        ref Node c1 = ref _nodes[na.Child1];
        ref Node c2 = ref _nodes[na.Child2];
        na.Box = TBox.Union(c1.Box, c2.Box);
        na.Height = 1 + (c1.Height > c2.Height ? c1.Height : c2.Height);
    }

    // Catto 2019: walk down choosing the child with the lower estimated added area, remembering the best sibling candidate seen;
    // stop once even a zero-cost descent could not beat it. Ties go to child 1.
    private int FindBestSibling(in TBox box)
    {
        double areaBox = box.Area;
        int best = _root;
        double bestCost = double.PositiveInfinity;
        double inherited = 0.0;
        int idx = _root;
        while (true)
        {
            ref Node n = ref _nodes[idx];
            double direct = TBox.Union(n.Box, box).Area;
            double cost = direct + inherited;
            if (cost < bestCost)
            {
                bestCost = cost;
                best = idx;
            }

            if (n.Height == 0) { break; }
            inherited += direct - n.Box.Area;
            if (areaBox + inherited >= bestCost) { break; }

            ref Node c1 = ref _nodes[n.Child1];
            ref Node c2 = ref _nodes[n.Child2];
            double d1 = TBox.Union(c1.Box, box).Area;
            double d2 = TBox.Union(c2.Box, box).Area;
            double e1 = c1.Height == 0 ? d1 + inherited : (d1 - c1.Box.Area) + inherited;
            double e2 = c2.Height == 0 ? d2 + inherited : (d2 - c2.Box.Area) + inherited;
            idx = e2 < e1 ? n.Child2 : n.Child1;
        }

        return best;
    }

    private void InsertLeaf(int leaf)
    {
        if (_root == NullNode)
        {
            _root = leaf;
            _nodes[leaf].Parent = NullNode;
            return;
        }

        int sibling = FindBestSibling(_nodes[leaf].Box);
        int oldParent = _nodes[sibling].Parent;
        int newParent = Alloc();
        ref Node np = ref _nodes[newParent];
        np.Parent = oldParent;
        np.Child1 = sibling;
        np.Child2 = leaf;
        _nodes[sibling].Parent = newParent;
        _nodes[leaf].Parent = newParent;
        if (oldParent == NullNode)
        {
            _root = newParent;
        }
        else if (_nodes[oldParent].Child1 == sibling)
        {
            _nodes[oldParent].Child1 = newParent;
        }
        else
        {
            _nodes[oldParent].Child2 = newParent;
        }

        // Walk back up, rotating to shrink area (Bittner et al.) and refitting boxes and heights.
        int i = newParent;
        while (i != NullNode)
        {
            Refit(i);
            Rotate(i);
            Refit(i);
            i = _nodes[i].Parent;
        }
    }

    // Detaches a leaf (the leaf stays allocated) and refits the ancestors.
    private void RemoveLeaf(int leaf)
    {
        if (leaf == _root)
        {
            _root = NullNode;
            return;
        }

        int parent = _nodes[leaf].Parent;
        int grand = _nodes[parent].Parent;
        int sibling = _nodes[parent].Child1 == leaf ? _nodes[parent].Child2 : _nodes[parent].Child1;
        _nodes[leaf].Parent = NullNode;
        if (grand == NullNode)
        {
            _root = sibling;
            _nodes[sibling].Parent = NullNode;
            Free(parent);
            return;
        }

        if (_nodes[grand].Child1 == parent) { _nodes[grand].Child1 = sibling; } else { _nodes[grand].Child2 = sibling; }
        _nodes[sibling].Parent = grand;
        Free(parent);
        for (int i = grand; i != NullNode; i = _nodes[i].Parent)
        {
            Refit(i);
        }
    }

    // Swaps node x with node y (children of different parents), fixing the links.
    private void SwapNodes(int x, int y)
    {
        int px = _nodes[x].Parent, py = _nodes[y].Parent;
        if (_nodes[px].Child1 == x) { _nodes[px].Child1 = y; } else { _nodes[px].Child2 = y; }
        if (_nodes[py].Child1 == y) { _nodes[py].Child1 = x; } else { _nodes[py].Child2 = x; }
        _nodes[x].Parent = py;
        _nodes[y].Parent = px;
    }

    // Tree rotation (Catto 2019): for node A with children B and C, consider swapping C with a child of B, or B with a child of C,
    // and take the swap that most reduces the area of the demoted internal node. Options are tried in a fixed order; only a strict improvement counts.
    private void Rotate(int a)
    {
        ref Node na = ref _nodes[a];
        if (na.Height < 2) { return; }
        int b = na.Child1, c = na.Child2;
        int bestOption = 0;
        double bestGain = 0.0;
        if (_nodes[b].Height > 0)
        {
            ref Node nb = ref _nodes[b];
            double areaB = nb.Box.Area;
            double g1 = areaB - TBox.Union(_nodes[c].Box, _nodes[nb.Child2].Box).Area;   // swap C with D: B' = {C, E}
            double g2 = areaB - TBox.Union(_nodes[nb.Child1].Box, _nodes[c].Box).Area;   // swap C with E: B' = {D, C}
            if (g1 > bestGain) { bestGain = g1; bestOption = 1; }
            if (g2 > bestGain) { bestGain = g2; bestOption = 2; }
        }

        if (_nodes[c].Height > 0)
        {
            ref Node nc = ref _nodes[c];
            double areaC = nc.Box.Area;
            double g3 = areaC - TBox.Union(_nodes[b].Box, _nodes[nc.Child2].Box).Area;   // swap B with F: C' = {B, G}
            double g4 = areaC - TBox.Union(_nodes[nc.Child1].Box, _nodes[b].Box).Area;   // swap B with G: C' = {F, B}
            if (g3 > bestGain) { bestGain = g3; bestOption = 3; }
            if (g4 > bestGain) { bestOption = 4; }
        }

        switch (bestOption)
        {
            case 1: SwapNodes(c, _nodes[b].Child1); Refit(b); break;
            case 2: SwapNodes(c, _nodes[b].Child2); Refit(b); break;
            case 3: SwapNodes(b, _nodes[c].Child1); Refit(c); break;
            case 4: SwapNodes(b, _nodes[c].Child2); Refit(c); break;
            default: break;
        }
    }

    // ---- rebuild --------------------------------------------------------------------------------------------------

    private struct BuildItem
    {
        public int Begin, End, Parent, Side;
    }

    /// <summary>
    /// Rebuilds the whole tree top-down with binned SAH (16 bins on every axis, the cheapest split wins; a degenerate split falls back to the median).
    /// Proxy ids and user ids are kept. The result depends only on the set of proxies and their boxes, not on the history that produced the tree,
    /// and the free list is reset to ascending order. Allocation-light: scratch arrays are sized by the proxy count.
    /// </summary>
    public void Rebuild()
    {
        if (_root == NullNode) { return; }
        int n = _leafCount;
        var leaves = new int[n];
        int k = 0;
        for (int i = 0; i < _used; i++)
        {
            if (_nodes[i].Height == 0) { leaves[k++] = i; }
        }

        _free = NullNode;
        for (int i = _used - 1; i >= 0; i--)
        {
            if (_nodes[i].Height != 0)
            {
                _nodes[i] = default;
                _nodes[i].Height = -1;
                _nodes[i].Parent = _free;
                _free = i;
            }
        }

        int axes = TBox.Axes;
        var centroids = new double[n * axes];
        for (int i = 0; i < n; i++)
        {
            ref Node leaf = ref _nodes[leaves[i]];
            leaf.Parent = NullNode;
            for (int a = 0; a < axes; a++) { centroids[(i * axes) + a] = (leaf.Box.Min(a) + leaf.Box.Max(a)) * 0.5; }
        }

        // centroids is permuted together with leaves during partitioning.
        var binCount = new int[BinCount];
        var binBox = new TBox[BinCount];
        var rightArea = new double[BinCount];
        var rightCount = new int[BinCount];
        var internals = new int[Math.Max(n - 1, 0)];
        int internalCount = 0;
        var stack = new BuildItem[64];
        int sp = 0;
        stack[sp++] = new BuildItem { Begin = 0, End = n, Parent = NullNode, Side = 0 };
        _root = NullNode;
        var cmin = new double[axes];
        var cmax = new double[axes];
        while (sp > 0)
        {
            BuildItem item = stack[--sp];
            int count = item.End - item.Begin;
            int node;
            if (count == 1)
            {
                node = leaves[item.Begin];
                Attach(item, node);
                continue;
            }

            node = Alloc();
            internals[internalCount++] = node;
            Attach(item, node);

            int mid = ChooseSplit(leaves, centroids, item.Begin, item.End, axes, cmin, cmax, binCount, binBox, rightArea, rightCount);
            if (sp + 2 > stack.Length) { Array.Resize(ref stack, stack.Length * 2); }
            stack[sp++] = new BuildItem { Begin = mid, End = item.End, Parent = node, Side = 1 };
            stack[sp++] = new BuildItem { Begin = item.Begin, End = mid, Parent = node, Side = 0 };
        }

        // Children are allocated after their parents, so walking the internals backwards fits boxes bottom-up.
        for (int i = internalCount - 1; i >= 0; i--) { Refit(internals[i]); }
    }

    private void Attach(in BuildItem item, int node)
    {
        _nodes[node].Parent = item.Parent;
        if (item.Parent == NullNode) { _root = node; }
        else if (item.Side == 0) { _nodes[item.Parent].Child1 = node; }
        else { _nodes[item.Parent].Child2 = node; }
    }

    // Returns the split index in (begin, end) after partitioning leaves/centroids in place. Binned SAH (Wald 2007 style binning): cost = left area * left count + right area * right count.
    private int ChooseSplit(int[] leaves, double[] centroids, int begin, int end, int axes, double[] cmin, double[] cmax, int[] binCount, TBox[] binBox, double[] rightArea, int[] rightCount)
    {
        int count = end - begin;
        if (count == 2) { return begin + 1; }
        for (int a = 0; a < axes; a++) { cmin[a] = double.PositiveInfinity; cmax[a] = double.NegativeInfinity; }
        for (int i = begin; i < end; i++)
        {
            for (int a = 0; a < axes; a++)
            {
                double c = centroids[(i * axes) + a];
                if (c < cmin[a]) { cmin[a] = c; }
                if (c > cmax[a]) { cmax[a] = c; }
            }
        }

        double bestCost = double.PositiveInfinity;
        int bestAxis = -1, bestBin = 0;
        for (int a = 0; a < axes; a++)
        {
            double ext = cmax[a] - cmin[a];
            if (!(ext > 0.0)) { continue; }
            Array.Clear(binCount);
            for (int i = begin; i < end; i++)
            {
                int b = BinOf(centroids[(i * axes) + a], cmin[a], ext);
                TBox lb = _nodes[leaves[i]].Box;
                binBox[b] = binCount[b] == 0 ? lb : TBox.Union(binBox[b], lb);
                binCount[b]++;
            }

            // Suffix sums for the right side.
            double ra = 0.0;
            int rc = 0;
            bool any = false;
            TBox acc = default;
            for (int b = BinCount - 1; b >= 1; b--)
            {
                if (binCount[b] > 0)
                {
                    acc = any ? TBox.Union(acc, binBox[b]) : binBox[b];
                    any = true;
                    rc += binCount[b];
                    ra = acc.Area;
                }

                rightArea[b] = ra;
                rightCount[b] = rc;
            }

            // Prefix sweep: split between bin b and b + 1 puts bins 0..b on the left.
            double la = 0.0;
            int lc = 0;
            any = false;
            for (int b = 0; b < BinCount - 1; b++)
            {
                if (binCount[b] > 0)
                {
                    acc = any ? TBox.Union(acc, binBox[b]) : binBox[b];
                    any = true;
                    lc += binCount[b];
                    la = acc.Area;
                }

                if (lc == 0 || rightCount[b + 1] == 0) { continue; }
                double cost = (la * lc) + (rightArea[b + 1] * rightCount[b + 1]);
                if (cost < bestCost)
                {
                    bestCost = cost;
                    bestAxis = a;
                    bestBin = b;
                }
            }
        }

        if (bestAxis < 0) { return begin + (count / 2); }

        double axisExt = cmax[bestAxis] - cmin[bestAxis];
        int lo = begin, hi = end - 1;
        while (lo <= hi)
        {
            if (BinOf(centroids[(lo * axes) + bestAxis], cmin[bestAxis], axisExt) <= bestBin)
            {
                lo++;
            }
            else
            {
                SwapLeaves(leaves, centroids, axes, lo, hi);
                hi--;
            }
        }

        return lo > begin && lo < end ? lo : begin + (count / 2);
    }

    private static int BinOf(double c, double min, double ext)
    {
        int b = (int)(BinCount * ((c - min) / ext));
        return b >= BinCount ? BinCount - 1 : (b < 0 ? 0 : b);
    }

    private static void SwapLeaves(int[] leaves, double[] centroids, int axes, int i, int j)
    {
        (leaves[i], leaves[j]) = (leaves[j], leaves[i]);
        for (int a = 0; a < axes; a++)
        {
            (centroids[(i * axes) + a], centroids[(j * axes) + a]) = (centroids[(j * axes) + a], centroids[(i * axes) + a]);
        }
    }

    // ---- queries --------------------------------------------------------------------------------------------------

    /// <summary>Reports every proxy whose box overlaps (or touches) <paramref name="box"/>, depth first with child 1 before child 2. The callback may stop the query by returning false.</summary>
    public void Query<TCallback>(in TBox box, ref TCallback callback) where TCallback : IOverlapCallback
    {
        if (_root == NullNode) { return; }
        var stack = new TraversalStack(stackalloc int[128]);
        stack.Push(_root);
        while (stack.TryPop(out int i))
        {
            ref Node n = ref _nodes[i];
            if (!TBox.Overlaps(n.Box, box)) { continue; }
            if (n.Height == 0)
            {
                if (!callback.Report(i, n.UserId)) { return; }
            }
            else
            {
                stack.Push(n.Child2);
                stack.Push(n.Child1);
            }
        }
    }

    /// <summary>
    /// Walks the tree with a ray (or a swept box), nearest subtrees first, calling the callback for each leaf reached. Returns the final max fraction after any clipping.
    /// A callback result of 0 stops the traversal and the return value is 0.
    /// </summary>
    public double RayCast<TRay, TCallback>(in TRay ray, double maxFraction, ref TCallback callback)
        where TRay : struct, IRay<TBox>
        where TCallback : IRayCallback
    {
        if (_root == NullNode || !(maxFraction > 0.0)) { return maxFraction; }
        var stack = new TraversalStack(stackalloc int[128]);
        stack.Push(_root);
        while (stack.TryPop(out int i))
        {
            ref Node n = ref _nodes[i];
            if (!ray.Hit(n.Box, maxFraction, out _)) { continue; }
            if (n.Height == 0)
            {
                double v = callback.Report(i, n.UserId, maxFraction);
                if (v < 0.0) { continue; }
                if (v == 0.0) { return 0.0; }
                if (v < maxFraction) { maxFraction = v; }
                continue;
            }

            bool h1 = ray.Hit(_nodes[n.Child1].Box, maxFraction, out double t1);
            bool h2 = ray.Hit(_nodes[n.Child2].Box, maxFraction, out double t2);
            if (h1 && h2)
            {
                // Farther child first on the stack so the nearer one pops first; equal entry goes to child 1.
                if (t2 < t1) { stack.Push(n.Child1); stack.Push(n.Child2); }
                else { stack.Push(n.Child2); stack.Push(n.Child1); }
            }
            else if (h1) { stack.Push(n.Child1); }
            else if (h2) { stack.Push(n.Child2); }
        }

        return maxFraction;
    }

    // ---- validation -----------------------------------------------------------------------------------------------

    /// <summary>
    /// Checks every structural invariant: parent links, child links, heights, that each internal box contains both children, that each live node is reachable exactly once,
    /// and that the free list holds exactly the free nodes. Throws <see cref="InvalidOperationException"/> naming the first violation.
    /// </summary>
    public void Validate()
    {
        var seen = new bool[_used];
        int leaves = 0;
        if (_root != NullNode)
        {
            if ((uint)_root >= (uint)_used) { throw Bad("root " + _root.ToString(CultureInfo.InvariantCulture) + " is out of range"); }
            if (_nodes[_root].Parent != NullNode) { throw Bad("root has a parent"); }
            var order = new int[_used];
            int top = 0;
            order[top++] = _root;
            int visited = 0;
            while (top > 0)
            {
                int i = order[--top];
                if ((uint)i >= (uint)_used || _nodes[i].Height < 0) { throw Bad("link to free or out-of-range node " + i.ToString(CultureInfo.InvariantCulture)); }
                if (seen[i]) { throw Bad("node " + i.ToString(CultureInfo.InvariantCulture) + " is reachable twice"); }
                seen[i] = true;
                visited++;
                ref Node n = ref _nodes[i];
                if (n.Height == 0)
                {
                    if (n.Child1 != NullNode || n.Child2 != NullNode) { throw Bad("leaf " + i.ToString(CultureInfo.InvariantCulture) + " has children"); }
                    leaves++;
                    continue;
                }

                int c1 = n.Child1, c2 = n.Child2;
                if ((uint)c1 >= (uint)_used || (uint)c2 >= (uint)_used || c1 == c2) { throw Bad("node " + i.ToString(CultureInfo.InvariantCulture) + " has bad children"); }
                if (_nodes[c1].Parent != i || _nodes[c2].Parent != i) { throw Bad("child of node " + i.ToString(CultureInfo.InvariantCulture) + " has the wrong parent"); }
                if (_nodes[c1].Height < 0 || _nodes[c2].Height < 0) { throw Bad("node " + i.ToString(CultureInfo.InvariantCulture) + " links a free node"); }
                int h = 1 + Math.Max(_nodes[c1].Height, _nodes[c2].Height);
                if (n.Height != h) { throw Bad("node " + i.ToString(CultureInfo.InvariantCulture) + " has height " + n.Height.ToString(CultureInfo.InvariantCulture) + ", expected " + h.ToString(CultureInfo.InvariantCulture)); }
                if (!TBox.Contains(n.Box, _nodes[c1].Box) || !TBox.Contains(n.Box, _nodes[c2].Box)) { throw Bad("node " + i.ToString(CultureInfo.InvariantCulture) + " does not contain its children"); }
                if (top + 2 > order.Length) { throw Bad("traversal overflow"); }
                order[top++] = c2;
                order[top++] = c1;
            }

            for (int i = 0; i < _used; i++)
            {
                if (_nodes[i].Height >= 0 && !seen[i]) { throw Bad("live node " + i.ToString(CultureInfo.InvariantCulture) + " is not reachable from the root"); }
            }
        }
        else
        {
            for (int i = 0; i < _used; i++)
            {
                if (_nodes[i].Height >= 0) { throw Bad("empty tree has live node " + i.ToString(CultureInfo.InvariantCulture)); }
            }
        }

        if (leaves != _leafCount) { throw Bad("leaf count " + _leafCount.ToString(CultureInfo.InvariantCulture) + " but " + leaves.ToString(CultureInfo.InvariantCulture) + " leaves are reachable"); }

        int freeCount = 0;
        for (int f = _free; f != NullNode; f = _nodes[f].Parent)
        {
            if ((uint)f >= (uint)_used || _nodes[f].Height != -1) { throw Bad("free list holds node " + f.ToString(CultureInfo.InvariantCulture) + " which is not free"); }
            if (++freeCount > _used) { throw Bad("free list has a cycle"); }
        }

        int freeNodes = 0;
        for (int i = 0; i < _used; i++)
        {
            if (_nodes[i].Height == -1) { freeNodes++; }
        }

        if (freeNodes != freeCount) { throw Bad("free list length " + freeCount.ToString(CultureInfo.InvariantCulture) + " but " + freeNodes.ToString(CultureInfo.InvariantCulture) + " nodes are free"); }
    }

    private static InvalidOperationException Bad(string what) => new("Tree invariant violated: " + what + ".");

    // ---- snapshot -------------------------------------------------------------------------------------------------

    /// <summary>
    /// Writes the exact structure: slot count, root, free-list head, then each slot in index order (height, parent, child 1, child 2, user id, flags, bounds).
    /// Capacity is not written, so equal structures give equal bytes. Restoring reproduces the tree node for node.
    /// </summary>
    public void Write(StateWriter writer)
    {
        ArgumentNullException.ThrowIfNull(writer);
        writer.WriteInt32(_used);
        writer.WriteInt32(_root);
        writer.WriteInt32(_free);
        for (int i = 0; i < _used; i++)
        {
            ref Node n = ref _nodes[i];
            writer.WriteInt32(n.Height);
            writer.WriteInt32(n.Parent);
            writer.WriteInt32(n.Child1);
            writer.WriteInt32(n.Child2);
            writer.WriteInt32(n.UserId);
            writer.WriteInt32(n.Flags);
            if (n.Height >= 0) { n.Box.Write(writer); }
        }
    }

    /// <summary>
    /// Replaces this tree with one read from <paramref name="reader"/>. The data is fully validated (<see cref="Validate"/> plus bounds checks) first;
    /// on failure a <see cref="SnapshotFormatException"/> is thrown and this tree is unchanged.
    /// </summary>
    public void Read(StateReader reader)
    {
        ArgumentNullException.ThrowIfNull(reader);
        int used = reader.ReadInt32();
        if (used < 0 || used > MaxNodes || used > reader.Remaining / 24) { throw reader.Error("tree slot count " + used.ToString(CultureInfo.InvariantCulture) + " is out of range"); }
        var tree = new DynamicTree<TBox>(Math.Max(used, 1))
        {
            _used = used,
            _root = reader.ReadInt32(),
            _free = reader.ReadInt32(),
        };
        int leaves = 0;
        for (int i = 0; i < used; i++)
        {
            ref Node n = ref tree._nodes[i];
            n.Height = reader.ReadInt32();
            n.Parent = reader.ReadInt32();
            n.Child1 = reader.ReadInt32();
            n.Child2 = reader.ReadInt32();
            n.UserId = reader.ReadInt32();
            n.Flags = reader.ReadInt32();
            if (n.Height < -1 || n.Height > used) { throw reader.Error("tree node " + i.ToString(CultureInfo.InvariantCulture) + " has height " + n.Height.ToString(CultureInfo.InvariantCulture)); }
            if (n.Height >= 0)
            {
                n.Box = TBox.Read(reader);
                if (!n.Box.IsValid(out string why)) { throw reader.Error("tree node " + i.ToString(CultureInfo.InvariantCulture) + ": " + why); }
            }
            if (n.Height == 0) { leaves++; }
        }

        tree._leafCount = leaves;
        try
        {
            tree.Validate();
        }
        catch (InvalidOperationException e)
        {
            throw reader.Error(e.Message);
        }

        _nodes = tree._nodes;
        _used = tree._used;
        _root = tree._root;
        _free = tree._free;
        _leafCount = tree._leafCount;
    }
}
