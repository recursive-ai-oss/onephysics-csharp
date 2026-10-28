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

/// <summary>The traversal stack of a ray cast: node indices with the fraction at which the ray enters each node's box, so a node popped later is dropped by one comparison when the ray has been clipped shorter.</summary>
internal ref struct RayStack
{
    private Span<int> _nodes;
    private Span<double> _enters;
    private int _count;

    public RayStack(Span<int> nodes, Span<double> enters)
    {
        _nodes = nodes;
        _enters = enters;
        _count = 0;
    }

    public void Push(int node, double enter)
    {
        if (_count == _nodes.Length)
        {
            var n = new int[_nodes.Length * 2];
            var e = new double[_nodes.Length * 2];
            _nodes.CopyTo(n);
            _enters.CopyTo(e);
            _nodes = n;
            _enters = e;
        }

        _nodes[_count] = node;
        _enters[_count++] = enter;
    }

    public bool TryPop(out int node, out double enter)
    {
        if (_count == 0)
        {
            node = 0;
            enter = 0.0;
            return false;
        }

        --_count;
        node = _nodes[_count];
        enter = _enters[_count];
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
    private const int EnlargedFlag = 2; // internal nodes only: this node's box grew, or a descendant's did, since the last rebuild of the enlarged region
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
    private int _enlarged;  // internal nodes carrying EnlargedFlag

    // Scratch of RebuildEnlarged, kept so a step allocates nothing.
    private int[] _rbItems = new int[16];
    private int[] _rbRegion = new int[16];
    private int[] _rbWalk = new int[64];
    private double[] _rbCentroids = new double[32];
    private BuildItem[] _rbStack = new BuildItem[64];
    private readonly int[] _rbBinCount = new int[BinCount];
    private readonly TBox[] _rbBinBox = new TBox[BinCount];
    private readonly double[] _rbRightArea = new double[BinCount];
    private readonly int[] _rbRightCount = new int[BinCount];

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

    /// <summary>Sizes the scratch lists of the subtree rebuilds for a tree of up to <paramref name="proxies"/> proxies, so rebuilding it never allocates.</summary>
    internal void ReserveScratch(int proxies)
    {
        int nodes = 2 * proxies + 1;
        if (_rbWalk.Length < nodes) { Array.Resize(ref _rbWalk, nodes); }
        if (_rbRegion.Length < nodes) { Array.Resize(ref _rbRegion, nodes); }
        if (_rbItems.Length < nodes) { Array.Resize(ref _rbItems, nodes); }
        if (_rbCentroids.Length < nodes * TBox.Axes) { Array.Resize(ref _rbCentroids, nodes * TBox.Axes); }
        if (_rbStack.Length < nodes + 2) { Array.Resize(ref _rbStack, nodes + 2); }
    }

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

    /// <summary>The stored bounds of a node the caller knows is live (taken from a proxy key that was checked when the proxy was made): no validation, for per-pair inner loops.</summary>
    internal ref readonly TBox BoundsOfLive(int node) => ref _nodes[node].Box;

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
        RebuildEnlarged(); // insertion rotates nodes, which would scatter the enlarged region
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
        RebuildEnlarged();
        RemoveLeaf(proxy);
        Free(proxy);
        _leafCount--;
    }

    /// <summary>Gives a proxy new bounds by removing and reinserting it (the proxy id is unchanged). The caller decides whether the move is worth it; see the broadphase for fattening.</summary>
    public void MoveProxy(int proxy, in TBox box)
    {
        CheckProxy(proxy);
        CheckBox(box);
        RebuildEnlarged();
        RemoveLeaf(proxy);
        _nodes[proxy].Box = box;
        InsertLeaf(proxy);
    }

    /// <summary>
    /// Gives a proxy new bounds without moving it in the tree: the boxes of its ancestors grow to contain the new box and nothing else changes (Catto, Box2D v3 release notes: enlarge instead of reinserting).
    /// The ancestors stay valid but may be looser than necessary; <see cref="EnlargedCount"/> tells how much of the tree is affected and <see cref="RebuildEnlarged"/> restores its quality.
    /// </summary>
    public void EnlargeProxy(int proxy, in TBox box)
    {
        CheckProxy(proxy);
        CheckBox(box);
        _nodes[proxy].Box = box;
        int p = _nodes[proxy].Parent;
        bool growing = true;
        while (p != NullNode)
        {
            ref Node n = ref _nodes[p];
            if (growing)
            {
                if (TBox.Contains(n.Box, box)) { growing = false; }
                else { n.Box = TBox.Union(n.Box, box); }
            }

            // The flag marks every ancestor of a changed node up to the root, so the flagged nodes always form a connected region at the top of the tree.
            if ((n.Flags & EnlargedFlag) != 0) { if (!growing) { break; } }
            else { n.Flags |= EnlargedFlag; _enlarged++; }
            p = n.Parent;
        }
    }

    /// <summary>The number of internal nodes whose boxes have been enlarged, or have an enlarged descendant, since the last <see cref="RebuildEnlarged"/>.</summary>
    public int EnlargedCount => _enlarged;

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
        if ((_nodes[n].Flags & EnlargedFlag) != 0 && _nodes[n].Height > 0) { _enlarged--; }
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
        public int Base; // index in the region list of this item's node (a parallel rebuild numbers the nodes of every subtree up front)
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
        _enlarged = 0;
    }

    /// <summary>
    /// Rebuilds only the part of the tree that <see cref="EnlargeProxy"/> touched: the subtrees below the flagged region stay as they are and become the leaves of a new top, built by binned SAH like <see cref="Rebuild"/>.
    /// The flagged internal nodes are reused, so no node is allocated. Does nothing when nothing is flagged. The result depends only on the tree's current structure and boxes.
    /// </summary>
    /// <param name="executor">When given and the tree is large, independent subtrees of the new top are built on several threads; the tree is the same as the serial build's.</param>
    public void RebuildEnlarged(TaskExecutor? executor = null)
    {
        if (_enlarged == 0) { return; }
        if (_root == NullNode || _nodes[_root].Height == 0 || (_nodes[_root].Flags & EnlargedFlag) == 0) { _enlarged = 0; return; }

        // Collect the flagged region (internal nodes) and the unflagged children hanging below it (the items to rebuild over).
        if (_rbWalk.Length < _used) { _rbWalk = new int[Math.Max(_used, _rbWalk.Length * 2)]; }
        if (_rbRegion.Length < _used) { _rbRegion = new int[Math.Max(_used, _rbRegion.Length * 2)]; }
        if (_rbItems.Length < _used) { _rbItems = new int[Math.Max(_used, _rbItems.Length * 2)]; }
        int walk = 0, regionCount = 0, itemCount = 0;
        _rbWalk[walk++] = _root;
        while (walk > 0)
        {
            int node = _rbWalk[--walk];
            _rbRegion[regionCount++] = node;
            int c1 = _nodes[node].Child1, c2 = _nodes[node].Child2;
            // Child 1 is handled first, so push child 2 first.
            if (_nodes[c2].Height > 0 && (_nodes[c2].Flags & EnlargedFlag) != 0) { _rbWalk[walk++] = c2; } else { _rbItems[itemCount++] = c2; }
            if (_nodes[c1].Height > 0 && (_nodes[c1].Flags & EnlargedFlag) != 0) { _rbWalk[walk++] = c1; } else { _rbItems[itemCount++] = c1; }
        }

        int axes = TBox.Axes;
        if (_rbCentroids.Length < itemCount * axes) { _rbCentroids = new double[Math.Max(itemCount * axes, _rbCentroids.Length * 2)]; }
        for (int i = 0; i < itemCount; i++)
        {
            ref Node it = ref _nodes[_rbItems[i]];
            for (int a = 0; a < axes; a++) { _rbCentroids[(i * axes) + a] = (it.Box.Min(a) + it.Box.Max(a)) * 0.5; }
        }

        for (int i = 0; i < regionCount; i++) { _nodes[_rbRegion[i]].Flags = 0; }
        _root = NullNode;
        if (executor != null && itemCount >= ParallelRebuildItems) { BuildTopParallel(itemCount, regionCount, executor); }
        else
        {
            if (_rbStack.Length < itemCount + 2) { _rbStack = new BuildItem[Math.Max(itemCount + 2, _rbStack.Length * 2)]; }
            BuildSubtree(new BuildItem { Begin = 0, End = itemCount, Parent = NullNode, Side = 0, Base = 0 }, _rbStack);
            // The reused nodes were taken in top-down order, so walking them backwards fits boxes and heights bottom-up.
            for (int i = regionCount - 1; i >= 0; i--) { Refit(_rbRegion[i]); }
        }

        _enlarged = 0;
    }

    /// <summary>
    /// Recomputes the boxes of the part of the tree that <see cref="EnlargeProxy"/> touched from their children, bottom-up, and keeps the structure. Much cheaper than <see cref="RebuildEnlarged"/>:
    /// the enlarged ancestors become as tight as the boxes below them again, but the nodes are not regrouped, so a tree that is only refitted slowly loses quality as proxies move.
    /// Does nothing when nothing is flagged.
    /// </summary>
    public void RefitEnlarged()
    {
        if (_enlarged == 0) { return; }
        if (_root == NullNode || _nodes[_root].Height == 0 || (_nodes[_root].Flags & EnlargedFlag) == 0) { _enlarged = 0; return; }
        if (_rbWalk.Length < _used) { _rbWalk = new int[Math.Max(_used, _rbWalk.Length * 2)]; }
        if (_rbRegion.Length < _used) { _rbRegion = new int[Math.Max(_used, _rbRegion.Length * 2)]; }
        int walk = 0, regionCount = 0;
        _rbWalk[walk++] = _root;
        while (walk > 0)
        {
            int node = _rbWalk[--walk];
            _rbRegion[regionCount++] = node;
            int c1 = _nodes[node].Child1, c2 = _nodes[node].Child2;
            if (_nodes[c2].Height > 0 && (_nodes[c2].Flags & EnlargedFlag) != 0) { _rbWalk[walk++] = c2; }
            if (_nodes[c1].Height > 0 && (_nodes[c1].Flags & EnlargedFlag) != 0) { _rbWalk[walk++] = c1; }
        }

        // The region list is in depth-first order, parents before descendants, so walking it backwards fits children before parents.
        for (int i = regionCount - 1; i >= 0; i--)
        {
            int node = _rbRegion[i];
            _nodes[node].Flags = 0;
            Refit(node);
        }

        _enlarged = 0;
    }

    private const int ParallelRebuildItems = 1024;
    private const int RebuildUnitItems = 256;
    private const int UnitStack = RebuildUnitItems + 2;
    private BuildItem[] _rbUnits = new BuildItem[16];
    private BuildItem[] _rbUnitStacks = new BuildItem[16];
    private int[] _rbTop = new int[16];

    // Builds the subtree over items [Begin, End) of the rebuild list top-down by midpoint splits (depth first, first child first), taking its internal nodes from the region list
    // starting at item.Base. A subtree of k items uses exactly k - 1 of them, so a parent can hand each child its own stretch of the list before either is built.
    private void BuildSubtree(BuildItem first, Span<BuildItem> stack)
    {
        int nextRegion = first.Base;
        int sp = 0;
        stack[sp++] = first;
        while (sp > 0)
        {
            BuildItem item = stack[--sp];
            if (item.End - item.Begin == 1)
            {
                Attach(item, _rbItems[item.Begin]);
                continue;
            }

            int node = _rbRegion[nextRegion++];
            Attach(item, node);
            int mid = SplitAtMidpoint(_rbItems, _rbCentroids, item.Begin, item.End);
            stack[sp++] = new BuildItem { Begin = mid, End = item.End, Parent = node, Side = 1 };
            stack[sp++] = new BuildItem { Begin = item.Begin, End = mid, Parent = node, Side = 0 };
        }
    }

    // The same tree as the serial build: the top of the tree is split here, serially, down to subtrees of at most RebuildUnitItems items; each of those (which owns its stretch of the region list,
    // its items and its centroids) is built and refit by a task; the nodes above are refit last, children before parents.
    private void BuildTopParallel(int itemCount, int regionCount, TaskExecutor executor)
    {
        int unitCount = 0, topCount = 0;
        if (_rbStack.Length < 64) { _rbStack = new BuildItem[64]; }
        var stack = _rbStack;
        int sp = 0;
        stack[sp++] = new BuildItem { Begin = 0, End = itemCount, Parent = NullNode, Side = 0, Base = 0 };
        while (sp > 0)
        {
            BuildItem item = stack[--sp];
            int count = item.End - item.Begin;
            if (count == 1) { Attach(item, _rbItems[item.Begin]); continue; }
            if (count <= RebuildUnitItems)
            {
                if (unitCount == _rbUnits.Length) { Array.Resize(ref _rbUnits, unitCount * 2); }
                _rbUnits[unitCount++] = item;
                continue;
            }

            int node = _rbRegion[item.Base];
            Attach(item, node);
            if (topCount == _rbTop.Length) { Array.Resize(ref _rbTop, topCount * 2); }
            _rbTop[topCount++] = node;
            int mid = SplitAtMidpoint(_rbItems, _rbCentroids, item.Begin, item.End);
            if (sp + 2 > stack.Length) { Array.Resize(ref stack, stack.Length * 2); _rbStack = stack; }
            int leftInternals = mid - item.Begin - 1; // a subtree of k items has k - 1 internal nodes
            stack[sp++] = new BuildItem { Begin = mid, End = item.End, Parent = node, Side = 1, Base = item.Base + 1 + leftInternals };
            stack[sp++] = new BuildItem { Begin = item.Begin, End = mid, Parent = node, Side = 0, Base = item.Base + 1 };
        }

        // Each unit builds with its own slice of the stack scratch (a subtree of at most RebuildUnitItems items needs fewer entries than that).
        if (_rbUnitStacks.Length < unitCount * UnitStack) { _rbUnitStacks = new BuildItem[Math.Max(unitCount * UnitStack, _rbUnitStacks.Length * 2)]; }
        executor.For(unitCount, new UnitBuild(this, unitCount));
        for (int i = topCount - 1; i >= 0; i--) { Refit(_rbTop[i]); }
        _ = regionCount;
    }

    private readonly struct UnitBuild : IParallelBody
    {
        private readonly DynamicTree<TBox> _t;
        private readonly int _units;

        public UnitBuild(DynamicTree<TBox> t, int units) { _t = t; _units = units; }

        public void Run(int task)
        {
            var t = _t;
            BuildItem unit = t._rbUnits[task];
            var scratch = new Span<BuildItem>(t._rbUnitStacks, task * UnitStack, UnitStack);
            t.BuildSubtree(unit, scratch);
            // Children before parents within the subtree: its nodes were taken in top-down order from item.Base.
            int count = unit.End - unit.Begin;
            for (int i = unit.Base + count - 2; i >= unit.Base; i--) { t.Refit(t._rbRegion[i]); }
        }
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

    // Cheap top-down split for the partial rebuild: cut the axis with the widest spread of centroids at the middle of that spread (a spatial median). It costs two passes over the items,
    // and the rebuild runs every step, so it must stay far cheaper than the reinsertions it replaces; the full Rebuild keeps the binned SAH.
    private static int SplitAtMidpoint(int[] items, double[] centroids, int begin, int end)
    {
        int count = end - begin;
        if (count == 2) { return begin + 1; }
        // The bounds of the centroids on every axis, in locals (the axis count is a constant of the box type), then the widest axis; ties go to the lower axis.
        int axes = TBox.Axes;
        double min0 = double.PositiveInfinity, max0 = double.NegativeInfinity, min1 = double.PositiveInfinity, max1 = double.NegativeInfinity, min2 = double.PositiveInfinity, max2 = double.NegativeInfinity;
        for (int i = begin; i < end; i++)
        {
            double c0 = centroids[i * axes], c1 = centroids[(i * axes) + 1];
            if (c0 < min0) { min0 = c0; }
            if (c0 > max0) { max0 = c0; }
            if (c1 < min1) { min1 = c1; }
            if (c1 > max1) { max1 = c1; }
            if (axes > 2)
            {
                double c2 = centroids[(i * axes) + 2];
                if (c2 < min2) { min2 = c2; }
                if (c2 > max2) { max2 = c2; }
            }
        }

        int axis = 0;
        double widest = max0 - min0, lowest = min0;
        if (max1 - min1 > widest) { widest = max1 - min1; axis = 1; lowest = min1; }
        if (axes > 2 && max2 - min2 > widest) { widest = max2 - min2; axis = 2; lowest = min2; }

        if (!(widest > 0.0)) { return begin + (count / 2); }
        double cut = lowest + (widest * 0.5);
        int lo = begin, hi = end - 1;
        while (lo <= hi)
        {
            if (centroids[(lo * axes) + axis] < cut) { lo++; }
            else
            {
                SwapLeaves(items, centroids, axes, lo, hi);
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
        if (_root == NullNode || !TBox.Overlaps(_nodes[_root].Box, box)) { return; }
        var stack = new TraversalStack(stackalloc int[128]);
        stack.Push(_root);
        while (stack.TryPop(out int i))
        {
            // Only nodes whose box overlaps the query are ever pushed, so a miss costs one box test and no push or pop. Child 1 is still visited before child 2.
            ref Node n = ref _nodes[i];
            if (n.Height == 0)
            {
                if (!callback.Report(i, n.UserId)) { return; }
                continue;
            }

            int c1 = n.Child1, c2 = n.Child2;
            if (TBox.Overlaps(_nodes[c2].Box, box)) { stack.Push(c2); }
            if (TBox.Overlaps(_nodes[c1].Box, box)) { stack.Push(c1); }
        }
    }

    /// <summary>
    /// Walks the tree with a ray (or a swept box), nearest subtrees first, calling the callback for each leaf reached. Returns the final max fraction after any clipping.
    /// A callback result of 0 stops the traversal and the return value is 0.
    /// </summary>
    [System.Runtime.CompilerServices.SkipLocalsInit] // the traversal stacks are written before they are read; zeroing 1.5 KB per call showed in query time
    public double RayCast<TRay, TCallback>(in TRay ray, double maxFraction, ref TCallback callback)
        where TRay : struct, IRay<TBox>
        where TCallback : IRayCallback
    {
        if (_root == NullNode || !(maxFraction > 0.0)) { return maxFraction; }
        // A box was accepted by its parent at some fraction limit no shorter than the current one, and its entry fraction does not depend on the limit, so "the ray still reaches it" is enter <= maxFraction.
        if (!ray.Hit(_nodes[_root].Box, maxFraction, out double rootEnter)) { return maxFraction; }
        var stack = new RayStack(stackalloc int[128], stackalloc double[128]);
        stack.Push(_root, rootEnter);
        while (stack.TryPop(out int i, out double enter))
        {
            if (enter > maxFraction) { continue; }
            ref Node n = ref _nodes[i];
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
                if (t2 < t1) { stack.Push(n.Child1, t1); stack.Push(n.Child2, t2); }
                else { stack.Push(n.Child2, t2); stack.Push(n.Child1, t1); }
            }
            else if (h1) { stack.Push(n.Child1, t1); }
            else if (h2) { stack.Push(n.Child2, t2); }
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
        // Scratch is kept on the tree so validating a restored snapshot allocates nothing once warm.
        if (_valSeen.Length < _used) { _valSeen = new bool[Math.Max(_used, _valSeen.Length * 2)]; }
        Array.Clear(_valSeen, 0, _used);
        var seen = _valSeen;
        int leaves = 0;
        if (_root != NullNode)
        {
            if ((uint)_root >= (uint)_used) { throw Bad("root " + _root.ToString(CultureInfo.InvariantCulture) + " is out of range"); }
            if (_nodes[_root].Parent != NullNode) { throw Bad("root has a parent"); }
            if (_valOrder.Length < _used) { _valOrder = new int[Math.Max(_used, _valOrder.Length * 2)]; }
            var order = _valOrder;
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

    private bool[] _valSeen = Array.Empty<bool>();
    private int[] _valOrder = Array.Empty<int>();

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
        ReadStaged(reader);
        CommitStaged();
    }

    private DynamicTree<TBox>? _staged;

    /// <summary>Node slots of the tree read by the last <see cref="ReadStaged"/>.</summary>
    internal int StagedNodeSlots => _staged!._used;

    /// <summary>Reads and validates a snapshot into this tree's staging copy, leaving the tree itself untouched until <see cref="CommitStaged"/>.</summary>
    internal void ReadStaged(StateReader reader)
    {
        ArgumentNullException.ThrowIfNull(reader);
        int used = reader.ReadInt32();
        if (used < 0 || used > MaxNodes || used > reader.Remaining / 24) { throw reader.Error("tree slot count " + used.ToString(CultureInfo.InvariantCulture) + " is out of range"); }
        var tree = _staged ??= new DynamicTree<TBox>(1);
        if (tree._nodes.Length < Math.Max(used, 1)) { tree._nodes = new Node[Math.Max(used, 1)]; }
        tree._used = used;
        tree._root = reader.ReadInt32();
        tree._free = reader.ReadInt32();
        int leaves = 0;
        for (int i = 0; i < used; i++)
        {
            ref Node n = ref tree._nodes[i];
            n = default;
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
        tree._enlarged = 0;
        for (int i = 0; i < used; i++)
        {
            if (tree._nodes[i].Height > 0 && (tree._nodes[i].Flags & EnlargedFlag) != 0) { tree._enlarged++; }
        }
        try
        {
            tree.Validate();
        }
        catch (InvalidOperationException e)
        {
            throw reader.Error(e.Message);
        }
    }

    /// <summary>Makes the tree read by <see cref="ReadStaged"/> the live one; the replaced structure becomes the next staging copy.</summary>
    internal void CommitStaged()
    {
        var t = _staged!;
        (_nodes, t._nodes) = (t._nodes, _nodes);
        (_used, t._used) = (t._used, _used);
        (_root, t._root) = (t._root, _root);
        (_free, t._free) = (t._free, _free);
        (_leafCount, t._leafCount) = (t._leafCount, _leafCount);
        (_enlarged, t._enlarged) = (t._enlarged, _enlarged);
    }
}
