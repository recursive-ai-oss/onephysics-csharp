using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Broadphase;

/// <summary>Tunables of the broadphase.</summary>
public sealed class BroadphaseOptions
{
    /// <summary>How far dynamic and kinematic proxies' stored boxes extend past their tight boxes on every side, in world units. Default 0.1. A larger margin means fewer tree updates and more candidate pairs.</summary>
    public double Margin { get; init; } = 0.1;

    /// <summary>Margin for static proxies. Default 0: static boxes rarely move and never need slack.</summary>
    public double StaticMargin { get; init; }

    /// <summary>How many steps of motion a moving proxy's box is stretched ahead of it (Catto, GDC 2019). Default 2; 0 disables the prediction.</summary>
    public double DisplacementMultiplier { get; init; } = 2.0;

    internal void Validate()
    {
        CheckRange(Margin, nameof(Margin), 1048576.0);
        CheckRange(StaticMargin, nameof(StaticMargin), 1048576.0);
        CheckRange(DisplacementMultiplier, nameof(DisplacementMultiplier), 64.0);
    }

    private static void CheckRange(double v, string name, double max)
    {
        if (!(v >= 0.0 && v <= max))
        {
            throw new ArgumentOutOfRangeException(name, v, name + " must be in [0, " + max.ToString("R", CultureInfo.InvariantCulture) + "], got " + v.ToString("R", CultureInfo.InvariantCulture) + ".");
        }
    }
}

/// <summary>Which of the three trees a query visits.</summary>
[Flags]
public enum BodyTypeMask
{
    /// <summary>No tree.</summary>
    None = 0,

    /// <summary>The tree of static proxies.</summary>
    Static = 1,

    /// <summary>The tree of kinematic proxies.</summary>
    Kinematic = 2,

    /// <summary>The tree of dynamic proxies.</summary>
    Dynamic = 4,

    /// <summary>All three trees.</summary>
    All = 7,
}

/// <summary>A proxy key packs the proxy's tree (the body type, 2 bits) and its node index in that tree.</summary>
public static class ProxyKey
{
    /// <summary>Packs a tree (0 static, 1 kinematic, 2 dynamic) and a node index.</summary>
    public static int Make(int tree, int node) => (node << 2) | tree;

    /// <summary>The tree a key belongs to: 0 static, 1 kinematic, 2 dynamic.</summary>
    public static int Tree(int key) => key & 3;

    /// <summary>The node index inside the tree.</summary>
    public static int Node(int key) => key >> 2;
}

/// <summary>
/// A pair of proxies whose stored boxes overlap. The pair is oriented so that (UserA, ProxyA) sorts before (UserB, ProxyB), user id first.
/// </summary>
public readonly struct ProxyPair : IEquatable<ProxyPair>
{
    /// <summary>Creates a pair.</summary>
    public ProxyPair(int proxyA, int proxyB, int userA, int userB)
    {
        ProxyA = proxyA;
        ProxyB = proxyB;
        UserA = userA;
        UserB = userB;
    }

    /// <summary>Key of the first proxy.</summary>
    public int ProxyA { get; }

    /// <summary>Key of the second proxy.</summary>
    public int ProxyB { get; }

    /// <summary>User id (for example a shape index) of the first proxy.</summary>
    public int UserA { get; }

    /// <summary>User id of the second proxy.</summary>
    public int UserB { get; }

    /// <summary>True when all four fields are equal.</summary>
    public bool Equals(ProxyPair other) => ProxyA == other.ProxyA && ProxyB == other.ProxyB && UserA == other.UserA && UserB == other.UserB;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is ProxyPair p && Equals(p);

    /// <inheritdoc/>
    public override int GetHashCode() => unchecked((((((ProxyA * 31) + ProxyB) * 31) + UserA) * 31) + UserB);

    /// <summary>Field-wise equality.</summary>
    public static bool operator ==(ProxyPair a, ProxyPair b) => a.Equals(b);

    /// <summary>Field-wise inequality.</summary>
    public static bool operator !=(ProxyPair a, ProxyPair b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() => FormattableString.Invariant($"({UserA}/{ProxyA}, {UserB}/{ProxyB})");
}

/// <summary>
/// Decides whether two overlapping proxies become a pair. This is the seam for layer and group filtering and for skipping pairs of one body's own shapes.
/// It runs on worker threads when the pair search is parallel, so it must be a pure function of its arguments.
/// </summary>
public interface IPairFilter
{
    /// <summary>True to keep the pair of proxies with these user ids.</summary>
    bool ShouldCollide(int userIdA, int userIdB);
}

/// <summary>A filter that keeps every pair.</summary>
public readonly struct AcceptAllPairs : IPairFilter
{
    /// <inheritdoc/>
    public bool ShouldCollide(int userIdA, int userIdB) => true;
}

/// <summary>
/// Three dynamic trees (static, kinematic, dynamic proxies), a move buffer and deterministic pair finding.
/// </summary>
/// <remarks>
/// A proxy is created with its tight bounds; the stored box is the fat box (margin plus motion prediction). <see cref="MoveProxy(int, in TBox, ReadOnlySpan{double})"/> touches the tree only when the new tight box leaves the fat box,
/// and only then is the proxy put in the move buffer. <see cref="UpdatePairs{TFilter}"/> queries the trees for every buffered proxy and returns the overlaps involving at least one of them:
/// a moved dynamic proxy against all three trees, a moved static or kinematic proxy against the dynamic tree only (static-static, static-kinematic and kinematic-kinematic never pair).
/// A pair of two moved proxies is reported once. The returned pairs are sorted by (user id A, user id B, proxy A, proxy B), a total order that does not depend on thread count or traversal order.
/// The caller keeps the set of pairs it already has (for example the contact table) and treats the output as candidates.
/// The whole state, tree structures and the move buffer included, is a snapshot section; see docs/broadphase.md.
/// </remarks>
/// <typeparam name="TBox">The box type: <see cref="Aabb2D"/> or <see cref="Aabb3D"/>.</typeparam>
public class Broadphase<TBox> : ISnapshotSection where TBox : struct, IBox<TBox>
{
    private const int ChunkSize = 64;
    private const int Version = 1;

    private sealed class PairChunk
    {
        public ProxyPair[] Items = new ProxyPair[32];
        public int Count;

        public void Add(in ProxyPair p)
        {
            if (Count == Items.Length) { Array.Resize(ref Items, Items.Length * 2); }
            Items[Count++] = p;
        }
    }

    private readonly BroadphaseOptions _options;
    private DynamicTree<TBox>[] _trees;
    private readonly List<int> _moves = new();
    private readonly List<PairChunk> _chunks = new();
    private int[] _moved = new int[64];
    private int _movedCount;
    private ProxyPair[] _pairs = new ProxyPair[256];
    private int _pairCount;

    /// <summary>Creates an empty broadphase. <paramref name="sectionId"/> is the snapshot section id and must be unique within the world.</summary>
    public Broadphase(BroadphaseOptions? options = null, int sectionId = 0x42503F00)
    {
        _options = options ?? new BroadphaseOptions();
        _options.Validate();
        SectionId = sectionId;
        _trees = new[] { new DynamicTree<TBox>(), new DynamicTree<TBox>(), new DynamicTree<TBox>() };
    }

    /// <inheritdoc/>
    public int SectionId { get; }

    /// <summary>The options this broadphase was created with.</summary>
    public BroadphaseOptions Options => _options;

    /// <summary>The pairs found by the last <see cref="UpdatePairs{TFilter}"/>.</summary>
    public ReadOnlySpan<ProxyPair> Pairs => new(_pairs, 0, _pairCount);

    /// <summary>Number of proxies waiting in the move buffer (stale entries of destroyed proxies may be included until the next update).</summary>
    public int PendingMoves => _moves.Count;

    /// <summary>The tree for a body type. The raw tier may inspect it (debug drawing) or query it directly; changing it directly bypasses the move buffer.</summary>
    public DynamicTree<TBox> GetTree(BodyType type)
    {
        CheckType(type);
        return _trees[(int)type];
    }

    /// <summary>Number of live proxies of a body type.</summary>
    public int GetProxyCount(BodyType type)
    {
        CheckType(type);
        return _trees[(int)type].ProxyCount;
    }

    private static void CheckType(BodyType type)
    {
        if ((uint)type > 2) { throw new ArgumentOutOfRangeException(nameof(type), type, "Body type must be Static, Kinematic or Dynamic."); }
    }

    private bool TryResolve(int key, out DynamicTree<TBox> tree, out int node)
    {
        int t = key & 3;
        node = key >> 2;
        tree = t < 3 ? _trees[t] : _trees[0];
        return key >= 0 && t < 3 && tree.IsProxy(node);
    }

    private DynamicTree<TBox> Resolve(int key, out int node)
    {
        if (!TryResolve(key, out var tree, out node))
        {
            throw new ArgumentException("Proxy key " + key.ToString(CultureInfo.InvariantCulture) + " is not a live proxy (never created or already destroyed).", nameof(key));
        }

        return tree;
    }

    /// <summary>True when the key names a live proxy. Keys are not generation-checked: a destroyed proxy's key may name a later proxy, so the owner of the shape handles must drop it on destruction.</summary>
    public bool IsProxy(int key) => TryResolve(key, out _, out _);

    /// <summary>The fat (stored) bounds of a live proxy.</summary>
    public TBox GetFatBounds(int key)
    {
        var tree = Resolve(key, out int node);
        return tree.GetBounds(node);
    }

    /// <summary>The user id stored with a live proxy.</summary>
    public int GetUserId(int key)
    {
        var tree = Resolve(key, out int node);
        return tree.GetUserId(node);
    }

    /// <summary>The body type a live proxy was created with.</summary>
    public BodyType GetBodyType(int key)
    {
        Resolve(key, out _);
        return (BodyType)(key & 3);
    }

    private double MarginOf(int tree) => tree == 0 ? _options.StaticMargin : _options.Margin;

    /// <summary>Adds a proxy for a body of the given type with its tight bounds and a user id (typically the shape index). The new proxy is placed in the move buffer. Returns its key.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The type is invalid, or the fat box is not finite, inverted or beyond <see cref="BoxLimits.MaxCoordinate"/>.</exception>
    public int CreateProxy(BodyType type, in TBox tight, int userId)
    {
        CheckType(type);
        if (!tight.IsValid(out string reason)) { throw new ArgumentOutOfRangeException(nameof(tight), reason); }
        int t = (int)type;
        TBox fat = tight.Fatten(MarginOf(t), default, 0.0);
        int node = _trees[t].CreateProxy(fat, userId);
        int key = ProxyKey.Make(t, node);
        MarkMoved(_trees[t], node, key);
        return key;
    }

    /// <summary>Removes a proxy. Its key may be reused by a later proxy.</summary>
    public void DestroyProxy(int key)
    {
        var tree = Resolve(key, out int node);
        tree.DestroyProxy(node);
    }

    /// <summary>
    /// Reports that a proxy's tight bounds are now <paramref name="tight"/>. If they still fit inside the stored box nothing changes and false is returned.
    /// Otherwise the proxy gets a new fat box stretched along <paramref name="displacement"/> (the motion since the last report, one value per axis, or empty), is reinserted, and is placed in the move buffer.
    /// </summary>
    public bool MoveProxy(int key, in TBox tight, ReadOnlySpan<double> displacement)
    {
        var tree = Resolve(key, out int node);
        if (!tight.IsValid(out string reason)) { throw new ArgumentOutOfRangeException(nameof(tight), reason); }
        if (displacement.Length != 0 && displacement.Length != TBox.Axes)
        {
            throw new ArgumentException("Displacement must be empty or have " + TBox.Axes.ToString(CultureInfo.InvariantCulture) + " components, got " + displacement.Length.ToString(CultureInfo.InvariantCulture) + ".", nameof(displacement));
        }

        foreach (double d in displacement)
        {
            if (!double.IsFinite(d)) { throw new ArgumentOutOfRangeException(nameof(displacement), d, "Displacement must be finite, got " + d.ToString("R", CultureInfo.InvariantCulture) + "."); }
        }

        if (TBox.Contains(tree.GetBounds(node), tight)) { return false; }
        int t = key & 3;
        TBox fat = tight.Fatten(MarginOf(t), displacement, t == 0 ? 0.0 : _options.DisplacementMultiplier);
        tree.MoveProxy(node, fat);
        MarkMoved(tree, node, key);
        return true;
    }

    /// <summary>Puts a live proxy in the move buffer without changing it (for example after its filter changed), so the next update re-queries its pairs.</summary>
    public void Touch(int key)
    {
        var tree = Resolve(key, out int node);
        MarkMoved(tree, node, key);
    }

    /// <summary>Puts every live proxy in the move buffer; the next update then reports every overlapping pair.</summary>
    public void TouchAll()
    {
        for (int t = 0; t < 3; t++)
        {
            var tree = _trees[t];
            for (int n = 0; n < tree.NodeSlots; n++)
            {
                if (tree.IsProxy(n)) { MarkMoved(tree, n, ProxyKey.Make(t, n)); }
            }
        }
    }

    private void MarkMoved(DynamicTree<TBox> tree, int node, int key)
    {
        int f = tree.GetFlags(node);
        if ((f & DynamicTree<TBox>.MovedFlag) != 0) { return; }
        tree.SetFlags(node, f | DynamicTree<TBox>.MovedFlag);
        _moves.Add(key);
        if (_moves.Count > 64 && _moves.Count > 2 * (_trees[0].ProxyCount + _trees[1].ProxyCount + _trees[2].ProxyCount))
        {
            CollectMoved();
            _moves.Clear();
            for (int i = 0; i < _movedCount; i++) { _moves.Add(_moved[i]); }
        }
    }

    // Fills _moved with the live, still-flagged keys of the move buffer, ascending and without duplicates.
    private void CollectMoved()
    {
        if (_moved.Length < _moves.Count) { _moved = new int[Math.Max(_moves.Count, _moved.Length * 2)]; }
        int n = 0;
        foreach (int key in _moves)
        {
            if (TryResolve(key, out var tree, out int node) && (tree.GetFlags(node) & DynamicTree<TBox>.MovedFlag) != 0) { _moved[n++] = key; }
        }

        Array.Sort(_moved, 0, n);
        int u = 0;
        for (int i = 0; i < n; i++)
        {
            if (u == 0 || _moved[i] != _moved[u - 1]) { _moved[u++] = _moved[i]; }
        }

        _movedCount = u;
    }

    /// <summary>Rebuilds the trees named by <paramref name="trees"/> with binned SAH (see <see cref="DynamicTree{TBox}.Rebuild"/>). Pair results are unaffected; query speed usually improves after heavy churn, and static trees should be rebuilt once after loading a level.</summary>
    public void Rebuild(BodyTypeMask trees = BodyTypeMask.All)
    {
        for (int t = 0; t < 3; t++)
        {
            if (((int)trees & (1 << t)) != 0) { _trees[t].Rebuild(); }
        }
    }

    // ---- pairs ----------------------------------------------------------------------------------------------------

    private struct PairComparer : IComparer<ProxyPair>
    {
        public readonly int Compare(ProxyPair x, ProxyPair y)
        {
            if (x.UserA != y.UserA) { return x.UserA < y.UserA ? -1 : 1; }
            if (x.UserB != y.UserB) { return x.UserB < y.UserB ? -1 : 1; }
            if (x.ProxyA != y.ProxyA) { return x.ProxyA < y.ProxyA ? -1 : 1; }
            if (x.ProxyB != y.ProxyB) { return x.ProxyB < y.ProxyB ? -1 : 1; }
            return 0;
        }
    }

    private struct PairCollector<TFilter> : IOverlapCallback where TFilter : IPairFilter
    {
        public DynamicTree<TBox> Tree;
        public PairChunk Chunk;
        public TFilter Filter;
        public int Tag;
        public int QueryKey;
        public int QueryUser;

        public bool Report(int node, int userId)
        {
            int other = ProxyKey.Make(Tag, node);
            if (other == QueryKey) { return true; }
            // Both moved: only the smaller key reports, so the pair appears once.
            if ((Tree.GetFlags(node) & DynamicTree<TBox>.MovedFlag) != 0 && QueryKey > other) { return true; }
            if (!Filter.ShouldCollide(QueryUser, userId)) { return true; }
            bool queryFirst = QueryUser < userId || (QueryUser == userId && QueryKey < other);
            Chunk.Add(queryFirst ? new ProxyPair(QueryKey, other, QueryUser, userId) : new ProxyPair(other, QueryKey, userId, QueryUser));
            return true;
        }
    }

    private readonly struct PairSearch<TFilter> : IParallelBody where TFilter : IPairFilter
    {
        private readonly Broadphase<TBox> _bp;
        private readonly TFilter _filter;

        public PairSearch(Broadphase<TBox> bp, TFilter filter)
        {
            _bp = bp;
            _filter = filter;
        }

        public void Run(int task)
        {
            PairChunk chunk = _bp._chunks[task];
            chunk.Count = 0;
            int begin = task * ChunkSize;
            int end = Math.Min(begin + ChunkSize, _bp._movedCount);
            var c = new PairCollector<TFilter> { Chunk = chunk, Filter = _filter };
            for (int i = begin; i < end; i++)
            {
                int key = _bp._moved[i];
                int type = key & 3;
                DynamicTree<TBox> own = _bp._trees[type];
                int node = key >> 2;
                TBox box = own.GetBounds(node);
                c.QueryKey = key;
                c.QueryUser = own.GetUserId(node);
                // A dynamic proxy meets every tree; a static or kinematic one meets only the dynamic tree.
                for (int t = type == 2 ? 0 : 2; t < 3; t++)
                {
                    c.Tag = t;
                    c.Tree = _bp._trees[t];
                    c.Tree.Query(box, ref c);
                }
            }
        }
    }

    /// <summary>
    /// Finds the pairs involving every proxy in the move buffer, clears the buffer and returns the sorted pairs (also available as <see cref="Pairs"/> until the next call).
    /// With an <paramref name="executor"/> the buffered proxies are searched in fixed chunks of 64 in parallel and merged in chunk order; the result is identical for any thread count.
    /// </summary>
    /// <param name="filter">Decides which overlaps become pairs; must be pure (it may run on several threads).</param>
    /// <param name="executor">Runs the search; null runs it on the calling thread.</param>
    public ReadOnlySpan<ProxyPair> UpdatePairs<TFilter>(TFilter filter, TaskExecutor? executor = null) where TFilter : IPairFilter
    {
        CollectMoved();
        int chunks = (_movedCount + ChunkSize - 1) / ChunkSize;
        while (_chunks.Count < chunks) { _chunks.Add(new PairChunk()); }
        var body = new PairSearch<TFilter>(this, filter);
        if (executor == null)
        {
            for (int i = 0; i < chunks; i++) { body.Run(i); }
        }
        else
        {
            executor.For(chunks, body);
        }

        int total = 0;
        for (int i = 0; i < chunks; i++) { total += _chunks[i].Count; }
        if (_pairs.Length < total) { _pairs = new ProxyPair[Math.Max(total, _pairs.Length * 2)]; }
        int at = 0;
        for (int i = 0; i < chunks; i++)
        {
            PairChunk ch = _chunks[i];
            Array.Copy(ch.Items, 0, _pairs, at, ch.Count);
            at += ch.Count;
        }

        _pairCount = total;
        _pairs.AsSpan(0, total).Sort(new PairComparer());

        for (int i = 0; i < _movedCount; i++)
        {
            TryResolve(_moved[i], out var tree, out int node);
            tree.SetFlags(node, tree.GetFlags(node) & ~DynamicTree<TBox>.MovedFlag);
        }

        _moves.Clear();
        _movedCount = 0;
        return Pairs;
    }

    /// <summary>Marks every proxy moved and returns all overlapping pairs (see <see cref="TouchAll"/>).</summary>
    public ReadOnlySpan<ProxyPair> FindAllPairs<TFilter>(TFilter filter, TaskExecutor? executor = null) where TFilter : IPairFilter
    {
        TouchAll();
        return UpdatePairs(filter, executor);
    }

    // ---- queries --------------------------------------------------------------------------------------------------

    private struct KeyedOverlap<TCallback> : IOverlapCallback where TCallback : IOverlapCallback
    {
        public TCallback Inner;
        public int Tag;
        public bool Stopped;

        public bool Report(int node, int userId)
        {
            if (Inner.Report(ProxyKey.Make(Tag, node), userId)) { return true; }
            Stopped = true;
            return false;
        }
    }

    private struct KeyedRay<TCallback> : IRayCallback where TCallback : IRayCallback
    {
        public TCallback Inner;
        public int Tag;

        public double Report(int node, int userId, double maxFraction) => Inner.Report(ProxyKey.Make(Tag, node), userId, maxFraction);
    }

    /// <summary>Reports every proxy in the chosen trees whose stored box overlaps <paramref name="box"/>: static tree first, then kinematic, then dynamic, each depth first. The callback receives proxy keys and may stop the query by returning false. Filtering by layer is the callback's job.</summary>
    public void QueryAabb<TCallback>(in TBox box, ref TCallback callback, BodyTypeMask trees = BodyTypeMask.All) where TCallback : IOverlapCallback
    {
        var k = new KeyedOverlap<TCallback> { Inner = callback };
        for (int t = 0; t < 3 && !k.Stopped; t++)
        {
            if (((int)trees & (1 << t)) == 0) { continue; }
            k.Tag = t;
            _trees[t].Query(box, ref k);
        }

        callback = k.Inner;
    }

    /// <summary>
    /// Casts a ray (<see cref="Ray2D"/>, <see cref="Ray3D"/>) or a swept box (<see cref="BoxCast2D"/>, <see cref="BoxCast3D"/>) through the chosen trees, one tree after another, nearest subtrees first inside each.
    /// The callback's return value clips the ray as described on <see cref="IRayCallback"/>. Returns the final max fraction (0 if the callback stopped the cast).
    /// </summary>
    public double RayCast<TRay, TCallback>(in TRay ray, double maxFraction, ref TCallback callback, BodyTypeMask trees = BodyTypeMask.All)
        where TRay : struct, IRay<TBox>
        where TCallback : IRayCallback
    {
        if (double.IsNaN(maxFraction)) { throw new ArgumentOutOfRangeException(nameof(maxFraction), "maxFraction must not be NaN."); }
        var k = new KeyedRay<TCallback> { Inner = callback };
        double f = maxFraction;
        for (int t = 0; t < 3; t++)
        {
            if (((int)trees & (1 << t)) == 0) { continue; }
            k.Tag = t;
            f = _trees[t].RayCast(ray, f, ref k);
            if (f == 0.0) { break; }
        }

        callback = k.Inner;
        return f;
    }

    /// <summary>Checks the invariants of all three trees; throws <see cref="InvalidOperationException"/> naming the first violation.</summary>
    public void Validate()
    {
        foreach (var tree in _trees) { tree.Validate(); }
        foreach (int key in _moves)
        {
            if (key < 0 || (key & 3) > 2) { throw new InvalidOperationException("Move buffer holds the invalid key " + key.ToString(CultureInfo.InvariantCulture) + "."); }
        }
    }

    // ---- snapshot -------------------------------------------------------------------------------------------------

    /// <inheritdoc/>
    public void Write(StateWriter writer)
    {
        ArgumentNullException.ThrowIfNull(writer);
        writer.WriteInt32(Version);
        writer.WriteDouble(_options.Margin);
        writer.WriteDouble(_options.StaticMargin);
        writer.WriteDouble(_options.DisplacementMultiplier);
        foreach (var tree in _trees) { tree.Write(writer); }
        writer.WriteInt32(_moves.Count);
        foreach (int key in _moves) { writer.WriteInt32(key); }
    }

    /// <inheritdoc/>
    public void Read(StateReader reader)
    {
        ArgumentNullException.ThrowIfNull(reader);
        int version = reader.ReadInt32();
        if (version != Version) { throw reader.Error("unsupported broadphase section version " + version.ToString(CultureInfo.InvariantCulture)); }
        if (BitConverter.DoubleToInt64Bits(reader.ReadDouble()) != BitConverter.DoubleToInt64Bits(_options.Margin)
            || BitConverter.DoubleToInt64Bits(reader.ReadDouble()) != BitConverter.DoubleToInt64Bits(_options.StaticMargin)
            || BitConverter.DoubleToInt64Bits(reader.ReadDouble()) != BitConverter.DoubleToInt64Bits(_options.DisplacementMultiplier))
        {
            throw reader.Error("broadphase options in the snapshot differ from this broadphase's options");
        }

        var trees = new[] { new DynamicTree<TBox>(), new DynamicTree<TBox>(), new DynamicTree<TBox>() };
        foreach (var tree in trees) { tree.Read(reader); }
        int count = reader.ReadCount(4, "move buffer");
        var moves = new List<int>(count);
        for (int i = 0; i < count; i++)
        {
            int key = reader.ReadInt32();
            if (key < 0 || (key & 3) > 2 || (key >> 2) >= trees[key & 3].NodeSlots) { throw reader.Error("move buffer entry " + key.ToString(CultureInfo.InvariantCulture) + " names no node slot"); }
            moves.Add(key);
        }

        _trees = trees;
        _moves.Clear();
        _moves.AddRange(moves);
        _pairCount = 0;
        _movedCount = 0;
    }
}
