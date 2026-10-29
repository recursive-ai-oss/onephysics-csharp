using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>
/// Persistent simulation islands: sets of dynamic bodies connected through contacts and joints, which sleep and wake together.
/// Islands only merge when a constraint is added between two of them and only split when a sleeping candidate with removed constraints is checked
/// (Catto, "Simulation Islands", Box2D v3 blog): removing a constraint merely counts, and the possibly disconnected island is split by a depth-first search
/// only when it is about to fall asleep, at most once per step.
/// </summary>
/// <remarks>
/// Determinism: a merge keeps the lower island id and merges the two ascending body lists; a split labels components by their lowest body index, the first keeps the island id
/// and the rest take the lowest free ids in that order. Each island's body list is always ascending. The saved state is only the island table (id liveness, sleep flag, removal count) and each body's island id;
/// the lists are rebuilt in ascending body order on restore, so they never depend on history.
/// </remarks>
[Experimental("OPHYS0001")]
public sealed class IslandSet
{
    private HandleTable _ids = new();
    private int[][] _bodies = new int[16][];
    private int[] _count = new int[16];
    private bool[] _asleep = new bool[16];
    private int[] _removed = new int[16];
    private bool[] _scratchVisited = new bool[64];
    private int[] _snapshot = new int[16];

    // Body lists are pooled by size class (a list of length L is in class floor(log2 L)). An island holds one list; when it outgrows it, the bigger list comes from the pool and the old one goes
    // back, and when a split or a merge leaves an island with a list much bigger than it needs, the list is traded for a smaller one. Reserve stocks every class for the worst case, so a
    // world within its reserved body count allocates no list however islands merge and split (a class-c list is held only by an island of more than c/2 bodies, so at most 2N/c of them exist).
    // Contents are never read from a pooled list.
    private readonly List<int[]>[] _pool = new List<int[]>[31];
    private const int MinList = 4;

    private static int CeilLog2(int n) => n <= 1 ? 0 : System.Numerics.BitOperations.Log2((uint)(n - 1)) + 1;
    private static int FloorLog2(int n) => System.Numerics.BitOperations.Log2((uint)n);

    private void Release(int[] list)
    {
        int k = FloorLog2(list.Length);
        (_pool[k] ??= new List<int[]>()).Add(list);
    }

    // The smallest pooled list of at least `need` ints, or null.
    private int[]? TryTake(int need)
    {
        for (int k = CeilLog2(Math.Max(need, MinList)); k < _pool.Length; k++)
        {
            var stack = _pool[k];
            if (stack != null && stack.Count > 0)
            {
                int[] taken = stack[stack.Count - 1];
                stack.RemoveAt(stack.Count - 1);
                return taken;
            }
        }

        return null;
    }

    private int[] Take(int need) => TryTake(need) ?? new int[1 << CeilLog2(Math.Max(need, MinList))];

    private int[] Grow(int[] old, int used, int need)
    {
        int[] grown = Take(need);
        Array.Copy(old, grown, used);
        Release(old);
        return grown;
    }

    // A list for an island read from a snapshot. Contents are not copied.
    private int[] Swap(int[]? old, int need)
    {
        int[] taken = Take(need);
        if (old != null) { Release(old); }
        return taken;
    }

    // Trades a list that is at least twice as long as the island needs for a smaller pooled one, when the pool has one.
    private void RightSize(int island)
    {
        int[] list = _bodies[island];
        int n = _count[island];
        if (list.Length < 2 * Math.Max(n, MinList)) { return; }
        int[]? smaller = TryTake(n);
        if (smaller == null) { return; }
        if (smaller.Length >= list.Length) { Release(smaller); return; }
        Array.Copy(list, smaller, n);
        _bodies[island] = smaller;
        Release(list);
    }

    private readonly List<int> _stack = new();
    private readonly List<int> _component = new();

    /// <summary>Number of live islands.</summary>
    public int Count => _ids.Count;

    /// <summary>One more than the highest island id ever used.</summary>
    public int HighWater => _ids.HighWater;

    /// <summary>True when the id is a live island.</summary>
    public bool IsLive(int island) => _ids.IsLive(island);

    /// <summary>True when the island is asleep.</summary>
    public bool IsAsleep(int island) => _asleep[island];

    /// <summary>Constraints removed from the island since it was last split or created.</summary>
    public int RemovedConstraints(int island) => _removed[island];

    /// <summary>The island's bodies, ascending.</summary>
    public ReadOnlySpan<int> Bodies(int island) => _bodies[island].AsSpan(0, _count[island]);

    internal void SetAsleep(int island, bool asleep) => _asleep[island] = asleep;

    /// <summary>Sizes the tables for up to <paramref name="islands"/> islands and <paramref name="bodies"/> bodies, and keeps a few body lists big enough for the biggest island in reserve.</summary>
    internal void Reserve(int islands, int bodies)
    {
        _ids.Reserve(islands);
        _stack.EnsureCapacity(bodies);
        _component.EnsureCapacity(bodies);
        if (_bodies.Length < islands)
        {
            Array.Resize(ref _bodies, islands); Array.Resize(ref _count, islands); Array.Resize(ref _asleep, islands); Array.Resize(ref _removed, islands);
        }

        if (_countsScratch.Length < islands) { Array.Resize(ref _countsScratch, islands); }
        if (_scratchVisited.Length < bodies) { Array.Resize(ref _scratchVisited, bodies); }
        if (_snapshot.Length < bodies) { Array.Resize(ref _snapshot, bodies); }
        StockPool(bodies);
    }

    // Brings every size class up to the worst case for `bodies` dynamic bodies: N+2 lists for the smallest class (every body its own island) and 2N/c+2 of class c above it.
    private void StockPool(int bodies)
    {
        int n = Math.Max(bodies, MinList);
        int top = CeilLog2(n);
        for (int k = FloorLog2(MinList); k <= top; k++)
        {
            int size = 1 << k;
            int want = k == FloorLog2(MinList) ? n + 2 : (int)(((2L * n) + size - 1) / size) + 2;
            int have = 0;
            for (int i = 0; i < _bodies.Length; i++) { if (_bodies[i] != null && _bodies[i].Length >= size) { have++; } }
            for (int j = k; j < _pool.Length; j++) { if (_pool[j] != null) { have += _pool[j].Count; } }
            var stack = _pool[k] ??= new List<int[]>();
            stack.EnsureCapacity(2 * want);
            for (; have < want; have++) { stack.Add(new int[size]); }
        }
    }

    private int NewId()
    {
        int id = _ids.Allocate();
        if (id >= _bodies.Length)
        {
            int cap = _bodies.Length * 2;
            Array.Resize(ref _bodies, cap); Array.Resize(ref _count, cap); Array.Resize(ref _asleep, cap); Array.Resize(ref _removed, cap);
        }

        // A freed island keeps its list for the next one, so splits and merges allocate nothing once warm.
        _bodies[id] ??= Take(MinList);
        _count[id] = 0; _asleep[id] = false; _removed[id] = 0;
        return id;
    }

    /// <summary>Creates an awake island holding one body and records it in the body store.</summary>
    internal int CreateFor(BodyStore bodies, int body)
    {
        int id = NewId();
        _bodies[id][0] = body; _count[id] = 1;
        bodies.IslandWritable[body] = id;
        return id;
    }

    /// <summary>Removes a body from its island; an empty island is freed.</summary>
    internal void RemoveBody(BodyStore bodies, int body)
    {
        int id = bodies.Island[body];
        if (id < 0) { return; }
        var list = _bodies[id];
        int n = _count[id];
        int at = Array.BinarySearch(list, 0, n, body);
        Array.Copy(list, at + 1, list, at, n - at - 1);
        _count[id] = n - 1;
        bodies.IslandWritable[body] = -1;
        if (n == 1)
        {
            _ids.Free(id);
            Release(_bodies[id]); _bodies[id] = null!;
            _asleep[id] = false; _removed[id] = 0; _count[id] = 0;
        }
        else
        {
            _removed[id]++;
        }
    }

    /// <summary>Counts a removed constraint against an island.</summary>
    internal void NoteRemoval(int island) => _removed[island]++;

    /// <summary>Merges two distinct awake islands; the lower id survives. Returns the survivor.</summary>
    internal int Merge(BodyStore bodies, int a, int b)
    {
        if (a == b) { return a; }
        int lo = Math.Min(a, b), hi = Math.Max(a, b);
        int nl = _count[lo], nh = _count[hi];
        var isl = bodies.IslandWritable;
        // Merge the smaller sorted list into the larger one from the back, in place, so a pile that keeps joining one big island allocates nothing.
        bool intoHi = nh > nl;
        int[] x = intoHi ? _bodies[hi] : _bodies[lo], y = intoHi ? _bodies[lo] : _bodies[hi];
        int nx = intoHi ? nh : nl, ny = intoHi ? nl : nh;
        int total = nx + ny;
        if (x.Length < total)
        {
            x = Grow(x, nx, total);
        }

        int ix = nx - 1, iy = ny - 1, k = total - 1;
        while (iy >= 0)
        {
            if (ix >= 0 && x[ix] > y[iy]) { x[k--] = x[ix--]; }
            else { x[k--] = y[iy--]; }
        }

        // Bodies that came from the other island (or all of them, when the survivor took over the higher island's list) now belong to the survivor.
        if (intoHi) { for (int t = 0; t < total; t++) { isl[x[t]] = lo; } }
        else { for (int t = 0; t < ny; t++) { isl[y[t]] = lo; } }
        var spare = intoHi ? _bodies[lo] : _bodies[hi];
        _bodies[lo] = x; _count[lo] = total;
        _removed[lo] += _removed[hi];
        _ids.Free(hi);
        _bodies[hi] = null!; _count[hi] = 0; _removed[hi] = 0; _asleep[hi] = false;
        Release(spare);
        return lo;
    }

    /// <summary>
    /// Splits an island into its connected components over the constraints that link two dynamic bodies. Returns the number of islands the body set now forms.
    /// </summary>
    internal int Split(BodyStore bodies, ConstraintStore cons, int island)
    {
        var list = _bodies[island];
        int n = _count[island];
        if (_scratchVisited.Length < bodies.HighWater) { _scratchVisited = new bool[Math.Max(bodies.HighWater, _scratchVisited.Length * 2)]; }
        var seen = _scratchVisited;
        var isl = bodies.IslandWritable;
        bool asleep = _asleep[island];
        int components = 0;
        // Components are discovered from the lowest unvisited body, so the first one holds the island's lowest body and keeps the id.
        if (_snapshot.Length < n) { _snapshot = new int[Math.Max(n, _snapshot.Length * 2)]; }
        var snapshot = _snapshot;
        Array.Copy(list, snapshot, n);
        for (int s = 0; s < n; s++)
        {
            int seed = snapshot[s];
            if (seen[seed]) { continue; }
            _component.Clear();
            _stack.Clear();
            _stack.Add(seed);
            seen[seed] = true;
            while (_stack.Count > 0)
            {
                int body = _stack[_stack.Count - 1];
                _stack.RemoveAt(_stack.Count - 1);
                _component.Add(body);
                for (int e = bodies.FirstEdge(body); e >= 0; e = cons.NextEdge(e))
                {
                    int c = e >> 1;
                    if (cons.Kind[c] == (byte)ConstraintKind.Sensor) { continue; }
                    int other = (e & 1) == 0 ? cons.BodyB[c] : cons.BodyA[c];
                    if (!bodies.IsDynamic(other) || seen[other]) { continue; }
                    seen[other] = true;
                    _stack.Add(other);
                }
            }

            _component.Sort();
            int id = components == 0 ? island : NewId();
            components++;
            int cn = _component.Count;
            if (_bodies[id].Length < cn) { _bodies[id] = Grow(_bodies[id], 0, cn); }
            var arr = _bodies[id];
            for (int k = 0; k < cn; k++) { arr[k] = _component[k]; isl[arr[k]] = id; }
            _count[id] = cn; _removed[id] = 0; _asleep[id] = asleep;
        }

        for (int s = 0; s < n; s++) { seen[snapshot[s]] = false; }
        RightSize(island);
        return components;
    }

    internal void Write(StateWriter w)
    {
        _ids.Write(w);
        for (int i = 0; i < _ids.HighWater; i++)
        {
            w.WriteBool(_asleep[i]);
            w.WriteInt32(_removed[i]);
        }
    }

    /// <summary>Reads the island table and rebuilds the lists from the bodies' island ids; validates that islands are non-empty and consistent with body sleep state and with the constraints.</summary>
    internal static IslandSet Read(StateReader r, BodyStore bodies, ConstraintStore cons)
    {
        var s = new IslandSet();
        s.ReadInto(r, bodies, cons);
        return s;
    }

    private int[] _countsScratch = new int[16];

    /// <summary>Fills this set from a snapshot, reusing its arrays and body lists when they are big enough. Only a set nobody uses (a spare) may be read into.</summary>
    internal void ReadInto(StateReader r, BodyStore bodies, ConstraintStore cons)
    {
        var s = this;
        s._ids.ReadInto(r, "island");
        int n = s._ids.HighWater;
        if ((long)n * 5 > r.Remaining) { throw r.Error("island columns are longer than the remaining bytes"); }
        int cap = Math.Max(16, n);
        if (s._bodies.Length < cap)
        {
            Array.Resize(ref s._bodies, cap); Array.Resize(ref s._count, cap); Array.Resize(ref s._asleep, cap); Array.Resize(ref s._removed, cap);
        }

        for (int i = 0; i < n; i++)
        {
            s._count[i] = 0;
            s._asleep[i] = r.ReadBool();
            s._removed[i] = r.ReadInt32();
            if (s._removed[i] < 0) { throw r.Error("negative island removal count"); }
            if (!s._ids.IsLive(i) && (s._asleep[i] || s._removed[i] != 0)) { throw r.Error("dead island slot is not zeroed"); }
        }

        if (_countsScratch.Length < n) { _countsScratch = new int[Math.Max(n, _countsScratch.Length * 2)]; }
        var counts = _countsScratch;
        Array.Clear(counts, 0, n);
        var islandOf = bodies.Island;
        for (int b = 0; b < bodies.HighWater; b++)
        {
            if (!bodies.IsLive(b)) { continue; }
            int id = islandOf[b];
            if (bodies.TypeOf(b) != BodyType.Dynamic) { continue; }
            if (id < 0 || id >= n || !s._ids.IsLive(id)) { throw r.Error("dynamic body " + b.ToString(System.Globalization.CultureInfo.InvariantCulture) + " has an invalid island"); }
            if (bodies.IsAwake(b) == s._asleep[id]) { throw r.Error("body sleep state disagrees with its island"); }
            counts[id]++;
        }

        for (int i = 0; i < n; i++)
        {
            if (s._ids.IsLive(i))
            {
                if (counts[i] == 0) { throw r.Error("empty island"); }
                int need = Math.Max(4, counts[i]);
                if (s._bodies[i] == null || s._bodies[i].Length < need) { s._bodies[i] = s.Swap(s._bodies[i], need); }
            }
            else if (s._bodies[i] != null)
            {
                s.Release(s._bodies[i]);
                s._bodies[i] = null!;
            }
        }

        for (int b = 0; b < bodies.HighWater; b++)
        {
            if (bodies.IsLive(b) && bodies.TypeOf(b) == BodyType.Dynamic)
            {
                int id = islandOf[b];
                s._bodies[id][s._count[id]++] = b;
            }
        }

        for (int c = 0; c < cons.HighWater; c++)
        {
            if (!cons.IsLive(c) || cons.Kind[c] == (byte)ConstraintKind.Sensor) { continue; }
            int a = cons.BodyA[c], b = cons.BodyB[c];
            if (bodies.IsDynamic(a) && bodies.IsDynamic(b) && islandOf[a] != islandOf[b]) { throw r.Error("constraint joins bodies of different islands"); }
        }
    }
}
